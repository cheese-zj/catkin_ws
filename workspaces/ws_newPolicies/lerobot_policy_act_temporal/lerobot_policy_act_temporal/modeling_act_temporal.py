"""ACT with Temporal Self-Attention via Recurrent Feature Buffer.

Design
------
The standard ACT backbone+encoder/decoder pipeline is left completely unchanged.
We insert one new operation between the backbone's per-frame feature extraction
and the ACT transformer encoder:

  backbone(frame_t)  ──►  proj  ──►  [T-frame buffer]  ──►  TemporalSelfAttentionLayer
                                                               │
                                                               ▼
                                                    ACT encoder  ──►  ACT decoder  ──►  actions

Temporal attention is applied across the T stacked image-feature-token sequences.
For every (spatial position, batch element) pair, the T time-steps form the
"sequence" that the attention layer reasons over.  After attention we take the
output at the latest timestep (-1) and feed it as the image tokens to the ACT
encoder — all other ACT encoder inputs (latent, robot-state token) are untouched.

Training vs. Inference
----------------------
Training  – the dataset delivers `n_obs_steps = temporal_buffer_size` consecutive
            frames per sample.  Each frame is encoded through the backbone
            independently; the resulting T feature-maps are stacked and passed
            through the temporal attention layer.

Inference – the deployment system (robot runner / env loop) maintains a rolling
            observation window of length `n_obs_steps` and passes the whole window
            to `select_action` at every step.  This is identical to how VQBeT
            handles its multi-step observation buffer.
"""

from itertools import chain

import einops
import numpy as np
import torch
import torch.nn.functional as F
import torchvision
from torch import Tensor, nn
from torchvision.models._utils import IntermediateLayerGetter
from torchvision.ops.misc import FrozenBatchNorm2d

from lerobot.policies.act.modeling_act import (
    ACT,
    ACTDecoder,
    ACTEncoder,
    ACTPolicy,
    ACTSinusoidalPositionEmbedding2d,
    ACTTemporalEnsembler,
    create_sinusoidal_pos_embedding,
    get_activation_fn,
)
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE

from lerobot_visual_transforms import adapt_backbone_first_conv, make_transform

from .configuration_act_temporal import ACTTemporalConfig


# ---------------------------------------------------------------------------
# Temporal self-attention layer
# ---------------------------------------------------------------------------

class TemporalSelfAttentionLayer(nn.Module):
    """Pre-norm self-attention + FFN block applied across the temporal axis.

    The input tensor has shape (T, S, B, D):
      T – number of time steps in the buffer
      S – number of spatial tokens (h*w summed over all cameras)
      B – batch size
      D – feature dimension (dim_model)

    For each (spatial position, batch element) pair the T time-steps are treated
    as an independent sequence.  We reshape to (T, S*B, D), apply standard
    self-attention over the T dimension, then reshape back to (T, S, B, D).

    Learnable temporal positional embeddings of shape (T, D) are broadcast-added
    before the attention so the layer knows which timestep it is processing.
    """

    def __init__(self, dim_model: int, n_heads: int, dropout: float = 0.1):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(dim_model, n_heads, dropout=dropout)
        # FFN: 4× expansion, GELU activation
        self.linear1 = nn.Linear(dim_model, dim_model * 4)
        self.linear2 = nn.Linear(dim_model * 4, dim_model)
        self.norm1 = nn.LayerNorm(dim_model)
        self.norm2 = nn.LayerNorm(dim_model)
        self.dropout = nn.Dropout(dropout)
        self.dropout1 = nn.Dropout(dropout)
        self.dropout2 = nn.Dropout(dropout)

    def forward(self, x: Tensor, temporal_pos: Tensor | None = None) -> Tensor:
        """
        Args:
            x:           (T, S, B, D)
            temporal_pos:(T, D)  learnable positional embeddings; broadcast
                         over S and B dimensions before the attention.
        Returns:
            (T, S, B, D)
        """
        T, S, B, D = x.shape

        # Add temporal positional embeddings.  Shape: (T, 1, 1, D) → broadcasts.
        if temporal_pos is not None:
            x = x + temporal_pos[:T].unsqueeze(1).unsqueeze(1)

        # Reshape so that each (spatial, batch) pair is a separate "sequence".
        x_flat = x.reshape(T, S * B, D)  # (T, S*B, D)

        # Pre-norm self-attention across T.
        skip = x_flat
        x_norm = self.norm1(x_flat)
        attn_out = self.self_attn(x_norm, x_norm, x_norm)[0]
        x_flat = skip + self.dropout1(attn_out)

        # Pre-norm FFN.
        skip = x_flat
        x_norm = self.norm2(x_flat)
        x_flat = skip + self.dropout2(
            self.linear2(self.dropout(F.gelu(self.linear1(x_norm))))
        )

        return x_flat.reshape(T, S, B, D)


# ---------------------------------------------------------------------------
# Main model
# ---------------------------------------------------------------------------

class ACTWithTemporalBuffer(ACT):
    """ACT model extended with a temporal self-attention layer.

    All standard ACT components (backbone, vae_encoder, encoder, decoder, etc.)
    are initialised by ``super().__init__(config)`` and remain unmodified.

    Additional components:
      temporal_attn_layers  – ModuleList of TemporalSelfAttentionLayer
      temporal_pos_embed    – nn.Embedding(temporal_buffer_size, dim_model)
    """

    def __init__(self, config: ACTTemporalConfig):
        super().__init__(config)  # initialise all standard ACT weights

        # Visual input transform (may be NoneTransform = identity).
        self.input_transform = make_transform(config.input_transform, config.framestack_k)
        c_in_total = 3 + self.input_transform.extra_channels

        # If the transform adds channels, patch the backbone's first conv so it
        # accepts the wider input while preserving the pretrained RGB weights.
        if c_in_total > 3 and config.image_features:
            adapt_backbone_first_conv(self.backbone, c_in_total)

        self.temporal_attn_layers = nn.ModuleList(
            [
                TemporalSelfAttentionLayer(
                    config.dim_model, config.temporal_n_heads, config.temporal_dropout
                )
                for _ in range(config.temporal_n_layers)
            ]
        )
        # Learnable temporal positional embedding: one vector per buffer slot.
        self.temporal_pos_embed = nn.Embedding(
            config.temporal_buffer_size, config.dim_model
        )

        # Xavier-uniform init for the new temporal parameters.
        for layer in self.temporal_attn_layers:
            for p in layer.parameters():
                if p.dim() > 1:
                    nn.init.xavier_uniform_(p)

    # ------------------------------------------------------------------
    # Forward
    # ------------------------------------------------------------------

    def forward(
        self, batch: dict[str, Tensor]
    ) -> tuple[Tensor, tuple[Tensor, Tensor] | tuple[None, None]]:
        """Forward pass with temporal feature buffer.

        Expected batch shapes (with T = temporal_buffer_size):
          batch[OBS_IMAGES]  – list of (B, T, C, H, W) tensors, one per camera.
          batch[OBS_STATE]   – (B, T, state_dim)   [optional]
          batch[ACTION]      – (B, chunk_size, action_dim)  [training only]
          batch["action_is_pad"] – (B, chunk_size)           [training only]

        The last time-step along dim-1 is treated as the *current* observation.
        All other time-steps are past context fed into the temporal buffer.
        """
        if self.config.use_vae and self.training:
            assert ACTION in batch, (
                "actions must be provided when using the variational objective in training mode."
            )
        
        for k,v in batch.items():
            if k == OBS_IMAGES and v.dim() == 4:
                batch[k] = v.unsqueeze(0)  # add batch dimension if missing, for single-sample inference
            elif k in [OBS_STATE, OBS_ENV_STATE] and v.dim() == 2:
                batch[k] = v.unsqueeze(0)  # add batch dimension if missing, for single-sample inference
                
        # Determine batch size from the first available modality.
        if OBS_IMAGES in batch:
            # batch[OBS_IMAGES][0] has shape (B, T, C, H, W)
            batch_size = batch[OBS_IMAGES][0].shape[0]
        else:
            batch_size = batch[OBS_ENV_STATE].shape[0]

        # Current-step state: take the last timestep from the temporal window.
        # Shape: (B, state_dim)
        current_state = (
            batch[OBS_STATE][:, -1, :]
            if (self.config.robot_state_feature and OBS_STATE in batch)
            else None
        )

        # ------------------------------------------------------------------
        # VAE encoder (training only)
        # ------------------------------------------------------------------
        if self.config.use_vae and ACTION in batch and self.training:
            cls_embed = einops.repeat(
                self.vae_encoder_cls_embed.weight, "1 d -> b 1 d", b=batch_size
            )
            if self.config.robot_state_feature:
                robot_state_embed = self.vae_encoder_robot_state_input_proj(
                    current_state
                ).unsqueeze(1)  # (B, 1, D)

            action_embed = self.vae_encoder_action_input_proj(batch[ACTION])  # (B, S, D)

            if self.config.robot_state_feature:
                vae_encoder_input = torch.cat(
                    [cls_embed, robot_state_embed, action_embed], dim=1
                )
            else:
                vae_encoder_input = torch.cat([cls_embed, action_embed], dim=1)

            pos_embed = self.vae_encoder_pos_enc.clone().detach()

            # Key-padding mask: False = not padded.
            device = batch[OBS_STATE].device if OBS_STATE in batch else batch[OBS_ENV_STATE].device
            cls_joint_is_pad = torch.full(
                (batch_size, 2 if self.config.robot_state_feature else 1),
                False,
                device=device,
            )
            key_padding_mask = torch.cat(
                [cls_joint_is_pad, batch["action_is_pad"]], dim=1
            )

            cls_token_out = self.vae_encoder(
                vae_encoder_input.permute(1, 0, 2),
                pos_embed=pos_embed.permute(1, 0, 2),
                key_padding_mask=key_padding_mask,
            )[0]
            latent_pdf_params = self.vae_encoder_latent_output_proj(cls_token_out)
            mu = latent_pdf_params[:, : self.config.latent_dim]
            log_sigma_x2 = latent_pdf_params[:, self.config.latent_dim :]
            latent_sample = mu + log_sigma_x2.div(2).exp() * torch.randn_like(mu)
        else:
            mu = log_sigma_x2 = None
            device = (
                batch[OBS_STATE].device
                if (OBS_STATE in batch)
                else batch[OBS_ENV_STATE].device
            )
            latent_sample = torch.zeros(
                [batch_size, self.config.latent_dim], dtype=torch.float32, device=device
            )

        # ------------------------------------------------------------------
        # Prepare 1-D encoder tokens: [latent, (robot_state), (env_state)]
        # ------------------------------------------------------------------
        encoder_in_tokens: list[Tensor] = [
            self.encoder_latent_input_proj(latent_sample)
        ]  # each entry: (B, D)
        encoder_in_pos_embed: list[Tensor] = list(
            self.encoder_1d_feature_pos_embed.weight.unsqueeze(1)
        )  # each: (1, D)

        if self.config.robot_state_feature:
            encoder_in_tokens.append(
                self.encoder_robot_state_input_proj(current_state)
            )

        if self.config.env_state_feature:
            encoder_in_tokens.append(
                self.encoder_env_state_input_proj(batch[OBS_ENV_STATE])
            )

        # ------------------------------------------------------------------
        # Image features with temporal attention
        # ------------------------------------------------------------------
        if self.config.image_features:
            # Per-camera: extract features for all T frames, rearrange to
            # (T, S_cam, B, D), accumulate into lists for temporal stacking.
            per_cam_temporal_feats: list[Tensor] = []  # each (T, S_cam, B, D)
            per_cam_pos_embeds: list[Tensor] = []      # each (S_cam, 1, D)

            for cam_img in batch[OBS_IMAGES]:
                # cam_img: (B, T, C, H, W)
                B, T, C, H, W = cam_img.shape

                # Apply visual transform (no-op for NoneTransform).
                # Output: (B, T, C_out, H, W)  where C_out = C + extra_channels
                cam_img = self.input_transform(cam_img)
                C = cam_img.shape[2]  # update C after transform

                # Flatten batch × time for a single backbone pass.
                img_flat = cam_img.reshape(B * T, C, H, W)

                feat_map = self.backbone(img_flat)["feature_map"]  # (B*T, C', h, w)

                # Sinusoidal 2-D position embedding is purely spatial (returns (1, D, h, w)).
                cam_pos_embed = self.encoder_cam_feat_pos_embed(feat_map)  # (1, D, h, w)

                # Project feature channels → dim_model.
                feat_proj = self.encoder_img_feat_input_proj(feat_map)  # (B*T, D, h, w)
                _, D_feat, h, w = feat_proj.shape

                # Rearrange to (T, S_cam, B, D).
                feat_proj = einops.rearrange(
                    feat_proj, "(b t) d h w -> t (h w) b d", b=B, t=T
                )

                # Position embed: (1, D, h, w) → (S_cam, 1, D)
                cam_pos = einops.rearrange(cam_pos_embed, "1 d h w -> (h w) 1 d")

                per_cam_temporal_feats.append(feat_proj)
                per_cam_pos_embeds.append(cam_pos)

            # Concatenate cameras along the spatial dimension.
            # (T, S_total, B, D)  where S_total = Σ S_cam
            stacked = torch.cat(per_cam_temporal_feats, dim=1)

            # Apply temporal self-attention layers.
            temporal_pos = self.temporal_pos_embed.weight  # (max_T, D)
            for layer in self.temporal_attn_layers:
                stacked = layer(stacked, temporal_pos=temporal_pos)

            # Take the most-recent timestep as the attended image representation.
            attended = stacked[-1]  # (S_total, B, D)

            # Add to encoder input lists.
            encoder_in_tokens.extend(list(attended))           # S_total × (B, D)
            all_cam_pos = torch.cat(per_cam_pos_embeds, dim=0)  # (S_total, 1, D)
            encoder_in_pos_embed.extend(list(all_cam_pos))

        # ------------------------------------------------------------------
        # Stack all encoder tokens and positional embeddings.
        # ------------------------------------------------------------------
        encoder_in_tokens = torch.stack(encoder_in_tokens, dim=0)       # (S, B, D)
        encoder_in_pos_embed = torch.stack(encoder_in_pos_embed, dim=0)  # (S, 1, D)

        # ------------------------------------------------------------------
        # ACT transformer encoder + decoder  (completely unchanged)
        # ------------------------------------------------------------------
        encoder_out = self.encoder(encoder_in_tokens, pos_embed=encoder_in_pos_embed)

        decoder_in = torch.zeros(
            (self.config.chunk_size, batch_size, self.config.dim_model),
            dtype=encoder_in_pos_embed.dtype,
            device=encoder_in_pos_embed.device,
        )
        decoder_out = self.decoder(
            decoder_in,
            encoder_out,
            encoder_pos_embed=encoder_in_pos_embed,
            decoder_pos_embed=self.decoder_pos_embed.weight.unsqueeze(1),
        )

        # (S, B, D) → (B, S, D) → (B, chunk_size, action_dim)
        decoder_out = decoder_out.transpose(0, 1)
        actions = self.action_head(decoder_out)

        return actions, (mu, log_sigma_x2)


# ---------------------------------------------------------------------------
# Policy wrapper
# ---------------------------------------------------------------------------

class ACTTemporalPolicy(ACTPolicy):
    """ACTPolicy subclass that uses ACTWithTemporalBuffer instead of ACT.

    All training / inference logic from ACTPolicy is preserved.  The only
    difference is the underlying model class.
    """

    config_class = ACTTemporalConfig
    name = "act_temporal"

    def __init__(self, config: ACTTemporalConfig, **kwargs):
        # Call PreTrainedPolicy.__init__ (grandparent) directly so we can
        # set self.model to ACTWithTemporalBuffer without ACTPolicy.__init__
        # creating a standard ACT model first.
        from lerobot.policies.pretrained import PreTrainedPolicy

        PreTrainedPolicy.__init__(self, config)
        config.validate_features()
        self.config = config

        # Replace standard ACT with the temporal variant.
        self.model = ACTWithTemporalBuffer(config)

        if config.temporal_ensemble_coeff is not None:
            self.temporal_ensembler = ACTTemporalEnsembler(
                config.temporal_ensemble_coeff, config.chunk_size
            )

        self.reset()
