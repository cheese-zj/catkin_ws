"""ACT with visual input transforms — no temporal self-attention buffer.

Forward pass summary
--------------------
For each camera:

  (B, T, C, H, W)
       │
       ▼  input_transform  (framediff / framestack / optflow / none)
  (B, T, C_out, H, W)
       │
       ▼  take last frame  [:, -1]
  (B, C_out, H, W)
       │
       ▼  backbone  →  ACT encoder  →  ACT decoder  →  actions

The temporal dimension T is consumed entirely by the transform.  The ACT
encoder and decoder are not modified at all.

State handling
--------------
With n_obs_steps = T > 1 the dataset provides state as (B, T, state_dim).
We always take the last (current) timestep: batch[OBS_STATE][:, -1, :].
"""

from itertools import chain

import einops
import torch
import torch.nn.functional as F
from torch import Tensor, nn

from lerobot.policies.act.modeling_act import (
    ACT,
    ACTPolicy,
    ACTTemporalEnsembler,
)
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE
from lerobot_visual_transforms import adapt_backbone_first_conv, make_transform

from .configuration_act_augmented import ACTAugmentedConfig


class ACTAugmented(ACT):
    """ACT model with a visual input transform inserted before the backbone.

    All standard ACT components are initialised by ``super().__init__(config)``.
    The only additions are:
      self.input_transform  — VisualTransform instance (may be NoneTransform)
    The backbone's first conv is widened if the transform adds extra channels.
    """

    def __init__(self, config: ACTAugmentedConfig):
        super().__init__(config)

        self.input_transform = make_transform(config.input_transform, config.framestack_k)
        c_in_total = 3 + self.input_transform.extra_channels

        if c_in_total > 3 and config.image_features:
            adapt_backbone_first_conv(self.backbone, c_in_total)

    # ------------------------------------------------------------------
    # Forward
    # ------------------------------------------------------------------

    def forward(
        self, batch: dict[str, Tensor]
    ) -> tuple[Tensor, tuple[Tensor, Tensor] | tuple[None, None]]:
        """Forward pass for ACT with input transform.

        Expected batch shapes (T = n_obs_steps):
          batch[OBS_IMAGES]      — list of (B, T, C, H, W), one per camera
          batch[OBS_STATE]       — (B, T, state_dim)           [optional]
          batch[ACTION]          — (B, chunk_size, action_dim) [training]
          batch["action_is_pad"] — (B, chunk_size, bool)       [training]

        The transform is applied to all T frames; then the *last* frame's
        enriched features are fed to the ACT encoder.
        """
        if self.config.use_vae and self.training:
            assert ACTION in batch, (
                "actions must be provided when using the variational objective in training mode."
            )

        # Determine batch size.
        if OBS_IMAGES in batch:
            batch_size = batch[OBS_IMAGES][0].shape[0]
        else:
            batch_size = batch[OBS_ENV_STATE].shape[0]

        # Current-step state: take the last timestep from the T-frame window.
        has_state = self.config.robot_state_feature and OBS_STATE in batch
        current_state = batch[OBS_STATE][:, -1, :] if has_state else None

        # Infer device.
        device = (
            batch[OBS_STATE].device if has_state else batch[OBS_ENV_STATE].device
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
                ).unsqueeze(1)
            action_embed = self.vae_encoder_action_input_proj(batch[ACTION])

            if self.config.robot_state_feature:
                vae_encoder_input = torch.cat(
                    [cls_embed, robot_state_embed, action_embed], dim=1
                )
            else:
                vae_encoder_input = torch.cat([cls_embed, action_embed], dim=1)

            pos_embed = self.vae_encoder_pos_enc.clone().detach()

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
            latent_sample = torch.zeros(
                [batch_size, self.config.latent_dim], dtype=torch.float32, device=device
            )

        # ------------------------------------------------------------------
        # 1-D encoder tokens: [latent, (state), (env_state)]
        # ------------------------------------------------------------------
        encoder_in_tokens: list[Tensor] = [
            self.encoder_latent_input_proj(latent_sample)
        ]
        encoder_in_pos_embed: list[Tensor] = list(
            self.encoder_1d_feature_pos_embed.weight.unsqueeze(1)
        )

        if self.config.robot_state_feature:
            encoder_in_tokens.append(
                self.encoder_robot_state_input_proj(current_state)
            )
        if self.config.env_state_feature:
            encoder_in_tokens.append(
                self.encoder_env_state_input_proj(batch[OBS_ENV_STATE])
            )

        # ------------------------------------------------------------------
        # Image features: transform → take last frame → backbone
        # ------------------------------------------------------------------
        if self.config.image_features:
            for cam_img in batch[OBS_IMAGES]:
                # cam_img: (B, T, C, H, W)

                # Apply visual transform across the temporal window.
                cam_img = self.input_transform(cam_img)   # (B, T, C_out, H, W)

                # Take only the current (last) timestep.
                current_frame = cam_img[:, -1]            # (B, C_out, H, W)

                cam_features = self.backbone(current_frame)["feature_map"]
                cam_pos_embed = self.encoder_cam_feat_pos_embed(cam_features)
                cam_features = self.encoder_img_feat_input_proj(cam_features)

                cam_features = einops.rearrange(cam_features, "b c h w -> (h w) b c")
                cam_pos_embed = einops.rearrange(cam_pos_embed, "b c h w -> (h w) b c")

                encoder_in_tokens.extend(list(cam_features))
                encoder_in_pos_embed.extend(list(cam_pos_embed))

        # ------------------------------------------------------------------
        # ACT encoder + decoder (unchanged)
        # ------------------------------------------------------------------
        encoder_in_tokens = torch.stack(encoder_in_tokens, dim=0)
        encoder_in_pos_embed = torch.stack(encoder_in_pos_embed, dim=0)

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

        decoder_out = decoder_out.transpose(0, 1)
        actions = self.action_head(decoder_out)

        return actions, (mu, log_sigma_x2)


# ---------------------------------------------------------------------------
# Policy wrapper
# ---------------------------------------------------------------------------

class ACTAugmentedPolicy(ACTPolicy):
    """ACTPolicy using ACTAugmented (with visual input transform)."""

    config_class = ACTAugmentedConfig
    name = "act_augmented"

    def __init__(self, config: ACTAugmentedConfig, **kwargs):
        from lerobot.policies.pretrained import PreTrainedPolicy

        PreTrainedPolicy.__init__(self, config)
        config.validate_features()
        self.config = config

        self.model = ACTAugmented(config)

        if config.temporal_ensemble_coeff is not None:
            self.temporal_ensembler = ACTTemporalEnsembler(
                config.temporal_ensemble_coeff, config.chunk_size
            )

        self.reset()
