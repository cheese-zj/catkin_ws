"""Configuration for ACT with Temporal Self-Attention via Recurrent Feature Buffer.

Architecture summary:
  - Backbone + ACT encoder/decoder are UNCHANGED from standard ACT.
  - After per-frame backbone feature extraction, a FIFO buffer of T encoded feature
    maps is maintained.
  - A TemporalSelfAttentionLayer attends across this T-frame buffer before the ACT
    encoder sees the image tokens.
  - Training:  dataset provides n_obs_steps = temporal_buffer_size frames;
               each is encoded independently then stacked to simulate the buffer.
  - Inference: the deployment system provides a rolling T-frame window (standard
               lerobot behaviour for n_obs_steps > 1).
"""

from dataclasses import dataclass

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig


@PreTrainedConfig.register_subclass("act_temporal")
@dataclass
class ACTTemporalConfig(ACTConfig):
    """ACT + Temporal Self-Attention via Recurrent Feature Buffer.

    Inherits all ACT parameters.  The three new knobs are:

    temporal_buffer_size: int
        Number of consecutive frames in the temporal buffer (= n_obs_steps).
        At training time the dataset will deliver this many consecutive
        observation frames for each sample.  At inference time the deployment
        system accumulates a rolling window of the same length.

    temporal_n_heads: int
        Number of attention heads in the temporal self-attention layer.
        Must evenly divide dim_model.

    temporal_dropout: float
        Dropout probability used inside the temporal attention layer.

    temporal_n_layers: int
        Number of stacked temporal self-attention layers.
    """

    # Temporal buffer / attention parameters
    temporal_buffer_size: int = 5
    temporal_n_heads: int = 8
    temporal_dropout: float = 0.1
    temporal_n_layers: int = 1

    # Visual input transform applied before the backbone.
    # "none"       — standard RGB input  (default, backward-compatible)
    # "framediff"  — append (frame[t] - frame[t-1])  →  C*2 channels
    # "framestack" — stack k consecutive frames       →  C*framestack_k channels
    # "optflow"    — append RAFT optical flow          →  C+2 channels
    input_transform: str = "none"
    framestack_k: int = 2  # only used when input_transform="framestack"

    def __post_init__(self):
        # Force n_obs_steps to equal temporal_buffer_size so the dataset
        # delivers the right number of frames.
        self.n_obs_steps = self.temporal_buffer_size

        # Call the grandparent (PreTrainedConfig) directly to set up device /
        # AMP, bypassing ACTConfig's hard "n_obs_steps must be 1" check.
        PreTrainedConfig.__post_init__(self)

        # Replicate the remaining ACTConfig validations verbatim.
        if not self.vision_backbone.startswith("resnet"):
            raise ValueError(
                f"`vision_backbone` must be one of the ResNet variants. Got {self.vision_backbone}."
            )
        if self.temporal_ensemble_coeff is not None and self.n_action_steps > 1:
            raise NotImplementedError(
                "`n_action_steps` must be 1 when using temporal ensembling. This is "
                "because the policy needs to be queried every step to compute the ensembled action."
            )
        if self.n_action_steps > self.chunk_size:
            raise ValueError(
                f"The chunk size is the upper bound for the number of action steps per model invocation. Got "
                f"{self.n_action_steps} for `n_action_steps` and {self.chunk_size} for `chunk_size`."
            )
        # n_obs_steps > 1 is intentional here — no check needed.

        # Validate transform compatibility.
        _TRANSFORM_MIN_FRAMES = {
            "none": 1, "framediff": 2, "framestack": self.framestack_k, "optflow": 2
        }
        if self.input_transform not in _TRANSFORM_MIN_FRAMES:
            raise ValueError(
                f"Unknown input_transform='{self.input_transform}'. "
                "Valid options: 'none', 'framediff', 'framestack', 'optflow'."
            )
        min_frames = _TRANSFORM_MIN_FRAMES[self.input_transform]
        if self.temporal_buffer_size < min_frames:
            raise ValueError(
                f"input_transform='{self.input_transform}' requires at least {min_frames} frames, "
                f"but temporal_buffer_size={self.temporal_buffer_size}."
            )

    @property
    def observation_delta_indices(self) -> list:
        """Return the T-1 past frames plus the current frame (index 0).

        Example: temporal_buffer_size=5 → [-4, -3, -2, -1, 0]
        """
        return list(range(1 - self.temporal_buffer_size, 1))

    @property
    def action_delta_indices(self) -> list:
        return list(range(self.chunk_size))

    @property
    def reward_delta_indices(self):
        return None
