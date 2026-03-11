"""Configuration for ACT with visual input transforms (no temporal buffer).

This is a minimal extension of standard ACT that adds configurable input
pre-processing before the backbone.  There is no temporal self-attention
layer; the temporal window (n_obs_steps) exists only to provide the context
frames that the transform needs, not to build a feature-level buffer.

After the transform is applied to the T-frame window, only the *last* (current)
frame's enriched features are passed to the ACT encoder and decoder.

Usage
-----
--policy.type act_augmented --policy.input_transform framediff
--policy.type act_augmented --policy.input_transform framestack --policy.framestack_k 3
--policy.type act_augmented --policy.input_transform optflow
--policy.type act_augmented --policy.input_transform none   # identical to --policy.type act
"""

from dataclasses import dataclass

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig


# Map each transform type to the minimum number of frames it requires.
_TRANSFORM_MIN_FRAMES: dict[str, int] = {
    "none":       1,
    "framediff":  2,
    "framestack": -1,  # filled dynamically from framestack_k
    "optflow":    2,
}


@PreTrainedConfig.register_subclass("act_augmented")
@dataclass
class ACTAugmentedConfig(ACTConfig):
    """ACT with a configurable visual input transform.

    input_transform: str
        Which transform to apply before the backbone.
        "none"       — unchanged RGB input; identical to standard ACT.
        "framediff"  — append (frame[t] − frame[t-1]).  Needs 2 frames.
        "framestack" — stack k frames as channels.      Needs k frames.
        "optflow"    — append RAFT optical flow.         Needs 2 frames.

    framestack_k: int
        Number of frames to stack.  Only used when input_transform="framestack".
        n_obs_steps is set to framestack_k automatically.
    """

    input_transform: str = "none"
    framestack_k: int = 2  # used only for "framestack"

    def __post_init__(self):
        # Determine required number of observation frames from the transform.
        self.n_obs_steps = self._required_frames()

        # Bypass ACTConfig's hard "n_obs_steps must be 1" check by calling the
        # grandparent directly, then replicating the remaining validations.
        PreTrainedConfig.__post_init__(self)

        valid_transforms = set(_TRANSFORM_MIN_FRAMES.keys())
        if self.input_transform not in valid_transforms:
            raise ValueError(
                f"Unknown input_transform='{self.input_transform}'. "
                f"Valid options: {sorted(valid_transforms)}."
            )
        if not self.vision_backbone.startswith("resnet"):
            raise ValueError(
                f"`vision_backbone` must be one of the ResNet variants. Got {self.vision_backbone}."
            )
        if self.temporal_ensemble_coeff is not None and self.n_action_steps > 1:
            raise NotImplementedError(
                "`n_action_steps` must be 1 when using temporal ensembling."
            )
        if self.n_action_steps > self.chunk_size:
            raise ValueError(
                f"n_action_steps ({self.n_action_steps}) must be ≤ chunk_size ({self.chunk_size})."
            )

    def _required_frames(self) -> int:
        """Minimum T that makes the chosen transform meaningful."""
        if self.input_transform == "framestack":
            return self.framestack_k
        return _TRANSFORM_MIN_FRAMES.get(self.input_transform, 1)

    @property
    def observation_delta_indices(self) -> list | None:
        """Return delta indices so the dataset delivers n_obs_steps frames.

        When n_obs_steps=1 (transform="none"), this is [0] rather than None,
        but the dataset treats both identically — a single-frame tensor with
        an explicit temporal dimension of size 1.
        """
        return list(range(1 - self.n_obs_steps, 1))

    @property
    def action_delta_indices(self) -> list:
        return list(range(self.chunk_size))

    @property
    def reward_delta_indices(self):
        return None
