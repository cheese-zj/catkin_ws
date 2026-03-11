"""Visual input transforms for ACT-based policies.

All transforms share the same interface:
  - Input:  (B, T, C, H, W)  — T frames of C-channel images per sample
  - Output: (B, T, C_out, H, W)  — original channels always preserved first

The temporal dimension T is always present.  For policies with n_obs_steps=1
(e.g. act_augmented with input_transform="none"), T=1 and the transform is a
pass-through.  For policies with n_obs_steps>1 the transform can exploit the
temporal window to compute motion signals.

Two helpers are exported alongside the transforms:
  make_transform(type, framestack_k) — factory
  adapt_backbone_first_conv(backbone, c_in_new) — patches ResNet's conv1
"""

import torch
import torch.nn as nn
from torch import Tensor

__all__ = [
    "VisualTransform",
    "NoneTransform",
    "FrameDiffTransform",
    "FrameStackTransform",
    "OpticalFlowTransform",
    "make_transform",
    "adapt_backbone_first_conv",
]


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class VisualTransform(nn.Module):
    """Abstract base class for visual input transforms."""

    @property
    def extra_channels(self) -> int:
        """Extra channels appended to the input (C_out = C + extra_channels)."""
        raise NotImplementedError

    @property
    def min_frames_needed(self) -> int:
        """Minimum number of temporal frames required for a meaningful output."""
        raise NotImplementedError

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: (B, T, C, H, W)
        Returns:
            (B, T, C + extra_channels, H, W)
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Concrete transforms
# ---------------------------------------------------------------------------

class NoneTransform(VisualTransform):
    """Identity — no modification to the input."""

    @property
    def extra_channels(self) -> int:
        return 0

    @property
    def min_frames_needed(self) -> int:
        return 1

    def forward(self, x: Tensor) -> Tensor:
        return x


class FrameDiffTransform(VisualTransform):
    """Append (frame[t] - frame[t-1]) as extra channels.

    The difference is zero for t=0 (no previous frame available).
    Together with the original RGB channels the backbone sees both
    *what is there* and *what changed* — a cheap motion cue.

    Input  C=3  →  Output C_out=6  (RGB ‖ Δ RGB)
    """

    @property
    def extra_channels(self) -> int:
        return 3  # same channel count as input RGB

    @property
    def min_frames_needed(self) -> int:
        return 2

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: (B, T, C, H, W)
        Returns:
            (B, T, 2*C, H, W)
        """
        diff = torch.zeros_like(x)
        diff[:, 1:] = x[:, 1:] - x[:, :-1]
        return torch.cat([x, diff], dim=2)


class FrameStackTransform(VisualTransform):
    """Concatenate k consecutive frames as channels at each timestep.

    At timestep t the output contains [frame[t], frame[t-1], ..., frame[t-k+1]].
    Frames that precede the start of the buffer are zero-padded.

    With k=2 and C=3: C_out=6  (current frame ‖ previous frame)
    With k=3 and C=3: C_out=9  (current ‖ t-1 ‖ t-2)

    The backbone implicitly infers velocity / direction from the stacked pixels.
    """

    def __init__(self, k: int = 2):
        super().__init__()
        if k < 2:
            raise ValueError("FrameStackTransform requires k >= 2; use NoneTransform for k=1.")
        self.k = k

    @property
    def extra_channels(self) -> int:
        return 3 * (self.k - 1)

    @property
    def min_frames_needed(self) -> int:
        return self.k

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: (B, T, C, H, W)
        Returns:
            (B, T, C*k, H, W)
        """
        frames = [x]
        for lag in range(1, self.k):
            lagged = torch.zeros_like(x)
            lagged[:, lag:] = x[:, :-lag]  # shift right, zero-pad left
            frames.append(lagged)
        return torch.cat(frames, dim=2)


class OpticalFlowTransform(VisualTransform):
    """Append RAFT-Small optical flow (dx, dy) between consecutive frames.

    Uses torchvision's pre-trained RAFT-Small model, always frozen.
    Flow is zero for t=0 (no previous frame).

    Input  C=3  →  Output C_out=5  (RGB ‖ dx ‖ dy)

    Normalization assumption
    -----------------------
    Images are assumed to be ImageNet-normalised
    (mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]).
    The transform un-normalises to [0, 255] before passing frames to RAFT
    (which expects that range per torchvision's API).

    Performance note
    ----------------
    RAFT is called inside torch.no_grad() and its weights are frozen, so it
    adds inference cost but zero training overhead beyond the forward pass.
    For very tight GPU budgets, prefer FrameDiffTransform instead.
    """

    _IMAGENET_MEAN = (0.485, 0.456, 0.406)
    _IMAGENET_STD  = (0.229, 0.224, 0.225)

    def __init__(self):
        super().__init__()
        try:
            from torchvision.models.optical_flow import raft_small, Raft_Small_Weights
        except ImportError as e:
            raise ImportError(
                "OpticalFlowTransform requires torchvision >= 0.15 with optical_flow support. "
                "Install via: pip install torchvision>=0.15"
            ) from e

        self.raft = raft_small(weights=Raft_Small_Weights.DEFAULT)
        for p in self.raft.parameters():
            p.requires_grad_(False)

        mean = torch.tensor(self._IMAGENET_MEAN).view(1, 3, 1, 1)
        std  = torch.tensor(self._IMAGENET_STD ).view(1, 3, 1, 1)
        self.register_buffer("_mean", mean)
        self.register_buffer("_std",  std)

    @property
    def extra_channels(self) -> int:
        return 2

    @property
    def min_frames_needed(self) -> int:
        return 2

    def _to_raft_range(self, x: Tensor) -> Tensor:
        """Un-normalise from ImageNet stats → [0, 255] as expected by RAFT."""
        return ((x * self._std + self._mean) * 255.0).clamp(0.0, 255.0)

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: (B, T, 3, H, W) — ImageNet-normalised
        Returns:
            (B, T, 5, H, W) — RGB ‖ flow_dx ‖ flow_dy  (flow in pixels/frame)
        """
        B, T, C, H, W = x.shape
        flows = torch.zeros(B, T, 2, H, W, device=x.device, dtype=x.dtype)

        # RAFT requires float32 regardless of AMP context.
        orig_dtype = x.dtype
        x_f32 = x.float()
        with torch.no_grad(), torch.amp.autocast("cuda", enabled=False):
            for t in range(1, T):
                frame1 = self._to_raft_range(x_f32[:, t - 1])  # (B, 3, H, W) float32
                frame2 = self._to_raft_range(x_f32[:, t])
                # raft returns a list of progressively refined predictions;
                # the last element is the most accurate.
                flows[:, t] = self.raft(frame1, frame2)[-1].to(orig_dtype)

        return torch.cat([x, flows], dim=2)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_transform(transform_type: str, framestack_k: int = 2) -> VisualTransform:
    """Instantiate a VisualTransform by name.

    Args:
        transform_type: "none" | "framediff" | "framestack" | "optflow"
        framestack_k:   Stack depth; only used when transform_type="framestack".
    """
    if transform_type == "none":
        return NoneTransform()
    elif transform_type == "framediff":
        return FrameDiffTransform()
    elif transform_type == "framestack":
        return FrameStackTransform(k=framestack_k)
    elif transform_type == "optflow":
        return OpticalFlowTransform()
    else:
        raise ValueError(
            f"Unknown input_transform='{transform_type}'. "
            "Valid options: 'none', 'framediff', 'framestack', 'optflow'."
        )


# ---------------------------------------------------------------------------
# Backbone adapter
# ---------------------------------------------------------------------------

def adapt_backbone_first_conv(backbone: nn.ModuleDict, c_in_new: int) -> None:
    """Patch a ResNet backbone's first conv to accept c_in_new input channels.

    The backbone is accessed as an IntermediateLayerGetter (nn.ModuleDict).
    ResNet's 'conv1' is stored as backbone['conv1'].

    Weight initialisation
    ---------------------
    - Channels 0:3  → copy pretrained RGB weights  (preserve ImageNet prior)
    - Channels 3:   → zero-initialise             (network learns from scratch)

    With zero init the backbone initially ignores the extra channels, which
    avoids a large loss spike at the start of training.

    Args:
        backbone:   IntermediateLayerGetter wrapping a ResNet.
        c_in_new:   Total input channels after concatenating transforms
                    (= 3 + transform.extra_channels).
    """
    if c_in_new == 3:
        return  # nothing to do

    old_conv: nn.Conv2d = backbone["conv1"]
    if old_conv.in_channels != 3:
        raise RuntimeError(
            f"adapt_backbone_first_conv: backbone conv1 already has "
            f"{old_conv.in_channels} input channels. Did you call this twice?"
        )

    new_conv = nn.Conv2d(
        in_channels=c_in_new,
        out_channels=old_conv.out_channels,
        kernel_size=old_conv.kernel_size,
        stride=old_conv.stride,
        padding=old_conv.padding,
        bias=old_conv.bias is not None,
    )
    with torch.no_grad():
        new_conv.weight[:, :3] = old_conv.weight       # copy pretrained RGB
        nn.init.zeros_(new_conv.weight[:, 3:])          # zero for extra channels
        if old_conv.bias is not None:
            new_conv.bias.copy_(old_conv.bias)

    backbone["conv1"] = new_conv
