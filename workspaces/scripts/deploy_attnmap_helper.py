#!/usr/bin/env python3
"""deployment with Real-time ACT cross-attention viewer.
"""
import os
import time
import numpy as np
import cv2
import torch
import types
import random

def patch_mha_capture_attn(mha: torch.nn.MultiheadAttention, store_list: list):
    """
    Patches ONE MultiheadAttention module to always return attn weights and store them.
    This avoids recursion by calling the CLASS forward() directly.
    """
    # If already patched, don't patch again
    if hasattr(mha, "_orig_forward_unpatched"):
        return

    mha._orig_forward_unpatched = mha.forward  # save whatever it currently is, for restore

    def forward_with_attn(self, query, key, value, **kwargs):
        # Remove user-provided flags so we control them
        kwargs.pop("need_weights", None)
        kwargs.pop("average_attn_weights", None)

        # IMPORTANT: call the CLASS method, not self.forward / saved bound methods
        out, attn = torch.nn.MultiheadAttention.forward(
            self,
            query,
            key,
            value,
            need_weights=True,
            average_attn_weights=False,
            **kwargs
        )
        store_list.append(attn.detach().cpu())
        return out, attn

    mha.forward = types.MethodType(forward_with_attn, mha)

def unpatch_mha(mha: torch.nn.MultiheadAttention):
    if hasattr(mha, "_orig_forward_unpatched"):
        mha.forward = mha._orig_forward_unpatched
        delattr(mha, "_orig_forward_unpatched")

def _to_uint8_rgb(chw_float01):
    """CHW float[0,1] -> HWC uint8 RGB"""
    x = np.clip(chw_float01, 0, 1)
    hwc = (np.transpose(x, (1, 2, 0)) * 255.0).astype(np.uint8)
    return hwc

def _overlay_heatmap(rgb_uint8, heat01, alpha=0.45):
    """
    rgb_uint8: HWC uint8 RGB
    heat01:    HW float in [0,1]
    """
    h, w = rgb_uint8.shape[:2]
    heat01 = np.clip(heat01, 0, 1)
    heat_u8 = (heat01 * 255).astype(np.uint8)
    color = cv2.applyColorMap(heat_u8, cv2.COLORMAP_JET)  # BGR
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

    out = (rgb_uint8.astype(np.float32) * (1 - alpha) + color.astype(np.float32) * alpha)
    return np.clip(out, 0, 255).astype(np.uint8)

def cross_attn_to_camera_grids(
    cross_attn,  # torch.Tensor [B, heads, DS, ES]
    n_1d_tokens=2,
    n_cams=3,
    cam_hw=(8, 8),
    query_index=0,
    reduce_queries="single",   # "single" or "mean"
    reduce_heads="mean",       # "mean" or "max"
):
    """
    Returns: np.ndarray shape (n_cams, Hf, Wf) attention for each camera.
    """
    A = cross_attn  # [B, H, DS, ES]
    if A.dim() != 4:
        raise RuntimeError(f"Expected cross-attn [B, heads, DS, ES], got {A.shape}")

    if reduce_heads == "mean":
        A = A.mean(dim=1)      # [B, DS, ES]
    elif reduce_heads == "max":
        A = A.max(dim=1).values
    else:
        raise ValueError("reduce_heads must be 'mean' or 'max'")

    if reduce_queries == "single":
        v = A[:, query_index, :]  # [B, ES]
    elif reduce_queries == "mean":
        v = A.mean(dim=1)         # [B, ES]
    else:
        raise ValueError("reduce_queries must be 'single' or 'mean'")

    v = v[0]  # B=1 -> [ES]
    # drop [latent, robot_state]
    v_img = v[n_1d_tokens:]  # length 192

    Hf, Wf = cam_hw
    per_cam = Hf * Wf
    assert v_img.numel() == n_cams * per_cam, f"Expected {n_cams*per_cam} img tokens, got {v_img.numel()}"

    grids = v_img.reshape(n_cams, Hf, Wf).detach().cpu().numpy()
    # normalize each camera grid to [0,1] for visualization
    out = []
    for g in grids:
        g = g - g.min()
        g = g / (g.max() + 1e-8)
        out.append(g)
    return np.stack(out, axis=0)  # (3, 8, 8)

def upsample_grid_to_image(grid_8x8, out_hw=(256,256)):
    return cv2.resize(grid_8x8.astype(np.float32), out_hw, interpolation=cv2.INTER_CUBIC)


class AttnVideoRecorder:
    def __init__(self, out_mp4: str = 'root/catkin_ws/attn_maps_x/', fps: int = 20):
        self.out_mp4 = out_mp4
        self.fps = fps
        self._writer = None

    def write(self, frame_rgb_uint8: np.ndarray):
        h, w = frame_rgb_uint8.shape[:2]
        if self._writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(self.out_mp4, fourcc, self.fps, (w, h))
        self._writer.write(cv2.cvtColor(frame_rgb_uint8, cv2.COLOR_RGB2BGR))

    def close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None


def _find_mha_candidates(policy):
    cands = []
    for name, m in policy.named_modules():
        if isinstance(m, torch.nn.MultiheadAttention):
            cands.append((name, m))
    return cands

def _pick_cross_attn(cands, n_expected_src=None):
    # Prefer names that look like cross-attn
    preferred = [c for c in cands if any(k in c[0].lower() for k in ["cross", "multihead_attn", "decoder"])]
    pool = preferred if preferred else cands

    # If you know expected src length (image tokens + 1d tokens), filter by it
    if n_expected_src is not None:
        filtered = []
        for name, m in pool:
            # Can't know src_len without a forward pass, so just return pool here.
            # We'll confirm after first capture.
            filtered.append((name, m))
        pool = filtered

    return pool[0] if pool else (None, None)


def _pick_self_attn(cands):
    preferred = [c for c in cands if any(k in c[0].lower() for k in [
        "self", "self_attn", "encoder", "attn"
    ])]
    pool = preferred if preferred else cands
    return pool[0] if pool else (None, None)

class RealTimeAttnViewer:
    """
    Real-time ACT cross-attention viewer.
    - Patches policy decoder cross-attn MHA once.
    - Each step: read latest cross-attn, build heatmaps, overlay on RGB, show in a window.
    """
    def __init__(
        self,
        policy,
        device="cuda",
        window_name="ACT Cross-Attn",
        n_1d_tokens=2,
        n_cams=3,
        cam_hw=(8, 8),          # your token grid per camera
        out_hw=(256, 256),      # image size for overlay
        reduce_heads="mean",
        reduce_queries="mean",  # "mean" is smoother for streaming
        query_index=0,          # used only if reduce_queries="single"
        alpha=0.45,
        max_store=8,            # keep store list bounded
        show_fps=True,
        save_dir = 'root/catkin_ws/attn_maps_x/frames',
    ):
        self.policy = policy
        self.device = device
        self.window_name = window_name

        self.n_1d_tokens = n_1d_tokens
        self.n_cams = n_cams
        self.cam_hw = cam_hw
        self.out_hw = out_hw
        self.reduce_heads = reduce_heads
        self.reduce_queries = reduce_queries
        self.query_index = query_index
        self.alpha = alpha
        self.max_store = max_store
        self.show_fps = show_fps

        self.self_attn_maps = []
        self.cross_attn_maps = []

        # --- Patch modules (adjust layer index if you want deeper layers)
        self.mha_self = self.policy.model.encoder.layers[0].self_attn
        self.mha_cross = self.policy.model.decoder.layers[0].multihead_attn
        patch_mha_capture_attn(self.mha_self, self.self_attn_maps)
        patch_mha_capture_attn(self.mha_cross, self.cross_attn_maps)

        # cands = _find_mha_candidates(self.policy)
        # print("[AttnViewer] found MHA:", [n for n,_ in cands])
        # self.mha_cross_name, self.mha_cross = _pick_cross_attn(cands)
        # print("[AttnViewer] using cross:", self.mha_cross_name)
        # patch_mha_capture_attn(self.mha_cross, self.cross_attn_maps)
        # self.mha_self_name,  self.mha_self  = _pick_self_attn(cands)
        # print("[AttnViewer] using self: ", self.mha_self_name)
        # patch_mha_capture_attn(self.mha_self,  self.self_attn_maps)

        self._last_t = time.time()
        self._fps = 0.0

        self.save_dir = save_dir

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def close(self):
        # restore model forwards
        unpatch_mha(self.mha_self)
        unpatch_mha(self.mha_cross)
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def _bounded_append(self, store_list, item):
        store_list.append(item)
        if len(store_list) > self.max_store:
            del store_list[:-self.max_store]

    def step_show_cross(self, top_chw01, left_chw01, right_chw01, save: bool = False, step_idx: int | None = None):
        """
        Call AFTER policy.select_action(...) so cross_attn_maps got populated.
        Inputs: CHW float[0,1] numpy arrays (your snap decoded images).
        Returns: overlayed RGB frame (H, W*3, 3) uint8 or None.
        If save=True, saves frame to disk (optionally with step_idx).
        """
        if len(self.cross_attn_maps) == 0:
            return  # not yet captured

        attn = self.cross_attn_maps[-1]  # torch.Tensor [B, heads, DS, ES]

        # if len(self.cross_attn_maps) > self.max_store:
        #     self.cross_attn_maps = self.cross_attn_maps[-self.max_store:]
        # if len(self.self_attn_maps) > self.max_store:
        #     self.self_attn_maps = self.self_attn_maps[-self.max_store:]
        if len(self.cross_attn_maps) > self.max_store:
            del self.cross_attn_maps[:-self.max_store]
        if len(self.self_attn_maps) > self.max_store:
            del self.self_attn_maps[:-self.max_store]

        # Build per-camera attention grids
        cam_grids = cross_attn_to_camera_grids(
            attn,
            n_1d_tokens=self.n_1d_tokens,
            n_cams=self.n_cams,
            cam_hw=self.cam_hw,
            query_index=self.query_index,
            reduce_queries=self.reduce_queries,
            reduce_heads=self.reduce_heads,
        )

        heat_top   = upsample_grid_to_image(cam_grids[0], out_hw=self.out_hw)
        heat_left  = upsample_grid_to_image(cam_grids[1], out_hw=self.out_hw)
        heat_right = upsample_grid_to_image(cam_grids[2], out_hw=self.out_hw)

        rgb_top   = _to_uint8_rgb(top_chw01)
        rgb_left  = _to_uint8_rgb(left_chw01)
        rgb_right = _to_uint8_rgb(right_chw01)

        ov_top   = _overlay_heatmap(rgb_top, heat_top, alpha=self.alpha)
        ov_left  = _overlay_heatmap(rgb_left, heat_left, alpha=self.alpha)
        ov_right = _overlay_heatmap(rgb_right, heat_right, alpha=self.alpha)

        grid = np.hstack([ov_left, ov_top, ov_right])
        grid = np.ascontiguousarray(grid)

        # FPS overlay (optional)
        if self.show_fps:
            now = time.time()
            dt = max(now - self._last_t, 1e-6)
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
            self._last_t = now
            cv2.putText(
                grid,
                f"FPS: {self._fps:.1f} | reduce_q={self.reduce_queries}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # Show (OpenCV expects BGR)
        cv2.imshow(self.window_name, cv2.cvtColor(grid, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

        # Save frame if requested
        if save and hasattr(self, "save_dir") and self.save_dir is not None:
            os.makedirs(self.save_dir, exist_ok=True)
            name = f"attn_{step_idx:06d}.png" if step_idx is not None else f"attn_{int(time.time()*1000)}.png"
            cv2.imwrite(os.path.join(self.save_dir, name), cv2.cvtColor(grid, cv2.COLOR_RGB2BGR))

        return grid
