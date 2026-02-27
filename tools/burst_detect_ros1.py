#!/usr/bin/env python3
"""Detect high-speed joint bursts from a single ROS1 bag episode."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


THRESHOLD_MODES = ("percentile", "norm_percentile", "norm_fixed", "global_percentile")
NORM_BASES = ("median", "p90")
SIGNAL_SPACES = ("speed", "speed_norm")
MERGE_MODES = ("gap", "fill_holes")


CSV_COLUMNS = [
    "burst_id",
    "start_s",
    "end_s",
    "duration_s",
    "peak_speed",
    "t_peak_speed_s",
    "peak_acc",
    "t_peak_acc_s",
    "peak_dtau",
    "t_peak_dtau_s",
    "start_abs_s",
    "end_abs_s",
    "t_peak_speed_abs_s",
    "t_peak_acc_abs_s",
    "t_peak_dtau_abs_s",
    "threshold_mode",
    "threshold_signal_space",
    "norm_base",
    "norm_scale",
    "norm_eps",
    "on_th_signal",
    "off_th_signal",
    "on_th_speed_equiv",
    "off_th_speed_equiv",
    "merge_gap_s",
    "merge_mode",
    "hole_min_norm",
    "peak_min_norm",
    "peak_min_raw",
    "peak_filter_norm_base",
    "peak_filter_norm_scale",
    "peak_filter_norm_eps",
]


@dataclass(frozen=True)
class TopicMeta:
    topic: str
    msg_type: str
    message_count: int


@dataclass
class JointSeries:
    t: np.ndarray
    q: np.ndarray
    tau: Optional[np.ndarray]


@dataclass(frozen=True)
class BurstWindow:
    start_s: float
    end_s: float


@dataclass(frozen=True)
class ThresholdInfo:
    mode: str
    signal_space: str
    norm_base: str
    norm_scale: float
    norm_eps: float
    on_th_signal: float
    off_th_signal: float
    on_th_speed_equiv: float
    off_th_speed_equiv: float


@dataclass
class EpisodeResult:
    bag_path: Path
    outdir: Path
    episode_name: str
    bursts_csv_path: Path
    timeseries_png_path: Path
    joint_topic: str
    num_bursts: int
    total_burst_time: float
    mean_peak_speed: float
    max_peak_speed: float
    mean_duration: float
    on_th: float
    off_th: float
    threshold_mode: str
    threshold_signal_space: str
    norm_base: str
    norm_scale: float
    norm_eps: float
    on_th_speed_equiv: float
    off_th_speed_equiv: float
    merge_gap_s: float
    merge_mode: str
    hole_min_norm: float
    peak_min_norm: float
    peak_min_raw: float
    peak_filter_norm_base: str
    peak_filter_norm_scale: float
    peak_filter_norm_eps: float


def _meta_type(meta) -> str:
    if hasattr(meta, "msg_type"):
        return str(meta.msg_type)
    if isinstance(meta, (tuple, list)) and meta:
        return str(meta[0])
    return ""


def _meta_count(meta) -> int:
    if hasattr(meta, "message_count"):
        return int(meta.message_count)
    if isinstance(meta, (tuple, list)) and len(meta) >= 2:
        return int(meta[1])
    return 0


def import_rosbag():
    try:
        import rosbag  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing ROS1 rosbag runtime. Run inside the ROS1 Python environment."
        ) from exc
    return rosbag


def collect_topics(bag) -> List[TopicMeta]:
    info = bag.get_type_and_topic_info()[1]
    topics: List[TopicMeta] = []
    for topic, meta in info.items():
        topics.append(
            TopicMeta(
                topic=topic,
                msg_type=_meta_type(meta),
                message_count=_meta_count(meta),
            )
        )
    return topics


def print_topics_table(topics: Sequence[TopicMeta]) -> None:
    print("topic\tmsg_type\tmessage_count")
    for item in sorted(topics, key=lambda x: x.topic):
        print(f"{item.topic}\t{item.msg_type}\t{item.message_count}")


def choose_joint_topic(topics: Sequence[TopicMeta], requested_topic: Optional[str]) -> str:
    joint_topics = [x.topic for x in topics if x.msg_type == "sensor_msgs/JointState"]
    if requested_topic:
        if requested_topic in joint_topics:
            return requested_topic
        raise RuntimeError(
            f"Requested --joint-topic not found or not JointState: {requested_topic}. "
            f"Available JointState topics: {joint_topics or 'none'}"
        )
    if "/joint_states" in joint_topics:
        return "/joint_states"
    if joint_topics:
        return joint_topics[0]
    raise RuntimeError("No sensor_msgs/JointState topics found in this bag.")


def msg_time_sec(msg, bag_time) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and hasattr(stamp, "to_sec"):
        sec = float(stamp.to_sec())
        if sec > 0.0:
            return sec
    return float(bag_time.to_sec())


def load_joint_series(bag, topic: str) -> JointSeries:
    t_values: List[float] = []
    q_values: List[np.ndarray] = []
    tau_values: List[np.ndarray] = []
    pos_dim: Optional[int] = None

    for _topic, msg, bag_time in bag.read_messages(topics=[topic]):
        q = np.asarray(getattr(msg, "position", []), dtype=np.float64)
        if q.ndim != 1 or q.size == 0:
            continue
        if pos_dim is None:
            pos_dim = int(q.size)
        if q.size != pos_dim:
            continue

        t_values.append(msg_time_sec(msg, bag_time))
        q_values.append(q)

        tau = np.asarray(getattr(msg, "effort", []), dtype=np.float64)
        if tau.ndim == 1 and tau.size == pos_dim:
            tau_values.append(tau)
        else:
            tau_values.append(np.full(pos_dim, np.nan, dtype=np.float64))

    if len(t_values) < 2:
        raise RuntimeError(f"Need at least 2 valid JointState samples on topic {topic}.")

    t = np.asarray(t_values, dtype=np.float64)
    q_arr = np.vstack(q_values)
    tau_arr = np.vstack(tau_values)

    order = np.argsort(t, kind="mergesort")
    t = t[order]
    q_arr = q_arr[order]
    tau_arr = tau_arr[order]

    tau_out: Optional[np.ndarray] = None
    if np.isfinite(tau_arr).sum() >= (2 * tau_arr.shape[1]):
        tau_out = tau_arr

    return JointSeries(t=t, q=q_arr, tau=tau_out)


def moving_average_matrix(x: np.ndarray, win: int) -> np.ndarray:
    if win <= 1:
        return x.copy()
    n, d = x.shape
    if n == 0:
        return x.copy()
    win = int(max(1, min(win, n)))
    if win == 1:
        return x.copy()
    left = win // 2
    right = win - 1 - left
    padded = np.pad(x, ((left, right), (0, 0)), mode="edge")
    csum = np.cumsum(padded, axis=0, dtype=np.float64)
    csum = np.vstack([np.zeros((1, d), dtype=np.float64), csum])
    return (csum[win:] - csum[:-win]) / float(win)


def differentiate_matrix(x: np.ndarray, t: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    out = np.zeros_like(x, dtype=np.float64)
    if x.shape[0] < 2:
        return out
    dt = np.diff(t)
    dt_safe = np.maximum(dt, eps)
    out[1:] = np.diff(x, axis=0) / dt_safe[:, None]
    out[0] = out[1]
    return out


def compute_signals(series: JointSeries, smooth_win: int) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    q_smoothed = moving_average_matrix(series.q, smooth_win)
    qdot = differentiate_matrix(q_smoothed, series.t)
    speed = np.linalg.norm(qdot, axis=1)

    qddot = differentiate_matrix(qdot, series.t)
    acc = np.linalg.norm(qddot, axis=1)

    dtau: Optional[np.ndarray] = None
    if series.tau is not None:
        tau_smoothed = moving_average_matrix(series.tau, smooth_win)
        valid = np.isfinite(tau_smoothed).all(axis=1)
        if int(valid.sum()) >= 2:
            t_valid = series.t[valid]
            tau_valid = tau_smoothed[valid]
            dtau_valid = np.linalg.norm(differentiate_matrix(tau_valid, t_valid), axis=1)
            if valid.all():
                dtau = dtau_valid
            else:
                t_unique, unique_idx = np.unique(t_valid, return_index=True)
                dtau_unique = dtau_valid[unique_idx]
                if t_unique.shape[0] >= 2:
                    dtau = np.interp(series.t, t_unique, dtau_unique, left=np.nan, right=np.nan)
    return speed, acc, dtau


def compute_norm_scale(speed: np.ndarray, norm_base: str) -> float:
    if norm_base == "median":
        return float(np.median(speed))
    if norm_base == "p90":
        return float(np.percentile(speed, 90.0))
    raise ValueError(f"Unsupported norm base: {norm_base}")


def normalize_speed(speed: np.ndarray, norm_base: str, norm_eps: float) -> Tuple[np.ndarray, float]:
    scale = compute_norm_scale(speed=speed, norm_base=norm_base)
    denom = scale + norm_eps
    if denom <= 0.0:
        denom = norm_eps
    return speed / denom, scale


def _ensure_hysteresis(
    on_th: float,
    off_th: float,
    *,
    auto_adjust_equal: bool,
) -> Tuple[float, float]:
    if off_th < on_th:
        return on_th, off_th
    if auto_adjust_equal:
        return on_th, float(np.nextafter(on_th, -np.inf))
    raise ValueError(f"off threshold must be < on threshold, got on={on_th} off={off_th}")


def merge_bursts(bursts: Sequence[BurstWindow], merge_gap: float) -> List[BurstWindow]:
    if merge_gap < 0.0:
        raise ValueError("merge_gap must be >= 0")
    if not bursts:
        return []

    sorted_bursts = sorted(bursts, key=lambda b: (b.start_s, b.end_s))
    merged: List[BurstWindow] = [sorted_bursts[0]]

    for cur in sorted_bursts[1:]:
        prev = merged[-1]
        gap = cur.start_s - prev.end_s
        if gap < merge_gap:
            merged[-1] = BurstWindow(start_s=prev.start_s, end_s=max(prev.end_s, cur.end_s))
        else:
            merged.append(cur)
    return merged


def merge_bursts_fill_holes(
    t: np.ndarray,
    hole_signal_norm: np.ndarray,
    bursts: Sequence[BurstWindow],
    merge_gap: float,
    hole_min_norm: float,
) -> List[BurstWindow]:
    if merge_gap < 0.0:
        raise ValueError("merge_gap must be >= 0")
    if hole_min_norm < 0.0:
        raise ValueError("hole_min_norm must be >= 0")
    if t.shape[0] != hole_signal_norm.shape[0]:
        raise ValueError("t and hole_signal_norm must have same length")
    if not bursts:
        return []

    sorted_bursts = sorted(bursts, key=lambda b: (b.start_s, b.end_s))
    merged: List[BurstWindow] = [sorted_bursts[0]]

    for cur in sorted_bursts[1:]:
        prev = merged[-1]
        gap = cur.start_s - prev.end_s

        # Overlap still merges directly.
        if gap <= 0.0:
            merged[-1] = BurstWindow(start_s=prev.start_s, end_s=max(prev.end_s, cur.end_s))
            continue

        if gap >= merge_gap:
            merged.append(cur)
            continue

        gap_idx = np.where((t > prev.end_s) & (t < cur.start_s))[0]
        if gap_idx.size > 0:
            can_fill = bool(np.all(hole_signal_norm[gap_idx] >= hole_min_norm))
        else:
            # If no sample falls strictly inside the hole, probe midpoint by interpolation.
            mid_t = 0.5 * (prev.end_s + cur.start_s)
            mid_val = float(np.interp(mid_t, t, hole_signal_norm))
            can_fill = mid_val >= hole_min_norm

        if can_fill:
            merged[-1] = BurstWindow(start_s=prev.start_s, end_s=max(prev.end_s, cur.end_s))
        else:
            merged.append(cur)

    return merged


def detect_burst_windows(
    t: np.ndarray,
    signal: np.ndarray,
    hole_signal_norm: np.ndarray,
    on_th: float,
    off_th: float,
    min_dur: float,
    pad: float,
    merge_gap: float,
    merge_mode: str,
    hole_min_norm: float,
) -> List[BurstWindow]:
    raw: List[Tuple[int, int]] = []
    active = False
    start_idx = 0

    for idx, value in enumerate(signal):
        if not active and value >= on_th:
            active = True
            start_idx = idx
            continue
        if active and value <= off_th:
            raw.append((start_idx, idx))
            active = False

    if active:
        raw.append((start_idx, signal.shape[0] - 1))

    windows: List[BurstWindow] = []
    t0 = float(t[0])
    t1 = float(t[-1])

    for s_idx, e_idx in raw:
        start_s = float(t[s_idx])
        end_s = float(t[e_idx])
        if end_s < start_s:
            start_s, end_s = end_s, start_s
        if (end_s - start_s) < min_dur:
            continue
        if pad > 0.0:
            start_s = max(t0, start_s - pad)
            end_s = min(t1, end_s + pad)
        windows.append(BurstWindow(start_s=start_s, end_s=end_s))

    if merge_mode == "gap":
        return merge_bursts(windows, merge_gap=merge_gap)
    if merge_mode == "fill_holes":
        return merge_bursts_fill_holes(
            t=t,
            hole_signal_norm=hole_signal_norm,
            bursts=windows,
            merge_gap=merge_gap,
            hole_min_norm=hole_min_norm,
        )
    raise ValueError(f"Unsupported merge_mode: {merge_mode}")


def filter_bursts_by_peak(
    t: np.ndarray,
    speed: np.ndarray,
    speed_norm: np.ndarray,
    windows: Sequence[BurstWindow],
    peak_min_raw: float,
    peak_min_norm: float,
) -> List[BurstWindow]:
    if peak_min_raw <= 0.0 and peak_min_norm <= 0.0:
        return list(windows)

    filtered: List[BurstWindow] = []
    for window in windows:
        idx = _window_indices(t, window)
        peak_raw = float(np.max(speed[idx]))
        peak_norm = float(np.max(speed_norm[idx]))

        if peak_min_raw > 0.0 and peak_raw < peak_min_raw:
            continue
        if peak_min_norm > 0.0 and peak_norm < peak_min_norm:
            continue
        filtered.append(window)
    return filtered


def _window_indices(t: np.ndarray, window: BurstWindow) -> np.ndarray:
    idx = np.where((t >= window.start_s) & (t <= window.end_s))[0]
    if idx.size > 0:
        return idx
    center = 0.5 * (window.start_s + window.end_s)
    nearest = int(np.argmin(np.abs(t - center)))
    return np.asarray([nearest], dtype=np.int64)


def _to_float(value: Any, key: str) -> float:
    try:
        return float(value)
    except Exception as exc:
        raise ValueError(f"Invalid numeric value for '{key}': {value}") from exc


def load_global_stats(path: Path) -> Dict[str, Any]:
    p = path.expanduser().resolve()
    if not p.is_file():
        raise ValueError(f"global stats file not found: {p}")

    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("global stats JSON must be an object")

    version = int(data.get("version", 1))
    if version != 1:
        raise ValueError(f"Unsupported global stats version: {version}")

    signal_space = str(data.get("signal_space", "")).strip()
    if signal_space not in SIGNAL_SPACES:
        raise ValueError(f"global stats 'signal_space' must be one of {SIGNAL_SPACES}")

    on_th = _to_float(data.get("on_th"), "on_th")
    off_th = _to_float(data.get("off_th"), "off_th")
    if off_th >= on_th:
        raise ValueError("global stats requires off_th < on_th")

    out: Dict[str, Any] = {
        "version": version,
        "signal_space": signal_space,
        "on_th": on_th,
        "off_th": off_th,
    }

    if signal_space == "speed_norm":
        norm_base = str(data.get("norm_base", "")).strip()
        if norm_base not in NORM_BASES:
            raise ValueError(f"global stats 'norm_base' must be one of {NORM_BASES} for speed_norm")
        norm_eps = _to_float(data.get("norm_eps"), "norm_eps")
        if norm_eps <= 0.0:
            raise ValueError("global stats 'norm_eps' must be > 0")
        out["norm_base"] = norm_base
        out["norm_eps"] = norm_eps

    return out


def resolve_threshold_info(
    speed: np.ndarray,
    threshold_mode: str,
    on_pct: float,
    off_pct: float,
    norm_base: str,
    norm_eps: float,
    on_abs: float,
    off_abs: float,
    global_stats: Optional[Path],
) -> Tuple[np.ndarray, ThresholdInfo]:
    if threshold_mode == "percentile":
        on_th = float(np.percentile(speed, on_pct))
        off_th = float(np.percentile(speed, off_pct))
        on_th, off_th = _ensure_hysteresis(on_th=on_th, off_th=off_th, auto_adjust_equal=True)
        return speed, ThresholdInfo(
            mode=threshold_mode,
            signal_space="speed",
            norm_base="none",
            norm_scale=float("nan"),
            norm_eps=float("nan"),
            on_th_signal=on_th,
            off_th_signal=off_th,
            on_th_speed_equiv=on_th,
            off_th_speed_equiv=off_th,
        )

    if threshold_mode == "norm_percentile":
        speed_norm, scale = normalize_speed(speed=speed, norm_base=norm_base, norm_eps=norm_eps)
        on_th = float(np.percentile(speed_norm, on_pct))
        off_th = float(np.percentile(speed_norm, off_pct))
        on_th, off_th = _ensure_hysteresis(on_th=on_th, off_th=off_th, auto_adjust_equal=True)
        denom = scale + norm_eps
        return speed_norm, ThresholdInfo(
            mode=threshold_mode,
            signal_space="speed_norm",
            norm_base=norm_base,
            norm_scale=scale,
            norm_eps=norm_eps,
            on_th_signal=on_th,
            off_th_signal=off_th,
            on_th_speed_equiv=float(on_th * denom),
            off_th_speed_equiv=float(off_th * denom),
        )

    if threshold_mode == "norm_fixed":
        speed_norm, scale = normalize_speed(speed=speed, norm_base=norm_base, norm_eps=norm_eps)
        on_th, off_th = _ensure_hysteresis(on_th=on_abs, off_th=off_abs, auto_adjust_equal=False)
        denom = scale + norm_eps
        return speed_norm, ThresholdInfo(
            mode=threshold_mode,
            signal_space="speed_norm",
            norm_base=norm_base,
            norm_scale=scale,
            norm_eps=norm_eps,
            on_th_signal=float(on_th),
            off_th_signal=float(off_th),
            on_th_speed_equiv=float(on_th * denom),
            off_th_speed_equiv=float(off_th * denom),
        )

    if threshold_mode == "global_percentile":
        if global_stats is None:
            raise ValueError("--global-stats is required for threshold-mode=global_percentile")

        stats = load_global_stats(global_stats)
        signal_space = str(stats["signal_space"])
        on_th = float(stats["on_th"])
        off_th = float(stats["off_th"])

        if signal_space == "speed":
            on_th, off_th = _ensure_hysteresis(on_th=on_th, off_th=off_th, auto_adjust_equal=False)
            return speed, ThresholdInfo(
                mode=threshold_mode,
                signal_space="speed",
                norm_base="none",
                norm_scale=float("nan"),
                norm_eps=float("nan"),
                on_th_signal=on_th,
                off_th_signal=off_th,
                on_th_speed_equiv=on_th,
                off_th_speed_equiv=off_th,
            )

        global_norm_base = str(stats["norm_base"])
        global_norm_eps = float(stats["norm_eps"])
        speed_norm, scale = normalize_speed(speed=speed, norm_base=global_norm_base, norm_eps=global_norm_eps)
        on_th, off_th = _ensure_hysteresis(on_th=on_th, off_th=off_th, auto_adjust_equal=False)
        denom = scale + global_norm_eps
        return speed_norm, ThresholdInfo(
            mode=threshold_mode,
            signal_space="speed_norm",
            norm_base=global_norm_base,
            norm_scale=scale,
            norm_eps=global_norm_eps,
            on_th_signal=on_th,
            off_th_signal=off_th,
            on_th_speed_equiv=float(on_th * denom),
            off_th_speed_equiv=float(off_th * denom),
        )

    raise ValueError(f"Unsupported threshold mode: {threshold_mode}")


def build_bursts_table(
    t: np.ndarray,
    speed: np.ndarray,
    acc: np.ndarray,
    dtau: Optional[np.ndarray],
    windows: Sequence[BurstWindow],
    threshold_info: ThresholdInfo,
    merge_gap_s: float,
    merge_mode: str,
    hole_min_norm: float,
    peak_min_norm: float,
    peak_min_raw: float,
    peak_filter_norm_base: str,
    peak_filter_norm_scale: float,
    peak_filter_norm_eps: float,
    t_abs: Optional[np.ndarray] = None,
) -> pd.DataFrame:
    if t_abs is None:
        t_abs = np.full_like(t, np.nan, dtype=np.float64)
    t0_abs = float(t_abs[0]) if t_abs.size > 0 else float("nan")

    rows = []
    for burst_id, window in enumerate(windows):
        idx = _window_indices(t, window)

        i_speed = int(idx[np.argmax(speed[idx])])
        i_acc = int(idx[np.argmax(acc[idx])])

        peak_dtau = float("nan")
        t_peak_dtau = float("nan")
        if dtau is not None:
            finite_idx = idx[np.isfinite(dtau[idx])]
            if finite_idx.size > 0:
                i_dtau = int(finite_idx[np.argmax(dtau[finite_idx])])
                peak_dtau = float(dtau[i_dtau])
                t_peak_dtau = float(t[i_dtau])
                t_peak_dtau_abs = float(t_abs[i_dtau])
            else:
                t_peak_dtau_abs = float("nan")
        else:
            t_peak_dtau_abs = float("nan")

        rows.append(
            {
                "burst_id": int(burst_id),
                "start_s": float(window.start_s),
                "end_s": float(window.end_s),
                "duration_s": float(max(0.0, window.end_s - window.start_s)),
                "peak_speed": float(speed[i_speed]),
                "t_peak_speed_s": float(t[i_speed]),
                "peak_acc": float(acc[i_acc]),
                "t_peak_acc_s": float(t[i_acc]),
                "peak_dtau": peak_dtau,
                "t_peak_dtau_s": t_peak_dtau,
                "start_abs_s": float(window.start_s + t0_abs),
                "end_abs_s": float(window.end_s + t0_abs),
                "t_peak_speed_abs_s": float(t_abs[i_speed]),
                "t_peak_acc_abs_s": float(t_abs[i_acc]),
                "t_peak_dtau_abs_s": t_peak_dtau_abs,
                "threshold_mode": threshold_info.mode,
                "threshold_signal_space": threshold_info.signal_space,
                "norm_base": threshold_info.norm_base,
                "norm_scale": threshold_info.norm_scale,
                "norm_eps": threshold_info.norm_eps,
                "on_th_signal": threshold_info.on_th_signal,
                "off_th_signal": threshold_info.off_th_signal,
                "on_th_speed_equiv": threshold_info.on_th_speed_equiv,
                "off_th_speed_equiv": threshold_info.off_th_speed_equiv,
                "merge_gap_s": float(merge_gap_s),
                "merge_mode": merge_mode,
                "hole_min_norm": float(hole_min_norm),
                "peak_min_norm": float(peak_min_norm),
                "peak_min_raw": float(peak_min_raw),
                "peak_filter_norm_base": peak_filter_norm_base,
                "peak_filter_norm_scale": float(peak_filter_norm_scale),
                "peak_filter_norm_eps": float(peak_filter_norm_eps),
            }
        )
    return pd.DataFrame(rows, columns=CSV_COLUMNS)


def plot_timeseries(
    out_png: Path,
    episode_name: str,
    t: np.ndarray,
    speed: np.ndarray,
    acc: np.ndarray,
    dtau: Optional[np.ndarray],
    on_th_speed_equiv: float,
    off_th_speed_equiv: float,
    threshold_mode: str,
    threshold_signal_space: str,
    windows: Sequence[BurstWindow],
    speed_ymin: Optional[float] = None,
    speed_ymax: Optional[float] = None,
) -> None:
    fig, ax1 = plt.subplots(figsize=(12, 4.5))
    ax1.plot(t, speed, color="tab:blue", linewidth=1.3, label="speed ||dq/dt||")
    ax1.axhline(
        on_th_speed_equiv,
        color="tab:orange",
        linestyle="--",
        linewidth=1.2,
        label=f"on_th(raw)={on_th_speed_equiv:.4f}",
    )
    ax1.axhline(
        off_th_speed_equiv,
        color="tab:gray",
        linestyle="--",
        linewidth=1.2,
        label=f"off_th(raw)={off_th_speed_equiv:.4f}",
    )

    for window in windows:
        ax1.axvspan(window.start_s, window.end_s, color="tab:orange", alpha=0.18)

    if speed_ymin is not None or speed_ymax is not None:
        cur_ymin, cur_ymax = ax1.get_ylim()
        y_min = speed_ymin if speed_ymin is not None else float(cur_ymin)
        y_max = speed_ymax if speed_ymax is not None else float(cur_ymax)
        if y_max <= y_min:
            raise ValueError(f"Invalid speed axis limits: ymin={y_min} ymax={y_max}")
        ax1.set_ylim(y_min, y_max)

    ax1.set_xlabel("time from episode start [s]")
    ax1.set_ylabel("speed")
    ax1.set_title(f"{episode_name} | Burst Detection ({threshold_mode}, {threshold_signal_space})")
    ax1.grid(True, alpha=0.25)

    ax2 = ax1.twinx()
    ax2.plot(t, acc, color="tab:green", alpha=0.75, linewidth=1.0, label="acc ||d2q/dt2||")
    if dtau is not None:
        ax2.plot(t, dtau, color="tab:red", alpha=0.75, linewidth=1.0, label="dtau ||d tau/dt||")
    ax2.set_ylabel("acc / contact proxy")

    handles_1, labels_1 = ax1.get_legend_handles_labels()
    handles_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(handles_1 + handles_2, labels_1 + labels_2, loc="upper right", fontsize=9)

    fig.tight_layout()
    fig.savefig(str(out_png), dpi=150)
    plt.close(fig)


def validate_knobs(smooth_win: int, on_pct: float, off_pct: float, min_dur: float, pad: float) -> None:
    if smooth_win < 1:
        raise ValueError("--smooth-win must be >= 1")
    if on_pct < 0.0 or on_pct > 100.0:
        raise ValueError("--on-pct must be in [0, 100]")
    if off_pct < 0.0 or off_pct > 100.0:
        raise ValueError("--off-pct must be in [0, 100]")
    if min_dur < 0.0:
        raise ValueError("--min-dur must be >= 0")
    if pad < 0.0:
        raise ValueError("--pad must be >= 0")


def validate_speed_axis(speed_ymin: Optional[float], speed_ymax: Optional[float]) -> None:
    if speed_ymin is not None and speed_ymax is not None and speed_ymax <= speed_ymin:
        raise ValueError("--speed-ymax must be > --speed-ymin")


def validate_threshold_options(
    threshold_mode: str,
    norm_base: str,
    norm_eps: float,
    on_pct: float,
    off_pct: float,
    on_abs: float,
    off_abs: float,
    global_stats: Optional[str],
    merge_gap: float,
    merge_mode: str,
    hole_min_norm: float,
    peak_min_norm: float,
    peak_min_raw: float,
) -> None:
    if threshold_mode not in THRESHOLD_MODES:
        raise ValueError(f"--threshold-mode must be one of {THRESHOLD_MODES}")
    if norm_base not in NORM_BASES:
        raise ValueError(f"--norm-base must be one of {NORM_BASES}")
    if norm_eps <= 0.0:
        raise ValueError("--norm-eps must be > 0")
    if merge_gap < 0.0:
        raise ValueError("--merge-gap must be >= 0")
    if merge_mode not in MERGE_MODES:
        raise ValueError(f"--merge-mode must be one of {MERGE_MODES}")
    if hole_min_norm < 0.0:
        raise ValueError("--hole-min-norm must be >= 0")
    if peak_min_norm < 0.0:
        raise ValueError("--peak-min-norm must be >= 0")
    if peak_min_raw < 0.0:
        raise ValueError("--peak-min-raw must be >= 0")

    if threshold_mode in ("percentile", "norm_percentile"):
        if off_pct >= on_pct:
            raise ValueError("--off-pct must be < --on-pct for percentile-based modes")

    if threshold_mode == "norm_fixed":
        if off_abs >= on_abs:
            raise ValueError("--off-abs must be < --on-abs for norm_fixed mode")

    if threshold_mode == "global_percentile":
        if not global_stats:
            raise ValueError("--global-stats is required for threshold-mode=global_percentile")


def scan_episode_speed_range(
    bag_path: Path,
    joint_topic: Optional[str],
    smooth_win: int,
) -> Tuple[float, float]:
    rosbag = import_rosbag()
    bag_path = bag_path.expanduser().resolve()
    with rosbag.Bag(str(bag_path), "r") as bag:
        topics = collect_topics(bag)
        chosen_topic = choose_joint_topic(topics=topics, requested_topic=joint_topic)
        series = load_joint_series(bag, topic=chosen_topic)
    speed, _acc, _dtau = compute_signals(series=series, smooth_win=smooth_win)
    return float(np.min(speed)), float(np.max(speed))


def list_topics_in_bag(bag_path: Path) -> None:
    rosbag = import_rosbag()
    with rosbag.Bag(str(bag_path), "r") as bag:
        print_topics_table(collect_topics(bag))


def run_episode(
    bag_path: Path,
    outdir: Path,
    episode_name: Optional[str],
    csv_out_path: Optional[Path],
    plot_out_path: Optional[Path],
    joint_topic: Optional[str],
    smooth_win: int,
    on_pct: float,
    off_pct: float,
    min_dur: float,
    pad: float,
    speed_ymin: Optional[float] = None,
    speed_ymax: Optional[float] = None,
    threshold_mode: str = "percentile",
    norm_base: str = "p90",
    norm_eps: float = 1e-6,
    on_abs: float = 2.0,
    off_abs: float = 1.3,
    global_stats: Optional[str] = None,
    merge_gap: float = 0.12,
    merge_mode: str = "gap",
    hole_min_norm: float = 0.5,
    peak_min_norm: float = 0.0,
    peak_min_raw: float = 0.0,
    print_summary: bool = True,
) -> EpisodeResult:
    validate_knobs(smooth_win=smooth_win, on_pct=on_pct, off_pct=off_pct, min_dur=min_dur, pad=pad)
    validate_speed_axis(speed_ymin=speed_ymin, speed_ymax=speed_ymax)
    validate_threshold_options(
        threshold_mode=threshold_mode,
        norm_base=norm_base,
        norm_eps=norm_eps,
        on_pct=on_pct,
        off_pct=off_pct,
        on_abs=on_abs,
        off_abs=off_abs,
        global_stats=global_stats,
        merge_gap=merge_gap,
        merge_mode=merge_mode,
        hole_min_norm=hole_min_norm,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
    )

    rosbag = import_rosbag()
    bag_path = bag_path.expanduser().resolve()
    outdir = outdir.expanduser()
    outdir.mkdir(parents=True, exist_ok=True)
    resolved_episode_name = episode_name if episode_name else bag_path.stem
    resolved_csv_out = (
        csv_out_path.expanduser()
        if csv_out_path is not None
        else (outdir / "csv" / f"{resolved_episode_name}.csv")
    )
    resolved_plot_out = (
        plot_out_path.expanduser()
        if plot_out_path is not None
        else (outdir / "plots" / f"{resolved_episode_name}.png")
    )
    resolved_csv_out.parent.mkdir(parents=True, exist_ok=True)
    resolved_plot_out.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(bag_path), "r") as bag:
        topics = collect_topics(bag)
        chosen_topic = choose_joint_topic(topics=topics, requested_topic=joint_topic)
        series = load_joint_series(bag, topic=chosen_topic)

    speed, acc, dtau = compute_signals(series=series, smooth_win=smooth_win)
    t_rel = series.t - float(series.t[0])

    signal, threshold_info = resolve_threshold_info(
        speed=speed,
        threshold_mode=threshold_mode,
        on_pct=on_pct,
        off_pct=off_pct,
        norm_base=norm_base,
        norm_eps=norm_eps,
        on_abs=on_abs,
        off_abs=off_abs,
        global_stats=(Path(global_stats) if global_stats else None),
    )

    peak_filter_norm_base = norm_base
    peak_filter_norm_eps = norm_eps
    if threshold_info.signal_space == "speed_norm" and threshold_info.norm_base in NORM_BASES:
        peak_filter_norm_base = threshold_info.norm_base
        peak_filter_norm_eps = threshold_info.norm_eps
    speed_norm_for_peak, peak_filter_norm_scale = normalize_speed(
        speed=speed,
        norm_base=peak_filter_norm_base,
        norm_eps=peak_filter_norm_eps,
    )

    windows = detect_burst_windows(
        t=t_rel,
        signal=signal,
        hole_signal_norm=speed_norm_for_peak,
        on_th=threshold_info.on_th_signal,
        off_th=threshold_info.off_th_signal,
        min_dur=min_dur,
        pad=pad,
        merge_gap=merge_gap,
        merge_mode=merge_mode,
        hole_min_norm=hole_min_norm,
    )
    windows = filter_bursts_by_peak(
        t=t_rel,
        speed=speed,
        speed_norm=speed_norm_for_peak,
        windows=windows,
        peak_min_raw=peak_min_raw,
        peak_min_norm=peak_min_norm,
    )

    bursts_df = build_bursts_table(
        t=t_rel,
        speed=speed,
        acc=acc,
        dtau=dtau,
        windows=windows,
        threshold_info=threshold_info,
        merge_gap_s=merge_gap,
        merge_mode=merge_mode,
        hole_min_norm=hole_min_norm,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
        peak_filter_norm_base=peak_filter_norm_base,
        peak_filter_norm_scale=peak_filter_norm_scale,
        peak_filter_norm_eps=peak_filter_norm_eps,
        t_abs=series.t,
    )
    bursts_df.to_csv(resolved_csv_out, index=False)

    plot_timeseries(
        out_png=resolved_plot_out,
        episode_name=resolved_episode_name,
        t=t_rel,
        speed=speed,
        acc=acc,
        dtau=dtau,
        on_th_speed_equiv=threshold_info.on_th_speed_equiv,
        off_th_speed_equiv=threshold_info.off_th_speed_equiv,
        threshold_mode=threshold_info.mode,
        threshold_signal_space=threshold_info.signal_space,
        windows=windows,
        speed_ymin=speed_ymin,
        speed_ymax=speed_ymax,
    )

    if bursts_df.empty:
        num_bursts = 0
        total_burst_time = 0.0
        mean_peak_speed = 0.0
        max_peak_speed = 0.0
        mean_duration = 0.0
    else:
        num_bursts = int(bursts_df.shape[0])
        total_burst_time = float(bursts_df["duration_s"].sum())
        mean_peak_speed = float(bursts_df["peak_speed"].mean())
        max_peak_speed = float(bursts_df["peak_speed"].max())
        mean_duration = float(bursts_df["duration_s"].mean())

    if print_summary:
        print(
            "mode={mode} signal={signal} norm_base={norm_base} on_th={on:.6f} off_th={off:.6f} "
            "on_th_speed={on_spd:.6f} off_th_speed={off_spd:.6f} "
            "merge_mode={merge_mode} merge_gap={merge_gap:.3f}s hole_min_norm={hole_min_norm:.3f} "
            "peak_min_norm={peak_norm:.3f} peak_min_raw={peak_raw:.3f} "
            "bursts={num} total_burst_time={total:.6f}s mean_peak_speed={mean_peak:.6f} "
            "max_peak_speed={max_peak:.6f}".format(
                mode=threshold_info.mode,
                signal=threshold_info.signal_space,
                norm_base=threshold_info.norm_base,
                on=threshold_info.on_th_signal,
                off=threshold_info.off_th_signal,
                on_spd=threshold_info.on_th_speed_equiv,
                off_spd=threshold_info.off_th_speed_equiv,
                merge_mode=merge_mode,
                merge_gap=merge_gap,
                hole_min_norm=hole_min_norm,
                peak_norm=peak_min_norm,
                peak_raw=peak_min_raw,
                num=num_bursts,
                total=total_burst_time,
                mean_peak=mean_peak_speed,
                max_peak=max_peak_speed,
            )
        )

    return EpisodeResult(
        bag_path=bag_path,
        outdir=outdir,
        episode_name=resolved_episode_name,
        bursts_csv_path=resolved_csv_out,
        timeseries_png_path=resolved_plot_out,
        joint_topic=chosen_topic,
        num_bursts=num_bursts,
        total_burst_time=total_burst_time,
        mean_peak_speed=mean_peak_speed,
        max_peak_speed=max_peak_speed,
        mean_duration=mean_duration,
        on_th=threshold_info.on_th_signal,
        off_th=threshold_info.off_th_signal,
        threshold_mode=threshold_info.mode,
        threshold_signal_space=threshold_info.signal_space,
        norm_base=threshold_info.norm_base,
        norm_scale=threshold_info.norm_scale,
        norm_eps=threshold_info.norm_eps,
        on_th_speed_equiv=threshold_info.on_th_speed_equiv,
        off_th_speed_equiv=threshold_info.off_th_speed_equiv,
        merge_gap_s=merge_gap,
        merge_mode=merge_mode,
        hole_min_norm=hole_min_norm,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
        peak_filter_norm_base=peak_filter_norm_base,
        peak_filter_norm_scale=peak_filter_norm_scale,
        peak_filter_norm_eps=peak_filter_norm_eps,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to a ROS1 episode .bag file.")
    parser.add_argument(
        "--outdir",
        default="",
        help="Output root (default: burst_out). Writes csv/<episode>.csv and plots/<episode>.png.",
    )
    parser.add_argument(
        "--joint-topic",
        default="",
        help="JointState topic. If omitted, auto-pick /joint_states else first JointState topic.",
    )
    parser.add_argument("--list-topics", action="store_true", help="List topics + message types and exit.")
    parser.add_argument(
        "--smooth-win",
        type=int,
        default=1,
        help="Moving-average window (samples) before differencing. 1 = no smoothing.",
    )
    parser.add_argument(
        "--on-pct",
        type=float,
        default=95.0,
        help="Speed percentile used as burst enter threshold.",
    )
    parser.add_argument(
        "--off-pct",
        type=float,
        default=80.0,
        help="Speed percentile used as burst exit threshold.",
    )
    parser.add_argument(
        "--min-dur",
        type=float,
        default=0.08,
        help="Minimum burst duration in seconds.",
    )
    parser.add_argument(
        "--pad",
        type=float,
        default=0.0,
        help="Pad each burst interval by this many seconds on both sides.",
    )
    parser.add_argument(
        "--merge-gap",
        type=float,
        default=0.12,
        help="Merge consecutive bursts when gap < merge-gap (seconds).",
    )
    parser.add_argument(
        "--merge-mode",
        choices=MERGE_MODES,
        default="gap",
        help="Merge strategy: 'gap' (time-only) or 'fill_holes' (merge short holes only if v_norm stays high).",
    )
    parser.add_argument(
        "--hole-min-norm",
        type=float,
        default=0.5,
        help="For merge-mode=fill_holes, require v_norm in the hole to stay >= this value.",
    )
    parser.add_argument(
        "--threshold-mode",
        choices=THRESHOLD_MODES,
        default="percentile",
        help="Threshold mode for burst detection.",
    )
    parser.add_argument(
        "--norm-base",
        choices=NORM_BASES,
        default="p90",
        help="Normalization base for norm_* modes.",
    )
    parser.add_argument(
        "--norm-eps",
        type=float,
        default=1e-6,
        help="Epsilon used in normalized speed denominator.",
    )
    parser.add_argument(
        "--on-abs",
        type=float,
        default=2.0,
        help="Fixed ON threshold in normalized signal space for norm_fixed mode.",
    )
    parser.add_argument(
        "--off-abs",
        type=float,
        default=1.3,
        help="Fixed OFF threshold in normalized signal space for norm_fixed mode.",
    )
    parser.add_argument(
        "--global-stats",
        default="",
        help="Global threshold JSON file used by global_percentile mode.",
    )
    parser.add_argument(
        "--peak-min-norm",
        type=float,
        default=0.0,
        help="Keep burst only if max(v_norm) in window >= this value. 0 disables.",
    )
    parser.add_argument(
        "--peak-min-raw",
        type=float,
        default=0.0,
        help="Keep burst only if max(v_raw) in window >= this value. 0 disables.",
    )
    parser.add_argument(
        "--speed-ymin",
        type=float,
        default=None,
        help="Optional fixed ymin for speed axis in timeseries plot.",
    )
    parser.add_argument(
        "--speed-ymax",
        type=float,
        default=None,
        help="Optional fixed ymax for speed axis in timeseries plot.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser()
    if not bag_path.is_file():
        raise SystemExit(f"Bag file not found: {bag_path}")

    if args.list_topics:
        list_topics_in_bag(bag_path)
        return 0

    try:
        validate_knobs(
            smooth_win=int(args.smooth_win),
            on_pct=float(args.on_pct),
            off_pct=float(args.off_pct),
            min_dur=float(args.min_dur),
            pad=float(args.pad),
        )
        validate_speed_axis(
            speed_ymin=(None if args.speed_ymin is None else float(args.speed_ymin)),
            speed_ymax=(None if args.speed_ymax is None else float(args.speed_ymax)),
        )
        validate_threshold_options(
            threshold_mode=str(args.threshold_mode),
            norm_base=str(args.norm_base),
            norm_eps=float(args.norm_eps),
            on_pct=float(args.on_pct),
            off_pct=float(args.off_pct),
            on_abs=float(args.on_abs),
            off_abs=float(args.off_abs),
            global_stats=(args.global_stats or None),
            merge_gap=float(args.merge_gap),
            merge_mode=str(args.merge_mode),
            hole_min_norm=float(args.hole_min_norm),
            peak_min_norm=float(args.peak_min_norm),
            peak_min_raw=float(args.peak_min_raw),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    outdir = Path(args.outdir).expanduser() if args.outdir else Path("burst_out")
    run_episode(
        bag_path=bag_path,
        outdir=outdir,
        episode_name=bag_path.stem,
        csv_out_path=None,
        plot_out_path=None,
        joint_topic=(args.joint_topic or None),
        smooth_win=int(args.smooth_win),
        on_pct=float(args.on_pct),
        off_pct=float(args.off_pct),
        min_dur=float(args.min_dur),
        pad=float(args.pad),
        speed_ymin=(None if args.speed_ymin is None else float(args.speed_ymin)),
        speed_ymax=(None if args.speed_ymax is None else float(args.speed_ymax)),
        threshold_mode=str(args.threshold_mode),
        norm_base=str(args.norm_base),
        norm_eps=float(args.norm_eps),
        on_abs=float(args.on_abs),
        off_abs=float(args.off_abs),
        global_stats=(args.global_stats or None),
        merge_gap=float(args.merge_gap),
        merge_mode=str(args.merge_mode),
        hole_min_norm=float(args.hole_min_norm),
        peak_min_norm=float(args.peak_min_norm),
        peak_min_raw=float(args.peak_min_raw),
        print_summary=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
