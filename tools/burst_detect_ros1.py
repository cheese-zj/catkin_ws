#!/usr/bin/env python3
"""Detect high-speed joint bursts from a single ROS1 bag episode."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


THRESHOLD_MODES = ("percentile", "norm_percentile", "norm_fixed", "global_percentile")
NORM_BASES = ("median", "p90")
SIGNAL_SPACES = ("speed", "speed_norm")
MERGE_MODES = ("gap", "fill_holes")
DETECTION_MODES = ("hysteresis", "event_centered")
EVENT_RANK_BY = ("snap_score", "peak_speed")
SNAP_SPEED_PEAK_MODES = ("max", "ma", "energy")
SNAP_TIME_MODES = ("score_peak", "speed_peak")


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
    "hole_quantile",
    "peak_min_norm",
    "peak_min_raw",
    "peak_filter_norm_base",
    "peak_filter_norm_scale",
    "peak_filter_norm_eps",
    "detection_mode",
    "snap_time_s",
    "snap_time_abs_s",
    "snap_score",
    "snap_score_th",
    "snap_speed_local_peak",
    "snap_speed_rel_th",
    "windup_start_s",
    "windup_end_s",
    "burst_start_s",
    "burst_end_s",
    "follow_start_s",
    "follow_end_s",
    "windup_duration_s",
    "burst_duration_s",
    "follow_duration_s",
    "burst_energy_search_start_s",
    "burst_energy_search_end_s",
    "energy_total_search",
    "energy_burst",
    "energy_windup",
    "energy_follow",
    "energy_burst_share",
    "context_energy_ratio",
    "energy_density_contrast",
    "z_amp_speed",
    "z_amp_acc",
    "z_dur",
    "z_dir_unit",
    "z_shape_dct",
    "score_norm_base",
    "score_norm_eps",
    "dtau_weight",
    "event_speed_alpha",
    "event_below_frames",
    "snap_peak_pct",
    "snap_peak_min_score",
    "snap_min_speed_norm",
    "snap_min_gap_s",
    "snap_peak_search_s",
    "snap_time_mode",
    "snap_speed_peak_mode",
    "snap_speed_peak_window_s",
    "burst_energy_rho",
    "burst_energy_left_s",
    "burst_energy_right_s",
    "shape_dct_k",
    "event_keep_topk",
    "event_rank_by",
    "event_force_primary",
    "event_attach_gap_s",
    "event_attach_peak_ratio",
    "windup_rest_frames",
    "windup_rest_speed_scale",
    "windup_rest_lookback_s",
    "windup_rest_min_gap_s",
    "follow_rest_frames",
    "follow_rest_speed_scale",
    "follow_rest_lookahead_s",
    "follow_rest_min_gap_s",
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


@dataclass(frozen=True)
class EventWindow:
    window: BurstWindow
    snap_idx: int
    speed_peak_idx: int
    snap_score: float
    score_th: float
    speed_peak: float
    speed_rel_th: float


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
    mean_windup_duration_s: float
    mean_burst_duration_s: float
    mean_follow_duration_s: float
    mean_burst_energy_share: float
    median_burst_energy_share: float
    mean_context_energy_ratio: float
    median_context_energy_ratio: float
    mean_energy_density_contrast: float
    median_energy_density_contrast: float
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
    hole_quantile: float
    peak_min_norm: float
    peak_min_raw: float
    peak_filter_norm_base: str
    peak_filter_norm_scale: float
    peak_filter_norm_eps: float
    detection_mode: str
    score_norm_base: str
    score_norm_eps: float
    dtau_weight: float
    event_speed_alpha: float
    event_below_frames: int
    snap_peak_pct: float
    snap_peak_min_score: float
    snap_min_speed_norm: float
    snap_min_gap_s: float
    snap_peak_search_s: float
    snap_time_mode: str
    snap_speed_peak_mode: str
    snap_speed_peak_window_s: float
    burst_energy_rho: float
    burst_energy_left_s: float
    burst_energy_right_s: float
    shape_dct_k: int
    event_keep_topk: int
    event_rank_by: str
    event_force_primary: bool
    event_attach_gap_s: float
    event_attach_peak_ratio: float
    windup_rest_frames: int
    windup_rest_speed_scale: float
    windup_rest_lookback_s: float
    windup_rest_min_gap_s: float
    follow_rest_frames: int
    follow_rest_speed_scale: float
    follow_rest_lookahead_s: float
    follow_rest_min_gap_s: float


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
        return rosbag
    except Exception:
        pass

    try:
        from rosbags.highlevel import AnyReader  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing ROS1 rosbag runtime. Either run inside ROS1 Python environment "
            "or install `rosbags` in current environment."
        ) from exc

    def _canonical_msg_type(msgtype: str) -> str:
        # rosbags uses ROS2-style names like sensor_msgs/msg/JointState.
        if "/msg/" in msgtype:
            return msgtype.replace("/msg/", "/")
        return msgtype

    @dataclass(frozen=True)
    class _RosbagsTopicMeta:
        msg_type: str
        message_count: int

    class _RosbagsBagTime:
        __slots__ = ("_sec",)

        def __init__(self, sec: float) -> None:
            self._sec = float(sec)

        def to_sec(self) -> float:
            return self._sec

    class _RosbagsBag:
        def __init__(self, path: str, mode: str = "r") -> None:
            if mode != "r":
                raise RuntimeError("rosbags fallback supports read-only mode.")
            self._path = Path(path).expanduser().resolve()
            self._reader = None
            self._connections = []

        def __enter__(self):
            reader = AnyReader([self._path])
            reader.open()
            self._reader = reader
            self._connections = list(reader.connections)
            return self

        def __exit__(self, exc_type, exc, tb) -> bool:
            if self._reader is not None:
                self._reader.close()
                self._reader = None
            self._connections = []
            return False

        def get_type_and_topic_info(self):
            if self._reader is None:
                raise RuntimeError("Bag is not opened.")
            topic_info: Dict[str, _RosbagsTopicMeta] = {}
            for conn in self._connections:
                topic = str(conn.topic)
                msg_type = _canonical_msg_type(str(conn.msgtype))
                message_count = int(getattr(conn, "msgcount", 0))
                if topic in topic_info:
                    prev = topic_info[topic]
                    topic_info[topic] = _RosbagsTopicMeta(
                        msg_type=prev.msg_type,
                        message_count=prev.message_count + message_count,
                    )
                else:
                    topic_info[topic] = _RosbagsTopicMeta(
                        msg_type=msg_type,
                        message_count=message_count,
                    )
            # Match rosbag API shape: tuple-like where index 1 is topic metadata map.
            return (None, topic_info)

        def _select_connections(self, topics: Optional[Iterable[str]]):
            if topics is None:
                return self._connections
            topic_set = {str(x) for x in topics}
            return [c for c in self._connections if str(c.topic) in topic_set]

        def read_messages(self, topics: Optional[Iterable[str]] = None):
            if self._reader is None:
                raise RuntimeError("Bag is not opened.")
            connections = self._select_connections(topics)
            for conn, timestamp_ns, rawdata in self._reader.messages(connections=connections):
                msg = self._reader.deserialize(rawdata, conn.msgtype)
                bag_time = _RosbagsBagTime(float(timestamp_ns) * 1e-9)
                yield str(conn.topic), msg, bag_time

    return SimpleNamespace(Bag=_RosbagsBag)


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
    if stamp is not None:
        sec_part = getattr(stamp, "sec", None)
        nsec_part = getattr(stamp, "nanosec", None)
        if nsec_part is None:
            nsec_part = getattr(stamp, "nsec", None)
        if sec_part is not None and nsec_part is not None:
            sec = float(sec_part) + float(nsec_part) * 1e-9
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


def compute_signals(
    series: JointSeries,
    smooth_win: int,
) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], np.ndarray]:
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
    return speed, acc, dtau, qdot


def compute_norm_scale(values: np.ndarray, norm_base: str) -> float:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0
    if norm_base == "median":
        return float(np.median(finite))
    if norm_base == "p90":
        return float(np.percentile(finite, 90.0))
    raise ValueError(f"Unsupported norm base: {norm_base}")


def normalize_signal(values: np.ndarray, norm_base: str, norm_eps: float) -> Tuple[np.ndarray, float]:
    scale = compute_norm_scale(values=values, norm_base=norm_base)
    denom = scale + norm_eps
    if denom <= 0.0:
        denom = norm_eps
    return values / denom, scale


def normalize_speed(speed: np.ndarray, norm_base: str, norm_eps: float) -> Tuple[np.ndarray, float]:
    return normalize_signal(values=speed, norm_base=norm_base, norm_eps=norm_eps)


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
    hole_quantile: float,
) -> List[BurstWindow]:
    if merge_gap < 0.0:
        raise ValueError("merge_gap must be >= 0")
    if hole_min_norm < 0.0:
        raise ValueError("hole_min_norm must be >= 0")
    if hole_quantile < 0.0 or hole_quantile > 1.0:
        raise ValueError("hole_quantile must be in [0, 1]")
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
            hole_vals = hole_signal_norm[gap_idx]
            hole_stat = float(np.quantile(hole_vals, hole_quantile))
            can_fill = hole_stat >= hole_min_norm
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
    hole_quantile: float,
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
            hole_quantile=hole_quantile,
        )
    raise ValueError(f"Unsupported merge_mode: {merge_mode}")


def compute_burst_score(
    acc: np.ndarray,
    dtau: Optional[np.ndarray],
    score_norm_base: str,
    score_norm_eps: float,
    dtau_weight: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    acc_norm, _acc_scale = normalize_signal(acc, norm_base=score_norm_base, norm_eps=score_norm_eps)

    dtau_norm = np.zeros_like(acc_norm, dtype=np.float64)
    if dtau is not None:
        dtau_norm_raw, _dtau_scale = normalize_signal(
            dtau.astype(np.float64, copy=False),
            norm_base=score_norm_base,
            norm_eps=score_norm_eps,
        )
        dtau_norm = np.nan_to_num(dtau_norm_raw, nan=0.0, posinf=0.0, neginf=0.0)

    score = acc_norm + float(dtau_weight) * dtau_norm
    score = np.nan_to_num(score, nan=0.0, posinf=0.0, neginf=0.0)
    return score, acc_norm, dtau_norm


def _local_peak_indices(x: np.ndarray) -> List[int]:
    n = int(x.shape[0])
    if n <= 0:
        return []

    peaks: List[int] = []
    if n == 1:
        return [0] if np.isfinite(x[0]) else []

    if np.isfinite(x[0]) and (not np.isfinite(x[1]) or x[0] > x[1]):
        peaks.append(0)

    for i in range(1, n - 1):
        if not np.isfinite(x[i]):
            continue
        left_ok = (not np.isfinite(x[i - 1])) or (x[i] >= x[i - 1])
        right_ok = (not np.isfinite(x[i + 1])) or (x[i] >= x[i + 1])
        strict = ((not np.isfinite(x[i - 1])) or (x[i] > x[i - 1])) or (
            (not np.isfinite(x[i + 1])) or (x[i] > x[i + 1])
        )
        if left_ok and right_ok and strict:
            peaks.append(i)

    if np.isfinite(x[-1]) and (not np.isfinite(x[-2]) or x[-1] > x[-2]):
        peaks.append(n - 1)
    return peaks


def _suppress_close_peaks(
    t: np.ndarray,
    peak_indices: Sequence[int],
    score: np.ndarray,
    min_gap_s: float,
) -> List[int]:
    if not peak_indices:
        return []
    if min_gap_s <= 0.0:
        return sorted({int(i) for i in peak_indices})

    ordered = sorted({int(i) for i in peak_indices}, key=lambda i: float(score[i]), reverse=True)
    keep: List[int] = []
    for idx in ordered:
        t_idx = float(t[idx])
        if all(abs(t_idx - float(t[j])) >= min_gap_s for j in keep):
            keep.append(idx)
    keep.sort()
    return keep


def _expand_left_by_relative_speed(
    speed: np.ndarray,
    peak_idx: int,
    rel_th: float,
    below_frames: int,
) -> int:
    below = 0
    for i in range(peak_idx, -1, -1):
        if speed[i] < rel_th:
            below += 1
        else:
            below = 0
        if below >= below_frames:
            return min(peak_idx, i + below_frames)
    return 0


def _expand_right_by_relative_speed(
    speed: np.ndarray,
    peak_idx: int,
    rel_th: float,
    below_frames: int,
) -> int:
    below = 0
    n = int(speed.shape[0])
    for i in range(peak_idx, n):
        if speed[i] < rel_th:
            below += 1
        else:
            below = 0
        if below >= below_frames:
            return max(peak_idx, i - below_frames)
    return n - 1


def _merge_event_windows(events: Sequence[EventWindow], merge_gap: float) -> List[EventWindow]:
    if not events:
        return []
    if merge_gap < 0.0:
        raise ValueError("merge_gap must be >= 0")

    merged = sorted(events, key=lambda e: (e.window.start_s, e.window.end_s))
    out: List[EventWindow] = [merged[0]]

    for cur in merged[1:]:
        prev = out[-1]
        gap = cur.window.start_s - prev.window.end_s
        if gap < merge_gap:
            keep = prev if prev.snap_score >= cur.snap_score else cur
            out[-1] = EventWindow(
                window=BurstWindow(
                    start_s=min(prev.window.start_s, cur.window.start_s),
                    end_s=max(prev.window.end_s, cur.window.end_s),
                ),
                snap_idx=keep.snap_idx,
                speed_peak_idx=keep.speed_peak_idx,
                snap_score=keep.snap_score,
                score_th=keep.score_th,
                speed_peak=keep.speed_peak,
                speed_rel_th=keep.speed_rel_th,
            )
        else:
            out.append(cur)
    return out


def detect_event_centered_windows(
    t: np.ndarray,
    speed: np.ndarray,
    speed_norm: np.ndarray,
    burst_score: np.ndarray,
    min_dur: float,
    pad: float,
    merge_gap: float,
    event_speed_alpha: float,
    event_below_frames: int,
    snap_peak_pct: float,
    snap_peak_min_score: float,
    snap_min_speed_norm: float,
    snap_min_gap_s: float,
    snap_peak_search_s: float,
    snap_time_mode: str,
    snap_speed_peak_mode: str,
    snap_speed_peak_window_s: float,
) -> List[EventWindow]:
    if t.shape[0] == 0:
        return []

    finite_score = burst_score[np.isfinite(burst_score)]
    if finite_score.size == 0:
        return []

    score_pct_th = float(np.percentile(finite_score, snap_peak_pct))
    score_th = max(float(snap_peak_min_score), score_pct_th)

    peak_idx = _local_peak_indices(burst_score)
    candidate_idx = [
        i
        for i in peak_idx
        if float(burst_score[i]) >= score_th and float(speed_norm[i]) >= float(snap_min_speed_norm)
    ]

    # Keep one strongest candidate to avoid empty output on low-energy episodes.
    if not candidate_idx:
        strongest_idx = int(np.argmax(burst_score))
        if float(speed_norm[strongest_idx]) >= float(snap_min_speed_norm):
            candidate_idx = [strongest_idx]

    selected_idx = _suppress_close_peaks(
        t=t,
        peak_indices=candidate_idx,
        score=burst_score,
        min_gap_s=float(snap_min_gap_s),
    )

    # Keep API compatibility for callers that still pass merge_gap.
    # Event-centered candidates are intentionally not pre-merged here to avoid
    # producing over-wide windows before top-1 ranking.
    _ = merge_gap

    events: List[EventWindow] = []
    t0 = float(t[0])
    t1 = float(t[-1])
    below_frames = int(max(1, event_below_frames))
    peak_search = float(max(0.0, snap_peak_search_s))

    for snap_idx in selected_idx:
        snap_t = float(t[snap_idx])
        if peak_search > 0.0:
            local_idx = np.where(np.abs(t - snap_t) <= peak_search)[0]
        else:
            local_idx = np.asarray([snap_idx], dtype=np.int64)
        if local_idx.size == 0:
            local_idx = np.asarray([snap_idx], dtype=np.int64)

        speed_peak_idx = _select_speed_peak_idx(
            t=t,
            speed=speed,
            local_idx=local_idx,
            mode=snap_speed_peak_mode,
            window_s=float(snap_speed_peak_window_s),
        )
        event_snap_idx = int(speed_peak_idx) if snap_time_mode == "speed_peak" else int(snap_idx)
        speed_peak = float(speed[speed_peak_idx])
        speed_rel_th = float(event_speed_alpha) * speed_peak

        left_idx = _expand_left_by_relative_speed(
            speed=speed,
            peak_idx=speed_peak_idx,
            rel_th=speed_rel_th,
            below_frames=below_frames,
        )
        right_idx = _expand_right_by_relative_speed(
            speed=speed,
            peak_idx=speed_peak_idx,
            rel_th=speed_rel_th,
            below_frames=below_frames,
        )

        start_s = float(t[left_idx])
        end_s = float(t[right_idx])
        if end_s < start_s:
            start_s, end_s = end_s, start_s
        if (end_s - start_s) < min_dur:
            continue
        if pad > 0.0:
            start_s = max(t0, start_s - pad)
            end_s = min(t1, end_s + pad)

        events.append(
            EventWindow(
                window=BurstWindow(start_s=start_s, end_s=end_s),
                snap_idx=event_snap_idx,
                speed_peak_idx=speed_peak_idx,
                snap_score=float(burst_score[snap_idx]),
                score_th=score_th,
                speed_peak=speed_peak,
                speed_rel_th=speed_rel_th,
            )
        )

    return _dedupe_event_windows_by_speed_peak(events)


def _dedupe_event_windows_by_speed_peak(events: Sequence[EventWindow]) -> List[EventWindow]:
    if not events:
        return []

    best_by_peak: Dict[int, EventWindow] = {}
    for event in events:
        key = int(event.speed_peak_idx)
        cur = best_by_peak.get(key)
        if cur is None:
            best_by_peak[key] = event
            continue

        cur_key = (
            float(cur.snap_score),
            float(cur.speed_peak),
            float(cur.window.end_s - cur.window.start_s),
        )
        new_key = (
            float(event.snap_score),
            float(event.speed_peak),
            float(event.window.end_s - event.window.start_s),
        )
        if new_key > cur_key:
            best_by_peak[key] = event

    out = list(best_by_peak.values())
    out.sort(key=lambda e: (e.window.start_s, e.window.end_s, e.snap_idx))
    return out


def _event_rank_tuple(event: EventWindow, rank_by: str) -> Tuple[float, float]:
    if rank_by == "peak_speed":
        return (float(event.speed_peak), float(event.snap_score))
    return (float(event.snap_score), float(event.speed_peak))


def select_top_event_windows(
    events: Sequence[EventWindow],
    keep_topk: int,
    rank_by: str,
) -> List[EventWindow]:
    if keep_topk < 1:
        raise ValueError("keep_topk must be >= 1")
    if rank_by not in EVENT_RANK_BY:
        raise ValueError(f"rank_by must be one of {EVENT_RANK_BY}")
    if not events:
        return []

    ranked = sorted(events, key=lambda e: _event_rank_tuple(e, rank_by), reverse=True)
    selected = ranked[:keep_topk]
    selected.sort(key=lambda e: (e.window.start_s, e.window.end_s))
    return selected


def _event_window_gap(a: BurstWindow, b: BurstWindow) -> float:
    if b.start_s >= a.end_s:
        return float(b.start_s - a.end_s)
    if a.start_s >= b.end_s:
        return float(a.start_s - b.end_s)
    return -1.0


def attach_strong_neighbor_events(
    primary: EventWindow,
    candidates: Sequence[EventWindow],
    attach_gap_s: float,
    attach_peak_ratio: float,
) -> EventWindow:
    if attach_gap_s <= 0.0:
        return primary
    if attach_peak_ratio <= 0.0:
        return primary

    base_peak = max(float(primary.speed_peak), 1e-9)
    min_neighbor_peak = float(attach_peak_ratio) * base_peak

    start = float(primary.window.start_s)
    end = float(primary.window.end_s)
    changed = True

    while changed:
        changed = False
        cur_window = BurstWindow(start_s=start, end_s=end)
        for event in candidates:
            if event is primary:
                continue
            if float(event.speed_peak) < min_neighbor_peak:
                continue
            gap = _event_window_gap(cur_window, event.window)
            if gap > attach_gap_s:
                continue
            new_start = min(start, float(event.window.start_s))
            new_end = max(end, float(event.window.end_s))
            if (new_start < start) or (new_end > end):
                start = new_start
                end = new_end
                changed = True

    return EventWindow(
        window=BurstWindow(start_s=start, end_s=end),
        snap_idx=primary.snap_idx,
        speed_peak_idx=primary.speed_peak_idx,
        snap_score=primary.snap_score,
        score_th=primary.score_th,
        speed_peak=primary.speed_peak,
        speed_rel_th=primary.speed_rel_th,
    )


def build_primary_event_fallback(
    t: np.ndarray,
    speed: np.ndarray,
    burst_score: np.ndarray,
    pad: float,
    event_speed_alpha: float,
    event_below_frames: int,
    snap_peak_search_s: float,
    snap_time_mode: str,
    snap_speed_peak_mode: str,
    snap_speed_peak_window_s: float,
) -> EventWindow:
    snap_idx = int(np.nanargmax(burst_score))
    snap_t = float(t[snap_idx])

    peak_search = float(max(0.0, snap_peak_search_s))
    if peak_search > 0.0:
        local_idx = np.where(np.abs(t - snap_t) <= peak_search)[0]
    else:
        local_idx = np.asarray([snap_idx], dtype=np.int64)
    if local_idx.size == 0:
        local_idx = np.asarray([snap_idx], dtype=np.int64)

    speed_peak_idx = _select_speed_peak_idx(
        t=t,
        speed=speed,
        local_idx=local_idx,
        mode=snap_speed_peak_mode,
        window_s=float(snap_speed_peak_window_s),
    )
    event_snap_idx = int(speed_peak_idx) if snap_time_mode == "speed_peak" else int(snap_idx)
    speed_peak = float(speed[speed_peak_idx])
    speed_rel_th = float(event_speed_alpha) * speed_peak
    below_frames = int(max(1, event_below_frames))

    left_idx = _expand_left_by_relative_speed(
        speed=speed,
        peak_idx=speed_peak_idx,
        rel_th=speed_rel_th,
        below_frames=below_frames,
    )
    right_idx = _expand_right_by_relative_speed(
        speed=speed,
        peak_idx=speed_peak_idx,
        rel_th=speed_rel_th,
        below_frames=below_frames,
    )

    start_s = float(t[left_idx])
    end_s = float(t[right_idx])
    t0 = float(t[0])
    t1 = float(t[-1])
    if pad > 0.0:
        start_s = max(t0, start_s - pad)
        end_s = min(t1, end_s + pad)

    return EventWindow(
        window=BurstWindow(start_s=start_s, end_s=end_s),
        snap_idx=event_snap_idx,
        speed_peak_idx=speed_peak_idx,
        snap_score=float(burst_score[snap_idx]),
        score_th=float("nan"),
        speed_peak=speed_peak,
        speed_rel_th=speed_rel_th,
    )


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


def filter_event_windows_by_peak(
    t: np.ndarray,
    speed: np.ndarray,
    speed_norm: np.ndarray,
    events: Sequence[EventWindow],
    peak_min_raw: float,
    peak_min_norm: float,
) -> List[EventWindow]:
    if peak_min_raw <= 0.0 and peak_min_norm <= 0.0:
        return list(events)

    filtered: List[EventWindow] = []
    for event in events:
        idx = _window_indices(t, event.window)
        peak_raw = float(np.max(speed[idx]))
        peak_norm = float(np.max(speed_norm[idx]))

        if peak_min_raw > 0.0 and peak_raw < peak_min_raw:
            continue
        if peak_min_norm > 0.0 and peak_norm < peak_min_norm:
            continue
        filtered.append(event)
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


def _indices_in_span(t: np.ndarray, start_s: float, end_s: float, fallback_idx: int) -> np.ndarray:
    idx = np.where((t >= start_s) & (t <= end_s))[0]
    if idx.size > 0:
        return idx
    return np.asarray([int(fallback_idx)], dtype=np.int64)


def _select_speed_peak_idx(
    t: np.ndarray,
    speed: np.ndarray,
    local_idx: np.ndarray,
    mode: str,
    window_s: float,
) -> int:
    if local_idx.size == 0:
        return 0
    if local_idx.size == 1:
        return int(local_idx[0])
    if mode not in SNAP_SPEED_PEAK_MODES:
        raise ValueError(f"speed peak mode must be one of {SNAP_SPEED_PEAK_MODES}")

    if mode == "max" or window_s <= 0.0:
        return int(local_idx[np.argmax(speed[local_idx])])

    half = 0.5 * float(window_s)
    best_idx = int(local_idx[0])
    best_key = (-float("inf"), -float("inf"))
    n = int(t.shape[0])
    for idx in local_idx:
        center_t = float(t[int(idx)])
        left = int(np.searchsorted(t, center_t - half, side="left"))
        right = int(np.searchsorted(t, center_t + half, side="right"))
        left = max(0, min(n - 1, left))
        right = max(left + 1, min(n, right))
        seg = speed[left:right]
        if seg.size == 0:
            score = float(speed[int(idx)])
        elif mode == "ma":
            score = float(np.mean(seg))
        else:
            score = float(np.mean(seg * seg))
        tie = float(speed[int(idx)])
        key = (score, tie)
        if key > best_key:
            best_key = key
            best_idx = int(idx)
    return best_idx


def _resolve_windup_start_from_rest(
    t: np.ndarray,
    speed: np.ndarray,
    burst_start_s: float,
    fallback_start_s: float,
    rest_speed_th: float,
    rest_frames: int,
    max_lookback_s: float,
    min_gap_s: float,
) -> float:
    if rest_frames < 1 or rest_speed_th <= 0.0:
        return float(fallback_start_s)
    if t.shape[0] == 0:
        return float(fallback_start_s)

    burst_idx = int(np.searchsorted(t, burst_start_s, side="right")) - 1
    if burst_idx < 0:
        return float(fallback_start_s)

    if max_lookback_s > 0.0:
        min_t = max(float(t[0]), float(burst_start_s) - float(max_lookback_s))
        min_idx = int(np.searchsorted(t, min_t, side="left"))
    else:
        min_idx = burst_idx

    run = 0
    run_end_idx = -1
    for i in range(burst_idx, min_idx - 1, -1):
        if float(speed[i]) < float(rest_speed_th):
            if run_end_idx < 0:
                run_end_idx = int(i)
            run += 1
            continue

        if run >= rest_frames and run_end_idx >= 0:
            rest_end_t = float(t[run_end_idx])
            if (float(burst_start_s) - rest_end_t) >= float(min_gap_s):
                return float(rest_end_t)
        run = 0
        run_end_idx = -1

    if run >= rest_frames and run_end_idx >= 0:
        rest_end_t = float(t[run_end_idx])
        if (float(burst_start_s) - rest_end_t) >= float(min_gap_s):
            return float(rest_end_t)
    return float(fallback_start_s)


def _resolve_follow_end_from_rest(
    t: np.ndarray,
    speed: np.ndarray,
    burst_end_s: float,
    fallback_end_s: float,
    rest_speed_th: float,
    rest_frames: int,
    max_lookahead_s: float,
    min_gap_s: float,
) -> float:
    if rest_frames < 1 or rest_speed_th <= 0.0:
        return float(fallback_end_s)
    if t.shape[0] == 0:
        return float(fallback_end_s)

    burst_idx = int(np.searchsorted(t, burst_end_s, side="left"))
    if burst_idx >= int(t.shape[0]):
        return float(fallback_end_s)

    if max_lookahead_s > 0.0:
        max_t = min(float(t[-1]), float(burst_end_s) + float(max_lookahead_s))
        max_idx = int(np.searchsorted(t, max_t, side="right")) - 1
    else:
        max_idx = burst_idx
    if max_idx < burst_idx:
        return float(fallback_end_s)

    run = 0
    run_start_idx = -1
    for i in range(burst_idx, max_idx + 1):
        if float(speed[i]) < float(rest_speed_th):
            if run_start_idx < 0:
                run_start_idx = int(i)
            run += 1
            continue

        if run >= rest_frames and run_start_idx >= 0:
            rest_start_t = float(t[run_start_idx])
            if (rest_start_t - float(burst_end_s)) >= float(min_gap_s):
                return float(rest_start_t)
        run = 0
        run_start_idx = -1

    if run >= rest_frames and run_start_idx >= 0:
        rest_start_t = float(t[run_start_idx])
        if (rest_start_t - float(burst_end_s)) >= float(min_gap_s):
            return float(rest_start_t)
    return float(fallback_end_s)


def _trapezoid_prefix_integral(t: np.ndarray, values: np.ndarray) -> np.ndarray:
    n = int(t.shape[0])
    if n <= 0:
        return np.asarray([], dtype=np.float64)
    out = np.zeros(n, dtype=np.float64)
    if n == 1:
        return out
    dt = np.maximum(np.diff(t), 0.0)
    seg = 0.5 * (values[:-1] + values[1:]) * dt
    out[1:] = np.cumsum(seg, dtype=np.float64)
    return out


def _energy_between_indices(prefix: np.ndarray, left_idx: int, right_idx: int) -> float:
    l = int(max(0, left_idx))
    r = int(max(0, right_idx))
    if r <= l or prefix.size == 0:
        return 0.0
    l = min(l, int(prefix.size - 1))
    r = min(r, int(prefix.size - 1))
    if r <= l:
        return 0.0
    return float(max(0.0, prefix[r] - prefix[l]))


def _resolve_energy_burst_span(
    t: np.ndarray,
    speed: np.ndarray,
    snap_time_s: float,
    search_left_s: float,
    search_right_s: float,
    energy_rho: float,
) -> Tuple[float, float, float, float, float, float]:
    if t.shape[0] == 0:
        snap_t = float(snap_time_s)
        return snap_t, snap_t, snap_t, snap_t, 0.0, 0.0

    t0 = float(t[0])
    t1 = float(t[-1])
    snap_t = float(np.clip(snap_time_s, t0, t1))
    search_start = max(t0, snap_t - max(0.0, float(search_left_s)))
    search_end = min(t1, snap_t + max(0.0, float(search_right_s)))
    if search_end < search_start:
        search_start = snap_t
        search_end = snap_t

    idx = np.where((t >= search_start) & (t <= search_end))[0]
    if idx.size == 0:
        nearest = int(np.argmin(np.abs(t - snap_t)))
        anchor = float(t[nearest])
        return anchor, anchor, anchor, anchor, 0.0, 0.0

    tr = t[idx]
    er = np.square(speed[idx]).astype(np.float64)
    c = _trapezoid_prefix_integral(tr, er)
    total_energy = float(c[-1]) if c.size > 0 else 0.0
    if (not np.isfinite(total_energy)) or total_energy <= 1e-12:
        snap_local = int(np.argmin(np.abs(tr - snap_t)))
        anchor = float(tr[snap_local])
        return anchor, anchor, float(tr[0]), float(tr[-1]), 0.0, 0.0

    rho = float(np.clip(energy_rho, 0.0, 1.0))
    target = rho * total_energy
    snap_local = int(np.argmin(np.abs(tr - snap_t)))

    best: Optional[Tuple[float, float, float, int, int]] = None
    for left in range(0, snap_local + 1):
        needed = float(c[left]) + target
        right = int(np.searchsorted(c, needed, side="left"))
        if right < snap_local:
            right = snap_local
        if right >= int(tr.shape[0]):
            continue
        covered = _energy_between_indices(c, left, right)
        duration = float(max(0.0, tr[right] - tr[left]))
        center_dist = abs(0.5 * float(tr[left] + tr[right]) - snap_t)
        candidate = (duration, -covered, center_dist, left, right)
        if best is None or candidate < best:
            best = candidate

    if best is None:
        anchor = float(tr[snap_local])
        return anchor, anchor, float(tr[0]), float(tr[-1]), total_energy, 0.0

    left = int(best[3])
    right = int(best[4])
    burst_start = float(tr[left])
    burst_end = float(tr[right])
    burst_energy = _energy_between_indices(c, left, right)
    return burst_start, burst_end, float(tr[0]), float(tr[-1]), total_energy, burst_energy


def _intent_direction_unit(qdot_window: np.ndarray) -> List[float]:
    if qdot_window.size == 0:
        return []
    vec = np.sum(qdot_window, axis=0)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12:
        return [0.0 for _ in range(int(qdot_window.shape[1]))]
    return [float(x) for x in (vec / norm)]


def _dct_shape(speed_window: np.ndarray, k: int) -> List[float]:
    if k <= 0:
        return []
    n = int(speed_window.shape[0])
    if n <= 0:
        return []
    centered = speed_window - float(np.mean(speed_window))
    nn = np.arange(n, dtype=np.float64)
    coeffs: List[float] = []
    for mode in range(1, int(k) + 1):
        basis = np.cos(np.pi * (nn + 0.5) * float(mode) / float(n))
        coeff = (2.0 / float(n)) * float(np.sum(centered * basis))
        coeffs.append(float(coeff))
    return coeffs


def _to_json_array(values: Sequence[float]) -> str:
    return json.dumps([float(v) for v in values], ensure_ascii=True, separators=(",", ":"))


def build_bursts_table(
    t: np.ndarray,
    speed: np.ndarray,
    acc: np.ndarray,
    dtau: Optional[np.ndarray],
    qdot: np.ndarray,
    burst_score: np.ndarray,
    windows: Sequence[BurstWindow],
    threshold_info: ThresholdInfo,
    merge_gap_s: float,
    merge_mode: str,
    hole_min_norm: float,
    hole_quantile: float,
    peak_min_norm: float,
    peak_min_raw: float,
    peak_filter_norm_base: str,
    peak_filter_norm_scale: float,
    peak_filter_norm_eps: float,
    detection_mode: str,
    score_norm_base: str,
    score_norm_eps: float,
    dtau_weight: float,
    event_speed_alpha: float,
    event_below_frames: int,
    snap_peak_pct: float,
    snap_peak_min_score: float,
    snap_min_speed_norm: float,
    snap_min_gap_s: float,
    snap_peak_search_s: float,
    snap_time_mode: str,
    snap_speed_peak_mode: str,
    snap_speed_peak_window_s: float,
    burst_energy_rho: float,
    burst_energy_left_s: float,
    burst_energy_right_s: float,
    shape_dct_k: int,
    event_keep_topk: int,
    event_rank_by: str,
    event_force_primary: bool,
    event_attach_gap_s: float,
    event_attach_peak_ratio: float,
    windup_rest_frames: int,
    windup_rest_speed_scale: float,
    windup_rest_lookback_s: float,
    windup_rest_min_gap_s: float,
    follow_rest_frames: int = 8,
    follow_rest_speed_scale: float = 1.3,
    follow_rest_lookahead_s: float = 2.5,
    follow_rest_min_gap_s: float = 0.15,
    event_windows: Optional[Sequence[EventWindow]] = None,
    t_abs: Optional[np.ndarray] = None,
) -> pd.DataFrame:
    if t_abs is None:
        t_abs = np.full_like(t, np.nan, dtype=np.float64)
    t0_abs = float(t_abs[0]) if t_abs.size > 0 else float("nan")
    speed_energy = np.square(speed).astype(np.float64)
    speed_energy_prefix = _trapezoid_prefix_integral(t, speed_energy)

    rows = []
    for burst_id, window in enumerate(windows):
        idx = _window_indices(t, window)

        i_speed = int(idx[np.argmax(speed[idx])])
        i_acc = int(idx[np.argmax(acc[idx])])

        event_meta: Optional[EventWindow] = None
        if event_windows is not None and burst_id < len(event_windows):
            event_meta = event_windows[burst_id]

        if event_meta is not None:
            i_snap = int(event_meta.snap_idx)
            snap_score = float(event_meta.snap_score)
            snap_score_th = float(event_meta.score_th)
            snap_speed_local_peak = float(event_meta.speed_peak)
            snap_speed_rel_th = float(event_meta.speed_rel_th)
        else:
            i_snap = int(idx[np.argmax(burst_score[idx])])
            snap_score = float(burst_score[i_snap])
            snap_score_th = float("nan")
            snap_speed_local_peak = float(speed[i_speed])
            snap_speed_rel_th = float("nan")

        snap_time_s = float(t[i_snap])
        snap_time_abs_s = float(t_abs[i_snap])

        burst_start, burst_end, burst_energy_search_start_s, burst_energy_search_end_s, energy_total_search, energy_burst = _resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=snap_time_s,
            search_left_s=float(burst_energy_left_s),
            search_right_s=float(burst_energy_right_s),
            energy_rho=float(burst_energy_rho),
        )

        windup_start = max(float(t[0]), burst_start - max(0.0, float(windup_rest_lookback_s)))
        if windup_rest_frames > 0 and np.isfinite(threshold_info.off_th_speed_equiv):
            rest_th = float(windup_rest_speed_scale) * float(threshold_info.off_th_speed_equiv)
            if rest_th > 0.0:
                windup_start = _resolve_windup_start_from_rest(
                    t=t,
                    speed=speed,
                    burst_start_s=burst_start,
                    fallback_start_s=windup_start,
                    rest_speed_th=rest_th,
                    rest_frames=int(windup_rest_frames),
                    max_lookback_s=float(windup_rest_lookback_s),
                    min_gap_s=float(windup_rest_min_gap_s),
                )

        follow_end = min(float(t[-1]), burst_end + max(0.0, float(follow_rest_lookahead_s)))
        if follow_rest_frames > 0 and np.isfinite(threshold_info.off_th_speed_equiv):
            rest_th = float(follow_rest_speed_scale) * float(threshold_info.off_th_speed_equiv)
            if rest_th > 0.0:
                follow_end = _resolve_follow_end_from_rest(
                    t=t,
                    speed=speed,
                    burst_end_s=burst_end,
                    fallback_end_s=follow_end,
                    rest_speed_th=rest_th,
                    rest_frames=int(follow_rest_frames),
                    max_lookahead_s=float(follow_rest_lookahead_s),
                    min_gap_s=float(follow_rest_min_gap_s),
                )

        windup_end = float(burst_start)
        follow_start = float(burst_end)
        if windup_start > windup_end:
            windup_start = windup_end
        if follow_end < follow_start:
            follow_end = follow_start
        burst_start = float(windup_end)
        burst_end = float(follow_start)
        burst_duration_s = float(max(0.0, burst_end - burst_start))
        windup_duration_s = float(max(0.0, windup_end - windup_start))
        follow_duration_s = float(max(0.0, follow_end - follow_start))
        idx_burst = _indices_in_span(t=t, start_s=burst_start, end_s=burst_end, fallback_idx=i_snap)

        z_amp_speed = float(np.max(speed[idx_burst]))
        z_amp_acc = float(np.max(acc[idx_burst]))
        z_dir = _intent_direction_unit(qdot[idx_burst])
        z_shape = _dct_shape(speed[idx_burst], k=shape_dct_k)

        idx_windup = _indices_in_span(
            t=t,
            start_s=windup_start,
            end_s=windup_end,
            fallback_idx=int(idx_burst[0]),
        )
        idx_follow = _indices_in_span(
            t=t,
            start_s=follow_start,
            end_s=follow_end,
            fallback_idx=int(idx_burst[-1]),
        )
        energy_windup = _energy_between_indices(
            prefix=speed_energy_prefix,
            left_idx=int(idx_windup[0]),
            right_idx=int(idx_windup[-1]),
        )
        energy_follow = _energy_between_indices(
            prefix=speed_energy_prefix,
            left_idx=int(idx_follow[0]),
            right_idx=int(idx_follow[-1]),
        )
        energy_burst_share = float(energy_burst / (energy_total_search + 1e-9))
        context_energy_ratio = float((energy_windup + energy_follow) / (energy_windup + energy_burst + energy_follow + 1e-9))
        burst_density = float(energy_burst / (burst_duration_s + 1e-9))
        context_density = float((energy_windup + energy_follow) / (windup_duration_s + follow_duration_s + 1e-9))
        energy_density_contrast = float(burst_density / (context_density + 1e-9))

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
                "hole_quantile": float(hole_quantile),
                "peak_min_norm": float(peak_min_norm),
                "peak_min_raw": float(peak_min_raw),
                "peak_filter_norm_base": peak_filter_norm_base,
                "peak_filter_norm_scale": float(peak_filter_norm_scale),
                "peak_filter_norm_eps": float(peak_filter_norm_eps),
                "detection_mode": detection_mode,
                "snap_time_s": snap_time_s,
                "snap_time_abs_s": snap_time_abs_s,
                "snap_score": snap_score,
                "snap_score_th": snap_score_th,
                "snap_speed_local_peak": snap_speed_local_peak,
                "snap_speed_rel_th": snap_speed_rel_th,
                "windup_start_s": windup_start,
                "windup_end_s": windup_end,
                "burst_start_s": burst_start,
                "burst_end_s": burst_end,
                "follow_start_s": follow_start,
                "follow_end_s": follow_end,
                "windup_duration_s": windup_duration_s,
                "burst_duration_s": burst_duration_s,
                "follow_duration_s": follow_duration_s,
                "burst_energy_search_start_s": float(burst_energy_search_start_s),
                "burst_energy_search_end_s": float(burst_energy_search_end_s),
                "energy_total_search": float(energy_total_search),
                "energy_burst": float(energy_burst),
                "energy_windup": float(energy_windup),
                "energy_follow": float(energy_follow),
                "energy_burst_share": energy_burst_share,
                "context_energy_ratio": context_energy_ratio,
                "energy_density_contrast": energy_density_contrast,
                "z_amp_speed": z_amp_speed,
                "z_amp_acc": z_amp_acc,
                "z_dur": burst_duration_s,
                "z_dir_unit": _to_json_array(z_dir),
                "z_shape_dct": _to_json_array(z_shape),
                "score_norm_base": score_norm_base,
                "score_norm_eps": float(score_norm_eps),
                "dtau_weight": float(dtau_weight),
                "event_speed_alpha": float(event_speed_alpha),
                "event_below_frames": int(event_below_frames),
                "snap_peak_pct": float(snap_peak_pct),
                "snap_peak_min_score": float(snap_peak_min_score),
                "snap_min_speed_norm": float(snap_min_speed_norm),
                "snap_min_gap_s": float(snap_min_gap_s),
                "snap_peak_search_s": float(snap_peak_search_s),
                "snap_time_mode": snap_time_mode,
                "snap_speed_peak_mode": snap_speed_peak_mode,
                "snap_speed_peak_window_s": float(snap_speed_peak_window_s),
                "burst_energy_rho": float(burst_energy_rho),
                "burst_energy_left_s": float(burst_energy_left_s),
                "burst_energy_right_s": float(burst_energy_right_s),
                "shape_dct_k": int(shape_dct_k),
                "event_keep_topk": int(event_keep_topk),
                "event_rank_by": event_rank_by,
                "event_force_primary": bool(event_force_primary),
                "event_attach_gap_s": float(event_attach_gap_s),
                "event_attach_peak_ratio": float(event_attach_peak_ratio),
                "windup_rest_frames": int(windup_rest_frames),
                "windup_rest_speed_scale": float(windup_rest_speed_scale),
                "windup_rest_lookback_s": float(windup_rest_lookback_s),
                "windup_rest_min_gap_s": float(windup_rest_min_gap_s),
                "follow_rest_frames": int(follow_rest_frames),
                "follow_rest_speed_scale": float(follow_rest_speed_scale),
                "follow_rest_lookahead_s": float(follow_rest_lookahead_s),
                "follow_rest_min_gap_s": float(follow_rest_min_gap_s),
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
    detection_mode: str,
    follow_spans: Optional[Sequence[Tuple[float, float]]] = None,
    windup_spans: Optional[Sequence[Tuple[float, float]]] = None,
    burst_spans: Optional[Sequence[Tuple[float, float]]] = None,
    snap_times: Optional[Sequence[float]] = None,
    plot_debug_window: bool = False,
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

    if plot_debug_window:
        for i, window in enumerate(windows):
            ax1.axvspan(
                float(window.start_s),
                float(window.end_s),
                facecolor="none",
                edgecolor="0.35",
                linewidth=0.8,
                linestyle="--",
                hatch="///",
                alpha=0.22,
                label="event window (debug)" if i == 0 else None,
            )
    if windup_spans:
        for i, (windup_start, windup_end) in enumerate(windup_spans):
            ax1.axvspan(
                float(windup_start),
                float(windup_end),
                color="tab:cyan",
                alpha=0.18,
                label="windup phase" if i == 0 else None,
            )
    if burst_spans:
        for i, (burst_start, burst_end) in enumerate(burst_spans):
            ax1.axvspan(
                float(burst_start),
                float(burst_end),
                color="tab:red",
                alpha=0.16,
                label="burst phase" if i == 0 else None,
            )
    if follow_spans:
        for i, (follow_start, follow_end) in enumerate(follow_spans):
            ax1.axvspan(
                float(follow_start),
                float(follow_end),
                color="tab:orange",
                alpha=0.18,
                label="follow phase" if i == 0 else None,
            )
    if snap_times:
        for snap_t in snap_times:
            ax1.axvline(float(snap_t), color="tab:red", linestyle="-", linewidth=0.9, alpha=0.7)

    if speed_ymin is not None or speed_ymax is not None:
        cur_ymin, cur_ymax = ax1.get_ylim()
        y_min = speed_ymin if speed_ymin is not None else float(cur_ymin)
        y_max = speed_ymax if speed_ymax is not None else float(cur_ymax)
        if y_max <= y_min:
            raise ValueError(f"Invalid speed axis limits: ymin={y_min} ymax={y_max}")
        ax1.set_ylim(y_min, y_max)

    ax1.set_xlabel("time from episode start [s]")
    ax1.set_ylabel("speed")
    ax1.set_title(
        f"{episode_name} | Burst Detection ({threshold_mode}, {threshold_signal_space}, {detection_mode})"
    )
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
    detection_mode: str,
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
    hole_quantile: float,
    peak_min_norm: float,
    peak_min_raw: float,
    score_norm_base: str,
    score_norm_eps: float,
    dtau_weight: float,
    event_speed_alpha: float,
    event_below_frames: int,
    snap_peak_pct: float,
    snap_peak_min_score: float,
    snap_min_speed_norm: float,
    snap_min_gap_s: float,
    snap_peak_search_s: float,
    snap_time_mode: str,
    snap_speed_peak_mode: str,
    snap_speed_peak_window_s: float,
    burst_energy_rho: float,
    burst_energy_left_s: float,
    burst_energy_right_s: float,
    shape_dct_k: int,
    event_keep_topk: int,
    event_rank_by: str,
    event_force_primary: bool,
    event_attach_gap_s: float,
    event_attach_peak_ratio: float,
    windup_rest_frames: int,
    windup_rest_speed_scale: float,
    windup_rest_lookback_s: float,
    windup_rest_min_gap_s: float,
    follow_rest_frames: int,
    follow_rest_speed_scale: float,
    follow_rest_lookahead_s: float,
    follow_rest_min_gap_s: float,
) -> None:
    if threshold_mode not in THRESHOLD_MODES:
        raise ValueError(f"--threshold-mode must be one of {THRESHOLD_MODES}")
    if detection_mode not in DETECTION_MODES:
        raise ValueError(f"--detection-mode must be one of {DETECTION_MODES}")
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
    if hole_quantile < 0.0 or hole_quantile > 1.0:
        raise ValueError("--hole-quantile must be in [0, 1]")
    if peak_min_norm < 0.0:
        raise ValueError("--peak-min-norm must be >= 0")
    if peak_min_raw < 0.0:
        raise ValueError("--peak-min-raw must be >= 0")
    if score_norm_base not in NORM_BASES:
        raise ValueError(f"--score-norm-base must be one of {NORM_BASES}")
    if score_norm_eps <= 0.0:
        raise ValueError("--score-norm-eps must be > 0")
    if dtau_weight < 0.0:
        raise ValueError("--dtau-weight must be >= 0")
    if event_speed_alpha <= 0.0:
        raise ValueError("--event-speed-alpha must be > 0")
    if event_below_frames < 1:
        raise ValueError("--event-below-frames must be >= 1")
    if snap_peak_pct < 0.0 or snap_peak_pct > 100.0:
        raise ValueError("--snap-peak-pct must be in [0, 100]")
    if snap_peak_min_score < 0.0:
        raise ValueError("--snap-peak-min-score must be >= 0")
    if snap_min_speed_norm < 0.0:
        raise ValueError("--snap-min-speed-norm must be >= 0")
    if snap_min_gap_s < 0.0:
        raise ValueError("--snap-min-gap must be >= 0")
    if snap_peak_search_s < 0.0:
        raise ValueError("--snap-peak-search must be >= 0")
    if snap_time_mode not in SNAP_TIME_MODES:
        raise ValueError(f"--snap-time-mode must be one of {SNAP_TIME_MODES}")
    if snap_speed_peak_mode not in SNAP_SPEED_PEAK_MODES:
        raise ValueError(f"--snap-speed-peak-mode must be one of {SNAP_SPEED_PEAK_MODES}")
    if snap_speed_peak_window_s < 0.0:
        raise ValueError("--snap-speed-peak-window must be >= 0")
    if burst_energy_rho < 0.0 or burst_energy_rho > 1.0:
        raise ValueError("--burst-energy-rho must be in [0, 1]")
    if burst_energy_left_s < 0.0:
        raise ValueError("--burst-energy-left must be >= 0")
    if burst_energy_right_s < 0.0:
        raise ValueError("--burst-energy-right must be >= 0")
    if shape_dct_k < 0:
        raise ValueError("--shape-dct-k must be >= 0")
    if event_keep_topk < 1:
        raise ValueError("--event-keep-topk must be >= 1")
    if event_rank_by not in EVENT_RANK_BY:
        raise ValueError(f"--event-rank-by must be one of {EVENT_RANK_BY}")
    if not isinstance(event_force_primary, bool):
        raise ValueError("--event-force-primary must be true/false")
    if event_attach_gap_s < 0.0:
        raise ValueError("--event-attach-gap must be >= 0")
    if event_attach_peak_ratio < 0.0:
        raise ValueError("--event-attach-peak-ratio must be >= 0")
    if windup_rest_frames < 0:
        raise ValueError("--windup-rest-frames must be >= 0")
    if windup_rest_speed_scale < 0.0:
        raise ValueError("--windup-rest-speed-scale must be >= 0")
    if windup_rest_lookback_s < 0.0:
        raise ValueError("--windup-rest-lookback must be >= 0")
    if windup_rest_min_gap_s < 0.0:
        raise ValueError("--windup-rest-min-gap must be >= 0")
    if follow_rest_frames < 0:
        raise ValueError("--follow-rest-frames must be >= 0")
    if follow_rest_speed_scale < 0.0:
        raise ValueError("--follow-rest-speed-scale must be >= 0")
    if follow_rest_lookahead_s < 0.0:
        raise ValueError("--follow-rest-lookahead must be >= 0")
    if follow_rest_min_gap_s < 0.0:
        raise ValueError("--follow-rest-min-gap must be >= 0")

    if detection_mode == "event_centered" and merge_mode != "gap":
        raise ValueError("--merge-mode must be 'gap' when --detection-mode=event_centered")

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
    speed, _acc, _dtau, _qdot = compute_signals(series=series, smooth_win=smooth_win)
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
    detection_mode: str = "event_centered",
    norm_base: str = "p90",
    norm_eps: float = 1e-6,
    on_abs: float = 2.0,
    off_abs: float = 1.3,
    global_stats: Optional[str] = None,
    merge_gap: float = 0.12,
    merge_mode: str = "gap",
    hole_min_norm: float = 0.5,
    hole_quantile: float = 0.0,
    peak_min_norm: float = 0.0,
    peak_min_raw: float = 0.0,
    score_norm_base: str = "p90",
    score_norm_eps: float = 1e-6,
    dtau_weight: float = 0.5,
    event_speed_alpha: float = 0.4,
    event_below_frames: int = 3,
    snap_peak_pct: float = 85.0,
    snap_peak_min_score: float = 0.0,
    snap_min_speed_norm: float = 0.6,
    snap_min_gap_s: float = 0.15,
    snap_peak_search_s: float = 0.2,
    snap_time_mode: str = "score_peak",
    snap_speed_peak_mode: str = "max",
    snap_speed_peak_window_s: float = 0.2,
    burst_energy_rho: float = 0.90,
    burst_energy_left_s: float = 1.5,
    burst_energy_right_s: float = 1.5,
    shape_dct_k: int = 4,
    event_keep_topk: int = 1,
    event_rank_by: str = "peak_speed",
    event_force_primary: bool = True,
    event_attach_gap_s: float = 0.18,
    event_attach_peak_ratio: float = 0.62,
    windup_rest_frames: int = 0,
    windup_rest_speed_scale: float = 1.0,
    windup_rest_lookback_s: float = 2.5,
    windup_rest_min_gap_s: float = 0.0,
    follow_rest_frames: int = 8,
    follow_rest_speed_scale: float = 1.3,
    follow_rest_lookahead_s: float = 2.5,
    follow_rest_min_gap_s: float = 0.15,
    plot_debug_window: bool = False,
    print_summary: bool = True,
) -> EpisodeResult:
    validate_knobs(smooth_win=smooth_win, on_pct=on_pct, off_pct=off_pct, min_dur=min_dur, pad=pad)
    validate_speed_axis(speed_ymin=speed_ymin, speed_ymax=speed_ymax)
    validate_threshold_options(
        threshold_mode=threshold_mode,
        detection_mode=detection_mode,
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
        hole_quantile=hole_quantile,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
        score_norm_base=score_norm_base,
        score_norm_eps=score_norm_eps,
        dtau_weight=dtau_weight,
        event_speed_alpha=event_speed_alpha,
        event_below_frames=event_below_frames,
        snap_peak_pct=snap_peak_pct,
        snap_peak_min_score=snap_peak_min_score,
        snap_min_speed_norm=snap_min_speed_norm,
        snap_min_gap_s=snap_min_gap_s,
        snap_peak_search_s=snap_peak_search_s,
        snap_time_mode=snap_time_mode,
        snap_speed_peak_mode=snap_speed_peak_mode,
        snap_speed_peak_window_s=snap_speed_peak_window_s,
        burst_energy_rho=burst_energy_rho,
        burst_energy_left_s=burst_energy_left_s,
        burst_energy_right_s=burst_energy_right_s,
        shape_dct_k=shape_dct_k,
        event_keep_topk=event_keep_topk,
        event_rank_by=event_rank_by,
        event_force_primary=event_force_primary,
        event_attach_gap_s=event_attach_gap_s,
        event_attach_peak_ratio=event_attach_peak_ratio,
        windup_rest_frames=windup_rest_frames,
        windup_rest_speed_scale=windup_rest_speed_scale,
        windup_rest_lookback_s=windup_rest_lookback_s,
        windup_rest_min_gap_s=windup_rest_min_gap_s,
        follow_rest_frames=follow_rest_frames,
        follow_rest_speed_scale=follow_rest_speed_scale,
        follow_rest_lookahead_s=follow_rest_lookahead_s,
        follow_rest_min_gap_s=follow_rest_min_gap_s,
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

    speed, acc, dtau, qdot = compute_signals(series=series, smooth_win=smooth_win)
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
    burst_score, _acc_norm, _dtau_norm = compute_burst_score(
        acc=acc,
        dtau=dtau,
        score_norm_base=score_norm_base,
        score_norm_eps=score_norm_eps,
        dtau_weight=dtau_weight,
    )

    event_windows: Optional[List[EventWindow]] = None
    if detection_mode == "event_centered":
        event_windows = detect_event_centered_windows(
            t=t_rel,
            speed=speed,
            speed_norm=speed_norm_for_peak,
            burst_score=burst_score,
            min_dur=min_dur,
            pad=pad,
            merge_gap=merge_gap,
            event_speed_alpha=event_speed_alpha,
            event_below_frames=event_below_frames,
            snap_peak_pct=snap_peak_pct,
            snap_peak_min_score=snap_peak_min_score,
            snap_min_speed_norm=snap_min_speed_norm,
            snap_min_gap_s=snap_min_gap_s,
            snap_peak_search_s=snap_peak_search_s,
            snap_time_mode=snap_time_mode,
            snap_speed_peak_mode=snap_speed_peak_mode,
            snap_speed_peak_window_s=snap_speed_peak_window_s,
        )
        event_windows = filter_event_windows_by_peak(
            t=t_rel,
            speed=speed,
            speed_norm=speed_norm_for_peak,
            events=event_windows,
            peak_min_raw=peak_min_raw,
            peak_min_norm=peak_min_norm,
        )
        event_windows_all = list(event_windows)
        if event_windows_all:
            event_windows = select_top_event_windows(
                events=event_windows_all,
                keep_topk=event_keep_topk,
                rank_by=event_rank_by,
            )
            if event_windows:
                primary = attach_strong_neighbor_events(
                    primary=event_windows[0],
                    candidates=event_windows_all,
                    attach_gap_s=event_attach_gap_s,
                    attach_peak_ratio=event_attach_peak_ratio,
                )
                event_windows = [primary]
        if (not event_windows) and event_force_primary:
            fallback = build_primary_event_fallback(
                t=t_rel,
                speed=speed,
                burst_score=burst_score,
                pad=pad,
                event_speed_alpha=event_speed_alpha,
                event_below_frames=event_below_frames,
                snap_peak_search_s=snap_peak_search_s,
                snap_time_mode=snap_time_mode,
                snap_speed_peak_mode=snap_speed_peak_mode,
                snap_speed_peak_window_s=snap_speed_peak_window_s,
            )
            event_windows = [fallback]
        windows: List[BurstWindow] = [event.window for event in event_windows]
    else:
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
            hole_quantile=hole_quantile,
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
        qdot=qdot,
        burst_score=burst_score,
        windows=windows,
        threshold_info=threshold_info,
        merge_gap_s=merge_gap,
        merge_mode=merge_mode,
        hole_min_norm=hole_min_norm,
        hole_quantile=hole_quantile,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
        peak_filter_norm_base=peak_filter_norm_base,
        peak_filter_norm_scale=peak_filter_norm_scale,
        peak_filter_norm_eps=peak_filter_norm_eps,
        detection_mode=detection_mode,
        score_norm_base=score_norm_base,
        score_norm_eps=score_norm_eps,
        dtau_weight=dtau_weight,
        event_speed_alpha=event_speed_alpha,
        event_below_frames=event_below_frames,
        snap_peak_pct=snap_peak_pct,
        snap_peak_min_score=snap_peak_min_score,
        snap_min_speed_norm=snap_min_speed_norm,
        snap_min_gap_s=snap_min_gap_s,
        snap_peak_search_s=snap_peak_search_s,
        snap_time_mode=snap_time_mode,
        snap_speed_peak_mode=snap_speed_peak_mode,
        snap_speed_peak_window_s=snap_speed_peak_window_s,
        burst_energy_rho=burst_energy_rho,
        burst_energy_left_s=burst_energy_left_s,
        burst_energy_right_s=burst_energy_right_s,
        shape_dct_k=shape_dct_k,
        event_keep_topk=event_keep_topk,
        event_rank_by=event_rank_by,
        event_force_primary=event_force_primary,
        event_attach_gap_s=event_attach_gap_s,
        event_attach_peak_ratio=event_attach_peak_ratio,
        windup_rest_frames=windup_rest_frames,
        windup_rest_speed_scale=windup_rest_speed_scale,
        windup_rest_lookback_s=windup_rest_lookback_s,
        windup_rest_min_gap_s=windup_rest_min_gap_s,
        follow_rest_frames=follow_rest_frames,
        follow_rest_speed_scale=follow_rest_speed_scale,
        follow_rest_lookahead_s=follow_rest_lookahead_s,
        follow_rest_min_gap_s=follow_rest_min_gap_s,
        event_windows=event_windows,
        t_abs=series.t,
    )
    bursts_df.to_csv(resolved_csv_out, index=False)

    snap_times: Optional[List[float]] = None
    windup_spans: Optional[List[Tuple[float, float]]] = None
    burst_spans: Optional[List[Tuple[float, float]]] = None
    follow_spans: Optional[List[Tuple[float, float]]] = None
    if not bursts_df.empty:
        snap_values = bursts_df["snap_time_s"].to_numpy(dtype=np.float64)
        snap_times = [float(v) for v in snap_values if np.isfinite(v)]

        windup_spans = []
        for windup_start_s, windup_end_s in zip(
            bursts_df["windup_start_s"].to_numpy(dtype=np.float64),
            bursts_df["windup_end_s"].to_numpy(dtype=np.float64),
        ):
            if np.isfinite(windup_start_s) and np.isfinite(windup_end_s):
                start_s = float(windup_start_s)
                end_s = float(windup_end_s)
                if end_s >= start_s:
                    windup_spans.append((start_s, end_s))
        if not windup_spans:
            windup_spans = None

        burst_spans = []
        for burst_start_s, burst_end_s in zip(
            bursts_df["burst_start_s"].to_numpy(dtype=np.float64),
            bursts_df["burst_end_s"].to_numpy(dtype=np.float64),
        ):
            if np.isfinite(burst_start_s) and np.isfinite(burst_end_s):
                start_s = float(burst_start_s)
                end_s = float(burst_end_s)
                if end_s >= start_s:
                    burst_spans.append((start_s, end_s))
        if not burst_spans:
            burst_spans = None

        follow_spans = []
        for follow_start_s, follow_end_s in zip(
            bursts_df["follow_start_s"].to_numpy(dtype=np.float64),
            bursts_df["follow_end_s"].to_numpy(dtype=np.float64),
        ):
            if np.isfinite(follow_start_s) and np.isfinite(follow_end_s):
                start_s = float(follow_start_s)
                end_s = float(follow_end_s)
                if end_s >= start_s:
                    follow_spans.append((start_s, end_s))
        if not follow_spans:
            follow_spans = None

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
        detection_mode=detection_mode,
        follow_spans=follow_spans,
        windup_spans=windup_spans,
        burst_spans=burst_spans,
        snap_times=snap_times,
        plot_debug_window=plot_debug_window,
        speed_ymin=speed_ymin,
        speed_ymax=speed_ymax,
    )

    if bursts_df.empty:
        num_bursts = 0
        total_burst_time = 0.0
        mean_peak_speed = 0.0
        max_peak_speed = 0.0
        mean_duration = 0.0
        mean_windup_duration_s = 0.0
        mean_burst_duration_s = 0.0
        mean_follow_duration_s = 0.0
        mean_burst_energy_share = 0.0
        median_burst_energy_share = 0.0
        mean_context_energy_ratio = 0.0
        median_context_energy_ratio = 0.0
        mean_energy_density_contrast = 0.0
        median_energy_density_contrast = 0.0
        mean_snap_score = 0.0
    else:
        num_bursts = int(bursts_df.shape[0])
        total_burst_time = float(bursts_df["duration_s"].sum())
        mean_peak_speed = float(bursts_df["peak_speed"].mean())
        max_peak_speed = float(bursts_df["peak_speed"].max())
        mean_duration = float(bursts_df["duration_s"].mean())
        mean_windup_duration_s = float(bursts_df["windup_duration_s"].mean())
        mean_burst_duration_s = float(bursts_df["burst_duration_s"].mean())
        mean_follow_duration_s = float(bursts_df["follow_duration_s"].mean())
        mean_burst_energy_share = float(bursts_df["energy_burst_share"].mean())
        median_burst_energy_share = float(bursts_df["energy_burst_share"].median())
        mean_context_energy_ratio = float(bursts_df["context_energy_ratio"].mean())
        median_context_energy_ratio = float(bursts_df["context_energy_ratio"].median())
        mean_energy_density_contrast = float(bursts_df["energy_density_contrast"].mean())
        median_energy_density_contrast = float(bursts_df["energy_density_contrast"].median())
        mean_snap_score = float(bursts_df["snap_score"].mean())

    if print_summary:
        print(
            "mode={mode} detector={detector} signal={signal} norm_base={norm_base} "
            "on_th={on:.6f} off_th={off:.6f} "
            "on_th_speed={on_spd:.6f} off_th_speed={off_spd:.6f} "
            "merge_mode={merge_mode} merge_gap={merge_gap:.3f}s hole_min_norm={hole_min_norm:.3f} "
            "hole_quantile={hole_quantile:.3f} "
            "peak_min_norm={peak_norm:.3f} peak_min_raw={peak_raw:.3f} "
            "score_norm_base={score_norm_base} dtau_weight={dtau_weight:.3f} "
            "snap_peak_pct={snap_peak_pct:.1f} snap_min_speed_norm={snap_min_speed_norm:.3f} "
            "snap_time_mode={snap_time_mode} "
            "snap_speed_peak_mode={snap_speed_peak_mode} "
            "mean_windup={mean_windup:.6f}s mean_burst={mean_burst:.6f}s mean_follow={mean_follow:.6f}s "
            "mean_burst_share={mean_burst_share:.6f} median_burst_share={median_burst_share:.6f} "
            "mean_context_ratio={mean_context_ratio:.6f} median_context_ratio={median_context_ratio:.6f} "
            "mean_energy_contrast={mean_energy_contrast:.6f} median_energy_contrast={median_energy_contrast:.6f} "
            "event_keep_topk={event_keep_topk} event_rank_by={event_rank_by} "
            "bursts={num} total_burst_time={total:.6f}s mean_peak_speed={mean_peak:.6f} "
            "max_peak_speed={max_peak:.6f} mean_snap_score={mean_snap_score:.6f}".format(
                mode=threshold_info.mode,
                detector=detection_mode,
                signal=threshold_info.signal_space,
                norm_base=threshold_info.norm_base,
                on=threshold_info.on_th_signal,
                off=threshold_info.off_th_signal,
                on_spd=threshold_info.on_th_speed_equiv,
                off_spd=threshold_info.off_th_speed_equiv,
                merge_mode=merge_mode,
                merge_gap=merge_gap,
                hole_min_norm=hole_min_norm,
                hole_quantile=hole_quantile,
                peak_norm=peak_min_norm,
                peak_raw=peak_min_raw,
                score_norm_base=score_norm_base,
                dtau_weight=dtau_weight,
                snap_peak_pct=snap_peak_pct,
                snap_min_speed_norm=snap_min_speed_norm,
                snap_time_mode=snap_time_mode,
                snap_speed_peak_mode=snap_speed_peak_mode,
                mean_windup=mean_windup_duration_s,
                mean_burst=mean_burst_duration_s,
                mean_follow=mean_follow_duration_s,
                mean_burst_share=mean_burst_energy_share,
                median_burst_share=median_burst_energy_share,
                mean_context_ratio=mean_context_energy_ratio,
                median_context_ratio=median_context_energy_ratio,
                mean_energy_contrast=mean_energy_density_contrast,
                median_energy_contrast=median_energy_density_contrast,
                event_keep_topk=event_keep_topk,
                event_rank_by=event_rank_by,
                num=num_bursts,
                total=total_burst_time,
                mean_peak=mean_peak_speed,
                max_peak=max_peak_speed,
                mean_snap_score=mean_snap_score,
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
        mean_windup_duration_s=mean_windup_duration_s,
        mean_burst_duration_s=mean_burst_duration_s,
        mean_follow_duration_s=mean_follow_duration_s,
        mean_burst_energy_share=mean_burst_energy_share,
        median_burst_energy_share=median_burst_energy_share,
        mean_context_energy_ratio=mean_context_energy_ratio,
        median_context_energy_ratio=median_context_energy_ratio,
        mean_energy_density_contrast=mean_energy_density_contrast,
        median_energy_density_contrast=median_energy_density_contrast,
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
        hole_quantile=hole_quantile,
        peak_min_norm=peak_min_norm,
        peak_min_raw=peak_min_raw,
        peak_filter_norm_base=peak_filter_norm_base,
        peak_filter_norm_scale=peak_filter_norm_scale,
        peak_filter_norm_eps=peak_filter_norm_eps,
        detection_mode=detection_mode,
        score_norm_base=score_norm_base,
        score_norm_eps=score_norm_eps,
        dtau_weight=dtau_weight,
        event_speed_alpha=event_speed_alpha,
        event_below_frames=event_below_frames,
        snap_peak_pct=snap_peak_pct,
        snap_peak_min_score=snap_peak_min_score,
        snap_min_speed_norm=snap_min_speed_norm,
        snap_min_gap_s=snap_min_gap_s,
        snap_peak_search_s=snap_peak_search_s,
        snap_time_mode=snap_time_mode,
        snap_speed_peak_mode=snap_speed_peak_mode,
        snap_speed_peak_window_s=snap_speed_peak_window_s,
        burst_energy_rho=burst_energy_rho,
        burst_energy_left_s=burst_energy_left_s,
        burst_energy_right_s=burst_energy_right_s,
        shape_dct_k=shape_dct_k,
        event_keep_topk=event_keep_topk,
        event_rank_by=event_rank_by,
        event_force_primary=event_force_primary,
        event_attach_gap_s=event_attach_gap_s,
        event_attach_peak_ratio=event_attach_peak_ratio,
        windup_rest_frames=windup_rest_frames,
        windup_rest_speed_scale=windup_rest_speed_scale,
        windup_rest_lookback_s=windup_rest_lookback_s,
        windup_rest_min_gap_s=windup_rest_min_gap_s,
        follow_rest_frames=follow_rest_frames,
        follow_rest_speed_scale=follow_rest_speed_scale,
        follow_rest_lookahead_s=follow_rest_lookahead_s,
        follow_rest_min_gap_s=follow_rest_min_gap_s,
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
        "--hole-quantile",
        type=float,
        default=0.0,
        help=(
            "For merge-mode=fill_holes, merge when quantile(hole v_norm, q) >= hole-min-norm. "
            "q=0 keeps old strict behavior (min value). q=0.1 allows small dips."
        ),
    )
    parser.add_argument(
        "--detection-mode",
        choices=DETECTION_MODES,
        default="event_centered",
        help="Burst detector: hysteresis (legacy) or event_centered (snap-driven).",
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
        "--score-norm-base",
        choices=NORM_BASES,
        default="p90",
        help="Normalization base for burst_score terms (acc, dtau).",
    )
    parser.add_argument(
        "--score-norm-eps",
        type=float,
        default=1e-6,
        help="Epsilon used in burst_score normalization denominator.",
    )
    parser.add_argument(
        "--dtau-weight",
        type=float,
        default=0.5,
        help="Weight λ in burst_score = acc_norm + λ * dtau_norm.",
    )
    parser.add_argument(
        "--event-speed-alpha",
        type=float,
        default=0.4,
        help="Relative speed threshold alpha for event window expansion around local speed peak.",
    )
    parser.add_argument(
        "--event-below-frames",
        type=int,
        default=3,
        help="Require this many consecutive frames below alpha*peak speed to close event boundaries.",
    )
    parser.add_argument(
        "--snap-peak-pct",
        type=float,
        default=85.0,
        help="Percentile threshold on burst_score for snap peak candidates.",
    )
    parser.add_argument(
        "--snap-peak-min-score",
        type=float,
        default=0.0,
        help="Absolute minimum burst_score for snap peak candidates.",
    )
    parser.add_argument(
        "--snap-min-speed-norm",
        type=float,
        default=0.6,
        help="Require normalized speed >= this value at snap candidates.",
    )
    parser.add_argument(
        "--snap-min-gap",
        type=float,
        default=0.15,
        help="Minimum time gap between retained snap peaks (seconds).",
    )
    parser.add_argument(
        "--snap-peak-search",
        type=float,
        default=0.2,
        help="Search radius (seconds) around snap for local speed peak.",
    )
    parser.add_argument(
        "--snap-time-mode",
        choices=SNAP_TIME_MODES,
        default="score_peak",
        help="How to set snap_time: score_peak (burst_score peak) or speed_peak (selected speed peak).",
    )
    parser.add_argument(
        "--snap-speed-peak-mode",
        choices=SNAP_SPEED_PEAK_MODES,
        default="max",
        help="How to choose speed peak near snap: max (instantaneous), ma (local mean), energy (local mean of speed^2).",
    )
    parser.add_argument(
        "--snap-speed-peak-window",
        type=float,
        default=0.2,
        help="Window size (seconds) for ma/energy local peak scoring.",
    )
    parser.add_argument(
        "--burst-energy-rho",
        type=float,
        default=0.90,
        help="Target energy coverage ratio rho for energy-burst span inside burst-energy search range.",
    )
    parser.add_argument(
        "--burst-energy-left",
        type=float,
        default=1.5,
        help="Search seconds to the left of snap_time for solving energy-burst span.",
    )
    parser.add_argument(
        "--burst-energy-right",
        type=float,
        default=1.5,
        help="Search seconds to the right of snap_time for solving energy-burst span.",
    )
    parser.add_argument(
        "--shape-dct-k",
        type=int,
        default=4,
        help="Number of non-DC DCT coefficients for z_shape.",
    )
    parser.add_argument(
        "--event-keep-topk",
        type=int,
        default=1,
        help="Keep only top-k events per episode after ranking (default=1 for single-intent prior).",
    )
    parser.add_argument(
        "--event-rank-by",
        choices=EVENT_RANK_BY,
        default="peak_speed",
        help="Ranking metric used by --event-keep-topk.",
    )
    parser.add_argument(
        "--event-force-primary",
        dest="event_force_primary",
        action="store_true",
        help="Ensure at least one event by falling back to global max burst_score when needed.",
    )
    parser.add_argument(
        "--no-event-force-primary",
        dest="event_force_primary",
        action="store_false",
        help="Disable fallback-to-primary event when no event survives filtering.",
    )
    parser.set_defaults(event_force_primary=True)
    parser.add_argument(
        "--event-attach-gap",
        type=float,
        default=0.18,
        help="After selecting top event, merge neighboring strong sub-peaks when gap <= this value (seconds).",
    )
    parser.add_argument(
        "--event-attach-peak-ratio",
        type=float,
        default=0.62,
        help="Neighbor event must satisfy peak_speed >= ratio * primary_peak_speed to be attached.",
    )
    parser.add_argument(
        "--windup-rest-frames",
        type=int,
        default=0,
        help="If >0, estimate windup_start via nearest pre-snap rest segment with this many consecutive frames below threshold.",
    )
    parser.add_argument(
        "--windup-rest-speed-scale",
        type=float,
        default=1.0,
        help="Rest threshold scale: speed < scale * off_th_speed_equiv.",
    )
    parser.add_argument(
        "--windup-rest-lookback",
        type=float,
        default=2.5,
        help="Maximum seconds to look back from burst_start when searching rest segment.",
    )
    parser.add_argument(
        "--windup-rest-min-gap",
        type=float,
        default=0.0,
        help="Ignore rest segments that end too close to burst_start (seconds).",
    )
    parser.add_argument(
        "--follow-rest-frames",
        type=int,
        default=8,
        help="If >0, estimate follow_end via nearest post-snap rest segment with this many consecutive frames below threshold.",
    )
    parser.add_argument(
        "--follow-rest-speed-scale",
        type=float,
        default=1.3,
        help="Rest threshold scale for follow: speed < scale * off_th_speed_equiv.",
    )
    parser.add_argument(
        "--follow-rest-lookahead",
        type=float,
        default=2.5,
        help="Maximum seconds to look ahead from burst_end when searching follow rest segment.",
    )
    parser.add_argument(
        "--follow-rest-min-gap",
        type=float,
        default=0.15,
        help="Ignore follow rest segments that start too close to burst_end (seconds).",
    )
    parser.add_argument(
        "--plot-debug-window",
        action="store_true",
        help="Overlay event windows for debugging (hatched outline only; does not encode phase semantics).",
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
            detection_mode=str(args.detection_mode),
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
            hole_quantile=float(args.hole_quantile),
            peak_min_norm=float(args.peak_min_norm),
            peak_min_raw=float(args.peak_min_raw),
            score_norm_base=str(args.score_norm_base),
            score_norm_eps=float(args.score_norm_eps),
            dtau_weight=float(args.dtau_weight),
            event_speed_alpha=float(args.event_speed_alpha),
            event_below_frames=int(args.event_below_frames),
            snap_peak_pct=float(args.snap_peak_pct),
            snap_peak_min_score=float(args.snap_peak_min_score),
            snap_min_speed_norm=float(args.snap_min_speed_norm),
            snap_min_gap_s=float(args.snap_min_gap),
            snap_peak_search_s=float(args.snap_peak_search),
            snap_time_mode=str(args.snap_time_mode),
            snap_speed_peak_mode=str(args.snap_speed_peak_mode),
            snap_speed_peak_window_s=float(args.snap_speed_peak_window),
            burst_energy_rho=float(args.burst_energy_rho),
            burst_energy_left_s=float(args.burst_energy_left),
            burst_energy_right_s=float(args.burst_energy_right),
            shape_dct_k=int(args.shape_dct_k),
            event_keep_topk=int(args.event_keep_topk),
            event_rank_by=str(args.event_rank_by),
            event_force_primary=bool(args.event_force_primary),
            event_attach_gap_s=float(args.event_attach_gap),
            event_attach_peak_ratio=float(args.event_attach_peak_ratio),
            windup_rest_frames=int(args.windup_rest_frames),
            windup_rest_speed_scale=float(args.windup_rest_speed_scale),
            windup_rest_lookback_s=float(args.windup_rest_lookback),
            windup_rest_min_gap_s=float(args.windup_rest_min_gap),
            follow_rest_frames=int(args.follow_rest_frames),
            follow_rest_speed_scale=float(args.follow_rest_speed_scale),
            follow_rest_lookahead_s=float(args.follow_rest_lookahead),
            follow_rest_min_gap_s=float(args.follow_rest_min_gap),
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
        detection_mode=str(args.detection_mode),
        norm_base=str(args.norm_base),
        norm_eps=float(args.norm_eps),
        on_abs=float(args.on_abs),
        off_abs=float(args.off_abs),
        global_stats=(args.global_stats or None),
        merge_gap=float(args.merge_gap),
        merge_mode=str(args.merge_mode),
        hole_min_norm=float(args.hole_min_norm),
        hole_quantile=float(args.hole_quantile),
        peak_min_norm=float(args.peak_min_norm),
        peak_min_raw=float(args.peak_min_raw),
        score_norm_base=str(args.score_norm_base),
        score_norm_eps=float(args.score_norm_eps),
        dtau_weight=float(args.dtau_weight),
        event_speed_alpha=float(args.event_speed_alpha),
        event_below_frames=int(args.event_below_frames),
        snap_peak_pct=float(args.snap_peak_pct),
        snap_peak_min_score=float(args.snap_peak_min_score),
        snap_min_speed_norm=float(args.snap_min_speed_norm),
        snap_min_gap_s=float(args.snap_min_gap),
        snap_peak_search_s=float(args.snap_peak_search),
        snap_time_mode=str(args.snap_time_mode),
        snap_speed_peak_mode=str(args.snap_speed_peak_mode),
        snap_speed_peak_window_s=float(args.snap_speed_peak_window),
        burst_energy_rho=float(args.burst_energy_rho),
        burst_energy_left_s=float(args.burst_energy_left),
        burst_energy_right_s=float(args.burst_energy_right),
        shape_dct_k=int(args.shape_dct_k),
        event_keep_topk=int(args.event_keep_topk),
        event_rank_by=str(args.event_rank_by),
        event_force_primary=bool(args.event_force_primary),
        event_attach_gap_s=float(args.event_attach_gap),
        event_attach_peak_ratio=float(args.event_attach_peak_ratio),
        windup_rest_frames=int(args.windup_rest_frames),
        windup_rest_speed_scale=float(args.windup_rest_speed_scale),
        windup_rest_lookback_s=float(args.windup_rest_lookback),
        windup_rest_min_gap_s=float(args.windup_rest_min_gap),
        follow_rest_frames=int(args.follow_rest_frames),
        follow_rest_speed_scale=float(args.follow_rest_speed_scale),
        follow_rest_lookahead_s=float(args.follow_rest_lookahead),
        follow_rest_min_gap_s=float(args.follow_rest_min_gap),
        plot_debug_window=bool(args.plot_debug_window),
        print_summary=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
