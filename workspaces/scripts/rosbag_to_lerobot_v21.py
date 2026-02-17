#!/usr/bin/env python3
"""Convert ROS1 dual-arm episode bags into a LeRobot v2.1 style local dataset.

This converter exports one bimanual episode per input bag episode:
  - observation.state = [L_pos, L_effort_raw, R_pos, R_effort_raw]
  - action            = [L_teleop_pos, R_teleop_pos]
  - observation.image (top camera)
  - observation.left_wrist_image
  - observation.right_wrist_image
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class JointTopicSpec:
    pos_dim: int
    effort_dim: int  # 0 for topics where effort is not required.
    role: str  # "robot" | "teleop"


@dataclass
class ConvertedEpisode:
    episode_index: int
    length: int
    parquet_path: Path
    top_video_path: Path
    left_wrist_video_path: Path
    right_wrist_video_path: Path
    stats: Dict[str, Dict]


class EpisodeSkipError(RuntimeError):
    """Expected per-episode skip condition."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert recorded ROS bags to local LeRobot v2.1 dataset files."
    )
    parser.add_argument(
        "--session-dir",
        required=True,
        help="Path to a rosbag session directory (contains episode_*/episode.bag).",
    )
    parser.add_argument(
        "--output-root",
        default="/Users/bamingyuan/Desktop/catkin_ws/demo_data",
        help="Root directory where dataset_id folder is created.",
    )
    parser.add_argument(
        "--dataset-id",
        default="omy_pnp_bimanual",
        help="Dataset folder name under output-root.",
    )
    parser.add_argument(
        "--task",
        default="teleop bimanual pick and place",
        help="Task text written to meta/tasks.jsonl.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=20.0,
        help="Target dataset FPS after temporal resampling.",
    )
    parser.add_argument(
        "--min-frames",
        type=int,
        default=32,
        help="Skip converted episodes shorter than this many valid synchronized frames.",
    )
    parser.add_argument(
        "--max-sync-lag-sec",
        type=float,
        default=0.08,
        help="Drop sampled frames if any stream nearest timestamp exceeds this lag.",
    )
    parser.add_argument(
        "--pos-dim-per-arm",
        type=int,
        default=7,
        help="Robot position dims per arm to keep in observation.state.",
    )
    parser.add_argument(
        "--effort-dim-per-arm",
        type=int,
        default=7,
        help="Robot raw effort dims per arm to keep in observation.state.",
    )
    parser.add_argument(
        "--action-dim-per-arm",
        type=int,
        default=7,
        help="Teleop position dims per arm to keep in action.",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=256,
        help="Resize output videos to this width.",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=256,
        help="Resize output videos to this height.",
    )
    parser.add_argument(
        "--video-codec",
        default="mp4v",
        help="OpenCV fourcc codec used for output mp4 files (default: mp4v).",
    )
    parser.add_argument(
        "--top-camera-topic",
        default="/realsense_top/color/image_raw/compressed",
        help="Overhead camera topic (CompressedImage).",
    )
    parser.add_argument(
        "--left-wrist-topic",
        default="/realsense_left/color/image_raw/compressed",
        help="Left wrist camera topic (CompressedImage).",
    )
    parser.add_argument(
        "--right-wrist-topic",
        default="/realsense_right/color/image_raw/compressed",
        help="Right wrist camera topic (CompressedImage).",
    )
    parser.add_argument(
        "--robot-left-topic",
        default="/robot/arm_left/joint_states_single",
        help="Left robot state topic (JointState).",
    )
    parser.add_argument(
        "--robot-right-topic",
        default="/robot/arm_right/joint_states_single",
        help="Right robot state topic (JointState).",
    )
    parser.add_argument(
        "--teleop-left-topic",
        default="/teleop/arm_left/joint_states_single",
        help="Left teleop command topic (JointState).",
    )
    parser.add_argument(
        "--teleop-right-topic",
        default="/teleop/arm_right/joint_states_single",
        help="Right teleop command topic (JointState).",
    )
    return parser.parse_args()


def import_runtime_deps():
    try:
        import cv2  # type: ignore
        import rosbag  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing runtime deps. Run in ROS1 env with python3 packages: rosbag and cv2."
        ) from exc
    # pandas parquet writer needs one engine: pyarrow or fastparquet.
    try:
        import pyarrow  # noqa: F401
    except Exception:
        try:
            import fastparquet  # noqa: F401
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "Missing parquet engine. Install 'pyarrow' (recommended) or 'fastparquet'."
            ) from exc
    return cv2, rosbag


def sanitize_video_key(key: str) -> str:
    # Keep dots for LeRobot-style keys, only replace path separators and spaces.
    return key.replace("/", "_").replace(" ", "_")


def msg_time_sec(msg, bag_time) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        secs = float(stamp.to_sec())
        if secs > 0.0:
            return secs
    return float(bag_time.to_sec())


def decode_compressed_image(msg, cv2, width: int, height: int):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if arr.size == 0:
        return None
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    if width > 0 and height > 0:
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    return img


def nearest_indices(source_ts: np.ndarray, query_ts: np.ndarray) -> np.ndarray:
    if source_ts.size == 0:
        raise ValueError("nearest_indices: source_ts is empty")
    if source_ts.size == 1:
        return np.zeros(query_ts.shape[0], dtype=np.int64)
    idx = np.searchsorted(source_ts, query_ts, side="left")
    idx = np.clip(idx, 1, source_ts.size - 1)
    prev_idx = idx - 1
    prev_dist = np.abs(query_ts - source_ts[prev_idx])
    next_dist = np.abs(source_ts[idx] - query_ts)
    choose_prev = prev_dist <= next_dist
    idx[choose_prev] = prev_idx[choose_prev]
    return idx.astype(np.int64)


def vector_stats(x: np.ndarray) -> Dict[str, List[float]]:
    # x shape: [N, D]
    return {
        "min": x.min(axis=0).astype(np.float64).tolist(),
        "max": x.max(axis=0).astype(np.float64).tolist(),
        "mean": x.mean(axis=0).astype(np.float64).tolist(),
        "std": x.std(axis=0).astype(np.float64).tolist(),
        "count": [int(x.shape[0])],
    }


def scalar_stats(x: np.ndarray) -> Dict[str, List[float]]:
    # x shape: [N]
    return {
        "min": [float(np.min(x))],
        "max": [float(np.max(x))],
        "mean": [float(np.mean(x))],
        "std": [float(np.std(x))],
        "count": [int(x.shape[0])],
    }


def image_channel_stats(frames: Sequence[np.ndarray]) -> Dict[str, List]:
    if not frames:
        raise ValueError("image_channel_stats: empty frames")
    mins = np.array([1.0, 1.0, 1.0], dtype=np.float64)
    maxs = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    sums = np.zeros(3, dtype=np.float64)
    sums2 = np.zeros(3, dtype=np.float64)
    for frame in frames:
        # BGR uint8 -> RGB float [0, 1]
        rgb = frame[:, :, ::-1].astype(np.float32) / 255.0
        flat = rgb.reshape(-1, 3)
        mins = np.minimum(mins, flat.min(axis=0))
        maxs = np.maximum(maxs, flat.max(axis=0))
        sums += flat.sum(axis=0)
        sums2 += (flat * flat).sum(axis=0)
    n_pixels = float(frames[0].shape[0] * frames[0].shape[1] * len(frames))
    means = sums / n_pixels
    var = np.maximum(sums2 / n_pixels - means * means, 0.0)
    stds = np.sqrt(var)
    # LeRobot v2.1 commonly stores image stats as [C,1,1]
    return {
        "min": [[[float(mins[0])]], [[float(mins[1])]], [[float(mins[2])]]],
        "max": [[[float(maxs[0])]], [[float(maxs[1])]], [[float(maxs[2])]]],
        "mean": [[[float(means[0])]], [[float(means[1])]], [[float(means[2])]]],
        "std": [[[float(stds[0])]], [[float(stds[1])]], [[float(stds[2])]]],
        "count": [int(len(frames))],
    }


def write_mp4(video_path: Path, frames: Sequence[np.ndarray], fps: float, codec: str, cv2) -> None:
    if not frames:
        raise ValueError(f"No frames for video {video_path}")
    h, w = frames[0].shape[:2]
    video_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(str(video_path), fourcc, fps, (w, h))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {video_path} with codec={codec}")
    try:
        for frame in frames:
            writer.write(frame)
    finally:
        writer.release()


def parse_episode_num(name: str) -> int:
    m = re.match(r"episode_(\d+)$", name)
    if not m:
        return 10**9
    return int(m.group(1))


def collect_bag_episode_dirs(session_dir: Path) -> List[Path]:
    eps = [p for p in session_dir.iterdir() if p.is_dir() and p.name.startswith("episode_")]
    eps.sort(key=lambda p: parse_episode_num(p.name))
    out = []
    for ep in eps:
        bag = ep / "episode.bag"
        if bag.is_file():
            out.append(ep)
    return out


def build_joint_and_image_streams(
    *,
    bag_path: Path,
    joint_specs: Mapping[str, JointTopicSpec],
    image_topics: Sequence[str],
    image_width: int,
    image_height: int,
    cv2,
    rosbag,
) -> Tuple[
    Dict[str, np.ndarray],
    Dict[str, np.ndarray],
    Dict[str, np.ndarray],
    Dict[str, np.ndarray],
    Dict[str, List[np.ndarray]],
    Dict[str, int],
]:
    joint_ts: Dict[str, List[float]] = {t: [] for t in joint_specs}
    joint_pos: Dict[str, List[np.ndarray]] = {t: [] for t in joint_specs}
    joint_effort: Dict[str, List[np.ndarray]] = {
        t: [] for t, spec in joint_specs.items() if spec.effort_dim > 0
    }
    image_ts: Dict[str, List[float]] = {t: [] for t in image_topics}
    image_frames: Dict[str, List[np.ndarray]] = {t: [] for t in image_topics}
    dropped_short: Dict[str, int] = {t: 0 for t in joint_specs}

    topics = list(joint_specs.keys()) + list(image_topics)
    bag = rosbag.Bag(str(bag_path), "r")
    try:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = msg_time_sec(msg, t)
            if topic in joint_specs:
                spec = joint_specs[topic]
                pos = list(getattr(msg, "position", []))
                if len(pos) < spec.pos_dim:
                    dropped_short[topic] += 1
                    continue
                joint_ts[topic].append(ts)
                joint_pos[topic].append(np.asarray(pos[: spec.pos_dim], dtype=np.float32))
                if spec.effort_dim > 0:
                    effort = list(getattr(msg, "effort", []))
                    if len(effort) < spec.effort_dim:
                        joint_ts[topic].pop()
                        joint_pos[topic].pop()
                        dropped_short[topic] += 1
                        continue
                    joint_effort[topic].append(np.asarray(effort[: spec.effort_dim], dtype=np.float32))
            elif topic in image_ts:
                frame = decode_compressed_image(msg, cv2, image_width, image_height)
                if frame is None:
                    continue
                image_ts[topic].append(ts)
                image_frames[topic].append(frame)
    finally:
        bag.close()

    jt = {k: np.asarray(v, dtype=np.float64) for k, v in joint_ts.items()}
    jp = {k: np.asarray(v, dtype=np.float32) for k, v in joint_pos.items()}
    je = {k: np.asarray(v, dtype=np.float32) for k, v in joint_effort.items()}
    it = {k: np.asarray(v, dtype=np.float64) for k, v in image_ts.items()}
    return jt, jp, je, it, image_frames, dropped_short


def ensure_nonempty_streams(
    *,
    joint_ts: Mapping[str, np.ndarray],
    joint_pos: Mapping[str, np.ndarray],
    joint_effort: Mapping[str, np.ndarray],
    image_ts: Mapping[str, np.ndarray],
    image_frames: Mapping[str, List[np.ndarray]],
    joint_specs: Mapping[str, JointTopicSpec],
    image_topics: Sequence[str],
) -> None:
    for topic, spec in joint_specs.items():
        if topic not in joint_ts or joint_ts[topic].size == 0:
            raise EpisodeSkipError(f"Missing joint stream: {topic}")
        if topic not in joint_pos or joint_pos[topic].shape[0] == 0:
            raise EpisodeSkipError(f"Missing joint positions: {topic}")
        if spec.effort_dim > 0:
            if topic not in joint_effort or joint_effort[topic].shape[0] == 0:
                raise EpisodeSkipError(f"Missing joint effort stream: {topic}")
    for topic in image_topics:
        if topic not in image_ts or image_ts[topic].size == 0:
            raise EpisodeSkipError(f"Missing image timestamps: {topic}")
        if topic not in image_frames or not image_frames[topic]:
            raise EpisodeSkipError(f"Missing image frames: {topic}")


def _joint_label(i: int) -> str:
    if i < 6:
        return f"joint{i + 1}"
    if i == 6:
        return "gripper"
    return f"joint{i + 1}"


def _make_joint_names(prefix: str, dim: int, suffix: str) -> List[str]:
    return [f"{prefix}.{_joint_label(i)}.{suffix}" for i in range(dim)]


def convert_episode(
    *,
    bag_episode_dir: Path,
    episode_index: int,
    dataset_dir: Path,
    fps: float,
    min_frames: int,
    max_sync_lag_sec: float,
    pos_dim_per_arm: int,
    effort_dim_per_arm: int,
    action_dim_per_arm: int,
    topic_robot_left: str,
    topic_robot_right: str,
    topic_teleop_left: str,
    topic_teleop_right: str,
    topic_top: str,
    topic_left_wrist: str,
    topic_right_wrist: str,
    image_width: int,
    image_height: int,
    video_codec: str,
    cv2,
    rosbag,
) -> ConvertedEpisode:
    bag_path = bag_episode_dir / "episode.bag"
    joint_specs = {
        topic_robot_left: JointTopicSpec(
            pos_dim=pos_dim_per_arm,
            effort_dim=effort_dim_per_arm,
            role="robot",
        ),
        topic_robot_right: JointTopicSpec(
            pos_dim=pos_dim_per_arm,
            effort_dim=effort_dim_per_arm,
            role="robot",
        ),
        topic_teleop_left: JointTopicSpec(
            pos_dim=action_dim_per_arm,
            effort_dim=0,
            role="teleop",
        ),
        topic_teleop_right: JointTopicSpec(
            pos_dim=action_dim_per_arm,
            effort_dim=0,
            role="teleop",
        ),
    }
    image_topics = [topic_top, topic_left_wrist, topic_right_wrist]
    joint_ts, joint_pos, joint_effort, image_ts, image_frames, dropped_short = build_joint_and_image_streams(
        bag_path=bag_path,
        joint_specs=joint_specs,
        image_topics=image_topics,
        image_width=image_width,
        image_height=image_height,
        cv2=cv2,
        rosbag=rosbag,
    )

    for topic, n in dropped_short.items():
        if n > 0:
            print(f"[convert] info {bag_episode_dir.name}: dropped_short_dims topic={topic} count={n}")

    ensure_nonempty_streams(
        joint_ts=joint_ts,
        joint_pos=joint_pos,
        joint_effort=joint_effort,
        image_ts=image_ts,
        image_frames=image_frames,
        joint_specs=joint_specs,
        image_topics=image_topics,
    )

    t_start = max(
        float(joint_ts[topic_robot_left][0]),
        float(joint_ts[topic_robot_right][0]),
        float(joint_ts[topic_teleop_left][0]),
        float(joint_ts[topic_teleop_right][0]),
        float(image_ts[topic_top][0]),
        float(image_ts[topic_left_wrist][0]),
        float(image_ts[topic_right_wrist][0]),
    )
    t_end = min(
        float(joint_ts[topic_robot_left][-1]),
        float(joint_ts[topic_robot_right][-1]),
        float(joint_ts[topic_teleop_left][-1]),
        float(joint_ts[topic_teleop_right][-1]),
        float(image_ts[topic_top][-1]),
        float(image_ts[topic_left_wrist][-1]),
        float(image_ts[topic_right_wrist][-1]),
    )
    if t_end <= t_start:
        raise EpisodeSkipError("No overlapping time window across required streams.")

    raw_frame_count = int(math.floor((t_end - t_start) * fps)) + 1
    if raw_frame_count <= 0:
        raise EpisodeSkipError("No frames after overlap/fps computation.")
    sample_ts = t_start + (np.arange(raw_frame_count, dtype=np.float64) / fps)

    robot_left_idx = nearest_indices(joint_ts[topic_robot_left], sample_ts)
    robot_right_idx = nearest_indices(joint_ts[topic_robot_right], sample_ts)
    teleop_left_idx = nearest_indices(joint_ts[topic_teleop_left], sample_ts)
    teleop_right_idx = nearest_indices(joint_ts[topic_teleop_right], sample_ts)
    top_idx = nearest_indices(image_ts[topic_top], sample_ts)
    left_wrist_idx = nearest_indices(image_ts[topic_left_wrist], sample_ts)
    right_wrist_idx = nearest_indices(image_ts[topic_right_wrist], sample_ts)

    lag_arrays = [
        np.abs(joint_ts[topic_robot_left][robot_left_idx] - sample_ts),
        np.abs(joint_ts[topic_robot_right][robot_right_idx] - sample_ts),
        np.abs(joint_ts[topic_teleop_left][teleop_left_idx] - sample_ts),
        np.abs(joint_ts[topic_teleop_right][teleop_right_idx] - sample_ts),
        np.abs(image_ts[topic_top][top_idx] - sample_ts),
        np.abs(image_ts[topic_left_wrist][left_wrist_idx] - sample_ts),
        np.abs(image_ts[topic_right_wrist][right_wrist_idx] - sample_ts),
    ]
    keep = np.ones(raw_frame_count, dtype=bool)
    for lag in lag_arrays:
        keep &= lag <= max_sync_lag_sec

    kept_count = int(np.sum(keep))
    dropped_lag = int(raw_frame_count - kept_count)
    if dropped_lag > 0:
        print(
            f"[convert] info {bag_episode_dir.name}: dropped_lag_frames={dropped_lag} "
            f"max_sync_lag_sec={max_sync_lag_sec}"
        )

    if kept_count < min_frames:
        raise EpisodeSkipError(
            f"Too few valid synchronized frames: kept={kept_count} min_frames={min_frames}"
        )

    sample_ts = sample_ts[keep]
    robot_left_idx = robot_left_idx[keep]
    robot_right_idx = robot_right_idx[keep]
    teleop_left_idx = teleop_left_idx[keep]
    teleop_right_idx = teleop_right_idx[keep]
    top_idx = top_idx[keep]
    left_wrist_idx = left_wrist_idx[keep]
    right_wrist_idx = right_wrist_idx[keep]

    left_pos = joint_pos[topic_robot_left][robot_left_idx]
    left_effort = joint_effort[topic_robot_left][robot_left_idx]
    right_pos = joint_pos[topic_robot_right][robot_right_idx]
    right_effort = joint_effort[topic_robot_right][robot_right_idx]
    left_teleop_pos = joint_pos[topic_teleop_left][teleop_left_idx]
    right_teleop_pos = joint_pos[topic_teleop_right][teleop_right_idx]

    left_state = np.concatenate([left_pos, left_effort], axis=1)
    right_state = np.concatenate([right_pos, right_effort], axis=1)
    obs_state = np.concatenate([left_state, right_state], axis=1).astype(np.float32)
    teleop_target = np.concatenate([left_teleop_pos, right_teleop_pos], axis=1).astype(np.float32)

    top_frames = [image_frames[topic_top][int(i)] for i in top_idx]
    left_wrist_frames = [image_frames[topic_left_wrist][int(i)] for i in left_wrist_idx]
    right_wrist_frames = [image_frames[topic_right_wrist][int(i)] for i in right_wrist_idx]

    frame_count = obs_state.shape[0]
    ep_chunk = episode_index // 1000
    parquet_path = (
        dataset_dir
        / "data"
        / f"chunk-{ep_chunk:03d}"
        / f"episode_{episode_index:06d}.parquet"
    )
    top_video_path = (
        dataset_dir
        / "videos"
        / f"chunk-{ep_chunk:03d}"
        / sanitize_video_key("observation.image")
        / f"episode_{episode_index:06d}.mp4"
    )
    left_wrist_video_path = (
        dataset_dir
        / "videos"
        / f"chunk-{ep_chunk:03d}"
        / sanitize_video_key("observation.left_wrist_image")
        / f"episode_{episode_index:06d}.mp4"
    )
    right_wrist_video_path = (
        dataset_dir
        / "videos"
        / f"chunk-{ep_chunk:03d}"
        / sanitize_video_key("observation.right_wrist_image")
        / f"episode_{episode_index:06d}.mp4"
    )
    parquet_path.parent.mkdir(parents=True, exist_ok=True)

    rel_ts = (sample_ts - sample_ts[0]).astype(np.float32)
    frame_index = np.arange(frame_count, dtype=np.int64)
    df = pd.DataFrame(
        {
            "observation.state": [row.tolist() for row in obs_state],
            "action": [row.tolist() for row in teleop_target],
            "timestamp": rel_ts.astype(np.float32),
            "frame_index": frame_index,
            "episode_index": np.full(frame_count, episode_index, dtype=np.int64),
            "index": np.zeros(frame_count, dtype=np.int64),  # global index filled later
            "task_index": np.zeros(frame_count, dtype=np.int64),
        }
    )
    df.to_parquet(parquet_path, index=False)

    write_mp4(top_video_path, top_frames, fps=fps, codec=video_codec, cv2=cv2)
    write_mp4(left_wrist_video_path, left_wrist_frames, fps=fps, codec=video_codec, cv2=cv2)
    write_mp4(right_wrist_video_path, right_wrist_frames, fps=fps, codec=video_codec, cv2=cv2)

    stats = {
        "action": vector_stats(teleop_target),
        "observation.state": vector_stats(obs_state),
        "observation.image": image_channel_stats(top_frames),
        "observation.left_wrist_image": image_channel_stats(left_wrist_frames),
        "observation.right_wrist_image": image_channel_stats(right_wrist_frames),
        "timestamp": scalar_stats(rel_ts.astype(np.float64)),
        "frame_index": scalar_stats(frame_index.astype(np.float64)),
        "episode_index": scalar_stats(np.full(frame_count, episode_index, dtype=np.float64)),
        "index": scalar_stats(frame_index.astype(np.float64)),  # placeholder; corrected later
        "task_index": scalar_stats(np.zeros(frame_count, dtype=np.float64)),
    }

    return ConvertedEpisode(
        episode_index=episode_index,
        length=frame_count,
        parquet_path=parquet_path,
        top_video_path=top_video_path,
        left_wrist_video_path=left_wrist_video_path,
        right_wrist_video_path=right_wrist_video_path,
        stats=stats,
    )


def rewrite_global_indices(converted: Sequence[ConvertedEpisode]) -> None:
    global_index = 0
    for ep in converted:
        df = pd.read_parquet(ep.parquet_path)
        n = len(df)
        idx = np.arange(global_index, global_index + n, dtype=np.int64)
        df["index"] = idx
        df.to_parquet(ep.parquet_path, index=False)
        ep.stats["index"] = scalar_stats(idx.astype(np.float64))
        global_index += n


def make_feature_desc(dtype: str, shape: List[int], names):
    return {"dtype": dtype, "shape": shape, "names": names}


def write_metadata_files(
    *,
    dataset_dir: Path,
    dataset_id: str,
    fps: float,
    pos_dim_per_arm: int,
    effort_dim_per_arm: int,
    action_dim_per_arm: int,
    task_text: str,
    converted: Sequence[ConvertedEpisode],
    image_height: int,
    image_width: int,
    video_codec: str,
) -> None:
    meta_dir = dataset_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    total_episodes = len(converted)
    total_frames = int(sum(ep.length for ep in converted))
    total_chunks = max(1, math.ceil(total_episodes / 1000))
    total_videos = total_episodes * 3

    left_pos_names = _make_joint_names("left", pos_dim_per_arm, "pos")
    left_effort_names = _make_joint_names("left", effort_dim_per_arm, "effort_raw")
    right_pos_names = _make_joint_names("right", pos_dim_per_arm, "pos")
    right_effort_names = _make_joint_names("right", effort_dim_per_arm, "effort_raw")
    state_names = left_pos_names + left_effort_names + right_pos_names + right_effort_names

    left_action_names = _make_joint_names("left", action_dim_per_arm, "pos")
    right_action_names = _make_joint_names("right", action_dim_per_arm, "pos")
    action_names = left_action_names + right_action_names

    state_total_dim = len(state_names)
    action_total_dim = len(action_names)

    video_info = {
        "video.fps": float(fps),
        "video.height": int(image_height),
        "video.width": int(image_width),
        "video.channels": 3,
        "video.codec": str(video_codec),
        "video.pix_fmt": "yuv420p",
        "video.is_depth_map": False,
        "has_audio": False,
    }
    video_feature = {
        "dtype": "video",
        "shape": [int(image_height), int(image_width), 3],
        "names": ["height", "width", "channels"],
        # Both keys appear in v2.1 datasets; keeping both improves compatibility.
        "video_info": video_info,
        "info": video_info,
    }

    info = {
        "codebase_version": "v2.1",
        "robot_type": "omy",
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_videos,
        "total_chunks": total_chunks,
        "chunks_size": 1000,
        "fps": float(fps),
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "action": make_feature_desc("float32", [action_total_dim], action_names),
            "observation.state": make_feature_desc("float32", [state_total_dim], state_names),
            "observation.image": video_feature,
            "observation.left_wrist_image": video_feature,
            "observation.right_wrist_image": video_feature,
            "timestamp": make_feature_desc("float32", [1], None),
            "frame_index": make_feature_desc("int64", [1], None),
            "episode_index": make_feature_desc("int64", [1], None),
            "index": make_feature_desc("int64", [1], None),
            "task_index": make_feature_desc("int64", [1], None),
        },
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    (meta_dir / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": task_text}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    with (meta_dir / "episodes.jsonl").open("w", encoding="utf-8") as f:
        for ep in converted:
            row = {"episode_index": ep.episode_index, "tasks": [task_text], "length": ep.length}
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    with (meta_dir / "episodes_stats.jsonl").open("w", encoding="utf-8") as f:
        for ep in converted:
            row = {"episode_index": ep.episode_index, "stats": ep.stats}
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    readme = (
        "# Local LeRobot v2.1 bimanual dataset\n\n"
        f"- dataset_id: `{dataset_id}`\n"
        f"- total_episodes: `{total_episodes}`\n"
        f"- total_frames: `{total_frames}`\n"
        f"- fps: `{fps}`\n"
        f"- observation.state dim: `{state_total_dim}` "
        f"(left pos `{pos_dim_per_arm}` + left effort `{effort_dim_per_arm}` + "
        f"right pos `{pos_dim_per_arm}` + right effort `{effort_dim_per_arm}`)\n"
        f"- action dim: `{action_total_dim}` "
        f"(left teleop pos `{action_dim_per_arm}` + right teleop pos `{action_dim_per_arm}`)\n"
    )
    (dataset_dir / "README.md").write_text(readme, encoding="utf-8")


def _validate_positive(name: str, value: int) -> None:
    if value <= 0:
        raise RuntimeError(f"{name} must be > 0, got {value}")


def main() -> int:
    args = parse_args()
    cv2, rosbag = import_runtime_deps()

    _validate_positive("pos-dim-per-arm", int(args.pos_dim_per_arm))
    _validate_positive("effort-dim-per-arm", int(args.effort_dim_per_arm))
    _validate_positive("action-dim-per-arm", int(args.action_dim_per_arm))
    _validate_positive("min-frames", int(args.min_frames))
    if float(args.fps) <= 0:
        raise RuntimeError(f"--fps must be > 0, got {args.fps}")
    if float(args.max_sync_lag_sec) < 0:
        raise RuntimeError(f"--max-sync-lag-sec must be >= 0, got {args.max_sync_lag_sec}")

    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        raise RuntimeError(f"session-dir not found: {session_dir}")

    output_root = Path(args.output_root).expanduser().resolve()
    dataset_dir = output_root / args.dataset_id
    dataset_dir.mkdir(parents=True, exist_ok=True)

    episode_dirs = collect_bag_episode_dirs(session_dir)
    if not episode_dirs:
        raise RuntimeError(f"No episode_*/episode.bag found under {session_dir}")

    converted: List[ConvertedEpisode] = []
    skip_reasons: Dict[str, int] = {}
    next_ep_idx = 0

    for bag_episode_dir in episode_dirs:
        print(f"[convert] {bag_episode_dir.name} -> episode_{next_ep_idx:06d}")
        try:
            out = convert_episode(
                bag_episode_dir=bag_episode_dir,
                episode_index=next_ep_idx,
                dataset_dir=dataset_dir,
                fps=float(args.fps),
                min_frames=int(args.min_frames),
                max_sync_lag_sec=float(args.max_sync_lag_sec),
                pos_dim_per_arm=int(args.pos_dim_per_arm),
                effort_dim_per_arm=int(args.effort_dim_per_arm),
                action_dim_per_arm=int(args.action_dim_per_arm),
                topic_robot_left=str(args.robot_left_topic),
                topic_robot_right=str(args.robot_right_topic),
                topic_teleop_left=str(args.teleop_left_topic),
                topic_teleop_right=str(args.teleop_right_topic),
                topic_top=str(args.top_camera_topic),
                topic_left_wrist=str(args.left_wrist_topic),
                topic_right_wrist=str(args.right_wrist_topic),
                image_width=int(args.image_width),
                image_height=int(args.image_height),
                video_codec=str(args.video_codec),
                cv2=cv2,
                rosbag=rosbag,
            )
            converted.append(out)
            next_ep_idx += 1
        except EpisodeSkipError as exc:
            reason = str(exc)
            skip_reasons[reason] = skip_reasons.get(reason, 0) + 1
            print(f"[convert] WARN skip {bag_episode_dir.name}: {reason}")
        except Exception as exc:
            reason = f"Unexpected error: {exc}"
            skip_reasons[reason] = skip_reasons.get(reason, 0) + 1
            print(f"[convert] WARN skip {bag_episode_dir.name}: {reason}")

    if not converted:
        details = ", ".join([f"{k} x{v}" for k, v in skip_reasons.items()]) or "no details"
        raise RuntimeError(f"No converted episodes were produced. skip_reasons: {details}")

    rewrite_global_indices(converted)
    write_metadata_files(
        dataset_dir=dataset_dir,
        dataset_id=str(args.dataset_id),
        fps=float(args.fps),
        pos_dim_per_arm=int(args.pos_dim_per_arm),
        effort_dim_per_arm=int(args.effort_dim_per_arm),
        action_dim_per_arm=int(args.action_dim_per_arm),
        task_text=str(args.task),
        converted=converted,
        image_height=int(args.image_height),
        image_width=int(args.image_width),
        video_codec=str(args.video_codec),
    )

    print(f"[convert] done: {len(converted)} episodes -> {dataset_dir}")
    if skip_reasons:
        print("[convert] skip summary:")
        for reason, count in sorted(skip_reasons.items(), key=lambda x: (-x[1], x[0])):
            print(f"[convert]   {count} x {reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
