#!/usr/bin/env python3
"""Reconstruct ROS1 episode bags from a local LeRobot dataset."""

from __future__ import annotations

import argparse
import json
import shutil
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Set

import numpy as np

from session_format_utils import build_standard_metadata, write_json


IMAGE_VIDEO_KEYS = (
    "observation.image",
    "observation.left_wrist_image",
    "observation.right_wrist_image",
)


@dataclass(frozen=True)
class JointLayout:
    left_pos_names: List[str]
    left_effort_names: List[str]
    right_pos_names: List[str]
    right_effort_names: List[str]
    left_action_names: List[str]
    right_action_names: List[str]

    @property
    def state_dim(self) -> int:
        return (
            len(self.left_pos_names)
            + len(self.left_effort_names)
            + len(self.right_pos_names)
            + len(self.right_effort_names)
        )

    @property
    def action_dim(self) -> int:
        return len(self.left_action_names) + len(self.right_action_names)


@dataclass(frozen=True)
class EpisodeSpec:
    episode_index: int
    length: int
    task_texts: List[str]
    data_path: Path
    data_from_index: int
    data_to_index: int
    video_paths: Dict[str, Path]


@dataclass
class VideoReader:
    backend: str
    resource: object
    iterator: object


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a local LeRobot dataset back into ROS1 episode bags."
    )
    parser.add_argument(
        "--dataset-dir",
        required=True,
        help="Path to the local LeRobot dataset folder.",
    )
    parser.add_argument(
        "--output-session-dir",
        default="data/rosbags/lerobot_reconstructed",
        help="Destination session directory containing episode_*/episode.bag.",
    )
    parser.add_argument(
        "--episodes",
        default="all",
        help="Episode indices to export, e.g. 'all', '0,3,8', or '2-5'.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove an existing output session directory before writing.",
    )
    parser.add_argument(
        "--skip-images",
        action="store_true",
        help="Write only joint topics and omit camera image topics.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality used for reconstructed CompressedImage messages.",
    )
    parser.add_argument(
        "--bag-start-time-sec",
        type=float,
        default=None,
        help=(
            "Absolute bag start time in Unix seconds. Defaults to current wall time "
            "for each episode."
        ),
    )
    parser.add_argument(
        "--top-camera-topic",
        default="/realsense_top/color/image_raw/compressed",
        help="Topic for observation.image.",
    )
    parser.add_argument(
        "--left-wrist-topic",
        default="/realsense_left/color/image_raw/compressed",
        help="Topic for observation.left_wrist_image.",
    )
    parser.add_argument(
        "--right-wrist-topic",
        default="/realsense_right/color/image_raw/compressed",
        help="Topic for observation.right_wrist_image.",
    )
    parser.add_argument(
        "--robot-left-topic",
        default="/robot/arm_left/joint_states_single",
        help="Left robot joint state topic.",
    )
    parser.add_argument(
        "--robot-right-topic",
        default="/robot/arm_right/joint_states_single",
        help="Right robot joint state topic.",
    )
    parser.add_argument(
        "--teleop-left-topic",
        default="/teleop/arm_left/joint_states_single",
        help="Left teleop joint state topic.",
    )
    parser.add_argument(
        "--teleop-right-topic",
        default="/teleop/arm_right/joint_states_single",
        help="Right teleop joint state topic.",
    )
    parser.add_argument(
        "--top-camera-frame-id",
        default="realsense_top_color_optical_frame",
        help="frame_id for observation.image messages.",
    )
    parser.add_argument(
        "--left-camera-frame-id",
        default="realsense_left_color_optical_frame",
        help="frame_id for observation.left_wrist_image messages.",
    )
    parser.add_argument(
        "--right-camera-frame-id",
        default="realsense_right_color_optical_frame",
        help="frame_id for observation.right_wrist_image messages.",
    )
    return parser.parse_args()


def import_runtime_deps():
    try:
        import cv2  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "opencv-python-headless is required to decode and encode video frames."
        ) from exc
    try:
        import pyarrow.dataset as pads  # type: ignore
        import pyarrow.parquet as pq  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("pyarrow is required to read LeRobot parquet metadata.") from exc
    try:
        import av  # type: ignore
    except Exception:
        av = None
    try:
        from rosbags.rosbag1 import Writer  # type: ignore
        from rosbags.typesys import Stores, get_typestore  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "rosbags is required to write ROS1 bag files without a ROS runtime."
        ) from exc
    return cv2, pads, pq, av, Writer, Stores, get_typestore


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def parse_episode_selection(text: str) -> Optional[Set[int]]:
    value = text.strip().lower()
    if value in ("", "all", "*"):
        return None

    selected: Set[int] = set()
    for part in text.split(","):
        token = part.strip()
        if not token:
            continue
        if "-" in token:
            left, right = token.split("-", 1)
            start = int(left)
            end = int(right)
            if end < start:
                raise RuntimeError(f"Invalid episode range: {token}")
            selected.update(range(start, end + 1))
            continue
        selected.add(int(token))

    if not selected:
        raise RuntimeError(f"Invalid --episodes value: {text}")
    return selected


def load_info(dataset_dir: Path) -> Dict:
    info_path = dataset_dir / "meta" / "info.json"
    if not info_path.is_file():
        raise RuntimeError(f"Missing info.json: {info_path}")
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if not isinstance(info, dict):
        raise RuntimeError(f"Expected JSON object in {info_path}")
    return info


def split_feature_name(name: str) -> tuple[str, str, str]:
    parts = name.split(".")
    if len(parts) < 3:
        raise RuntimeError(f"Unexpected feature name: {name}")
    side = parts[0]
    field = parts[-1]
    joint_name = ".".join(parts[1:-1])
    return side, joint_name, field


def parse_joint_layout(info: Mapping) -> JointLayout:
    features = info.get("features", {})
    if not isinstance(features, Mapping):
        raise RuntimeError("info.json missing feature descriptions")

    state = features.get("observation.state")
    action = features.get("action")
    if not isinstance(state, Mapping) or not isinstance(action, Mapping):
        raise RuntimeError("Dataset must contain observation.state and action features")

    state_names = state.get("names")
    action_names = action.get("names")
    if not isinstance(state_names, list) or not isinstance(action_names, list):
        raise RuntimeError("Dataset feature names are missing in info.json")

    left_pos_names: List[str] = []
    left_effort_names: List[str] = []
    right_pos_names: List[str] = []
    right_effort_names: List[str] = []
    left_action_names: List[str] = []
    right_action_names: List[str] = []

    for raw_name in state_names:
        if not isinstance(raw_name, str):
            continue
        side, joint_name, field = split_feature_name(raw_name)
        if side == "left" and field == "pos":
            left_pos_names.append(joint_name)
        elif side == "left" and field == "effort_raw":
            left_effort_names.append(joint_name)
        elif side == "right" and field == "pos":
            right_pos_names.append(joint_name)
        elif side == "right" and field == "effort_raw":
            right_effort_names.append(joint_name)

    for raw_name in action_names:
        if not isinstance(raw_name, str):
            continue
        side, joint_name, field = split_feature_name(raw_name)
        if field != "pos":
            continue
        if side == "left":
            left_action_names.append(joint_name)
        elif side == "right":
            right_action_names.append(joint_name)

    layout = JointLayout(
        left_pos_names=left_pos_names,
        left_effort_names=left_effort_names,
        right_pos_names=right_pos_names,
        right_effort_names=right_effort_names,
        left_action_names=left_action_names or list(left_pos_names),
        right_action_names=right_action_names or list(right_pos_names),
    )
    if layout.state_dim <= 0 or layout.action_dim <= 0:
        raise RuntimeError("Failed to infer joint layout from info.json")
    return layout


def _format_dataset_path(
    template: str,
    *,
    episode_index: int,
    video_key: Optional[str] = None,
    chunk_index: Optional[int] = None,
    file_index: Optional[int] = None,
) -> Path:
    values = {
        "episode_index": int(episode_index),
        "episode_chunk": int(episode_index) // 1000,
        "chunk_index": 0 if chunk_index is None else int(chunk_index),
        "file_index": 0 if file_index is None else int(file_index),
        "video_key": video_key or "",
    }
    try:
        return Path(template.format(**values))
    except KeyError as exc:
        raise RuntimeError(f"Unsupported path template: {template} (missing {exc})") from exc


def load_v30_episode_specs(dataset_dir: Path, info: Mapping, pads) -> List[EpisodeSpec]:
    episodes_dir = dataset_dir / "meta" / "episodes"
    if not episodes_dir.is_dir():
        return []

    table = pads.dataset(str(episodes_dir), format="parquet").to_table()
    rows = table.to_pylist()
    data_template = str(info.get("data_path", ""))
    video_template = str(info.get("video_path", ""))
    if not data_template or not video_template:
        raise RuntimeError("info.json missing data_path/video_path templates")

    specs: List[EpisodeSpec] = []
    for row in rows:
        episode_index = int(row["episode_index"])
        length = int(row["length"])
        data_chunk = int(row["data/chunk_index"])
        data_file = int(row["data/file_index"])
        data_from = int(row["dataset_from_index"])
        data_to = int(row["dataset_to_index"])

        video_paths: Dict[str, Path] = {}
        for video_key in IMAGE_VIDEO_KEYS:
            chunk_key = f"videos/{video_key}/chunk_index"
            file_key = f"videos/{video_key}/file_index"
            if chunk_key not in row or file_key not in row:
                continue
            video_paths[video_key] = dataset_dir / _format_dataset_path(
                video_template,
                episode_index=episode_index,
                video_key=video_key,
                chunk_index=int(row[chunk_key]),
                file_index=int(row[file_key]),
            )

        task_texts = [str(item) for item in row.get("tasks", []) if isinstance(item, str)]
        specs.append(
            EpisodeSpec(
                episode_index=episode_index,
                length=length,
                task_texts=task_texts,
                data_path=dataset_dir
                / _format_dataset_path(
                    data_template,
                    episode_index=episode_index,
                    chunk_index=data_chunk,
                    file_index=data_file,
                ),
                data_from_index=data_from,
                data_to_index=data_to,
                video_paths=video_paths,
            )
        )
    return sorted(specs, key=lambda spec: spec.episode_index)


def load_v21_episode_specs(dataset_dir: Path, info: Mapping) -> List[EpisodeSpec]:
    episodes_path = dataset_dir / "meta" / "episodes.jsonl"
    if not episodes_path.is_file():
        return []

    data_template = str(info.get("data_path", ""))
    video_template = str(info.get("video_path", ""))
    if not data_template or not video_template:
        raise RuntimeError("info.json missing data_path/video_path templates")

    specs: List[EpisodeSpec] = []
    with episodes_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            row = json.loads(text)
            episode_index = int(row["episode_index"])
            length = int(row["length"])
            task_texts = [str(item) for item in row.get("tasks", []) if isinstance(item, str)]
            video_paths = {
                key: dataset_dir
                / _format_dataset_path(
                    video_template,
                    episode_index=episode_index,
                    video_key=key,
                )
                for key in IMAGE_VIDEO_KEYS
            }
            specs.append(
                EpisodeSpec(
                    episode_index=episode_index,
                    length=length,
                    task_texts=task_texts,
                    data_path=dataset_dir
                    / _format_dataset_path(
                        data_template,
                        episode_index=episode_index,
                    ),
                    data_from_index=0,
                    data_to_index=length,
                    video_paths=video_paths,
                )
            )
    return sorted(specs, key=lambda spec: spec.episode_index)


def load_episode_specs(dataset_dir: Path, info: Mapping, pads) -> List[EpisodeSpec]:
    specs = load_v30_episode_specs(dataset_dir, info, pads)
    if specs:
        return specs
    specs = load_v21_episode_specs(dataset_dir, info)
    if specs:
        return specs
    raise RuntimeError(
        "Could not find episode metadata. Expected meta/episodes/*.parquet or meta/episodes.jsonl."
    )


def read_episode_table_slice(spec: EpisodeSpec, pq):
    if not spec.data_path.is_file():
        raise RuntimeError(f"Missing episode parquet: {spec.data_path}")
    table = pq.read_table(
        spec.data_path,
        columns=["observation.state", "action", "timestamp", "frame_index", "episode_index"],
    )
    start = int(spec.data_from_index)
    stop = int(spec.data_to_index)
    if stop < start:
        raise RuntimeError(f"Invalid row slice for episode {spec.episode_index}: {start}:{stop}")
    if stop > table.num_rows:
        raise RuntimeError(
            f"Episode {spec.episode_index} slice exceeds parquet rows: {stop}>{table.num_rows}"
        )
    return table.slice(start, stop - start)


def validate_monotonic_timestamps(timestamps: np.ndarray, *, episode_index: int) -> None:
    if timestamps.size == 0:
        raise RuntimeError(f"Episode {episode_index} has no frames")
    deltas = np.diff(timestamps)
    if np.any(deltas < -1e-9):
        raise RuntimeError(f"Episode {episode_index} timestamps are not monotonically increasing")


def split_state_action(
    state: np.ndarray,
    action: np.ndarray,
    layout: JointLayout,
) -> Dict[str, np.ndarray]:
    if state.shape[1] != layout.state_dim:
        raise RuntimeError(
            f"State dim mismatch: parquet has {state.shape[1]}, inferred {layout.state_dim}"
        )
    if action.shape[1] != layout.action_dim:
        raise RuntimeError(
            f"Action dim mismatch: parquet has {action.shape[1]}, inferred {layout.action_dim}"
        )

    left_pos_end = len(layout.left_pos_names)
    left_effort_end = left_pos_end + len(layout.left_effort_names)
    right_pos_end = left_effort_end + len(layout.right_pos_names)

    left_action_end = len(layout.left_action_names)

    return {
        "robot_left_pos": state[:, :left_pos_end],
        "robot_left_effort": state[:, left_pos_end:left_effort_end],
        "robot_right_pos": state[:, left_effort_end:right_pos_end],
        "robot_right_effort": state[:, right_pos_end:],
        "teleop_left_pos": action[:, :left_action_end],
        "teleop_right_pos": action[:, left_action_end:],
    }


def open_video_reader(video_path: Path, cv2, av_module) -> VideoReader:
    if av_module is not None:
        container = None
        try:
            container = av_module.open(str(video_path))
            stream = container.streams.video[0]
            return VideoReader(
                backend="av",
                resource=container,
                iterator=iter(container.decode(stream)),
            )
        except Exception:
            try:
                if container is not None:
                    container.close()
            except Exception:
                pass

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video file: {video_path}")
    return VideoReader(backend="cv2", resource=cap, iterator=None)


def read_next_frame(reader: VideoReader, video_path: Path) -> np.ndarray:
    if reader.backend == "av":
        try:
            frame = next(reader.iterator)
        except StopIteration as exc:
            raise RuntimeError(f"Video ended early while reading {video_path}") from exc
        return frame.to_ndarray(format="bgr24")

    ok, frame = reader.resource.read()
    if not ok or frame is None:
        raise RuntimeError(f"Video ended early while reading {video_path}")
    return frame


def close_video_reader(reader: VideoReader) -> None:
    if reader.backend == "av":
        reader.resource.close()
        return
    reader.resource.release()


def bag_time_ns(base_time_sec: float, relative_time_sec: float) -> int:
    return int(round((float(base_time_sec) + float(relative_time_sec)) * 1e9))


def stamp_message_time(stamp_ns: int, time_cls):
    sec = int(stamp_ns // 1_000_000_000)
    nanosec = int(stamp_ns % 1_000_000_000)
    return time_cls(sec=sec, nanosec=nanosec)


def build_joint_state_msg(
    *,
    seq: int,
    stamp_ns: int,
    frame_id: str,
    joint_names: Sequence[str],
    positions: np.ndarray,
    efforts: np.ndarray,
    header_cls,
    time_cls,
    joint_state_cls,
):
    return joint_state_cls(
        header=header_cls(
            seq=int(seq),
            stamp=stamp_message_time(stamp_ns, time_cls),
            frame_id=frame_id,
        ),
        name=list(joint_names),
        position=np.asarray(positions, dtype=np.float64),
        velocity=np.asarray([], dtype=np.float64),
        effort=np.asarray(efforts, dtype=np.float64),
    )


def build_compressed_image_msg(
    *,
    seq: int,
    stamp_ns: int,
    frame_id: str,
    frame: np.ndarray,
    jpeg_quality: int,
    cv2,
    header_cls,
    time_cls,
    compressed_image_cls,
):
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError("Failed to JPEG-encode a video frame")
    return compressed_image_cls(
        header=header_cls(
            seq=int(seq),
            stamp=stamp_message_time(stamp_ns, time_cls),
            frame_id=frame_id,
        ),
        format="jpeg",
        data=np.asarray(encoded, dtype=np.uint8),
    )


def ensure_clean_output_dir(path: Path, *, overwrite: bool) -> None:
    if path.exists():
        if not overwrite:
            raise RuntimeError(f"Output directory already exists: {path} (use --overwrite)")
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def write_episode_bag(
    *,
    dataset_dir: Path,
    output_session_dir: Path,
    spec: EpisodeSpec,
    layout: JointLayout,
    args: argparse.Namespace,
    info: Mapping,
    cv2,
    av_module,
    pq,
    Writer,
    typestore,
) -> Dict[str, object]:
    table = read_episode_table_slice(spec, pq)
    frame_count = table.num_rows
    if frame_count <= 0:
        raise RuntimeError(f"Episode {spec.episode_index} has no rows")
    if spec.length and frame_count != int(spec.length):
        raise RuntimeError(
            f"Episode {spec.episode_index} row count mismatch: metadata={spec.length} actual={frame_count}"
        )

    columns = table.to_pydict()
    state = np.asarray(columns["observation.state"], dtype=np.float64)
    action = np.asarray(columns["action"], dtype=np.float64)
    timestamps = np.asarray(columns["timestamp"], dtype=np.float64)
    validate_monotonic_timestamps(timestamps, episode_index=spec.episode_index)

    split = split_state_action(state, action, layout)

    episode_name = f"episode_{spec.episode_index:03d}"
    episode_dir = output_session_dir / episode_name
    episode_dir.mkdir(parents=True, exist_ok=True)
    bag_path = episode_dir / "episode.bag"

    bag_start_sec = (
        float(args.bag_start_time_sec) if args.bag_start_time_sec is not None else time.time()
    )
    started_at_utc = utc_now_iso()

    header_cls = typestore.types["std_msgs/msg/Header"]
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    joint_state_cls = typestore.types["sensor_msgs/msg/JointState"]
    compressed_image_cls = typestore.types["sensor_msgs/msg/CompressedImage"]

    joint_topics = {
        args.robot_left_topic: (
            layout.left_pos_names,
            split["robot_left_pos"],
            split["robot_left_effort"],
        ),
        args.robot_right_topic: (
            layout.right_pos_names,
            split["robot_right_pos"],
            split["robot_right_effort"],
        ),
        args.teleop_left_topic: (
            layout.left_action_names,
            split["teleop_left_pos"],
            np.empty((frame_count, 0)),
        ),
        args.teleop_right_topic: (
            layout.right_action_names,
            split["teleop_right_pos"],
            np.empty((frame_count, 0)),
        ),
    }
    image_topics = {
        "observation.image": (args.top_camera_topic, args.top_camera_frame_id),
        "observation.left_wrist_image": (args.left_wrist_topic, args.left_camera_frame_id),
        "observation.right_wrist_image": (args.right_wrist_topic, args.right_camera_frame_id),
    }

    video_caps = {}
    if not args.skip_images:
        for video_key, video_path in spec.video_paths.items():
            if not video_path.is_file():
                raise RuntimeError(
                    f"Missing video for episode {spec.episode_index}: {video_path}"
                )
            video_caps[video_key] = open_video_reader(video_path, cv2, av_module)

    try:
        with Writer(str(bag_path)) as writer:
            joint_connections = {
                topic: writer.add_connection(
                    topic,
                    "sensor_msgs/msg/JointState",
                    typestore=typestore,
                )
                for topic in joint_topics
            }
            image_connections = {}
            if not args.skip_images:
                image_connections = {
                    image_topics[video_key][0]: writer.add_connection(
                        image_topics[video_key][0],
                        "sensor_msgs/msg/CompressedImage",
                        typestore=typestore,
                    )
                    for video_key in spec.video_paths
                }

            for idx in range(frame_count):
                stamp_ns = bag_time_ns(bag_start_sec, float(timestamps[idx]))
                for topic, (joint_names, positions, efforts) in joint_topics.items():
                    msg = build_joint_state_msg(
                        seq=idx,
                        stamp_ns=stamp_ns,
                        frame_id="",
                        joint_names=joint_names,
                        positions=positions[idx],
                        efforts=efforts[idx],
                        header_cls=header_cls,
                        time_cls=time_cls,
                        joint_state_cls=joint_state_cls,
                    )
                    writer.write(
                        joint_connections[topic],
                        stamp_ns,
                        typestore.serialize_ros1(msg, "sensor_msgs/msg/JointState"),
                    )

                if not args.skip_images:
                    for video_key, cap in video_caps.items():
                        frame = read_next_frame(cap, spec.video_paths[video_key])
                        topic, frame_id = image_topics[video_key]
                        msg = build_compressed_image_msg(
                            seq=idx,
                            stamp_ns=stamp_ns,
                            frame_id=frame_id,
                            frame=frame,
                            jpeg_quality=int(args.jpeg_quality),
                            cv2=cv2,
                            header_cls=header_cls,
                            time_cls=time_cls,
                            compressed_image_cls=compressed_image_cls,
                        )
                        writer.write(
                            image_connections[topic],
                            stamp_ns,
                            typestore.serialize_ros1(msg, "sensor_msgs/msg/CompressedImage"),
                        )
    finally:
        for reader in video_caps.values():
            close_video_reader(reader)

    ended_at_utc = utc_now_iso()
    bag_end_sec = bag_start_sec + float(timestamps[-1])
    bag_duration = float(timestamps[-1] - timestamps[0]) if frame_count > 1 else 0.0

    per_topic = {
        topic: {
            "messages": int(frame_count),
            "type": "sensor_msgs/JointState",
            "bags": 1,
        }
        for topic in joint_topics
    }
    if not args.skip_images:
        for video_key, (topic, _) in image_topics.items():
            if video_key not in spec.video_paths:
                continue
            per_topic[topic] = {
                "messages": int(frame_count),
                "type": "sensor_msgs/CompressedImage",
                "bags": 1,
            }

    task_text = spec.task_texts[0] if spec.task_texts else dataset_dir.name
    bag_stats = {
        "bag_files": [str(bag_path)],
        "bag_count": 1,
        "total_size_bytes": int(bag_path.stat().st_size if bag_path.exists() else 0),
        "total_messages": int(frame_count * len(per_topic)),
        "start_time": float(bag_start_sec + float(timestamps[0])),
        "end_time": float(bag_end_sec),
        "duration_sec": float(bag_duration),
        "per_topic": per_topic,
        "parse_warnings": [],
    }

    source_meta = {
        "recording": {
            "compression": "none",
            "split_enabled": False,
            "split_size_mb": None,
            "camera_transport": "compressed" if not args.skip_images else None,
            "keep_debug_logs": False,
        },
        "topics": {
            "recorded_topics": list(per_topic.keys()),
            "required_joint_topics": [
                args.robot_left_topic,
                args.robot_right_topic,
                args.teleop_left_topic,
                args.teleop_right_topic,
            ],
            "source_dataset_dir": str(dataset_dir),
        },
        "environment": {
            "source_dataset_dir": str(dataset_dir),
            "codebase_version": info.get("codebase_version"),
            "robot_type": info.get("robot_type"),
        },
        "notes": (
            "Reconstructed from LeRobot dataset. Original asynchronous ROS timings cannot "
            "be recovered; all exported topics share the dataset frame timestamps."
        ),
    }
    metadata = build_standard_metadata(
        source_meta=source_meta,
        session_name=output_session_dir.name,
        episode_name=episode_name,
        episode_index=spec.episode_index,
        episode_dir=episode_dir,
        bag_stats=bag_stats,
        started_at_utc=started_at_utc,
        ended_at_utc=ended_at_utc,
        duration_sec=bag_duration,
        stop_reason=f"lerobot_to_rosbag:{task_text}",
        force_single_bag=True,
    )
    write_json(episode_dir / "metadata.json", metadata)

    return {
        "episode_index": spec.episode_index,
        "frame_count": frame_count,
        "bag_path": bag_path,
        "task_text": task_text,
        "topic_count": len(per_topic),
    }


def main() -> int:
    args = parse_args()
    cv2, pads, pq, av_module, Writer, Stores, get_typestore = import_runtime_deps()

    dataset_dir = Path(args.dataset_dir).expanduser().resolve()
    if not dataset_dir.is_dir():
        raise RuntimeError(f"Dataset directory not found: {dataset_dir}")

    output_session_dir = Path(args.output_session_dir).expanduser().resolve()
    ensure_clean_output_dir(output_session_dir, overwrite=bool(args.overwrite))

    selected_episodes = parse_episode_selection(args.episodes)
    info = load_info(dataset_dir)
    layout = parse_joint_layout(info)
    specs = load_episode_specs(dataset_dir, info, pads)
    if selected_episodes is not None:
        specs = [spec for spec in specs if spec.episode_index in selected_episodes]
    if not specs:
        raise RuntimeError("No episodes selected for export")

    typestore = get_typestore(Stores.ROS1_NOETIC)
    results = []
    for spec in specs:
        print(f"[convert] episode_{spec.episode_index:03d}")
        result = write_episode_bag(
            dataset_dir=dataset_dir,
            output_session_dir=output_session_dir,
            spec=spec,
            layout=layout,
            args=args,
            info=info,
            cv2=cv2,
            av_module=av_module,
            pq=pq,
            Writer=Writer,
            typestore=typestore,
        )
        results.append(result)

    total_frames = sum(int(item["frame_count"]) for item in results)
    print(
        f"[done] wrote {len(results)} episode bags under {output_session_dir} "
        f"(frames={total_frames})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
