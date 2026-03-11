#!/usr/bin/env python3
"""Shared helpers for strict rosbag session format compatibility."""

from __future__ import annotations

import copy
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("PyYAML is required (python3-yaml).") from exc


STANDARD_METADATA_KEYS: Tuple[str, ...] = (
    "schema_version",
    "session_name",
    "episode_name",
    "episode_index",
    "episode_dir",
    "started_at_utc",
    "ended_at_utc",
    "duration_sec",
    "stop_reason",
    "recording",
    "topics",
    "preflight",
    "hz_monitor",
    "bag_stats",
    "environment",
    "notes",
    "debug_logs",
    "warnings",
    "rosbag_process_terminated_cleanly",
    "save_job_id",
)

STANDARD_RECORDING_KEYS: Tuple[str, ...] = (
    "compression",
    "split_enabled",
    "split_size_mb",
    "camera_transport",
    "keep_debug_logs",
)


def parse_episode_num(name: str) -> int:
    match = re.match(r"episode_(\d+)$", name)
    if not match:
        return 10**9
    return int(match.group(1))


def list_episode_dirs(session_dir: Path) -> List[Path]:
    episodes = [p for p in session_dir.iterdir() if p.is_dir() and p.name.startswith("episode_")]
    episodes.sort(key=lambda p: parse_episode_num(p.name))
    return episodes


def load_json_dict(path: Path) -> Dict[str, Any]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(obj, dict):
        raise RuntimeError(f"Expected JSON object in {path}")
    return obj


def write_json(path: Path, obj: Mapping[str, Any]) -> None:
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=True), encoding="utf-8")


def copy_or_link_file(src: Path, dst: Path, copy_mode: str) -> None:
    if copy_mode == "copy":
        shutil.copy2(src, dst)
        return
    if copy_mode == "symlink":
        os.symlink(src, dst)
        return
    if copy_mode == "hardlink":
        os.link(src, dst)
        return
    raise RuntimeError(f"Unsupported copy_mode: {copy_mode}")


def _deepcopy_dict(value: Any) -> Dict[str, Any]:
    if isinstance(value, dict):
        return copy.deepcopy(value)
    return {}


def _deepcopy_list(value: Any) -> List[Any]:
    if isinstance(value, list):
        return copy.deepcopy(value)
    return []


def _to_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _to_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off"):
            return False
    return default


def _to_float_or_none(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _run_cmd(cmd: Sequence[str], timeout: Optional[float] = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(cmd),
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def _candidate_rosbag_bins() -> List[str]:
    candidates: List[str] = ["rosbag"]
    distro = os.environ.get("ROS_DISTRO", "").strip()
    if distro:
        candidates.append(f"/opt/ros/{distro}/bin/rosbag")
    candidates.append("/opt/ros/noetic/bin/rosbag")
    dedup: List[str] = []
    seen = set()
    for item in candidates:
        if item in seen:
            continue
        seen.add(item)
        dedup.append(item)
    return dedup


def _fill_stats_from_rosbag_yaml(info: Mapping[str, Any]) -> Tuple[Optional[float], Optional[float], int, Dict[str, Dict[str, Any]]]:
    min_start: Optional[float] = None
    max_end: Optional[float] = None
    total_messages = 0
    per_topic: Dict[str, Dict[str, Any]] = {}

    start = info.get("start")
    end = info.get("end")
    messages = info.get("messages", 0)
    if isinstance(start, (int, float)):
        min_start = float(start)
    if isinstance(end, (int, float)):
        max_end = float(end)
    if isinstance(messages, int):
        total_messages = int(messages)

    topics_raw = info.get("topics", [])
    if isinstance(topics_raw, list):
        for topic_info in topics_raw:
            if not isinstance(topic_info, dict):
                continue
            topic_name = topic_info.get("topic")
            if not topic_name:
                continue
            per_topic[str(topic_name)] = {
                "messages": int(topic_info.get("messages", 0)),
                "type": topic_info.get("type"),
                "bags": 1,
            }
    return min_start, max_end, total_messages, per_topic


def _fill_stats_from_rosbag_python(bag_path: Path) -> Tuple[Optional[float], Optional[float], int, Dict[str, Dict[str, Any]]]:
    try:
        import rosbag  # type: ignore
    except Exception as exc:
        raise RuntimeError(
            "python rosbag module unavailable. "
            "Install ROS python bindings or run inside ROS environment."
        ) from exc

    with rosbag.Bag(str(bag_path), "r") as bag:
        total_messages = int(bag.get_message_count())
        min_start = float(bag.get_start_time()) if total_messages > 0 else None
        max_end = float(bag.get_end_time()) if total_messages > 0 else None

        per_topic: Dict[str, Dict[str, Any]] = {}
        try:
            info = bag.get_type_and_topic_info()
            topics = getattr(info, "topics", None)
            if topics is None and isinstance(info, tuple) and len(info) >= 2:
                topics = info[1]
            if isinstance(topics, dict):
                for topic_name, topic_meta in topics.items():
                    msg_count = int(getattr(topic_meta, "message_count", 0))
                    msg_type = getattr(topic_meta, "msg_type", None)
                    per_topic[str(topic_name)] = {
                        "messages": msg_count,
                        "type": msg_type,
                        "bags": 1,
                    }
        except Exception:
            per_topic = {}
    return min_start, max_end, total_messages, per_topic


def aggregate_single_bag_stats(bag_path: Path) -> Dict[str, Any]:
    total_size = 0
    total_messages = 0
    min_start = None
    max_end = None
    per_topic: Dict[str, Dict[str, Any]] = {}
    parse_warnings: List[str] = []

    try:
        total_size = bag_path.stat().st_size
    except Exception:
        total_size = 0

    rosbag_yaml_loaded = False
    rosbag_cli_errors: List[str] = []
    for rosbag_bin in _candidate_rosbag_bins():
        cmd = [rosbag_bin, "info", "--yaml", str(bag_path)]
        try:
            res = _run_cmd(cmd, timeout=30.0)
        except FileNotFoundError:
            rosbag_cli_errors.append(f"{rosbag_bin}: not found")
            continue
        except Exception as exc:
            rosbag_cli_errors.append(f"{rosbag_bin}: execution error: {exc}")
            continue

        if res.returncode != 0:
            stderr = (res.stderr or "").strip()
            rosbag_cli_errors.append(
                f"{rosbag_bin}: rc={res.returncode} stderr={stderr[:200]}"
            )
            continue

        try:
            info_obj = yaml.safe_load(res.stdout) or {}
            if not isinstance(info_obj, dict):
                raise RuntimeError("yaml root is not object")
        except Exception as exc:
            rosbag_cli_errors.append(f"{rosbag_bin}: yaml parse failed: {exc}")
            continue

        (
            min_start,
            max_end,
            total_messages,
            per_topic,
        ) = _fill_stats_from_rosbag_yaml(info_obj)
        rosbag_yaml_loaded = True
        break

    if not rosbag_yaml_loaded:
        if rosbag_cli_errors:
            parse_warnings.append(
                "rosbag CLI unavailable or failed: " + " | ".join(rosbag_cli_errors)
            )
        try:
            (
                min_start,
                max_end,
                total_messages,
                per_topic,
            ) = _fill_stats_from_rosbag_python(bag_path)
        except Exception as exc:
            raise RuntimeError(
                "Cannot collect bag_stats for strict-format metadata. "
                "Neither rosbag CLI nor python rosbag runtime worked. "
                f"bag={bag_path} error={exc}"
            ) from exc

    duration = None
    if min_start is not None and max_end is not None:
        duration = float(max_end) - float(min_start)

    return {
        "bag_files": [str(bag_path)],
        "bag_count": 1,
        "total_size_bytes": total_size,
        "total_messages": total_messages,
        "start_time": min_start,
        "end_time": max_end,
        "duration_sec": duration,
        "per_topic": per_topic,
        "parse_warnings": parse_warnings,
    }


def normalize_recording(recording_src: Any, *, force_single_bag: bool) -> Dict[str, Any]:
    src = recording_src if isinstance(recording_src, dict) else {}
    split_enabled = False if force_single_bag else _to_bool(src.get("split_enabled"), False)
    split_size_mb = None if not split_enabled else src.get("split_size_mb")
    return {
        "compression": src.get("compression", "lz4"),
        "split_enabled": split_enabled,
        "split_size_mb": split_size_mb,
        "camera_transport": src.get("camera_transport"),
        "keep_debug_logs": _to_bool(src.get("keep_debug_logs"), False),
    }


def build_standard_metadata(
    *,
    source_meta: Optional[Mapping[str, Any]],
    session_name: str,
    episode_name: str,
    episode_index: int,
    episode_dir: Path,
    bag_stats: Mapping[str, Any],
    started_at_utc: Optional[str] = None,
    ended_at_utc: Optional[str] = None,
    duration_sec: Optional[float] = None,
    stop_reason: Optional[str] = None,
    force_single_bag: bool = True,
) -> Dict[str, Any]:
    src = source_meta if isinstance(source_meta, Mapping) else {}
    final_duration = duration_sec
    if final_duration is None:
        bag_duration = _to_float_or_none(bag_stats.get("duration_sec"))
        final_duration = bag_duration if bag_duration is not None else _to_float_or_none(src.get("duration_sec"))

    final_stop_reason = stop_reason if stop_reason is not None else src.get("stop_reason")
    final_started = started_at_utc if started_at_utc is not None else src.get("started_at_utc")
    final_ended = ended_at_utc if ended_at_utc is not None else src.get("ended_at_utc")

    metadata = {
        "schema_version": _to_int(src.get("schema_version"), 1),
        "session_name": str(session_name),
        "episode_name": str(episode_name),
        "episode_index": int(episode_index),
        "episode_dir": str(episode_dir),
        "started_at_utc": final_started,
        "ended_at_utc": final_ended,
        "duration_sec": final_duration,
        "stop_reason": final_stop_reason,
        "recording": normalize_recording(src.get("recording"), force_single_bag=force_single_bag),
        "topics": _deepcopy_dict(src.get("topics")),
        "preflight": _deepcopy_dict(src.get("preflight")),
        "hz_monitor": _deepcopy_dict(src.get("hz_monitor")),
        "bag_stats": copy.deepcopy(dict(bag_stats)),
        "environment": _deepcopy_dict(src.get("environment")),
        "notes": src.get("notes", ""),
        "debug_logs": _deepcopy_dict(src.get("debug_logs")),
        "warnings": _deepcopy_list(src.get("warnings")),
        "rosbag_process_terminated_cleanly": _to_bool(
            src.get("rosbag_process_terminated_cleanly"), True
        ),
        "save_job_id": _to_int(src.get("save_job_id"), 1),
    }
    return metadata


def collect_standard_validation_errors(
    *,
    episode_dir: Path,
    expected_episode_name: str,
    expected_episode_index: int,
) -> List[str]:
    errors: List[str] = []
    bag_path = episode_dir / "episode.bag"
    meta_path = episode_dir / "metadata.json"

    if not bag_path.is_file():
        errors.append(f"missing episode bag: {bag_path}")
    if not meta_path.is_file():
        errors.append(f"missing metadata file: {meta_path}")
        return errors

    try:
        metadata = load_json_dict(meta_path)
    except Exception as exc:
        errors.append(f"invalid metadata json: {exc}")
        return errors

    top_keys = set(metadata.keys())
    expected_top_keys = set(STANDARD_METADATA_KEYS)
    if top_keys != expected_top_keys:
        missing = sorted(expected_top_keys - top_keys)
        extra = sorted(top_keys - expected_top_keys)
        errors.append(
            "metadata top-level keys mismatch: "
            f"missing={missing} extra={extra}"
        )

    recording = metadata.get("recording")
    if not isinstance(recording, dict):
        errors.append("metadata.recording must be an object")
    else:
        rec_keys = set(recording.keys())
        expected_rec_keys = set(STANDARD_RECORDING_KEYS)
        if rec_keys != expected_rec_keys:
            missing = sorted(expected_rec_keys - rec_keys)
            extra = sorted(rec_keys - expected_rec_keys)
            errors.append(
                "metadata.recording keys mismatch: "
                f"missing={missing} extra={extra}"
            )

    actual_name = metadata.get("episode_name")
    if actual_name != expected_episode_name:
        errors.append(
            f"episode_name mismatch: expected={expected_episode_name} actual={actual_name}"
        )

    actual_index = metadata.get("episode_index")
    try:
        actual_index_int = int(actual_index)
    except Exception:
        actual_index_int = None
    if actual_index_int != int(expected_episode_index):
        errors.append(
            f"episode_index mismatch: expected={expected_episode_index} actual={actual_index}"
        )

    actual_episode_dir = metadata.get("episode_dir")
    actual_dir_path: Optional[Path] = None
    try:
        actual_dir_path = Path(str(actual_episode_dir)).resolve()
    except Exception:
        actual_dir_path = None
    if actual_dir_path != episode_dir.resolve():
        errors.append(
            "episode_dir mismatch: "
            f"expected={episode_dir.resolve()} actual={actual_episode_dir}"
        )

    return errors


def validate_episode_or_raise(
    *,
    episode_dir: Path,
    expected_episode_name: str,
    expected_episode_index: int,
    strict: bool,
) -> None:
    errors = collect_standard_validation_errors(
        episode_dir=episode_dir,
        expected_episode_name=expected_episode_name,
        expected_episode_index=expected_episode_index,
    )
    if errors and strict:
        detail = "\n".join(f"- {line}" for line in errors)
        raise RuntimeError(
            f"Episode validation failed for {episode_dir}:\n{detail}"
        )
    if errors:
        print(f"[warn] validation issues for {episode_dir}:")
        for line in errors:
            print(f"[warn]   {line}")


def ensure_all_episodes_are_standard(
    *,
    episode_dirs: Iterable[Path],
    strict: bool,
) -> None:
    for ep_dir in episode_dirs:
        expected_index = parse_episode_num(ep_dir.name)
        validate_episode_or_raise(
            episode_dir=ep_dir,
            expected_episode_name=ep_dir.name,
            expected_episode_index=expected_index,
            strict=strict,
        )
