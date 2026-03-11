#!/usr/bin/env python3
"""Interactive keyboard switch for /robot/arm_opp master source.

Keys:
  1 : follow /teleop/arm_left/joint_states_single
  2 : follow /teleop/arm_right/joint_states_single
  SPACE : start/stop recording episode
  h : hold current /robot/arm_opp/joint_states_single
  s : print current status
  q : quit
"""

from __future__ import annotations

import argparse
import ast
import datetime as dt
import json
import re
import select
import shutil
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rosbag
import rosgraph
import roslib.message
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from topic_tools.srv import MuxSelect
from piper_msgs.srv import Enable

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
DEFAULT_RECORD_PROFILE = "/home/jameszhao2004/catkin_ws/workspaces/config/rosbag_profiles/act_rgb_3arm_profile.yaml"
DEFAULT_RECORD_ROOT = "/home/jameszhao2004/catkin_ws/data/rosbags"
DEFAULT_RECORD_OPP_JOINT_STATE_TOPIC = "/robot/arm_opp/joint_states_single"
DEFAULT_RECORD_OPP_EFFORT_TOPIC = "/robot/arm_opp/joint_states_compensated"


class RawTerminal:
    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise RuntimeError("Interactive TTY is required.")
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


def _parse_float_vector(text: str, *, expected_len: int, arg_name: str) -> List[float]:
    raw = text.strip()
    if not raw:
        raise ValueError(f"{arg_name} must not be empty")
    try:
        if raw.startswith("["):
            values = ast.literal_eval(raw)
        else:
            values = [part.strip() for part in raw.split(",")]
    except Exception as exc:
        raise ValueError(f"{arg_name} parse failed: {exc}") from exc
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"{arg_name} must be a list/tuple or comma-separated values")
    try:
        out = [float(v) for v in values]
    except Exception as exc:
        raise ValueError(f"{arg_name} contains non-numeric values: {exc}") from exc
    if len(out) != expected_len:
        raise ValueError(f"{arg_name} length must be {expected_len}, got {len(out)}")
    return out


def _clamp(v: float, lo: float, hi: float) -> float:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def _local_now_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def _dedup_keep_order(items: List[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


class InProcessEpisodeRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        if yaml is None:
            raise RuntimeError("PyYAML is required for recording profile parsing (python3-yaml).")

        self.profile = self._load_profile(args.record_profile)
        if args.record_camera_transport != "profile":
            self.profile["camera_transport"] = args.record_camera_transport
        self._validate_profile(self.profile)

        self.camera_transport = str(self.profile["camera_transport"])
        self.required_topics = list(self.profile.get("required_topics", []))
        self.camera_streams = list(self.profile.get("camera_streams", []))
        self.optional_topics = list(self.profile.get("optional_topics", []))
        default_required_joint_topics = [
            "/robot/arm_left/joint_states_single",
            "/robot/arm_right/joint_states_single",
            "/robot/arm_opp/joint_states_single",
            "/teleop/arm_left/joint_states_single",
            "/teleop/arm_right/joint_states_single",
        ]
        self.required_joint_topics = list(self.profile.get("required_joint_topics", default_required_joint_topics))

        self.camera_record_topics = self._build_camera_record_topics()
        self.camera_health_topics = [entry["image_base"] for entry in self.camera_streams]
        self.camera_info_topics = [entry["camera_info"] for entry in self.camera_streams]
        self.required_presence_topics = self._build_required_presence_topics()
        self.profile_record_topics = _dedup_keep_order(self.required_topics + self.camera_record_topics)

        self.session_root = Path(args.record_session_root).expanduser().resolve()
        self.session_name = args.record_session_name or f"{args.record_prefix}_{_local_now_tag()}"
        self.session_dir = self.session_root / self.session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.next_episode_index = self._scan_next_episode_index()

        self.recording = False
        self.bag: Optional[rosbag.Bag] = None
        self.subscribers: Dict[str, rospy.Subscriber] = {}
        self.msg_type_map: Dict[str, str] = {}
        self.topic_counts: Dict[str, int] = {}
        self.effort_nonempty_counts: Dict[str, int] = {}
        self.current_topics: List[str] = []
        self.current_optional_recorded: List[str] = []
        self.current_optional_missing: List[str] = []
        self.current_preflight: Dict[str, object] = {}
        self.current_warnings: List[str] = []

        self.current_episode_name: Optional[str] = None
        self.current_episode_dir: Optional[Path] = None
        self.current_bag_path: Optional[Path] = None
        self.current_started_utc: Optional[str] = None
        self.current_started_mono: Optional[float] = None
        self.current_started_local: Optional[str] = None
        self.required_opp_topics = [
            str(self.args.record_opp_joint_state_topic).strip(),
            str(self.args.record_opp_effort_topic).strip(),
        ]

        self.bag_lock = threading.Lock()
        self.last_toggle_ts = 0.0
        self.debounce_sec = max(0.0, float(args.record_debounce_ms) / 1000.0)
        self.last_status_ts = 0.0
        self.last_disk_check_ts = 0.0

        self.key_rosparams = [
            "/piper_ctrl_left_node/can_port",
            "/piper_ctrl_right_node/can_port",
            "/piper_ctrl_opp_node/can_port",
            "/piper_teleop_left_node/can_port",
            "/piper_teleop_right_node/can_port",
            "/piper_teleop_left_node/gripper_reverse",
            "/piper_teleop_right_node/gripper_reverse",
            "/piper_teleop_left_node/mit/enable_tor",
            "/piper_teleop_right_node/mit/enable_tor",
            "/piper_teleop_left_node/mit/gravity_mix_mode",
            "/piper_teleop_right_node/mit/gravity_mix_mode",
            "/piper_gravity_compensation_node/enable_opp_arm",
        ]

    def _load_profile(self, path: str) -> dict:
        profile_path = Path(path).expanduser().resolve()
        if not profile_path.exists():
            raise RuntimeError(f"Recording profile not found: {profile_path}")
        with profile_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        data["_profile_path"] = str(profile_path)
        return data

    def _validate_profile(self, profile: dict) -> None:
        required_keys = ["profile_name", "camera_transport", "required_topics", "camera_streams"]
        for key in required_keys:
            if key not in profile:
                raise RuntimeError(f"Profile missing key: {key}")
        if profile["camera_transport"] not in ("compressed", "raw"):
            raise RuntimeError("camera_transport must be 'compressed' or 'raw'")
        if not isinstance(profile.get("required_topics", []), list):
            raise RuntimeError("required_topics must be a list")
        if not isinstance(profile.get("camera_streams", []), list):
            raise RuntimeError("camera_streams must be a list")
        for entry in profile["camera_streams"]:
            if "image_base" not in entry or "camera_info" not in entry:
                raise RuntimeError("camera_streams entries must include image_base and camera_info")
        if "required_joint_topics" in profile and not isinstance(profile["required_joint_topics"], list):
            raise RuntimeError("required_joint_topics must be a list when provided")

    def _build_camera_record_topics(self) -> List[str]:
        topics: List[str] = []
        for entry in self.camera_streams:
            image_base = entry["image_base"]
            camera_info = entry["camera_info"]
            if self.camera_transport == "compressed":
                topics.append(f"{image_base}/compressed")
            else:
                topics.append(image_base)
            topics.append(camera_info)
        return _dedup_keep_order(topics)

    def _build_required_presence_topics(self) -> List[str]:
        topics = list(self.required_topics) + list(self.camera_info_topics)
        if self.camera_transport == "raw":
            topics.extend(self.camera_health_topics)
        return _dedup_keep_order(topics)

    def _scan_next_episode_index(self) -> int:
        used = set()
        for entry in self.session_dir.iterdir():
            if not entry.is_dir():
                continue
            match = re.match(r"^episode_(\d+)$", entry.name)
            if match:
                used.add(int(match.group(1)))
        idx = 1
        while idx in used:
            idx += 1
        return idx

    def _ros_master_ok(self) -> Tuple[bool, str]:
        try:
            master = rosgraph.Master(rospy.get_name())
            master.getUri()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def _query_topics(self) -> Tuple[Dict[str, str], Dict[str, int]]:
        master = rosgraph.Master(rospy.get_name())
        published_pairs = master.getPublishedTopics("/")
        topic_types = {name: msg_type for name, msg_type in published_pairs}
        system_state = master.getSystemState()
        publishers_state = system_state[0] if len(system_state) >= 1 else []
        pub_count: Dict[str, int] = {}
        for topic, nodes in publishers_state:
            pub_count[topic] = len(nodes)
        return topic_types, pub_count

    def _check_disk_ok(self) -> Tuple[bool, float]:
        usage = shutil.disk_usage(str(self.session_root))
        free_gb = float(usage.free) / (1024.0 ** 3)
        return free_gb >= float(self.args.record_min_free_gb), free_gb

    def _fetch_rosparams(self) -> Dict[str, object]:
        snapshot: Dict[str, object] = {}
        for key in self.key_rosparams:
            try:
                if rospy.has_param(key):
                    snapshot[key] = rospy.get_param(key)
                else:
                    snapshot[key] = "<unset>"
            except Exception as exc:
                snapshot[key] = f"<error:{exc}>"
        return snapshot

    def _wait_preflight(self, timeout_sec: float) -> Tuple[bool, Dict[str, object]]:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        last_missing_presence: List[str] = []
        last_joint_no_pub: List[str] = []
        last_camera_unhealthy: List[str] = []
        last_optional_present: List[str] = []
        last_optional_missing: List[str] = []

        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                topic_types, pub_count = self._query_topics()
            except Exception:
                time.sleep(0.2)
                continue

            topics = set(topic_types.keys())
            missing_presence = sorted([t for t in self.required_presence_topics if t not in topics])

            joint_no_pub = []
            for topic in self.required_joint_topics:
                if topic in topics and pub_count.get(topic, 0) <= 0:
                    joint_no_pub.append(topic)

            camera_unhealthy = []
            if self.camera_transport == "raw":
                for topic in self.camera_health_topics:
                    if topic in topics and pub_count.get(topic, 0) <= 0:
                        camera_unhealthy.append(topic)

            optional_present = sorted([t for t in self.optional_topics if t in topics])
            optional_missing = sorted([t for t in self.optional_topics if t not in topics])

            last_missing_presence = missing_presence
            last_joint_no_pub = joint_no_pub
            last_camera_unhealthy = camera_unhealthy
            last_optional_present = optional_present
            last_optional_missing = optional_missing

            if not missing_presence and not joint_no_pub and not camera_unhealthy:
                return True, {
                    "status": "ok",
                    "checked_utc": _utc_now_iso(),
                    "camera_transport": self.camera_transport,
                    "required_presence_topics": list(self.required_presence_topics),
                    "required_joint_topics": list(self.required_joint_topics),
                    "camera_health_topics": list(self.camera_health_topics),
                    "optional_present": optional_present,
                    "optional_missing": optional_missing,
                }
            time.sleep(0.2)

        return False, {
            "status": "timeout",
            "checked_utc": _utc_now_iso(),
            "reason": "preflight_timeout",
            "camera_transport": self.camera_transport,
            "required_presence_topics": list(self.required_presence_topics),
            "required_joint_topics": list(self.required_joint_topics),
            "camera_health_topics": list(self.camera_health_topics),
            "last_missing_presence_topics": last_missing_presence,
            "last_joint_no_publisher_topics": last_joint_no_pub,
            "last_camera_unhealthy_topics": last_camera_unhealthy,
            "optional_present": last_optional_present,
            "optional_missing": last_optional_missing,
        }

    def _resolve_message_class(self, topic: str) -> Optional[type]:
        msg_type = self.msg_type_map.get(topic, "")
        if not msg_type:
            return None
        return roslib.message.get_message_class(msg_type)

    def _topic_callback(self, topic: str, msg) -> None:
        if not self.recording:
            return
        stamp = rospy.Time.now()
        header = getattr(msg, "header", None)
        if header is not None:
            msg_stamp = getattr(header, "stamp", None)
            if isinstance(msg_stamp, rospy.Time) and msg_stamp.to_sec() > 0.0:
                stamp = msg_stamp
        try:
            with self.bag_lock:
                if not self.recording or self.bag is None:
                    return
                self.bag.write(topic, msg, t=stamp)
                self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1
                if isinstance(msg, JointState) and len(getattr(msg, "effort", [])) > 0:
                    self.effort_nonempty_counts[topic] = self.effort_nonempty_counts.get(topic, 0) + 1
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[record] bag write failed topic=%s err=%s", topic, str(exc))

    def _create_subscribers(self) -> None:
        for topic in self.current_topics:
            msg_cls = self._resolve_message_class(topic)
            if msg_cls is None:
                raise RuntimeError(f"Cannot resolve message type for topic: {topic}")
            sub = rospy.Subscriber(
                topic,
                msg_cls,
                lambda msg, t=topic: self._topic_callback(t, msg),
                queue_size=max(1, int(self.args.record_sub_queue_size)),
                tcp_nodelay=True,
            )
            self.subscribers[topic] = sub

    def _clear_subscribers(self) -> None:
        for sub in self.subscribers.values():
            try:
                sub.unregister()
            except Exception:
                pass
        self.subscribers = {}

    def _open_bag(self, bag_path: Path) -> rosbag.Bag:
        compression = str(self.args.record_bag_compression)
        if compression == "none":
            return rosbag.Bag(str(bag_path), mode="w")
        try:
            return rosbag.Bag(str(bag_path), mode="w", compression=compression)
        except Exception as exc:
            rospy.logwarn(
                "[record] Failed to open bag with compression=%s (%s). Falling back to none.",
                compression,
                str(exc),
            )
            return rosbag.Bag(str(bag_path), mode="w")

    def _next_episode_slot(self) -> Tuple[int, str, Path]:
        idx = self._scan_next_episode_index()
        while True:
            name = f"episode_{idx:03d}"
            path = self.session_dir / name
            if not path.exists():
                return idx, name, path
            idx += 1

    def start_episode(self) -> None:
        if self.recording:
            rospy.logwarn("[record] already recording")
            return

        ok_master, master_msg = self._ros_master_ok()
        if not ok_master:
            rospy.logerr("[record] ROS master unavailable: %s", master_msg)
            return
        disk_ok, free_gb = self._check_disk_ok()
        if not disk_ok:
            rospy.logerr(
                "[record] Disk free too low: %.2f GB < %.2f GB",
                free_gb,
                float(self.args.record_min_free_gb),
            )
            return

        preflight_ok, preflight = self._wait_preflight(float(self.args.record_wait_timeout_sec))
        if not preflight_ok:
            rospy.logerr("[record] Preflight failed: %s", preflight.get("reason", "unknown"))
            rospy.logerr(
                "[record] missing=%s no_pub=%s camera=%s",
                preflight.get("last_missing_presence_topics", [])[:5],
                preflight.get("last_joint_no_publisher_topics", [])[:5],
                preflight.get("last_camera_unhealthy_topics", [])[:5],
            )
            return

        idx, episode_name, episode_dir = self._next_episode_slot()
        episode_dir.mkdir(parents=True, exist_ok=False)
        self.current_episode_name = episode_name
        self.current_episode_dir = episode_dir
        self.current_bag_path = episode_dir / "episode.bag"
        self.current_started_utc = _utc_now_iso()
        self.current_started_local = dt.datetime.now().isoformat(timespec="seconds")
        self.current_started_mono = time.monotonic()
        self.current_preflight = preflight
        self.current_warnings = []
        self.current_optional_recorded = list(preflight.get("optional_present", []))
        self.current_optional_missing = list(preflight.get("optional_missing", []))
        self.current_topics = _dedup_keep_order(
            self.profile_record_topics + self.current_optional_recorded + self.required_opp_topics
        )

        try:
            topic_types, pub_count = self._query_topics()
            self.msg_type_map = dict(topic_types)
            missing_opp_type = [t for t in self.required_opp_topics if t not in self.msg_type_map]
            if missing_opp_type:
                raise RuntimeError(f"Required third-arm topics are missing: {missing_opp_type}")
            no_pub_opp = [t for t in self.required_opp_topics if pub_count.get(t, 0) <= 0]
            if no_pub_opp:
                raise RuntimeError(f"Required third-arm topics have no publishers: {no_pub_opp}")
            self.topic_counts = {topic: 0 for topic in self.current_topics}
            self.effort_nonempty_counts = {topic: 0 for topic in self.current_topics}
            self.bag = self._open_bag(self.current_bag_path)
            self._create_subscribers()
            self.recording = True
            self.last_status_ts = 0.0
            self.next_episode_index = idx
        except Exception as exc:
            self._clear_subscribers()
            if self.bag is not None:
                try:
                    self.bag.close()
                except Exception:
                    pass
            self.bag = None
            self.recording = False
            self.current_warnings.append(f"Recorder start failure: {exc}")
            rospy.logerr("[record] start failed: %s", str(exc))
            return

        rospy.loginfo("[record] START %s", episode_name)
        rospy.loginfo("[record] episode_path=%s", str(episode_dir))
        rospy.loginfo("[record] bag_path=%s", str(self.current_bag_path))
        rospy.loginfo(
            "[record] topics=%d optional=%d camera_transport=%s",
            len(self.current_topics),
            len(self.current_optional_recorded),
            self.camera_transport,
        )

    def stop_episode(self, reason: str) -> None:
        if not self.recording:
            return

        episode_name = self.current_episode_name or f"episode_{self.next_episode_index:03d}"
        episode_dir = self.current_episode_dir
        bag_path = self.current_bag_path
        started_utc = self.current_started_utc
        started_local = self.current_started_local
        started_mono = self.current_started_mono

        self.recording = False
        self._clear_subscribers()
        with self.bag_lock:
            if self.bag is not None:
                try:
                    self.bag.close()
                except Exception as exc:
                    self.current_warnings.append(f"bag close failed: {exc}")
                self.bag = None

        ended_utc = _utc_now_iso()
        ended_local = dt.datetime.now().isoformat(timespec="seconds")
        duration_sec = None
        if started_mono is not None:
            duration_sec = max(0.0, time.monotonic() - started_mono)

        if episode_dir is not None:
            metadata = {
                "saved_utc": ended_utc,
                "session_name": self.session_name,
                "session_dir": str(self.session_dir),
                "episode_name": episode_name,
                "episode_index": int(episode_name.split("_")[-1]) if "_" in episode_name else -1,
                "episode_dir": str(episode_dir),
                "bag_path": str(bag_path) if bag_path is not None else None,
                "recording": {
                    "in_process_writer": True,
                    "bag_compression": str(self.args.record_bag_compression),
                    "camera_transport": self.camera_transport,
                    "stop_reason": reason,
                    "start_utc": started_utc,
                    "start_local": started_local,
                    "stop_utc": ended_utc,
                    "stop_local": ended_local,
                    "duration_sec": duration_sec,
                },
                "profile": {
                    "profile_name": self.profile.get("profile_name"),
                    "profile_path": self.profile.get("_profile_path"),
                },
                "topics": {
                    "profile_required": list(self.required_topics),
                    "required_joint_topics": list(self.required_joint_topics),
                    "profile_camera_record_topics": list(self.camera_record_topics),
                    "recorded_topics": list(self.current_topics),
                    "optional_recorded": list(self.current_optional_recorded),
                    "optional_missing": list(self.current_optional_missing),
                    "message_count_by_topic": dict(self.topic_counts),
                    "effort_nonempty_count_by_topic": dict(self.effort_nonempty_counts),
                    "required_third_arm_topics": list(self.required_opp_topics),
                },
                "preflight": dict(self.current_preflight),
                "rosparams_snapshot": self._fetch_rosparams(),
                "warnings": list(self.current_warnings),
            }

            if self.camera_transport == "compressed":
                counts = metadata["topics"]["message_count_by_topic"]
                for entry in self.camera_streams:
                    compressed_topic = f"{entry['image_base']}/compressed"
                    if int(counts.get(compressed_topic, 0)) <= 0:
                        metadata["warnings"].append(f"Compressed stream has zero messages: {compressed_topic}")

            counts = metadata["topics"]["message_count_by_topic"]
            for topic in self.required_topics:
                if int(counts.get(topic, 0)) <= 0:
                    metadata["warnings"].append(f"Required topic has zero messages: {topic}")

            for topic in self.required_joint_topics:
                if int(counts.get(topic, 0)) <= 0:
                    metadata["warnings"].append(f"Required joint topic has zero messages: {topic}")

            effort_counts = metadata["topics"]["effort_nonempty_count_by_topic"]
            for topic in self.required_opp_topics:
                if topic and topic in self.current_topics and int(counts.get(topic, 0)) <= 0:
                    metadata["warnings"].append(f"Third-arm topic has zero messages: {topic}")
                if topic and topic in self.current_topics and int(effort_counts.get(topic, 0)) <= 0:
                    metadata["warnings"].append(f"Third-arm effort appears empty on topic: {topic}")

            metadata_path = episode_dir / "metadata.json"
            try:
                metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=True), encoding="utf-8")
            except Exception as exc:
                rospy.logerr("[record] failed writing metadata: %s", str(exc))

        self.next_episode_index = self._scan_next_episode_index()
        rospy.loginfo("[record] STOP %s reason=%s duration=%.2fs", episode_name, reason, duration_sec or 0.0)
        rospy.loginfo("[record] next=episode_%03d", self.next_episode_index)

        self.current_episode_name = None
        self.current_episode_dir = None
        self.current_bag_path = None
        self.current_started_utc = None
        self.current_started_local = None
        self.current_started_mono = None
        self.current_topics = []
        self.current_optional_recorded = []
        self.current_optional_missing = []
        self.current_preflight = {}
        self.current_warnings = []
        self.msg_type_map = {}
        self.effort_nonempty_counts = {}

    def toggle_record(self) -> None:
        now = time.monotonic()
        if (now - self.last_toggle_ts) < self.debounce_sec:
            return
        self.last_toggle_ts = now
        if self.recording:
            self.stop_episode("user_stop")
        else:
            self.start_episode()

    def tick(self) -> None:
        if not self.recording:
            return
        now = time.monotonic()
        if (now - self.last_status_ts) >= max(0.1, float(self.args.record_status_interval_sec)):
            elapsed = 0.0
            if self.current_started_mono is not None:
                elapsed = max(0.0, now - self.current_started_mono)
            total_msgs = sum(int(v) for v in self.topic_counts.values())
            rospy.loginfo_throttle(
                2.0,
                "[record] ACTIVE %s elapsed=%.1fs total_msgs=%d topics=%d",
                self.current_episode_name,
                elapsed,
                total_msgs,
                len(self.current_topics),
            )
            self.last_status_ts = now

        if (now - self.last_disk_check_ts) >= 2.0:
            self.last_disk_check_ts = now
            disk_ok, free_gb = self._check_disk_ok()
            if not disk_ok:
                rospy.logerr(
                    "[record] Disk free dropped below threshold: %.2f GB < %.2f GB",
                    free_gb,
                    float(self.args.record_min_free_gb),
                )
                self.stop_episode("disk_low")

    def shutdown(self) -> None:
        if self.recording:
            self.stop_episode("shutdown")


class OppMasterSwitcher:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.recorder: Optional[InProcessEpisodeRecorder] = None
        self.gravity_feedback_auto_disabled = False
        self.latest_joint: Optional[JointState] = None
        self.latest_selected: Optional[str] = None
        self.latest_left_master_joint: Optional[JointState] = None
        self.latest_right_master_joint: Optional[JointState] = None
        self.latest_left_slave_joint: Optional[JointState] = None
        self.latest_right_slave_joint: Optional[JointState] = None
        self.latest_left_torque_selected: Optional[str] = None
        self.latest_right_torque_selected: Optional[str] = None
        self.active_master_side: Optional[str] = None
        self.latest_opp_msg_monotonic: Optional[float] = None
        self.last_stale_zero_publish_monotonic: Optional[float] = None
        if self.args.record_enable:
            self.recorder = InProcessEpisodeRecorder(self.args)
            rospy.loginfo(
                "In-process recorder enabled: session_dir=%s profile=%s",
                str(self.recorder.session_dir),
                str(self.recorder.profile.get("_profile_path", self.args.record_profile)),
            )

        left_sign = _parse_float_vector(
            args.opp_torque_sign_left,
            expected_len=6,
            arg_name="--opp-torque-sign-left",
        )
        right_sign = _parse_float_vector(
            args.opp_torque_sign_right,
            expected_len=6,
            arg_name="--opp-torque-sign-right",
        )
        if args.invert_opp_torque_left:
            left_sign = [-v for v in left_sign]
        if args.invert_opp_torque_right:
            right_sign = [-v for v in right_sign]
        self.left_opp_torque_sign = left_sign
        self.right_opp_torque_sign = right_sign
        self.left_opp_torque_scale = _parse_float_vector(
            args.opp_torque_scale_left,
            expected_len=6,
            arg_name="--opp-torque-scale-left",
        )
        self.right_opp_torque_scale = _parse_float_vector(
            args.opp_torque_scale_right,
            expected_len=6,
            arg_name="--opp-torque-scale-right",
        )
        self.left_opp_torque_max = _parse_float_vector(
            args.opp_torque_max_left,
            expected_len=6,
            arg_name="--opp-torque-max-left",
        )
        self.right_opp_torque_max = _parse_float_vector(
            args.opp_torque_max_right,
            expected_len=6,
            arg_name="--opp-torque-max-right",
        )
        self.left_opp_torque_deadband = _parse_float_vector(
            args.opp_torque_deadband_left,
            expected_len=6,
            arg_name="--opp-torque-deadband-left",
        )
        self.right_opp_torque_deadband = _parse_float_vector(
            args.opp_torque_deadband_right,
            expected_len=6,
            arg_name="--opp-torque-deadband-right",
        )
        self.left_opp_torque_rate_limit = _parse_float_vector(
            args.opp_torque_rate_limit_left,
            expected_len=6,
            arg_name="--opp-torque-rate-limit-left",
        )
        self.right_opp_torque_rate_limit = _parse_float_vector(
            args.opp_torque_rate_limit_right,
            expected_len=6,
            arg_name="--opp-torque-rate-limit-right",
        )
        self.left_opp_torque_alpha = _clamp(float(args.opp_torque_alpha_left), 0.0, 1.0)
        self.right_opp_torque_alpha = _clamp(float(args.opp_torque_alpha_right), 0.0, 1.0)
        self.opp_torque_ramp_sec = max(0.0, float(args.opp_torque_ramp_sec))
        self.left_opp_torque_filtered = [0.0] * 6
        self.right_opp_torque_filtered = [0.0] * 6
        self.left_attach_started_monotonic: Optional[float] = None
        self.right_attach_started_monotonic: Optional[float] = None

        self.hold_pub = rospy.Publisher(args.hold_topic, JointState, queue_size=1, latch=True)
        self.enable_flag_pub = rospy.Publisher(args.enable_flag_topic, Bool, queue_size=1, latch=True)
        self.left_opp_torque_pub = rospy.Publisher(args.left_opp_torque_topic, JointState, queue_size=1)
        self.right_opp_torque_pub = rospy.Publisher(args.right_opp_torque_topic, JointState, queue_size=1)
        self.left_align_pub = rospy.Publisher(args.left_master_align_topic, JointState, queue_size=1)
        self.right_align_pub = rospy.Publisher(args.right_master_align_topic, JointState, queue_size=1)
        self.mux_select = rospy.ServiceProxy(args.mux_service, MuxSelect)
        self.enable_srv = rospy.ServiceProxy(args.enable_service, Enable)
        self.block_srv = rospy.ServiceProxy(args.block_service, SetBool)
        self.left_slave_mux = rospy.ServiceProxy(args.left_slave_mux_service, MuxSelect)
        self.right_slave_mux = rospy.ServiceProxy(args.right_slave_mux_service, MuxSelect)
        self.left_torque_mux = rospy.ServiceProxy(args.left_torque_mux_service, MuxSelect)
        self.right_torque_mux = rospy.ServiceProxy(args.right_torque_mux_service, MuxSelect)
        self.left_torque_mux_fallback = None
        self.right_torque_mux_fallback = None
        if args.left_torque_mux_service_fallback and args.left_torque_mux_service_fallback != args.left_torque_mux_service:
            self.left_torque_mux_fallback = rospy.ServiceProxy(args.left_torque_mux_service_fallback, MuxSelect)
        if args.right_torque_mux_service_fallback and args.right_torque_mux_service_fallback != args.right_torque_mux_service:
            self.right_torque_mux_fallback = rospy.ServiceProxy(args.right_torque_mux_service_fallback, MuxSelect)
        self.left_gravity_mux = rospy.ServiceProxy(args.left_gravity_mux_service, MuxSelect)
        self.right_gravity_mux = rospy.ServiceProxy(args.right_gravity_mux_service, MuxSelect)
        self.left_master_cmd_mux = rospy.ServiceProxy(args.left_master_cmd_mux_service, MuxSelect)
        self.right_master_cmd_mux = rospy.ServiceProxy(args.right_master_cmd_mux_service, MuxSelect)
        self.left_master_enable_srv = rospy.ServiceProxy(args.left_master_enable_service, Enable)
        self.right_master_enable_srv = rospy.ServiceProxy(args.right_master_enable_service, Enable)
        self.left_master_block_srv = rospy.ServiceProxy(args.left_master_block_service, SetBool)
        self.right_master_block_srv = rospy.ServiceProxy(args.right_master_block_service, SetBool)
        self.slave_follow_flag_pub = rospy.Publisher(args.slave_follow_flag_topic, Bool, queue_size=1, latch=True)
        self.left_master_force_cmd_pub = rospy.Publisher(args.left_master_force_cmd_topic, Bool, queue_size=1, latch=True)
        self.right_master_force_cmd_pub = rospy.Publisher(args.right_master_force_cmd_topic, Bool, queue_size=1, latch=True)

        rospy.Subscriber(args.opp_state_topic, JointState, self._cb_opp_joint, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(args.selected_topic, String, self._cb_selected, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(args.left_master_topic, JointState, self._cb_left_master_joint, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            args.right_master_topic,
            JointState,
            self._cb_right_master_joint,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(args.left_slave_state_topic, JointState, self._cb_left_slave_joint, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            args.right_slave_state_topic,
            JointState,
            self._cb_right_slave_joint,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            args.left_torque_selected_topic,
            String,
            self._cb_left_torque_selected,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            args.right_torque_selected_topic,
            String,
            self._cb_right_torque_selected,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "OPP torque shaping left: sign=%s scale=%s max=%s deadband=%s rate_limit=%s alpha=%.2f",
            self.left_opp_torque_sign,
            self.left_opp_torque_scale,
            self.left_opp_torque_max,
            self.left_opp_torque_deadband,
            self.left_opp_torque_rate_limit,
            self.left_opp_torque_alpha,
        )
        rospy.loginfo(
            "OPP torque shaping right: sign=%s scale=%s max=%s deadband=%s rate_limit=%s alpha=%.2f",
            self.right_opp_torque_sign,
            self.right_opp_torque_scale,
            self.right_opp_torque_max,
            self.right_opp_torque_deadband,
            self.right_opp_torque_rate_limit,
            self.right_opp_torque_alpha,
        )
        rospy.loginfo(
            "OPP torque ramp=%.2fs stale_timeout=%.3fs pass_through_gripper_effort=%s",
            self.opp_torque_ramp_sec,
            self.args.opp_torque_stale_sec,
            self.args.opp_pass_through_gripper_effort,
        )
        rospy.loginfo(
            "Gravity switch guard: freeze=%s settle=%.3fs calm_mode=%s calm_min=%.3fs calm_max=%.3fs calm_consecutive=%d effort_step_max=%.4f pos_step_max=%.4f",
            self.args.gravity_switch_freeze_mode,
            self.args.gravity_switch_settle_sec,
            self.args.gravity_switch_calm_mode,
            self.args.gravity_switch_calm_min_sec,
            self.args.gravity_switch_calm_max_sec,
            self.args.gravity_switch_calm_consecutive,
            self.args.gravity_switch_calm_effort_step_max,
            self.args.gravity_switch_calm_pos_step_max,
        )
        rospy.loginfo(
            "Gravity align-hold: enabled=%s rate=%.1fHz pre_hold=%.3fs",
            self.args.gravity_switch_align_lock_mode,
            self.args.gravity_switch_align_lock_rate_hz,
            self.args.gravity_switch_align_lock_pre_hold_sec,
        )
        rospy.loginfo(
            "Attach switch guard: hold_force_cmd=%s post_settle=%.3fs",
            self.args.attach_switch_hold_force_cmd,
            self.args.attach_switch_post_settle_sec,
        )
        if args.left_master_to_opp_sign != "1,1,1,1,1,1,1" or args.right_master_to_opp_sign != "1,1,1,1,1,1,1":
            rospy.logwarn(
                "Mirror sign args are deprecated/ignored in non-mirror mode: left=%s right=%s",
                args.left_master_to_opp_sign,
                args.right_master_to_opp_sign,
            )
        if args.left_master_to_opp_offset != "0,0,0,0,0,0,0" or args.right_master_to_opp_offset != "0,0,0,0,0,0,0":
            rospy.logwarn(
                "Mirror offset args are deprecated/ignored in non-mirror mode: left=%s right=%s",
                args.left_master_to_opp_offset,
                args.right_master_to_opp_offset,
            )
        rospy.loginfo(
            "Torque mux service left primary=%s fallback=%s selected_topic=%s",
            self.args.left_torque_mux_service,
            self.args.left_torque_mux_service_fallback,
            self.args.left_torque_selected_topic,
        )
        rospy.loginfo(
            "Torque mux service right primary=%s fallback=%s selected_topic=%s",
            self.args.right_torque_mux_service,
            self.args.right_torque_mux_service_fallback,
            self.args.right_torque_selected_topic,
        )

    def _cb_opp_joint(self, msg: JointState) -> None:
        self.latest_joint = msg
        self.latest_opp_msg_monotonic = time.monotonic()
        self._publish_transformed_opp_torque(msg)

    def _cb_selected(self, msg: String) -> None:
        self.latest_selected = msg.data

    def _cb_left_master_joint(self, msg: JointState) -> None:
        self.latest_left_master_joint = msg

    def _cb_right_master_joint(self, msg: JointState) -> None:
        self.latest_right_master_joint = msg

    def _cb_left_slave_joint(self, msg: JointState) -> None:
        self.latest_left_slave_joint = msg

    def _cb_right_slave_joint(self, msg: JointState) -> None:
        self.latest_right_slave_joint = msg

    def _cb_left_torque_selected(self, msg: String) -> None:
        self.latest_left_torque_selected = msg.data

    def _cb_right_torque_selected(self, msg: String) -> None:
        self.latest_right_torque_selected = msg.data

    @staticmethod
    def _copy_joint_state(src: JointState) -> JointState:
        out = JointState()
        out.header = src.header
        out.name = list(src.name)
        out.position = list(src.position)
        out.velocity = list(src.velocity)
        out.effort = list(src.effort)
        return out

    @staticmethod
    def _extract_pos7(src: JointState) -> List[float]:
        out = [0.0] * 7
        n = min(7, len(src.position))
        for i in range(n):
            out[i] = float(src.position[i])
        return out

    @staticmethod
    def _max_abs_joint_error(a: List[float], b: List[float], dof: int) -> float:
        return max(abs(float(a[i]) - float(b[i])) for i in range(dof))

    def _latest_master_joint(self, side: str) -> Optional[JointState]:
        return self.latest_left_master_joint if side == "left" else self.latest_right_master_joint

    def _latest_slave_joint(self, side: str) -> Optional[JointState]:
        return self.latest_left_slave_joint if side == "left" else self.latest_right_slave_joint

    def _wait_joint_msg(self, topic: str, label: str) -> JointState:
        try:
            return rospy.wait_for_message(
                topic,
                JointState,
                timeout=self.args.service_timeout_sec,
            )
        except Exception as exc:
            raise RuntimeError(f"Timeout waiting {label} topic: {topic}") from exc

    def _set_master_cmd_mux(self, side: str, topic: str) -> None:
        proxy = self.left_master_cmd_mux if side == "left" else self.right_master_cmd_mux
        service_name = self.args.left_master_cmd_mux_service if side == "left" else self.args.right_master_cmd_mux_service
        try:
            resp = proxy(topic)
            prev_topic = getattr(resp, "prev_topic", "")
            rospy.loginfo("%s master-cmd mux switch: prev=%s -> now=%s", side, prev_topic, topic)
        except Exception as exc:
            raise RuntimeError(f"Failed to call {service_name} with topic={topic}: {exc}") from exc

    def _force_teleop_mode_for_alignment(self) -> None:
        if not self.args.align_force_teleop_flag:
            return
        self._wait_for_pub_subscriber(
            self.slave_follow_flag_pub,
            self.args.slave_follow_flag_topic,
            self.args.align_flag_subscriber_wait_sec,
        )
        repeats = max(1, int(self.args.align_force_flag_repeat))
        interval = max(0.0, float(self.args.align_force_flag_interval_sec))
        msg = Bool(data=False)
        for _ in range(repeats):
            self.slave_follow_flag_pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)
        rospy.loginfo(
            "alignment pre-step: published %s=false (%d times, interval=%.3fs)",
            self.args.slave_follow_flag_topic,
            repeats,
            interval,
        )

    def _enable_master_for_alignment(self, side: str) -> None:
        if not self.args.align_enable_master_arm:
            return
        proxy = self.left_master_enable_srv if side == "left" else self.right_master_enable_srv
        service_name = self.args.left_master_enable_service if side == "left" else self.args.right_master_enable_service
        try:
            resp = proxy(True)
            ok = bool(getattr(resp, "enable_response", False))
            if ok:
                rospy.loginfo("alignment pre-step: %s(true) -> %s", service_name, ok)
            else:
                rospy.logwarn("alignment pre-step: %s(true) returned false", service_name)
        except Exception as exc:
            rospy.logwarn("alignment pre-step: failed calling %s(true): %s", service_name, str(exc))

    def _unblock_master_for_alignment(self, side: str) -> None:
        if not self.args.align_unblock_master_arm:
            return
        proxy = self.left_master_block_srv if side == "left" else self.right_master_block_srv
        service_name = self.args.left_master_block_service if side == "left" else self.args.right_master_block_service
        try:
            resp = proxy(False)
            # Service returns success=False when unblocking in current implementation; log for visibility.
            ok = bool(getattr(resp, "success", False))
            msg = str(getattr(resp, "message", ""))
            rospy.loginfo("alignment pre-step: %s(false) -> success=%s msg=%s", service_name, ok, msg)
        except Exception as exc:
            rospy.logwarn("alignment pre-step: failed calling %s(false): %s", service_name, str(exc))

    def _set_master_force_cmd_mode(self, side: str, enabled: bool) -> None:
        if not self.args.align_force_master_cmd_mode:
            return
        self._publish_master_force_cmd_mode(side, enabled)

    def _publish_master_force_cmd_mode(self, side: str, enabled: bool) -> None:
        pub = self.left_master_force_cmd_pub if side == "left" else self.right_master_force_cmd_pub
        topic = self.args.left_master_force_cmd_topic if side == "left" else self.args.right_master_force_cmd_topic
        self._wait_for_pub_subscriber(
            pub,
            topic,
            self.args.align_flag_subscriber_wait_sec,
        )
        repeats = max(1, int(self.args.align_force_master_cmd_repeat))
        interval = max(0.0, float(self.args.align_force_master_cmd_interval_sec))
        msg = Bool(data=bool(enabled))
        for _ in range(repeats):
            pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)
        rospy.loginfo(
            "alignment pre-step: published %s=%s (%d times, interval=%.3fs)",
            topic,
            str(bool(enabled)).lower(),
            repeats,
            interval,
        )

    def _start_master_align_lock(self, side: str) -> Tuple[threading.Event, threading.Thread]:
        master_topic = self.args.left_master_topic if side == "left" else self.args.right_master_topic
        align_topic = self.args.left_master_align_topic if side == "left" else self.args.right_master_align_topic
        pub = self.left_align_pub if side == "left" else self.right_align_pub

        master_msg = self._latest_master_joint(side)
        if master_msg is None:
            master_msg = self._wait_joint_msg(master_topic, f"{side} master")
        pos7 = self._extract_pos7(master_msg)
        names = list(master_msg.name[:7]) if len(master_msg.name) >= 7 else list(DEFAULT_JOINT_NAMES)

        self._unblock_master_for_alignment(side)
        self._enable_master_for_alignment(side)
        self._wait_for_pub_subscriber(pub, align_topic, self.args.align_flag_subscriber_wait_sec)
        self._set_master_cmd_mux(side, align_topic)

        stop_event = threading.Event()
        rate_hz = max(5.0, float(self.args.gravity_switch_align_lock_rate_hz))

        def _lock_loop() -> None:
            rate = rospy.Rate(rate_hz)
            while not rospy.is_shutdown() and not stop_event.is_set():
                pub.publish(self._build_align_cmd(names, pos7))
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break

        thread = threading.Thread(
            target=_lock_loop,
            name=f"gravity_align_lock_{side}",
            daemon=True,
        )
        thread.start()

        warmup_sec = max(0.0, float(self.args.gravity_switch_align_lock_pre_hold_sec))
        if warmup_sec > 0.0:
            rospy.sleep(warmup_sec)
        rospy.loginfo(
            "%s gravity align-hold engaged at current pose (rate=%.1fHz pre_hold=%.3fs).",
            side,
            rate_hz,
            warmup_sec,
        )
        return stop_event, thread

    def _stop_master_align_lock(self, side: str, stop_event: threading.Event, thread: threading.Thread) -> None:
        stop_event.set()
        if thread.is_alive():
            thread.join(timeout=0.5)
        default_cmd_topic = (
            self.args.left_master_default_cmd_topic if side == "left" else self.args.right_master_default_cmd_topic
        )
        try:
            self._set_master_cmd_mux(side, default_cmd_topic)
        except Exception as exc:
            rospy.logwarn("%s failed restoring master-cmd mux after gravity align-lock: %s", side, str(exc))

    def _switch_gravity_mux_safely(self, side: str, topic: str, reason: str, *, release_force_cmd: bool = True) -> None:
        if not self.args.gravity_switch_freeze_mode:
            self._set_gravity_mux(side, topic)
            return
        # When align-hold is enabled, pause the arm by streaming its current pose through the
        # alignment command path. Otherwise fall back to the older force-cmd freeze behavior.
        lock_state: Optional[Tuple[threading.Event, threading.Thread]] = None
        self._force_teleop_mode_for_alignment()
        use_force_cmd_freeze = not self.args.gravity_switch_align_lock_mode
        if use_force_cmd_freeze:
            self._publish_master_force_cmd_mode(side, True)
        try:
            if self.args.gravity_switch_align_lock_mode:
                lock_state = self._start_master_align_lock(side)
            self._set_gravity_mux(side, topic)
            if self.args.gravity_switch_calm_mode:
                self._wait_for_gravity_source_calm(side=side, topic=topic, reason=reason)
            else:
                settle_sec = max(0.0, float(self.args.gravity_switch_settle_sec))
                if settle_sec > 0.0:
                    rospy.sleep(settle_sec)
        finally:
            if lock_state is not None:
                self._stop_master_align_lock(side, lock_state[0], lock_state[1])
            if use_force_cmd_freeze and release_force_cmd:
                self._publish_master_force_cmd_mode(side, False)
        rospy.loginfo(
            "%s gravity switch freeze window completed (reason=%s, settle=%.3fs).",
            side,
            reason,
            float(self.args.gravity_switch_settle_sec),
        )

    def _wait_for_gravity_source_calm(self, *, side: str, topic: str, reason: str) -> None:
        min_sec = max(0.0, float(self.args.gravity_switch_calm_min_sec))
        settle_sec = max(0.0, float(self.args.gravity_switch_settle_sec))
        min_sec = max(min_sec, settle_sec)
        max_sec = max(min_sec, float(self.args.gravity_switch_calm_max_sec))
        required = max(1, int(self.args.gravity_switch_calm_consecutive))
        effort_step_max = max(0.0, float(self.args.gravity_switch_calm_effort_step_max))
        pos_step_max = max(0.0, float(self.args.gravity_switch_calm_pos_step_max))
        sample_timeout_sec = max(1e-3, float(self.args.gravity_switch_calm_sample_timeout_sec))

        start_mono = time.monotonic()
        prev_msg: Optional[JointState] = None
        stable_count = 0
        sample_count = 0
        last_eff_step = 0.0
        last_pos_step = 0.0

        while not rospy.is_shutdown():
            elapsed = time.monotonic() - start_mono
            remaining = max_sec - elapsed
            if remaining <= 0.0:
                rospy.logwarn(
                    "%s gravity calm timeout (reason=%s topic=%s elapsed=%.3fs samples=%d stable_count=%d/%d last_eff_step=%.4f last_pos_step=%.4f).",
                    side,
                    reason,
                    topic,
                    elapsed,
                    sample_count,
                    stable_count,
                    required,
                    last_eff_step,
                    last_pos_step,
                )
                return

            wait_timeout = min(sample_timeout_sec, remaining)
            try:
                msg = rospy.wait_for_message(topic, JointState, timeout=wait_timeout)
            except Exception:
                continue

            sample_count += 1
            if prev_msg is not None:
                prev_pos = self._extract_pos7(prev_msg)
                curr_pos = self._extract_pos7(msg)
                last_pos_step = max(abs(curr_pos[i] - prev_pos[i]) for i in range(6))

                effort_ready = len(prev_msg.effort) >= 6 and len(msg.effort) >= 6
                if effort_ready:
                    prev_eff = self._raw_effort6(prev_msg)
                    curr_eff = self._raw_effort6(msg)
                    last_eff_step = max(abs(curr_eff[i] - prev_eff[i]) for i in range(6))
                else:
                    last_eff_step = 0.0

                pos_ok = last_pos_step <= pos_step_max
                effort_ok = (not effort_ready) or (last_eff_step <= effort_step_max)
                if pos_ok and effort_ok:
                    stable_count += 1
                else:
                    stable_count = 0
            prev_msg = msg

            elapsed = time.monotonic() - start_mono
            if elapsed < min_sec:
                continue
            if stable_count >= required:
                rospy.loginfo(
                    "%s gravity calm ready (reason=%s topic=%s elapsed=%.3fs samples=%d stable_count=%d/%d last_eff_step=%.4f last_pos_step=%.4f).",
                    side,
                    reason,
                    topic,
                    elapsed,
                    sample_count,
                    stable_count,
                    required,
                    last_eff_step,
                    last_pos_step,
                )
                return

    @staticmethod
    def _wait_for_pub_subscriber(pub, topic: str, timeout_sec: float) -> None:
        timeout = max(0.0, float(timeout_sec))
        if timeout <= 0.0:
            return
        t0 = time.monotonic()
        while not rospy.is_shutdown():
            if pub.get_num_connections() > 0:
                return
            if (time.monotonic() - t0) >= timeout:
                rospy.logwarn(
                    "No subscriber observed for %s within %.2fs; publishing anyway.",
                    topic,
                    timeout,
                )
                return
            rospy.sleep(0.02)

    def _build_align_cmd(self, names: List[str], pos7: List[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(names)
        msg.position = [float(v) for v in pos7]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

    def _align_master_to_target(self, side: str, target_msg: JointState, target_label: str) -> bool:
        master_topic = self.args.left_master_topic if side == "left" else self.args.right_master_topic
        align_topic = self.args.left_master_align_topic if side == "left" else self.args.right_master_align_topic
        default_cmd_topic = (
            self.args.left_master_default_cmd_topic if side == "left" else self.args.right_master_default_cmd_topic
        )
        pub = self.left_align_pub if side == "left" else self.right_align_pub

        self._force_teleop_mode_for_alignment()
        self._unblock_master_for_alignment(side)
        self._enable_master_for_alignment(side)

        master_msg = self._latest_master_joint(side)
        if master_msg is None:
            master_msg = self._wait_joint_msg(master_topic, f"{side} master")

        start_pos = self._extract_pos7(master_msg)
        target_pos = self._extract_pos7(target_msg)
        names = list(target_msg.name[:7]) if len(target_msg.name) >= 7 else list(DEFAULT_JOINT_NAMES)
        dof = 7 if self.args.align_include_gripper else 6
        if dof == 6:
            target_pos[6] = start_pos[6]
        max_err = self._max_abs_joint_error(start_pos, target_pos, dof)
        tol = max(1e-4, float(self.args.align_master_tolerance_rad))
        if max_err <= tol:
            rospy.loginfo("%s align->%s skipped: max_err=%.4f <= tol=%.4f", side, target_label, max_err, tol)
            return True

        speed = max(1e-3, float(self.args.align_master_max_joint_speed))
        min_dur = max(0.0, float(self.args.align_master_min_duration_sec))
        max_dur = max(min_dur, float(self.args.align_master_max_duration_sec))
        duration = _clamp(max_err / speed, min_dur, max_dur)
        rate_hz = max(5.0, float(self.args.align_master_rate_hz))
        steps = max(2, int(duration * rate_hz))
        settle_needed = max(2, int(max(0.05, float(self.args.align_master_settle_sec)) * rate_hz))
        timeout_sec = max(float(self.args.align_master_timeout_sec), duration + 0.5)

        rospy.loginfo(
            "%s align master->%s: max_err=%.4f tol=%.4f duration=%.3fs steps=%d",
            side,
            target_label,
            max_err,
            tol,
            duration,
            steps,
        )

        success = False
        min_err = max_err
        final_err = max_err
        t0 = time.monotonic()
        next_log_mono = t0 + 0.5
        try:
            self._set_master_force_cmd_mode(side, True)
            self._set_master_cmd_mux(side, align_topic)
            rate = rospy.Rate(rate_hz)
            for idx in range(1, steps + 1):
                alpha = float(idx) / float(steps)
                pos = [start_pos[i] + alpha * (target_pos[i] - start_pos[i]) for i in range(7)]
                pub.publish(self._build_align_cmd(names, pos))
                rate.sleep()

            stable_count = 0
            while not rospy.is_shutdown():
                pub.publish(self._build_align_cmd(names, target_pos))
                current = self._latest_master_joint(side)
                if current is not None:
                    cur_pos = self._extract_pos7(current)
                    err = self._max_abs_joint_error(cur_pos, target_pos, dof)
                    final_err = err
                    if err < min_err:
                        min_err = err
                    now_mono = time.monotonic()
                    if now_mono >= next_log_mono:
                        rospy.loginfo(
                            "%s align->%s progress: err=%.4f min_err=%.4f tol=%.4f",
                            side,
                            target_label,
                            err,
                            min_err,
                            tol,
                        )
                        next_log_mono = now_mono + 0.5
                    if err <= tol:
                        stable_count += 1
                    else:
                        stable_count = 0
                    if stable_count >= settle_needed:
                        success = True
                        break
                if (time.monotonic() - t0) > timeout_sec:
                    break
                rate.sleep()
        finally:
            try:
                self._set_master_cmd_mux(side, default_cmd_topic)
            except Exception as exc:
                rospy.logwarn("%s failed restoring master-cmd mux: %s", side, str(exc))
            self._set_master_force_cmd_mode(side, False)

        if success:
            rospy.loginfo("%s align->%s completed successfully.", side, target_label)
        else:
            rospy.logwarn(
                "%s align->%s timed out; continuing. final_err=%.4f min_err=%.4f tol=%.4f",
                side,
                target_label,
                final_err,
                min_err,
                tol,
            )
        return success

    def _align_master_to_slave(self, side: str) -> bool:
        if not self.args.align_before_restore:
            return True

        slave_topic = self.args.left_slave_state_topic if side == "left" else self.args.right_slave_state_topic
        slave_msg = self._latest_slave_joint(side)
        if slave_msg is None:
            slave_msg = self._wait_joint_msg(slave_topic, f"{side} slave")
        return self._align_master_to_target(side, slave_msg, "slave")

    def _align_master_to_opp(self, side: str) -> bool:
        if not self.args.align_before_attach:
            return True
        opp_msg = self.latest_joint
        if opp_msg is None:
            opp_msg = self._wait_joint_msg(self.args.opp_state_topic, "opp")
        return self._align_master_to_target(side, opp_msg, "opp")

    def _compute_side_gain(self, side: str, now_mono: float) -> float:
        if self.active_master_side != side:
            return 0.0
        if self.opp_torque_ramp_sec <= 1e-9:
            return 1.0
        started = self.left_attach_started_monotonic if side == "left" else self.right_attach_started_monotonic
        if started is None:
            return 1.0
        ratio = (now_mono - started) / self.opp_torque_ramp_sec
        return _clamp(ratio, 0.0, 1.0)

    @staticmethod
    def _raw_effort6(src: JointState) -> List[float]:
        vals = [0.0] * 6
        if len(src.effort) >= 6:
            for i in range(6):
                vals[i] = float(src.effort[i])
        return vals

    def _shape_opp_effort(
        self,
        *,
        raw_effort: List[float],
        sign_vec: List[float],
        scale_vec: List[float],
        max_vec: List[float],
        deadband_vec: List[float],
        rate_limit_vec: List[float],
        alpha: float,
        filtered_cache: List[float],
        gain: float,
    ) -> List[float]:
        out = [0.0] * 6
        for i in range(6):
            val = raw_effort[i] * sign_vec[i] * scale_vec[i]
            if abs(val) < abs(deadband_vec[i]):
                val = 0.0
            prev = filtered_cache[i]
            filt = alpha * val + (1.0 - alpha) * prev
            step_lim = abs(rate_limit_vec[i])
            if step_lim > 0.0:
                filt = _clamp(filt, prev - step_lim, prev + step_lim)
            lim = abs(max_vec[i])
            if lim > 0.0:
                filt = _clamp(filt, -lim, lim)
            filtered_cache[i] = filt
            out[i] = filt * gain
        return out

    def _publish_transformed_opp_torque(self, msg: JointState) -> None:
        raw_effort = self._raw_effort6(msg)
        raw_abs_peak = max(abs(v) for v in raw_effort) if raw_effort else 0.0
        if self.active_master_side is not None and raw_abs_peak < self.args.opp_torque_raw_effort_warn_threshold:
            rospy.logwarn_throttle(
                1.0,
                "OPP raw effort peak is low (%.5f < %.5f). Transformed torque feedback may feel absent.",
                raw_abs_peak,
                self.args.opp_torque_raw_effort_warn_threshold,
            )
        now_mono = time.monotonic()
        left_gain = self._compute_side_gain("left", now_mono)
        right_gain = self._compute_side_gain("right", now_mono)

        left_eff = self._shape_opp_effort(
            raw_effort=raw_effort,
            sign_vec=self.left_opp_torque_sign,
            scale_vec=self.left_opp_torque_scale,
            max_vec=self.left_opp_torque_max,
            deadband_vec=self.left_opp_torque_deadband,
            rate_limit_vec=self.left_opp_torque_rate_limit,
            alpha=self.left_opp_torque_alpha,
            filtered_cache=self.left_opp_torque_filtered,
            gain=left_gain,
        )
        right_eff = self._shape_opp_effort(
            raw_effort=raw_effort,
            sign_vec=self.right_opp_torque_sign,
            scale_vec=self.right_opp_torque_scale,
            max_vec=self.right_opp_torque_max,
            deadband_vec=self.right_opp_torque_deadband,
            rate_limit_vec=self.right_opp_torque_rate_limit,
            alpha=self.right_opp_torque_alpha,
            filtered_cache=self.right_opp_torque_filtered,
            gain=right_gain,
        )

        left_msg = self._copy_joint_state(msg)
        right_msg = self._copy_joint_state(msg)
        if len(left_msg.effort) < 6:
            left_msg.effort = [0.0] * 6
        if len(right_msg.effort) < 6:
            right_msg.effort = [0.0] * 6
        for i in range(6):
            left_msg.effort[i] = left_eff[i]
            right_msg.effort[i] = right_eff[i]
        if len(left_msg.effort) < 7:
            left_msg.effort = list(left_msg.effort) + [0.0] * (7 - len(left_msg.effort))
        if len(right_msg.effort) < 7:
            right_msg.effort = list(right_msg.effort) + [0.0] * (7 - len(right_msg.effort))
        if self.args.opp_pass_through_gripper_effort and len(msg.effort) >= 7:
            gripper_eff = float(msg.effort[6])
        else:
            gripper_eff = 0.0
        left_msg.effort[6] = gripper_eff
        right_msg.effort[6] = gripper_eff

        if self.active_master_side == "left" and self.left_opp_torque_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(
                1.0,
                "Left transformed OPP torque topic has no subscribers: %s",
                self.args.left_opp_torque_topic,
            )
        if self.active_master_side == "right" and self.right_opp_torque_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(
                1.0,
                "Right transformed OPP torque topic has no subscribers: %s",
                self.args.right_opp_torque_topic,
            )

        self.left_opp_torque_pub.publish(left_msg)
        self.right_opp_torque_pub.publish(right_msg)

    def _publish_zero_opp_torque(self) -> None:
        if self.latest_joint is not None:
            left_msg = self._copy_joint_state(self.latest_joint)
            right_msg = self._copy_joint_state(self.latest_joint)
        else:
            left_msg = JointState()
            right_msg = JointState()
            left_msg.name = list(DEFAULT_JOINT_NAMES)
            right_msg.name = list(DEFAULT_JOINT_NAMES)
            left_msg.position = [0.0] * 7
            right_msg.position = [0.0] * 7
            left_msg.velocity = [0.0] * 7
            right_msg.velocity = [0.0] * 7

        if len(left_msg.effort) < 7:
            left_msg.effort = list(left_msg.effort) + [0.0] * (7 - len(left_msg.effort))
        if len(right_msg.effort) < 7:
            right_msg.effort = list(right_msg.effort) + [0.0] * (7 - len(right_msg.effort))
        for i in range(7):
            left_msg.effort[i] = 0.0
            right_msg.effort[i] = 0.0
        now = rospy.Time.now()
        left_msg.header.stamp = now
        right_msg.header.stamp = now
        self.left_opp_torque_pub.publish(left_msg)
        self.right_opp_torque_pub.publish(right_msg)

    def _zero_opp_torque_if_stale(self) -> None:
        if not self.args.route_torque_feedback or not self.args.use_transformed_opp_torque:
            return
        if self.active_master_side is None:
            return
        now_mono = time.monotonic()
        if self.latest_opp_msg_monotonic is None:
            stale = True
        else:
            stale = (now_mono - self.latest_opp_msg_monotonic) > self.args.opp_torque_stale_sec
        if not stale:
            return
        if self.last_stale_zero_publish_monotonic is not None:
            if (now_mono - self.last_stale_zero_publish_monotonic) < 0.05:
                return
        self._publish_zero_opp_torque()
        self.last_stale_zero_publish_monotonic = now_mono
        rospy.logwarn_throttle(
            1.0,
            "OPP torque feedback stale (>%.3fs), publishing zero transformed torque.",
            self.args.opp_torque_stale_sec,
        )

    def _build_hold_msg(self) -> JointState:
        if self.latest_joint is None:
            raise RuntimeError("No /robot/arm_opp/joint_states_single received yet.")
        if len(self.latest_joint.position) < 7:
            raise RuntimeError(
                "Latest /robot/arm_opp/joint_states_single has <7 position dimensions."
            )

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        names = list(self.latest_joint.name[:7]) if len(self.latest_joint.name) >= 7 else list(DEFAULT_JOINT_NAMES)
        msg.name = names
        msg.position = [float(v) for v in self.latest_joint.position[:7]]
        # Hold should be a pure position command; forwarding live effort/velocity can cause tiny twitches.
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

    def _select_source(self, source_topic: str) -> None:
        try:
            resp = self.mux_select(source_topic)
            prev_topic = getattr(resp, "prev_topic", "")
            rospy.loginfo("mux switch: prev=%s -> now=%s", prev_topic, source_topic)
        except Exception as exc:
            raise RuntimeError(f"Failed to select mux source {source_topic}: {exc}") from exc

    def _set_slave_mux(self, side: str, topic: str) -> None:
        proxy = self.left_slave_mux if side == "left" else self.right_slave_mux
        service_name = self.args.left_slave_mux_service if side == "left" else self.args.right_slave_mux_service
        try:
            resp = proxy(topic)
            prev_topic = getattr(resp, "prev_topic", "")
            rospy.loginfo("%s mux switch: prev=%s -> now=%s", side, prev_topic, topic)
        except Exception as exc:
            raise RuntimeError(f"Failed to call {service_name} with topic={topic}: {exc}") from exc

    def _restore_default_slave_route(self, side: str) -> None:
        target = self.args.left_default_topic if side == "left" else self.args.right_default_topic
        self._set_slave_mux(side, target)

    def _detach_slave_route(self, side: str) -> None:
        target = self.args.left_detach_topic if side == "left" else self.args.right_detach_topic
        self._set_slave_mux(side, target)

    def _latest_torque_selected(self, side: str) -> Optional[str]:
        return self.latest_left_torque_selected if side == "left" else self.latest_right_torque_selected

    def _torque_selected_topic(self, side: str) -> str:
        return self.args.left_torque_selected_topic if side == "left" else self.args.right_torque_selected_topic

    def _verify_torque_mux_selected(self, side: str, expected_topic: str) -> bool:
        selected = self._latest_torque_selected(side)
        if selected == expected_topic:
            return True
        timeout_sec = max(0.0, float(self.args.mux_select_verify_timeout_sec))
        selected_topic = self._torque_selected_topic(side)
        if timeout_sec > 0.0:
            try:
                msg = rospy.wait_for_message(selected_topic, String, timeout=timeout_sec)
                selected = msg.data
                if side == "left":
                    self.latest_left_torque_selected = selected
                else:
                    self.latest_right_torque_selected = selected
            except Exception:
                selected = self._latest_torque_selected(side)
        ok = selected == expected_topic
        if not ok:
            rospy.logwarn(
                "%s torque mux selected mismatch: expected=%s actual=%s (selected_topic=%s)",
                side,
                expected_topic,
                selected if selected is not None else "<unknown>",
                selected_topic,
            )
        return ok

    @staticmethod
    def _is_unknown_selected(selected: Optional[str]) -> bool:
        if selected is None:
            return True
        s = str(selected).strip().lower()
        return (not s) or s == "<unknown>"

    def _verify_transformed_torque_subscriber(self, side: str, target_topic: str) -> None:
        expected_topic = self.args.left_opp_torque_topic if side == "left" else self.args.right_opp_torque_topic
        if target_topic != expected_topic:
            return
        pub = self.left_opp_torque_pub if side == "left" else self.right_opp_torque_pub
        timeout_sec = max(0.0, float(self.args.transformed_torque_subscriber_wait_sec))
        if timeout_sec <= 0.0:
            return
        t0 = time.monotonic()
        while not rospy.is_shutdown():
            if pub.get_num_connections() > 0:
                return
            if (time.monotonic() - t0) >= timeout_sec:
                msg = (
                    f"{side} transformed OPP torque route selected ({target_topic}) but publisher has no subscribers "
                    f"within {timeout_sec:.2f}s."
                )
                if self.args.strict_torque_mux_verify:
                    raise RuntimeError(msg)
                rospy.logwarn("%s", msg)
                return
            rospy.sleep(0.02)

    def _try_call_mux(self, proxy, service_name: str, topic: str, side: str, kind: str) -> Optional[str]:
        if proxy is None:
            return None
        resp = proxy(topic)
        prev_topic = getattr(resp, "prev_topic", "")
        rospy.loginfo("%s %s mux switch via %s: prev=%s -> now=%s", side, kind, service_name, prev_topic, topic)
        return prev_topic

    def _set_torque_mux(self, side: str, topic: str) -> None:
        primary_proxy = self.left_torque_mux if side == "left" else self.right_torque_mux
        primary_service = self.args.left_torque_mux_service if side == "left" else self.args.right_torque_mux_service
        fallback_proxy = self.left_torque_mux_fallback if side == "left" else self.right_torque_mux_fallback
        fallback_service = (
            self.args.left_torque_mux_service_fallback if side == "left" else self.args.right_torque_mux_service_fallback
        )

        errors: List[str] = []
        tried_fallback = False
        try:
            prev_topic = self._try_call_mux(primary_proxy, primary_service, topic, side, "torque")
            ok = self._verify_torque_mux_selected(side, topic)
            # Some mux variants do not publish /.../selected on every call (especially when topic is unchanged).
            # If service reports prev==target, route is already at desired topic; accept as success.
            if not ok and prev_topic == topic:
                rospy.logwarn(
                    "%s torque mux selected topic not observable, but prev==target (%s). Accepting route.",
                    side,
                    topic,
                )
                ok = True

            if not ok and fallback_proxy is not None:
                tried_fallback = True
                rospy.logwarn(
                    "%s torque mux verify failed via primary service %s; retrying via fallback %s",
                    side,
                    primary_service,
                    fallback_service,
                )
                prev_topic = self._try_call_mux(fallback_proxy, fallback_service, topic, side, "torque")
                ok = self._verify_torque_mux_selected(side, topic)
                if not ok and prev_topic == topic:
                    rospy.logwarn(
                        "%s torque mux selected topic not observable via fallback, but prev==target (%s). Accepting route.",
                        side,
                        topic,
                    )
                    ok = True
            if not ok and self._is_unknown_selected(self._latest_torque_selected(side)):
                rospy.logwarn(
                    "%s torque mux selected topic is unavailable/unknown; accepting service result as best-effort.",
                    side,
                )
                ok = True
            if not ok and self.args.strict_torque_mux_verify:
                raise RuntimeError(
                    f"{side} torque mux failed to select expected topic={topic} "
                    f"(selected_topic={self._torque_selected_topic(side)})"
                )
            self._verify_transformed_torque_subscriber(side, topic)
            return
        except Exception as exc:
            errors.append(str(exc))

        if fallback_proxy is not None and not tried_fallback:
            try:
                prev_topic = self._try_call_mux(fallback_proxy, fallback_service, topic, side, "torque")
                ok = self._verify_torque_mux_selected(side, topic)
                if not ok and prev_topic == topic:
                    rospy.logwarn(
                        "%s torque mux selected topic not observable via fallback, but prev==target (%s). Accepting route.",
                        side,
                        topic,
                    )
                    ok = True
                if not ok and self._is_unknown_selected(self._latest_torque_selected(side)):
                    rospy.logwarn(
                        "%s torque mux selected topic is unavailable/unknown via fallback; accepting service result.",
                        side,
                    )
                    ok = True
                if not ok and self.args.strict_torque_mux_verify:
                    raise RuntimeError(
                        f"{side} torque mux failed to select expected topic={topic} "
                        f"(selected_topic={self._torque_selected_topic(side)})"
                    )
                self._verify_transformed_torque_subscriber(side, topic)
                return
            except Exception as exc:
                errors.append(str(exc))

        raise RuntimeError(
            f"Failed to call torque mux for {side} with topic={topic}. "
            f"primary={primary_service} fallback={fallback_service} errors={errors}"
        )

    def _restore_default_torque_route(self, side: str) -> None:
        if not self.args.route_torque_feedback:
            return
        target = self.args.left_default_torque_topic if side == "left" else self.args.right_default_torque_topic
        self._set_torque_mux(side, target)

    def _route_torque_to_opp(self, side: str) -> None:
        if not self.args.route_torque_feedback:
            return
        if self.args.use_transformed_opp_torque:
            target = self.args.left_opp_torque_topic if side == "left" else self.args.right_opp_torque_topic
        else:
            target = self.args.opp_torque_topic
        self._set_torque_mux(side, target)

    def _set_gravity_mux(self, side: str, topic: str) -> None:
        proxy = self.left_gravity_mux if side == "left" else self.right_gravity_mux
        service_name = self.args.left_gravity_mux_service if side == "left" else self.args.right_gravity_mux_service
        try:
            resp = proxy(topic)
            prev_topic = getattr(resp, "prev_topic", "")
            rospy.loginfo("%s gravity mux switch: prev=%s -> now=%s", side, prev_topic, topic)
        except Exception as exc:
            raise RuntimeError(f"Failed to call {service_name} with topic={topic}: {exc}") from exc

    def _restore_default_gravity_route(self, side: str) -> None:
        if not self.args.route_gravity_feedback:
            return
        target = self.args.left_default_gravity_topic if side == "left" else self.args.right_default_gravity_topic
        self._switch_gravity_mux_safely(side, target, "restore_default")

    def _route_gravity_to_opp(self, side: str, *, release_force_cmd: bool = True) -> None:
        if not self.args.route_gravity_feedback:
            return
        self._switch_gravity_mux_safely(
            side,
            self.args.opp_gravity_topic,
            "route_to_opp",
            release_force_cmd=release_force_cmd,
        )

    def _attach_opp_to_master(self, side: str) -> None:
        if self.active_master_side and self.active_master_side != side:
            rospy.loginfo(
                "Switching OPP source from %s master to %s master: releasing previous side first.",
                self.active_master_side,
                side,
            )
            self._disconnect_opp_source(restore_master=True)
        if self.args.exclusive_master_routing:
            self._detach_slave_route(side)
        elif self.args.align_before_attach:
            rospy.logwarn_throttle(
                2.0,
                "align_before_attach is on while exclusive_master_routing is off. "
                "Master alignment motion may move the default slave as well.",
            )
        self._align_master_to_opp(side)

        if side == "left":
            self.left_opp_torque_filtered = [0.0] * 6
            self.left_attach_started_monotonic = time.monotonic()
            self.right_attach_started_monotonic = None
        else:
            self.right_opp_torque_filtered = [0.0] * 6
            self.right_attach_started_monotonic = time.monotonic()
            self.left_attach_started_monotonic = None
        self.last_stale_zero_publish_monotonic = None

        source = self.args.left_master_topic if side == "left" else self.args.right_master_topic
        hold_force_cmd = bool(self.args.attach_switch_hold_force_cmd and not self.args.gravity_switch_align_lock_mode)
        if self.args.attach_switch_hold_force_cmd and self.args.gravity_switch_align_lock_mode:
            rospy.loginfo(
                "%s attach force-cmd hold suppressed; using gravity align-hold current-pose pause instead.",
                side,
            )
        if hold_force_cmd:
            self._force_teleop_mode_for_alignment()
            self._publish_master_force_cmd_mode(side, True)
        try:
            self._route_torque_to_opp(side)
            self._route_gravity_to_opp(side, release_force_cmd=not hold_force_cmd)
            self._select_source(source)
            if hold_force_cmd:
                settle_sec = max(0.0, float(self.args.attach_switch_post_settle_sec))
                if settle_sec > 0.0:
                    rospy.sleep(settle_sec)
        finally:
            if hold_force_cmd:
                self._publish_master_force_cmd_mode(side, False)
        self.active_master_side = side

    def _release_master_if_needed(self) -> None:
        if self.active_master_side is None:
            return
        active_side = self.active_master_side
        self._restore_default_torque_route(active_side)
        self._restore_default_gravity_route(active_side)
        if self.args.exclusive_master_routing:
            # Keep slave route detached while aligning master to target slave pose.
            self._detach_slave_route(active_side)
            self._align_master_to_slave(active_side)
            self._restore_default_slave_route(active_side)
        self.left_attach_started_monotonic = None
        self.right_attach_started_monotonic = None
        self.left_opp_torque_filtered = [0.0] * 6
        self.right_opp_torque_filtered = [0.0] * 6
        self.last_stale_zero_publish_monotonic = None
        self.active_master_side = None

    def _disconnect_opp_source(self, restore_master: bool = True) -> None:
        hold_msg: Optional[JointState] = None
        try:
            hold_msg = self._build_hold_msg()
        except Exception as exc:
            rospy.logwarn("hold pre-publish skipped: %s", str(exc))

        if hold_msg is not None:
            for _ in range(max(1, self.args.hold_pub_repeat)):
                hold_msg.header.stamp = rospy.Time.now()
                self.hold_pub.publish(hold_msg)
                rospy.sleep(self.args.hold_pub_interval_sec)

        self._select_source(self.args.hold_topic)

        if hold_msg is not None:
            for _ in range(max(1, self.args.hold_pub_repeat)):
                hold_msg.header.stamp = rospy.Time.now()
                self.hold_pub.publish(hold_msg)
                rospy.sleep(self.args.hold_pub_interval_sec)

            rospy.loginfo(
                "hold applied on %s; first3_pos=%s",
                self.args.hold_topic,
                [round(v, 4) for v in hold_msg.position[:3]],
            )
        else:
            rospy.loginfo("hold source selected on %s (without hold pre/post publish)", self.args.hold_topic)

        if restore_master:
            self._release_master_if_needed()

    def _enter_hold_mode(self) -> None:
        self._disconnect_opp_source(restore_master=True)

    def _print_status(self) -> None:
        selected = self.latest_selected or "<unknown>"
        joint_ok = self.latest_joint is not None and len(self.latest_joint.position) >= 7
        rospy.loginfo(
            "status: selected=%s opp_joint_ready=%s active_master_side=%s torque_selected[left=%s right=%s] exclusive_master_routing=%s route_torque_feedback=%s route_gravity_feedback=%s gravity_auto_disabled=%s align_before_restore=%s align_before_attach=%s",
            selected,
            joint_ok,
            self.active_master_side or "<none>",
            self.latest_left_torque_selected or "<unknown>",
            self.latest_right_torque_selected or "<unknown>",
            self.args.exclusive_master_routing,
            self.args.route_torque_feedback,
            self.args.route_gravity_feedback,
            self.gravity_feedback_auto_disabled,
            self.args.align_before_restore,
            self.args.align_before_attach,
        )

    def _ensure_arm_ready(self) -> None:
        if self.args.skip_arm_ready:
            return

        rospy.loginfo("Waiting for arm services: %s, %s", self.args.enable_service, self.args.block_service)
        rospy.wait_for_service(self.args.enable_service, timeout=self.args.service_timeout_sec)
        rospy.wait_for_service(self.args.block_service, timeout=self.args.service_timeout_sec)

        try:
            block_resp = self.block_srv(False)
            rospy.loginfo(
                "block_arm(false): success=%s msg=%s",
                bool(getattr(block_resp, "success", False)),
                str(getattr(block_resp, "message", "")),
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to call {self.args.block_service}: {exc}") from exc

        ok = self._enable_arm_with_retries()
        if ok:
            return

        msg = (
            f"enable_srv returned false after {self.args.enable_retries} attempts. "
            "Likely CAN/power/driver-enable issue on arm_opp."
        )
        fallback_ok = self._publish_enable_flag_fallback()
        if fallback_ok:
            rospy.logwarn(
                "%s Falling back to %s=true.",
                msg,
                self.args.enable_flag_topic,
            )
            return

        if self.args.strict_arm_ready:
            raise RuntimeError(msg)
        rospy.logwarn("%s Continuing anyway (--strict-arm-ready to fail fast).", msg)

    def _enable_arm_with_retries(self) -> bool:
        attempts = max(1, int(self.args.enable_retries))
        for idx in range(1, attempts + 1):
            ok = False
            err: Optional[str] = None
            try:
                enable_resp = self.enable_srv(True)
                ok = bool(getattr(enable_resp, "enable_response", False))
            except Exception as exc:
                err = str(exc)

            if err:
                rospy.logwarn("enable_srv(true) attempt %d/%d exception: %s", idx, attempts, err)
            else:
                rospy.loginfo("enable_srv(true) attempt %d/%d -> %s", idx, attempts, ok)

            if ok:
                return True
            if idx < attempts:
                rospy.sleep(max(0.0, float(self.args.enable_retry_interval_sec)))
        return False

    def _publish_enable_flag_fallback(self) -> bool:
        if not self.args.enable_flag_fallback:
            return False
        repeats = max(1, int(self.args.enable_flag_repeat))
        interval = max(0.0, float(self.args.enable_flag_interval_sec))
        rospy.loginfo(
            "Fallback: publishing %s=true %d times (interval=%.3fs)",
            self.args.enable_flag_topic,
            repeats,
            interval,
        )
        msg = Bool(data=True)
        for _ in range(repeats):
            self.enable_flag_pub.publish(msg)
            rospy.sleep(interval)
        return True

    def _collect_gravity_topic_diagnostics(self) -> str:
        details: List[str] = []
        param_name = "/piper_gravity_compensation_node/enable_opp_arm"
        try:
            if rospy.has_param(param_name):
                value = rospy.get_param(param_name)
                details.append(f"{param_name}={value}")
            else:
                details.append(f"{param_name}=<unset>")
        except Exception as exc:
            details.append(f"{param_name}=<error:{exc}>")

        quick_checks = [
            ("left_gravity", self.args.left_default_gravity_topic),
            ("right_gravity", self.args.right_default_gravity_topic),
            ("opp_state", self.args.opp_state_topic),
        ]
        for label, topic in quick_checks:
            try:
                rospy.wait_for_message(topic, JointState, timeout=0.35)
                details.append(f"{label}:ok({topic})")
            except Exception:
                details.append(f"{label}:missing({topic})")
        return "; ".join(details)

    def _wait_for_feedback_topics_ready(self) -> None:
        if self.args.route_torque_feedback and self.args.use_transformed_opp_torque:
            rospy.loginfo("Waiting for OPP torque source topic: %s", self.args.opp_state_topic)
            try:
                rospy.wait_for_message(
                    self.args.opp_state_topic,
                    JointState,
                    timeout=self.args.service_timeout_sec,
                )
            except Exception as exc:
                raise RuntimeError(
                    f"No messages on {self.args.opp_state_topic}; cannot build transformed OPP torque feedback."
                ) from exc

        if self.args.route_gravity_feedback:
            rospy.loginfo("Waiting for OPP gravity topic: %s", self.args.opp_gravity_topic)
            try:
                rospy.wait_for_message(
                    self.args.opp_gravity_topic,
                    JointState,
                    timeout=self.args.service_timeout_sec,
                )
            except Exception as exc:
                diag = self._collect_gravity_topic_diagnostics()
                if self.args.strict_gravity_feedback_topic:
                    raise RuntimeError(
                        f"No messages on {self.args.opp_gravity_topic}. "
                        "Launch teleop gravity node with enable_opp_arm:=true. "
                        f"Diagnostics: {diag}"
                    ) from exc
                param_name = "/piper_gravity_compensation_node/enable_opp_arm"
                param_note = ""
                try:
                    if rospy.has_param(param_name):
                        param_note = f" Observed {param_name}={rospy.get_param(param_name)}."
                except Exception:
                    param_note = ""
                rospy.logwarn(
                    "No messages on %s within %.2fs; auto-disabling gravity feedback routing for this session.%s "
                    "This fallback is enabled by --no-strict-gravity-feedback-topic. Diagnostics: %s",
                    self.args.opp_gravity_topic,
                    float(self.args.service_timeout_sec),
                    param_note,
                    diag,
                )
                rospy.logwarn(
                    "To enable OPP gravity routing, launch teleop gravity node with enable_opp_arm:=true "
                    "(e.g. roslaunch teleop_setup start_teleop_all.launch enable_opp_arm:=true)."
                )
                self.args.route_gravity_feedback = False
                self.gravity_feedback_auto_disabled = True

    def _wait_for_alignment_topics_ready(self) -> None:
        need_restore_align = self.args.align_before_restore and self.args.exclusive_master_routing
        need_attach_align = self.args.align_before_attach
        need_gravity_align_lock = self.args.gravity_switch_align_lock_mode
        if not need_restore_align and not need_attach_align and not need_gravity_align_lock:
            return
        checks = [("left master", self.args.left_master_topic), ("right master", self.args.right_master_topic)]
        if need_restore_align:
            checks.extend(
                [
                    ("left slave", self.args.left_slave_state_topic),
                    ("right slave", self.args.right_slave_state_topic),
                ]
            )
        if need_attach_align:
            checks.append(("opp", self.args.opp_state_topic))
        for label, topic in checks:
            rospy.loginfo("Waiting for %s topic: %s", label, topic)
            try:
                rospy.wait_for_message(
                    topic,
                    JointState,
                    timeout=self.args.service_timeout_sec,
                )
            except Exception as exc:
                raise RuntimeError(f"No messages on {topic}; cannot enable smooth handover alignment.") from exc

    def _wait_for_service_with_fallback(self, primary: str, fallback: str, label: str) -> None:
        timeout_sec = self.args.service_timeout_sec
        try:
            rospy.wait_for_service(primary, timeout=timeout_sec)
            return
        except Exception as primary_exc:
            if not fallback or fallback == primary:
                raise RuntimeError(f"{label} service unavailable: {primary}") from primary_exc
            rospy.logwarn(
                "%s primary service unavailable (%s). Trying fallback: %s",
                label,
                primary,
                fallback,
            )
            try:
                rospy.wait_for_service(fallback, timeout=timeout_sec)
                return
            except Exception as fallback_exc:
                raise RuntimeError(
                    f"{label} services unavailable. primary={primary} fallback={fallback}"
                ) from fallback_exc

    def run(self) -> None:
        rospy.loginfo("Waiting for mux service: %s", self.args.mux_service)
        rospy.wait_for_service(self.args.mux_service, timeout=self.args.service_timeout_sec)
        if self.args.exclusive_master_routing:
            rospy.loginfo(
                "Waiting for slave mux services: %s, %s",
                self.args.left_slave_mux_service,
                self.args.right_slave_mux_service,
            )
            rospy.wait_for_service(self.args.left_slave_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_slave_mux_service, timeout=self.args.service_timeout_sec)
        if self.args.route_torque_feedback:
            rospy.loginfo(
                "Waiting for torque mux services (primary/fallback): left=%s|%s right=%s|%s",
                self.args.left_torque_mux_service,
                self.args.left_torque_mux_service_fallback,
                self.args.right_torque_mux_service,
                self.args.right_torque_mux_service_fallback,
            )
            self._wait_for_service_with_fallback(
                self.args.left_torque_mux_service,
                self.args.left_torque_mux_service_fallback,
                "left torque mux",
            )
            self._wait_for_service_with_fallback(
                self.args.right_torque_mux_service,
                self.args.right_torque_mux_service_fallback,
                "right torque mux",
            )
        if self.args.route_gravity_feedback:
            rospy.loginfo(
                "Waiting for gravity mux services: %s, %s",
                self.args.left_gravity_mux_service,
                self.args.right_gravity_mux_service,
            )
            rospy.wait_for_service(self.args.left_gravity_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_gravity_mux_service, timeout=self.args.service_timeout_sec)
        need_master_cmd_mux = self.args.gravity_switch_align_lock_mode or self.args.align_before_attach or (
            self.args.align_before_restore and self.args.exclusive_master_routing
        )
        if need_master_cmd_mux:
            rospy.loginfo(
                "Waiting for master-cmd mux services: %s, %s",
                self.args.left_master_cmd_mux_service,
                self.args.right_master_cmd_mux_service,
            )
            rospy.wait_for_service(self.args.left_master_cmd_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_master_cmd_mux_service, timeout=self.args.service_timeout_sec)
        self._wait_for_feedback_topics_ready()
        if self.gravity_feedback_auto_disabled:
            rospy.logwarn("Gravity feedback routing is disabled for this run (OPP gravity source unavailable).")
        self._wait_for_alignment_topics_ready()
        self._ensure_arm_ready()

        rospy.loginfo("opp_master_switch ready")
        rospy.loginfo(
            "keys: [1]=follow_left [2]=follow_right [SPACE]=record_start_stop [h]=hold [s]=status [q]=quit "
            "(exclusive_master_routing=%s)",
            self.args.exclusive_master_routing,
        )

        try:
            with RawTerminal():
                while not rospy.is_shutdown():
                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        self._zero_opp_torque_if_stale()
                        if self.recorder is not None:
                            self.recorder.tick()
                        continue

                    ch = sys.stdin.read(1)
                    if ch == "1":
                        self._attach_opp_to_master("left")
                    elif ch == "2":
                        self._attach_opp_to_master("right")
                    elif ch == " ":
                        if self.recorder is None:
                            rospy.logwarn("Recording is disabled (--no-record-enable).")
                        else:
                            self.recorder.toggle_record()
                    elif ch in ("h", "H"):
                        self._enter_hold_mode()
                    elif ch in ("s", "S"):
                        self._print_status()
                    elif ch in ("q", "Q"):
                        if self.recorder is not None and self.recorder.recording:
                            self.recorder.stop_episode("user_quit")
                        self._disconnect_opp_source(restore_master=self.args.restore_on_quit)
                        rospy.loginfo("quit requested")
                        break
        finally:
            if self.recorder is not None:
                self.recorder.shutdown()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mux-service", default="/robot/arm_opp/joint_cmd_mux_select")
    parser.add_argument("--selected-topic", default="/robot/arm_opp/joint_cmd_mux/selected")
    parser.add_argument("--opp-state-topic", default="/robot/arm_opp/joint_states_single")
    parser.add_argument("--enable-service", default="/robot/arm_opp/enable_srv")
    parser.add_argument("--enable-flag-topic", default="/robot/arm_opp/enable_flag")
    parser.add_argument("--block-service", default="/robot/arm_opp/block_arm")
    parser.add_argument("--left-master-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--right-master-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument(
        "--left-master-to-opp-topic",
        default="/robot/arm_opp/cmd_from_left_master",
        help="Deprecated (ignored): legacy transformed-left command topic.",
    )
    parser.add_argument(
        "--right-master-to-opp-topic",
        default="/robot/arm_opp/cmd_from_right_master",
        help="Deprecated (ignored): legacy transformed-right command topic.",
    )
    parser.add_argument(
        "--left-master-to-opp-sign",
        default="1,1,1,1,1,1,1",
        help="Deprecated (ignored): legacy mirror sign map for left master -> OPP.",
    )
    parser.add_argument(
        "--right-master-to-opp-sign",
        default="1,1,1,1,1,1,1",
        help="Deprecated (ignored): legacy mirror sign map for right master -> OPP.",
    )
    parser.add_argument(
        "--left-master-to-opp-offset",
        default="0,0,0,0,0,0,0",
        help="Deprecated (ignored): legacy mirror offset map for left master -> OPP.",
    )
    parser.add_argument(
        "--right-master-to-opp-offset",
        default="0,0,0,0,0,0,0",
        help="Deprecated (ignored): legacy mirror offset map for right master -> OPP.",
    )
    parser.add_argument("--left-slave-mux-service", default="/robot/arm_left/joint_cmd_mux_select")
    parser.add_argument("--right-slave-mux-service", default="/robot/arm_right/joint_cmd_mux_select")
    parser.add_argument("--left-slave-state-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--right-slave-state-topic", default="/robot/arm_right/joint_states_single")
    parser.add_argument("--left-default-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--right-default-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument("--left-detach-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--right-detach-topic", default="/robot/arm_right/vla_joint_cmd")
    parser.add_argument("--left-master-cmd-mux-service", default="/teleop/arm_left/master_cmd_mux_select")
    parser.add_argument("--right-master-cmd-mux-service", default="/teleop/arm_right/master_cmd_mux_select")
    parser.add_argument("--left-master-enable-service", default="/teleop/arm_left/enable_srv")
    parser.add_argument("--right-master-enable-service", default="/teleop/arm_right/enable_srv")
    parser.add_argument("--left-master-block-service", default="/teleop/arm_left/block_arm")
    parser.add_argument("--right-master-block-service", default="/teleop/arm_right/block_arm")
    parser.add_argument("--slave-follow-flag-topic", default="/conrft_robot/slave_follow_flag")
    parser.add_argument("--left-master-force-cmd-topic", default="/teleop/arm_left/master_force_cmd_flag")
    parser.add_argument("--right-master-force-cmd-topic", default="/teleop/arm_right/master_force_cmd_flag")
    parser.add_argument(
        "--align-force-teleop-flag",
        action="store_true",
        default=True,
        help=(
            "Before alignment, publish slave_follow_flag=false so teleop master consumes alignment cmd stream "
            "(default: on)."
        ),
    )
    parser.add_argument(
        "--disable-align-force-teleop-flag",
        action="store_false",
        dest="align_force_teleop_flag",
        help="Disable forced slave_follow_flag=false publish before alignment.",
    )
    parser.add_argument("--align-force-flag-repeat", type=int, default=3)
    parser.add_argument("--align-force-flag-interval-sec", type=float, default=0.05)
    parser.add_argument("--align-flag-subscriber-wait-sec", type=float, default=0.40)
    parser.add_argument(
        "--align-enable-master-arm",
        action="store_true",
        default=True,
        help="Before alignment, best-effort call /teleop/arm_*/enable_srv true (default: on).",
    )
    parser.add_argument(
        "--disable-align-enable-master-arm",
        action="store_false",
        dest="align_enable_master_arm",
        help="Disable master enable_srv pre-step before alignment.",
    )
    parser.add_argument(
        "--align-unblock-master-arm",
        action="store_true",
        default=True,
        help="Before alignment, best-effort call /teleop/arm_*/block_arm false (default: on).",
    )
    parser.add_argument(
        "--disable-align-unblock-master-arm",
        action="store_false",
        dest="align_unblock_master_arm",
        help="Disable master block_arm(false) pre-step before alignment.",
    )
    parser.add_argument(
        "--align-force-master-cmd-mode",
        action="store_true",
        default=True,
        help="Before alignment, publish /teleop/arm_*/master_force_cmd_flag=true (default: on).",
    )
    parser.add_argument(
        "--disable-align-force-master-cmd-mode",
        action="store_false",
        dest="align_force_master_cmd_mode",
        help="Disable force master-cmd mode publish around alignment.",
    )
    parser.add_argument("--align-force-master-cmd-repeat", type=int, default=3)
    parser.add_argument("--align-force-master-cmd-interval-sec", type=float, default=0.03)
    parser.add_argument(
        "--gravity-switch-freeze-mode",
        action="store_true",
        default=True,
        help=(
            "During gravity mux source switch, briefly force master command mode to suppress "
            "transient heaviness/glitches (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-gravity-switch-freeze-mode",
        action="store_false",
        dest="gravity_switch_freeze_mode",
        help="Disable transient freeze guard during gravity mux switching.",
    )
    parser.add_argument(
        "--gravity-switch-settle-sec",
        type=float,
        default=0.08,
        help="Hold freeze window this long after gravity mux switch.",
    )
    parser.add_argument(
        "--gravity-switch-calm-mode",
        action="store_true",
        default=True,
        help=(
            "After gravity mux switch, keep master in force-cmd mode until target gravity topic "
            "passes a calm-window check or times out (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-gravity-switch-calm-mode",
        action="store_false",
        dest="gravity_switch_calm_mode",
        help="Disable calm-window guard after gravity mux switching.",
    )
    parser.add_argument(
        "--gravity-switch-calm-min-sec",
        type=float,
        default=0.20,
        help="Minimum freeze duration for gravity calm window.",
    )
    parser.add_argument(
        "--gravity-switch-calm-max-sec",
        type=float,
        default=0.80,
        help="Maximum freeze duration while waiting gravity source to become calm.",
    )
    parser.add_argument(
        "--gravity-switch-calm-consecutive",
        type=int,
        default=5,
        help="Required consecutive calm samples before releasing freeze.",
    )
    parser.add_argument(
        "--gravity-switch-calm-effort-step-max",
        type=float,
        default=0.12,
        help="Per-sample max allowed effort jump (6 joints) for calm detection.",
    )
    parser.add_argument(
        "--gravity-switch-calm-pos-step-max",
        type=float,
        default=0.03,
        help="Per-sample max allowed position jump (6 joints, rad) for calm detection.",
    )
    parser.add_argument(
        "--gravity-switch-calm-sample-timeout-sec",
        type=float,
        default=0.06,
        help="Single-sample wait timeout while evaluating gravity calm window.",
    )
    parser.add_argument(
        "--gravity-switch-align-lock-mode",
        action="store_true",
        default=True,
        help=(
            "Before switching gravity mux, temporarily switch master-cmd mux to alignment_joint_cmd "
            "and hold the current pose via the alignment command path (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-gravity-switch-align-lock-mode",
        action="store_false",
        dest="gravity_switch_align_lock_mode",
        help="Disable align-hold current-pose pause during gravity mux switching.",
    )
    parser.add_argument(
        "--gravity-switch-align-lock-rate-hz",
        type=float,
        default=80.0,
        help="Publish rate for gravity-switch align-hold command.",
    )
    parser.add_argument(
        "--gravity-switch-align-lock-pre-hold-sec",
        type=float,
        default=0.05,
        help="Warmup hold time after entering align-hold before switching gravity mux.",
    )
    parser.add_argument(
        "--attach-switch-hold-force-cmd",
        action="store_true",
        default=True,
        help=(
            "During attach route (torque/gravity/source select), keep master_force_cmd_flag=true "
            "until post-settle to reduce residual slam (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-attach-switch-hold-force-cmd",
        action="store_false",
        dest="attach_switch_hold_force_cmd",
        help="Disable attach-route force-cmd hold guard.",
    )
    parser.add_argument(
        "--attach-switch-post-settle-sec",
        type=float,
        default=0.16,
        help="Extra settle hold after attach route before releasing force-cmd guard.",
    )
    parser.add_argument("--left-master-default-cmd-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--right-master-default-cmd-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument("--left-master-align-topic", default="/teleop/arm_left/alignment_joint_cmd")
    parser.add_argument("--right-master-align-topic", default="/teleop/arm_right/alignment_joint_cmd")
    parser.add_argument("--left-torque-mux-service", default="/teleop/arm_left/torque_feedback_mux_select")
    parser.add_argument("--right-torque-mux-service", default="/teleop/arm_right/torque_feedback_mux_select")
    parser.add_argument("--left-torque-mux-service-fallback", default="")
    parser.add_argument("--right-torque-mux-service-fallback", default="")
    parser.add_argument("--left-torque-selected-topic", default="/teleop/arm_left/torque_feedback_mux/selected")
    parser.add_argument("--right-torque-selected-topic", default="/teleop/arm_right/torque_feedback_mux/selected")
    parser.add_argument(
        "--mux-select-verify-timeout-sec",
        type=float,
        default=0.30,
        help="Timeout when validating mux selected-topic after service call.",
    )
    parser.add_argument(
        "--strict-torque-mux-verify",
        action="store_true",
        help="Fail fast if torque mux selected topic does not match expected route.",
    )
    parser.add_argument(
        "--transformed-torque-subscriber-wait-sec",
        type=float,
        default=0.35,
        help="Wait this long for transformed OPP torque publisher to gain subscribers after routing.",
    )
    parser.add_argument("--left-gravity-mux-service", default="/teleop/arm_left/gravity_feedback_mux_select")
    parser.add_argument("--right-gravity-mux-service", default="/teleop/arm_right/gravity_feedback_mux_select")
    parser.add_argument("--left-default-torque-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--right-default-torque-topic", default="/robot/arm_right/joint_states_single")
    parser.add_argument("--left-default-gravity-topic", default="/robot/arm_left/joint_states_compensated")
    parser.add_argument("--right-default-gravity-topic", default="/robot/arm_right/joint_states_compensated")
    parser.add_argument("--opp-torque-topic", default="/robot/arm_opp/joint_states_single")
    parser.add_argument("--opp-gravity-topic", default="/robot/arm_opp/joint_states_compensated")
    parser.add_argument("--left-opp-torque-topic", default="/teleop/arm_left/opp_torque_feedback")
    parser.add_argument("--right-opp-torque-topic", default="/teleop/arm_right/opp_torque_feedback")
    parser.add_argument("--opp-torque-sign-left", default="1,1,1,1,1,1")
    parser.add_argument("--opp-torque-sign-right", default="1,1,1,1,1,1")
    parser.add_argument(
        "--invert-opp-torque-left",
        action="store_true",
        help="Invert all 6 OPP torque joints before left-master mapping.",
    )
    parser.add_argument(
        "--invert-opp-torque-right",
        action="store_true",
        help="Invert all 6 OPP torque joints before right-master mapping.",
    )
    parser.add_argument("--opp-torque-scale-left", default="1,1,1,1,1,1")
    parser.add_argument("--opp-torque-scale-right", default="1,1,1,1,1,1")
    parser.add_argument("--opp-torque-max-left", default="0.35,0.35,0.35,0.25,0.25,0.20")
    parser.add_argument("--opp-torque-max-right", default="0.35,0.35,0.35,0.25,0.25,0.20")
    parser.add_argument("--opp-torque-deadband-left", default="0.03,0.03,0.03,0.02,0.02,0.02")
    parser.add_argument("--opp-torque-deadband-right", default="0.03,0.03,0.03,0.02,0.02,0.02")
    parser.add_argument("--opp-torque-rate-limit-left", default="0,0,0,0,0,0")
    parser.add_argument("--opp-torque-rate-limit-right", default="0,0,0,0,0,0")
    parser.add_argument("--opp-torque-alpha-left", type=float, default=0.25)
    parser.add_argument("--opp-torque-alpha-right", type=float, default=0.25)
    parser.add_argument(
        "--opp-torque-ramp-sec",
        type=float,
        default=0.8,
        help="Ramp-in time for OPP torque feedback when attaching a master side.",
    )
    parser.add_argument(
        "--opp-torque-stale-sec",
        type=float,
        default=0.15,
        help="If OPP state is stale while attached, publish zero transformed torque.",
    )
    parser.add_argument(
        "--opp-torque-raw-effort-warn-threshold",
        type=float,
        default=0.001,
        help="Warn if OPP raw effort abs-peak stays below this value while attached.",
    )
    parser.add_argument(
        "--opp-pass-through-gripper-effort",
        action="store_true",
        default=True,
        help="Pass raw OPP gripper effort (joint7) into transformed torque topics.",
    )
    parser.add_argument(
        "--disable-opp-pass-through-gripper-effort",
        action="store_false",
        dest="opp_pass_through_gripper_effort",
        help="Force transformed OPP gripper effort to zero.",
    )
    parser.add_argument("--hold-topic", default="/robot/arm_opp/hold_joint_cmd")
    parser.add_argument("--service-timeout-sec", type=float, default=8.0)
    parser.add_argument("--enable-retries", type=int, default=4)
    parser.add_argument("--enable-retry-interval-sec", type=float, default=1.0)
    parser.add_argument(
        "--enable-flag-fallback",
        action="store_true",
        default=True,
        help="When enable_srv keeps failing, publish enable_flag=true as fallback (default: on).",
    )
    parser.add_argument(
        "--disable-enable-flag-fallback",
        action="store_false",
        dest="enable_flag_fallback",
        help="Disable enable_flag fallback behavior.",
    )
    parser.add_argument("--enable-flag-repeat", type=int, default=3)
    parser.add_argument("--enable-flag-interval-sec", type=float, default=0.05)
    parser.add_argument("--hold-pub-repeat", type=int, default=3)
    parser.add_argument("--hold-pub-interval-sec", type=float, default=0.05)
    parser.add_argument(
        "--skip-arm-ready",
        action="store_true",
        help="Skip startup unblock+enable calls for /robot/arm_opp.",
    )
    parser.add_argument(
        "--strict-arm-ready",
        action="store_true",
        help="Fail startup if arm_opp enable cannot be confirmed.",
    )
    parser.add_argument(
        "--route-torque-feedback",
        action="store_true",
        default=True,
        help=(
            "Switch borrowed master's torque feedback source to OPP, and restore on release/quit "
            "(default: on)."
        ),
    )
    parser.add_argument(
        "--disable-route-torque-feedback",
        action="store_false",
        dest="route_torque_feedback",
        help="Disable torque feedback source routing.",
    )
    parser.add_argument(
        "--route-gravity-feedback",
        action="store_true",
        default=True,
        help=(
            "Switch borrowed master's gravity compensation source to OPP, and restore on release/quit "
            "(default: on)."
        ),
    )
    parser.add_argument(
        "--disable-route-gravity-feedback",
        action="store_false",
        dest="route_gravity_feedback",
        help="Disable gravity compensation source routing.",
    )
    parser.add_argument(
        "--strict-gravity-feedback-topic",
        action="store_true",
        default=True,
        help=(
            "Fail startup if --opp-gravity-topic has no messages (default: on)."
        ),
    )
    parser.add_argument(
        "--no-strict-gravity-feedback-topic",
        action="store_false",
        dest="strict_gravity_feedback_topic",
        help=(
            "Allow startup fallback: auto-disable gravity routing when --opp-gravity-topic has no messages."
        ),
    )
    parser.add_argument(
        "--use-transformed-opp-torque",
        action="store_true",
        default=True,
        help="Route torque feedback through transformed OPP topics (default: on).",
    )
    parser.add_argument(
        "--use-raw-opp-torque",
        action="store_false",
        dest="use_transformed_opp_torque",
        help="Route torque feedback from raw /robot/arm_opp/joint_states_single.",
    )
    parser.add_argument(
        "--exclusive-master-routing",
        action="store_true",
        default=True,
        help=(
            "When OPP borrows left/right master, detach that master's default slave route. "
            "Restore route when returning OPP to hold."
        ),
    )
    parser.add_argument(
        "--disable-exclusive-master-routing",
        action="store_false",
        dest="exclusive_master_routing",
        help="Disable detaching/restoring left-right default slave routing.",
    )
    parser.add_argument(
        "--align-before-restore",
        action="store_true",
        default=True,
        help=(
            "Before restoring a borrowed master back to its slave route, "
            "temporarily align master pose to slave pose via alignment command topic (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-align-before-restore",
        action="store_false",
        dest="align_before_restore",
        help="Disable smooth master alignment before restoring slave route.",
    )
    parser.add_argument(
        "--align-before-attach",
        action="store_true",
        default=True,
        help=(
            "Before attaching OPP to a master side, "
            "temporarily align master pose to OPP pose via alignment command topic (default: on)."
        ),
    )
    parser.add_argument(
        "--disable-align-before-attach",
        action="store_false",
        dest="align_before_attach",
        help="Disable smooth master alignment before attaching OPP route.",
    )
    parser.add_argument("--align-master-rate-hz", type=float, default=80.0)
    parser.add_argument("--align-master-max-joint-speed", type=float, default=0.40)
    parser.add_argument("--align-master-min-duration-sec", type=float, default=0.30)
    parser.add_argument("--align-master-max-duration-sec", type=float, default=2.50)
    parser.add_argument("--align-master-timeout-sec", type=float, default=4.00)
    parser.add_argument("--align-master-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--align-master-settle-sec", type=float, default=0.20)
    parser.add_argument(
        "--align-include-gripper",
        action="store_true",
        default=False,
        help="Include gripper joint in smooth handover alignment.",
    )
    parser.add_argument(
        "--restore-on-quit",
        action="store_true",
        default=True,
        help="Restore borrowed master back to default slave route on quit (default: on).",
    )
    parser.add_argument(
        "--no-restore-on-quit",
        action="store_false",
        dest="restore_on_quit",
        help="Do not restore slave route on quit.",
    )
    parser.add_argument(
        "--record-enable",
        action="store_true",
        default=True,
        help="Enable in-process rosbag recording controlled by SPACE (default: on).",
    )
    parser.add_argument(
        "--no-record-enable",
        action="store_false",
        dest="record_enable",
        help="Disable in-process rosbag recording integration.",
    )
    parser.add_argument(
        "--record-profile",
        default=DEFAULT_RECORD_PROFILE,
        help="YAML profile used for integrated recording.",
    )
    parser.add_argument(
        "--record-camera-transport",
        choices=("profile", "compressed", "raw"),
        default="profile",
        help="Override camera transport from profile for integrated recording.",
    )
    parser.add_argument(
        "--record-session-root",
        default=DEFAULT_RECORD_ROOT,
        help="Root directory for integrated recording session output.",
    )
    parser.add_argument(
        "--record-session-name",
        "--session-name",
        dest="record_session_name",
        default="",
        help=(
            "Optional fixed session directory name (default: auto timestamp). "
            "Alias: --session-name."
        ),
    )
    parser.add_argument(
        "--record-prefix",
        default="intervention",
        help="Session name prefix when --record-session-name is empty.",
    )
    parser.add_argument(
        "--record-wait-timeout-sec",
        type=float,
        default=30.0,
        help="Preflight wait timeout before starting recording.",
    )
    parser.add_argument(
        "--record-min-free-gb",
        type=float,
        default=5.0,
        help="Minimum free disk threshold during recording.",
    )
    parser.add_argument(
        "--record-debounce-ms",
        type=int,
        default=250,
        help="SPACE key debounce interval in milliseconds for recording toggle.",
    )
    parser.add_argument(
        "--record-status-interval-sec",
        type=float,
        default=2.0,
        help="Runtime status print interval while recording.",
    )
    parser.add_argument(
        "--record-sub-queue-size",
        type=int,
        default=50,
        help="Subscriber queue size per recorded topic.",
    )
    parser.add_argument(
        "--record-opp-joint-state-topic",
        default=DEFAULT_RECORD_OPP_JOINT_STATE_TOPIC,
        help="Third-arm joint-state topic that must be recorded.",
    )
    parser.add_argument(
        "--record-opp-effort-topic",
        default=DEFAULT_RECORD_OPP_EFFORT_TOPIC,
        help="Third-arm effort-bearing topic that must be recorded.",
    )
    parser.add_argument(
        "--record-bag-compression",
        choices=("none", "bz2", "lz4"),
        default="lz4",
        help="Compression mode for in-process bag writing.",
    )
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()

    if args.service_timeout_sec <= 0.0:
        parser.error("--service-timeout-sec must be > 0")
    if args.hold_pub_repeat <= 0:
        parser.error("--hold-pub-repeat must be > 0")
    if args.hold_pub_interval_sec < 0.0:
        parser.error("--hold-pub-interval-sec must be >= 0")
    if args.enable_retries <= 0:
        parser.error("--enable-retries must be > 0")
    if args.enable_retry_interval_sec < 0.0:
        parser.error("--enable-retry-interval-sec must be >= 0")
    if args.enable_flag_repeat <= 0:
        parser.error("--enable-flag-repeat must be > 0")
    if args.enable_flag_interval_sec < 0.0:
        parser.error("--enable-flag-interval-sec must be >= 0")
    if args.align_force_flag_repeat <= 0:
        parser.error("--align-force-flag-repeat must be > 0")
    if args.align_force_flag_interval_sec < 0.0:
        parser.error("--align-force-flag-interval-sec must be >= 0")
    if args.align_flag_subscriber_wait_sec < 0.0:
        parser.error("--align-flag-subscriber-wait-sec must be >= 0")
    if args.align_force_master_cmd_repeat <= 0:
        parser.error("--align-force-master-cmd-repeat must be > 0")
    if args.align_force_master_cmd_interval_sec < 0.0:
        parser.error("--align-force-master-cmd-interval-sec must be >= 0")
    if args.gravity_switch_settle_sec < 0.0:
        parser.error("--gravity-switch-settle-sec must be >= 0")
    if args.gravity_switch_calm_min_sec < 0.0:
        parser.error("--gravity-switch-calm-min-sec must be >= 0")
    if args.gravity_switch_calm_max_sec < args.gravity_switch_calm_min_sec:
        parser.error("--gravity-switch-calm-max-sec must be >= --gravity-switch-calm-min-sec")
    if args.gravity_switch_calm_consecutive <= 0:
        parser.error("--gravity-switch-calm-consecutive must be > 0")
    if args.gravity_switch_calm_effort_step_max < 0.0:
        parser.error("--gravity-switch-calm-effort-step-max must be >= 0")
    if args.gravity_switch_calm_pos_step_max < 0.0:
        parser.error("--gravity-switch-calm-pos-step-max must be >= 0")
    if args.gravity_switch_calm_sample_timeout_sec <= 0.0:
        parser.error("--gravity-switch-calm-sample-timeout-sec must be > 0")
    if args.gravity_switch_align_lock_rate_hz <= 0.0:
        parser.error("--gravity-switch-align-lock-rate-hz must be > 0")
    if args.gravity_switch_align_lock_pre_hold_sec < 0.0:
        parser.error("--gravity-switch-align-lock-pre-hold-sec must be >= 0")
    if args.attach_switch_post_settle_sec < 0.0:
        parser.error("--attach-switch-post-settle-sec must be >= 0")
    if args.opp_torque_alpha_left < 0.0 or args.opp_torque_alpha_left > 1.0:
        parser.error("--opp-torque-alpha-left must be in [0,1]")
    if args.opp_torque_alpha_right < 0.0 or args.opp_torque_alpha_right > 1.0:
        parser.error("--opp-torque-alpha-right must be in [0,1]")
    if args.opp_torque_ramp_sec < 0.0:
        parser.error("--opp-torque-ramp-sec must be >= 0")
    if args.opp_torque_stale_sec < 0.0:
        parser.error("--opp-torque-stale-sec must be >= 0")
    if args.opp_torque_raw_effort_warn_threshold < 0.0:
        parser.error("--opp-torque-raw-effort-warn-threshold must be >= 0")
    if args.mux_select_verify_timeout_sec < 0.0:
        parser.error("--mux-select-verify-timeout-sec must be >= 0")
    if args.transformed_torque_subscriber_wait_sec < 0.0:
        parser.error("--transformed-torque-subscriber-wait-sec must be >= 0")
    if args.align_master_rate_hz <= 0.0:
        parser.error("--align-master-rate-hz must be > 0")
    if args.align_master_max_joint_speed <= 0.0:
        parser.error("--align-master-max-joint-speed must be > 0")
    if args.align_master_min_duration_sec < 0.0:
        parser.error("--align-master-min-duration-sec must be >= 0")
    if args.align_master_max_duration_sec < args.align_master_min_duration_sec:
        parser.error("--align-master-max-duration-sec must be >= --align-master-min-duration-sec")
    if args.align_master_timeout_sec <= 0.0:
        parser.error("--align-master-timeout-sec must be > 0")
    if args.align_master_tolerance_rad <= 0.0:
        parser.error("--align-master-tolerance-rad must be > 0")
    if args.align_master_settle_sec < 0.0:
        parser.error("--align-master-settle-sec must be >= 0")
    if args.record_wait_timeout_sec <= 0.0:
        parser.error("--record-wait-timeout-sec must be > 0")
    if args.record_min_free_gb < 0.0:
        parser.error("--record-min-free-gb must be >= 0")
    if args.record_debounce_ms < 0:
        parser.error("--record-debounce-ms must be >= 0")
    if args.record_status_interval_sec <= 0.0:
        parser.error("--record-status-interval-sec must be > 0")
    if args.record_sub_queue_size <= 0:
        parser.error("--record-sub-queue-size must be > 0")
    if not str(args.record_opp_joint_state_topic).strip():
        parser.error("--record-opp-joint-state-topic must not be empty")
    if not str(args.record_opp_effort_topic).strip():
        parser.error("--record-opp-effort-topic must not be empty")

    rospy.init_node("opp_master_switch")

    switcher = OppMasterSwitcher(args)
    try:
        switcher.run()
    except Exception as exc:
        rospy.logerr("opp_master_switch failed: %s", str(exc))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
