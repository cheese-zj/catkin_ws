#!/usr/bin/env python3
"""Keyboard-driven ACT rosbag recorder.

Controls:
  SPACE: start/stop current episode recording
  Q:     stop recording if needed and quit
"""

from __future__ import annotations

import argparse
import atexit
import copy
import datetime as dt
import json
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import termios
import threading
import time
import tty
import xmlrpc.client
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyYAML is required (python3-yaml).") from exc


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def local_now_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def sanitize_topic(topic: str) -> str:
    return topic.strip("/").replace("/", "_").replace(":", "_") or "root"


def dedup_keep_order(items: List[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


class RawTerminal:
    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise RuntimeError("Interactive TTY is required for keyboard control.")
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


class Recorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.profile = self._load_profile(args.profile)
        if args.camera_transport != "profile":
            self.profile["camera_transport"] = args.camera_transport
        self._validate_profile(self.profile)

        self.camera_transport = self.profile["camera_transport"]
        self.required_topics = list(self.profile.get("required_topics", []))
        self.camera_streams = list(self.profile.get("camera_streams", []))
        self.optional_topics = list(self.profile.get("optional_topics", []))
        self.hz_monitor_topics = list(self.profile.get("hz_monitor_topics", []))

        self.camera_record_topics = self._build_camera_record_topics()
        self.camera_health_topics = [entry["image_base"] for entry in self.camera_streams]
        self.camera_info_topics = [entry["camera_info"] for entry in self.camera_streams]
        self.required_presence_topics = self._build_required_presence_topics()
        self.required_joint_topics = [
            "/robot/arm_left/joint_states_single",
            "/robot/arm_right/joint_states_single",
            "/teleop/arm_left/joint_states_single",
            "/teleop/arm_right/joint_states_single",
        ]
        self.profile_record_topics = dedup_keep_order(self.required_topics + self.camera_record_topics)

        self.session_root = Path(args.session_root).expanduser().resolve()
        self.session_name = args.session_name or f"{args.prefix}_{local_now_tag()}"
        self.session_dir = self.session_root / self.session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.next_episode_index = self._scan_next_episode_index()

        self.recording = False
        self.shutdown_requested = False
        self.signal_requested = False
        self.pending_stop_reason: Optional[str] = None
        self.pending_lock = threading.Lock()

        self.rosbag_proc: Optional[subprocess.Popen] = None
        self.rosbag_log_handle = None
        self.hz_monitors: Dict[str, Tuple[subprocess.Popen, Path, object]] = {}
        self.debug_log_paths: List[Path] = []

        self.current_episode_name: Optional[str] = None
        self.current_episode_dir: Optional[Path] = None
        self.current_started_utc: Optional[str] = None
        self.current_started_monotonic: Optional[float] = None
        self.current_topics: List[str] = []
        self.current_optional_recorded: List[str] = []
        self.current_optional_missing: List[str] = []
        self.current_warnings: List[str] = []
        self.current_preflight: Dict[str, object] = {}
        self.last_stop_monotonic: Optional[float] = None
        self.last_stop_reason: Optional[str] = None

        self.debounce_sec = max(0.0, args.debounce_ms / 1000.0)
        self.last_space_ts = 0.0
        self.record_status_interval_sec = max(0.1, args.record_status_interval_sec)
        self.save_status_interval_sec = max(0.1, args.save_status_interval_sec)
        self.last_record_status_ts = 0.0
        self.last_save_status_ts = 0.0
        self.use_color = self._supports_color()

        self.key_rosparams = [
            "/piper_ctrl_left_node/can_port",
            "/piper_ctrl_right_node/can_port",
            "/piper_teleop_left_node/can_port",
            "/piper_teleop_right_node/can_port",
            "/piper_teleop_left_node/gripper_reverse",
            "/piper_teleop_right_node/gripper_reverse",
            "/piper_teleop_left_node/mit/enable_tor",
            "/piper_teleop_right_node/mit/enable_tor",
            "/piper_teleop_left_node/mit/gravity_mix_mode",
            "/piper_teleop_right_node/mit/gravity_mix_mode",
            "/piper_teleop_left_node/mit/torque_scale",
            "/piper_teleop_right_node/mit/torque_scale",
            "/piper_teleop_left_node/mit/torque_feedback_sign",
            "/piper_teleop_right_node/mit/torque_feedback_sign",
            "/piper_teleop_left_node/master_slave/enable",
            "/piper_teleop_right_node/master_slave/enable",
        ]

        self.save_workers = max(1, min(int(args.save_workers), 8))
        self.save_executor = ThreadPoolExecutor(
            max_workers=self.save_workers,
            thread_name_prefix="act_save",
        )
        self.save_executor_shutdown = False
        self.save_lock = threading.Lock()
        self.next_save_job_id = 1
        self.save_jobs: Dict[int, Dict[str, object]] = {}
        self.save_futures: Dict[int, Future] = {}

        self._ensure_tools()
        atexit.register(self._atexit_cleanup)

        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def _load_profile(self, path: str) -> dict:
        profile_path = Path(path).expanduser().resolve()
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
        for entry in profile["camera_streams"]:
            if "image_base" not in entry or "camera_info" not in entry:
                raise RuntimeError("camera_streams entries must include image_base and camera_info")

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
        return dedup_keep_order(topics)

    def _build_required_presence_topics(self) -> List[str]:
        topics = list(self.required_topics) + list(self.camera_info_topics)
        if self.camera_transport == "raw":
            topics.extend(self.camera_health_topics)
        return dedup_keep_order(topics)

    def _ensure_tools(self) -> None:
        missing = []
        for cmd in ("rosbag", "rostopic", "rosparam"):
            if shutil.which(cmd) is None:
                missing.append(cmd)
        if missing:
            raise RuntimeError(f"Missing required command(s) in PATH: {', '.join(missing)}")

    def _scan_next_episode_index(self) -> int:
        used_indices = set()
        for entry in self.session_dir.iterdir():
            if not entry.is_dir():
                continue
            match = re.match(r"^episode_(\d+)$", entry.name)
            if match:
                used_indices.add(int(match.group(1)))

        idx = 1
        while idx in used_indices:
            idx += 1
        return idx

    def _run_cmd(
        self,
        cmd: List[str],
        timeout: Optional[float] = None,
        check: bool = False,
    ) -> subprocess.CompletedProcess:
        result = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        if check and result.returncode != 0:
            joined = " ".join(cmd)
            raise RuntimeError(f"Command failed ({joined}): {result.stderr.strip()}")
        return result

    def _ros_master_ok(self) -> Tuple[bool, str]:
        uri = os.environ.get("ROS_MASTER_URI", "")
        if not uri:
            return False, "ROS_MASTER_URI is not set."
        try:
            proxy = xmlrpc.client.ServerProxy(uri)
            code, msg, _val = proxy.getUri("/record_act_keyboard")
            if code == 1:
                return True, msg
            return False, msg
        except Exception as exc:
            return False, str(exc)

    def _topic_list(self) -> List[str]:
        res = self._run_cmd(["rostopic", "list"], timeout=8.0)
        if res.returncode != 0:
            return []
        return [line.strip() for line in res.stdout.splitlines() if line.strip()]

    def _topic_has_publishers(self, topic: str) -> bool:
        res = self._run_cmd(["rostopic", "info", topic], timeout=8.0)
        if res.returncode != 0:
            return False
        output = res.stdout
        if re.search(r"Publishers:\s*None", output):
            return False

        in_publishers = False
        pub_count = 0
        for line in output.splitlines():
            stripped = line.strip()
            if stripped.startswith("Publishers:"):
                in_publishers = True
                continue
            if stripped.startswith("Subscribers:"):
                break
            if in_publishers and stripped.startswith("*"):
                pub_count += 1
        return pub_count > 0

    def _check_disk_ok(self) -> Tuple[bool, float]:
        usage = shutil.disk_usage(self.session_root)
        free_gb = usage.free / float(1024 ** 3)
        return free_gb >= self.args.min_free_gb, free_gb

    def _fetch_rosparams(self) -> Dict[str, object]:
        out: Dict[str, object] = {}
        for key in self.key_rosparams:
            res = self._run_cmd(["rosparam", "get", key], timeout=2.0)
            if res.returncode == 0:
                value = res.stdout.strip()
                try:
                    parsed = yaml.safe_load(value)
                    out[key] = parsed
                except Exception:
                    out[key] = value
            else:
                out[key] = None
        return out

    @staticmethod
    def _local_now_str() -> str:
        return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def _format_duration(duration_sec: Optional[float]) -> str:
        if duration_sec is None:
            return "unknown"
        return f"{duration_sec:.2f}s"

    @staticmethod
    def _supports_color() -> bool:
        if os.environ.get("NO_COLOR"):
            return False
        if os.environ.get("FORCE_COLOR"):
            return True
        term = os.environ.get("TERM", "")
        return bool(sys.stdout.isatty() and term and term.lower() != "dumb")

    def _colorize(self, text: str, color: Optional[str]) -> str:
        if not self.use_color or not color:
            return text
        palette = {
            "green": "\033[32m",
            "red": "\033[31m",
            "reset": "\033[0m",
        }
        code = palette.get(color)
        if code is None:
            return text
        return f"{code}{text}{palette['reset']}"

    def _status_color(self, status: str) -> Optional[str]:
        s = status.upper()
        if s in ("OK", "DONE", "RUN", "WAIT", "KEPT"):
            return "green"
        if s in ("WARN", "FAIL", "FAILED", "ERROR"):
            return "red"
        return None

    def _print_colored(self, text: str, color: Optional[str] = None) -> None:
        print(self._colorize(text, color))

    @staticmethod
    def _progress_bar(done: int, total: int, width: int = 24) -> str:
        if total <= 0:
            total = 1
        done = max(0, min(done, total))
        filled = int(round((done / float(total)) * width))
        return "[" + ("#" * filled) + ("-" * (width - filled)) + "]"

    def _print_save_progress(self, done: int, total: int, label: str, status: str) -> None:
        bar = self._progress_bar(done, total)
        color = self._status_color(status)
        self._print_colored(f"[save] {bar} {done:>2}/{total:<2} {label} [{status}]", color=color)
        sys.stdout.flush()

    def _update_save_job(
        self,
        job_id: int,
        *,
        state: Optional[str] = None,
        step: Optional[int] = None,
        total: Optional[int] = None,
        stage: Optional[str] = None,
        status: Optional[str] = None,
        error: Optional[str] = None,
        result: Optional[Dict[str, object]] = None,
    ) -> None:
        with self.save_lock:
            job = self.save_jobs.get(job_id)
            if job is None:
                return
            if state is not None:
                job["state"] = state
            if step is not None:
                job["step"] = step
            if total is not None:
                job["total"] = total
            if stage is not None:
                job["stage"] = stage
            if status is not None:
                job["status"] = status
            if error is not None:
                job["error"] = error
            if result is not None:
                job["result"] = result
            job["updated_monotonic"] = time.monotonic()

    def _snapshot_save_jobs(self) -> List[Tuple[int, Dict[str, object]]]:
        with self.save_lock:
            return sorted(
                [(job_id, dict(info)) for job_id, info in self.save_jobs.items()],
                key=lambda x: x[0],
            )

    def _pending_save_count(self) -> int:
        with self.save_lock:
            return sum(1 for info in self.save_jobs.values() if info.get("state") in ("queued", "running"))

    def _mark_save_reported(self, job_ids: List[int]) -> None:
        if not job_ids:
            return
        with self.save_lock:
            for job_id in job_ids:
                info = self.save_jobs.get(job_id)
                if info is not None:
                    info["reported_done"] = True

    def _on_save_future_done(self, job_id: int, fut: Future) -> None:
        try:
            result = fut.result()
            with self.save_lock:
                total = int(self.save_jobs.get(job_id, {}).get("total", 1))
            self._update_save_job(
                job_id,
                state="done",
                step=total,
                stage="complete",
                status=str(result.get("save_status", "OK")),
                result=result,
            )
        except Exception as exc:
            self._update_save_job(
                job_id,
                state="failed",
                stage="error",
                status="FAIL",
                error=str(exc),
            )

    def _submit_save_job(self, job_data: Dict[str, object]) -> int:
        with self.save_lock:
            job_id = self.next_save_job_id
            self.next_save_job_id += 1
            self.save_jobs[job_id] = {
                "episode_name": job_data.get("episode_name"),
                "state": "queued",
                "step": 0,
                "total": 6,
                "stage": "queued",
                "status": "WAIT",
                "reported_done": False,
                "error": "",
                "result": None,
                "updated_monotonic": time.monotonic(),
            }
        fut = self.save_executor.submit(self._finalize_episode_job, job_id, job_data)
        fut.add_done_callback(lambda done_fut, jid=job_id: self._on_save_future_done(jid, done_fut))
        with self.save_lock:
            self.save_futures[job_id] = fut
        return job_id

    def _emit_save_progress(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self.last_save_status_ts) < self.save_status_interval_sec:
            return

        snapshots = self._snapshot_save_jobs()
        if not snapshots:
            return

        active = []
        finished = []
        for job_id, info in snapshots:
            state = str(info.get("state", ""))
            if state in ("queued", "running"):
                active.append((job_id, info))
            elif state in ("done", "failed") and not bool(info.get("reported_done")):
                finished.append((job_id, info))

        if not active and not finished and not force:
            return

        for job_id, info in active:
            bar = self._progress_bar(int(info.get("step", 0)), int(info.get("total", 1)))
            line = (
                f"[save][job={job_id}][{info.get('episode_name')}] "
                f"{bar} {int(info.get('step', 0)):>2}/{int(info.get('total', 1)):<2} "
                f"{info.get('stage', '')} [{info.get('status', 'RUN')}]"
            )
            self._print_colored(line, color=self._status_color(str(info.get("status", "RUN"))))

        reported_ids: List[int] = []
        for job_id, info in finished:
            reported_ids.append(job_id)
            if info.get("state") == "done":
                result = info.get("result") or {}
                status_text = str(result.get("save_status", "OK"))
                line = (
                    f"[save][job={job_id}][{info.get('episode_name')}] DONE "
                    f"status={status_text} metadata={result.get('metadata_path')}"
                )
                self._print_colored(line, color=self._status_color(status_text))
            else:
                line = (
                    f"[save][job={job_id}][{info.get('episode_name')}] FAILED "
                    f"error={info.get('error', 'unknown')}"
                )
                self._print_colored(line, color="red")

        self._mark_save_reported(reported_ids)
        self.last_save_status_ts = now
        sys.stdout.flush()

    def _emit_runtime_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if self.recording and self.current_started_monotonic is not None:
            if force or (now - self.last_record_status_ts) >= self.record_status_interval_sec:
                elapsed = now - self.current_started_monotonic
                print(
                    f"[record] ACTIVE {self.current_episode_name} elapsed={self._format_duration(elapsed)} "
                    f"pending_saves={self._pending_save_count()}"
                )
                self.last_record_status_ts = now
                sys.stdout.flush()
        self._emit_save_progress(force=force)

    def _print_idle_help(self) -> None:
        print("")
        print("== ACT Keyboard Recorder ==")
        print(f"Session: {self.session_name}")
        print(f"Session dir: {self.session_dir}")
        print(f"Camera transport: {self.camera_transport}")
        print(f"Next episode: episode_{self.next_episode_index:03d}")
        print(f"Async save workers: {self.save_workers}")
        print("Keys: SPACE=start/stop, Q=quit")
        print("")
        sys.stdout.flush()

    def _wait_preflight(
        self,
        timeout_sec: float,
        skip_camera_publishers: bool = False,
    ) -> Tuple[bool, Dict[str, object]]:
        started = time.time()
        deadline = started + timeout_sec
        poll_sec = 0.2 if timeout_sec <= 5.0 else 1.0
        last_master_msg = ""
        last_missing_presence: List[str] = []
        last_joint_no_publisher: List[str] = []
        last_camera_unhealthy: List[str] = []

        while time.time() < deadline:
            ok_master, master_msg = self._ros_master_ok()
            if not ok_master:
                last_master_msg = master_msg
                time.sleep(poll_sec)
                continue

            topics = set(self._topic_list())
            missing_presence = sorted([t for t in self.required_presence_topics if t not in topics])
            if missing_presence:
                last_missing_presence = missing_presence
                time.sleep(poll_sec)
                continue
            last_missing_presence = []

            joint_no_publisher: List[str] = []
            for topic in self.required_joint_topics:
                if topic in topics and not self._topic_has_publishers(topic):
                    joint_no_publisher.append(topic)
            if joint_no_publisher:
                last_joint_no_publisher = joint_no_publisher
                time.sleep(poll_sec)
                continue
            last_joint_no_publisher = []

            camera_unhealthy: List[str] = []
            if not skip_camera_publishers:
                for cam_topic in self.camera_health_topics:
                    if not self._topic_has_publishers(cam_topic):
                        camera_unhealthy.append(cam_topic)
                if camera_unhealthy:
                    last_camera_unhealthy = camera_unhealthy
                    time.sleep(poll_sec)
                    continue
            last_camera_unhealthy = []

            disk_ok, free_gb = self._check_disk_ok()
            if not disk_ok:
                return False, {
                    "reason": "disk_low",
                    "free_gb": free_gb,
                    "min_free_gb": self.args.min_free_gb,
                }

            optional_present = sorted([t for t in self.optional_topics if t in topics])
            optional_missing = sorted([t for t in self.optional_topics if t not in topics])
            return True, {
                "reason": "ok",
                "wait_sec": round(time.time() - started, 3),
                "ros_master_msg": master_msg,
                "required_presence_topics": list(self.required_presence_topics),
                "required_joint_topics": list(self.required_joint_topics),
                "camera_health_topics": list(self.camera_health_topics),
                "skip_camera_publishers": bool(skip_camera_publishers),
                "optional_present": optional_present,
                "optional_missing": optional_missing,
            }

        return False, {
            "reason": "preflight_timeout",
            "wait_timeout_sec": timeout_sec,
            "required_presence_topics": list(self.required_presence_topics),
            "required_joint_topics": list(self.required_joint_topics),
            "camera_health_topics": list(self.camera_health_topics),
            "skip_camera_publishers": bool(skip_camera_publishers),
            "last_master_msg": last_master_msg,
            "last_missing_presence_topics": last_missing_presence,
            "last_joint_no_publisher_topics": last_joint_no_publisher,
            "last_camera_unhealthy_topics": last_camera_unhealthy,
        }

    def _get_next_episode_slot(self) -> Tuple[int, str, Path]:
        idx = self._scan_next_episode_index()
        while True:
            name = f"episode_{idx:03d}"
            ep_dir = self.session_dir / name
            if not ep_dir.exists():
                return idx, name, ep_dir
            idx += 1

    def _start_hz_monitors(self, episode_dir: Path) -> None:
        self.hz_monitors = {}
        for topic in self.hz_monitor_topics:
            log_path = episode_dir / f"hz_{sanitize_topic(topic)}.log"
            fh = log_path.open("w", encoding="utf-8")
            self.debug_log_paths.append(log_path)
            cmd = ["rostopic", "hz", "-w", str(self.args.hz_window), topic]
            proc = subprocess.Popen(
                cmd,
                stdout=fh,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True,
            )
            self.hz_monitors[topic] = (proc, log_path, fh)

    def _stop_hz_monitors(self) -> Dict[str, Path]:
        log_paths: Dict[str, Path] = {}
        for topic, (proc, log_path, fh) in self.hz_monitors.items():
            self._stop_process_group(proc, name=f"hz:{topic}")
            fh.flush()
            fh.close()
            log_paths[topic] = log_path
        self.hz_monitors = {}
        return log_paths

    def _cleanup_debug_logs(self, paths: List[Path], warnings: Optional[List[str]] = None) -> List[str]:
        removed: List[str] = []
        for path in paths:
            if not path.exists():
                continue
            try:
                path.unlink()
                removed.append(str(path))
            except Exception as exc:
                if warnings is not None:
                    warnings.append(f"Failed to remove debug log {path.name}: {exc}")
                else:
                    self.current_warnings.append(f"Failed to remove debug log {path.name}: {exc}")
        return removed

    def _parse_hz_log(self, path: Path) -> Dict[str, object]:
        if not path.exists():
            return {"status": "no_log"}
        content = path.read_text(encoding="utf-8", errors="replace")
        pattern = re.compile(
            r"average rate:\s*([0-9.]+).*?"
            r"min:\s*([0-9.]+)s\s+max:\s*([0-9.]+)s\s+std dev:\s*([0-9.]+)s\s+window:\s*([0-9]+)",
            re.DOTALL,
        )
        matches = list(pattern.finditer(content))
        if not matches:
            if "no new messages" in content.lower():
                return {"status": "no_messages"}
            return {"status": "unavailable"}
        match = matches[-1]
        return {
            "status": "ok",
            "avg_hz": float(match.group(1)),
            "min_period_s": float(match.group(2)),
            "max_period_s": float(match.group(3)),
            "std_dev_s": float(match.group(4)),
            "window": int(match.group(5)),
        }

    def _stop_process_group(self, proc: subprocess.Popen, name: str) -> bool:
        if proc.poll() is not None:
            return True
        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            return proc.poll() is not None

        sequence = [
            (signal.SIGINT, 5.0),
            (signal.SIGTERM, 3.0),
            (signal.SIGKILL, 1.0),
        ]
        for sig, wait_sec in sequence:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return True
            except Exception:
                pass

            deadline = time.time() + wait_sec
            while time.time() < deadline:
                if proc.poll() is not None:
                    return True
                time.sleep(0.1)

        return proc.poll() is not None

    def _collect_bag_files(self, episode_dir: Path) -> List[Path]:
        files = sorted(episode_dir.glob("episode*.bag*"))
        return [f for f in files if f.is_file()]

    def _aggregate_bag_stats(
        self,
        bag_files: List[Path],
        on_bag_done: Optional[Callable[[int, int, Path], None]] = None,
    ) -> Dict[str, object]:
        total_size = 0
        total_messages = 0
        min_start = None
        max_end = None
        per_topic: Dict[str, Dict[str, object]] = {}
        parse_warnings: List[str] = []

        total_bags = len(bag_files)
        for idx, bag_file in enumerate(bag_files, start=1):
            try:
                try:
                    total_size += bag_file.stat().st_size
                except Exception:
                    pass

                res = self._run_cmd(["rosbag", "info", "--yaml", str(bag_file)], timeout=20.0)
                if res.returncode != 0:
                    parse_warnings.append(f"rosbag info failed: {bag_file.name}")
                    continue
                try:
                    info = yaml.safe_load(res.stdout) or {}
                except Exception:
                    parse_warnings.append(f"yaml parse failed: {bag_file.name}")
                    continue

                start = info.get("start")
                end = info.get("end")
                messages = info.get("messages", 0)
                if isinstance(start, (int, float)):
                    min_start = start if min_start is None else min(min_start, start)
                if isinstance(end, (int, float)):
                    max_end = end if max_end is None else max(max_end, end)
                if isinstance(messages, int):
                    total_messages += messages

                for t in info.get("topics", []) or []:
                    topic_name = t.get("topic")
                    if not topic_name:
                        continue
                    entry = per_topic.setdefault(
                        topic_name,
                        {
                            "messages": 0,
                            "type": t.get("type"),
                            "bags": 0,
                        },
                    )
                    entry["messages"] += int(t.get("messages", 0))
                    entry["bags"] += 1
            finally:
                if on_bag_done is not None:
                    on_bag_done(idx, total_bags, bag_file)

        duration = None
        if min_start is not None and max_end is not None:
            duration = float(max_end) - float(min_start)

        return {
            "bag_files": [str(f) for f in bag_files],
            "bag_count": len(bag_files),
            "total_size_bytes": total_size,
            "total_messages": total_messages,
            "start_time": min_start,
            "end_time": max_end,
            "duration_sec": duration,
            "per_topic": per_topic,
            "parse_warnings": parse_warnings,
        }

    def _finalize_episode_job(self, job_id: int, job_data: Dict[str, object]) -> Dict[str, object]:
        self._update_save_job(job_id, state="running", total=1, step=0, stage="init", status="RUN")

        episode_name = str(job_data["episode_name"])
        episode_dir = Path(str(job_data["episode_dir"]))
        warnings = list(job_data.get("warnings", []))

        bag_files = self._collect_bag_files(episode_dir)
        hz_stats: Dict[str, object] = {}
        hz_log_paths = dict(job_data.get("hz_log_paths", {}))
        bag_steps = max(1, len(bag_files))
        hz_steps = max(1, len(hz_log_paths))
        total_steps = 1 + bag_steps + hz_steps + 3
        step = 1
        self._update_save_job(
            job_id,
            total=total_steps,
            step=step,
            stage=f"collect bag files ({len(bag_files)})",
            status="OK" if bag_files else "WARN",
        )

        if bag_files:
            def _on_bag_done(idx: int, total: int, bag_file: Path) -> None:
                nonlocal step
                step += 1
                self._update_save_job(
                    job_id,
                    step=step,
                    stage=f"bag info {idx}/{total}: {bag_file.name}",
                    status="RUN",
                )

            bag_stats = self._aggregate_bag_stats(bag_files, on_bag_done=_on_bag_done)
        else:
            bag_stats = self._aggregate_bag_stats([])
            step += 1
            self._update_save_job(
                job_id,
                step=step,
                stage="bag info skipped (no bag files)",
                status="WARN",
            )

        for warning in bag_stats.get("parse_warnings", []):
            warnings.append(str(warning))
        if not bag_files:
            warnings.append("No bag files found in episode directory.")

        if hz_log_paths:
            total_hz = len(hz_log_paths)
            for idx, (topic, log_path) in enumerate(hz_log_paths.items(), start=1):
                hz_result = self._parse_hz_log(Path(str(log_path)))
                hz_stats[str(topic)] = hz_result
                step += 1
                hz_status = "OK" if hz_result.get("status") == "ok" else "WARN"
                self._update_save_job(
                    job_id,
                    step=step,
                    stage=f"hz {idx}/{total_hz}: {topic}",
                    status=hz_status,
                )
        else:
            step += 1
            self._update_save_job(
                job_id,
                step=step,
                stage="hz logs skipped (none)",
                status="WARN",
            )

        debug_paths = [Path(str(p)) for p in list(job_data.get("debug_log_paths", []))]
        existing_debug_logs = [p for p in debug_paths if p.exists()]
        removed_debug_logs: List[str] = []
        if bool(job_data.get("keep_debug_logs", False)):
            kept_debug_logs = [str(p) for p in existing_debug_logs]
            debug_log_status = "KEPT"
        else:
            removed_debug_logs = self._cleanup_debug_logs(existing_debug_logs, warnings=warnings)
            kept_debug_logs = [str(p) for p in existing_debug_logs if p.exists()]
            debug_log_status = "OK"
        step += 1
        self._update_save_job(job_id, step=step, stage="debug logs", status=debug_log_status)

        if str(job_data.get("camera_transport")) == "compressed":
            per_topic = bag_stats.get("per_topic", {})
            for entry in list(job_data.get("camera_streams", [])):
                if not isinstance(entry, dict):
                    continue
                image_base = entry.get("image_base")
                if not image_base:
                    continue
                compressed_topic = f"{image_base}/compressed"
                info = per_topic.get(compressed_topic)
                if not info or int(info.get("messages", 0)) == 0:
                    warnings.append(f"Compressed stream has zero messages: {compressed_topic}")

        per_topic = bag_stats.get("per_topic", {})
        for topic in self.required_joint_topics:
            info = per_topic.get(topic)
            msg_count = int(info.get("messages", 0)) if isinstance(info, dict) else 0
            if msg_count <= 0:
                warnings.append(f"Required joint topic has zero messages: {topic}")

        key_rosparams = self._fetch_rosparams()
        step += 1
        self._update_save_job(job_id, step=step, stage="build metadata", status="OK")

        metadata = {
            "schema_version": 1,
            "session_name": job_data.get("session_name"),
            "episode_name": episode_name,
            "episode_index": int(episode_name.split("_")[-1]),
            "episode_dir": str(episode_dir),
            "started_at_utc": job_data.get("started_at_utc"),
            "ended_at_utc": job_data.get("ended_at_utc"),
            "duration_sec": job_data.get("duration_sec"),
            "stop_reason": job_data.get("stop_reason"),
            "recording": {
                "compression": "lz4",
                "split_enabled": bool(job_data.get("split_enabled", False)),
                "split_size_mb": job_data.get("split_size_mb"),
                "camera_transport": job_data.get("camera_transport"),
                "keep_debug_logs": bool(job_data.get("keep_debug_logs", False)),
            },
            "topics": {
                "profile_required": list(job_data.get("required_topics", [])),
                "required_joint_topics": list(self.required_joint_topics),
                "profile_camera_record_topics": list(job_data.get("camera_record_topics", [])),
                "recorded_topics": list(job_data.get("recorded_topics", [])),
                "optional_recorded": list(job_data.get("optional_recorded", [])),
                "optional_missing": list(job_data.get("optional_missing", [])),
                "camera_health_topics": list(job_data.get("camera_health_topics", [])),
            },
            "preflight": dict(job_data.get("preflight", {})),
            "hz_monitor": {
                "window": job_data.get("hz_window"),
                "topics": hz_stats,
            },
            "bag_stats": bag_stats,
            "environment": {
                "host": socket.gethostname(),
                "user": os.environ.get("USER"),
                "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
                "profile_path": job_data.get("profile_path"),
                "script_args": dict(job_data.get("script_args", {})),
                "key_rosparams": key_rosparams,
            },
            "notes": job_data.get("notes", ""),
            "debug_logs": {
                "kept": kept_debug_logs,
                "removed": removed_debug_logs,
            },
            "warnings": warnings,
            "rosbag_process_terminated_cleanly": bool(job_data.get("rosbag_ok", False)),
            "save_job_id": job_id,
        }

        metadata_path = episode_dir / "metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=True), encoding="utf-8")
        step += 1
        self._update_save_job(job_id, step=step, stage="write metadata", status="OK")

        warning_count = len(warnings)
        bag_count = int(bag_stats.get("bag_count", 0))
        total_size_mb = float(bag_stats.get("total_size_bytes", 0)) / float(1024 ** 2)
        save_status = "OK"
        if warning_count > 0 or bag_count == 0 or not bool(job_data.get("rosbag_ok", False)):
            save_status = "WARN"

        return {
            "episode_name": episode_name,
            "episode_dir": str(episode_dir),
            "metadata_path": str(metadata_path),
            "bag_count": bag_count,
            "total_size_mb": round(total_size_mb, 2),
            "warning_count": warning_count,
            "save_status": save_status,
            "started_at_utc": job_data.get("started_at_utc"),
            "ended_at_utc": job_data.get("ended_at_utc"),
            "duration_sec": job_data.get("duration_sec"),
            "stop_reason": job_data.get("stop_reason"),
        }

    def _reset_current_episode_state(self) -> None:
        self.recording = False
        self.rosbag_proc = None
        self.current_episode_name = None
        self.current_episode_dir = None
        self.current_started_utc = None
        self.current_started_monotonic = None
        self.current_topics = []
        self.current_optional_recorded = []
        self.current_optional_missing = []
        self.current_preflight = {}
        self.current_warnings = []
        self.debug_log_paths = []

    def _wait_for_save_jobs(self) -> None:
        pending = self._pending_save_count()
        if pending <= 0:
            self._emit_save_progress(force=True)
            return
        print(f"[save] waiting for {pending} pending save job(s)...")
        sys.stdout.flush()
        while True:
            self._emit_save_progress(force=False)
            pending = self._pending_save_count()
            if pending <= 0:
                break
            time.sleep(0.2)
        self._emit_save_progress(force=True)
        print("[save] all save jobs completed.")
        sys.stdout.flush()

    def _shutdown_save_executor(self) -> None:
        if self.save_executor_shutdown:
            return
        self.save_executor.shutdown(wait=True)
        self.save_executor_shutdown = True

    def _set_pending_stop(self, reason: str) -> None:
        with self.pending_lock:
            if self.pending_stop_reason is None:
                self.pending_stop_reason = reason

    def _pop_pending_stop(self) -> Optional[str]:
        with self.pending_lock:
            reason = self.pending_stop_reason
            self.pending_stop_reason = None
            return reason

    def _watchdog_loop(self) -> None:
        while not self.shutdown_requested:
            time.sleep(1.0)
            if not self.recording:
                continue

            ok_master, _msg = self._ros_master_ok()
            if not ok_master:
                self._set_pending_stop("ros_master_lost")
                continue

            disk_ok, _free = self._check_disk_ok()
            if not disk_ok:
                self._set_pending_stop("disk_low")
                continue

            if self.rosbag_proc is not None and self.rosbag_proc.poll() is not None:
                self._set_pending_stop("process_error")

    def _start_episode(self) -> None:
        if self.recording:
            return

        ok_master, master_msg = self._ros_master_ok()
        if not ok_master:
            print(f"[record] ROS master unavailable: {master_msg}")
            return

        disk_ok, free_gb = self._check_disk_ok()
        if not disk_ok:
            print(
                f"[record] Disk free too low: {free_gb:.2f} GB < {self.args.min_free_gb:.2f} GB"
            )
            return

        now = time.monotonic()
        quick_restart = (
            self.last_stop_reason == "user_stop"
            and self.last_stop_monotonic is not None
            and (now - self.last_stop_monotonic) <= self.args.restart_preflight_grace_sec
        )
        preflight_timeout = self.args.restart_preflight_timeout_sec if quick_restart else self.args.wait_timeout_sec
        skip_camera_publishers = bool(quick_restart)
        if quick_restart:
            print(
                f"[record] quick restart preflight enabled "
                f"(timeout={preflight_timeout:.1f}s skip_camera_publishers=true)"
            )
        ok, preflight = self._wait_preflight(
            timeout_sec=preflight_timeout,
            skip_camera_publishers=skip_camera_publishers,
        )
        if not ok:
            reason = preflight.get("reason", "preflight_failed")
            print(f"[record] Preflight failed: {reason}")
            missing = list(preflight.get("last_missing_presence_topics", []))
            no_publisher = list(preflight.get("last_joint_no_publisher_topics", []))
            unhealthy = list(preflight.get("last_camera_unhealthy_topics", []))
            if missing:
                print(f"[record] preflight missing topics: {missing[:5]}")
            if no_publisher:
                print(f"[record] preflight joint topics without publishers: {no_publisher[:5]}")
            if unhealthy:
                print(f"[record] preflight camera unhealthy: {unhealthy[:5]}")
            return

        idx, ep_name, ep_dir = self._get_next_episode_slot()
        ep_dir.mkdir(parents=True, exist_ok=False)

        self.current_episode_name = ep_name
        self.current_episode_dir = ep_dir
        self.current_started_utc = utc_now_iso()
        self.current_started_monotonic = time.monotonic()
        self.current_preflight = preflight
        self.current_warnings = []
        self.current_optional_recorded = list(preflight.get("optional_present", []))
        self.current_optional_missing = list(preflight.get("optional_missing", []))
        self.current_topics = dedup_keep_order(self.profile_record_topics + self.current_optional_recorded)
        self.debug_log_paths = []

        bag_prefix = ep_dir / "episode"
        rosbag_log_path = ep_dir / "rosbag_record.log"
        self.rosbag_log_handle = rosbag_log_path.open("w", encoding="utf-8")
        self.debug_log_paths.append(rosbag_log_path)

        cmd = ["rosbag", "record", "--lz4", "-O", str(bag_prefix)]
        if self.args.split:
            cmd.extend(["--split", "--size", str(self.args.split_size_mb)])
        cmd.extend(self.current_topics)

        self.rosbag_proc = subprocess.Popen(
            cmd,
            stdout=self.rosbag_log_handle,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
        )

        self._start_hz_monitors(ep_dir)
        self.recording = True
        self.next_episode_index = idx

        self._print_colored(f"[record] START {ep_name}", color="green")
        print(f"[record] start_utc={self.current_started_utc}")
        print(f"[record] start_local={self._local_now_str()}")
        print(f"[record] episode_path={ep_dir}")
        print(f"[record] bag_prefix={bag_prefix}")
        print(f"[record] topics={len(self.current_topics)} optional={len(self.current_optional_recorded)}")

    def _stop_episode(self, stop_reason: str) -> None:
        if not self.recording:
            return

        episode_name = self.current_episode_name or f"episode_{self.next_episode_index:03d}"
        episode_dir = self.current_episode_dir
        if episode_dir is None:
            return

        start_utc = self.current_started_utc
        stop_local = self._local_now_str()
        stop_color = "green" if stop_reason == "user_stop" else "red"
        self._print_colored(f"[record] STOP {episode_name} reason={stop_reason}", color=stop_color)
        print(f"[record] episode_path={episode_dir}")

        hz_log_paths = self._stop_hz_monitors()

        rosbag_ok = False
        if self.rosbag_proc is not None:
            rosbag_ok = self._stop_process_group(self.rosbag_proc, name="rosbag")
            if not rosbag_ok:
                self.current_warnings.append("Failed to terminate rosbag process cleanly.")
        if self.rosbag_log_handle is not None:
            self.rosbag_log_handle.flush()
            self.rosbag_log_handle.close()
            self.rosbag_log_handle = None

        ended_utc = utc_now_iso()
        duration_sec = None
        if self.current_started_monotonic is not None:
            duration_sec = time.monotonic() - self.current_started_monotonic

        job_data: Dict[str, object] = {
            "session_name": self.session_name,
            "episode_name": episode_name,
            "episode_dir": str(episode_dir),
            "started_at_utc": start_utc,
            "ended_at_utc": ended_utc,
            "duration_sec": duration_sec,
            "stop_reason": stop_reason,
            "camera_transport": self.camera_transport,
            "camera_streams": copy.deepcopy(self.camera_streams),
            "required_topics": list(self.required_topics),
            "camera_record_topics": list(self.camera_record_topics),
            "recorded_topics": list(self.current_topics),
            "optional_recorded": list(self.current_optional_recorded),
            "optional_missing": list(self.current_optional_missing),
            "camera_health_topics": list(self.camera_health_topics),
            "preflight": copy.deepcopy(self.current_preflight),
            "hz_window": self.args.hz_window,
            "hz_log_paths": {topic: str(path) for topic, path in hz_log_paths.items()},
            "debug_log_paths": [str(p) for p in self.debug_log_paths],
            "split_enabled": bool(self.args.split),
            "split_size_mb": self.args.split_size_mb if self.args.split else None,
            "keep_debug_logs": bool(self.args.keep_debug_logs),
            "notes": self.args.notes,
            "profile_path": self.profile.get("_profile_path"),
            "script_args": dict(vars(self.args)),
            "warnings": list(self.current_warnings),
            "rosbag_ok": rosbag_ok,
        }

        save_job_id = self._submit_save_job(job_data)

        self.last_stop_monotonic = time.monotonic()
        self.last_stop_reason = stop_reason
        self._reset_current_episode_state()
        self.next_episode_index = self._scan_next_episode_index()

        print(f"[record] start_utc={start_utc}")
        print(f"[record] stop_utc={ended_utc}")
        print(f"[record] stop_local={stop_local}")
        print(f"[record] duration={self._format_duration(duration_sec)}")
        self._print_colored(f"[save] queued job={save_job_id} episode={episode_name}", color="green")
        print(f"[record] next=episode_{self.next_episode_index:03d}")
        self._emit_save_progress(force=True)

    def _atexit_cleanup(self) -> None:
        if self.recording:
            self._stop_episode("signal_interrupt")
        self._wait_for_save_jobs()
        self._shutdown_save_executor()

    def _signal_handler(self, _sig, _frame) -> None:
        self.signal_requested = True
        if self.recording:
            self._set_pending_stop("signal_interrupt")
        self.shutdown_requested = True

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._print_idle_help()

        try:
            with RawTerminal():
                while not self.shutdown_requested:
                    pending = self._pop_pending_stop()
                    if pending and self.recording:
                        self._stop_episode(pending)

                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        self._emit_runtime_status(force=False)
                        continue

                    ch = sys.stdin.read(1)
                    now = time.monotonic()

                    if ch == " ":
                        if now - self.last_space_ts < self.debounce_sec:
                            continue
                        self.last_space_ts = now
                        if self.recording:
                            self._stop_episode("user_stop")
                        else:
                            self._start_episode()
                    elif ch in ("q", "Q"):
                        if self.recording:
                            self._stop_episode("user_stop")
                        self.shutdown_requested = True
                    self._emit_runtime_status(force=False)
        finally:
            if self.recording:
                reason = "signal_interrupt" if self.signal_requested else "user_stop"
                self._stop_episode(reason)
            self._wait_for_save_jobs()
            self._shutdown_save_executor()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard-driven ACT rosbag recorder")
    parser.add_argument(
        "--profile",
        default="/home/jameszhao2004/catkin_ws/workspaces/config/rosbag_profiles/act_rgb_profile.yaml",
        help="Path to YAML topic profile.",
    )
    parser.add_argument(
        "--camera-transport",
        choices=("profile", "compressed", "raw"),
        default="profile",
        help="Override camera transport from profile. Use 'compressed' or 'raw' (default: profile).",
    )
    parser.add_argument(
        "--session-root",
        default="/home/jameszhao2004/catkin_ws/data/rosbags",
        help="Root directory for rosbag sessions.",
    )
    parser.add_argument("--prefix", default="act", help="Session name prefix when --session-name is not provided.")
    parser.add_argument("--session-name", default="", help="Optional explicit session directory name.")
    parser.add_argument("--notes", default="", help="Optional notes written to metadata.")
    parser.add_argument("--wait-timeout-sec", type=float, default=30.0, help="Preflight topic wait timeout.")
    parser.add_argument(
        "--restart-preflight-grace-sec",
        type=float,
        default=8.0,
        help="If a new start happens within this many seconds after user stop, use fast preflight.",
    )
    parser.add_argument(
        "--restart-preflight-timeout-sec",
        type=float,
        default=3.0,
        help="Fast preflight timeout used during quick restart.",
    )
    parser.add_argument(
        "--split",
        action="store_true",
        help="Enable rosbag file splitting (default: disabled).",
    )
    parser.add_argument("--split-size-mb", type=int, default=512, help="rosbag split size in MB when --split is used.")
    parser.add_argument("--min-free-gb", type=float, default=5.0, help="Minimum free disk threshold before/while recording.")
    parser.add_argument("--debounce-ms", type=int, default=250, help="SPACE key debounce in milliseconds.")
    parser.add_argument("--hz-window", type=int, default=200, help="Window size used by rostopic hz monitors.")
    parser.add_argument(
        "--record-status-interval-sec",
        type=float,
        default=0.2,
        help="Interval for recording status printing while active.",
    )
    parser.add_argument(
        "--save-workers",
        type=int,
        default=2,
        help="Number of asynchronous save workers for episode finalization.",
    )
    parser.add_argument(
        "--save-status-interval-sec",
        type=float,
        default=0.2,
        help="Interval for save progress printing while recording.",
    )
    parser.add_argument(
        "--keep-debug-logs",
        action="store_true",
        help="Keep rosbag_record.log and hz_*.log files in episode directory (default: auto-clean).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        recorder = Recorder(args)
        recorder.run()
        return 0
    except Exception as exc:
        print(f"[record] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
