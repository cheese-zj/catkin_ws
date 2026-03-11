#!/usr/bin/env python3
"""Keyboard runner for intervention switch logic with mock recording.

Extracted from record_act_keyboard_intervention.py:
  - start_align switch (same steps as resume)
  - pause switch
  - resume switch

Controls:
  S:     start a new mock episode
  SPACE: stop current mock episode
  P:     pause current mock episode + switch to teleop control
  R:     resume mock episode + switch back to robot policy control
  Q:     stop if needed and quit
"""

from __future__ import annotations

import argparse
import datetime as dt
import select
import shlex
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Sequence, Tuple


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


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


class SwitchOnlyRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.shutdown_requested = False
        self.signal_requested = False

        self.episode_open = False
        self.recording = False
        self.paused = False
        self.episode_index = 0
        self.current_episode_name: Optional[str] = None
        self.current_started_utc: Optional[str] = None

        self.last_start_ts = 0.0
        self.last_stop_ts = 0.0
        self.debounce_sec = max(0.0, float(args.debounce_ms) / 1000.0)

        self.switch_step_timeout_sec = max(0.1, float(args.step_timeout_sec))
        self.pause_switch_steps: List[List[str]] = [
            [
                "bash",
                "/home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh",
                "teleop",
            ],
            [
                "rosservice",
                "call",
                "/robot/arm_left/joint_cmd_mux_select",
                "/teleop/arm_left/joint_states_single",
            ],
            [
                "rosservice",
                "call",
                "/robot/arm_right/joint_cmd_mux_select",
                "/teleop/arm_right/joint_states_single",
            ],
        ]
        self.resume_switch_steps: List[List[str]] = [
            [
                "rosservice",
                "call",
                "/robot/arm_left/joint_cmd_mux_select",
                "/robot/arm_left/vla_joint_cmd",
            ],
            [
                "rosservice",
                "call",
                "/robot/arm_right/joint_cmd_mux_select",
                "/robot/arm_right/vla_joint_cmd",
            ],
            [
                "bash",
                "/home/jameszhao2004/catkin_ws/workspaces/scripts/set_teleop_mode.sh",
                "follow",
            ],
        ]

        self.mode_switch_events: List[Dict[str, object]] = []
        self._ensure_tools()

    def _ensure_tools(self) -> None:
        if self.args.dry_run:
            return
        missing = []
        for cmd in ("bash", "rosservice"):
            if shutil.which(cmd) is None:
                missing.append(cmd)
        if missing:
            raise RuntimeError(f"Missing required command(s) in PATH: {', '.join(missing)}")

    @staticmethod
    def _cmd_to_string(cmd: Sequence[str]) -> str:
        return " ".join(shlex.quote(part) for part in cmd)

    @staticmethod
    def _trim_command_output(text: Optional[str], max_chars: int = 2000) -> str:
        if not text:
            return ""
        if len(text) <= max_chars:
            return text
        return text[:max_chars] + f"\n... [truncated, total={len(text)} chars]"

    def _run_switch_steps(self, action_name: str, steps: List[List[str]]) -> Tuple[bool, Dict[str, object]]:
        event: Dict[str, object] = {
            "action": action_name,
            "started_at_utc": utc_now_iso(),
            "ended_at_utc": None,
            "ok": True,
            "step_timeout_sec": self.switch_step_timeout_sec,
            "dry_run": bool(self.args.dry_run),
            "steps": [],
        }

        all_ok = True
        total_steps = len(steps)
        for idx, cmd in enumerate(steps, start=1):
            cmd_str = self._cmd_to_string(cmd)
            print(f"[switch][{action_name}] step {idx}/{total_steps} cmd={cmd_str}")
            step_started_mono = time.monotonic()
            step_info: Dict[str, object] = {
                "index": idx,
                "command": cmd_str,
                "argv": list(cmd),
                "started_at_utc": utc_now_iso(),
                "ended_at_utc": None,
                "duration_sec": None,
                "ok": False,
                "returncode": None,
                "stdout": "",
                "stderr": "",
            }

            if self.args.dry_run:
                step_info["ok"] = True
                step_info["ended_at_utc"] = utc_now_iso()
                step_info["duration_sec"] = max(0.0, time.monotonic() - step_started_mono)
                cast_steps = event["steps"]
                if isinstance(cast_steps, list):
                    cast_steps.append(step_info)
                print(f"[switch][{action_name}] step {idx}/{total_steps} DRY-RUN OK")
                continue

            try:
                res = subprocess.run(
                    cmd,
                    text=True,
                    capture_output=True,
                    timeout=self.switch_step_timeout_sec,
                    check=False,
                )
                step_info["returncode"] = res.returncode
                step_info["stdout"] = self._trim_command_output(res.stdout)
                step_info["stderr"] = self._trim_command_output(res.stderr)
                step_ok = (res.returncode == 0)
            except subprocess.TimeoutExpired as exc:
                step_info["stderr"] = self._trim_command_output(
                    f"timeout after {self.switch_step_timeout_sec:.1f}s: {exc}"
                )
                step_ok = False
            except Exception as exc:  # pragma: no cover
                step_info["stderr"] = self._trim_command_output(f"exception: {exc}")
                step_ok = False

            step_info["ok"] = step_ok
            step_info["ended_at_utc"] = utc_now_iso()
            step_info["duration_sec"] = max(0.0, time.monotonic() - step_started_mono)
            cast_steps = event["steps"]
            if isinstance(cast_steps, list):
                cast_steps.append(step_info)

            if step_ok:
                print(f"[switch][{action_name}] step {idx}/{total_steps} OK")
                continue

            all_ok = False
            print(
                f"[switch][{action_name}] step {idx}/{total_steps} FAIL "
                f"rc={step_info.get('returncode')}"
            )
            stderr_text = str(step_info.get("stderr", "")).strip()
            if stderr_text:
                print(f"[switch][{action_name}] stderr={stderr_text[:400]}")
            break

        event["ended_at_utc"] = utc_now_iso()
        event["ok"] = all_ok
        return all_ok, event

    def _mock_record_start(self) -> None:
        print(f"[record][mock] START episode={self.current_episode_name}")

    def _mock_record_stop(self, reason: str) -> None:
        print(f"[record][mock] STOP episode={self.current_episode_name} reason={reason}")

    def _next_episode_name(self) -> str:
        self.episode_index += 1
        return f"episode_{self.episode_index:03d}"

    def _start_episode(self) -> None:
        if self.episode_open:
            return

        episode_name = self._next_episode_name()
        switch_ok, switch_event = self._run_switch_steps("start_align", self.resume_switch_steps)
        self.mode_switch_events.append(switch_event)
        if not switch_ok:
            print("[record][mock] START aborted: start_align switch failed.")
            return

        self.current_episode_name = episode_name
        self.current_started_utc = utc_now_iso()
        self.episode_open = True
        self.recording = True
        self.paused = False

        print(f"[record][mock] START {episode_name} start_utc={self.current_started_utc}")
        self._mock_record_start()

    def _pause_episode(self) -> None:
        if not self.episode_open or self.paused or not self.recording:
            return

        self.recording = False
        self.paused = True
        self._mock_record_stop("user_pause")
        print(f"[record][mock] PAUSE {self.current_episode_name}")

        switch_ok, switch_event = self._run_switch_steps("pause", self.pause_switch_steps)
        self.mode_switch_events.append(switch_event)
        if not switch_ok:
            print("[record][mock] WARN pause switch failed; state remains paused.")

    def _resume_episode(self) -> None:
        if not self.episode_open or not self.paused:
            return

        switch_ok, switch_event = self._run_switch_steps("resume", self.resume_switch_steps)
        self.mode_switch_events.append(switch_event)
        if not switch_ok:
            print("[record][mock] WARN resume switch failed; keep paused.")
            return

        self.paused = False
        self.recording = True
        print(f"[record][mock] RESUME {self.current_episode_name}")
        self._mock_record_start()

    def _stop_episode(self, reason: str) -> None:
        if not self.episode_open:
            return

        if self.recording:
            self._mock_record_stop(reason)

        print(f"[record][mock] STOP {self.current_episode_name} reason={reason}")
        self.episode_open = False
        self.recording = False
        self.paused = False
        self.current_episode_name = None
        self.current_started_utc = None

    def _signal_handler(self, _sig, _frame) -> None:
        self.signal_requested = True
        self.shutdown_requested = True

    def _print_intro(self) -> None:
        print("")
        print("== ACT Intervention Switch-Only Keyboard Runner ==")
        print(f"Mode: {'dry-run' if self.args.dry_run else 'live'}")
        print(f"Step timeout: {self.switch_step_timeout_sec:.1f}s")
        print("Keys: S=start, SPACE=stop, P=pause+switch, R=resume+switch, Q=quit")
        print("[switch][pause]")
        for idx, step in enumerate(self.pause_switch_steps, start=1):
            print(f"  {idx}. {self._cmd_to_string(step)}")
        print("[switch][resume/start_align]")
        for idx, step in enumerate(self.resume_switch_steps, start=1):
            print(f"  {idx}. {self._cmd_to_string(step)}")
        print("")
        sys.stdout.flush()

    def _print_summary(self) -> None:
        print("")
        print("== Summary ==")
        print(f"episodes_started={self.episode_index}")
        print(f"switch_events={len(self.mode_switch_events)}")
        if self.signal_requested:
            print("exit_reason=signal_interrupt")
        else:
            print("exit_reason=user_quit")
        print("")
        sys.stdout.flush()

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._print_intro()

        try:
            with RawTerminal():
                while not self.shutdown_requested:
                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        continue

                    ch = sys.stdin.read(1)
                    now = time.monotonic()

                    if ch in ("s", "S"):
                        if now - self.last_start_ts < self.debounce_sec:
                            continue
                        self.last_start_ts = now
                        self._start_episode()
                    elif ch == " ":
                        if now - self.last_stop_ts < self.debounce_sec:
                            continue
                        self.last_stop_ts = now
                        self._stop_episode("user_stop")
                    elif ch in ("p", "P"):
                        self._pause_episode()
                    elif ch in ("r", "R"):
                        self._resume_episode()
                    elif ch in ("q", "Q"):
                        if self.episode_open:
                            self._stop_episode("user_stop")
                        self.shutdown_requested = True
                    elif ch == "\x03":
                        self.signal_requested = True
                        self.shutdown_requested = True
        finally:
            if self.episode_open:
                reason = "signal_interrupt" if self.signal_requested else "user_stop"
                self._stop_episode(reason)
            self._print_summary()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Switch-only keyboard runner extracted from intervention recorder"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print switch commands only and do not execute them.",
    )
    parser.add_argument(
        "--step-timeout-sec",
        type=float,
        default=15.0,
        help="Timeout for each switch step in seconds.",
    )
    parser.add_argument(
        "--debounce-ms",
        type=int,
        default=250,
        help="Debounce for S/SPACE keys in milliseconds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.step_timeout_sec <= 0:
        print("[error] --step-timeout-sec must be > 0", file=sys.stderr)
        return 2
    if args.debounce_ms < 0:
        print("[error] --debounce-ms must be >= 0", file=sys.stderr)
        return 2

    try:
        runner = SwitchOnlyRunner(args)
        runner.run()
        return 0
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
