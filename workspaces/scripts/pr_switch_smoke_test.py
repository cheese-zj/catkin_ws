#!/usr/bin/env python3
"""Isolated P/R switch smoke test.

Keyboard:
  P: stop policy, then switch to teleop-controls-robot mode
  R: switch back to robot-controls-teleop mode, then start policy
  Q: quit
"""

from __future__ import annotations

import argparse
import os
import select
import shlex
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty
from typing import List, Optional, Sequence


DEFAULT_POLICY_START_CMD = (
    "python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py "
    "--checkpoint-dir "
    "/home/jameszhao2004/training_codebase/outputs/train/"
    "act_20260224_182749_chunk100_obs1_80k_bs8_amp/checkpoints/060000/pretrained_model/ "
    "--device cuda --rate 30 --temporal-ensemble-coeff 0.006 --guard-profile medium --debug-streams"
)


P_STEPS: List[List[str]] = [
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

R_STEPS: List[List[str]] = [
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


class RawTerminal:
    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old: Optional[List[object]] = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise RuntimeError("Interactive TTY is required for keyboard control.")
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


class SwitchSmokeTester:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.state = "robot_controls_teleop"
        self.shutdown_requested = False
        self.signal_requested = False
        self.p_ok_count = 0
        self.r_ok_count = 0
        self.failed_action_count = 0
        self.policy_start_cmd = str(args.policy_start_cmd).strip()
        self.policy_proc: Optional[subprocess.Popen] = None
        self._ensure_tools()

    def _ensure_tools(self) -> None:
        missing = []
        for cmd in ("bash", "rosservice", "python3"):
            if shutil.which(cmd) is None:
                missing.append(cmd)
        if missing:
            raise RuntimeError(f"Missing required command(s) in PATH: {', '.join(missing)}")

    @staticmethod
    def _cmd_to_string(cmd: Sequence[str]) -> str:
        return " ".join(shlex.quote(part) for part in cmd)

    def _run_steps(self, action_name: str, steps: Sequence[Sequence[str]]) -> bool:
        total_steps = len(steps)
        print(f"[switch][{action_name}] START steps={total_steps} dry_run={self.args.dry_run}")
        for idx, cmd in enumerate(steps, start=1):
            cmd_str = self._cmd_to_string(cmd)
            print(f"[switch][{action_name}] step {idx}/{total_steps} cmd={cmd_str}")
            sys.stdout.flush()

            if self.args.dry_run:
                print(f"[switch][{action_name}] step {idx}/{total_steps} DRY-RUN OK")
                continue

            step_started = time.monotonic()
            try:
                result = subprocess.run(
                    list(cmd),
                    text=True,
                    stdout=sys.stdout,
                    stderr=sys.stderr,
                    timeout=self.args.step_timeout_sec,
                    check=False,
                )
                duration = max(0.0, time.monotonic() - step_started)
                if result.returncode == 0:
                    print(f"[switch][{action_name}] step {idx}/{total_steps} OK duration={duration:.2f}s")
                    continue

                print(
                    f"[switch][{action_name}] step {idx}/{total_steps} FAIL "
                    f"rc={result.returncode} duration={duration:.2f}s"
                )
                return False
            except subprocess.TimeoutExpired as exc:
                duration = max(0.0, time.monotonic() - step_started)
                print(
                    f"[switch][{action_name}] step {idx}/{total_steps} FAIL timeout "
                    f"after {self.args.step_timeout_sec:.1f}s duration={duration:.2f}s"
                )
                print(f"[stderr] {exc}")
                return False
            except Exception as exc:  # pragma: no cover
                duration = max(0.0, time.monotonic() - step_started)
                print(
                    f"[switch][{action_name}] step {idx}/{total_steps} FAIL exception "
                    f"duration={duration:.2f}s"
                )
                print(f"[stderr] {exc}")
                return False

        print(f"[switch][{action_name}] DONE OK")
        return True

    @staticmethod
    def _stop_process_group(proc: subprocess.Popen, timeout_sec: float = 5.0) -> bool:
        if proc.poll() is not None:
            return True
        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            return proc.poll() is not None

        phases = (
            (signal.SIGINT, min(2.0, timeout_sec)),
            (signal.SIGTERM, min(2.0, timeout_sec)),
            (signal.SIGKILL, min(1.0, timeout_sec)),
        )
        for sig, wait_sec in phases:
            if proc.poll() is not None:
                return True
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return True
            except Exception:
                pass
            deadline = time.monotonic() + wait_sec
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    return True
                time.sleep(0.05)
        return proc.poll() is not None

    def _start_policy_process(self) -> bool:
        if self.args.dry_run:
            print("[policy] DRY-RUN skip policy start")
            return True
        if not self.policy_start_cmd:
            print("[policy] FAIL empty --policy-start-cmd")
            return False
        if self.policy_proc is not None and self.policy_proc.poll() is None:
            print("[policy] already running")
            return True

        print(f"[policy] START cmd={self.policy_start_cmd}")
        sys.stdout.flush()
        try:
            self.policy_proc = subprocess.Popen(
                self.policy_start_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=sys.stdout,
                stderr=sys.stderr,
                preexec_fn=os.setsid,
                text=True,
            )
            time.sleep(0.2)
            early_rc = self.policy_proc.poll()
            if early_rc is not None:
                print(f"[policy] FAIL exited immediately rc={early_rc}")
                self.policy_proc = None
                return False
            print(f"[policy] OK pid={self.policy_proc.pid}")
            return True
        except Exception as exc:
            print(f"[policy] FAIL exception={exc}")
            self.policy_proc = None
            return False

    def _stop_policy_process(self) -> bool:
        if self.args.dry_run:
            print("[policy] DRY-RUN skip policy stop")
            return True
        if self.policy_proc is None:
            return True
        if self.policy_proc.poll() is not None:
            self.policy_proc = None
            return True
        print("[policy] STOP managed process")
        ok = self._stop_process_group(self.policy_proc, timeout_sec=5.0)
        print(f"[policy] STOP {'OK' if ok else 'FAIL'}")
        self.policy_proc = None
        return ok

    def _handle_pause(self) -> None:
        if self.state != "robot_controls_teleop":
            print(f"[state] Ignore P because state={self.state}; expected robot_controls_teleop.")
            return
        if not self._stop_policy_process():
            self.failed_action_count += 1
            print("[state] P failed; could not stop policy.")
            return
        ok = self._run_steps("P->teleop_controls_robot", P_STEPS)
        if ok:
            self.state = "teleop_controls_robot"
            self.p_ok_count += 1
            print("[state] robot_controls_teleop -> teleop_controls_robot")
            return
        self.failed_action_count += 1
        print("[state] P switch failed; keep robot_controls_teleop")

    def _handle_resume(self) -> None:
        if self.state != "teleop_controls_robot":
            print(f"[state] Ignore R because state={self.state}; expected teleop_controls_robot.")
            return
        ok = self._run_steps("R->robot_controls_teleop", R_STEPS)
        if not ok:
            self.failed_action_count += 1
            print("[state] R switch failed; keep teleop_controls_robot")
            return
        self.state = "robot_controls_teleop"
        self.r_ok_count += 1
        print("[state] teleop_controls_robot -> robot_controls_teleop")
        if not self._start_policy_process():
            self.failed_action_count += 1
            print("[state] R post-switch policy start failed; state remains robot_controls_teleop")

    def _signal_handler(self, _sig, _frame) -> None:
        self.signal_requested = True
        self.shutdown_requested = True

    def _print_intro(self) -> None:
        print("")
        print("== PR Switch Smoke Test ==")
        print("Env prerequisite in this same terminal:")
        print("  ros1")
        print("  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh")
        print("  source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate")
        print(f"Mode: {'dry-run' if self.args.dry_run else 'live'}")
        print(f"Step timeout: {self.args.step_timeout_sec:.1f}s")
        print("Policy startup: on script boot")
        print(f"Policy cmd: {self.policy_start_cmd}")
        print("Initial state assumption: robot_controls_teleop")
        print("Keys: P=stop_policy+switch_to_teleop_control, R=switch_back+start_policy, Q=quit")
        print("")
        print("[commands][P: teleop_controls_robot]")
        for idx, cmd in enumerate(P_STEPS, start=1):
            print(f"  {idx}. {self._cmd_to_string(cmd)}")
        print("[commands][R: robot_controls_teleop]")
        for idx, cmd in enumerate(R_STEPS, start=1):
            print(f"  {idx}. {self._cmd_to_string(cmd)}")
        print("")
        sys.stdout.flush()

    def _print_summary(self) -> None:
        print("")
        print("== Summary ==")
        print(f"final_state={self.state}")
        print(f"p_ok_count={self.p_ok_count}")
        print(f"r_ok_count={self.r_ok_count}")
        print(f"failed_action_count={self.failed_action_count}")
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
            if not self._start_policy_process():
                self.failed_action_count += 1
                self.shutdown_requested = True
                return
            with RawTerminal():
                while not self.shutdown_requested:
                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        continue

                    ch = sys.stdin.read(1)
                    if ch in ("p", "P"):
                        self._handle_pause()
                    elif ch in ("r", "R"):
                        self._handle_resume()
                    elif ch in ("q", "Q"):
                        self.shutdown_requested = True
                    elif ch == "\x03":
                        self.signal_requested = True
                        self.shutdown_requested = True
        finally:
            self._stop_policy_process()
            self._print_summary()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Isolated PR switch smoke test")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print switch commands only and do not execute them.",
    )
    parser.add_argument(
        "--policy-start-cmd",
        default=DEFAULT_POLICY_START_CMD,
        help="Shell command to start policy at script startup.",
    )
    parser.add_argument(
        "--step-timeout-sec",
        type=float,
        default=15.0,
        help="Timeout for each switch command step in seconds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.step_timeout_sec <= 0:
        print("[error] --step-timeout-sec must be > 0", file=sys.stderr)
        return 2

    try:
        tester = SwitchSmokeTester(args)
        tester.run()
        return 0
    except RuntimeError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
