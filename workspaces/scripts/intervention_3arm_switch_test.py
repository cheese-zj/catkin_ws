#!/usr/bin/env python3
"""3-arm intervention keyboard switch test.

Keyboard:
  1: L->OPP, R->R, Left Hold (policy stopped)
  2: L->L, R->OPP, Right Hold (policy stopped)
  H: L->L, R->R, OPP Hold (policy stopped)
  F: L/R/OPP -> VLA (policy running)
  S: print status
  Q: stop policy + hold all + quit
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Callable, Dict, List, Optional

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from topic_tools.srv import MuxSelect

try:
    from piper_msgs.srv import Enable
except Exception as exc:  # pragma: no cover
    raise SystemExit("piper_msgs.srv.Enable is required in this ROS environment.") from exc


DEFAULT_POLICY_START_CMD = (
    "python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_act_checkpoint_ros.py "
    "--checkpoint-dir "
    "/home/jameszhao2004/training_codebase/outputs/train/"
    "act_20260224_182749_chunk100_obs1_80k_bs8_amp/checkpoints/060000/pretrained_model/ "
    "--device cuda --rate 30 --temporal-ensemble-coeff 0.006 --guard-profile medium --debug-streams"
)

MODE_INIT = "INIT"
MODE_AUTO_POLICY = "AUTO_POLICY"
MODE_TAKEOVER_1 = "TAKEOVER_1"
MODE_TAKEOVER_2 = "TAKEOVER_2"
MODE_TAKEOVER_H = "TAKEOVER_H"
MODE_SAFE_HOLD = "SAFE_HOLD"


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def _clamp(value: float, lower: float, upper: float) -> float:
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


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


class ThreeArmInterventionSwitch:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args

        self.shutdown_requested = False
        self.signal_requested = False
        self.mode = MODE_INIT

        self.policy_proc: Optional[subprocess.Popen] = None
        self.policy_log_handle = None
        self.policy_launch_index = 0
        self.last_action_ts: Dict[str, float] = {}

        self.robot_states: Dict[str, Optional[JointState]] = {
            "left": None,
            "right": None,
            "opp": None,
        }
        self.master_states: Dict[str, Optional[JointState]] = {
            "left": None,
            "right": None,
        }
        self.selected_routes: Dict[str, str] = {
            "left": "",
            "right": "",
            "opp": "",
        }
        self.selected_feedback_routes: Dict[str, str] = {
            "left_gravity": "",
            "right_gravity": "",
            "left_torque": "",
            "right_torque": "",
        }

        self.left_hold_pub = rospy.Publisher(args.left_hold_topic, JointState, queue_size=1, latch=True)
        self.right_hold_pub = rospy.Publisher(args.right_hold_topic, JointState, queue_size=1, latch=True)
        self.opp_hold_pub = rospy.Publisher(args.opp_hold_topic, JointState, queue_size=1, latch=True)
        self.left_align_pub = rospy.Publisher(args.left_master_align_topic, JointState, queue_size=1)
        self.right_align_pub = rospy.Publisher(args.right_master_align_topic, JointState, queue_size=1)
        self.follow_flag_pub = rospy.Publisher(args.slave_follow_flag_topic, Bool, queue_size=1, latch=True)
        self.left_master_force_cmd_pub = rospy.Publisher(
            args.left_master_force_cmd_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.right_master_force_cmd_pub = rospy.Publisher(
            args.right_master_force_cmd_topic,
            Bool,
            queue_size=1,
            latch=True,
        )

        self.left_mux = rospy.ServiceProxy(args.left_mux_service, MuxSelect)
        self.right_mux = rospy.ServiceProxy(args.right_mux_service, MuxSelect)
        self.opp_mux = rospy.ServiceProxy(args.opp_mux_service, MuxSelect)
        self.left_gravity_mux = rospy.ServiceProxy(args.left_gravity_mux_service, MuxSelect)
        self.right_gravity_mux = rospy.ServiceProxy(args.right_gravity_mux_service, MuxSelect)
        self.left_torque_mux = rospy.ServiceProxy(args.left_torque_mux_service, MuxSelect)
        self.right_torque_mux = rospy.ServiceProxy(args.right_torque_mux_service, MuxSelect)

        self.left_master_cmd_mux = rospy.ServiceProxy(args.left_master_cmd_mux_service, MuxSelect)
        self.right_master_cmd_mux = rospy.ServiceProxy(args.right_master_cmd_mux_service, MuxSelect)
        self.left_master_enable_srv = rospy.ServiceProxy(args.left_master_enable_service, Enable)
        self.right_master_enable_srv = rospy.ServiceProxy(args.right_master_enable_service, Enable)
        self.left_master_block_srv = rospy.ServiceProxy(args.left_master_block_service, SetBool)
        self.right_master_block_srv = rospy.ServiceProxy(args.right_master_block_service, SetBool)

        self.subscribers = [
            rospy.Subscriber(
                args.robot_left_state_topic,
                JointState,
                self._cb_robot_left,
                queue_size=1,
                tcp_nodelay=True,
            ),
            rospy.Subscriber(
                args.robot_right_state_topic,
                JointState,
                self._cb_robot_right,
                queue_size=1,
                tcp_nodelay=True,
            ),
            rospy.Subscriber(
                args.robot_opp_state_topic,
                JointState,
                self._cb_robot_opp,
                queue_size=1,
                tcp_nodelay=True,
            ),
            rospy.Subscriber(
                args.left_master_topic,
                JointState,
                self._cb_master_left,
                queue_size=1,
                tcp_nodelay=True,
            ),
            rospy.Subscriber(
                args.right_master_topic,
                JointState,
                self._cb_master_right,
                queue_size=1,
                tcp_nodelay=True,
            ),
        ]
        self.left_gravity_selected_sub = rospy.Subscriber(
            args.left_gravity_selected_topic,
            String,
            self._cb_left_gravity_selected,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.right_gravity_selected_sub = rospy.Subscriber(
            args.right_gravity_selected_topic,
            String,
            self._cb_right_gravity_selected,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.latest_left_gravity_selected = ""
        self.latest_right_gravity_selected = ""

    def _cb_robot_left(self, msg: JointState) -> None:
        self.robot_states["left"] = msg

    def _cb_robot_right(self, msg: JointState) -> None:
        self.robot_states["right"] = msg

    def _cb_robot_opp(self, msg: JointState) -> None:
        self.robot_states["opp"] = msg

    def _cb_master_left(self, msg: JointState) -> None:
        self.master_states["left"] = msg

    def _cb_master_right(self, msg: JointState) -> None:
        self.master_states["right"] = msg

    def _cb_left_gravity_selected(self, msg: String) -> None:
        self.latest_left_gravity_selected = str(msg.data)

    def _cb_right_gravity_selected(self, msg: String) -> None:
        self.latest_right_gravity_selected = str(msg.data)

    def _signal_handler(self, _sig, _frame) -> None:
        self.signal_requested = True
        self.shutdown_requested = True

    @staticmethod
    def _extract_pos7(src: JointState) -> List[float]:
        if len(src.position) < 7:
            raise RuntimeError("joint_states message has < 7 position dimensions.")
        return [float(v) for v in src.position[:7]]

    @staticmethod
    def _max_abs_joint_error(a: List[float], b: List[float], dof: int) -> float:
        return max(abs(float(a[i]) - float(b[i])) for i in range(dof))

    @staticmethod
    def _build_joint_msg(names: List[str], pos7: List[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(names)
        msg.position = [float(v) for v in pos7]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        return msg

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

    def _wait_joint_msg(self, topic: str, label: str) -> JointState:
        try:
            msg = rospy.wait_for_message(topic, JointState, timeout=self.args.service_timeout_sec)
        except Exception as exc:
            raise RuntimeError(f"Timeout waiting {label} topic: {topic}") from exc
        if len(msg.position) < 7:
            raise RuntimeError(f"{label} topic has <7 position dims: {topic}")
        return msg

    def _latest_robot_or_wait(self, arm: str) -> JointState:
        msg = self.robot_states[arm]
        if msg is not None and len(msg.position) >= 7:
            return msg
        topic = {
            "left": self.args.robot_left_state_topic,
            "right": self.args.robot_right_state_topic,
            "opp": self.args.robot_opp_state_topic,
        }[arm]
        msg = self._wait_joint_msg(topic, f"robot {arm}")
        self.robot_states[arm] = msg
        return msg

    def _latest_master_or_wait(self, side: str) -> JointState:
        msg = self.master_states[side]
        if msg is not None and len(msg.position) >= 7:
            return msg
        topic = self.args.left_master_topic if side == "left" else self.args.right_master_topic
        msg = self._wait_joint_msg(topic, f"master {side}")
        self.master_states[side] = msg
        return msg

    def _publish_follow_flag_false(
        self,
        *,
        repeat: int,
        interval_sec: float,
        force: bool = False,
    ) -> None:
        if (not force) and (not self.args.set_free_mode_on_auto):
            return
        repeat = max(1, int(repeat))
        interval_sec = max(0.0, float(interval_sec))
        self._wait_for_pub_subscriber(
            self.follow_flag_pub,
            self.args.slave_follow_flag_topic,
            self.args.align_flag_subscriber_wait_sec,
        )
        for _ in range(repeat):
            self.follow_flag_pub.publish(Bool(data=False))
            if interval_sec > 0.0:
                rospy.sleep(interval_sec)

    def _force_teleop_mode_for_alignment(self) -> None:
        if not self.args.align_force_teleop_flag:
            return
        self._publish_follow_flag_false(
            repeat=self.args.align_force_flag_repeat,
            interval_sec=self.args.align_force_flag_interval_sec,
            force=True,
        )

    def _set_master_cmd_mux(self, side: str, topic: str) -> None:
        proxy = self.left_master_cmd_mux if side == "left" else self.right_master_cmd_mux
        service_name = self.args.left_master_cmd_mux_service if side == "left" else self.args.right_master_cmd_mux_service
        try:
            resp = proxy(topic)
            prev = getattr(resp, "prev_topic", "")
            rospy.loginfo("%s master-cmd mux switch: prev=%s -> now=%s", side, prev, topic)
        except Exception as exc:
            raise RuntimeError(f"Failed calling {service_name} with topic={topic}: {exc}") from exc

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
            ok = bool(getattr(resp, "success", False))
            msg = str(getattr(resp, "message", ""))
            rospy.loginfo("alignment pre-step: %s(false) -> success=%s msg=%s", service_name, ok, msg)
        except Exception as exc:
            rospy.logwarn("alignment pre-step: failed calling %s(false): %s", service_name, str(exc))

    def _set_master_force_cmd_mode(self, side: str, enabled: bool) -> None:
        if not self.args.align_force_master_cmd_mode:
            return
        pub = self.left_master_force_cmd_pub if side == "left" else self.right_master_force_cmd_pub
        topic = self.args.left_master_force_cmd_topic if side == "left" else self.args.right_master_force_cmd_topic
        self._wait_for_pub_subscriber(pub, topic, self.args.align_flag_subscriber_wait_sec)
        repeat = max(1, int(self.args.align_force_master_cmd_repeat))
        interval = max(0.0, float(self.args.align_force_master_cmd_interval_sec))
        msg = Bool(data=bool(enabled))
        for _ in range(repeat):
            pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)

    def _align_master_to_target(self, side: str, target_msg: JointState, target_label: str) -> bool:
        master_topic = self.args.left_master_topic if side == "left" else self.args.right_master_topic
        align_topic = self.args.left_master_align_topic if side == "left" else self.args.right_master_align_topic
        default_topic = self.args.left_master_default_cmd_topic if side == "left" else self.args.right_master_default_cmd_topic
        pub = self.left_align_pub if side == "left" else self.right_align_pub

        self._force_teleop_mode_for_alignment()
        self._unblock_master_for_alignment(side)
        self._enable_master_for_alignment(side)

        master_msg = self._latest_master_or_wait(side)
        start_pos = self._extract_pos7(master_msg)
        target_pos = self._extract_pos7(target_msg)
        names = list(target_msg.name[:7]) if len(target_msg.name) >= 7 else [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]

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
            "%s align->%s begin: max_err=%.4f tol=%.4f duration=%.3fs steps=%d topic=%s",
            side,
            target_label,
            max_err,
            tol,
            duration,
            steps,
            master_topic,
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
                pub.publish(self._build_joint_msg(names, pos))
                rate.sleep()

            stable_count = 0
            while not rospy.is_shutdown():
                pub.publish(self._build_joint_msg(names, target_pos))
                current = self.master_states[side]
                if current is not None and len(current.position) >= 7:
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
                self._set_master_cmd_mux(side, default_topic)
            except Exception as exc:
                rospy.logwarn("%s failed restoring master-cmd mux: %s", side, str(exc))
            self._set_master_force_cmd_mode(side, False)

        if success:
            rospy.loginfo("%s align->%s success", side, target_label)
        else:
            rospy.logwarn(
                "%s align->%s timeout. final_err=%.4f min_err=%.4f tol=%.4f",
                side,
                target_label,
                final_err,
                min_err,
                tol,
            )
        return success

    def _align_for_mode(self, mode: str) -> None:
        if not self.args.align_before_switch:
            return
        targets: Dict[str, JointState]
        if mode == MODE_TAKEOVER_1:
            targets = {
                "left": self._latest_robot_or_wait("opp"),
                "right": self._latest_robot_or_wait("right"),
            }
        elif mode == MODE_TAKEOVER_2:
            targets = {
                "left": self._latest_robot_or_wait("left"),
                "right": self._latest_robot_or_wait("opp"),
            }
        elif mode == MODE_TAKEOVER_H:
            targets = {
                "left": self._latest_robot_or_wait("left"),
                "right": self._latest_robot_or_wait("right"),
            }
        else:
            return

        if not self._align_master_to_target("left", targets["left"], f"{mode}:left_target"):
            raise RuntimeError(f"Alignment failed for left master in mode={mode}")
        if not self._align_master_to_target("right", targets["right"], f"{mode}:right_target"):
            raise RuntimeError(f"Alignment failed for right master in mode={mode}")

    def _desired_robot_routes(self, mode: str) -> Dict[str, str]:
        if mode == MODE_AUTO_POLICY:
            return {
                "left": self.args.left_vla_topic,
                "right": self.args.right_vla_topic,
                "opp": self.args.opp_vla_topic,
            }
        if mode == MODE_TAKEOVER_1:
            return {
                "left": self.args.left_hold_topic,
                "right": self.args.right_master_topic,
                "opp": self.args.left_master_topic,
            }
        if mode == MODE_TAKEOVER_2:
            return {
                "left": self.args.left_master_topic,
                "right": self.args.right_hold_topic,
                "opp": self.args.right_master_topic,
            }
        if mode == MODE_TAKEOVER_H:
            return {
                "left": self.args.left_master_topic,
                "right": self.args.right_master_topic,
                "opp": self.args.opp_hold_topic,
            }
        if mode == MODE_SAFE_HOLD:
            return {
                "left": self.args.left_hold_topic,
                "right": self.args.right_hold_topic,
                "opp": self.args.opp_hold_topic,
            }
        raise RuntimeError(f"Unsupported mode for robot routes: {mode}")

    def _desired_feedback_targets(self, mode: str) -> Dict[str, str]:
        if mode == MODE_TAKEOVER_1:
            return {"left": "opp", "right": "right"}
        if mode == MODE_TAKEOVER_2:
            return {"left": "left", "right": "opp"}
        if mode in (MODE_TAKEOVER_H, MODE_AUTO_POLICY, MODE_SAFE_HOLD):
            return {"left": "left", "right": "right"}
        raise RuntimeError(f"Unsupported mode for feedback routes: {mode}")

    def _feedback_topic_for_arm(self, *, kind: str, arm: str) -> str:
        if kind == "gravity":
            topic_map = {
                "left": self.args.left_default_gravity_topic,
                "right": self.args.right_default_gravity_topic,
                "opp": self.args.opp_gravity_topic,
            }
            if arm in topic_map:
                return topic_map[arm]
            raise RuntimeError(f"Unsupported gravity arm key: {arm}")
        if kind == "torque":
            topic_map = {
                "left": self.args.left_default_torque_topic,
                "right": self.args.right_default_torque_topic,
                "opp": self.args.opp_torque_topic,
            }
            if arm in topic_map:
                return topic_map[arm]
            raise RuntimeError(f"Unsupported torque arm key: {arm}")
        raise RuntimeError(f"Unsupported feedback kind: {kind}")

    def _set_feedback_mux(self, *, side: str, kind: str, topic: str) -> None:
        if kind == "gravity":
            proxy = self.left_gravity_mux if side == "left" else self.right_gravity_mux
            service_name = self.args.left_gravity_mux_service if side == "left" else self.args.right_gravity_mux_service
            route_key = "left_gravity" if side == "left" else "right_gravity"
        elif kind == "torque":
            proxy = self.left_torque_mux if side == "left" else self.right_torque_mux
            service_name = self.args.left_torque_mux_service if side == "left" else self.args.right_torque_mux_service
            route_key = "left_torque" if side == "left" else "right_torque"
        else:
            raise RuntimeError(f"Unsupported feedback kind: {kind}")

        try:
            resp = proxy(topic)
            prev_topic = getattr(resp, "prev_topic", "")
            self.selected_feedback_routes[route_key] = topic
            rospy.loginfo(
                "%s %s mux switch via %s: prev=%s -> now=%s",
                side,
                kind,
                service_name,
                prev_topic,
                topic,
            )
        except Exception as exc:
            raise RuntimeError(f"Failed calling {service_name} with topic={topic}: {exc}") from exc

    def _apply_feedback_routes_for_mode(self, mode: str) -> None:
        if (not self.args.route_gravity_feedback) and (not self.args.route_torque_feedback):
            return
        targets = self._desired_feedback_targets(mode)
        left_arm = targets["left"]
        right_arm = targets["right"]

        if self.args.route_gravity_feedback:
            self._set_feedback_mux(
                side="left",
                kind="gravity",
                topic=self._feedback_topic_for_arm(kind="gravity", arm=left_arm),
            )
            self._set_feedback_mux(
                side="right",
                kind="gravity",
                topic=self._feedback_topic_for_arm(kind="gravity", arm=right_arm),
            )

        if self.args.route_torque_feedback:
            self._set_feedback_mux(
                side="left",
                kind="torque",
                topic=self._feedback_topic_for_arm(kind="torque", arm=left_arm),
            )
            self._set_feedback_mux(
                side="right",
                kind="torque",
                topic=self._feedback_topic_for_arm(kind="torque", arm=right_arm),
            )

    def _restore_default_feedback_routes(self, *, best_effort: bool) -> None:
        try:
            self._apply_feedback_routes_for_mode(MODE_AUTO_POLICY)
        except Exception as exc:
            if best_effort:
                rospy.logwarn("Best-effort feedback route restore failed: %s", str(exc))
                return
            raise

    def _masters_to_align_for_mode(self, mode: str) -> List[str]:
        if (not self.args.align_before_switch) or mode not in (MODE_TAKEOVER_1, MODE_TAKEOVER_2, MODE_TAKEOVER_H):
            return []
        return ["left", "right"]

    def _pre_detach_master_consumers_for_alignment(self, mode: str) -> None:
        masters = self._masters_to_align_for_mode(mode)
        if not masters:
            return
        master_topics = set()
        if "left" in masters:
            master_topics.add(self.args.left_master_topic)
        if "right" in masters:
            master_topics.add(self.args.right_master_topic)
        for arm in ("left", "right", "opp"):
            current_source = self.selected_routes.get(arm, "")
            if current_source in master_topics:
                rospy.loginfo(
                    "pre-align detach: arm=%s source=%s -> hold",
                    arm,
                    current_source,
                )
                self._hold_arm_and_select(arm)

    def _apply_robot_routes_for_mode(self, mode: str) -> None:
        routes = self._desired_robot_routes(mode)
        for arm in ("left", "right", "opp"):
            source_topic = routes[arm]
            if source_topic == self._hold_topic(arm):
                self._hold_arm_and_select(arm)
            else:
                self._set_arm_source(arm, source_topic)

    def _mux_proxy_and_name(self, arm: str):
        if arm == "left":
            return self.left_mux, self.args.left_mux_service
        if arm == "right":
            return self.right_mux, self.args.right_mux_service
        if arm == "opp":
            return self.opp_mux, self.args.opp_mux_service
        raise RuntimeError(f"Unsupported arm key: {arm}")

    def _set_arm_source(self, arm: str, source_topic: str) -> None:
        proxy, service_name = self._mux_proxy_and_name(arm)
        try:
            resp = proxy(source_topic)
            prev_topic = getattr(resp, "prev_topic", "")
            self.selected_routes[arm] = source_topic
            rospy.loginfo("%s mux switch via %s: prev=%s -> now=%s", arm, service_name, prev_topic, source_topic)
        except Exception as exc:
            raise RuntimeError(f"Failed calling {service_name} with topic={source_topic}: {exc}") from exc

    def _hold_topic(self, arm: str) -> str:
        if arm == "left":
            return self.args.left_hold_topic
        if arm == "right":
            return self.args.right_hold_topic
        if arm == "opp":
            return self.args.opp_hold_topic
        raise RuntimeError(f"Unsupported arm key: {arm}")

    def _hold_publisher(self, arm: str):
        if arm == "left":
            return self.left_hold_pub
        if arm == "right":
            return self.right_hold_pub
        if arm == "opp":
            return self.opp_hold_pub
        raise RuntimeError(f"Unsupported arm key: {arm}")

    def _build_hold_msg(self, arm: str) -> JointState:
        robot_msg = self._latest_robot_or_wait(arm)
        names = list(robot_msg.name[:7]) if len(robot_msg.name) >= 7 else [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]
        pos7 = self._extract_pos7(robot_msg)
        return self._build_joint_msg(names, pos7)

    def _publish_hold_repeated(self, arm: str, msg: JointState) -> None:
        pub = self._hold_publisher(arm)
        repeat = max(1, int(self.args.hold_pub_repeat))
        interval = max(0.0, float(self.args.hold_pub_interval_sec))
        for _ in range(repeat):
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)

    def _hold_arm_and_select(self, arm: str) -> None:
        hold_msg = self._build_hold_msg(arm)
        hold_topic = self._hold_topic(arm)
        self._publish_hold_repeated(arm, hold_msg)
        self._set_arm_source(arm, hold_topic)
        self._publish_hold_repeated(arm, hold_msg)
        rospy.loginfo("%s hold applied on %s", arm, hold_topic)

    def _hold_all_arms(self, *, best_effort: bool) -> None:
        errors: List[str] = []
        for arm in ("left", "right", "opp"):
            try:
                self._hold_arm_and_select(arm)
            except Exception as exc:
                errors.append(f"{arm}: {exc}")
        if errors:
            if best_effort:
                rospy.logerr("Hold-all best-effort errors: %s", "; ".join(errors))
                return
            raise RuntimeError("Hold-all failed: " + "; ".join(errors))

    @staticmethod
    def _trim_command_output(text: Optional[str], max_chars: int = 2000) -> str:
        if not text:
            return ""
        if len(text) <= max_chars:
            return text
        return text[:max_chars] + f"\n... [truncated, total={len(text)} chars]"

    def _close_policy_log_handle(self) -> None:
        if self.policy_log_handle is None:
            return
        self.policy_log_handle.flush()
        self.policy_log_handle.close()
        self.policy_log_handle = None

    def _start_policy_process(self, trigger: str) -> bool:
        if self.policy_proc is not None and self.policy_proc.poll() is None:
            rospy.loginfo("policy already running (trigger=%s pid=%d)", trigger, self.policy_proc.pid)
            return True
        if not self.args.policy_start_cmd.strip():
            rospy.logerr("Policy start command is empty.")
            return False

        log_dir = Path(self.args.policy_log_dir).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.policy_launch_index += 1
        log_path = log_dir / f"policy_run_{self.policy_launch_index:03d}.log"

        try:
            self.policy_log_handle = log_path.open("w", encoding="utf-8")
            self.policy_proc = subprocess.Popen(
                self.args.policy_start_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=self.policy_log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True,
            )
            time.sleep(0.2)
            early_rc = self.policy_proc.poll()
            if early_rc is not None:
                rospy.logerr("Policy exited immediately (trigger=%s rc=%s log=%s)", trigger, early_rc, log_path)
                self.policy_proc = None
                self._close_policy_log_handle()
                return False
        except Exception as exc:
            rospy.logerr("Policy start exception (trigger=%s): %s", trigger, str(exc))
            self.policy_proc = None
            self._close_policy_log_handle()
            return False

        rospy.loginfo(
            "Policy started (trigger=%s pid=%d log=%s)",
            trigger,
            self.policy_proc.pid,
            str(log_path),
        )
        return True

    def _stop_process_group(self, proc: subprocess.Popen, *, name: str, timeout_sec: float = 5.0) -> bool:
        if proc.poll() is not None:
            return True
        timeout_sec = max(0.1, float(timeout_sec))
        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            pgid = None

        def _wait_until(deadline: float) -> bool:
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    return True
                time.sleep(0.05)
            return proc.poll() is not None

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGINT)
            except Exception:
                pass
        if _wait_until(time.monotonic() + min(timeout_sec, 3.0)):
            return True

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
        if _wait_until(time.monotonic() + min(timeout_sec, 2.0)):
            return True

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
        if _wait_until(time.monotonic() + 1.0):
            return True

        rospy.logerr("Failed to stop %s process group (pid=%s).", name, proc.pid)
        return False

    def _stop_policy_process(self, trigger: str) -> bool:
        overall_ok = True
        stop_cmd = self.args.policy_stop_cmd.strip()
        if stop_cmd:
            try:
                res = subprocess.run(
                    stop_cmd,
                    shell=True,
                    executable="/bin/bash",
                    text=True,
                    capture_output=True,
                    timeout=self.args.policy_hook_timeout_sec,
                    check=False,
                )
                if res.returncode != 0:
                    overall_ok = False
                    rospy.logwarn(
                        "policy_stop_cmd failed rc=%s trigger=%s stderr=%s",
                        res.returncode,
                        trigger,
                        self._trim_command_output(res.stderr),
                    )
            except Exception as exc:
                overall_ok = False
                rospy.logwarn("policy_stop_cmd exception trigger=%s err=%s", trigger, str(exc))

        if self.policy_proc is not None:
            ok = True
            if self.policy_proc.poll() is None:
                ok = self._stop_process_group(self.policy_proc, name="policy")
            overall_ok = overall_ok and ok
            self.policy_proc = None

        self._close_policy_log_handle()
        if overall_ok:
            rospy.loginfo("Policy stopped (trigger=%s)", trigger)
        else:
            rospy.logwarn("Policy stop had warnings/errors (trigger=%s)", trigger)
        return overall_ok

    def _enter_mode_auto_policy(self) -> None:
        policy_running = self.policy_proc is not None and self.policy_proc.poll() is None
        if self.mode == MODE_AUTO_POLICY and policy_running:
            rospy.loginfo("already in mode=%s", MODE_AUTO_POLICY)
            return

        self._apply_robot_routes_for_mode(MODE_AUTO_POLICY)
        self._apply_feedback_routes_for_mode(MODE_AUTO_POLICY)
        if self.args.set_free_mode_on_auto:
            self._publish_follow_flag_false(
                repeat=self.args.free_mode_publish_repeat,
                interval_sec=self.args.free_mode_publish_interval_sec,
                force=False,
            )
        if not self._start_policy_process("enter_auto_policy"):
            raise RuntimeError("Failed to start policy process in AUTO_POLICY mode.")
        self.mode = MODE_AUTO_POLICY
        rospy.loginfo("Mode -> %s", self.mode)

    def _enter_mode_takeover(self, mode: str) -> None:
        if mode not in (MODE_TAKEOVER_1, MODE_TAKEOVER_2, MODE_TAKEOVER_H):
            raise RuntimeError(f"Unsupported takeover mode: {mode}")
        # Safety requirement: 1/2/H must always stop policy first.
        self._stop_policy_process(f"enter_{mode}")
        if self.mode == mode:
            rospy.loginfo("already in mode=%s (policy stop enforced)", mode)
            return
        self._pre_detach_master_consumers_for_alignment(mode)
        self._apply_feedback_routes_for_mode(mode)
        self._align_for_mode(mode)
        self._apply_robot_routes_for_mode(mode)
        self.mode = mode
        rospy.loginfo("Mode -> %s", self.mode)

    def _enter_safe_hold(self, reason: str) -> None:
        rospy.logerr("Entering SAFE_HOLD due to: %s", reason)
        self._stop_policy_process(f"safe_hold:{reason}")
        self._hold_all_arms(best_effort=True)
        self._restore_default_feedback_routes(best_effort=True)
        self.mode = MODE_SAFE_HOLD
        rospy.logwarn("Mode -> %s", self.mode)

    def _transition_or_safe_hold(self, transition_name: str, fn: Callable[[], None]) -> None:
        try:
            fn()
        except Exception as exc:
            rospy.logerr("Transition %s failed: %s", transition_name, str(exc))
            self._enter_safe_hold(f"{transition_name}:{exc}")

    def _wait_for_services_and_topics(self) -> None:
        rospy.loginfo("Waiting for mux services...")
        rospy.wait_for_service(self.args.left_mux_service, timeout=self.args.service_timeout_sec)
        rospy.wait_for_service(self.args.right_mux_service, timeout=self.args.service_timeout_sec)
        rospy.wait_for_service(self.args.opp_mux_service, timeout=self.args.service_timeout_sec)

        if self.args.route_gravity_feedback:
            rospy.loginfo("Waiting for teleop gravity feedback mux services...")
            rospy.wait_for_service(self.args.left_gravity_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_gravity_mux_service, timeout=self.args.service_timeout_sec)

        if self.args.route_torque_feedback:
            rospy.loginfo("Waiting for teleop torque feedback mux services...")
            rospy.wait_for_service(self.args.left_torque_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_torque_mux_service, timeout=self.args.service_timeout_sec)

        if self.args.align_before_switch:
            rospy.loginfo("Waiting for master cmd mux services...")
            rospy.wait_for_service(self.args.left_master_cmd_mux_service, timeout=self.args.service_timeout_sec)
            rospy.wait_for_service(self.args.right_master_cmd_mux_service, timeout=self.args.service_timeout_sec)

        rospy.loginfo("Waiting for required state topics...")
        self.robot_states["left"] = self._wait_joint_msg(self.args.robot_left_state_topic, "robot left")
        self.robot_states["right"] = self._wait_joint_msg(self.args.robot_right_state_topic, "robot right")
        self.robot_states["opp"] = self._wait_joint_msg(self.args.robot_opp_state_topic, "robot opp")
        self.master_states["left"] = self._wait_joint_msg(self.args.left_master_topic, "master left")
        self.master_states["right"] = self._wait_joint_msg(self.args.right_master_topic, "master right")

        if self.args.route_gravity_feedback:
            rospy.loginfo("Waiting for gravity feedback source topics...")
            self._wait_joint_msg(self.args.left_default_gravity_topic, "left default gravity")
            self._wait_joint_msg(self.args.right_default_gravity_topic, "right default gravity")
            try:
                self._wait_joint_msg(self.args.opp_gravity_topic, "opp gravity")
            except Exception as exc:
                raise RuntimeError(
                    f"No messages on {self.args.opp_gravity_topic}. "
                    "Launch teleop gravity node with enable_opp_arm:=true."
                ) from exc

    def _key_debounced(self, key: str) -> bool:
        now = time.monotonic()
        last = self.last_action_ts.get(key, 0.0)
        if (now - last) < (float(self.args.debounce_ms) / 1000.0):
            return True
        self.last_action_ts[key] = now
        return False

    def _print_intro(self) -> None:
        print("")
        print("== 3-Arm Intervention Switch Test ==")
        print(f"started_utc={utc_now_iso()}")
        print(f"policy_start_cmd={self.args.policy_start_cmd}")
        print(f"set_free_mode_on_auto={self.args.set_free_mode_on_auto}")
        print("keys: 1,2,H,F,S,Q")
        print("")
        sys.stdout.flush()

    def _print_status(self) -> None:
        policy_running = self.policy_proc is not None and self.policy_proc.poll() is None
        rospy.loginfo(
            (
                "status mode=%s policy_running=%s route[left=%s right=%s opp=%s] "
                "feedback[left_gravity=%s right_gravity=%s left_torque=%s right_torque=%s]"
            ),
            self.mode,
            policy_running,
            self.selected_routes.get("left", ""),
            self.selected_routes.get("right", ""),
            self.selected_routes.get("opp", ""),
            self.selected_feedback_routes.get("left_gravity", ""),
            self.selected_feedback_routes.get("right_gravity", ""),
            self.selected_feedback_routes.get("left_torque", ""),
            self.selected_feedback_routes.get("right_torque", ""),
        )

    def _quit_safely(self) -> None:
        self._stop_policy_process("quit")
        self._hold_all_arms(best_effort=True)
        self._restore_default_feedback_routes(best_effort=True)
        self.mode = MODE_SAFE_HOLD
        self.shutdown_requested = True
        rospy.loginfo("Quit requested. System held safely.")

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._wait_for_services_and_topics()
        self._transition_or_safe_hold("startup_auto_policy", self._enter_mode_auto_policy)
        self._print_intro()

        try:
            with RawTerminal():
                while not self.shutdown_requested and not rospy.is_shutdown():
                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        continue
                    ch = sys.stdin.read(1)

                    if ch == "1":
                        if self._key_debounced("1"):
                            continue
                        self._transition_or_safe_hold(
                            "mode_1",
                            lambda: self._enter_mode_takeover(MODE_TAKEOVER_1),
                        )
                    elif ch == "2":
                        if self._key_debounced("2"):
                            continue
                        self._transition_or_safe_hold(
                            "mode_2",
                            lambda: self._enter_mode_takeover(MODE_TAKEOVER_2),
                        )
                    elif ch in ("h", "H"):
                        if self._key_debounced("H"):
                            continue
                        self._transition_or_safe_hold(
                            "mode_h",
                            lambda: self._enter_mode_takeover(MODE_TAKEOVER_H),
                        )
                    elif ch in ("f", "F"):
                        if self._key_debounced("F"):
                            continue
                        self._transition_or_safe_hold("mode_auto", self._enter_mode_auto_policy)
                    elif ch in ("s", "S"):
                        self._print_status()
                    elif ch in ("q", "Q"):
                        self._quit_safely()
                    elif ch == "\x03":
                        self.signal_requested = True
                        self.shutdown_requested = True
        finally:
            if self.signal_requested and not rospy.is_shutdown():
                self._quit_safely()
            self._close_policy_log_handle()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-start-cmd", default=DEFAULT_POLICY_START_CMD)
    parser.add_argument("--policy-stop-cmd", default="")
    parser.add_argument("--policy-hook-timeout-sec", type=float, default=15.0)
    parser.add_argument(
        "--policy-log-dir",
        default="/tmp/intervention_3arm_switch_test_logs",
        help="Directory for managed policy stdout/stderr logs.",
    )

    parser.add_argument("--left-mux-service", default="/robot/arm_left/joint_cmd_mux_select")
    parser.add_argument("--right-mux-service", default="/robot/arm_right/joint_cmd_mux_select")
    parser.add_argument("--opp-mux-service", default="/robot/arm_opp/joint_cmd_mux_select")
    parser.add_argument("--left-gravity-mux-service", default="/teleop/arm_left/gravity_feedback_mux_select")
    parser.add_argument("--right-gravity-mux-service", default="/teleop/arm_right/gravity_feedback_mux_select")
    parser.add_argument("--left-torque-mux-service", default="/teleop/arm_left/torque_feedback_mux_select")
    parser.add_argument("--right-torque-mux-service", default="/teleop/arm_right/torque_feedback_mux_select")

    parser.add_argument("--left-vla-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--right-vla-topic", default="/robot/arm_right/vla_joint_cmd")
    parser.add_argument("--opp-vla-topic", default="/robot/arm_opp/vla_joint_cmd")

    parser.add_argument("--left-hold-topic", default="/robot/arm_left/hold_joint_cmd")
    parser.add_argument("--right-hold-topic", default="/robot/arm_right/hold_joint_cmd")
    parser.add_argument("--opp-hold-topic", default="/robot/arm_opp/hold_joint_cmd")
    parser.add_argument("--hold-pub-repeat", type=int, default=3)
    parser.add_argument("--hold-pub-interval-sec", type=float, default=0.05)

    parser.add_argument("--robot-left-state-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--robot-right-state-topic", default="/robot/arm_right/joint_states_single")
    parser.add_argument("--robot-opp-state-topic", default="/robot/arm_opp/joint_states_single")
    parser.add_argument("--left-master-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--right-master-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument("--left-default-gravity-topic", default="/robot/arm_left/joint_states_compensated")
    parser.add_argument("--right-default-gravity-topic", default="/robot/arm_right/joint_states_compensated")
    parser.add_argument("--opp-gravity-topic", default="/robot/arm_opp/joint_states_compensated")
    parser.add_argument("--left-default-torque-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--right-default-torque-topic", default="/robot/arm_right/joint_states_single")
    parser.add_argument("--opp-torque-topic", default="/robot/arm_opp/joint_states_single")
    parser.add_argument(
        "--route-gravity-feedback",
        action="store_true",
        default=True,
        help="Switch teleop gravity feedback route with takeover modes (default: on).",
    )
    parser.add_argument(
        "--disable-route-gravity-feedback",
        action="store_false",
        dest="route_gravity_feedback",
        help="Disable teleop gravity feedback route switching.",
    )
    parser.add_argument(
        "--route-torque-feedback",
        action="store_true",
        default=True,
        help="Switch teleop torque feedback route with takeover modes (default: on).",
    )
    parser.add_argument(
        "--disable-route-torque-feedback",
        action="store_false",
        dest="route_torque_feedback",
        help="Disable teleop torque feedback route switching.",
    )

    parser.add_argument(
        "--set-free-mode-on-auto",
        action="store_true",
        default=True,
        help="Publish slave_follow_flag=false when entering auto policy mode (default: on).",
    )
    parser.add_argument(
        "--no-set-free-mode-on-auto",
        action="store_false",
        dest="set_free_mode_on_auto",
        help="Do not publish slave_follow_flag=false in auto policy mode.",
    )
    parser.add_argument("--slave-follow-flag-topic", default="/conrft_robot/slave_follow_flag")
    parser.add_argument("--free-mode-publish-repeat", type=int, default=3)
    parser.add_argument("--free-mode-publish-interval-sec", type=float, default=0.05)

    parser.add_argument(
        "--align-before-switch",
        action="store_true",
        default=True,
        help="Align teleop masters to target robot poses before 1/2/H route switch (default: on).",
    )
    parser.add_argument(
        "--disable-align-before-switch",
        action="store_false",
        dest="align_before_switch",
        help="Disable master alignment before route switch.",
    )
    parser.add_argument("--left-master-cmd-mux-service", default="/teleop/arm_left/master_cmd_mux_select")
    parser.add_argument("--right-master-cmd-mux-service", default="/teleop/arm_right/master_cmd_mux_select")
    parser.add_argument("--left-master-default-cmd-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--right-master-default-cmd-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument("--left-master-align-topic", default="/teleop/arm_left/alignment_joint_cmd")
    parser.add_argument("--right-master-align-topic", default="/teleop/arm_right/alignment_joint_cmd")
    parser.add_argument("--left-master-enable-service", default="/teleop/arm_left/enable_srv")
    parser.add_argument("--right-master-enable-service", default="/teleop/arm_right/enable_srv")
    parser.add_argument("--left-master-block-service", default="/teleop/arm_left/block_arm")
    parser.add_argument("--right-master-block-service", default="/teleop/arm_right/block_arm")
    parser.add_argument("--left-master-force-cmd-topic", default="/teleop/arm_left/master_force_cmd_flag")
    parser.add_argument("--right-master-force-cmd-topic", default="/teleop/arm_right/master_force_cmd_flag")
    parser.add_argument(
        "--align-force-teleop-flag",
        action="store_true",
        default=True,
        help="Before alignment, publish slave_follow_flag=false (default: on).",
    )
    parser.add_argument(
        "--disable-align-force-teleop-flag",
        action="store_false",
        dest="align_force_teleop_flag",
        help="Disable forced slave_follow_flag publish before alignment.",
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
        help="Disable enable_srv pre-step before alignment.",
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
        help="Disable block_arm(false) pre-step before alignment.",
    )
    parser.add_argument(
        "--align-force-master-cmd-mode",
        action="store_true",
        default=True,
        help="Publish /teleop/arm_*/master_force_cmd_flag around alignment (default: on).",
    )
    parser.add_argument(
        "--disable-align-force-master-cmd-mode",
        action="store_false",
        dest="align_force_master_cmd_mode",
        help="Disable master_force_cmd_flag publish around alignment.",
    )
    parser.add_argument("--align-force-master-cmd-repeat", type=int, default=3)
    parser.add_argument("--align-force-master-cmd-interval-sec", type=float, default=0.03)
    parser.add_argument("--align-master-rate-hz", type=float, default=80.0)
    parser.add_argument("--align-master-max-joint-speed", type=float, default=0.40)
    parser.add_argument("--align-master-min-duration-sec", type=float, default=0.30)
    parser.add_argument("--align-master-max-duration-sec", type=float, default=2.50)
    parser.add_argument("--align-master-timeout-sec", type=float, default=4.00)
    parser.add_argument("--align-master-tolerance-rad", type=float, default=0.070)
    parser.add_argument("--align-master-settle-sec", type=float, default=0.20)
    parser.add_argument(
        "--align-include-gripper",
        action="store_true",
        default=False,
        help="Include gripper joint in alignment checks.",
    )

    parser.add_argument("--service-timeout-sec", type=float, default=8.0)
    parser.add_argument("--debounce-ms", type=int, default=200)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.policy_hook_timeout_sec <= 0.0:
        raise RuntimeError("--policy-hook-timeout-sec must be > 0")
    if args.service_timeout_sec <= 0.0:
        raise RuntimeError("--service-timeout-sec must be > 0")
    if args.debounce_ms < 0:
        raise RuntimeError("--debounce-ms must be >= 0")
    if args.hold_pub_repeat <= 0:
        raise RuntimeError("--hold-pub-repeat must be > 0")
    if args.hold_pub_interval_sec < 0.0:
        raise RuntimeError("--hold-pub-interval-sec must be >= 0")
    if args.free_mode_publish_repeat <= 0:
        raise RuntimeError("--free-mode-publish-repeat must be > 0")
    if args.free_mode_publish_interval_sec < 0.0:
        raise RuntimeError("--free-mode-publish-interval-sec must be >= 0")
    if args.align_force_flag_repeat <= 0:
        raise RuntimeError("--align-force-flag-repeat must be > 0")
    if args.align_force_flag_interval_sec < 0.0:
        raise RuntimeError("--align-force-flag-interval-sec must be >= 0")
    if args.align_flag_subscriber_wait_sec < 0.0:
        raise RuntimeError("--align-flag-subscriber-wait-sec must be >= 0")
    if args.align_force_master_cmd_repeat <= 0:
        raise RuntimeError("--align-force-master-cmd-repeat must be > 0")
    if args.align_force_master_cmd_interval_sec < 0.0:
        raise RuntimeError("--align-force-master-cmd-interval-sec must be >= 0")
    if args.align_master_rate_hz <= 0.0:
        raise RuntimeError("--align-master-rate-hz must be > 0")
    if args.align_master_max_joint_speed <= 0.0:
        raise RuntimeError("--align-master-max-joint-speed must be > 0")
    if args.align_master_min_duration_sec < 0.0:
        raise RuntimeError("--align-master-min-duration-sec must be >= 0")
    if args.align_master_max_duration_sec < args.align_master_min_duration_sec:
        raise RuntimeError("--align-master-max-duration-sec must be >= --align-master-min-duration-sec")
    if args.align_master_timeout_sec <= 0.0:
        raise RuntimeError("--align-master-timeout-sec must be > 0")
    if args.align_master_tolerance_rad <= 0.0:
        raise RuntimeError("--align-master-tolerance-rad must be > 0")
    if args.align_master_settle_sec < 0.0:
        raise RuntimeError("--align-master-settle-sec must be >= 0")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        rospy.init_node("intervention_3arm_switch_test")
        runner = ThreeArmInterventionSwitch(args)
        runner.run()
        return 0
    except Exception as exc:
        print(f"[intervention_3arm_switch_test] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
