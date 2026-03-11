#!/usr/bin/env python3
"""Publish dual-slave manual command sequence.

Phase 1:
- Slowly open gripper only (other joints hold current positions).

Phase 2:
- Slowly move all 7 joints to zero for each arm.

This node is publish-only:
- no mux service calls
- no mode switching
"""

from __future__ import annotations

import argparse
import threading
from typing import Optional, Tuple

import numpy as np
import rospy
from sensor_msgs.msg import JointState


PER_ARM_DIM = 7
DEFAULT_JOINT_NAMES = "joint1,joint2,joint3,joint4,joint5,joint6,gripper"


class TwoSlaveZeroNode:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.lock = threading.Lock()
        self.joint_names = self._parse_joint_names(args.joint_names)

        self.left_pos: Optional[np.ndarray] = None
        self.right_pos: Optional[np.ndarray] = None
        self.left_stamp: Optional[float] = None
        self.right_stamp: Optional[float] = None

        self.pub_left = rospy.Publisher(args.out_left_topic, JointState, queue_size=1)
        self.pub_right = rospy.Publisher(args.out_right_topic, JointState, queue_size=1)

        # Keep strong refs to avoid callback drop by GC.
        self.subscribers = [
            rospy.Subscriber(
                args.robot_left_topic, JointState, self._cb_left_joint, queue_size=1, tcp_nodelay=True
            ),
            rospy.Subscriber(
                args.robot_right_topic, JointState, self._cb_right_joint, queue_size=1, tcp_nodelay=True
            ),
        ]

    def _parse_joint_names(self, raw: str):
        names = [x.strip() for x in raw.split(",") if x.strip()]
        if len(names) != PER_ARM_DIM:
            raise RuntimeError(f"--joint-names must contain exactly {PER_ARM_DIM} names, got {len(names)}")
        return names

    def _stamp_to_sec(self, stamp: rospy.Time) -> float:
        if stamp is not None and hasattr(stamp, "to_sec"):
            sec = float(stamp.to_sec())
            if sec > 0.0:
                return sec
        return rospy.Time.now().to_sec()

    def _cb_left_joint(self, msg: JointState):
        if len(msg.position) < PER_ARM_DIM:
            rospy.logwarn_throttle(
                2.0, "Left state short dims. Need >=%d position, got %d", PER_ARM_DIM, len(msg.position)
            )
            return
        with self.lock:
            self.left_pos = np.asarray(msg.position[:PER_ARM_DIM], dtype=np.float32)
            self.left_stamp = self._stamp_to_sec(msg.header.stamp)

    def _cb_right_joint(self, msg: JointState):
        if len(msg.position) < PER_ARM_DIM:
            rospy.logwarn_throttle(
                2.0, "Right state short dims. Need >=%d position, got %d", PER_ARM_DIM, len(msg.position)
            )
            return
        with self.lock:
            self.right_pos = np.asarray(msg.position[:PER_ARM_DIM], dtype=np.float32)
            self.right_stamp = self._stamp_to_sec(msg.header.stamp)

    def _snapshot(self) -> Optional[Tuple[np.ndarray, np.ndarray, Tuple[float, float]]]:
        with self.lock:
            if self.left_pos is None or self.right_pos is None:
                return None
            return (
                self.left_pos.copy(),
                self.right_pos.copy(),
                (float(self.left_stamp), float(self.right_stamp)),
            )

    def _publish_single(self, pub, position: np.ndarray, stamp: rospy.Time):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [float(v) for v in position]
        msg.velocity = [0.0] * PER_ARM_DIM
        msg.effort = [0.0] * PER_ARM_DIM
        pub.publish(msg)

    def _publish_pair(self, left: np.ndarray, right: np.ndarray):
        stamp = rospy.Time.now()
        self._publish_single(self.pub_left, left, stamp)
        self._publish_single(self.pub_right, right, stamp)

    @staticmethod
    def _lerp(start: np.ndarray, end: np.ndarray, alpha: float) -> np.ndarray:
        return (start + alpha * (end - start)).astype(np.float32)

    def _run_phase(
        self,
        left_start: np.ndarray,
        right_start: np.ndarray,
        left_end: np.ndarray,
        right_end: np.ndarray,
        duration_sec: float,
        rate: rospy.Rate,
        phase_name: str,
    ):
        t0 = rospy.Time.now().to_sec()
        duration = max(0.001, float(duration_sec))
        rospy.loginfo("%s started: duration=%.2f sec", phase_name, duration)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            alpha = min(1.0, max(0.0, (now - t0) / duration))
            left_cmd = self._lerp(left_start, left_end, alpha)
            right_cmd = self._lerp(right_start, right_end, alpha)
            self._publish_pair(left_cmd, right_cmd)
            if alpha >= 1.0:
                break
            rate.sleep()
        # Publish final once more to make endpoint explicit.
        self._publish_pair(left_end, right_end)
        rospy.loginfo("%s completed", phase_name)

    def run(self):
        rate = rospy.Rate(self.args.rate)
        if self.args.startup_grace_sec > 0.0:
            rospy.loginfo("Startup grace: %.2f sec", self.args.startup_grace_sec)
            rospy.sleep(self.args.startup_grace_sec)

        rospy.loginfo("run_gripper_open_then_zero_ros_2slave started")
        rospy.loginfo(
            "Output topics: left=%s right=%s",
            self.args.out_left_topic,
            self.args.out_right_topic,
        )

        while not rospy.is_shutdown():
            snap = self._snapshot()
            if snap is None:
                rospy.loginfo_throttle(2.0, "Waiting for 2-arm state streams...")
                rate.sleep()
                continue
            left_start, right_start, stamps = snap
            if self.args.debug_streams:
                rospy.loginfo(
                    "Initial stamps(left,right)=(%.3f,%.3f)",
                    stamps[0], stamps[1],
                )
            break

        if rospy.is_shutdown():
            return

        # Phase 1: open gripper only, keep other joints unchanged.
        open_pos = float(np.clip(self.args.gripper_open_pos, self.args.gripper_min, self.args.gripper_max))
        left_open = left_start.copy()
        right_open = right_start.copy()
        left_open[6] = open_pos
        right_open[6] = open_pos
        self._run_phase(
            left_start, right_start,
            left_open, right_open,
            self.args.open_duration_sec, rate,
            phase_name="phase1 gripper_open",
        )

        # Phase 2: whole arm slowly returns to zero (all 7 joints).
        left_zero = np.zeros(PER_ARM_DIM, dtype=np.float32)
        right_zero = np.zeros(PER_ARM_DIM, dtype=np.float32)
        self._run_phase(
            left_open, right_open,
            left_zero, right_zero,
            self.args.zero_duration_sec, rate,
            phase_name="phase2 all_joint_zero",
        )

        if self.args.done_mode == "exit":
            rospy.loginfo("Done mode=exit, node exits now.")
            return

        rospy.loginfo("Done mode=hold, keep publishing zero commands.")
        while not rospy.is_shutdown():
            self._publish_pair(left_zero, right_zero)
            rate.sleep()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz).")
    parser.add_argument("--open-duration-sec", type=float, default=6.0, help="Phase1 duration: gripper open.")
    parser.add_argument("--zero-duration-sec", type=float, default=12.0, help="Phase2 duration: all joints to zero.")
    parser.add_argument("--gripper-open-pos", type=float, default=0.08, help="Target gripper open position.")
    parser.add_argument("--gripper-min", type=float, default=0.0)
    parser.add_argument("--gripper-max", type=float, default=0.08)
    parser.add_argument("--joint-names", default=DEFAULT_JOINT_NAMES)
    parser.add_argument(
        "--done-mode",
        choices=["hold", "exit"],
        default="hold",
        help="After finishing sequence: hold zero command or exit.",
    )
    parser.add_argument("--startup-grace-sec", type=float, default=1.0)
    parser.add_argument("--debug-streams", action="store_true")

    parser.add_argument("--robot-left-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--robot-right-topic", default="/robot/arm_right/joint_states_single")

    parser.add_argument("--out-left-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--out-right-topic", default="/robot/arm_right/vla_joint_cmd")
    return parser


def main():
    parser = make_parser()
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.open_duration_sec <= 0.0:
        parser.error("--open-duration-sec must be > 0")
    if args.zero_duration_sec <= 0.0:
        parser.error("--zero-duration-sec must be > 0")
    if args.gripper_min > args.gripper_max:
        parser.error("--gripper-min must be <= --gripper-max")
    if args.startup_grace_sec < 0.0:
        parser.error("--startup-grace-sec must be >= 0")

    rospy.init_node("run_gripper_open_then_zero_ros_2slave")
    node = TwoSlaveZeroNode(args)
    node.run()


if __name__ == "__main__":
    main()
