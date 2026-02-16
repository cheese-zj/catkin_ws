#!/usr/bin/env python3
"""Repeat A/B joint targets to fake a policy command stream.

Example:
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
  python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/fake_policy_loop.py

Optional custom waypoints:
  python3 fake_policy_loop.py \
    --left-waypoint-a "0.10,0.20,0,0,0,0,0.02" \
    --left-waypoint-b "0.20,0.10,0,0,0,0,0.04"
"""

from __future__ import annotations

import argparse
import ast
import math
from typing import List

import rospy
from sensor_msgs.msg import JointState


DEFAULT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
# Slightly larger default motion than the original demo path.
DEFAULT_LEFT_A = [0.08, 0.22, 0.60, 0.05, 0.50, 0.00, 0.02]
DEFAULT_LEFT_B = [0.42, 0.08, 0.60, 0.05, 0.00, 0.00, 0.04]


def parse_waypoint(text: str, expected_len: int, arg_name: str) -> List[float]:
    """Parse waypoint from '[..]' or 'v1,v2,...' format."""
    raw = text.strip()
    if not raw:
        raise argparse.ArgumentTypeError(f"{arg_name} must not be empty")

    try:
        if raw.startswith("["):
            values = ast.literal_eval(raw)
        else:
            values = [v.strip() for v in raw.split(",")]
    except Exception as exc:
        raise argparse.ArgumentTypeError(f"{arg_name} parse failed: {exc}") from exc

    if not isinstance(values, (list, tuple)):
        raise argparse.ArgumentTypeError(f"{arg_name} must be a list/tuple or comma-separated values")

    try:
        out = [float(v) for v in values]
    except Exception as exc:
        raise argparse.ArgumentTypeError(f"{arg_name} contains non-numeric value: {exc}") from exc

    if len(out) != expected_len:
        raise argparse.ArgumentTypeError(f"{arg_name} length must be {expected_len}, got {len(out)}")

    return out


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--right-topic", default="/robot/arm_right/vla_joint_cmd")
    parser.add_argument("--disable-left", action="store_true", help="Do not publish left arm command")
    parser.add_argument("--disable-right", action="store_true", help="Do not publish right arm command")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate (Hz)")
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=2.0,
        help="A->B travel time (seconds) for smooth cosine replay",
    )
    parser.add_argument(
        "--names",
        default=",".join(DEFAULT_NAMES),
        help="Joint names, comma-separated. Must match waypoint length.",
    )
    parser.add_argument(
        "--left-waypoint-a",
        default=",".join(str(v) for v in DEFAULT_LEFT_A),
        help="Left arm waypoint A, list or comma-separated floats",
    )
    parser.add_argument(
        "--left-waypoint-b",
        default=",".join(str(v) for v in DEFAULT_LEFT_B),
        help="Left arm waypoint B, list or comma-separated floats",
    )
    parser.add_argument(
        "--right-waypoint-a",
        default="",
        help="Right arm waypoint A, if empty uses left-waypoint-a",
    )
    parser.add_argument(
        "--right-waypoint-b",
        default="",
        help="Right arm waypoint B, if empty uses left-waypoint-b",
    )
    return parser


def parse_joint_names(names_arg: str) -> List[str]:
    names = [s.strip() for s in names_arg.split(",") if s.strip()]
    if not names:
        raise ValueError("--names must contain at least one joint name")
    return names


def build_joint_state(names: List[str], position: List[float], stamp: rospy.Time) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = names
    msg.position = position
    msg.velocity = [0.0] * len(names)
    msg.effort = [0.0] * len(names)
    return msg


def interpolate(a: List[float], b: List[float], alpha: float) -> List[float]:
    return [av + alpha * (bv - av) for av, bv in zip(a, b)]


def main() -> None:
    parser = make_parser()
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.hold_sec <= 0.0:
        parser.error("--hold-sec must be > 0")
    if args.disable_left and args.disable_right:
        parser.error("Both arms are disabled. Enable at least one publisher.")

    names = parse_joint_names(args.names)
    n = len(names)

    left_a = parse_waypoint(args.left_waypoint_a, n, "--left-waypoint-a")
    left_b = parse_waypoint(args.left_waypoint_b, n, "--left-waypoint-b")
    right_a = parse_waypoint(args.right_waypoint_a, n, "--right-waypoint-a") if args.right_waypoint_a else left_a
    right_b = parse_waypoint(args.right_waypoint_b, n, "--right-waypoint-b") if args.right_waypoint_b else left_b

    rospy.init_node("fake_policy_loop")

    pub_left = None if args.disable_left else rospy.Publisher(args.left_topic, JointState, queue_size=1)
    pub_right = None if args.disable_right else rospy.Publisher(args.right_topic, JointState, queue_size=1)

    rospy.loginfo("fake_policy_loop started")
    if pub_left is not None:
        rospy.loginfo("  left topic: %s", args.left_topic)
        rospy.loginfo("  left A: %s", [round(v, 4) for v in left_a])
        rospy.loginfo("  left B: %s", [round(v, 4) for v in left_b])
    if pub_right is not None:
        rospy.loginfo("  right topic: %s", args.right_topic)
        rospy.loginfo("  right A: %s", [round(v, 4) for v in right_a])
        rospy.loginfo("  right B: %s", [round(v, 4) for v in right_b])
    rospy.loginfo("  rate: %.2f Hz, hold: %.2f s", args.rate, args.hold_sec)

    rate = rospy.Rate(args.rate)
    start_time = rospy.Time.now()
    rospy.loginfo("Publishing smooth replay (cosine interpolation)")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # alpha in [0, 1], periodic and C1-smooth at A/B endpoints.
        elapsed = max(0.0, (now - start_time).to_sec())
        alpha = 0.5 * (1.0 - math.cos(math.pi * elapsed / args.hold_sec))
        left_pos = interpolate(left_a, left_b, alpha)
        right_pos = interpolate(right_a, right_b, alpha)

        if pub_left is not None:
            pub_left.publish(build_joint_state(names, left_pos, now))
        if pub_right is not None:
            pub_right.publish(build_joint_state(names, right_pos, now))
        rate.sleep()


if __name__ == "__main__":
    main()
