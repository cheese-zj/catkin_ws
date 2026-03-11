#!/usr/bin/env python3
"""Replay 3-arm command topics from a rosbag to VLA command topics.

Default source mapping (from intervention/3arm recording):
  - /teleop/arm_left/joint_states_single  -> /robot/arm_left/vla_joint_cmd
  - /teleop/arm_right/joint_states_single -> /robot/arm_right/vla_joint_cmd
  - /robot/arm_opp/joint_cmd_mux          -> /robot/arm_opp/vla_joint_cmd

This tool is publish-only. It does not call mux services.
"""

from __future__ import annotations

import argparse
import time
from typing import Dict, Iterable, List, Tuple

import rosbag
import rospy
from sensor_msgs.msg import JointState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to episode.bag")
    parser.add_argument("--rate", type=float, default=1.0, help="Replay speed multiplier (>0).")
    parser.add_argument("--start-offset-sec", type=float, default=0.0, help="Skip this many seconds from bag start.")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="Replay at most this many seconds after start offset (0 means full).",
    )
    parser.add_argument("--loop", action="store_true", help="Loop replay until Ctrl-C.")
    parser.add_argument(
        "--wait-subscribers-sec",
        type=float,
        default=2.0,
        help="Wait up to this many seconds for output subscribers.",
    )
    parser.add_argument("--robot-left-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--robot-right-topic", default="/robot/arm_right/vla_joint_cmd")
    parser.add_argument("--robot-opp-topic", default="/robot/arm_opp/vla_joint_cmd")
    parser.add_argument("--src-left-topic", default="/teleop/arm_left/joint_states_single")
    parser.add_argument("--src-right-topic", default="/teleop/arm_right/joint_states_single")
    parser.add_argument("--src-opp-topic", default="/robot/arm_opp/joint_cmd_mux")
    parser.add_argument("--trim-dim", type=int, default=7, help="Trim JointState position/velocity/effort to this dim.")
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def _is_joint_state_like(msg: object) -> bool:
    return hasattr(msg, "position") and hasattr(msg, "velocity") and hasattr(msg, "effort")


def _trim_joint_state(msg: object, dim: int) -> JointState:
    out = JointState()
    out.header.stamp = rospy.Time.now()
    msg_name = getattr(msg, "name", [])
    msg_position = list(getattr(msg, "position", []) or [])
    msg_velocity = list(getattr(msg, "velocity", []) or [])
    msg_effort = list(getattr(msg, "effort", []) or [])
    out.name = list(msg_name[:dim]) if msg_name else []
    out.position = [float(v) for v in msg_position[:dim]]
    out.velocity = [float(v) for v in msg_velocity[:dim]] if msg_velocity else [0.0] * len(out.position)
    out.effort = [float(v) for v in msg_effort[:dim]] if msg_effort else [0.0] * len(out.position)
    if len(out.velocity) < len(out.position):
        out.velocity.extend([0.0] * (len(out.position) - len(out.velocity)))
    if len(out.effort) < len(out.position):
        out.effort.extend([0.0] * (len(out.position) - len(out.effort)))
    return out


def _collect_selected_messages(
    bag_path: str,
    selected_topics: Iterable[str],
    start_offset_sec: float,
    duration_sec: float,
) -> List[Tuple[str, JointState, float]]:
    topics = list(selected_topics)
    rows: List[Tuple[str, JointState, float]] = []
    with rosbag.Bag(bag_path, "r") as bag:
        bag_start = bag.get_start_time()
        t_start = bag_start + max(0.0, float(start_offset_sec))
        t_end = float("inf")
        if duration_sec > 0.0:
            t_end = t_start + float(duration_sec)

        for topic, msg, t in bag.read_messages(topics=topics):
            if not _is_joint_state_like(msg):
                continue
            ts = float(t.to_sec())
            if ts < t_start:
                continue
            if ts > t_end:
                break
            rows.append((topic, msg, ts))
    rows.sort(key=lambda x: x[2])
    return rows


def _selected_topic_stats(bag_path: str, selected_topics: Iterable[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    with rosbag.Bag(bag_path, "r") as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics
        for topic in selected_topics:
            meta = topics.get(topic)
            if meta is None:
                out[topic] = "missing"
            else:
                out[topic] = f"type={meta.msg_type}, count={meta.message_count}"
    return out


def _wait_for_subscribers(pubs: Iterable[rospy.Publisher], timeout_sec: float) -> None:
    if timeout_sec <= 0.0:
        return
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if all(pub.get_num_connections() > 0 for pub in pubs):
            return
        rospy.sleep(0.05)


def _run_once(
    rows: List[Tuple[str, JointState, float]],
    topic_to_pub: Dict[str, rospy.Publisher],
    rate_scale: float,
    trim_dim: int,
    debug: bool,
) -> Dict[str, int]:
    if not rows:
        return {}
    counts: Dict[str, int] = {k: 0 for k in topic_to_pub.keys()}
    t0_bag = rows[0][2]
    t0_wall = time.monotonic()
    for topic, msg, ts in rows:
        if rospy.is_shutdown():
            break
        target_wall = t0_wall + (ts - t0_bag) / rate_scale
        sleep_sec = target_wall - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)
        out = _trim_joint_state(msg, trim_dim)
        topic_to_pub[topic].publish(out)
        counts[topic] = counts.get(topic, 0) + 1
    if debug:
        rospy.loginfo("replay counts=%s", counts)
    return counts


def main() -> int:
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0")
    if args.start_offset_sec < 0.0:
        raise SystemExit("--start-offset-sec must be >= 0")
    if args.duration_sec < 0.0:
        raise SystemExit("--duration-sec must be >= 0")
    if args.trim_dim <= 0:
        raise SystemExit("--trim-dim must be > 0")

    rospy.init_node("replay_rosbag_3arm_vla")

    pub_left = rospy.Publisher(args.robot_left_topic, JointState, queue_size=1)
    pub_right = rospy.Publisher(args.robot_right_topic, JointState, queue_size=1)
    pub_opp = rospy.Publisher(args.robot_opp_topic, JointState, queue_size=1)

    selected_topics = [args.src_left_topic, args.src_right_topic, args.src_opp_topic]
    topic_to_pub = {
        args.src_left_topic: pub_left,
        args.src_right_topic: pub_right,
        args.src_opp_topic: pub_opp,
    }

    rows = _collect_selected_messages(
        bag_path=args.bag,
        selected_topics=selected_topics,
        start_offset_sec=float(args.start_offset_sec),
        duration_sec=float(args.duration_sec),
    )
    if not rows:
        stats = _selected_topic_stats(args.bag, selected_topics)
        rospy.logerr("No JointState-like messages found in selected topics/time range.")
        for topic in selected_topics:
            rospy.logerr("topic check: %s -> %s", topic, stats.get(topic, "unknown"))
        return 1

    rospy.loginfo("Loaded %d replay messages from bag: %s", len(rows), args.bag)
    rospy.loginfo(
        "Source -> output mapping: %s->%s, %s->%s, %s->%s",
        args.src_left_topic,
        args.robot_left_topic,
        args.src_right_topic,
        args.robot_right_topic,
        args.src_opp_topic,
        args.robot_opp_topic,
    )
    _wait_for_subscribers([pub_left, pub_right, pub_opp], float(args.wait_subscribers_sec))

    while not rospy.is_shutdown():
        _run_once(
            rows=rows,
            topic_to_pub=topic_to_pub,
            rate_scale=float(args.rate),
            trim_dim=int(args.trim_dim),
            debug=bool(args.debug),
        )
        if not args.loop:
            break
        rospy.loginfo("Replay loop restart")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
