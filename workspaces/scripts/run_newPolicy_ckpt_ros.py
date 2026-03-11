#!/usr/bin/env python3
"""
Run ACT-variants checkpoints online inference and publish robot-side VLA commands.

This node is publish-only by design:
- It does NOT call mux services.
- It does NOT publish slave_follow_flag.
- It does NOT change teleop modes.

Example:
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
  source /home/jameszhao2004/catkin_ws/.venv_train_act/bin/activate
  python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/run_newPolicy_ckpt_ros.py \
    --checkpoint-dir /home/jameszhao2004/catkin_ws/outputs/train/act_20260218_smoke/checkpoints/000200/pretrained_model \
    --policy-type act
"""

from __future__ import annotations

import argparse
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage, JointState


try:
    import cv2  # type: ignore
except Exception as exc:  # pragma: no cover
    raise SystemExit("opencv-python is required in this environment.") from exc

try:
    import torch
except Exception as exc:  # pragma: no cover
    raise SystemExit("torch is required in this environment.") from exc

DEFAULT_JOINT_NAMES = "joint1,joint2,joint3,joint4,joint5,joint6,gripper"


@dataclass(frozen=True)
class GuardProfile:
    ema_alpha: float
    max_joint_step: float
    max_gripper_step: float
    gripper_min: float
    gripper_max: float


GUARD_PRESETS: Dict[str, GuardProfile] = {
    "conservative": GuardProfile(
        ema_alpha=0.25,
        max_joint_step=0.03,
        max_gripper_step=0.003,
        gripper_min=0.0,
        gripper_max=0.08,
    ),
    "medium": GuardProfile(
        ema_alpha=0.40,
        max_joint_step=0.05,
        max_gripper_step=0.005,
        gripper_min=0.0,
        gripper_max=0.08,
    ),
    "aggressive": GuardProfile(
        ema_alpha=0.60,
        max_joint_step=0.08,
        max_gripper_step=0.01,
        gripper_min=0.0,
        gripper_max=0.08,
    ),
}


@dataclass
class Snapshot:
    left_pos: np.ndarray
    left_effort: np.ndarray
    right_pos: np.ndarray
    right_effort: np.ndarray
    top_image: np.ndarray
    left_image: np.ndarray
    right_image: np.ndarray
    stamps: Tuple[float, float, float, float, float]


class PolicyPublisherNode:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.lock = threading.Lock()

        self.left_pos: Optional[np.ndarray] = None
        self.left_effort: Optional[np.ndarray] = None
        self.left_stamp: Optional[float] = None
        self.right_pos: Optional[np.ndarray] = None
        self.right_effort: Optional[np.ndarray] = None
        self.right_stamp: Optional[float] = None

        self.top_image: Optional[np.ndarray] = None
        self.top_stamp: Optional[float] = None
        self.left_image: Optional[np.ndarray] = None
        self.left_img_stamp: Optional[float] = None
        self.right_image: Optional[np.ndarray] = None
        self.right_img_stamp: Optional[float] = None

        self.prev_left_cmd: Optional[np.ndarray] = None
        self.prev_right_cmd: Optional[np.ndarray] = None
        self.cb_count = {
            "left_joint": 0,
            "right_joint": 0,
            "top_image": 0,
            "left_wrist_image": 0,
            "right_wrist_image": 0,
        }
        self.cb_raw_count = {
            "left_joint": 0,
            "right_joint": 0,
            "top_image": 0,
            "left_wrist_image": 0,
            "right_wrist_image": 0,
        }
        self.cb_drop_count = {
            "left_joint_short": 0,
            "right_joint_short": 0,
            "top_decode_fail": 0,
            "left_decode_fail": 0,
            "right_decode_fail": 0,
        }

        self.guard = GUARD_PRESETS[args.guard_profile]
        self.joint_names = self._parse_joint_names(args.joint_names)
        self.device = self._resolve_device(args.device)

        self.policy, self.preprocessor, self.postprocessor = self._load_policy_and_processors()

        self.pub_left = rospy.Publisher(args.out_left_topic, JointState, queue_size=1)
        self.pub_right = rospy.Publisher(args.out_right_topic, JointState, queue_size=1)

        # Keep strong references to subscribers. Otherwise Python GC can drop
        # them and callbacks may silently stop.
        self.subscribers = [
            rospy.Subscriber(
                args.robot_left_topic, JointState, self._cb_left_joint, queue_size=1, tcp_nodelay=True
            ),
            rospy.Subscriber(
                args.robot_right_topic, JointState, self._cb_right_joint, queue_size=1, tcp_nodelay=True
            ),
            rospy.Subscriber(
                args.top_camera_topic, CompressedImage, self._cb_top_image, queue_size=1, tcp_nodelay=True
            ),
            rospy.Subscriber(
                args.left_wrist_topic, CompressedImage, self._cb_left_image, queue_size=1, tcp_nodelay=True
            ),
            rospy.Subscriber(
                args.right_wrist_topic, CompressedImage, self._cb_right_image, queue_size=1, tcp_nodelay=True
            ),
        ]

    def _resolve_device(self, requested: str) -> str:
        req = requested.strip().lower()
        if req.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("Requested device=%s but CUDA is unavailable. Falling back to cpu.", requested)
            return "cpu"
        return requested

    def _load_policy_and_processors(self):
        ckpt = Path(self.args.checkpoint_dir).expanduser().resolve()
        if not ckpt.is_dir():
            raise RuntimeError(f"Checkpoint dir does not exist: {ckpt}")
        if not (ckpt / "config.json").is_file() or not (ckpt / "model.safetensors").is_file():
            raise RuntimeError(f"Checkpoint dir must contain config.json and model.safetensors: {ckpt}")

        cli_overrides = [
            "--n_action_steps",
            "1",
            "--temporal_ensemble_coeff",
            str(self.args.temporal_ensemble_coeff),
            "--device",
            self.device,
        ]

        rospy.loginfo("Loading ACT policy from: %s", ckpt)
        # policy = ACTPolicy.from_pretrained(str(ckpt), local_files_only=True, cli_overrides=cli_overrides)
        policy = ACTTemporalPolicy.from_pretrained(str(ckpt), local_files_only=True, cli_overrides=cli_overrides)
        preprocessor, postprocessor = make_pre_post_processors(
            policy.config,
            pretrained_path=str(ckpt),
            preprocessor_overrides={"device_processor": {"device": self.device}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )
        rospy.loginfo(
            "Policy loaded. device=%s temporal_ensemble_coeff=%.4f n_action_steps=%d",
            policy.config.device,
            policy.config.temporal_ensemble_coeff or 0.0,
            policy.config.n_action_steps,
        )
        return policy, preprocessor, postprocessor

    def _parse_joint_names(self, value: str):
        names = [x.strip() for x in value.split(",") if x.strip()]
        if len(names) != 7:
            raise RuntimeError(f"--joint-names must contain exactly 7 names, got {len(names)}")
        return names

    def _stamp_to_sec(self, stamp: rospy.Time) -> float:
        if stamp is not None and hasattr(stamp, "to_sec"):
            sec = float(stamp.to_sec())
            if sec > 0.0:
                return sec
        return rospy.Time.now().to_sec()

    def _decode_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size == 0:
            return None
        img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if img is None:
            return None
        if img.ndim == 2:
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        elif img.ndim == 3 and img.shape[2] == 4:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        elif img.ndim == 3 and img.shape[2] == 3:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            raise RuntimeError(f"Unsupported decoded image shape: {img.shape}")

        if rgb.shape[1] != self.args.image_width or rgb.shape[0] != self.args.image_height:
            rgb = cv2.resize(
                rgb,
                (self.args.image_width, self.args.image_height),
                interpolation=cv2.INTER_AREA,
            )
        chw = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
        return chw

    def _cb_left_joint(self, msg: JointState):
        with self.lock:
            self.cb_raw_count["left_joint"] += 1
        if len(msg.position) < 7 or len(msg.effort) < 7:
            with self.lock:
                self.cb_drop_count["left_joint_short"] += 1
            rospy.logwarn_throttle(
                2.0,
                "Left joint_states_single short dims. Need >=7 position and >=7 effort, got pos=%d effort=%d",
                len(msg.position),
                len(msg.effort),
            )
            return
        with self.lock:
            self.left_pos = np.asarray(msg.position[:7], dtype=np.float32)
            self.left_effort = np.asarray(msg.effort[:7], dtype=np.float32)
            self.left_stamp = self._stamp_to_sec(msg.header.stamp)
            self.cb_count["left_joint"] += 1

    def _cb_right_joint(self, msg: JointState):
        with self.lock:
            self.cb_raw_count["right_joint"] += 1
        if len(msg.position) < 7 or len(msg.effort) < 7:
            with self.lock:
                self.cb_drop_count["right_joint_short"] += 1
            rospy.logwarn_throttle(
                2.0,
                "Right joint_states_single short dims. Need >=7 position and >=7 effort, got pos=%d effort=%d",
                len(msg.position),
                len(msg.effort),
            )
            return
        with self.lock:
            self.right_pos = np.asarray(msg.position[:7], dtype=np.float32)
            self.right_effort = np.asarray(msg.effort[:7], dtype=np.float32)
            self.right_stamp = self._stamp_to_sec(msg.header.stamp)
            self.cb_count["right_joint"] += 1

    def _cb_top_image(self, msg: CompressedImage):
        try:
            with self.lock:
                self.cb_raw_count["top_image"] += 1
            chw = self._decode_compressed(msg)
            if chw is None:
                with self.lock:
                    self.cb_drop_count["top_decode_fail"] += 1
                rospy.logwarn_throttle(2.0, "Failed to decode top compressed image.")
                return
            with self.lock:
                self.top_image = chw
                self.top_stamp = self._stamp_to_sec(msg.header.stamp)
                self.cb_count["top_image"] += 1
        except Exception as exc:
            with self.lock:
                self.cb_drop_count["top_decode_fail"] += 1
            rospy.logerr_throttle(2.0, "Top image callback exception: %s", str(exc))

    def _cb_left_image(self, msg: CompressedImage):
        try:
            with self.lock:
                self.cb_raw_count["left_wrist_image"] += 1
            chw = self._decode_compressed(msg)
            if chw is None:
                with self.lock:
                    self.cb_drop_count["left_decode_fail"] += 1
                rospy.logwarn_throttle(2.0, "Failed to decode left wrist compressed image.")
                return
            with self.lock:
                self.left_image = chw
                self.left_img_stamp = self._stamp_to_sec(msg.header.stamp)
                self.cb_count["left_wrist_image"] += 1
        except Exception as exc:
            with self.lock:
                self.cb_drop_count["left_decode_fail"] += 1
            rospy.logerr_throttle(2.0, "Left wrist image callback exception: %s", str(exc))

    def _cb_right_image(self, msg: CompressedImage):
        try:
            with self.lock:
                self.cb_raw_count["right_wrist_image"] += 1
            chw = self._decode_compressed(msg)
            if chw is None:
                with self.lock:
                    self.cb_drop_count["right_decode_fail"] += 1
                rospy.logwarn_throttle(2.0, "Failed to decode right wrist compressed image.")
                return
            with self.lock:
                self.right_image = chw
                self.right_img_stamp = self._stamp_to_sec(msg.header.stamp)
                self.cb_count["right_wrist_image"] += 1
        except Exception as exc:
            with self.lock:
                self.cb_drop_count["right_decode_fail"] += 1
            rospy.logerr_throttle(2.0, "Right wrist image callback exception: %s", str(exc))

    def _snapshot(self) -> Optional[Snapshot]:
        with self.lock:
            if self._missing_streams_unlocked():
                return None
            return Snapshot(
                left_pos=self.left_pos.copy(),  # type: ignore[arg-type]
                left_effort=self.left_effort.copy(),  # type: ignore[arg-type]
                right_pos=self.right_pos.copy(),  # type: ignore[arg-type]
                right_effort=self.right_effort.copy(),  # type: ignore[arg-type]
                top_image=self.top_image.copy(),  # type: ignore[arg-type]
                left_image=self.left_image.copy(),  # type: ignore[arg-type]
                right_image=self.right_image.copy(),  # type: ignore[arg-type]
                stamps=(
                    float(self.left_stamp),  # type: ignore[arg-type]
                    float(self.right_stamp),  # type: ignore[arg-type]
                    float(self.top_stamp),  # type: ignore[arg-type]
                    float(self.left_img_stamp),  # type: ignore[arg-type]
                    float(self.right_img_stamp),  # type: ignore[arg-type]
                ),
            )

    def _missing_streams_unlocked(self):
        missing = []
        if self.left_pos is None or self.left_effort is None or self.left_stamp is None:
            missing.append("left_joint")
        if self.right_pos is None or self.right_effort is None or self.right_stamp is None:
            missing.append("right_joint")
        if self.top_image is None or self.top_stamp is None:
            missing.append("top_image")
        if self.left_image is None or self.left_img_stamp is None:
            missing.append("left_wrist_image")
        if self.right_image is None or self.right_img_stamp is None:
            missing.append("right_wrist_image")
        return missing

    def _missing_streams(self):
        with self.lock:
            return self._missing_streams_unlocked()

    def _debug_streams(self):
        with self.lock:
            rospy.loginfo_throttle(
                2.0,
                "debug streams raw=%s valid=%s drop=%s stamps(left,right,top,lw,rw)=(%s,%s,%s,%s,%s)",
                self.cb_raw_count,
                self.cb_count,
                self.cb_drop_count,
                "None" if self.left_stamp is None else f"{self.left_stamp:.3f}",
                "None" if self.right_stamp is None else f"{self.right_stamp:.3f}",
                "None" if self.top_stamp is None else f"{self.top_stamp:.3f}",
                "None" if self.left_img_stamp is None else f"{self.left_img_stamp:.3f}",
                "None" if self.right_img_stamp is None else f"{self.right_img_stamp:.3f}",
            )

    def _health_ok(self, snap: Snapshot, now_sec: float) -> Tuple[bool, str]:
        for ts in snap.stamps:
            if now_sec - ts > self.args.max_input_staleness_sec:
                return False, f"stale input (>{self.args.max_input_staleness_sec:.3f}s)"
        if max(snap.stamps) - min(snap.stamps) > self.args.max_sync_delta_sec:
            return False, f"sync delta too large (>{self.args.max_sync_delta_sec:.3f}s)"
        return True, ""

    def _infer_action(self, snap: Snapshot) -> np.ndarray:
        obs_state = np.concatenate(
            [snap.left_pos, snap.left_effort, snap.right_pos, snap.right_effort], axis=0
        ).astype(np.float32)
        obs = {
            "observation.state": obs_state.reshape(1, 28),
            "observation.image": snap.top_image.reshape(1, 3, self.args.image_height, self.args.image_width),
            "observation.left_wrist_image": snap.left_image.reshape(
                1, 3, self.args.image_height, self.args.image_width
            ),
            "observation.right_wrist_image": snap.right_image.reshape(
                1, 3, self.args.image_height, self.args.image_width
            ),
        }
        processed = self.preprocessor(obs)
        # Keep only policy-required keys and force device alignment.
        model_inputs = {}
        for key in self.policy.config.input_features.keys():
            if key not in processed:
                raise RuntimeError(f"Preprocessor output missing required key: {key}")
            value = processed[key]
            if isinstance(value, torch.Tensor):
                model_inputs[key] = value.to(self.policy.config.device, non_blocking=True)
            else:
                model_inputs[key] = value
        with torch.inference_mode():
            action = self.policy.select_action(model_inputs)
        
        action = self.postprocessor(action)
        if isinstance(action, torch.Tensor):
            action_np = action.detach().cpu().numpy()
        else:
            action_np = np.asarray(action)
        action_np = np.asarray(action_np, dtype=np.float32).reshape(-1)
        if action_np.size != 14:
            raise RuntimeError(f"Policy output must be 14-D, got {action_np.size}")
        if not np.all(np.isfinite(action_np)):
            raise RuntimeError("Policy output contains non-finite values.")
        return action_np

    def _apply_guard(
        self, raw_left: np.ndarray, raw_right: np.ndarray, snap: Snapshot
    ) -> Tuple[np.ndarray, np.ndarray]:
        if not self.args.enable_gripper:
            raw_left = raw_left.copy()
            raw_right = raw_right.copy()
            raw_left[6] = snap.left_pos[6]
            raw_right[6] = snap.right_pos[6]

        if self.prev_left_cmd is None:
            self.prev_left_cmd = snap.left_pos.copy()
        if self.prev_right_cmd is None:
            self.prev_right_cmd = snap.right_pos.copy()

        left = self._guard_single(raw_left, self.prev_left_cmd)
        right = self._guard_single(raw_right, self.prev_right_cmd)
        self.prev_left_cmd = left
        self.prev_right_cmd = right
        return left, right

    def _guard_single(self, raw: np.ndarray, prev: np.ndarray) -> np.ndarray:
        target = raw.copy()
        target[6] = np.clip(target[6], self.guard.gripper_min, self.guard.gripper_max)

        delta = target - prev
        delta[:6] = np.clip(delta[:6], -self.guard.max_joint_step, self.guard.max_joint_step)
        delta[6] = float(np.clip(delta[6], -self.guard.max_gripper_step, self.guard.max_gripper_step))
        stepped = prev + delta

        cmd = self.guard.ema_alpha * stepped + (1.0 - self.guard.ema_alpha) * prev
        cmd[6] = np.clip(cmd[6], self.guard.gripper_min, self.guard.gripper_max)
        return cmd.astype(np.float32)

    def _publish_joint(self, pub, position: np.ndarray, stamp: rospy.Time):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [float(v) for v in position]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.args.rate)
        rospy.loginfo("run_act_checkpoint_ros started")
        rospy.loginfo("publish-only mode: no mux/follow/teleop switching calls in this node")
        rospy.loginfo("Output topics: left=%s right=%s", self.args.out_left_topic, self.args.out_right_topic)
        if self.args.startup_grace_sec > 0.0:
            rospy.loginfo("Startup grace: %.2f s (waiting for subscriber handshakes)", self.args.startup_grace_sec)
            rospy.sleep(self.args.startup_grace_sec)

        while not rospy.is_shutdown():
            snap = self._snapshot()
            if snap is None:
                missing = ",".join(self._missing_streams())
                rospy.loginfo_throttle(2.0, "Waiting for required streams: %s", missing)
                if self.args.debug_streams:
                    self._debug_streams()
                rate.sleep()
                continue

            now = rospy.Time.now().to_sec()
            healthy, reason = self._health_ok(snap, now)
            if not healthy:
                rospy.logwarn_throttle(2.0, "Input unhealthy, skip publish: %s", reason)
                rate.sleep()
                continue

            try:
                action = self._infer_action(snap)
            except Exception as exc:
                rospy.logerr_throttle(2.0, "Inference failed: %s", str(exc))
                rate.sleep()
                continue

            left_raw = action[:7]
            right_raw = action[7:]
            left_cmd, right_cmd = self._apply_guard(left_raw, right_raw, snap)
            stamp = rospy.Time.now()
            self._publish_joint(self.pub_left, left_cmd, stamp)
            self._publish_joint(self.pub_right, right_cmd, stamp)
            rate.sleep()

def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint-dir", required=True, help="Path to checkpoint pretrained_model directory.")
    parser.add_argument("--policy-type", type=str, default='act', help="what policy to deploy")
    parser.add_argument("--device", default="cuda", help="Torch device, e.g. cuda or cpu.")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish/inference loop rate in Hz.")
    parser.add_argument(
        "--temporal-ensemble-coeff",
        type=float,
        default=0.01,
        help="ACT temporal ensemble coefficient. n_action_steps is forced to 1.",
    )
    parser.add_argument(
        "--guard-profile",
        choices=sorted(GUARD_PRESETS.keys()),
        default="medium",
        help="Command guard profile.",
    )
    parser.add_argument(
        "--enable-gripper",
        action="store_true",
        default=True,
        help="Enable gripper control from policy action dims 7/14 (default: enabled).",
    )
    parser.add_argument(
        "--disable-gripper",
        action="store_false",
        dest="enable_gripper",
        help="Disable gripper control; keep current gripper positions.",
    )
    parser.add_argument("--joint-names", default=DEFAULT_JOINT_NAMES, help="Comma-separated 7 joint names.")
    parser.add_argument(
        "--max-input-staleness-sec",
        type=float,
        default=0.30,
        help="Drop publish when any input stream is older than this threshold.",
    )
    parser.add_argument(
        "--max-sync-delta-sec",
        type=float,
        default=0.08,
        help="Drop publish when max stream timestamp skew exceeds this threshold.",
    )
    parser.add_argument("--image-width", type=int, default=256, help="Model image width.")
    parser.add_argument("--image-height", type=int, default=256, help="Model image height.")

    parser.add_argument("--robot-left-topic", default="/robot/arm_left/joint_states_single")
    parser.add_argument("--robot-right-topic", default="/robot/arm_right/joint_states_single")
    parser.add_argument("--top-camera-topic", default="/realsense_top/color/image_raw/compressed")
    parser.add_argument("--left-wrist-topic", default="/realsense_left/color/image_raw/compressed")
    parser.add_argument("--right-wrist-topic", default="/realsense_right/color/image_raw/compressed")
    parser.add_argument("--out-left-topic", default="/robot/arm_left/vla_joint_cmd")
    parser.add_argument("--out-right-topic", default="/robot/arm_right/vla_joint_cmd")
    parser.add_argument(
        "--startup-grace-sec",
        type=float,
        default=1.0,
        help="Initial wait before input checks, to allow ROS subscriber handshakes.",
    )
    parser.add_argument(
        "--debug-streams",
        action="store_true",
        help="Print callback counters/timestamps while waiting for inputs.",
    )
    return parser


def main():
    parser = make_parser()
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    if args.temporal_ensemble_coeff <= 0.0:
        parser.error("--temporal-ensemble-coeff must be > 0")
    if args.max_input_staleness_sec <= 0.0:
        parser.error("--max-input-staleness-sec must be > 0")
    if args.max_sync_delta_sec <= 0.0:
        parser.error("--max-sync-delta-sec must be > 0")
    if args.startup_grace_sec < 0.0:
        parser.error("--startup-grace-sec must be >= 0")

    if args.policy_type == 'act':
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy
            from lerobot.policies.factory import make_pre_post_processors
        except Exception as exc:  # pragma: no cover
            raise SystemExit(
                "lerobot is required in this environment. Use Python>=3.10 venv in ROS container."
            ) from exc
    elif args.policy_type == 'act-temporal':
        try:
            from lerobot_policy_act_temporal import ACTTemporalPolicy
            from lerobot.policies.factory import make_pre_post_processors
        except Exception as exc:  # pragma: no cover
            raise SystemExit(
                "lerobot_policy_act_temporal is required in this environment. Use Python>=3.10 venv in ROS container."
            ) from exc

    rospy.init_node("run_act_checkpoint_ros")
    node = PolicyPublisherNode(args)
    node.run()
    


if __name__ == "__main__":
    main()
