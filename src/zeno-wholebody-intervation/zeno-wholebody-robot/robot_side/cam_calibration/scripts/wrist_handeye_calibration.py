#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure wrist-camera AX=XB hand-eye calibration using a fixed AprilTag."""

from __future__ import annotations

import json
import math
import os
import threading
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse


def normalize_quaternion_xyzw(quat_xyzw: Sequence[float]) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is zero")
    return quat / norm


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    R = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return normalize_quaternion_xyzw([qx, qy, qz, qw])


def quaternion_xyzw_to_rotation_matrix(quat_xyzw: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = normalize_quaternion_xyzw(quat_xyzw)
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def quaternion_conjugate_xyzw(quat_xyzw: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = normalize_quaternion_xyzw(quat_xyzw)
    return np.asarray([-qx, -qy, -qz, qw], dtype=np.float64)


def quaternion_multiply_xyzw(lhs_xyzw: Sequence[float], rhs_xyzw: Sequence[float]) -> np.ndarray:
    lx, ly, lz, lw = normalize_quaternion_xyzw(lhs_xyzw)
    rx, ry, rz, rw = normalize_quaternion_xyzw(rhs_xyzw)
    return normalize_quaternion_xyzw(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]
    )


def quaternion_angular_error_deg(ref_xyzw: Sequence[float], sample_xyzw: Sequence[float]) -> float:
    delta = quaternion_multiply_xyzw(quaternion_conjugate_xyzw(ref_xyzw), sample_xyzw)
    w = max(-1.0, min(1.0, abs(float(delta[3]))))
    return math.degrees(2.0 * math.acos(w))


def average_quaternions_xyzw(quat_xyzw_arr: np.ndarray) -> np.ndarray:
    quats = np.asarray(quat_xyzw_arr, dtype=np.float64).reshape(-1, 4)
    if quats.shape[0] == 0:
        raise ValueError("No quaternions to average")
    ref = normalize_quaternion_xyzw(quats[0])
    aligned = []
    for quat in quats:
        q = normalize_quaternion_xyzw(quat)
        if float(np.dot(q, ref)) < 0.0:
            q = -q
        aligned.append(q)
    A = np.zeros((4, 4), dtype=np.float64)
    for quat in aligned:
        A += np.outer(quat, quat)
    eigvals, eigvecs = np.linalg.eigh(A)
    mean_quat = normalize_quaternion_xyzw(eigvecs[:, int(np.argmax(eigvals))])
    if float(np.dot(mean_quat, ref)) < 0.0:
        mean_quat = -mean_quat
    return mean_quat


def quaternion_xyzw_to_euler_xyz_deg(quat_xyzw: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = normalize_quaternion_xyzw(quat_xyzw)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.asarray([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)], dtype=np.float64)


def pose_is_valid(pose: PoseStamped) -> bool:
    values = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    return all(math.isfinite(float(val)) for val in values)


def pose_to_rt(pose: PoseStamped) -> Tuple[np.ndarray, np.ndarray]:
    p = pose.pose.position
    o = pose.pose.orientation
    rotation = quaternion_xyzw_to_rotation_matrix([o.x, o.y, o.z, o.w])
    translation = np.asarray([p.x, p.y, p.z], dtype=np.float64).reshape(3, 1)
    return rotation, translation


def rt_to_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def transform_to_pose_dict(rotation: np.ndarray, translation: np.ndarray, frame_id: str) -> Dict[str, object]:
    quat = rotation_matrix_to_quaternion_xyzw(rotation)
    t = np.asarray(translation, dtype=np.float64).reshape(3)
    return {
        "frame_id": str(frame_id),
        "position": {
            "x": float(t[0]),
            "y": float(t[1]),
            "z": float(t[2]),
        },
        "orientation": {
            "x": float(quat[0]),
            "y": float(quat[1]),
            "z": float(quat[2]),
            "w": float(quat[3]),
        },
    }


def compute_base_tag_transforms(
    R_base2gripper_samples: Sequence[np.ndarray],
    t_base2gripper_samples: Sequence[np.ndarray],
    R_cam2target_samples: Sequence[np.ndarray],
    t_cam2target_samples: Sequence[np.ndarray],
    R_gripper2cam: np.ndarray,
    t_gripper2cam: np.ndarray,
) -> List[np.ndarray]:
    T_gripper2cam = rt_to_transform(R_gripper2cam, t_gripper2cam)
    transforms: List[np.ndarray] = []
    for R_b2g, t_b2g, R_c2t, t_c2t in zip(
        R_base2gripper_samples,
        t_base2gripper_samples,
        R_cam2target_samples,
        t_cam2target_samples,
    ):
        T_base2gripper = rt_to_transform(R_b2g, t_b2g)
        T_cam2target = rt_to_transform(R_c2t, t_c2t)
        transforms.append(T_base2gripper @ T_gripper2cam @ T_cam2target)
    return transforms


def compute_consistency_metrics(
    R_base2gripper_samples: Sequence[np.ndarray],
    t_base2gripper_samples: Sequence[np.ndarray],
    R_cam2target_samples: Sequence[np.ndarray],
    t_cam2target_samples: Sequence[np.ndarray],
    R_gripper2cam: np.ndarray,
    t_gripper2cam: np.ndarray,
) -> Dict[str, object]:
    transforms = compute_base_tag_transforms(
        R_base2gripper_samples,
        t_base2gripper_samples,
        R_cam2target_samples,
        t_cam2target_samples,
        R_gripper2cam,
        t_gripper2cam,
    )
    if not transforms:
        raise ValueError("No transforms available for consistency metrics")

    positions = np.asarray([T[:3, 3] for T in transforms], dtype=np.float64).reshape(-1, 3)
    quats = np.asarray(
        [rotation_matrix_to_quaternion_xyzw(T[:3, :3]) for T in transforms],
        dtype=np.float64,
    ).reshape(-1, 4)

    position_median = np.median(positions, axis=0)
    position_std = np.std(positions, axis=0, ddof=0)
    position_residuals = positions - position_median
    position_residual_norm = np.linalg.norm(position_residuals, axis=1)

    mean_quat = average_quaternions_xyzw(quats)
    orientation_errors = np.asarray(
        [quaternion_angular_error_deg(mean_quat, quat) for quat in quats],
        dtype=np.float64,
    )

    thresholds = {
        "translation_std_axis_max_m": 0.01,
        "translation_residual_norm_median_m": 0.015,
        "orientation_error_median_deg": 2.0,
    }
    passes = {
        "translation_std_axis_max_m": bool(np.all(position_std < thresholds["translation_std_axis_max_m"])),
        "translation_residual_norm_median_m": bool(
            float(np.median(position_residual_norm)) < thresholds["translation_residual_norm_median_m"]
        ),
        "orientation_error_median_deg": bool(
            float(np.median(orientation_errors)) < thresholds["orientation_error_median_deg"]
        ),
    }

    return {
        "num_samples": int(len(transforms)),
        "base_tag_position_median_m": [float(x) for x in position_median],
        "base_tag_position_std_m": [float(x) for x in position_std],
        "base_tag_position_residual_norm_median_m": float(np.median(position_residual_norm)),
        "base_tag_position_residual_norm_max_m": float(np.max(position_residual_norm)),
        "base_tag_orientation_mean_quaternion_xyzw": [float(x) for x in mean_quat],
        "base_tag_orientation_mean_euler_xyz_deg": [
            float(x) for x in quaternion_xyzw_to_euler_xyz_deg(mean_quat)
        ],
        "base_tag_orientation_error_deg_median": float(np.median(orientation_errors)),
        "base_tag_orientation_error_deg_std": float(np.std(orientation_errors, ddof=0)),
        "base_tag_orientation_error_deg_max": float(np.max(orientation_errors)),
        "thresholds": thresholds,
        "passes": passes,
        "all_pass": bool(all(passes.values())),
    }


def resolve_handeye_method(method_name: str) -> int:
    lookup = {
        "TSAI": getattr(cv2, "CALIB_HAND_EYE_TSAI", None),
        "PARK": getattr(cv2, "CALIB_HAND_EYE_PARK", None),
        "HORAUD": getattr(cv2, "CALIB_HAND_EYE_HORAUD", None),
        "ANDREFF": getattr(cv2, "CALIB_HAND_EYE_ANDREFF", None),
        "DANIILIDIS": getattr(cv2, "CALIB_HAND_EYE_DANIILIDIS", None),
    }
    key = str(method_name).strip().upper()
    if key not in lookup or lookup[key] is None:
        raise ValueError(f"Unsupported hand-eye method: {method_name}")
    return int(lookup[key])


class WristHandEyeCalibrator:
    def __init__(self) -> None:
        self.end_pose_topic = rospy.get_param("~end_pose_topic", "/robot/arm_left/end_pose")
        self.target_pose_topic = rospy.get_param("~target_pose_topic", "/handeye/left_cam_from_tag")
        self.parent_frame = rospy.get_param("~parent_frame", "arm_left/gripper_base")
        self.child_frame = rospy.get_param("~child_frame", "realsense_left_color_optical_frame")
        self.tag_family = rospy.get_param("~tag_family", "tag36h11")
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.tag_size_m = float(rospy.get_param("~tag_size_m", 0.10))
        self.min_samples = int(rospy.get_param("~min_samples", 30))
        self.method_name = rospy.get_param("~method", "PARK")
        self.method = resolve_handeye_method(self.method_name)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("cam_calibration")
        self.samples_dir = rospy.get_param("~samples_dir", os.path.join(pkg_path, "data", "samples", "left"))
        os.makedirs(self.samples_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.end_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.R_base2gripper_samples: List[np.ndarray] = []
        self.t_base2gripper_samples: List[np.ndarray] = []
        self.R_cam2target_samples: List[np.ndarray] = []
        self.t_cam2target_samples: List[np.ndarray] = []
        self.R_gripper2cam: Optional[np.ndarray] = None
        self.t_gripper2cam: Optional[np.ndarray] = None
        self.consistency_metrics: Optional[Dict[str, object]] = None
        self.calib_result: Optional[Dict[str, object]] = None

        rospy.Subscriber(self.end_pose_topic, PoseStamped, self._end_pose_cb, queue_size=1)
        rospy.Subscriber(self.target_pose_topic, PoseStamped, self._target_pose_cb, queue_size=1)

        rospy.Service("~capture_sample", Trigger, self._capture_sample_srv)
        rospy.Service("~clear_samples", Trigger, self._clear_samples_srv)
        rospy.Service("~compute_calibration", Trigger, self._compute_calibration_srv)
        rospy.Service("~save_result", Trigger, self._save_result_srv)
        rospy.Service("~status", Trigger, self._status_srv)

        rospy.loginfo("WristHandEyeCalibrator initialized")
        rospy.loginfo("  end_pose_topic: %s", self.end_pose_topic)
        rospy.loginfo("  target_pose_topic: %s", self.target_pose_topic)
        rospy.loginfo("  parent_frame: %s", self.parent_frame)
        rospy.loginfo("  child_frame: %s", self.child_frame)
        rospy.loginfo("  tag: family=%s id=%d size=%.4fm", self.tag_family, self.tag_id, self.tag_size_m)
        rospy.loginfo("  min_samples: %d", self.min_samples)
        rospy.loginfo("  method: %s", self.method_name)
        rospy.loginfo("  samples_dir: %s", self.samples_dir)

    def _end_pose_cb(self, msg: PoseStamped) -> None:
        with self.lock:
            self.end_pose = msg

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        with self.lock:
            self.target_pose = msg if pose_is_valid(msg) else None

    def _capture_sample_srv(self, _req) -> TriggerResponse:
        with self.lock:
            if self.end_pose is None:
                return TriggerResponse(success=False, message="No end pose received yet")
            if self.target_pose is None:
                return TriggerResponse(success=False, message="No valid AprilTag pose received yet")

            R_base2gripper, t_base2gripper = pose_to_rt(self.end_pose)
            R_cam2target, t_cam2target = pose_to_rt(self.target_pose)

            self.R_base2gripper_samples.append(R_base2gripper)
            self.t_base2gripper_samples.append(t_base2gripper)
            self.R_cam2target_samples.append(R_cam2target)
            self.t_cam2target_samples.append(t_cam2target)

            num_samples = len(self.R_base2gripper_samples)
            return TriggerResponse(
                success=True,
                message=f"Captured sample {num_samples}/{self.min_samples}",
            )

    def _clear_samples_srv(self, _req) -> TriggerResponse:
        with self.lock:
            self.R_base2gripper_samples.clear()
            self.t_base2gripper_samples.clear()
            self.R_cam2target_samples.clear()
            self.t_cam2target_samples.clear()
            self.R_gripper2cam = None
            self.t_gripper2cam = None
            self.consistency_metrics = None
            self.calib_result = None
        return TriggerResponse(success=True, message="All wrist hand-eye samples cleared")

    def _compute_calibration_srv(self, _req) -> TriggerResponse:
        with self.lock:
            num_samples = len(self.R_base2gripper_samples)
            if num_samples < self.min_samples:
                return TriggerResponse(
                    success=False,
                    message=f"Not enough samples. Current={num_samples}, required={self.min_samples}",
                )

            if not hasattr(cv2, "calibrateHandEye"):
                return TriggerResponse(success=False, message="OpenCV build does not provide calibrateHandEye")

            R_base2gripper_samples = [mat.copy() for mat in self.R_base2gripper_samples]
            t_base2gripper_samples = [vec.copy() for vec in self.t_base2gripper_samples]
            R_cam2target_samples = [mat.copy() for mat in self.R_cam2target_samples]
            t_cam2target_samples = [vec.copy() for vec in self.t_cam2target_samples]

        try:
            R_gripper2cam, t_gripper2cam = cv2.calibrateHandEye(
                R_base2gripper_samples,
                t_base2gripper_samples,
                R_cam2target_samples,
                t_cam2target_samples,
                method=self.method,
            )
        except Exception as exc:
            rospy.logerr("Hand-eye calibration failed: %s", exc)
            return TriggerResponse(success=False, message=f"Calibration failed: {exc}")

        R_gripper2cam = np.asarray(R_gripper2cam, dtype=np.float64).reshape(3, 3)
        t_gripper2cam = np.asarray(t_gripper2cam, dtype=np.float64).reshape(3, 1)

        try:
            consistency_metrics = compute_consistency_metrics(
                R_base2gripper_samples,
                t_base2gripper_samples,
                R_cam2target_samples,
                t_cam2target_samples,
                R_gripper2cam,
                t_gripper2cam,
            )
        except Exception as exc:
            rospy.logerr("Consistency metric computation failed: %s", exc)
            return TriggerResponse(success=False, message=f"Consistency metric failed: {exc}")

        quat = rotation_matrix_to_quaternion_xyzw(R_gripper2cam)
        euler = quaternion_xyzw_to_euler_xyz_deg(quat)
        translation = t_gripper2cam.reshape(3)

        with self.lock:
            self.R_gripper2cam = R_gripper2cam
            self.t_gripper2cam = t_gripper2cam
            self.consistency_metrics = consistency_metrics
            self.calib_result = transform_to_pose_dict(R_gripper2cam, t_gripper2cam, self.parent_frame)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Wrist hand-eye calibration completed")
        rospy.loginfo(
            "  translation[m]: [%.5f, %.5f, %.5f]",
            translation[0],
            translation[1],
            translation[2],
        )
        rospy.loginfo(
            "  quaternion[xyzw]: [%.5f, %.5f, %.5f, %.5f]",
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )
        rospy.loginfo(
            "  euler_xyz[deg]: [%.3f, %.3f, %.3f]",
            euler[0],
            euler[1],
            euler[2],
        )
        rospy.loginfo("  consistency all_pass: %s", consistency_metrics["all_pass"])
        rospy.loginfo("  translation_std_m: %s", consistency_metrics["base_tag_position_std_m"])
        rospy.loginfo(
            "  translation_residual_norm_median_m: %.6f",
            consistency_metrics["base_tag_position_residual_norm_median_m"],
        )
        rospy.loginfo(
            "  orientation_error_deg_median: %.6f",
            consistency_metrics["base_tag_orientation_error_deg_median"],
        )
        rospy.loginfo("=" * 60)

        return TriggerResponse(
            success=True,
            message=(
                f"Calibration completed with {num_samples} samples. "
                f"translation=[{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]"
            ),
        )

    def _build_state_dict(self) -> Dict[str, object]:
        if self.R_gripper2cam is None or self.t_gripper2cam is None:
            raise RuntimeError("No calibration result available")

        return {
            "solver": "cv2.calibrateHandEye",
            "method": str(self.method_name).upper(),
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "tag_family": self.tag_family,
            "tag_id": int(self.tag_id),
            "tag_size_m": float(self.tag_size_m),
            "num_samples": int(len(self.R_base2gripper_samples)),
            "R_gripper2cam": self.R_gripper2cam.tolist(),
            "t_gripper2cam": self.t_gripper2cam.reshape(3).tolist(),
            "R_base2gripper_samples": [mat.tolist() for mat in self.R_base2gripper_samples],
            "t_base2gripper_samples": [vec.reshape(3).tolist() for vec in self.t_base2gripper_samples],
            "R_cam2target_samples": [mat.tolist() for mat in self.R_cam2target_samples],
            "t_cam2target_samples": [vec.reshape(3).tolist() for vec in self.t_cam2target_samples],
            "consistency_metrics": self.consistency_metrics,
            "calib_result": self.calib_result,
        }

    def _resolve_save_path(self) -> str:
        save_path = str(rospy.get_param("~save_path", "")).strip()
        if not save_path:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            return os.path.join(self.samples_dir, f"handeye_axxb_{timestamp}.json")
        if not os.path.isabs(save_path):
            return os.path.join(self.samples_dir, save_path)
        return save_path

    def _save_result_srv(self, _req) -> TriggerResponse:
        with self.lock:
            try:
                data = self._build_state_dict()
            except Exception as exc:
                return TriggerResponse(success=False, message=str(exc))

        save_path = self._resolve_save_path()
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=True)
        except Exception as exc:
            rospy.logerr("Failed to save wrist hand-eye result: %s", exc)
            return TriggerResponse(success=False, message=f"Failed to save: {exc}")
        return TriggerResponse(success=True, message=f"Saved wrist hand-eye result to {save_path}")

    def _status_srv(self, _req) -> TriggerResponse:
        with self.lock:
            num_samples = int(len(self.R_base2gripper_samples))
            min_samples = int(self.min_samples)
            payload = {
                "num_samples": num_samples,
                "min_samples": min_samples,
                "remaining_samples": max(0, min_samples - num_samples),
                "progress_ratio": float(min(1.0, num_samples / float(max(1, min_samples)))),
                "can_compute": bool(num_samples >= min_samples),
                "has_result": bool(self.R_gripper2cam is not None and self.t_gripper2cam is not None),
                "has_end_pose": bool(self.end_pose is not None),
                "has_target_pose": bool(self.target_pose is not None),
                "method": str(self.method_name).upper(),
                "parent_frame": str(self.parent_frame),
                "child_frame": str(self.child_frame),
            }
        return TriggerResponse(success=True, message=json.dumps(payload, ensure_ascii=True))


def main() -> int:
    rospy.init_node("wrist_handeye_calibration", anonymous=False)
    try:
        WristHandEyeCalibrator()
        rospy.spin()
    except Exception as exc:
        rospy.logerr("Wrist hand-eye calibration node failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
