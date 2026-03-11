#!/usr/bin/env python3
"""Tests for the wrist hand-eye calibration pipeline."""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CAM_CALIB_DIR = (
    REPO_ROOT
    / "src"
    / "zeno-wholebody-intervation"
    / "zeno-wholebody-robot"
    / "robot_side"
    / "cam_calibration"
    / "scripts"
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


wrist_module = _load_module(
    "wrist_handeye_calibration",
    CAM_CALIB_DIR / "wrist_handeye_calibration.py",
)
detector_module = _load_module(
    "detect_apriltag_target",
    CAM_CALIB_DIR / "detect_apriltag_target.py",
)
tf_publisher_module = _load_module(
    "tf_publisher",
    CAM_CALIB_DIR / "tf_publisher.py",
)

np = wrist_module.np
cv2 = wrist_module.cv2


def make_transform(rotation: np.ndarray, translation_xyz) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(translation_xyz, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rotation_xyz_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    ax = math.radians(rx)
    ay = math.radians(ry)
    az = math.radians(rz)
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)

    Rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    Ry = np.asarray([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    Rz = np.asarray([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return Rz @ Ry @ Rx


@unittest.skipUnless(hasattr(cv2, "calibrateHandEye"), "OpenCV hand-eye API unavailable")
class WristHandEyeCalibrationMathTest(unittest.TestCase):
    def test_calibrate_handeye_recovers_known_transform(self):
        T_gripper2cam_true = make_transform(rotation_xyz_deg(8.0, -18.0, 22.0), [0.04, -0.03, 0.08])
        T_base2tag_true = make_transform(rotation_xyz_deg(12.0, -5.0, 30.0), [0.55, 0.10, 0.18])

        sample_transforms = [
            make_transform(rotation_xyz_deg(0.0, -15.0, 10.0), [0.30, -0.10, 0.25]),
            make_transform(rotation_xyz_deg(20.0, 5.0, -18.0), [0.28, 0.06, 0.22]),
            make_transform(rotation_xyz_deg(-12.0, 10.0, 28.0), [0.36, -0.04, 0.30]),
            make_transform(rotation_xyz_deg(15.0, -22.0, 40.0), [0.41, 0.01, 0.24]),
            make_transform(rotation_xyz_deg(-18.0, -8.0, -35.0), [0.25, 0.11, 0.28]),
            make_transform(rotation_xyz_deg(10.0, 18.0, 12.0), [0.34, -0.09, 0.21]),
        ]

        R_base2gripper = []
        t_base2gripper = []
        R_cam2tag = []
        t_cam2tag = []
        for T_base2gripper_i in sample_transforms:
            T_cam2tag_i = invert_transform(T_gripper2cam_true) @ invert_transform(T_base2gripper_i) @ T_base2tag_true
            R_base2gripper.append(T_base2gripper_i[:3, :3])
            t_base2gripper.append(T_base2gripper_i[:3, 3].reshape(3, 1))
            R_cam2tag.append(T_cam2tag_i[:3, :3])
            t_cam2tag.append(T_cam2tag_i[:3, 3].reshape(3, 1))

        R_gripper2cam_est, t_gripper2cam_est = cv2.calibrateHandEye(
            R_base2gripper,
            t_base2gripper,
            R_cam2tag,
            t_cam2tag,
            method=cv2.CALIB_HAND_EYE_PARK,
        )

        T_gripper2cam_est = make_transform(R_gripper2cam_est, np.asarray(t_gripper2cam_est).reshape(3))
        self.assertTrue(np.allclose(T_gripper2cam_est, T_gripper2cam_true, atol=1e-5))

        metrics = wrist_module.compute_consistency_metrics(
            R_base2gripper,
            t_base2gripper,
            R_cam2tag,
            t_cam2tag,
            R_gripper2cam_est,
            t_gripper2cam_est,
        )
        self.assertTrue(metrics["all_pass"])
        self.assertLess(metrics["base_tag_position_residual_norm_median_m"], 1e-6)
        self.assertLess(metrics["base_tag_orientation_error_deg_median"], 1e-6)


@unittest.skipUnless(hasattr(cv2, "projectPoints"), "OpenCV projectPoints unavailable")
class AprilTagPoseEstimationTest(unittest.TestCase):
    def test_single_tag_pose_estimation_matches_projected_corners(self):
        camera_matrix = np.asarray(
            [
                [435.144287109375, 0.0, 314.1404113769531],
                [0.0, 433.9437561035156, 241.35781860351562],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.asarray(
            [
                -0.05208737030625343,
                0.05731990560889244,
                -0.0002043730637524277,
                0.00017627282068133354,
                -0.018378810957074165,
            ],
            dtype=np.float64,
        )
        tag_size_m = 0.10
        object_points = detector_module.build_tag_object_points(tag_size_m)
        R_true = rotation_xyz_deg(14.0, -9.0, 6.0)
        rvec_true, _ = cv2.Rodrigues(R_true)
        tvec_true = np.asarray([[0.03], [-0.02], [0.42]], dtype=np.float64)

        corners, _ = cv2.projectPoints(object_points, rvec_true, tvec_true, camera_matrix, dist_coeffs)
        R_est, t_est, _rvec_est, _tvec_est = detector_module.estimate_pose_from_corners(
            corners.reshape(4, 2),
            tag_size_m,
            camera_matrix,
            dist_coeffs,
        )

        self.assertTrue(np.allclose(R_est, R_true, atol=1e-5))
        self.assertTrue(np.allclose(t_est.reshape(3, 1), tvec_true, atol=1e-5))


class TfPublisherCompatibilityTest(unittest.TestCase):
    def test_prefers_new_wrist_keys(self):
        R_key, t_key = tf_publisher_module.get_wrist_transform_keys(
            {
                "R_gripper2cam": [[1.0, 0.0, 0.0]] * 3,
                "t_gripper2cam": [0.0, 0.0, 0.0],
                "R_gripper2left": [[1.0, 0.0, 0.0]] * 3,
                "t_gripper2left": [1.0, 2.0, 3.0],
            }
        )
        self.assertEqual(("R_gripper2cam", "t_gripper2cam"), (R_key, t_key))

    def test_supports_legacy_left_keys(self):
        R_key, t_key = tf_publisher_module.get_wrist_transform_keys(
            {
                "R_gripper2left": [[1.0, 0.0, 0.0]] * 3,
                "t_gripper2left": [0.0, 0.0, 0.0],
            }
        )
        self.assertEqual(("R_gripper2left", "t_gripper2left"), (R_key, t_key))


if __name__ == "__main__":
    unittest.main()
