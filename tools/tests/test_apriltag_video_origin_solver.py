#!/usr/bin/env python3
"""Unit tests for AprilTag video origin camera solver math."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "workspaces" / "scripts" / "estimate_camera_origin_from_apriltag_video.py"

spec = importlib.util.spec_from_file_location("apriltag_origin_solver", str(SCRIPT_PATH))
if spec is None or spec.loader is None:
    raise RuntimeError(f"Failed to load script: {SCRIPT_PATH}")
module = importlib.util.module_from_spec(spec)
sys.modules["apriltag_origin_solver"] = module
spec.loader.exec_module(module)


class TransformChainTest(unittest.TestCase):
    def test_origin_camera_transform_matches_ground_truth(self):
        # Known transforms in origin frame.
        T_ot = module.compose_origin_tag_transform([0.0, 0.4, 0.0], yaw_deg=30.0)
        T_oc_true = module.make_transform(
            module.rotation_z_deg(-20.0),
            [0.22, -0.18, 0.93],
        )

        # Build a synthetic camera observation transform: ^cT_t = inv(^oT_c) * ^oT_t.
        T_ct = module.invert_transform(T_oc_true) @ T_ot

        # Recover camera transform and compare.
        T_oc_est = module.compute_origin_camera_transform(T_ot, T_ct)
        self.assertTrue(np.allclose(T_oc_est, T_oc_true, atol=1e-9))

    def test_invert_transform_round_trip(self):
        T = module.make_transform(module.rotation_z_deg(12.0), [0.1, -0.2, 0.3])
        T_inv = module.invert_transform(T)
        ident = T @ T_inv
        self.assertTrue(np.allclose(ident, np.eye(4), atol=1e-9))

    def test_rotation_quaternion_round_trip(self):
        R = module.rotation_z_deg(37.5)
        quat = module.rotation_matrix_to_quaternion_xyzw(R)
        R_back = module.quaternion_xyzw_to_rotation_matrix(quat)
        self.assertTrue(np.allclose(R, R_back, atol=1e-9))

    def test_average_quaternions_preserves_consistent_heading(self):
        quats = np.asarray(
            [
                module.rotation_matrix_to_quaternion_xyzw(module.rotation_z_deg(10.0)),
                module.rotation_matrix_to_quaternion_xyzw(module.rotation_z_deg(11.0)),
                module.rotation_matrix_to_quaternion_xyzw(module.rotation_z_deg(9.0)),
            ],
            dtype=np.float64,
        )
        quat_avg = module.average_quaternions_xyzw(quats)
        euler_xyz = module.quaternion_xyzw_to_euler_xyz_deg(quat_avg)
        self.assertAlmostEqual(10.0, float(euler_xyz[2]), places=1)


class OutlierFilterTest(unittest.TestCase):
    def test_reject_outliers_with_mad(self):
        inliers = np.asarray(
            [
                [0.1, 0.2, 0.3],
                [0.11, 0.21, 0.31],
                [0.09, 0.19, 0.29],
                [0.105, 0.205, 0.305],
                [0.102, 0.203, 0.298],
            ],
            dtype=np.float64,
        )
        outlier = np.asarray([[5.0, -3.0, 2.0]], dtype=np.float64)
        pts = np.vstack([inliers, outlier])
        mask = module.reject_outliers_mad_or_iqr(pts, threshold=3.5)
        self.assertEqual(pts.shape[0], mask.size)
        self.assertFalse(bool(mask[-1]))
        self.assertGreaterEqual(int(np.sum(mask)), 4)

    def test_origin_z_axis_constraint_summary(self):
        pts = np.asarray(
            [
                [0.02, -0.03, 0.40],
                [0.01, -0.02, 0.41],
                [0.03, -0.01, 0.39],
            ],
            dtype=np.float64,
        )
        summary = module.summarize_origin_z_axis_constraint(pts)
        self.assertEqual([0.0, 0.0, 0.4], summary["camera_position_on_origin_z_axis"])
        self.assertAlmostEqual(0.4, summary["camera_z_in_origin"], places=9)
        self.assertTrue(np.allclose([0.02, -0.02], summary["camera_xy_residual_median"], atol=1e-9))


if __name__ == "__main__":
    unittest.main()
