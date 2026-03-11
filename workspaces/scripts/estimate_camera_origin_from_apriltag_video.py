#!/usr/bin/env python3
"""Estimate overhead camera position in origin frame from AprilTag video.

This script computes camera pose in an origin frame from:
1) tag pose detected in camera frame for each video frame (^{c}T_{t})
2) known tag pose in origin frame (^{o}T_{t})

The per-frame relation is:
    ^{o}T_{c} = ^{o}T_{t} * inv(^{c}T_{t})

Final camera position is reported as the per-axis median after robust outlier
rejection over all valid frames.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


APRILTAG_FAMILIES = ("tag16h5", "tag25h9", "tag36h10", "tag36h11")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video-path", required=True, help="Input video path.")
    parser.add_argument("--tag-id", type=int, required=True, help="Target AprilTag ID to track.")
    parser.add_argument("--tag-size-m", type=float, required=True, help="Tag edge length in meters.")
    parser.add_argument(
        "--tag-family",
        default="tag36h11",
        choices=APRILTAG_FAMILIES,
        help="AprilTag family (default: tag36h11).",
    )
    parser.add_argument(
        "--camera-info-yaml",
        default="",
        help="ROS camera calibration YAML path (optional).",
    )
    parser.add_argument(
        "--k",
        default="",
        help='Camera intrinsic K as 9 csv floats, e.g. "fx,0,cx,0,fy,cy,0,0,1".',
    )
    parser.add_argument(
        "--d",
        default="",
        help='Distortion coeffs csv, e.g. "k1,k2,p1,p2,k3". Defaults to zero distortion if omitted.',
    )
    parser.add_argument(
        "--origin-tag-position",
        default="0,0.4,0",
        help='Tag position in origin frame, csv "x,y,z" meters (default: 0,0.4,0).',
    )
    parser.add_argument(
        "--origin-tag-yaw-deg",
        type=float,
        default=0.0,
        help="Tag yaw in origin frame in degrees (default: 0).",
    )
    parser.add_argument(
        "--min-valid-frames",
        type=int,
        default=30,
        help="Minimum valid detection frames required before output (default: 30).",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Process at most this many frames (0 means all).",
    )
    parser.add_argument(
        "--outlier-threshold",
        type=float,
        default=3.5,
        help="Modified z-score threshold for MAD outlier filtering (default: 3.5).",
    )
    parser.add_argument(
        "--output-json",
        default="",
        help="Output JSON path (default: <video_stem>_camera_origin_result.json).",
    )
    return parser.parse_args()


def import_runtime_deps():
    try:
        import cv2  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "OpenCV is required. Install python3-opencv/opencv-contrib and retry."
        ) from exc
    return cv2


def parse_csv_floats(raw: str, expected_len: Optional[int], name: str) -> np.ndarray:
    text = str(raw).strip()
    if not text:
        if expected_len is None:
            return np.asarray([], dtype=np.float64)
        raise ValueError(f"{name} is empty")
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if expected_len is not None and len(vals) != expected_len:
        raise ValueError(f"{name} expects {expected_len} floats, got {len(vals)}")
    return np.asarray(vals, dtype=np.float64)


def _extract_calib_array(obj: Any, key: str) -> Optional[np.ndarray]:
    val = obj.get(key)
    if val is None:
        return None
    if isinstance(val, dict) and "data" in val:
        val = val["data"]
    if isinstance(val, (list, tuple)):
        try:
            arr = np.asarray([float(x) for x in val], dtype=np.float64)
        except Exception:
            return None
        return arr
    return None


def load_camera_info_yaml(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    try:
        import yaml  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("PyYAML is required to load --camera-info-yaml.") from exc

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid YAML root in {path}")

    k_arr = _extract_calib_array(data, "K")
    if k_arr is None:
        k_arr = _extract_calib_array(data, "camera_matrix")
    if k_arr is None or k_arr.size != 9:
        raise RuntimeError(f"Failed to parse 3x3 K from {path}")

    d_arr = _extract_calib_array(data, "D")
    if d_arr is None:
        d_arr = _extract_calib_array(data, "distortion_coefficients")
    if d_arr is None:
        d_arr = np.zeros(5, dtype=np.float64)

    return k_arr.reshape(3, 3), d_arr.reshape(-1)


def resolve_intrinsics(args: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray]:
    k: Optional[np.ndarray] = None
    d: Optional[np.ndarray] = None

    if args.camera_info_yaml:
        k, d = load_camera_info_yaml(Path(args.camera_info_yaml).expanduser().resolve())

    if str(args.k).strip():
        k = parse_csv_floats(args.k, expected_len=9, name="--k").reshape(3, 3)
    if str(args.d).strip():
        d = parse_csv_floats(args.d, expected_len=None, name="--d").reshape(-1)

    if k is None:
        raise RuntimeError("Missing intrinsics K. Provide --k or --camera-info-yaml.")
    if d is None or d.size == 0:
        d = np.zeros(5, dtype=np.float64)

    return k.astype(np.float64), d.astype(np.float64)


def rotation_z_deg(yaw_deg: float) -> np.ndarray:
    yaw = math.radians(float(yaw_deg))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.asarray(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def make_transform(rotation: np.ndarray, translation_xyz: Sequence[float]) -> np.ndarray:
    t = np.asarray(translation_xyz, dtype=np.float64).reshape(3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = t
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def compose_origin_tag_transform(position_xyz: Sequence[float], yaw_deg: float) -> np.ndarray:
    # Assumption: tag lies flat on table, with normal aligned +Z in origin frame.
    R_ot = rotation_z_deg(yaw_deg)
    return make_transform(R_ot, position_xyz)


def compute_origin_camera_transform(T_ot: np.ndarray, T_ct: np.ndarray) -> np.ndarray:
    return np.asarray(T_ot, dtype=np.float64).reshape(4, 4) @ invert_transform(T_ct)


def normalize_quaternion_xyzw(quat_xyzw: Sequence[float]) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is zero")
    return quat / norm


def rotation_matrix_to_quaternion_xyzw(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
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


def quaternion_angular_error_deg(ref_xyzw: Sequence[float], sample_xyzw: Sequence[float]) -> float:
    delta = quaternion_multiply_xyzw(quaternion_conjugate_xyzw(ref_xyzw), sample_xyzw)
    w = max(-1.0, min(1.0, abs(float(delta[3]))))
    return math.degrees(2.0 * math.acos(w))


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


def _lookup_apriltag_dict_id(cv2, tag_family: str) -> int:
    expected = f"DICT_APRILTAG_{tag_family[3:]}"  # tag36h11 -> DICT_APRILTAG_36h11
    for attr in dir(cv2.aruco):
        if attr.lower() == expected.lower():
            return int(getattr(cv2.aruco, attr))
    raise RuntimeError(
        f"OpenCV AprilTag dictionary {expected} is unavailable. "
        "Install opencv-contrib-python with aruco AprilTag support."
    )


def build_apriltag_detector(cv2, tag_family: str):
    dict_id = _lookup_apriltag_dict_id(cv2, tag_family)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, aruco_dict
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
        return (aruco_dict, params), aruco_dict
    raise RuntimeError("OpenCV aruco detector API is unavailable in this build.")


def detect_target_corners(detector_obj, frame_bgr: np.ndarray, target_tag_id: int, cv2):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    if hasattr(detector_obj, "detectMarkers"):
        corners, ids, _ = detector_obj.detectMarkers(gray)
    else:
        aruco_dict, params = detector_obj
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or len(ids) == 0:
        return None
    ids_flat = ids.reshape(-1)
    matches = np.where(ids_flat == int(target_tag_id))[0]
    if matches.size == 0:
        return None

    if matches.size == 1:
        return corners[int(matches[0])]

    # If duplicated IDs appear, keep the largest marker area candidate.
    best_idx = int(matches[0])
    best_area = -1.0
    for idx in matches:
        poly = np.asarray(corners[int(idx)]).reshape(-1, 2).astype(np.float32)
        area = float(cv2.contourArea(poly))
        if area > best_area:
            best_area = area
            best_idx = int(idx)
    return corners[best_idx]


def estimate_ct_from_corners(corner_4x2: np.ndarray, tag_size_m: float, K: np.ndarray, D: np.ndarray, cv2):
    corners = np.asarray(corner_4x2, dtype=np.float64).reshape(1, 4, 2)

    rvec = None
    tvec = None
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, float(tag_size_m), K, D)
        if rvecs is not None and tvecs is not None and len(rvecs) > 0:
            rvec = np.asarray(rvecs[0], dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvecs[0], dtype=np.float64).reshape(3, 1)

    if rvec is None or tvec is None:
        half = float(tag_size_m) * 0.5
        obj_pts = np.asarray(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        img_pts = corners.reshape(4, 2).astype(np.float64)
        pnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=pnp_flag)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None

    R_ct, _ = cv2.Rodrigues(rvec)
    return make_transform(R_ct, np.asarray(tvec, dtype=np.float64).reshape(3))


def reject_outliers_mad_or_iqr(points_xyz: np.ndarray, threshold: float = 3.5) -> np.ndarray:
    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    n = pts.shape[0]
    if n < 4:
        return np.ones(n, dtype=bool)

    med = np.median(pts, axis=0)
    dists = np.linalg.norm(pts - med, axis=1)

    med_d = float(np.median(dists))
    mad = float(np.median(np.abs(dists - med_d)))
    if mad > 1e-12:
        mod_z = 0.6745 * (dists - med_d) / mad
        mask = np.abs(mod_z) <= float(threshold)
    else:
        q1, q3 = np.percentile(dists, [25.0, 75.0])
        iqr = float(q3 - q1)
        if iqr <= 1e-12:
            mask = np.ones(n, dtype=bool)
        else:
            lo = q1 - 1.5 * iqr
            hi = q3 + 1.5 * iqr
            mask = (dists >= lo) & (dists <= hi)

    if not np.any(mask):
        return np.ones(n, dtype=bool)
    return mask


def summarize_origin_z_axis_constraint(points_xyz: np.ndarray) -> Dict[str, Any]:
    pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        raise ValueError("No positions to summarize")

    xy = pts[:, :2]
    z = pts[:, 2]
    xy_norm = np.linalg.norm(xy, axis=1)

    z_median = float(np.median(z))
    z_std = float(np.std(z, ddof=0))
    xy_residual_median = np.median(xy, axis=0)
    xy_residual_std = np.std(xy, axis=0, ddof=0)

    return {
        "camera_position_on_origin_z_axis": [0.0, 0.0, z_median],
        "camera_z_in_origin": z_median,
        "camera_z_std": z_std,
        "camera_xy_residual_median": [float(x) for x in xy_residual_median],
        "camera_xy_residual_std": [float(x) for x in xy_residual_std],
        "camera_xy_residual_norm_median": float(np.median(xy_norm)),
        "camera_xy_residual_norm_max": float(np.max(xy_norm)),
    }


def default_output_json_path(video_path: Path) -> Path:
    return video_path.with_name(f"{video_path.stem}_camera_origin_result.json")


def run(args: argparse.Namespace) -> int:
    if args.tag_size_m <= 0.0:
        raise RuntimeError("--tag-size-m must be > 0")
    if args.min_valid_frames <= 0:
        raise RuntimeError("--min-valid-frames must be > 0")

    cv2 = import_runtime_deps()
    K, D = resolve_intrinsics(args)
    tag_position = parse_csv_floats(args.origin_tag_position, expected_len=3, name="--origin-tag-position")
    T_ot = compose_origin_tag_transform(tag_position, args.origin_tag_yaw_deg)

    video_path = Path(args.video_path).expanduser().resolve()
    if not video_path.is_file():
        raise RuntimeError(f"Video file not found: {video_path}")

    detector_obj, _aruco_dict = build_apriltag_detector(cv2, args.tag_family)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")

    total_frames = 0
    valid_frames = 0
    positions: list[np.ndarray] = []
    quaternions_xyzw: list[np.ndarray] = []

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            total_frames += 1
            if args.max_frames > 0 and total_frames > args.max_frames:
                break

            target_corners = detect_target_corners(detector_obj, frame, args.tag_id, cv2)
            if target_corners is None:
                continue

            T_ct = estimate_ct_from_corners(target_corners, args.tag_size_m, K, D, cv2)
            if T_ct is None:
                continue

            T_oc = compute_origin_camera_transform(T_ot, T_ct)
            positions.append(T_oc[:3, 3].copy())
            quaternions_xyzw.append(rotation_matrix_to_quaternion_xyzw(T_oc[:3, :3]))
            valid_frames += 1
    finally:
        cap.release()

    if valid_frames < args.min_valid_frames:
        raise RuntimeError(
            f"Not enough valid frames for tag {args.tag_id}. "
            f"valid={valid_frames}, required={args.min_valid_frames}, total={total_frames}."
        )

    pos_arr = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    quat_arr = np.asarray(quaternions_xyzw, dtype=np.float64).reshape(-1, 4)
    keep_mask = reject_outliers_mad_or_iqr(pos_arr, threshold=args.outlier_threshold)
    kept_arr = pos_arr[keep_mask]
    kept_quat_arr = quat_arr[keep_mask]
    if kept_arr.shape[0] < args.min_valid_frames:
        raise RuntimeError(
            f"Not enough frames after outlier rejection: kept={kept_arr.shape[0]}, "
            f"required={args.min_valid_frames}."
        )

    pos_median = np.median(kept_arr, axis=0)
    pos_std = np.std(kept_arr, axis=0, ddof=0)
    z_axis_summary = summarize_origin_z_axis_constraint(kept_arr)
    quat_mean = average_quaternions_xyzw(kept_quat_arr)
    rot_mean = quaternion_xyzw_to_rotation_matrix(quat_mean)
    euler_xyz_deg = quaternion_xyzw_to_euler_xyz_deg(quat_mean)
    ang_err_deg = np.asarray(
        [quaternion_angular_error_deg(quat_mean, q) for q in kept_quat_arr],
        dtype=np.float64,
    )
    ang_err_std_deg = float(np.std(ang_err_deg, ddof=0))
    ang_err_max_deg = float(np.max(ang_err_deg)) if ang_err_deg.size > 0 else 0.0

    output_json = (
        Path(args.output_json).expanduser().resolve()
        if str(args.output_json).strip()
        else default_output_json_path(video_path)
    )

    result: Dict[str, Any] = {
        "video_path": str(video_path),
        "tag_family": str(args.tag_family),
        "used_tag_id": int(args.tag_id),
        "tag_size_m": float(args.tag_size_m),
        "camera_position_median": [float(x) for x in pos_median],
        "camera_position_std": [float(x) for x in pos_std],
        "camera_orientation_quaternion_xyzw": [float(x) for x in quat_mean],
        "camera_orientation_euler_xyz_deg": [float(x) for x in euler_xyz_deg],
        "camera_rotation_matrix": [float(x) for x in rot_mean.reshape(-1)],
        "camera_orientation_std_deg": ang_err_std_deg,
        "camera_orientation_max_error_deg": ang_err_max_deg,
        "camera_position_on_origin_z_axis": list(z_axis_summary["camera_position_on_origin_z_axis"]),
        "camera_z_in_origin": float(z_axis_summary["camera_z_in_origin"]),
        "camera_z_std": float(z_axis_summary["camera_z_std"]),
        "camera_xy_residual_median": list(z_axis_summary["camera_xy_residual_median"]),
        "camera_xy_residual_std": list(z_axis_summary["camera_xy_residual_std"]),
        "camera_xy_residual_norm_median": float(z_axis_summary["camera_xy_residual_norm_median"]),
        "camera_xy_residual_norm_max": float(z_axis_summary["camera_xy_residual_norm_max"]),
        "num_valid_frames": int(valid_frames),
        "num_total_frames": int(total_frames),
        "num_used_after_outlier_rejection": int(kept_arr.shape[0]),
        "outlier_filter": {
            "method": "mad_or_iqr",
            "threshold": float(args.outlier_threshold),
        },
        "assumed_origin_tag_pose": {
            "position_m": [float(x) for x in tag_position],
            "yaw_deg": float(args.origin_tag_yaw_deg),
            "assumptions": {
                "flat_on_table": True,
                "aligned_xy_axes": True,
                "normal_direction": "+Z in origin frame",
            },
        },
        "intrinsics": {
            "K": [float(x) for x in K.reshape(-1)],
            "D": [float(x) for x in D.reshape(-1)],
        },
    }

    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(result, indent=2, ensure_ascii=True), encoding="utf-8")

    print(
        "camera_position_in_origin (median, m): "
        f"[{pos_median[0]:.6f}, {pos_median[1]:.6f}, {pos_median[2]:.6f}]"
    )
    print(
        "camera_position_on_origin_z_axis (m): "
        f"[0.000000, 0.000000, {z_axis_summary['camera_z_in_origin']:.6f}]"
    )
    print(
        "camera_xy_residual_median (m): "
        f"[{z_axis_summary['camera_xy_residual_median'][0]:.6f}, "
        f"{z_axis_summary['camera_xy_residual_median'][1]:.6f}]"
    )
    print(f"camera_xy_residual_norm_median (m): {z_axis_summary['camera_xy_residual_norm_median']:.6f}")
    print(
        "camera_orientation_in_origin (xyzw): "
        f"[{quat_mean[0]:.6f}, {quat_mean[1]:.6f}, {quat_mean[2]:.6f}, {quat_mean[3]:.6f}]"
    )
    print(
        "camera_orientation_in_origin (xyz deg): "
        f"[{euler_xyz_deg[0]:.3f}, {euler_xyz_deg[1]:.3f}, {euler_xyz_deg[2]:.3f}]"
    )
    print(
        f"valid_frames={valid_frames}, total_frames={total_frames}, "
        f"used_after_outlier_rejection={kept_arr.shape[0]}"
    )
    print(f"result_json={output_json}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
