#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detect a single AprilTag and publish its pose in the camera frame."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image


APRILTAG_FAMILIES = ("tag16h5", "tag25h9", "tag36h10", "tag36h11")


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


def lookup_apriltag_dict_id(tag_family: str) -> int:
    family = str(tag_family).strip().lower()
    if family not in APRILTAG_FAMILIES:
        raise ValueError(f"Unsupported AprilTag family: {tag_family}")
    expected = f"DICT_APRILTAG_{family[3:]}"
    for attr in dir(cv2.aruco):
        if attr.lower() == expected.lower():
            return int(getattr(cv2.aruco, attr))
    raise RuntimeError(
        f"OpenCV AprilTag dictionary {expected} is unavailable. "
        "Install OpenCV with aruco AprilTag support."
    )


def build_apriltag_detector(tag_family: str):
    dict_id = lookup_apriltag_dict_id(tag_family)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, aruco_dict
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
        return (aruco_dict, params), aruco_dict
    raise RuntimeError("OpenCV aruco detector API is unavailable in this build.")


def detect_apriltag_markers(detector_obj, gray_image: np.ndarray):
    if hasattr(detector_obj, "detectMarkers"):
        return detector_obj.detectMarkers(gray_image)
    aruco_dict, params = detector_obj
    return cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=params)


def select_target_corners(corners, ids, tag_id: int) -> Optional[np.ndarray]:
    if ids is None or len(ids) == 0:
        return None

    ids_flat = np.asarray(ids, dtype=np.int64).reshape(-1)
    matches = np.where(ids_flat == int(tag_id))[0]
    if matches.size == 0:
        return None

    if matches.size == 1:
        return np.asarray(corners[int(matches[0])], dtype=np.float64).reshape(4, 2)

    best_idx = int(matches[0])
    best_area = -1.0
    for idx in matches:
        poly = np.asarray(corners[int(idx)], dtype=np.float32).reshape(-1, 2)
        area = float(cv2.contourArea(poly))
        if area > best_area:
            best_area = area
            best_idx = int(idx)
    return np.asarray(corners[best_idx], dtype=np.float64).reshape(4, 2)


def build_tag_object_points(tag_size_m: float) -> np.ndarray:
    half = float(tag_size_m) * 0.5
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_pose_from_corners(
    corner_4x2: np.ndarray,
    tag_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    obj_points = build_tag_object_points(tag_size_m)
    img_points = np.asarray(corner_4x2, dtype=np.float64).reshape(4, 2)
    pnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, flags=pnp_flag)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed for AprilTag pose estimation")
    rotation, _ = cv2.Rodrigues(rvec)
    return rotation, np.asarray(tvec, dtype=np.float64).reshape(3), rvec, tvec


def create_pose_stamped(frame_id: str, stamp: rospy.Time, rotation: np.ndarray, translation: np.ndarray) -> PoseStamped:
    quat = rotation_matrix_to_quaternion_xyzw(rotation)
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = str(frame_id)
    pose.pose.position.x = float(translation[0])
    pose.pose.position.y = float(translation[1])
    pose.pose.position.z = float(translation[2])
    pose.pose.orientation.x = float(quat[0])
    pose.pose.orientation.y = float(quat[1])
    pose.pose.orientation.z = float(quat[2])
    pose.pose.orientation.w = float(quat[3])
    return pose


def create_invalid_pose(frame_id: str, stamp: rospy.Time) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = str(frame_id)
    pose.pose.position.x = float("inf")
    pose.pose.position.y = float("inf")
    pose.pose.position.z = float("inf")
    pose.pose.orientation.x = float("inf")
    pose.pose.orientation.y = float("inf")
    pose.pose.orientation.z = float("inf")
    pose.pose.orientation.w = float("inf")
    return pose


class SingleAprilTagDetector:
    def __init__(self) -> None:
        self.image_topic = rospy.get_param("~image_topic", "/realsense_left/color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/realsense_left/color/camera_info")
        self.target_pose_topic = rospy.get_param("~target_pose_topic", "/handeye/left_cam_from_tag")
        self.tag_family = rospy.get_param("~tag_family", "tag36h11")
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.tag_size_m = float(rospy.get_param("~tag_size_m", 0.10))
        self.show_visualization = bool(rospy.get_param("~show_visualization", True))
        self.window_name = rospy.get_param("~window_name", "Single AprilTag Detection")

        if self.tag_size_m <= 0.0:
            raise ValueError("~tag_size_m must be > 0")

        self.detector_obj, _aruco_dict = build_apriltag_detector(self.tag_family)
        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame_id: str = ""
        self.last_pose: Optional[PoseStamped] = None
        self.last_vis_image: Optional[np.ndarray] = None

        if self.show_visualization:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.pose_pub = rospy.Publisher(self.target_pose_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        rospy.loginfo(
            "SingleAprilTagDetector initialized: family=%s id=%d size=%.4fm image=%s",
            self.tag_family,
            self.tag_id,
            self.tag_size_m,
            self.image_topic,
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.D, dtype=np.float64).reshape(-1)
        self.camera_frame_id = str(msg.header.frame_id)
        rospy.loginfo("Camera intrinsics received from %s", self.camera_frame_id)

    def _image_cb(self, msg: Image) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "CvBridge error: %s", exc)
            return

        pose, vis = self._detect_pose(image, msg.header.stamp)
        self.pose_pub.publish(pose)
        if not math.isinf(pose.pose.position.x):
            self.last_pose = pose
        self.last_vis_image = vis

    def _detect_pose(self, image: np.ndarray, stamp: rospy.Time) -> Tuple[PoseStamped, np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = detect_apriltag_markers(self.detector_obj, gray)
        target_corners = select_target_corners(corners, ids, self.tag_id)

        vis = image.copy()
        if corners is not None and ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        if target_corners is None:
            self._draw_status(vis, detected=False)
            return create_invalid_pose(self.camera_frame_id, stamp), vis

        try:
            rotation, translation, rvec, tvec = estimate_pose_from_corners(
                target_corners,
                self.tag_size_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "AprilTag pose estimation failed: %s", exc)
            self._draw_status(vis, detected=False)
            return create_invalid_pose(self.camera_frame_id, stamp), vis

        cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.tag_size_m * 0.75)
        pose = create_pose_stamped(self.camera_frame_id, stamp, rotation, translation)
        self._draw_status(vis, detected=True, translation=translation)
        return pose, vis

    def _draw_status(
        self,
        image: np.ndarray,
        detected: bool,
        translation: Optional[np.ndarray] = None,
    ) -> None:
        if detected:
            status = f"Tag {self.tag_id} detected"
            color = (0, 255, 0)
        else:
            status = f"Searching tag {self.tag_id}"
            color = (0, 0, 255)
        cv2.putText(image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        if translation is not None:
            cv2.putText(
                image,
                f"xyz[m]: [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}]",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

    def spin(self) -> None:
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.show_visualization and self.last_vis_image is not None:
                cv2.imshow(self.window_name, self.last_vis_image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            rate.sleep()
        if self.show_visualization:
            cv2.destroyWindow(self.window_name)


def main() -> int:
    rospy.init_node("detect_apriltag_target", anonymous=False)
    try:
        node = SingleAprilTagDetector()
        node.spin()
    except Exception as exc:
        rospy.logerr("Single AprilTag detector failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
