#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV GUI for wrist hand-eye calibration workflow."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


@dataclass
class UIButton:
    label: str
    hotkey: str
    action: Callable[[], None]
    rect: Tuple[int, int, int, int] = (0, 0, 0, 0)


def pose_is_finite(msg: PoseStamped) -> bool:
    values = (
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    )
    return all(math.isfinite(float(v)) for v in values)


def side_defaults(side: str) -> Dict[str, str]:
    normalized = str(side).strip().lower()
    if normalized not in ("left", "right"):
        raise ValueError("~side must be 'left' or 'right'")
    if normalized == "left":
        return {
            "image_topic": "/realsense_left/color/image_raw",
            "target_pose_topic": "/handeye/left_cam_from_tag",
            "calibrator_ns": "/left_wrist_handeye_calibration",
        }
    return {
        "image_topic": "/realsense_right/color/image_raw",
        "target_pose_topic": "/handeye/right_cam_from_tag",
        "calibrator_ns": "/right_wrist_handeye_calibration",
    }


class WristHandEyeGUI:
    def __init__(self) -> None:
        side = rospy.get_param("~side", "left")
        defaults = side_defaults(side)

        self.side = str(side).strip().lower()
        self.image_topic = rospy.get_param("~image_topic", defaults["image_topic"])
        self.target_pose_topic = rospy.get_param("~target_pose_topic", defaults["target_pose_topic"])
        self.calibrator_ns = rospy.get_param("~calibrator_ns", defaults["calibrator_ns"])
        self.window_name = rospy.get_param("~window_name", f"Wrist HandEye GUI ({self.side})")
        self.min_samples = int(rospy.get_param("~min_samples", 30))
        self.preview_scale = float(rospy.get_param("~preview_scale", 1.0))
        self.service_timeout = float(rospy.get_param("~service_timeout_sec", 1.0))
        self.status_poll_hz = float(rospy.get_param("~status_poll_hz", 2.0))

        if self.preview_scale <= 0.0:
            raise ValueError("~preview_scale must be > 0")
        if self.min_samples <= 0:
            raise ValueError("~min_samples must be > 0")
        if self.status_poll_hz <= 0.0:
            raise ValueError("~status_poll_hz must be > 0")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.tag_valid = False
        self.last_pose_xyz: Optional[np.ndarray] = None
        self.sample_count = 0
        self.remote_sample_count: Optional[int] = None
        self.remote_remaining_samples: Optional[int] = None
        self.remote_progress_ratio = 0.0
        self.remote_can_compute = False
        self.remote_has_result = False
        self.remote_has_end_pose = False
        self.remote_has_target_pose = False
        self.remote_status_error = ""
        self.last_status_time = 0.0
        self.next_status_poll_time = 0.0
        self.last_message = "Ready"
        self.last_message_is_error = False
        self.last_message_time = time.time()

        self.capture_srv = rospy.ServiceProxy(f"{self.calibrator_ns}/capture_sample", Trigger)
        self.clear_srv = rospy.ServiceProxy(f"{self.calibrator_ns}/clear_samples", Trigger)
        self.compute_srv = rospy.ServiceProxy(f"{self.calibrator_ns}/compute_calibration", Trigger)
        self.save_srv = rospy.ServiceProxy(f"{self.calibrator_ns}/save_result", Trigger)
        self.status_srv = rospy.ServiceProxy(f"{self.calibrator_ns}/status", Trigger)

        self.buttons = [
            UIButton("Capture", "C / SPACE", self.capture_sample),
            UIButton("Clear", "R", self.clear_samples),
            UIButton("Compute", "K", self.compute_calibration),
            UIButton("Save", "S", self.save_result),
            UIButton("Compute+Save", "A", self.compute_and_save),
            UIButton("Quit", "Q", self.request_shutdown),
        ]
        self.shutdown_requested = False

        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        rospy.Subscriber(self.target_pose_topic, PoseStamped, self._target_pose_cb, queue_size=1)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            self.window_name,
            int(640 * self.preview_scale),
            int((480 + 210) * self.preview_scale),
        )
        cv2.setMouseCallback(self.window_name, self._on_mouse)

        self._set_message(
            f"GUI started. side={self.side}, calibrator={self.calibrator_ns}",
            is_error=False,
        )
        rospy.loginfo("WristHandEyeGUI started: side=%s, image=%s", self.side, self.image_topic)

    def _refresh_status(self, force: bool = False) -> None:
        now = time.time()
        if not force and now < self.next_status_poll_time:
            return
        self.next_status_poll_time = now + (1.0 / self.status_poll_hz)

        try:
            self.status_srv.wait_for_service(timeout=min(self.service_timeout, 0.1))
            resp = self.status_srv()
        except Exception as exc:
            err = f"status unavailable: {exc}"
            with self.lock:
                self.remote_status_error = err
            rospy.logwarn_throttle(5.0, "WristHandEyeGUI status query failed: %s", exc)
            return

        if not bool(resp.success):
            err = f"status failed: {resp.message}"
            with self.lock:
                self.remote_status_error = err
            rospy.logwarn_throttle(5.0, "WristHandEyeGUI status response failed: %s", resp.message)
            return

        try:
            payload = json.loads(resp.message) if resp.message else {}
            sample_count = int(payload.get("num_samples", 0))
            remaining = int(payload.get("remaining_samples", max(0, self.min_samples - sample_count)))
            progress = float(payload.get("progress_ratio", sample_count / float(max(1, self.min_samples))))
            progress = max(0.0, min(1.0, progress))
        except Exception as exc:
            err = f"status decode failed: {exc}"
            with self.lock:
                self.remote_status_error = err
            rospy.logwarn_throttle(5.0, "WristHandEyeGUI status decode failed: %s", exc)
            return

        with self.lock:
            self.remote_sample_count = sample_count
            self.remote_remaining_samples = max(0, remaining)
            self.remote_progress_ratio = progress
            self.remote_can_compute = bool(payload.get("can_compute", sample_count >= self.min_samples))
            self.remote_has_result = bool(payload.get("has_result", False))
            self.remote_has_end_pose = bool(payload.get("has_end_pose", False))
            self.remote_has_target_pose = bool(payload.get("has_target_pose", False))
            self.remote_status_error = ""
            self.last_status_time = time.time()

    def _set_message(self, text: str, is_error: bool) -> None:
        with self.lock:
            self.last_message = str(text)
            self.last_message_is_error = bool(is_error)
            self.last_message_time = time.time()
        if is_error:
            rospy.logwarn(text)
        else:
            rospy.loginfo(text)

    def _image_cb(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self._set_message(f"CvBridge error: {exc}", is_error=True)
            return
        with self.lock:
            self.latest_image = img

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        valid = pose_is_finite(msg)
        xyz = None
        if valid:
            xyz = np.asarray(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                dtype=np.float64,
            )
        with self.lock:
            self.tag_valid = valid
            self.last_pose_xyz = xyz

    def _call_trigger(self, proxy: rospy.ServiceProxy, name: str) -> bool:
        try:
            proxy.wait_for_service(timeout=self.service_timeout)
        except Exception as exc:
            self._set_message(f"{name}: service unavailable ({exc})", is_error=True)
            return False

        try:
            resp = proxy()
        except Exception as exc:
            self._set_message(f"{name}: call failed ({exc})", is_error=True)
            return False

        text = f"{name}: {resp.message}"
        self._set_message(text, is_error=not bool(resp.success))
        return bool(resp.success)

    def capture_sample(self) -> None:
        with self.lock:
            tag_valid = self.tag_valid
        if not tag_valid:
            self._set_message("Capture blocked: AprilTag not valid", is_error=True)
            return
        ok = self._call_trigger(self.capture_srv, "capture_sample")
        if ok:
            with self.lock:
                self.sample_count += 1
            self._refresh_status(force=True)

    def clear_samples(self) -> None:
        ok = self._call_trigger(self.clear_srv, "clear_samples")
        if ok:
            with self.lock:
                self.sample_count = 0
            self._refresh_status(force=True)

    def compute_calibration(self) -> None:
        self._call_trigger(self.compute_srv, "compute_calibration")

    def save_result(self) -> None:
        self._call_trigger(self.save_srv, "save_result")

    def compute_and_save(self) -> None:
        ok = self._call_trigger(self.compute_srv, "compute_calibration")
        if ok:
            self._call_trigger(self.save_srv, "save_result")

    def request_shutdown(self) -> None:
        self.shutdown_requested = True

    def _on_mouse(self, event, x: int, y: int, _flags, _userdata) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for button in self.buttons:
            x1, y1, x2, y2 = button.rect
            if x1 <= x <= x2 and y1 <= y <= y2:
                button.action()
                break

    def _draw_buttons(self, canvas: np.ndarray, panel_y: int) -> None:
        h, w = canvas.shape[:2]
        btn_h = 38
        gap = 10
        margin = 10
        n = len(self.buttons)
        total_gap = gap * (n - 1)
        btn_w = max(110, int((w - 2 * margin - total_gap) / n))
        x = margin
        y1 = panel_y + 10
        y2 = y1 + btn_h
        for button in self.buttons:
            x2 = x + btn_w
            button.rect = (x, y1, x2, y2)
            cv2.rectangle(canvas, (x, y1), (x2, y2), (50, 60, 70), thickness=-1)
            cv2.rectangle(canvas, (x, y1), (x2, y2), (150, 170, 190), thickness=1)
            cv2.putText(
                canvas,
                button.label,
                (x + 8, y1 + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.47,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                button.hotkey,
                (x + 8, y1 + 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (180, 200, 220),
                1,
                cv2.LINE_AA,
            )
            x = x2 + gap

    def _render_frame(self) -> np.ndarray:
        with self.lock:
            img = None if self.latest_image is None else self.latest_image.copy()
            tag_valid = self.tag_valid
            xyz = None if self.last_pose_xyz is None else self.last_pose_xyz.copy()
            sample_count_local = self.sample_count
            sample_count_remote = self.remote_sample_count
            remaining_remote = self.remote_remaining_samples
            progress_remote = self.remote_progress_ratio
            can_compute = self.remote_can_compute
            has_result = self.remote_has_result
            has_end_pose = self.remote_has_end_pose
            has_target_pose = self.remote_has_target_pose
            status_error = self.remote_status_error
            status_age = (time.time() - self.last_status_time) if self.last_status_time > 0.0 else float("inf")
            msg = self.last_message
            msg_err = self.last_message_is_error
            msg_age = time.time() - self.last_message_time

        if sample_count_remote is None:
            sample_count = sample_count_local
            progress = float(min(1.0, sample_count / float(max(1, self.min_samples))))
            remaining = max(0, self.min_samples - sample_count)
        else:
            sample_count = sample_count_remote
            progress = progress_remote
            remaining = max(
                0,
                remaining_remote if remaining_remote is not None else (self.min_samples - sample_count_remote),
            )

        if img is None:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                img,
                "Waiting for image topic...",
                (30, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (180, 180, 180),
                2,
                cv2.LINE_AA,
            )

        if self.preview_scale != 1.0:
            img = cv2.resize(
                img,
                (int(img.shape[1] * self.preview_scale), int(img.shape[0] * self.preview_scale)),
                interpolation=cv2.INTER_AREA,
            )

        # Always-visible HUD in image area so sample count is readable even if panel is cropped.
        hud_x1, hud_y1 = 10, 10
        hud_x2, hud_y2 = 360, 92
        cv2.rectangle(img, (hud_x1, hud_y1), (hud_x2, hud_y2), (15, 15, 15), thickness=-1)
        cv2.rectangle(img, (hud_x1, hud_y1), (hud_x2, hud_y2), (120, 140, 160), thickness=1)
        cv2.putText(
            img,
            f"SAMPLES {sample_count}/{self.min_samples}",
            (hud_x1 + 10, hud_y1 + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (235, 245, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"REMAINING {remaining}   {progress * 100.0:.1f}%",
            (hud_x1 + 10, hud_y1 + 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            (150, 220, 160),
            2,
            cv2.LINE_AA,
        )

        h, w = img.shape[:2]
        panel_h = 210
        canvas = np.zeros((h + panel_h, w, 3), dtype=np.uint8)
        canvas[:h, :, :] = img

        # top status strip
        status_color = (0, 180, 0) if tag_valid else (0, 0, 220)
        cv2.rectangle(canvas, (0, 0), (w - 1, 34), (20, 20, 20), thickness=-1)
        cv2.putText(
            canvas,
            (
                f"Side: {self.side} | Tag: {'OK' if tag_valid else 'MISSING'} | "
                f"Samples: {sample_count}/{self.min_samples} ({progress * 100.0:.1f}%)"
            ),
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.circle(canvas, (w - 18, 17), 8, status_color, thickness=-1)

        if xyz is not None:
            cv2.putText(
                canvas,
                f"camera->tag xyz [m]: [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                canvas,
                "camera->tag xyz [m]: [n/a]",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            canvas,
            (
                f"Status: end_pose={'OK' if has_end_pose else 'MISSING'} | "
                f"target_pose={'OK' if has_target_pose else 'MISSING'} | "
                f"compute={'READY' if can_compute else 'WAIT'} | "
                f"result={'YES' if has_result else 'NO'}"
            ),
            (10, h + 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 220, 235),
            1,
            cv2.LINE_AA,
        )

        bar_x1 = 10
        bar_x2 = w - 10
        bar_y1 = h + 78
        bar_y2 = h + 98
        cv2.rectangle(canvas, (bar_x1, bar_y1), (bar_x2, bar_y2), (130, 140, 150), thickness=1)
        fill_width = int((bar_x2 - bar_x1 - 2) * progress)
        if fill_width > 0:
            cv2.rectangle(
                canvas,
                (bar_x1 + 1, bar_y1 + 1),
                (bar_x1 + 1 + fill_width, bar_y2 - 1),
                (60, 170, 95),
                thickness=-1,
            )
        cv2.putText(
            canvas,
            f"Collected: {sample_count}   Remaining: {remaining}",
            (10, h + 116),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (230, 235, 240),
            1,
            cv2.LINE_AA,
        )

        msg_color = (70, 100, 240) if msg_err else (100, 220, 120)
        if msg_age > 10.0:
            msg_color = (140, 140, 140)
        cv2.putText(
            canvas,
            f"Last: {msg}",
            (10, h + 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            msg_color,
            1,
            cv2.LINE_AA,
        )
        if status_error:
            status_color_text = (80, 165, 255)
            status_text = f"Calibrator status: {status_error}"
        elif status_age == float("inf"):
            status_color_text = (120, 180, 230)
            status_text = "Calibrator status: waiting for first status response..."
        else:
            status_color_text = (120, 200, 140) if status_age < 2.0 else (150, 170, 200)
            status_text = f"Calibrator status age: {status_age:.1f}s"
        cv2.putText(
            canvas,
            status_text,
            (10, h + 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            status_color_text,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Calibrator NS: {self.calibrator_ns}",
            (10, h + 138),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 210),
            1,
            cv2.LINE_AA,
        )

        self._draw_buttons(canvas, h + 146)
        return canvas

    def run(self) -> None:
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.shutdown_requested:
            self._refresh_status()
            canvas = self._render_frame()
            cv2.imshow(self.window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.request_shutdown()
            elif key in (ord("c"), 32):
                self.capture_sample()
            elif key == ord("r"):
                self.clear_samples()
            elif key == ord("k"):
                self.compute_calibration()
            elif key == ord("s"):
                self.save_result()
            elif key == ord("a"):
                self.compute_and_save()
            rate.sleep()
        cv2.destroyWindow(self.window_name)


def main() -> int:
    rospy.init_node("wrist_handeye_gui", anonymous=False)
    try:
        gui = WristHandEyeGUI()
        gui.run()
    except Exception as exc:
        rospy.logerr("wrist_handeye_gui failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
