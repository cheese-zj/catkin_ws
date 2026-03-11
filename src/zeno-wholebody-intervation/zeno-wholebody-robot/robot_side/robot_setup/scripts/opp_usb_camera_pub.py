#!/usr/bin/env python3
"""Publish a V4L2 USB camera as ROS Image + CameraInfo topics.

Default namespace usage in launch:
  ns=/wrist_opp
  topics:
    /wrist_opp/image_raw
    /wrist_opp/camera_info
"""

from __future__ import annotations

import glob
import os
import subprocess
import time
import traceback
from typing import List, Optional

import cv2
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def _safe_fourcc(code: str) -> int:
    raw = (code or "").strip().upper()
    if len(raw) != 4:
        raw = "MJPG"
    return cv2.VideoWriter_fourcc(*raw)

def _decode_fourcc(value: float) -> str:
    try:
        ivalue = int(value)
    except Exception:
        return "UNKNOWN"

    text = "".join(chr((ivalue >> (8 * idx)) & 0xFF) for idx in range(4))
    text = text.replace("\x00", "").strip().upper()
    if len(text) != 4 or not text.isprintable():
        return "UNKNOWN"
    return text


def _build_camera_info(width: int, height: int, frame_id: str, stamp: rospy.Time) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = max(0, int(width))
    msg.height = max(0, int(height))
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    # Fallback intrinsics only for visualization; use real calibration if available.
    fx = float(max(1, width))
    fy = float(max(1, width))
    cx = float(width) / 2.0
    cy = float(height) / 2.0
    msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class OppUsbCameraPublisher:
    def __init__(self) -> None:
        self.configured_device = str(rospy.get_param("~device", "/dev/video4")).strip()
        self.device = self._normalize_capture_device(self.configured_device)
        self.usb_port = str(rospy.get_param("~usb_port", "")).strip()
        self.frame_id = rospy.get_param("~frame_id", "wrist_opp_camera_optical_frame")
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.fourcc = str(rospy.get_param("~fourcc", "MJPG")).strip().upper()
        self.capture_backend = str(rospy.get_param("~capture_backend", "v4l2")).strip().lower()
        self.gstreamer_pipeline = str(rospy.get_param("~gstreamer_pipeline", "")).strip()
        if self.capture_backend not in {"v4l2", "usb", "any"}:
            rospy.logwarn("Unsupported capture_backend=%s, fallback to v4l2.", self.capture_backend)
            self.capture_backend = "v4l2"
        self.output_encoding = str(rospy.get_param("~output_encoding", "bgr8")).strip().lower()
        if self.output_encoding not in {"bgr8", "rgb8"}:
            rospy.logwarn("Unsupported output_encoding=%s, fallback to bgr8.", self.output_encoding)
            self.output_encoding = "bgr8"
        self.flip_horizontal = bool(rospy.get_param("~flip_horizontal", False))
        self.flip_vertical = bool(rospy.get_param("~flip_vertical", False))
        self.reconnect_sec = float(rospy.get_param("~reconnect_sec", 1.0))
        self.publish_camera_info = bool(rospy.get_param("~publish_camera_info", True))
        self.publish_raw = bool(rospy.get_param("~publish_raw", True))
        self.publish_compressed = bool(rospy.get_param("~publish_compressed", True))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))
        if not self.publish_raw and not self.publish_compressed:
            rospy.logwarn("Both publish_raw and publish_compressed are disabled; forcing publish_raw=true.")
            self.publish_raw = True

        self._refresh_device_resolution(initial=True)

        self.image_pub = rospy.Publisher("image_raw", Image, queue_size=1) if self.publish_raw else None
        self.compressed_pub = (
            rospy.Publisher("image_raw/compressed", CompressedImage, queue_size=1)
            if self.publish_compressed
            else None
        )
        self.camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1) if self.publish_camera_info else None
        self.cap: Optional[cv2.VideoCapture] = None

    @staticmethod
    def _usb_port_tokens(usb_port: str) -> List[str]:
        raw = (usb_port or "").strip()
        if not raw:
            return []
        tokens = [raw]
        if ":" in raw:
            base = raw.split(":", 1)[0]
            if base and base not in tokens:
                tokens.append(base)
        return tokens

    @staticmethod
    def _contains_usb_token(sys_path: str, tokens: List[str]) -> bool:
        return any(token in sys_path for token in tokens if token)

    @staticmethod
    def _device_sys_path(dev_path: str) -> str:
        base = os.path.basename(str(dev_path).strip())
        sysfs_link = os.path.join("/sys/class/video4linux", base, "device")
        if os.path.exists(sysfs_link):
            try:
                return os.path.realpath(sysfs_link)
            except Exception:
                pass

        try:
            return subprocess.check_output(
                ["udevadm", "info", "--query=path", "--name", dev_path],
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip()
        except Exception:
            return ""

    @staticmethod
    def _normalize_capture_device(device: str) -> str:
        raw = str(device).strip()
        if not raw.startswith("/dev/"):
            return raw

        # Keep stable symlink paths as-is to avoid /dev/videoX drift after USB re-enumeration.
        if (raw.startswith("/dev/v4l/by-id/") or raw.startswith("/dev/v4l/by-path/")) and os.path.exists(raw):
            return raw

        resolved = os.path.realpath(raw)
        if resolved.startswith("/dev/") and os.path.exists(resolved):
            return resolved
        return raw

    def _resolve_device_by_usb_port(self, usb_port: str) -> Optional[str]:
        tokens = self._usb_port_tokens(usb_port)
        if not tokens:
            return None

        # Prefer stable by-id links and pick index0 first for USB webcams.
        matches = []
        for link in sorted(glob.glob("/dev/v4l/by-id/*")):
            if not os.path.islink(link):
                continue
            target = os.path.realpath(link)
            sys_path = self._device_sys_path(target)
            if not sys_path or not self._contains_usb_token(sys_path, tokens):
                continue
            score = 0 if "video-index0" in os.path.basename(link) else 1
            matches.append((score, link))

        if matches:
            matches.sort(key=lambda item: (item[0], item[1]))
            return os.path.realpath(matches[0][1])

        # Fallback to raw /dev/video* scan when by-id is missing.
        for dev in sorted(glob.glob("/dev/video*")):
            sys_path = self._device_sys_path(dev)
            if sys_path and self._contains_usb_token(sys_path, tokens):
                return dev
        return None

    def _refresh_device_resolution(self, initial: bool = False) -> bool:
        if not self.usb_port:
            self.device = self._normalize_capture_device(self.configured_device)
            if initial:
                rospy.loginfo("OPP camera capture device path: %s", self.device)
            return True

        resolved = self._resolve_device_by_usb_port(self.usb_port)
        if not resolved:
            rospy.logwarn_throttle(
                2.0,
                "OPP camera usb_port=%s not resolved; refusing to fall back to configured device=%s",
                self.usb_port,
                self.configured_device,
            )
            return False

        normalized = self._normalize_capture_device(resolved)
        if initial or normalized != self.device:
            rospy.loginfo("OPP camera resolved from usb_port=%s to device=%s", self.usb_port, normalized)
            rospy.loginfo("OPP camera capture device path: %s", normalized)
        self.device = normalized
        return True

    def _open_camera_with_api(self, api_preference: int) -> Optional[cv2.VideoCapture]:
        cap = cv2.VideoCapture(self.device, api_preference)
        if not cap.isOpened():
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        cap.set(cv2.CAP_PROP_FOURCC, float(_safe_fourcc(self.fourcc)))

        for _ in range(6):
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            time.sleep(0.03)

        cap.release()
        return None

    def _open_camera_with_fourcc(self) -> Optional[cv2.VideoCapture]:
        return self._open_camera_with_api(cv2.CAP_V4L2)

    def _open_camera_with_any_backend(self) -> Optional[cv2.VideoCapture]:
        return self._open_camera_with_api(cv2.CAP_ANY)

    def _build_usb_gstreamer_pipeline(self) -> str:
        if self.gstreamer_pipeline:
            return self.gstreamer_pipeline

        fps_num = max(1, int(round(float(self.fps))))
        if self.fourcc == "MJPG":
            caps = f"image/jpeg,width={self.width},height={self.height},framerate={fps_num}/1"
            decoder = "jpegdec ! videoconvert"
        elif self.fourcc == "YUYV":
            caps = f"video/x-raw,format=YUY2,width={self.width},height={self.height},framerate={fps_num}/1"
            decoder = "videoconvert"
        else:
            caps = f"video/x-raw,width={self.width},height={self.height},framerate={fps_num}/1"
            decoder = "videoconvert"

        return (
            f"v4l2src device={self.device} io-mode=2 do-timestamp=true ! {caps} ! "
            f"{decoder} ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )

    def _open_camera_with_usb_backend(self) -> Optional[cv2.VideoCapture]:
        gst_backend = getattr(cv2, "CAP_GSTREAMER", cv2.CAP_ANY)
        pipeline = self._build_usb_gstreamer_pipeline()
        cap = cv2.VideoCapture(pipeline, gst_backend)
        if not cap.isOpened():
            return None

        for _ in range(6):
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            time.sleep(0.03)

        cap.release()
        return None

    def _open_camera(self) -> bool:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None

        if not self._refresh_device_resolution():
            return False

        requested_backend = self.capture_backend
        selected_backend = requested_backend
        rospy.loginfo(
            "OPP camera opening with backend=%s requested FOURCC=%s",
            requested_backend,
            self.fourcc,
        )
        if requested_backend == "usb":
            cap = self._open_camera_with_usb_backend()
            if cap is None:
                selected_backend = "v4l2_fallback"
                rospy.logwarn("OPP camera usb backend open failed, fallback to v4l2.")
                cap = self._open_camera_with_fourcc()
        elif requested_backend == "any":
            cap = self._open_camera_with_any_backend()
            if cap is None:
                selected_backend = "v4l2_fallback"
                rospy.logwarn("OPP camera any backend open failed, fallback to v4l2.")
                cap = self._open_camera_with_fourcc()
        else:
            cap = self._open_camera_with_fourcc()

        if cap is None:
            rospy.logwarn_throttle(
                2.0,
                "OPP camera open/read failed with backend=%s requested FOURCC=%s: %s",
                requested_backend,
                self.fourcc,
                self.device,
            )
            return False

        self.cap = cap
        rospy.set_param("~active_capture_backend", selected_backend)
        active_fourcc = _decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
        rospy.set_param("~active_fourcc", active_fourcc)
        if active_fourcc != self.fourcc:
            rospy.logwarn(
                "OPP camera FOURCC fallback detected: requested=%s active=%s (driver/device overrode request).",
                self.fourcc,
                active_fourcc,
            )
        rospy.loginfo(
            "OPP camera opened: device=%s width=%d height=%d fps=%.2f backend=%s requested_fourcc=%s active_fourcc=%s",
            self.device,
            self.width,
            self.height,
            self.fps,
            selected_backend,
            self.fourcc,
            active_fourcc,
        )
        return True

    def _close_camera(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def _publish_frame(self, frame) -> None:
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.flip_vertical:
            frame = cv2.flip(frame, 0)

        output_frame = frame
        encoding = "bgr8"
        if self.output_encoding == "rgb8":
            output_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            encoding = "rgb8"

        height, width = output_frame.shape[:2]
        stamp = rospy.Time.now()

        if self.publish_raw and self.image_pub is not None:
            img = Image()
            img.header.stamp = stamp
            img.header.frame_id = self.frame_id
            img.height = int(height)
            img.width = int(width)
            img.encoding = encoding
            img.is_bigendian = 0
            img.step = int(width * 3)
            img.data = output_frame.tobytes()
            self.image_pub.publish(img)

        if self.publish_compressed and self.compressed_pub is not None and self.compressed_pub.get_num_connections() > 0:
            quality = max(1, min(100, int(self.jpeg_quality)))
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if ok and encoded is not None:
                comp = CompressedImage()
                comp.header.stamp = stamp
                comp.header.frame_id = self.frame_id
                comp.format = "jpeg"
                comp.data = encoded.tobytes()
                self.compressed_pub.publish(comp)
            else:
                rospy.logwarn_throttle(2.0, "Failed to encode OPP compressed frame.")

        if self.publish_camera_info and self.camera_info_pub is not None:
            self.camera_info_pub.publish(_build_camera_info(width, height, self.frame_id, stamp))

    def run(self) -> None:
        hz = max(1.0, float(self.fps))
        rate = rospy.Rate(hz)
        reconnect_sleep = max(0.1, float(self.reconnect_sec))

        while not rospy.is_shutdown():
            if self.cap is None and not self._open_camera():
                time.sleep(reconnect_sleep)
                continue

            assert self.cap is not None
            try:
                ok, frame = self.cap.read()
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "OPP camera read raised exception on %s: %s", self.device, str(exc))
                self._close_camera()
                time.sleep(reconnect_sleep)
                continue

            if not ok or frame is None:
                rospy.logwarn_throttle(2.0, "OPP camera read failed: %s", self.device)
                self._close_camera()
                time.sleep(reconnect_sleep)
                continue

            try:
                self._publish_frame(frame)
            except cv2.error as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "OPP camera publish failed (OpenCV) on %s: %s",
                    self.device,
                    str(exc),
                )
                self._close_camera()
                time.sleep(reconnect_sleep)
                continue
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "OPP camera publish failed on %s: %s",
                    self.device,
                    str(exc),
                )
                self._close_camera()
                time.sleep(reconnect_sleep)
                continue

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def shutdown(self) -> None:
        self._close_camera()


def main() -> int:
    rospy.init_node("opp_usb_camera_pub")
    node = OppUsbCameraPublisher()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
        return 0
    except Exception as exc:
        rospy.logerr("opp_usb_camera_pub failed: %s", str(exc))
        rospy.logerr(traceback.format_exc())
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
