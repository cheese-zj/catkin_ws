#!/usr/bin/env python3
"""Record the overhead RealSense stream to an MP4 file.

Example:
  source /home/jameszhao2004/catkin_ws/workspaces/scripts/use_robot.sh
  python3 /home/jameszhao2004/catkin_ws/workspaces/scripts/record_overhead_mp4.py
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/realsense_top/color/image_raw/compressed",
        help="Overhead camera topic to record (default: /realsense_top/color/image_raw/compressed).",
    )
    parser.add_argument(
        "--topic-type",
        choices=("auto", "compressed", "raw"),
        default="auto",
        help="Message type for --topic (default: auto).",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output mp4 path (default: /home/jameszhao2004/catkin_ws/data/videos/overhead_<timestamp>.mp4).",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Output video fps (default: 30).")
    parser.add_argument("--codec", default="mp4v", help="OpenCV fourcc codec (default: mp4v).")
    parser.add_argument("--max-seconds", type=float, default=0.0, help="Stop after this many seconds (0 = unlimited).")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many written frames (0 = unlimited).")
    parser.add_argument("--resize-width", type=int, default=0, help="Optional output width.")
    parser.add_argument("--resize-height", type=int, default=0, help="Optional output height.")
    parser.add_argument("--show-preview", action="store_true", help="Preview frames in an OpenCV window while recording.")
    parser.add_argument("--queue-size", type=int, default=1, help="ROS subscriber queue size (default: 1).")
    return parser.parse_args()


def import_runtime_deps():
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
        import rospy  # type: ignore
        import rostopic  # type: ignore
        from sensor_msgs.msg import CompressedImage, Image  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing runtime deps. Run in ROS1 env with python3 packages: rospy, rostopic, sensor_msgs, cv2, numpy."
        ) from exc
    return cv2, np, rospy, rostopic, CompressedImage, Image


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("/home/jameszhao2004/catkin_ws/data/videos") / f"overhead_{stamp}.mp4"


def normalize_output_path(raw: str) -> Path:
    path = Path(raw).expanduser().resolve() if raw.strip() else default_output_path()
    if path.suffix.lower() != ".mp4":
        path = path.with_suffix(".mp4")
    return path


def resolve_topic_type(topic: str, requested: str, rostopic_mod, compressed_cls, raw_cls) -> str:
    if requested in ("compressed", "raw"):
        return requested
    if topic.endswith("/compressed"):
        return "compressed"
    try:
        msg_cls, _real_topic, _eval_fn = rostopic_mod.get_topic_class(topic, blocking=False)
    except Exception:
        msg_cls = None
    if msg_cls is compressed_cls:
        return "compressed"
    if msg_cls is raw_cls:
        return "raw"
    raise RuntimeError(f"Could not infer topic type for {topic}. Pass --topic-type compressed|raw.")


def decode_compressed_image(msg, cv2, np):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if arr.size == 0:
        return None
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _reshape_with_step_8u(buf, height: int, width: int, channels: int, step: int, np):
    row_bytes = width * channels
    if step < row_bytes:
        return None
    arr = np.frombuffer(buf, dtype=np.uint8)
    if arr.size < height * step:
        return None
    arr = arr[: height * step].reshape(height, step)
    arr = arr[:, :row_bytes]
    if channels > 1:
        return arr.reshape(height, width, channels)
    return arr.reshape(height, width)


def _reshape_with_step_16u(buf, height: int, width: int, step: int, np):
    row_bytes = width * 2
    if step < row_bytes:
        return None
    arr = np.frombuffer(buf, dtype=np.uint8)
    if arr.size < height * step:
        return None
    arr = arr[: height * step].reshape(height, step)
    arr = arr[:, :row_bytes]
    arr = arr.reshape(height, width, 2)
    return arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8)


def decode_raw_image(msg, cv2, np):
    enc = str(msg.encoding).lower()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)
    if h <= 0 or w <= 0:
        return None

    if enc == "bgr8":
        return _reshape_with_step_8u(msg.data, h, w, 3, step, np)
    if enc == "rgb8":
        img = _reshape_with_step_8u(msg.data, h, w, 3, step, np)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if img is not None else None
    if enc == "bgra8":
        img = _reshape_with_step_8u(msg.data, h, w, 4, step, np)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) if img is not None else None
    if enc == "rgba8":
        img = _reshape_with_step_8u(msg.data, h, w, 4, step, np)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if img is not None else None
    if enc in ("mono8", "8uc1"):
        img = _reshape_with_step_8u(msg.data, h, w, 1, step, np)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img is not None else None
    if enc in ("mono16", "16uc1"):
        img16 = _reshape_with_step_16u(msg.data, h, w, step, np)
        if img16 is None:
            return None
        img8 = (img16 / 256).astype(np.uint8)
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
    return None


class OverheadMp4Recorder:
    def __init__(self, args: argparse.Namespace, cv2, np, rospy, compressed_cls, raw_cls):
        self.args = args
        self.cv2 = cv2
        self.np = np
        self.rospy = rospy
        self.compressed_cls = compressed_cls
        self.raw_cls = raw_cls
        self.lock = threading.Lock()

        self.output_path = normalize_output_path(args.output)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.writer = None
        self.writer_size: Optional[Tuple[int, int]] = None
        self.frames_received = 0
        self.frames_written = 0
        self.decode_failures = 0
        self.last_preview_frame = None
        self.start_wall = time.monotonic()
        self.last_log_wall = self.start_wall
        if self.args.show_preview:
            self.cv2.namedWindow("overhead_mp4_recorder", self.cv2.WINDOW_NORMAL)

    def _ensure_writer(self, frame):
        if self.writer is not None:
            return
        h, w = frame.shape[:2]
        if self.args.resize_width > 0 and self.args.resize_height > 0:
            w = int(self.args.resize_width)
            h = int(self.args.resize_height)
        self.writer_size = (w, h)
        fourcc = self.cv2.VideoWriter_fourcc(*str(self.args.codec)[:4])
        self.writer = self.cv2.VideoWriter(str(self.output_path), fourcc, float(self.args.fps), (w, h))
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {self.output_path} codec={self.args.codec}")

    def _prepare_frame(self, frame):
        if frame is None:
            return None
        if self.writer_size is None:
            self._ensure_writer(frame)
        if self.writer_size is not None and (
            frame.shape[1] != self.writer_size[0] or frame.shape[0] != self.writer_size[1]
        ):
            frame = self.cv2.resize(frame, self.writer_size, interpolation=self.cv2.INTER_AREA)
        return frame

    def _handle_frame(self, frame):
        with self.lock:
            self.frames_received += 1
            frame = self._prepare_frame(frame)
            if frame is None:
                self.decode_failures += 1
                return
            self.writer.write(frame)
            self.frames_written += 1
            if self.args.show_preview:
                self.last_preview_frame = frame.copy()

            now = time.monotonic()
            if now - self.last_log_wall >= 2.0:
                self.last_log_wall = now
                self.rospy.loginfo(
                    "record_overhead_mp4: received=%d written=%d output=%s",
                    self.frames_received,
                    self.frames_written,
                    str(self.output_path),
                )
            if self.args.max_frames > 0 and self.frames_written >= self.args.max_frames:
                self.rospy.signal_shutdown("max_frames reached")

    def cb_compressed(self, msg):
        frame = decode_compressed_image(msg, self.cv2, self.np)
        if frame is None:
            with self.lock:
                self.frames_received += 1
                self.decode_failures += 1
            return
        self._handle_frame(frame)

    def cb_raw(self, msg):
        frame = decode_raw_image(msg, self.cv2, self.np)
        if frame is None:
            with self.lock:
                self.frames_received += 1
                self.decode_failures += 1
            return
        self._handle_frame(frame)

    def maybe_show_preview(self):
        if not self.args.show_preview:
            return
        with self.lock:
            frame = None if self.last_preview_frame is None else self.last_preview_frame.copy()
        if frame is None:
            return
        self.cv2.imshow("overhead_mp4_recorder", frame)
        if self.cv2.waitKey(1) & 0xFF == ord("q"):
            self.rospy.signal_shutdown("preview closed by user")

    def should_stop_on_time(self) -> bool:
        return self.args.max_seconds > 0.0 and (time.monotonic() - self.start_wall) >= self.args.max_seconds

    def close(self):
        with self.lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
        if self.args.show_preview:
            self.cv2.destroyAllWindows()

    def validate_output(self) -> Tuple[bool, str]:
        if not self.output_path.is_file():
            return False, "file_not_found"
        if self.output_path.stat().st_size <= 0:
            return False, "file_empty"
        cap = self.cv2.VideoCapture(str(self.output_path))
        if not cap.isOpened():
            cap.release()
            return False, "capture_open_failed"
        ok, _frame = cap.read()
        cap.release()
        if not ok:
            return False, "read_failed"
        return True, "ok"


def main() -> int:
    args = parse_args()
    try:
        cv2, np, rospy, rostopic, compressed_cls, raw_cls = import_runtime_deps()
        topic_type = resolve_topic_type(args.topic, args.topic_type, rostopic, compressed_cls, raw_cls)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    rospy.init_node("record_overhead_mp4", anonymous=False)
    recorder = OverheadMp4Recorder(args, cv2, np, rospy, compressed_cls, raw_cls)

    callback = recorder.cb_compressed if topic_type == "compressed" else recorder.cb_raw
    msg_cls = compressed_cls if topic_type == "compressed" else raw_cls
    subscriber = rospy.Subscriber(args.topic, msg_cls, callback, queue_size=max(1, int(args.queue_size)), tcp_nodelay=True)

    rospy.loginfo(
        "Recording topic=%s type=%s to %s fps=%.2f codec=%s",
        args.topic,
        topic_type,
        str(recorder.output_path),
        float(args.fps),
        str(args.codec),
    )
    if args.max_seconds > 0.0:
        rospy.loginfo("Will stop after %.2f seconds.", float(args.max_seconds))
    if args.max_frames > 0:
        rospy.loginfo("Will stop after %d frames.", int(args.max_frames))

    try:
        while not rospy.is_shutdown():
            if recorder.should_stop_on_time():
                rospy.signal_shutdown("max_seconds reached")
                break
            recorder.maybe_show_preview()
            time.sleep(0.01)
    except KeyboardInterrupt:
        rospy.signal_shutdown("keyboard interrupt")
    finally:
        subscriber.unregister()
        recorder.close()

    ok, reason = recorder.validate_output()
    if not ok:
        print(
            f"[ERROR] output validation failed: {reason} path={recorder.output_path}",
            file=sys.stderr,
        )
        return 1

    print(f"saved_mp4={recorder.output_path}")
    print(f"frames_written={recorder.frames_written}")
    print(f"frames_received={recorder.frames_received}")
    print(f"decode_failures={recorder.decode_failures}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
