#!/usr/bin/env python3
"""Extract image topics from a ROS1 bag into video files.

Examples:
  python3 unpack_bag_videos.py --bag /path/to/episode.bag
  python3 unpack_bag_videos.py --bag /path/to/episode.bag --list-only
  python3 unpack_bag_videos.py --bag /path/to/episode.bag \\
      --topics /realsense_left/color/image_raw/compressed /realsense_top/color/image_raw/compressed
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def sanitize_topic(topic: str) -> str:
    return re.sub(r"[^a-zA-Z0-9._-]+", "_", topic.strip("/")) or "root"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract image topics from a ROS1 bag into videos.")
    parser.add_argument("--bag", required=True, help="Input ROS bag path.")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory (default: <bag_stem>_videos next to bag).",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=None,
        help="Image topics to export. If omitted, all image topics are exported.",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Video FPS (default: 30).")
    parser.add_argument("--codec", default="avc1", help="Preferred FourCC codec (default: avc1).")
    parser.add_argument(
        "--container",
        choices=("auto", "avi", "mp4"),
        default="auto",
        help="Output container extension (default: auto; mp4 for mp4 codecs, else avi).",
    )
    parser.add_argument("--max-frames", type=int, default=0, help="Per-topic max frames (0 = unlimited).")
    parser.add_argument("--list-only", action="store_true", help="Only list detected image topics.")
    parser.add_argument(
        "--no-validate-output",
        action="store_true",
        help="Skip post-export readability check.",
    )
    parser.add_argument(
        "--no-fallback-frames",
        action="store_true",
        help="Disable fallback extraction of compressed image frames on video failure.",
    )
    parser.add_argument(
        "--fallback-mux-mp4",
        action="store_true",
        help="If fallback frames are extracted and ffmpeg is available, mux an MP4.",
    )
    return parser.parse_args()


def import_runtime_deps():
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
        import rosbag  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing runtime deps. Run in ROS1 env with python3 packages: rosbag, cv2, numpy."
        ) from exc
    return cv2, np, rosbag


def _meta_type(meta) -> str:
    if hasattr(meta, "msg_type"):
        return meta.msg_type
    if isinstance(meta, (tuple, list)) and meta:
        return meta[0]
    return ""


def _meta_count(meta) -> int:
    if hasattr(meta, "message_count"):
        return int(meta.message_count)
    if isinstance(meta, (tuple, list)) and len(meta) >= 2:
        return int(meta[1])
    return 0


def discover_image_topics(bag) -> List[Tuple[str, str, int]]:
    topics_info = bag.get_type_and_topic_info()[1]
    out: List[Tuple[str, str, int]] = []
    for topic, meta in topics_info.items():
        msg_type = _meta_type(meta)
        if msg_type in ("sensor_msgs/CompressedImage", "sensor_msgs/Image"):
            out.append((topic, msg_type, _meta_count(meta)))
    out.sort(key=lambda x: x[0])
    return out


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
    return arr.reshape(height, width, channels) if channels > 1 else arr.reshape(height, width)


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
    return (arr[:, :, 0].astype(np.uint16) | (arr[:, :, 1].astype(np.uint16) << 8))


def decode_raw_image(msg, cv2, np):
    enc = msg.encoding.lower()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)
    data = msg.data
    if h <= 0 or w <= 0:
        return None

    if enc in ("bgr8",):
        img = _reshape_with_step_8u(data, h, w, 3, step, np)
        return img
    if enc in ("rgb8",):
        img = _reshape_with_step_8u(data, h, w, 3, step, np)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if img is not None else None
    if enc in ("bgra8",):
        img = _reshape_with_step_8u(data, h, w, 4, step, np)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) if img is not None else None
    if enc in ("rgba8",):
        img = _reshape_with_step_8u(data, h, w, 4, step, np)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if img is not None else None
    if enc in ("mono8", "8uc1"):
        img = _reshape_with_step_8u(data, h, w, 1, step, np)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img is not None else None
    if enc in ("mono16", "16uc1"):
        img16 = _reshape_with_step_16u(data, h, w, step, np)
        if img16 is None:
            return None
        img8 = (img16 / 256).astype(np.uint8)
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
    return None


def decode_image(msg, msg_type: str, cv2, np):
    if msg_type == "sensor_msgs/CompressedImage":
        return decode_compressed_image(msg, cv2, np)
    if msg_type == "sensor_msgs/Image":
        return decode_raw_image(msg, cv2, np)
    return None


def output_extension(codec: str, container: str) -> str:
    if container in ("avi", "mp4"):
        return container
    mp4_codecs = {"mp4v", "avc1", "h264", "x264"}
    return "mp4" if codec.lower() in mp4_codecs else "avi"


def writer_candidates(codec: str, container: str) -> List[Tuple[str, str]]:
    candidates = [(codec, container)]
    # Prefer broadly compatible MP4 before MJPG AVI on Linux desktops.
    if codec.lower() != "mp4v":
        candidates.append(("mp4v", "mp4"))
    if codec.upper() != "MJPG":
        candidates.append(("MJPG", "avi"))
    dedup: List[Tuple[str, str]] = []
    seen = set()
    for item in candidates:
        if item not in seen:
            seen.add(item)
            dedup.append(item)
    return dedup


def validate_video(path: Path, cv2) -> Tuple[bool, str]:
    if not path.is_file():
        return False, "file_not_found"
    if path.stat().st_size <= 0:
        return False, "file_empty"
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        cap.release()
        return False, "capture_open_failed"
    ok, _frame = cap.read()
    cap.release()
    if not ok:
        return False, "no_decodable_frames"
    return True, "ok"


def infer_compressed_ext(msg) -> str:
    fmt = (getattr(msg, "format", "") or "").lower()
    if "jpeg" in fmt or "jpg" in fmt:
        return "jpg"
    if "png" in fmt:
        return "png"
    payload = bytes(msg.data)
    if payload.startswith(b"\xff\xd8\xff"):
        return "jpg"
    if payload.startswith(b"\x89PNG\r\n\x1a\n"):
        return "png"
    return "bin"


def dump_compressed_frames(
    bag,
    topics: List[str],
    out_dir: Path,
    max_frames: Optional[int],
) -> Dict[str, Dict[str, object]]:
    frame_dirs = {topic: out_dir / f"{sanitize_topic(topic)}_frames" for topic in topics}
    frame_ext: Dict[str, str] = {}
    frame_count = {topic: 0 for topic in topics}
    done_topics = set()

    for topic, msg, _t in bag.read_messages(topics=topics):
        if topic in done_topics:
            continue
        if topic not in frame_ext:
            frame_ext[topic] = infer_compressed_ext(msg)
            frame_dirs[topic].mkdir(parents=True, exist_ok=True)
        ext = frame_ext[topic]
        idx = frame_count[topic]
        out_path = frame_dirs[topic] / f"{idx:06d}.{ext}"
        out_path.write_bytes(bytes(msg.data))
        frame_count[topic] += 1
        if max_frames is not None and frame_count[topic] >= max_frames:
            done_topics.add(topic)
            if len(done_topics) == len(topics):
                break

    result: Dict[str, Dict[str, object]] = {}
    for topic in topics:
        result[topic] = {
            "frame_dir": str(frame_dirs[topic]),
            "frame_ext": frame_ext.get(topic, ""),
            "frame_count": frame_count[topic],
        }
    return result


def mux_frames_with_ffmpeg(frame_dir: Path, frame_ext: str, out_path: Path, fps: float) -> Tuple[bool, str]:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        return False, "ffmpeg_not_found"
    if not frame_ext:
        return False, "frame_extension_unknown"
    pattern = str(frame_dir / f"%06d.{frame_ext}")
    cmd = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        pattern,
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(out_path),
    ]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout).strip()
        return False, err or f"ffmpeg_exit_{proc.returncode}"
    return True, "ok"


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_file():
        print(f"[unpack] bag not found: {bag_path}", file=sys.stderr)
        return 1
    if len(args.codec) != 4:
        print("[unpack] codec must be 4 chars (FourCC), e.g., mp4v, XVID, avc1", file=sys.stderr)
        return 1
    if args.fps <= 0:
        print("[unpack] fps must be > 0", file=sys.stderr)
        return 1

    cv2, np, rosbag = import_runtime_deps()

    if args.output_dir:
        out_dir = Path(args.output_dir).expanduser().resolve()
    else:
        out_dir = bag_path.parent / f"{bag_path.stem}_videos"
    out_dir.mkdir(parents=True, exist_ok=True)

    bag = rosbag.Bag(str(bag_path), "r")
    try:
        discovered = discover_image_topics(bag)
        if not discovered:
            print("[unpack] no sensor_msgs/Image or sensor_msgs/CompressedImage topics found.")
            return 0

        print("[unpack] discovered image topics:")
        for topic, msg_type, count in discovered:
            print(f"  - {topic} [{msg_type}] ({count} msgs)")

        if args.list_only:
            return 0

        discovered_map = {topic: msg_type for topic, msg_type, _ in discovered}
        if args.topics:
            topics = args.topics
            unknown = [t for t in topics if t not in discovered_map]
            if unknown:
                print("[unpack] topics not found as image topics:", file=sys.stderr)
                for t in unknown:
                    print(f"  - {t}", file=sys.stderr)
                return 2
        else:
            topics = [topic for topic, _, _ in discovered]

        topic_types = {topic: discovered_map[topic] for topic in topics}
        writers: Dict[str, object] = {}
        writer_open_errors: Dict[str, str] = {}
        stats: Dict[str, Dict[str, object]] = {}

        for topic in topics:
            stats[topic] = {
                "frames_written": 0,
                "frames_failed": 0,
                "output": "",
                "video_codec": "",
                "video_container": "",
            }

        max_frames = args.max_frames if args.max_frames > 0 else None
        done_topics = set()

        for topic, msg, _t in bag.read_messages(topics=topics):
            if topic in done_topics:
                continue

            frame = decode_image(msg, topic_types[topic], cv2, np)
            if frame is None:
                stats[topic]["frames_failed"] += 1
                continue

            if topic not in writers:
                h, w = frame.shape[:2]
                opened = False
                tried: List[str] = []
                for codec_try, container_try in writer_candidates(args.codec, args.container):
                    ext = output_extension(codec_try, container_try)
                    out_path = out_dir / f"{sanitize_topic(topic)}.{ext}"
                    fourcc = cv2.VideoWriter_fourcc(*codec_try)
                    writer = cv2.VideoWriter(str(out_path), fourcc, args.fps, (w, h))
                    tried.append(f"{codec_try}:{ext}")
                    if not writer.isOpened():
                        writer.release()
                        continue
                    writers[topic] = writer
                    stats[topic]["output"] = str(out_path)
                    stats[topic]["video_codec"] = codec_try
                    stats[topic]["video_container"] = ext
                    opened = True
                    break
                if not opened:
                    reason = f"writer_open_failed tried={','.join(tried)}"
                    writer_open_errors[topic] = reason
                    stats[topic]["writer_error"] = reason
                    done_topics.add(topic)
                    continue

            writers[topic].write(frame)
            stats[topic]["frames_written"] += 1

            if max_frames is not None and stats[topic]["frames_written"] >= max_frames:
                done_topics.add(topic)
                if topic in writers:
                    writers[topic].release()
                    del writers[topic]
                if len(done_topics) == len(topics):
                    break

        for writer in writers.values():
            writer.release()

        invalid_outputs: List[Tuple[str, str, str]] = []
        if not args.no_validate_output:
            for topic in topics:
                if topic in writer_open_errors:
                    stats[topic]["playable"] = False
                    invalid_outputs.append((topic, str(stats[topic]["output"]), writer_open_errors[topic]))
                    continue
                if not stats[topic]["output"]:
                    stats[topic]["playable"] = False
                    invalid_outputs.append((topic, "", "no_output_path"))
                    continue
                out_path = Path(str(stats[topic]["output"]))
                valid, reason = validate_video(out_path, cv2)
                stats[topic]["playable"] = valid
                if not valid:
                    invalid_outputs.append((topic, str(out_path), reason))

        fallback_info: Dict[str, Dict[str, object]] = {}
        use_fallback_frames = not args.no_fallback_frames
        if invalid_outputs and use_fallback_frames:
            fallback_topics = [
                topic for topic, _path, _reason in invalid_outputs if topic_types[topic] == "sensor_msgs/CompressedImage"
            ]
            if fallback_topics:
                print("[unpack] exporting fallback compressed frames for unreadable outputs...")
                fallback_info = dump_compressed_frames(bag, fallback_topics, out_dir, max_frames)
                for topic, info in fallback_info.items():
                    stats[topic]["fallback_frame_dir"] = info["frame_dir"]
                    stats[topic]["fallback_frame_ext"] = info["frame_ext"]
                    stats[topic]["fallback_frame_count"] = info["frame_count"]
                    if args.fallback_mux_mp4:
                        frame_dir = Path(str(info["frame_dir"]))
                        frame_ext = str(info["frame_ext"])
                        out_mp4 = out_dir / f"{sanitize_topic(topic)}_fallback.mp4"
                        ok, reason = mux_frames_with_ffmpeg(frame_dir, frame_ext, out_mp4, args.fps)
                        stats[topic]["fallback_mp4"] = str(out_mp4) if ok else ""
                        stats[topic]["fallback_mp4_status"] = reason

        print("[unpack] export summary:")
        for topic in topics:
            s = stats[topic]
            print(
                f"  - {topic}\n"
                f"    output: {s['output']}\n"
                f"    codec: {s.get('video_codec', '')}\n"
                f"    container: {s.get('video_container', '')}\n"
                f"    frames_written: {s['frames_written']}\n"
                f"    frames_failed: {s['frames_failed']}\n"
                f"    playable: {s.get('playable', 'not_checked')}\n"
                f"    fallback_frame_count: {s.get('fallback_frame_count', 0)}\n"
                f"    fallback_frame_dir: {s.get('fallback_frame_dir', '')}\n"
                f"    fallback_mp4: {s.get('fallback_mp4', '')}\n"
                f"    fallback_mp4_status: {s.get('fallback_mp4_status', '')}"
            )
        if invalid_outputs:
            print("[unpack] ERROR: unreadable output videos detected:", file=sys.stderr)
            for topic, path, reason in invalid_outputs:
                print(f"  - {topic}: {path} ({reason})", file=sys.stderr)
            if use_fallback_frames:
                print(
                    "[unpack] Fallback frame extraction completed for compressed topics. "
                    "You can inspect frame folders even if video backend is broken.",
                    file=sys.stderr,
                )
                if not args.fallback_mux_mp4:
                    print(
                        "[unpack] Optional: add --fallback-mux-mp4 to auto-create MP4 via ffmpeg (if installed).",
                        file=sys.stderr,
                    )
            else:
                print(
                    "[unpack] Hint: rerun with --codec MJPG --container avi "
                    "or enable fallback extraction.",
                    file=sys.stderr,
                )
            return 4
        return 0
    finally:
        bag.close()


if __name__ == "__main__":
    raise SystemExit(main())
