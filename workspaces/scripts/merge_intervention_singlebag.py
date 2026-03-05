#!/usr/bin/env python3
"""Merge intervention multi-segment rosbags into single-bag episodes.

This script reads an intervention recording session (episode_*/episode*.bag*)
and writes a new session where each episode only contains one episode.bag.
The input folder is never modified.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fnmatch
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyYAML is required (python3-yaml).") from exc


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def parse_episode_num(name: str) -> int:
    match = re.match(r"episode_(\d+)$", name)
    if not match:
        return 10**9
    return int(match.group(1))


def load_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if isinstance(obj, dict):
        return obj
    return None


def run_cmd(cmd: Sequence[str], timeout: Optional[float] = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(cmd),
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def aggregate_bag_stats(bag_files: Sequence[Path]) -> Dict[str, Any]:
    total_size = 0
    total_messages = 0
    min_start = None
    max_end = None
    per_topic: Dict[str, Dict[str, Any]] = {}
    parse_warnings: List[str] = []

    for bag_file in bag_files:
        try:
            try:
                total_size += bag_file.stat().st_size
            except Exception:
                pass

            res = run_cmd(["rosbag", "info", "--yaml", str(bag_file)], timeout=30.0)
            if res.returncode != 0:
                parse_warnings.append(f"rosbag info failed: {bag_file.name}")
                continue
            try:
                info = yaml.safe_load(res.stdout) or {}
            except Exception:
                parse_warnings.append(f"yaml parse failed: {bag_file.name}")
                continue

            start = info.get("start")
            end = info.get("end")
            messages = info.get("messages", 0)
            if isinstance(start, (int, float)):
                min_start = start if min_start is None else min(min_start, start)
            if isinstance(end, (int, float)):
                max_end = end if max_end is None else max(max_end, end)
            if isinstance(messages, int):
                total_messages += messages

            for topic_info in info.get("topics", []) or []:
                topic_name = topic_info.get("topic")
                if not topic_name:
                    continue
                entry = per_topic.setdefault(
                    topic_name,
                    {
                        "messages": 0,
                        "type": topic_info.get("type"),
                        "bags": 0,
                    },
                )
                entry["messages"] += int(topic_info.get("messages", 0))
                entry["bags"] += 1
        except Exception as exc:
            parse_warnings.append(f"bag stats exception for {bag_file.name}: {exc}")

    duration = None
    if min_start is not None and max_end is not None:
        duration = float(max_end) - float(min_start)

    return {
        "bag_files": [str(p) for p in bag_files],
        "bag_count": len(bag_files),
        "total_size_bytes": total_size,
        "total_messages": total_messages,
        "start_time": min_start,
        "end_time": max_end,
        "duration_sec": duration,
        "per_topic": per_topic,
        "parse_warnings": parse_warnings,
    }


def parse_episode_filter(raw: str) -> Set[int]:
    values: Set[int] = set()
    text = raw.strip()
    if not text:
        return values
    for part in text.split(","):
        token = part.strip()
        if not token:
            continue
        if "-" in token:
            left, right = token.split("-", 1)
            start = int(left.strip())
            end = int(right.strip())
            if end < start:
                raise ValueError(f"Invalid episode filter range: {token}")
            for idx in range(start, end + 1):
                values.add(idx)
        else:
            values.add(int(token))
    return values


def is_episode_bag_like_name(name: str) -> bool:
    return fnmatch.fnmatch(name, "episode*.bag*")


def is_final_bag_file(path: Path) -> bool:
    if not path.is_file():
        return False
    name = path.name
    if not is_episode_bag_like_name(name):
        return False
    if name.endswith(".active"):
        return False
    return path.suffix == ".bag"


def bag_segment_sort_key(path: Path) -> Tuple[int, str]:
    stem = path.name.split(".bag", 1)[0]
    match = re.match(r"episode_seg(\d+)", stem)
    if match:
        return int(match.group(1)), path.name
    if path.name.startswith("episode"):
        return 1, path.name
    return 10**9, path.name


def collect_segment_bag_files(episode_dir: Path, prefix_name: str) -> List[Path]:
    candidates = sorted(episode_dir.glob(f"{prefix_name}.bag*"))
    return [p for p in candidates if is_final_bag_file(p)]


def collect_fallback_bag_files(episode_dir: Path) -> List[Path]:
    candidates = [p for p in episode_dir.glob("episode*.bag*") if is_final_bag_file(p)]
    return sorted(candidates, key=bag_segment_sort_key)


def copy_item(src: Path, dst: Path) -> None:
    if src.is_dir():
        shutil.copytree(src, dst)
    else:
        shutil.copy2(src, dst)


def try_import_rosbag():
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Failed to import rosbag. Run in ROS1 environment, e.g. 'ros1' + source use_robot.sh."
        ) from exc
    return rosbag


class InterventionMerger:
    MERGED_SEGMENT_GAP_SEC = 1e-4

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.session_dir = Path(args.session_dir).expanduser().resolve()
        if not self.session_dir.is_dir():
            raise RuntimeError(f"session-dir not found: {self.session_dir}")

        if args.output_root:
            self.output_root = Path(args.output_root).expanduser().resolve()
        else:
            self.output_root = self.session_dir.parent / f"{self.session_dir.name}_singlebag"
        if self.output_root == self.session_dir:
            raise RuntimeError("output-root cannot be the same as session-dir")

        self.episode_filter: Optional[Set[int]] = None
        if args.episode_filter.strip():
            self.episode_filter = parse_episode_filter(args.episode_filter)

        self.summary: Dict[str, Any] = {
            "created_at_utc": utc_now_iso(),
            "session_dir": str(self.session_dir),
            "output_root": str(self.output_root),
            "dry_run": bool(args.dry_run),
            "preserve_time_gaps": bool(args.preserve_time_gaps),
            "compressed_segment_gap_sec": self.MERGED_SEGMENT_GAP_SEC,
            "episode_filter": sorted(self.episode_filter) if self.episode_filter is not None else None,
            "episodes": [],
        }

    def _list_episode_dirs(self) -> List[Path]:
        episodes = [p for p in self.session_dir.iterdir() if p.is_dir() and p.name.startswith("episode_")]
        episodes.sort(key=lambda p: parse_episode_num(p.name))
        if self.episode_filter is None:
            return episodes
        out: List[Path] = []
        for ep in episodes:
            idx = parse_episode_num(ep.name)
            if idx in self.episode_filter:
                out.append(ep)
        return out

    def _prepare_output_root(self) -> None:
        if self.args.dry_run:
            return
        if self.output_root.exists():
            if not self.args.overwrite:
                raise RuntimeError(
                    f"output-root already exists: {self.output_root} (use --overwrite to replace)"
                )
            shutil.rmtree(self.output_root)
        self.output_root.mkdir(parents=True, exist_ok=False)

    def _copy_session_level_items(self) -> None:
        for item in sorted(self.session_dir.iterdir(), key=lambda p: p.name):
            if item.is_dir() and item.name.startswith("episode_"):
                continue
            if self.args.dry_run:
                print(f"[dry-run] copy session item: {item.name}")
                continue
            copy_item(item, self.output_root / item.name)

    def _resolve_episode_bag_order(
        self, episode_dir: Path, src_meta: Optional[Dict[str, Any]]
    ) -> Tuple[List[Path], List[Dict[str, Any]], List[str], str]:
        warnings: List[str] = []
        segments_detail: List[Dict[str, Any]] = []
        ordered_files: List[Path] = []

        active_segments = None
        if isinstance(src_meta, dict):
            timing = src_meta.get("timing")
            if isinstance(timing, dict):
                active_segments = timing.get("active_segments")

        if isinstance(active_segments, list) and active_segments:
            sortable: List[Tuple[int, Dict[str, Any]]] = []
            for seg in active_segments:
                if not isinstance(seg, dict):
                    continue
                idx_raw = seg.get("segment_index", 10**9)
                try:
                    idx = int(idx_raw)
                except Exception:
                    idx = 10**9
                sortable.append((idx, seg))
            sortable.sort(key=lambda x: x[0])

            for _, seg in sortable:
                prefix_raw = seg.get("bag_prefix")
                if not prefix_raw:
                    warnings.append("active_segments entry missing bag_prefix.")
                    continue
                prefix_name = Path(str(prefix_raw)).name
                seg_files = collect_segment_bag_files(episode_dir, prefix_name)
                if not seg_files:
                    warnings.append(f"No .bag files found for segment prefix: {prefix_name}")
                ordered_files.extend(seg_files)
                segments_detail.append(
                    {
                        "segment_index": seg.get("segment_index"),
                        "segment_mode": seg.get("segment_mode"),
                        "bag_prefix": prefix_name,
                        "files": [str(p) for p in seg_files],
                    }
                )

            dedup: List[Path] = []
            seen = set()
            for p in ordered_files:
                key = str(p.resolve())
                if key in seen:
                    continue
                seen.add(key)
                dedup.append(p)
            if dedup:
                return dedup, segments_detail, warnings, "active_segments"
            warnings.append("active_segments produced zero files, fallback to filename scan.")

        fallback = collect_fallback_bag_files(episode_dir)
        if not fallback:
            warnings.append("Fallback filename scan found no .bag files.")
        return fallback, segments_detail, warnings, "filename_fallback"

    def _copy_episode_non_bag_files(self, src_ep_dir: Path, dst_ep_dir: Path) -> None:
        for item in sorted(src_ep_dir.iterdir(), key=lambda p: p.name):
            name = item.name
            if is_episode_bag_like_name(name):
                continue
            if name in ("metadata.json", "metadata_intervention.json", "merge_report.json"):
                continue
            copy_item(item, dst_ep_dir / name)

    def _merge_bags(
        self,
        ordered_bags: Sequence[Path],
        output_bag: Path,
    ) -> Tuple[int, List[Dict[str, Any]], List[str], Dict[str, Any]]:
        rosbag = try_import_rosbag()
        tmp_path = output_bag.with_suffix(".bag.tmp")
        if tmp_path.exists():
            tmp_path.unlink()

        total_messages = 0
        per_input: List[Dict[str, Any]] = []
        warnings: List[str] = []
        previous_source_end_sec: Optional[float] = None
        next_target_start_sec: Optional[float] = None
        total_compressed_gap_sec = 0.0
        gap_events: List[Dict[str, Any]] = []
        preserve_time_gaps = bool(self.args.preserve_time_gaps)

        try:
            with rosbag.Bag(str(tmp_path), "w") as out_bag:
                for src in ordered_bags:
                    source_messages = 0
                    source_start_sec: Optional[float] = None
                    source_end_sec: Optional[float] = None
                    source_shift_sec = 0.0
                    shifted_end_sec: Optional[float] = None
                    original_gap_sec: Optional[float] = None
                    written_gap_sec: Optional[float] = None
                    compressed_gap_sec: Optional[float] = None
                    with rosbag.Bag(str(src), "r") as in_bag:
                        for topic, msg, stamp in in_bag.read_messages():
                            stamp_sec = float(stamp.to_sec())
                            if source_start_sec is None:
                                source_start_sec = stamp_sec
                                if previous_source_end_sec is not None:
                                    original_gap_sec = max(0.0, source_start_sec - previous_source_end_sec)
                                if next_target_start_sec is not None and not preserve_time_gaps:
                                    source_shift_sec = next_target_start_sec - source_start_sec
                            source_end_sec = stamp_sec
                            shifted_stamp = stamp
                            if abs(source_shift_sec) > 1e-12:
                                shifted_stamp = self._shift_time(stamp, source_shift_sec)
                            out_bag.write(topic, msg, shifted_stamp)
                            shifted_end_sec = stamp_sec + source_shift_sec
                            source_messages += 1
                            total_messages += 1

                    if source_messages <= 0:
                        warnings.append(f"Input bag contains zero messages: {src.name}")
                    if source_start_sec is not None and source_end_sec is not None:
                        previous_source_end_sec = source_end_sec
                    if shifted_end_sec is not None:
                        next_target_start_sec = (
                            shifted_end_sec + self.MERGED_SEGMENT_GAP_SEC
                            if not preserve_time_gaps
                            else None
                        )

                    if (
                        source_start_sec is not None
                        and source_shift_sec != 0.0
                        and original_gap_sec is not None
                    ):
                        written_gap_sec = self.MERGED_SEGMENT_GAP_SEC
                        compressed_gap_sec = max(0.0, original_gap_sec - written_gap_sec)
                        total_compressed_gap_sec += compressed_gap_sec
                        gap_events.append(
                            {
                                "input_bag": str(src),
                                "original_gap_sec": original_gap_sec,
                                "written_gap_sec": written_gap_sec,
                                "compressed_gap_sec": compressed_gap_sec,
                            }
                        )

                    per_input.append(
                        {
                            "path": str(src),
                            "messages": source_messages,
                            "source_start_sec": source_start_sec,
                            "source_end_sec": source_end_sec,
                            "shift_sec": source_shift_sec,
                            "original_gap_sec": original_gap_sec,
                            "written_gap_sec": written_gap_sec,
                            "compressed_gap_sec": compressed_gap_sec,
                        }
                    )
            tmp_path.replace(output_bag)
        except Exception as exc:
            try:
                if tmp_path.exists():
                    tmp_path.unlink()
            except Exception:
                pass
            raise RuntimeError(f"Failed to merge bags into {output_bag.name}: {exc}") from exc

        if total_messages <= 0:
            warnings.append("Merged bag contains zero messages.")
        time_transform = {
            "preserve_time_gaps": preserve_time_gaps,
            "compressed_segment_gap_sec": None if preserve_time_gaps else self.MERGED_SEGMENT_GAP_SEC,
            "total_compressed_gap_sec": total_compressed_gap_sec,
            "gap_events": gap_events,
        }
        return total_messages, per_input, warnings, time_transform

    @staticmethod
    def _shift_time(stamp: Any, delta_sec: float) -> Any:
        if abs(delta_sec) <= 1e-12:
            return stamp
        sec_value = float(stamp.to_sec()) + delta_sec
        if sec_value < 0.0:
            sec_value = 0.0
        try:
            return type(stamp).from_sec(sec_value)
        except Exception:
            pass
        try:
            return stamp.__class__.from_sec(sec_value)
        except Exception:
            pass
        sec = int(sec_value)
        nsec = int(round((sec_value - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        if nsec < 0:
            nsec = 0
        try:
            return type(stamp)(sec, nsec)
        except Exception:
            return stamp

    def _build_standard_metadata(
        self,
        *,
        src_meta: Optional[Dict[str, Any]],
        episode_name: str,
        output_episode_dir: Path,
        bag_stats: Dict[str, Any],
        merge_warnings: Sequence[str],
    ) -> Dict[str, Any]:
        src = src_meta if isinstance(src_meta, dict) else {}

        def src_dict(key: str) -> Dict[str, Any]:
            val = src.get(key)
            return dict(val) if isinstance(val, dict) else {}

        episode_index = parse_episode_num(episode_name)
        if episode_index >= 10**9:
            episode_index = 0

        recording_src = src_dict("recording")
        recording_out = {
            "compression": recording_src.get("compression", "lz4"),
            "split_enabled": False,
            "split_size_mb": None,
            "camera_transport": recording_src.get("camera_transport"),
            "keep_debug_logs": bool(recording_src.get("keep_debug_logs", False)),
        }

        topics = src_dict("topics")
        preflight = src_dict("preflight")
        hz_monitor = src_dict("hz_monitor")
        debug_logs = src_dict("debug_logs")
        env_src = src_dict("environment")
        source_warnings = src.get("warnings")
        warning_list = list(source_warnings) if isinstance(source_warnings, list) else []
        warning_list.extend(list(merge_warnings))
        warning_list.extend([str(w) for w in bag_stats.get("parse_warnings", [])])

        environment_out = {
            "host": socket.gethostname(),
            "user": os.environ.get("USER"),
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
            "profile_path": env_src.get("profile_path"),
            "script_args": {
                "merge_intervention_singlebag": dict(vars(self.args)),
                "source_script_args": env_src.get("script_args"),
            },
            "key_rosparams": env_src.get("key_rosparams", {}),
            "source_session_dir": str(self.session_dir),
        }

        duration_sec = bag_stats.get("duration_sec")
        if duration_sec is None:
            duration_sec = src.get("duration_sec")

        metadata = {
            "schema_version": int(src.get("schema_version", 1)),
            "session_name": src.get("session_name", self.output_root.name),
            "episode_name": src.get("episode_name", episode_name),
            "episode_index": int(src.get("episode_index", episode_index)),
            "episode_dir": str(output_episode_dir),
            "started_at_utc": src.get("started_at_utc"),
            "ended_at_utc": src.get("ended_at_utc"),
            "duration_sec": duration_sec,
            "stop_reason": src.get("stop_reason"),
            "recording": recording_out,
            "topics": topics,
            "preflight": preflight,
            "hz_monitor": hz_monitor,
            "bag_stats": bag_stats,
            "environment": environment_out,
            "notes": src.get("notes", ""),
            "debug_logs": {
                "kept": list(debug_logs.get("kept", [])),
                "removed": list(debug_logs.get("removed", [])),
            },
            "warnings": warning_list,
            "rosbag_process_terminated_cleanly": bool(
                src.get("rosbag_process_terminated_cleanly", True)
            ),
            "save_job_id": src.get("save_job_id", 1),
        }
        return metadata

    def _process_episode(self, src_ep_dir: Path) -> Dict[str, Any]:
        episode_name = src_ep_dir.name
        src_meta_path = src_ep_dir / "metadata.json"
        src_meta = load_json(src_meta_path)
        ordered_bags, segment_details, order_warnings, order_source = self._resolve_episode_bag_order(
            src_ep_dir, src_meta
        )

        report: Dict[str, Any] = {
            "episode_name": episode_name,
            "source_episode_dir": str(src_ep_dir),
            "output_episode_dir": str(self.output_root / episode_name),
            "created_at_utc": utc_now_iso(),
            "order_source": order_source,
            "segment_details": segment_details,
            "input_bag_files": [str(p) for p in ordered_bags],
            "warnings": list(order_warnings),
            "success": False,
            "preserve_time_gaps": bool(self.args.preserve_time_gaps),
        }

        if not ordered_bags:
            report["error"] = "No input bag files found for episode."
            return report

        if self.args.dry_run:
            report["success"] = True
            report["dry_run"] = True
            report["planned_merged_bag"] = str((self.output_root / episode_name / "episode.bag"))
            return report

        dst_ep_dir = self.output_root / episode_name
        dst_ep_dir.mkdir(parents=True, exist_ok=False)
        self._copy_episode_non_bag_files(src_ep_dir, dst_ep_dir)

        if src_meta is not None:
            (dst_ep_dir / "metadata_intervention.json").write_text(
                json.dumps(src_meta, indent=2, ensure_ascii=True),
                encoding="utf-8",
            )

        merged_bag = dst_ep_dir / "episode.bag"
        total_messages = 0
        per_input_messages: List[Dict[str, Any]] = []
        merge_warnings: List[str] = list(order_warnings)
        try:
            (
                total_messages,
                per_input_messages,
                merge_extra_warnings,
                time_transform,
            ) = self._merge_bags(ordered_bags, merged_bag)
            merge_warnings.extend(merge_extra_warnings)
            bag_stats = aggregate_bag_stats([merged_bag])
            metadata = self._build_standard_metadata(
                src_meta=src_meta,
                episode_name=episode_name,
                output_episode_dir=dst_ep_dir,
                bag_stats=bag_stats,
                merge_warnings=merge_warnings,
            )
            (dst_ep_dir / "metadata.json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=True),
                encoding="utf-8",
            )

            report.update(
                {
                    "merged_bag": str(merged_bag),
                    "merged_total_messages": total_messages,
                    "per_input_messages": per_input_messages,
                    "time_transform": time_transform,
                    "bag_stats": bag_stats,
                    "success": True,
                }
            )
            if merge_warnings:
                report["warnings"] = merge_warnings
        except Exception as exc:
            report["error"] = str(exc)
            report["traceback"] = traceback.format_exc(limit=3)
            report["success"] = False

        (dst_ep_dir / "merge_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=True),
            encoding="utf-8",
        )
        return report

    def run(self) -> int:
        episodes = self._list_episode_dirs()
        if not episodes:
            raise RuntimeError(f"No episode_* folders found under {self.session_dir}")

        self._prepare_output_root()
        self._copy_session_level_items()

        if self.args.dry_run:
            print(f"[dry-run] session={self.session_dir}")
            print(f"[dry-run] output={self.output_root}")
            print(f"[dry-run] episodes={len(episodes)}")
            print(f"[dry-run] preserve_time_gaps={bool(self.args.preserve_time_gaps)}")
        else:
            print(f"[merge] session={self.session_dir}")
            print(f"[merge] output={self.output_root}")
            print(f"[merge] episodes={len(episodes)}")
            print(f"[merge] preserve_time_gaps={bool(self.args.preserve_time_gaps)}")

        success_count = 0
        fail_count = 0
        for idx, src_ep_dir in enumerate(episodes, start=1):
            tag = "[dry-run]" if self.args.dry_run else "[merge]"
            print(f"{tag} {idx}/{len(episodes)} {src_ep_dir.name}")
            report = self._process_episode(src_ep_dir)
            self.summary["episodes"].append(report)
            if report.get("success"):
                success_count += 1
            else:
                fail_count += 1
                print(f"{tag} FAIL {src_ep_dir.name}: {report.get('error', 'unknown error')}")

        self.summary["success_count"] = success_count
        self.summary["fail_count"] = fail_count
        self.summary["episode_count"] = len(episodes)
        self.summary["finished_at_utc"] = utc_now_iso()

        if not self.args.dry_run:
            (self.output_root / "merge_summary.json").write_text(
                json.dumps(self.summary, indent=2, ensure_ascii=True),
                encoding="utf-8",
            )

        print(
            f"{'[dry-run]' if self.args.dry_run else '[merge]'} done "
            f"success={success_count} fail={fail_count}"
        )

        if fail_count > 0:
            return 2
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge intervention multi-segment bags into single-bag episodes."
    )
    parser.add_argument(
        "--session-dir",
        required=True,
        help="Input rosbag session directory containing episode_* folders.",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Output session directory (default: <session-dir>_singlebag).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove output-root first when it already exists.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print merge plan without writing files.",
    )
    parser.add_argument(
        "--episode-filter",
        default="",
        help="Episode indices to process, e.g. '1,3,5-8'. Empty means all.",
    )
    parser.add_argument(
        "--preserve-time-gaps",
        action="store_true",
        help="Keep original inter-segment timestamp gaps (default: compress gaps).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        merger = InterventionMerger(args)
        return merger.run()
    except Exception as exc:
        print(f"[merge] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
