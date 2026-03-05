#!/usr/bin/env python3
"""Extract teleop takeover segments into a standard single-bag session."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

SCRIPT_DIR = Path(__file__).resolve().parent
import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from session_format_utils import (  # noqa: E402
    aggregate_single_bag_stats,
    build_standard_metadata,
    copy_or_link_file,
    list_episode_dirs,
    load_json_dict,
    parse_episode_num,
    validate_episode_or_raise,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract teleop segments from intervention raw session "
            "(timing.active_segments) into a standard episode.bag dataset."
        )
    )
    parser.add_argument(
        "--session-dir",
        required=True,
        help="Input intervention raw session directory.",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Output session directory (default: <session-dir>_teleop_only).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output directory if it exists.",
    )
    parser.add_argument(
        "--copy-mode",
        choices=("copy", "symlink", "hardlink"),
        default="copy",
        help="How to place episode bag files into output (default: copy).",
    )
    parser.add_argument(
        "--strict",
        dest="strict",
        action="store_true",
        default=True,
        help="Strict mode (default): fail on missing/invalid teleop segments.",
    )
    parser.add_argument(
        "--no-strict",
        dest="strict",
        action="store_false",
        help="Non-strict mode: skip invalid episodes/segments.",
    )
    return parser.parse_args()


def _load_intervention_metadata(episode_dir: Path) -> Dict[str, Any]:
    preferred = episode_dir / "metadata.json"
    fallback = episode_dir / "metadata_intervention.json"
    if preferred.is_file():
        return load_json_dict(preferred)
    if fallback.is_file():
        return load_json_dict(fallback)
    raise RuntimeError(
        f"Missing metadata file in {episode_dir} "
        "(expected metadata.json or metadata_intervention.json)"
    )


def _sort_segment_key(segment: Mapping[str, Any]) -> int:
    value = segment.get("segment_index")
    try:
        return int(value)
    except Exception:
        return 10**9


def _extract_teleop_segments(metadata: Mapping[str, Any]) -> List[Dict[str, Any]]:
    timing = metadata.get("timing")
    if not isinstance(timing, dict):
        return []
    active_segments = timing.get("active_segments")
    if not isinstance(active_segments, list):
        return []
    teleop_segments: List[Dict[str, Any]] = []
    for item in active_segments:
        if not isinstance(item, dict):
            continue
        if str(item.get("segment_mode")) != "teleop":
            continue
        teleop_segments.append(dict(item))
    teleop_segments.sort(key=_sort_segment_key)
    return teleop_segments


def _resolve_segment_bag(
    *,
    episode_dir: Path,
    segment: Mapping[str, Any],
    strict: bool,
) -> Optional[Path]:
    prefix_raw = segment.get("bag_prefix")
    if not prefix_raw:
        message = f"teleop segment missing bag_prefix in {episode_dir}"
        if strict:
            raise RuntimeError(message)
        print(f"[warn] {message}")
        return None

    prefix_name = Path(str(prefix_raw)).name
    candidates = sorted(episode_dir.glob(f"{prefix_name}.bag*"))
    final_bags = [
        p
        for p in candidates
        if p.is_file() and p.suffix == ".bag" and not p.name.endswith(".active")
    ]
    if len(final_bags) == 1:
        return final_bags[0]

    if len(final_bags) == 0:
        message = f"no final .bag found for teleop segment prefix={prefix_name} in {episode_dir}"
    else:
        message = (
            f"multiple final .bag files found for teleop segment prefix={prefix_name} "
            f"in {episode_dir}: {[p.name for p in final_bags]}"
        )
    if strict:
        raise RuntimeError(message)
    print(f"[warn] {message}")
    return None


def _prepare_output_root(output_root: Path, overwrite: bool) -> None:
    if output_root.exists():
        if not overwrite:
            raise RuntimeError(
                f"Output already exists: {output_root} (use --overwrite to replace)"
            )
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=False)


def _to_optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def run(args: argparse.Namespace) -> int:
    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        raise RuntimeError(f"session-dir not found: {session_dir}")

    if args.output_root:
        output_root = Path(args.output_root).expanduser().resolve()
    else:
        output_root = session_dir.parent / f"{session_dir.name}_teleop_only"

    _prepare_output_root(output_root, overwrite=bool(args.overwrite))
    print(f"[extract] session={session_dir}")
    print(f"[extract] output={output_root}")
    print(f"[extract] copy_mode={args.copy_mode} strict={bool(args.strict)}")

    source_episodes = list_episode_dirs(session_dir)
    if not source_episodes:
        raise RuntimeError(f"No episode_* found under {session_dir}")

    output_episode_index = 1
    extracted_count = 0
    teleop_segment_count = 0

    for src_episode_dir in source_episodes:
        source_meta = _load_intervention_metadata(src_episode_dir)
        teleop_segments = _extract_teleop_segments(source_meta)
        if not teleop_segments:
            if args.strict and source_meta.get("timing") is None:
                raise RuntimeError(
                    f"Missing timing.active_segments in {src_episode_dir}/metadata.json "
                    "for strict intervention extraction."
                )
            print(f"[extract] skip {src_episode_dir.name}: no teleop segments")
            continue

        teleop_segment_count += len(teleop_segments)
        for segment in teleop_segments:
            src_bag = _resolve_segment_bag(
                episode_dir=src_episode_dir,
                segment=segment,
                strict=bool(args.strict),
            )
            if src_bag is None:
                continue

            output_episode_name = f"episode_{output_episode_index:03d}"
            output_episode_dir = output_root / output_episode_name
            output_episode_dir.mkdir(parents=True, exist_ok=False)

            output_bag = output_episode_dir / "episode.bag"
            copy_or_link_file(src_bag, output_bag, copy_mode=str(args.copy_mode))

            bag_stats = aggregate_single_bag_stats(output_bag)
            segment_duration = _to_optional_float(segment.get("duration_sec"))
            duration_sec = _to_optional_float(bag_stats.get("duration_sec"))
            if duration_sec is None:
                duration_sec = segment_duration

            metadata = build_standard_metadata(
                source_meta=source_meta,
                session_name=output_root.name,
                episode_name=output_episode_name,
                episode_index=output_episode_index,
                episode_dir=output_episode_dir,
                bag_stats=bag_stats,
                started_at_utc=segment.get("started_at_utc"),
                ended_at_utc=segment.get("ended_at_utc"),
                duration_sec=duration_sec,
                stop_reason=segment.get("stop_reason"),
                force_single_bag=True,
            )
            write_json(output_episode_dir / "metadata.json", metadata)

            validate_episode_or_raise(
                episode_dir=output_episode_dir,
                expected_episode_name=output_episode_name,
                expected_episode_index=output_episode_index,
                strict=bool(args.strict),
            )
            print(
                f"[extract] {src_episode_dir.name} seg={segment.get('segment_index')} "
                f"-> {output_episode_name}"
            )

            output_episode_index += 1
            extracted_count += 1

    if extracted_count <= 0:
        raise RuntimeError(
            "No teleop episode extracted. "
            f"source_episodes={len(source_episodes)} teleop_segments={teleop_segment_count}"
        )

    print(f"[extract] done episodes={extracted_count}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"[extract] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
