#!/usr/bin/env python3
"""Merge two standard rosbag sessions into one standard session."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Dict, List, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from session_format_utils import (  # noqa: E402
    copy_or_link_file,
    ensure_all_episodes_are_standard,
    list_episode_dirs,
    load_json_dict,
    validate_episode_or_raise,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Merge two standard rosbag sessions (episode.bag + metadata.json) "
            "into one standard session."
        )
    )
    parser.add_argument(
        "--demo-session-dir",
        required=True,
        help="Input standard demo session directory.",
    )
    parser.add_argument(
        "--teleop-session-dir",
        required=True,
        help="Input standard teleop session directory.",
    )
    parser.add_argument(
        "--output-root",
        required=True,
        help="Output merged standard session directory.",
    )
    parser.add_argument(
        "--order",
        choices=("demo_first", "teleop_first"),
        default="demo_first",
        help="Merge order (default: demo_first).",
    )
    parser.add_argument(
        "--copy-mode",
        choices=("copy", "symlink", "hardlink"),
        default="copy",
        help="How to place episode bag files into output (default: copy).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output directory if it exists.",
    )
    parser.add_argument(
        "--strict",
        dest="strict",
        action="store_true",
        default=True,
        help="Strict mode (default): fail when source or output is non-standard.",
    )
    parser.add_argument(
        "--no-strict",
        dest="strict",
        action="store_false",
        help="Non-strict mode: print warnings and continue when possible.",
    )
    return parser.parse_args()


def _prepare_output_root(output_root: Path, overwrite: bool) -> None:
    if output_root.exists():
        if not overwrite:
            raise RuntimeError(
                f"Output already exists: {output_root} (use --overwrite to replace)"
            )
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=False)


def _collect_standard_episodes(
    *,
    session_dir: Path,
    strict: bool,
) -> List[Path]:
    if not session_dir.is_dir():
        raise RuntimeError(f"session-dir not found: {session_dir}")
    episodes = list_episode_dirs(session_dir)
    if not episodes:
        raise RuntimeError(f"No episode_* found under {session_dir}")
    ensure_all_episodes_are_standard(episode_dirs=episodes, strict=strict)
    return episodes


def _rewrite_metadata_for_output(
    *,
    src_metadata: Dict,
    output_session_name: str,
    output_episode_name: str,
    output_episode_index: int,
    output_episode_dir: Path,
) -> Dict:
    out = dict(src_metadata)
    out["session_name"] = output_session_name
    out["episode_name"] = output_episode_name
    out["episode_index"] = int(output_episode_index)
    out["episode_dir"] = str(output_episode_dir)
    return out


def run(args: argparse.Namespace) -> int:
    demo_session_dir = Path(args.demo_session_dir).expanduser().resolve()
    teleop_session_dir = Path(args.teleop_session_dir).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()

    demo_episodes = _collect_standard_episodes(
        session_dir=demo_session_dir,
        strict=bool(args.strict),
    )
    teleop_episodes = _collect_standard_episodes(
        session_dir=teleop_session_dir,
        strict=bool(args.strict),
    )

    _prepare_output_root(output_root, overwrite=bool(args.overwrite))
    print(f"[merge] demo={demo_session_dir} episodes={len(demo_episodes)}")
    print(f"[merge] teleop={teleop_session_dir} episodes={len(teleop_episodes)}")
    print(f"[merge] output={output_root}")
    print(f"[merge] order={args.order} copy_mode={args.copy_mode} strict={bool(args.strict)}")

    ordered_sources: List[Tuple[str, Path]] = []
    if args.order == "demo_first":
        ordered_sources.extend([("demo", ep) for ep in demo_episodes])
        ordered_sources.extend([("teleop", ep) for ep in teleop_episodes])
    else:
        ordered_sources.extend([("teleop", ep) for ep in teleop_episodes])
        ordered_sources.extend([("demo", ep) for ep in demo_episodes])

    out_index = 1
    for source_tag, src_episode_dir in ordered_sources:
        output_episode_name = f"episode_{out_index:03d}"
        output_episode_dir = output_root / output_episode_name
        output_episode_dir.mkdir(parents=True, exist_ok=False)

        src_bag = src_episode_dir / "episode.bag"
        src_meta_path = src_episode_dir / "metadata.json"
        output_bag = output_episode_dir / "episode.bag"
        output_meta = output_episode_dir / "metadata.json"

        copy_or_link_file(src_bag, output_bag, copy_mode=str(args.copy_mode))
        src_meta = load_json_dict(src_meta_path)
        out_meta = _rewrite_metadata_for_output(
            src_metadata=src_meta,
            output_session_name=output_root.name,
            output_episode_name=output_episode_name,
            output_episode_index=out_index,
            output_episode_dir=output_episode_dir,
        )
        write_json(output_meta, out_meta)

        validate_episode_or_raise(
            episode_dir=output_episode_dir,
            expected_episode_name=output_episode_name,
            expected_episode_index=out_index,
            strict=bool(args.strict),
        )
        print(
            f"[merge] {source_tag}:{src_episode_dir.name} -> {output_episode_name}"
        )
        out_index += 1

    merged_count = out_index - 1
    print(f"[merge] done episodes={merged_count}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"[merge] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
