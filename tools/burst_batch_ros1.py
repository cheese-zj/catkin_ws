#!/usr/bin/env python3
"""Batch burst detection over many ROS1 bag episodes."""

from __future__ import annotations

import argparse
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd

import burst_detect_ros1 as detector


SUMMARY_COLUMNS = [
    "episode",
    "num_bursts",
    "total_burst_time",
    "mean_peak_speed",
    "max_peak_speed",
    "mean_duration",
    "threshold_mode",
    "threshold_signal_space",
    "norm_base",
    "norm_scale",
    "norm_eps",
    "on_th_signal",
    "off_th_signal",
    "on_th_speed_equiv",
    "off_th_speed_equiv",
    "merge_gap_s",
    "merge_mode",
    "hole_min_norm",
    "peak_min_norm",
    "peak_min_raw",
    "peak_filter_norm_base",
    "peak_filter_norm_scale",
    "peak_filter_norm_eps",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        required=True,
        help="Directory to scan recursively for *.bag episodes.",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="Batch output root.",
    )
    parser.add_argument(
        "--joint-topic",
        default="",
        help="JointState topic override. If omitted, auto-pick per bag.",
    )
    parser.add_argument("--smooth-win", type=int, default=1, help="Moving-average window (samples).")
    parser.add_argument("--on-pct", type=float, default=95.0, help="Burst enter percentile.")
    parser.add_argument("--off-pct", type=float, default=80.0, help="Burst exit percentile.")
    parser.add_argument("--min-dur", type=float, default=0.08, help="Minimum burst duration in seconds.")
    parser.add_argument("--pad", type=float, default=0.0, help="Burst padding in seconds.")
    parser.add_argument(
        "--merge-gap",
        type=float,
        default=0.12,
        help="Merge consecutive bursts when gap < merge-gap (seconds).",
    )
    parser.add_argument(
        "--merge-mode",
        choices=detector.MERGE_MODES,
        default="gap",
        help="Merge strategy: gap-only or fill_holes with normalized-speed condition.",
    )
    parser.add_argument(
        "--hole-min-norm",
        type=float,
        default=0.5,
        help="For merge-mode=fill_holes, require v_norm in the hole to stay >= this value.",
    )
    parser.add_argument(
        "--threshold-mode",
        choices=detector.THRESHOLD_MODES,
        default="percentile",
        help="Threshold mode for burst detection.",
    )
    parser.add_argument(
        "--norm-base",
        choices=detector.NORM_BASES,
        default="p90",
        help="Normalization base for norm_* modes.",
    )
    parser.add_argument(
        "--norm-eps",
        type=float,
        default=1e-6,
        help="Epsilon used in normalized speed denominator.",
    )
    parser.add_argument(
        "--on-abs",
        type=float,
        default=2.0,
        help="Fixed ON threshold in normalized signal space for norm_fixed mode.",
    )
    parser.add_argument(
        "--off-abs",
        type=float,
        default=1.3,
        help="Fixed OFF threshold in normalized signal space for norm_fixed mode.",
    )
    parser.add_argument(
        "--global-stats",
        default="",
        help="Global threshold JSON file used by global_percentile mode.",
    )
    parser.add_argument(
        "--peak-min-norm",
        type=float,
        default=0.0,
        help="Keep burst only if max(v_norm) in window >= this value. 0 disables.",
    )
    parser.add_argument(
        "--peak-min-raw",
        type=float,
        default=0.0,
        help="Keep burst only if max(v_raw) in window >= this value. 0 disables.",
    )
    parser.add_argument(
        "--speed-ymin",
        type=float,
        default=None,
        help="Optional fixed ymin for speed axis in each timeseries plot.",
    )
    parser.add_argument(
        "--speed-ymax",
        type=float,
        default=None,
        help="Optional fixed ymax for speed axis in each timeseries plot.",
    )
    parser.add_argument(
        "--uniform-speed-axis",
        action="store_true",
        help=(
            "Pre-scan all bags to auto-select one shared speed y-axis for every plot. "
            "If --speed-ymax is provided, it is used directly."
        ),
    )
    return parser.parse_args()


def discover_bags(input_dir: Path) -> List[Path]:
    return sorted(p for p in input_dir.rglob("*.bag") if p.is_file())


def make_episode_name(bag_path: Path, seen: Dict[str, int]) -> str:
    stem = bag_path.stem
    count = seen.get(stem, 0)
    seen[stem] = count + 1
    if count == 0:
        return stem
    return f"{stem}_{count}"


def write_errors_log(errors_log: Path, error_blocks: List[str]) -> None:
    with errors_log.open("w", encoding="utf-8") as f:
        for block in error_blocks:
            f.write(block.rstrip())
            f.write("\n")
            f.write("=" * 80)
            f.write("\n")


def resolve_speed_axis(
    bag_paths: List[Path],
    joint_topic: Optional[str],
    smooth_win: int,
    speed_ymin: Optional[float],
    speed_ymax: Optional[float],
    uniform_speed_axis: bool,
) -> Tuple[Optional[float], Optional[float]]:
    detector.validate_speed_axis(speed_ymin=speed_ymin, speed_ymax=speed_ymax)

    if not uniform_speed_axis:
        return speed_ymin, speed_ymax

    resolved_ymin = speed_ymin if speed_ymin is not None else 0.0
    if speed_ymax is not None:
        detector.validate_speed_axis(speed_ymin=resolved_ymin, speed_ymax=speed_ymax)
        print(f"Using uniform speed axis: ymin={resolved_ymin:.6f} ymax={speed_ymax:.6f}")
        return resolved_ymin, speed_ymax

    global_max: Optional[float] = None
    total = len(bag_paths)
    for idx, bag_path in enumerate(bag_paths, start=1):
        try:
            _min_speed, max_speed = detector.scan_episode_speed_range(
                bag_path=bag_path,
                joint_topic=joint_topic,
                smooth_win=smooth_win,
            )
            if global_max is None or max_speed > global_max:
                global_max = max_speed
        except Exception as exc:
            print(f"[pre-scan {idx}/{total}] WARN {bag_path}: {exc}")

    if global_max is None:
        raise SystemExit("Failed to estimate uniform speed axis: no valid bag speed signal found.")

    margin = max(1e-6, 0.05 * global_max)
    resolved_ymax = global_max + margin
    detector.validate_speed_axis(speed_ymin=resolved_ymin, speed_ymax=resolved_ymax)
    print(f"Using uniform speed axis: ymin={resolved_ymin:.6f} ymax={resolved_ymax:.6f}")
    return resolved_ymin, resolved_ymax


def main() -> int:
    args = parse_args()

    input_dir = Path(args.input_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser()

    if not input_dir.is_dir():
        raise SystemExit(f"Input directory not found: {input_dir}")

    try:
        detector.validate_knobs(
            smooth_win=int(args.smooth_win),
            on_pct=float(args.on_pct),
            off_pct=float(args.off_pct),
            min_dur=float(args.min_dur),
            pad=float(args.pad),
        )
        detector.validate_speed_axis(
            speed_ymin=(None if args.speed_ymin is None else float(args.speed_ymin)),
            speed_ymax=(None if args.speed_ymax is None else float(args.speed_ymax)),
        )
        detector.validate_threshold_options(
            threshold_mode=str(args.threshold_mode),
            norm_base=str(args.norm_base),
            norm_eps=float(args.norm_eps),
            on_pct=float(args.on_pct),
            off_pct=float(args.off_pct),
            on_abs=float(args.on_abs),
            off_abs=float(args.off_abs),
            global_stats=(args.global_stats or None),
            merge_gap=float(args.merge_gap),
            merge_mode=str(args.merge_mode),
            hole_min_norm=float(args.hole_min_norm),
            peak_min_norm=float(args.peak_min_norm),
            peak_min_raw=float(args.peak_min_raw),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    bag_paths = discover_bags(input_dir)
    if not bag_paths:
        raise SystemExit(f"No .bag files found under: {input_dir}")

    out_dir.mkdir(parents=True, exist_ok=True)
    resolved_speed_ymin, resolved_speed_ymax = resolve_speed_axis(
        bag_paths=bag_paths,
        joint_topic=(args.joint_topic or None),
        smooth_win=int(args.smooth_win),
        speed_ymin=(None if args.speed_ymin is None else float(args.speed_ymin)),
        speed_ymax=(None if args.speed_ymax is None else float(args.speed_ymax)),
        uniform_speed_axis=bool(args.uniform_speed_axis),
    )

    seen_stems: Dict[str, int] = {}
    summary_rows = []
    error_blocks: List[str] = []

    total = len(bag_paths)
    for idx, bag_path in enumerate(bag_paths, start=1):
        episode_name = make_episode_name(bag_path=bag_path, seen=seen_stems)
        csv_out = out_dir / "csv" / f"{episode_name}.csv"
        plot_out = out_dir / "plots" / f"{episode_name}.png"

        try:
            result = detector.run_episode(
                bag_path=bag_path,
                outdir=out_dir,
                episode_name=episode_name,
                csv_out_path=csv_out,
                plot_out_path=plot_out,
                joint_topic=(args.joint_topic or None),
                smooth_win=int(args.smooth_win),
                on_pct=float(args.on_pct),
                off_pct=float(args.off_pct),
                min_dur=float(args.min_dur),
                pad=float(args.pad),
                speed_ymin=resolved_speed_ymin,
                speed_ymax=resolved_speed_ymax,
                threshold_mode=str(args.threshold_mode),
                norm_base=str(args.norm_base),
                norm_eps=float(args.norm_eps),
                on_abs=float(args.on_abs),
                off_abs=float(args.off_abs),
                global_stats=(args.global_stats or None),
                merge_gap=float(args.merge_gap),
                merge_mode=str(args.merge_mode),
                hole_min_norm=float(args.hole_min_norm),
                peak_min_norm=float(args.peak_min_norm),
                peak_min_raw=float(args.peak_min_raw),
                print_summary=False,
            )
            summary_rows.append(
                {
                    "episode": episode_name,
                    "num_bursts": result.num_bursts,
                    "total_burst_time": result.total_burst_time,
                    "mean_peak_speed": result.mean_peak_speed,
                    "max_peak_speed": result.max_peak_speed,
                    "mean_duration": result.mean_duration,
                    "threshold_mode": result.threshold_mode,
                    "threshold_signal_space": result.threshold_signal_space,
                    "norm_base": result.norm_base,
                    "norm_scale": result.norm_scale,
                    "norm_eps": result.norm_eps,
                    "on_th_signal": result.on_th,
                    "off_th_signal": result.off_th,
                    "on_th_speed_equiv": result.on_th_speed_equiv,
                    "off_th_speed_equiv": result.off_th_speed_equiv,
                    "merge_gap_s": result.merge_gap_s,
                    "merge_mode": result.merge_mode,
                    "hole_min_norm": result.hole_min_norm,
                    "peak_min_norm": result.peak_min_norm,
                    "peak_min_raw": result.peak_min_raw,
                    "peak_filter_norm_base": result.peak_filter_norm_base,
                    "peak_filter_norm_scale": result.peak_filter_norm_scale,
                    "peak_filter_norm_eps": result.peak_filter_norm_eps,
                }
            )
            print(
                f"[{idx}/{total}] OK {bag_path} -> csv:{csv_out.name} plot:{plot_out.name} "
                f"bursts={result.num_bursts} mode={result.threshold_mode}"
            )
        except Exception as exc:
            tb = traceback.format_exc()
            error_blocks.append(f"{bag_path}\n{exc}\n{tb}")
            print(f"[{idx}/{total}] ERROR {bag_path}: {exc}")

    summary_df = pd.DataFrame(summary_rows, columns=SUMMARY_COLUMNS)
    summary_path = out_dir / "summary.csv"
    summary_df.to_csv(summary_path, index=False)

    if error_blocks:
        errors_log = out_dir / "errors.log"
        write_errors_log(errors_log=errors_log, error_blocks=error_blocks)
        print(
            f"Finished {total} bags. Success={len(summary_rows)} Failed={len(error_blocks)}. "
            f"See {errors_log}"
        )
    else:
        print(f"Finished {total} bags. Success={len(summary_rows)} Failed=0.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
