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
    "mean_windup_duration_s",
    "mean_burst_duration_s",
    "mean_follow_duration_s",
    "mean_burst_energy_share",
    "median_burst_energy_share",
    "mean_context_energy_ratio",
    "median_context_energy_ratio",
    "mean_energy_density_contrast",
    "median_energy_density_contrast",
    "detection_mode",
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
    "hole_quantile",
    "peak_min_norm",
    "peak_min_raw",
    "peak_filter_norm_base",
    "peak_filter_norm_scale",
    "peak_filter_norm_eps",
    "score_norm_base",
    "score_norm_eps",
    "dtau_weight",
    "event_speed_alpha",
    "event_below_frames",
    "snap_peak_pct",
    "snap_peak_min_score",
    "snap_min_speed_norm",
    "snap_min_gap_s",
    "snap_peak_search_s",
    "snap_time_mode",
    "snap_speed_peak_mode",
    "snap_speed_peak_window_s",
    "burst_energy_rho",
    "burst_energy_left_s",
    "burst_energy_right_s",
    "shape_dct_k",
    "event_keep_topk",
    "event_rank_by",
    "event_force_primary",
    "event_attach_gap_s",
    "event_attach_peak_ratio",
    "windup_rest_frames",
    "windup_rest_speed_scale",
    "windup_rest_lookback_s",
    "windup_rest_min_gap_s",
    "follow_rest_frames",
    "follow_rest_speed_scale",
    "follow_rest_lookahead_s",
    "follow_rest_min_gap_s",
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
    parser.add_argument(
        "--episodes",
        default="",
        help="Optional comma-separated bag stems to run (e.g. episode_3,episode_13).",
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
        "--hole-quantile",
        type=float,
        default=0.0,
        help=(
            "For merge-mode=fill_holes, merge when quantile(hole v_norm, q) >= hole-min-norm. "
            "q=0 keeps old strict behavior (min value). q=0.1 allows small dips."
        ),
    )
    parser.add_argument(
        "--threshold-mode",
        choices=detector.THRESHOLD_MODES,
        default="percentile",
        help="Threshold mode for burst detection.",
    )
    parser.add_argument(
        "--detection-mode",
        choices=detector.DETECTION_MODES,
        default="event_centered",
        help="Burst detector: hysteresis (legacy) or event_centered (snap-driven).",
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
        "--score-norm-base",
        choices=detector.NORM_BASES,
        default="p90",
        help="Normalization base for burst_score terms (acc, dtau).",
    )
    parser.add_argument(
        "--score-norm-eps",
        type=float,
        default=1e-6,
        help="Epsilon used in burst_score normalization denominator.",
    )
    parser.add_argument(
        "--dtau-weight",
        type=float,
        default=0.5,
        help="Weight λ in burst_score = acc_norm + λ * dtau_norm.",
    )
    parser.add_argument(
        "--event-speed-alpha",
        type=float,
        default=0.4,
        help="Relative speed threshold alpha for event window expansion around local speed peak.",
    )
    parser.add_argument(
        "--event-below-frames",
        type=int,
        default=3,
        help="Require this many consecutive frames below alpha*peak speed to close event boundaries.",
    )
    parser.add_argument(
        "--snap-peak-pct",
        type=float,
        default=85.0,
        help="Percentile threshold on burst_score for snap peak candidates.",
    )
    parser.add_argument(
        "--snap-peak-min-score",
        type=float,
        default=0.0,
        help="Absolute minimum burst_score for snap peak candidates.",
    )
    parser.add_argument(
        "--snap-min-speed-norm",
        type=float,
        default=0.6,
        help="Require normalized speed >= this value at snap candidates.",
    )
    parser.add_argument(
        "--snap-min-gap",
        type=float,
        default=0.15,
        help="Minimum time gap between retained snap peaks (seconds).",
    )
    parser.add_argument(
        "--snap-peak-search",
        type=float,
        default=0.2,
        help="Search radius (seconds) around snap for local speed peak.",
    )
    parser.add_argument(
        "--snap-time-mode",
        choices=detector.SNAP_TIME_MODES,
        default="score_peak",
        help="How to set snap_time: score_peak (burst_score peak) or speed_peak (selected speed peak).",
    )
    parser.add_argument(
        "--snap-speed-peak-mode",
        choices=detector.SNAP_SPEED_PEAK_MODES,
        default="max",
        help="How to choose speed peak near snap: max (instantaneous), ma (local mean), energy (local mean of speed^2).",
    )
    parser.add_argument(
        "--snap-speed-peak-window",
        type=float,
        default=0.2,
        help="Window size (seconds) for ma/energy local peak scoring.",
    )
    parser.add_argument(
        "--burst-energy-rho",
        type=float,
        default=0.90,
        help="Target energy coverage ratio rho for energy-burst span inside burst-energy search range.",
    )
    parser.add_argument(
        "--burst-energy-left",
        type=float,
        default=1.5,
        help="Search seconds to the left of snap_time for solving energy-burst span.",
    )
    parser.add_argument(
        "--burst-energy-right",
        type=float,
        default=1.5,
        help="Search seconds to the right of snap_time for solving energy-burst span.",
    )
    parser.add_argument(
        "--shape-dct-k",
        type=int,
        default=4,
        help="Number of non-DC DCT coefficients for z_shape.",
    )
    parser.add_argument(
        "--event-keep-topk",
        type=int,
        default=1,
        help="Keep only top-k events per episode after ranking (default=1 for single-intent prior).",
    )
    parser.add_argument(
        "--event-rank-by",
        choices=detector.EVENT_RANK_BY,
        default="peak_speed",
        help="Ranking metric used by --event-keep-topk.",
    )
    parser.add_argument(
        "--event-force-primary",
        dest="event_force_primary",
        action="store_true",
        help="Ensure at least one event by falling back to global max burst_score when needed.",
    )
    parser.add_argument(
        "--no-event-force-primary",
        dest="event_force_primary",
        action="store_false",
        help="Disable fallback-to-primary event when no event survives filtering.",
    )
    parser.set_defaults(event_force_primary=True)
    parser.add_argument(
        "--event-attach-gap",
        type=float,
        default=0.18,
        help="After selecting top event, merge neighboring strong sub-peaks when gap <= this value (seconds).",
    )
    parser.add_argument(
        "--event-attach-peak-ratio",
        type=float,
        default=0.62,
        help="Neighbor event must satisfy peak_speed >= ratio * primary_peak_speed to be attached.",
    )
    parser.add_argument(
        "--windup-rest-frames",
        type=int,
        default=0,
        help="If >0, estimate windup_start via nearest pre-snap rest segment with this many consecutive frames below threshold.",
    )
    parser.add_argument(
        "--windup-rest-speed-scale",
        type=float,
        default=1.0,
        help="Rest threshold scale: speed < scale * off_th_speed_equiv.",
    )
    parser.add_argument(
        "--windup-rest-lookback",
        type=float,
        default=2.5,
        help="Maximum seconds to look back from burst_start when searching rest segment.",
    )
    parser.add_argument(
        "--windup-rest-min-gap",
        type=float,
        default=0.0,
        help="Ignore rest segments that end too close to burst_start (seconds).",
    )
    parser.add_argument(
        "--follow-rest-frames",
        type=int,
        default=8,
        help="If >0, estimate follow_end via nearest post-snap rest segment with this many consecutive frames below threshold.",
    )
    parser.add_argument(
        "--follow-rest-speed-scale",
        type=float,
        default=1.3,
        help="Rest threshold scale for follow: speed < scale * off_th_speed_equiv.",
    )
    parser.add_argument(
        "--follow-rest-lookahead",
        type=float,
        default=2.5,
        help="Maximum seconds to look ahead from burst_end when searching follow rest segment.",
    )
    parser.add_argument(
        "--follow-rest-min-gap",
        type=float,
        default=0.15,
        help="Ignore follow rest segments that start too close to burst_end (seconds).",
    )
    parser.add_argument(
        "--plot-debug-window",
        action="store_true",
        help="Overlay event windows for debugging (hatched outline only; does not encode phase semantics).",
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


def filter_bags_by_episode_names(bag_paths: List[Path], episodes_arg: str) -> List[Path]:
    tokens = [x.strip() for x in episodes_arg.split(",") if x.strip()]
    if not tokens:
        return bag_paths

    allowed = set()
    for token in tokens:
        stem = Path(token).stem if token.endswith(".bag") else token
        if stem:
            allowed.add(stem)
    return [p for p in bag_paths if p.stem in allowed]


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
            detection_mode=str(args.detection_mode),
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
            hole_quantile=float(args.hole_quantile),
            peak_min_norm=float(args.peak_min_norm),
            peak_min_raw=float(args.peak_min_raw),
            score_norm_base=str(args.score_norm_base),
            score_norm_eps=float(args.score_norm_eps),
            dtau_weight=float(args.dtau_weight),
            event_speed_alpha=float(args.event_speed_alpha),
            event_below_frames=int(args.event_below_frames),
            snap_peak_pct=float(args.snap_peak_pct),
            snap_peak_min_score=float(args.snap_peak_min_score),
            snap_min_speed_norm=float(args.snap_min_speed_norm),
            snap_min_gap_s=float(args.snap_min_gap),
            snap_peak_search_s=float(args.snap_peak_search),
            snap_time_mode=str(args.snap_time_mode),
            snap_speed_peak_mode=str(args.snap_speed_peak_mode),
            snap_speed_peak_window_s=float(args.snap_speed_peak_window),
            burst_energy_rho=float(args.burst_energy_rho),
            burst_energy_left_s=float(args.burst_energy_left),
            burst_energy_right_s=float(args.burst_energy_right),
            shape_dct_k=int(args.shape_dct_k),
            event_keep_topk=int(args.event_keep_topk),
            event_rank_by=str(args.event_rank_by),
            event_force_primary=bool(args.event_force_primary),
            event_attach_gap_s=float(args.event_attach_gap),
            event_attach_peak_ratio=float(args.event_attach_peak_ratio),
            windup_rest_frames=int(args.windup_rest_frames),
            windup_rest_speed_scale=float(args.windup_rest_speed_scale),
            windup_rest_lookback_s=float(args.windup_rest_lookback),
            windup_rest_min_gap_s=float(args.windup_rest_min_gap),
            follow_rest_frames=int(args.follow_rest_frames),
            follow_rest_speed_scale=float(args.follow_rest_speed_scale),
            follow_rest_lookahead_s=float(args.follow_rest_lookahead),
            follow_rest_min_gap_s=float(args.follow_rest_min_gap),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    bag_paths = discover_bags(input_dir)
    bag_paths = filter_bags_by_episode_names(bag_paths=bag_paths, episodes_arg=str(args.episodes or ""))
    if not bag_paths:
        if args.episodes:
            raise SystemExit(f"No matching .bag files under {input_dir} for episodes={args.episodes!r}")
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
                detection_mode=str(args.detection_mode),
                norm_base=str(args.norm_base),
                norm_eps=float(args.norm_eps),
                on_abs=float(args.on_abs),
                off_abs=float(args.off_abs),
                global_stats=(args.global_stats or None),
                merge_gap=float(args.merge_gap),
                merge_mode=str(args.merge_mode),
                hole_min_norm=float(args.hole_min_norm),
                hole_quantile=float(args.hole_quantile),
                peak_min_norm=float(args.peak_min_norm),
                peak_min_raw=float(args.peak_min_raw),
                score_norm_base=str(args.score_norm_base),
                score_norm_eps=float(args.score_norm_eps),
                dtau_weight=float(args.dtau_weight),
                event_speed_alpha=float(args.event_speed_alpha),
                event_below_frames=int(args.event_below_frames),
                snap_peak_pct=float(args.snap_peak_pct),
                snap_peak_min_score=float(args.snap_peak_min_score),
                snap_min_speed_norm=float(args.snap_min_speed_norm),
                snap_min_gap_s=float(args.snap_min_gap),
                snap_peak_search_s=float(args.snap_peak_search),
                snap_time_mode=str(args.snap_time_mode),
                snap_speed_peak_mode=str(args.snap_speed_peak_mode),
                snap_speed_peak_window_s=float(args.snap_speed_peak_window),
                burst_energy_rho=float(args.burst_energy_rho),
                burst_energy_left_s=float(args.burst_energy_left),
                burst_energy_right_s=float(args.burst_energy_right),
                shape_dct_k=int(args.shape_dct_k),
                event_keep_topk=int(args.event_keep_topk),
                event_rank_by=str(args.event_rank_by),
                event_force_primary=bool(args.event_force_primary),
                event_attach_gap_s=float(args.event_attach_gap),
                event_attach_peak_ratio=float(args.event_attach_peak_ratio),
                windup_rest_frames=int(args.windup_rest_frames),
                windup_rest_speed_scale=float(args.windup_rest_speed_scale),
                windup_rest_lookback_s=float(args.windup_rest_lookback),
                windup_rest_min_gap_s=float(args.windup_rest_min_gap),
                follow_rest_frames=int(args.follow_rest_frames),
                follow_rest_speed_scale=float(args.follow_rest_speed_scale),
                follow_rest_lookahead_s=float(args.follow_rest_lookahead),
                follow_rest_min_gap_s=float(args.follow_rest_min_gap),
                plot_debug_window=bool(args.plot_debug_window),
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
                    "mean_windup_duration_s": result.mean_windup_duration_s,
                    "mean_burst_duration_s": result.mean_burst_duration_s,
                    "mean_follow_duration_s": result.mean_follow_duration_s,
                    "mean_burst_energy_share": result.mean_burst_energy_share,
                    "median_burst_energy_share": result.median_burst_energy_share,
                    "mean_context_energy_ratio": result.mean_context_energy_ratio,
                    "median_context_energy_ratio": result.median_context_energy_ratio,
                    "mean_energy_density_contrast": result.mean_energy_density_contrast,
                    "median_energy_density_contrast": result.median_energy_density_contrast,
                    "detection_mode": result.detection_mode,
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
                    "hole_quantile": result.hole_quantile,
                    "peak_min_norm": result.peak_min_norm,
                    "peak_min_raw": result.peak_min_raw,
                    "peak_filter_norm_base": result.peak_filter_norm_base,
                    "peak_filter_norm_scale": result.peak_filter_norm_scale,
                    "peak_filter_norm_eps": result.peak_filter_norm_eps,
                    "score_norm_base": result.score_norm_base,
                    "score_norm_eps": result.score_norm_eps,
                    "dtau_weight": result.dtau_weight,
                    "event_speed_alpha": result.event_speed_alpha,
                    "event_below_frames": result.event_below_frames,
                    "snap_peak_pct": result.snap_peak_pct,
                    "snap_peak_min_score": result.snap_peak_min_score,
                    "snap_min_speed_norm": result.snap_min_speed_norm,
                    "snap_min_gap_s": result.snap_min_gap_s,
                    "snap_peak_search_s": result.snap_peak_search_s,
                    "snap_time_mode": result.snap_time_mode,
                    "snap_speed_peak_mode": result.snap_speed_peak_mode,
                    "snap_speed_peak_window_s": result.snap_speed_peak_window_s,
                    "burst_energy_rho": result.burst_energy_rho,
                    "burst_energy_left_s": result.burst_energy_left_s,
                    "burst_energy_right_s": result.burst_energy_right_s,
                    "shape_dct_k": result.shape_dct_k,
                    "event_keep_topk": result.event_keep_topk,
                    "event_rank_by": result.event_rank_by,
                    "event_force_primary": result.event_force_primary,
                    "event_attach_gap_s": result.event_attach_gap_s,
                    "event_attach_peak_ratio": result.event_attach_peak_ratio,
                    "windup_rest_frames": result.windup_rest_frames,
                    "windup_rest_speed_scale": result.windup_rest_speed_scale,
                    "windup_rest_lookback_s": result.windup_rest_lookback_s,
                    "windup_rest_min_gap_s": result.windup_rest_min_gap_s,
                    "follow_rest_frames": result.follow_rest_frames,
                    "follow_rest_speed_scale": result.follow_rest_speed_scale,
                    "follow_rest_lookahead_s": result.follow_rest_lookahead_s,
                    "follow_rest_min_gap_s": result.follow_rest_min_gap_s,
                }
            )
            print(
                f"[{idx}/{total}] OK {bag_path} -> csv:{csv_out.name} plot:{plot_out.name} "
                f"bursts={result.num_bursts} detector={result.detection_mode} mode={result.threshold_mode}"
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
