#!/usr/bin/env python3
"""Run a small burst-parameter sweep and generate side-by-side visual reports."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

import pandas as pd


KNOB_SPECS: Dict[str, Dict[str, Any]] = {
    "burst_energy_rho": {"flag": "--burst-energy-rho", "abbr": "ber", "type": float},
    "burst_energy_left": {"flag": "--burst-energy-left", "abbr": "bel", "type": float},
    "burst_energy_right": {"flag": "--burst-energy-right", "abbr": "brt", "type": float},
    "follow_rest_lookahead": {"flag": "--follow-rest-lookahead", "abbr": "fla", "type": float},
    "follow_rest_frames": {"flag": "--follow-rest-frames", "abbr": "ffr", "type": int},
    "follow_rest_speed_scale": {"flag": "--follow-rest-speed-scale", "abbr": "fss", "type": float},
    "windup_rest_speed_scale": {"flag": "--windup-rest-speed-scale", "abbr": "wss", "type": float},
    "windup_rest_frames": {"flag": "--windup-rest-frames", "abbr": "wfr", "type": int},
    "windup_rest_lookback": {"flag": "--windup-rest-lookback", "abbr": "wlb", "type": float},
    "windup_rest_min_gap": {"flag": "--windup-rest-min-gap", "abbr": "wmg", "type": float},
}

DEFAULT_GRID: Dict[str, List[Any]] = {
    "burst_energy_rho": [0.88, 0.90, 0.93],
    "burst_energy_left": [1.2, 1.5],
    "burst_energy_right": [1.2, 1.5],
    "windup_rest_speed_scale": [0.7, 0.8],
    "windup_rest_frames": [8, 12],
    "follow_rest_lookahead": [2.0],
    "follow_rest_frames": [8],
    "follow_rest_speed_scale": [1.3],
}

KEY_EPISODES = ["episode_3", "episode_13", "episode_14", "episode_19", "episode_33", "episode_5"]

# Baseline tuned profile (mirrors recent tunedL runbook parameters).
BASELINE_BATCH_ARGS: List[str] = [
    "--detection-mode",
    "event_centered",
    "--threshold-mode",
    "percentile",
    "--merge-gap",
    "0.35",
    "--merge-mode",
    "gap",
    "--hole-min-norm",
    "0.5",
    "--hole-quantile",
    "0.0",
    "--peak-min-norm",
    "0.0",
    "--peak-min-raw",
    "0.0",
    "--score-norm-base",
    "p90",
    "--score-norm-eps",
    "1e-6",
    "--dtau-weight",
    "0.2",
    "--event-speed-alpha",
    "0.25",
    "--event-below-frames",
    "7",
    "--snap-peak-pct",
    "85.0",
    "--snap-peak-min-score",
    "0.0",
    "--snap-min-speed-norm",
    "0.6",
    "--snap-min-gap",
    "0.15",
    "--snap-peak-search",
    "0.45",
    "--snap-time-mode",
    "speed_peak",
    "--snap-speed-peak-mode",
    "energy",
    "--snap-speed-peak-window",
    "0.35",
    "--burst-energy-rho",
    "0.90",
    "--burst-energy-left",
    "1.5",
    "--burst-energy-right",
    "1.5",
    "--shape-dct-k",
    "4",
    "--event-keep-topk",
    "1",
    "--event-rank-by",
    "peak_speed",
    "--event-force-primary",
    "--event-attach-gap",
    "0.18",
    "--event-attach-peak-ratio",
    "0.62",
    "--windup-rest-frames",
    "8",
    "--windup-rest-speed-scale",
    "1.3",
    "--windup-rest-lookback",
    "2.5",
    "--windup-rest-min-gap",
    "0.15",
    "--follow-rest-min-gap",
    "0.15",
    "--uniform-speed-axis",
]


@dataclass(frozen=True)
class SweepConfig:
    cfg_id: int
    shortname: str
    params: Dict[str, Any]

    @property
    def outdir_name(self) -> str:
        return f"cfg_{self.cfg_id:02d}_{self.shortname}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True, help="Directory containing .bag files.")
    parser.add_argument("--base-out-dir", required=True, help="Sweep output directory.")
    parser.add_argument(
        "--episodes",
        default="",
        help="Optional comma-separated bag stems to include (e.g. episode_3,episode_13).",
    )
    parser.add_argument(
        "--grid-json",
        default="",
        help="Optional JSON file path or JSON string describing the sweep grid.",
    )
    return parser.parse_args()


def _load_json_spec(raw: str) -> Any:
    path = Path(raw).expanduser()
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return json.loads(raw)


def _token(value: float) -> str:
    text = f"{value:g}"
    return text.replace("-", "m").replace(".", "p")


def _cast_knob_value(key: str, value: Any) -> Any:
    spec = KNOB_SPECS[key]
    cast = spec["type"]
    return cast(value)


def _shortname_from_params(params: Dict[str, Any]) -> str:
    parts: List[str] = []
    for key in sorted(params.keys()):
        spec = KNOB_SPECS[key]
        value = params[key]
        if spec["type"] is int:
            token = str(int(value))
        else:
            token = _token(float(value))
        parts.append(f"{spec['abbr']}{token}")
    return "_".join(parts)


def _parse_episodes_filter(episodes_arg: str) -> Optional[set[str]]:
    names = [x.strip() for x in episodes_arg.split(",") if x.strip()]
    if not names:
        return None
    return {Path(name).stem if name.endswith(".bag") else name for name in names}


def _discover_bag_stems(input_dir: Path, allowed: Optional[set[str]]) -> List[str]:
    stems = sorted({p.stem for p in input_dir.rglob("*.bag") if p.is_file()})
    if allowed is None:
        return stems
    return [stem for stem in stems if stem in allowed]


def expand_grid(grid_json: str) -> List[SweepConfig]:
    if grid_json:
        spec = _load_json_spec(grid_json)
    else:
        spec = DEFAULT_GRID

    configs: List[SweepConfig] = []
    if isinstance(spec, dict):
        keys = list(spec.keys())
        if not keys:
            raise ValueError("grid object is empty")
        for key in keys:
            if key not in KNOB_SPECS:
                raise ValueError(f"unsupported grid key: {key}")
            if not isinstance(spec[key], list) or not spec[key]:
                raise ValueError(f"grid key {key} must be a non-empty list")
        idx = 1
        value_lists = [spec[key] for key in keys]
        for combo in itertools.product(*value_lists):
            params = {key: _cast_knob_value(key, value) for key, value in zip(keys, combo)}
            cfg = SweepConfig(
                cfg_id=idx,
                shortname=_shortname_from_params(params),
                params=params,
            )
            configs.append(cfg)
            idx += 1
    elif isinstance(spec, list):
        idx = 1
        for entry in spec:
            if not isinstance(entry, dict):
                raise ValueError("grid list entries must be objects")
            if not entry:
                raise ValueError("grid entry object cannot be empty")
            for key in entry.keys():
                if key not in KNOB_SPECS:
                    raise ValueError(f"unsupported grid key: {key}")
            params = {key: _cast_knob_value(key, value) for key, value in entry.items()}
            cfg = SweepConfig(
                cfg_id=idx,
                shortname=_shortname_from_params(params),
                params=params,
            )
            configs.append(cfg)
            idx += 1
    else:
        raise ValueError("grid-json must resolve to an object or list")

    if not configs:
        raise ValueError("sweep config list is empty")
    return configs


def _read_all_episode_rows(csv_dir: Path) -> pd.DataFrame:
    if not csv_dir.is_dir():
        return pd.DataFrame()
    frames: List[pd.DataFrame] = []
    for csv_path in sorted(csv_dir.glob("*.csv")):
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        if df.empty:
            continue
        frames.append(df)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def compute_cfg_metrics(cfg: SweepConfig, cfg_out_dir: Path) -> Dict[str, Any]:
    per_burst = _read_all_episode_rows(cfg_out_dir / "csv")
    cfg_fields: Dict[str, Any] = {key: cfg.params.get(key) for key in sorted(cfg.params.keys())}
    if per_burst.empty:
        return {
            "cfg_id": cfg.cfg_id,
            "shortname": cfg.shortname,
            "cfg_outdir": str(cfg_out_dir),
            **cfg_fields,
            "num_bursts": 0,
            "mean_windup_duration_s": 0.0,
            "median_windup_duration_s": 0.0,
            "mean_burst_duration_s": 0.0,
            "median_burst_duration_s": 0.0,
            "mean_follow_duration_s": 0.0,
            "median_follow_duration_s": 0.0,
            "mean_burst_energy_share": 0.0,
            "median_burst_energy_share": 0.0,
            "mean_context_energy_ratio": 1.0,
            "median_context_energy_ratio": 1.0,
            "mean_energy_density_contrast": 0.0,
            "median_energy_density_contrast": 0.0,
            "ratio_windup_lt_0p1": 1.0,
            "ratio_follow_lt_0p1": 1.0,
            "ratio_burst_lt_0p1": 1.0,
            "ratio_burst_gt_2p5": 0.0,
            "ratio_follow_gt_2p5": 0.0,
            "mean_peak_speed": 0.0,
            "max_peak_speed": 0.0,
            "rank_score_semantic": 9.6,
            "status": "ok_empty",
        }

    windup_dur = per_burst["windup_duration_s"].astype(float)
    burst_dur = per_burst["burst_duration_s"].astype(float)
    follow_dur = per_burst["follow_duration_s"].astype(float)
    burst_share = per_burst["energy_burst_share"].astype(float)
    context_ratio = per_burst["context_energy_ratio"].astype(float)
    contrast = per_burst["energy_density_contrast"].astype(float)
    peak_speed = per_burst["peak_speed"].astype(float)

    # Keep compatibility with older pandas versions available in ROS1 environments.
    contrast_clean = contrast[contrast.map(lambda x: math.isfinite(float(x)))]
    if contrast_clean.empty:
        mean_contrast = 0.0
        median_contrast = 0.0
    else:
        mean_contrast = float(contrast_clean.mean())
        median_contrast = float(contrast_clean.median())

    ratio_windup_short = float((windup_dur < 0.1).mean())
    ratio_follow_short = float((follow_dur < 0.1).mean())
    ratio_follow_long = float((follow_dur > 2.5).mean())
    ratio_burst_short = float((burst_dur < 0.1).mean())
    ratio_burst_long = float((burst_dur > 2.5).mean())
    median_burst_share = float(burst_share.median())
    mean_context_ratio = float(context_ratio.mean())

    if median_contrast > 0.0:
        log_term = math.log10(median_contrast)
    else:
        log_term = -1.0
    log_term = float(max(-1.0, min(1.0, log_term)))
    rank_score_semantic = (
        4.0 * (1.0 - median_burst_share)
        + 3.0 * mean_context_ratio
        + 1.5 * ratio_windup_short
        + 1.5 * ratio_follow_short
        + 1.0 * ratio_burst_short
        + 0.6 * ratio_burst_long
        - 0.5 * log_term
    )
    return {
        "cfg_id": cfg.cfg_id,
        "shortname": cfg.shortname,
        "cfg_outdir": str(cfg_out_dir),
        **cfg_fields,
        "num_bursts": int(per_burst.shape[0]),
        "mean_windup_duration_s": float(windup_dur.mean()),
        "median_windup_duration_s": float(windup_dur.median()),
        "mean_burst_duration_s": float(burst_dur.mean()),
        "median_burst_duration_s": float(burst_dur.median()),
        "mean_follow_duration_s": float(follow_dur.mean()),
        "median_follow_duration_s": float(follow_dur.median()),
        "mean_burst_energy_share": float(burst_share.mean()),
        "median_burst_energy_share": median_burst_share,
        "mean_context_energy_ratio": mean_context_ratio,
        "median_context_energy_ratio": float(context_ratio.median()),
        "mean_energy_density_contrast": mean_contrast,
        "median_energy_density_contrast": median_contrast,
        "ratio_windup_lt_0p1": ratio_windup_short,
        "ratio_follow_lt_0p1": ratio_follow_short,
        "ratio_burst_lt_0p1": ratio_burst_short,
        "ratio_burst_gt_2p5": ratio_burst_long,
        "ratio_follow_gt_2p5": ratio_follow_long,
        "mean_peak_speed": float(peak_speed.mean()),
        "max_peak_speed": float(peak_speed.max()),
        "rank_score_semantic": float(rank_score_semantic),
        "status": "ok",
    }


def run_single_config(
    cfg: SweepConfig,
    input_dir: Path,
    base_out_dir: Path,
    episodes_arg: str,
) -> Dict[str, Any]:
    tools_dir = Path(__file__).resolve().parent
    batch_script = tools_dir / "burst_batch_ros1.py"
    cfg_out_dir = base_out_dir / cfg.outdir_name
    cfg_out_dir.mkdir(parents=True, exist_ok=True)
    cmd: List[str] = [
        sys.executable,
        str(batch_script),
        "--input-dir",
        str(input_dir),
        "--out-dir",
        str(cfg_out_dir),
    ]
    cmd.extend(BASELINE_BATCH_ARGS)
    for key in sorted(cfg.params.keys()):
        spec = KNOB_SPECS[key]
        value = cfg.params[key]
        if spec["type"] is int:
            value_str = str(int(value))
        else:
            value_str = f"{float(value):g}"
        cmd.extend([str(spec["flag"]), value_str])
    if episodes_arg:
        cmd.extend(["--episodes", episodes_arg])

    param_desc = ", ".join(f"{k}={cfg.params[k]}" for k in sorted(cfg.params.keys()))
    print(f"[sweep] cfg={cfg.cfg_id:02d} {cfg.shortname} ({param_desc})")
    proc = subprocess.run(
        cmd,
        cwd=str(Path(__file__).resolve().parents[1]),
        text=True,
        capture_output=True,
        check=False,
    )
    log_path = cfg_out_dir / "run.log"
    with log_path.open("w", encoding="utf-8") as f:
        f.write("CMD:\n")
        f.write(" ".join(cmd))
        f.write("\n\nSTDOUT:\n")
        f.write(proc.stdout)
        f.write("\nSTDERR:\n")
        f.write(proc.stderr)

    if proc.returncode != 0:
        tail = "\n".join((proc.stderr or proc.stdout).splitlines()[-20:])
        print(f"[sweep] cfg={cfg.cfg_id:02d} failed, see {log_path}")
        return {
            "cfg_id": cfg.cfg_id,
            "shortname": cfg.shortname,
            "cfg_outdir": str(cfg_out_dir),
            **{key: cfg.params.get(key) for key in sorted(cfg.params.keys())},
            "status": "failed",
            "error_tail": tail,
        }

    metrics = compute_cfg_metrics(cfg=cfg, cfg_out_dir=cfg_out_dir)
    print(
        f"[sweep] cfg={cfg.cfg_id:02d} done: bursts={metrics['num_bursts']} "
        f"burst_share_med={metrics['median_burst_energy_share']:.3f} "
        f"context_ratio={metrics['mean_context_energy_ratio']:.3f} "
        f"windup<0.1={metrics['ratio_windup_lt_0p1']:.3f} "
        f"follow<0.1={metrics['ratio_follow_lt_0p1']:.3f} "
        f"burst>2.5={metrics['ratio_burst_gt_2p5']:.3f}"
    )
    return metrics


def choose_report_episodes(available_stems: Sequence[str]) -> List[str]:
    available = set(available_stems)
    selected = [ep for ep in KEY_EPISODES if ep in available]
    if selected:
        return selected
    return list(available_stems[:6])


def collect_available_episode_names(result_rows: Sequence[Dict[str, Any]]) -> List[str]:
    for row in result_rows:
        if row.get("status") != "ok":
            continue
        summary_path = Path(str(row["cfg_outdir"])) / "summary.csv"
        if not summary_path.is_file():
            continue
        try:
            summary_df = pd.read_csv(summary_path)
        except Exception:
            continue
        if "episode" not in summary_df.columns:
            continue
        names = [str(x) for x in summary_df["episode"].dropna().tolist()]
        if names:
            return names
    return []


def write_html_report(
    base_out_dir: Path,
    cfgs: Sequence[SweepConfig],
    report_episodes: Sequence[str],
) -> None:
    report_dir = base_out_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)
    out_html = report_dir / "index.html"

    lines: List[str] = []
    lines.append("<!doctype html>")
    lines.append("<html><head><meta charset='utf-8'><title>Burst Sweep Report</title>")
    lines.append(
        "<style>"
        "body{font-family:Arial,sans-serif;margin:16px;background:#f7f7f7;color:#222;}"
        "h1,h2{margin:10px 0;}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:10px;}"
        ".card{background:#fff;border:1px solid #ddd;border-radius:8px;padding:8px;}"
        ".meta{font-size:12px;color:#555;margin-bottom:6px;}"
        "img{width:100%;height:auto;border:1px solid #eee;background:#fafafa;}"
        ".missing{height:140px;display:flex;align-items:center;justify-content:center;background:#fafafa;border:1px dashed #ccc;color:#777;font-size:12px;}"
        "</style>"
    )
    lines.append("</head><body>")
    lines.append("<h1>Burst Sweep Comparison</h1>")
    lines.append(f"<p>Configs: {len(cfgs)} | Episodes shown: {len(report_episodes)}</p>")

    for episode in report_episodes:
        lines.append(f"<h2>{escape(episode)}</h2>")
        lines.append("<div class='grid'>")
        for cfg in cfgs:
            img_path = base_out_dir / cfg.outdir_name / "plots" / f"{episode}.png"
            lines.append("<div class='card'>")
            lines.append(f"<div class='meta'>cfg_{cfg.cfg_id:02d} {escape(cfg.shortname)}</div>")
            if img_path.is_file():
                rel = img_path.relative_to(base_out_dir)
                src = f"../{rel.as_posix()}"
                lines.append(f"<a href='{escape(src)}' target='_blank'><img src='{escape(src)}' alt='{escape(episode)}'></a>")
            else:
                lines.append("<div class='missing'>missing plot</div>")
            lines.append("</div>")
        lines.append("</div>")

    lines.append("</body></html>")
    out_html.write_text("\n".join(lines), encoding="utf-8")


def rank_top_configs(rows: Iterable[Dict[str, Any]], topk: int = 3) -> List[Dict[str, Any]]:
    ok_rows = [row for row in rows if row.get("status") == "ok"]
    ok_rows.sort(
        key=lambda row: (
            float(row["rank_score_semantic"]),
            -float(row["median_burst_energy_share"]),
            float(row["mean_context_energy_ratio"]),
            float(row["ratio_windup_lt_0p1"]),
            float(row["ratio_follow_lt_0p1"]),
            float(row["ratio_burst_lt_0p1"]),
            float(row["ratio_burst_gt_2p5"]),
        )
    )
    return ok_rows[:topk]


def main() -> int:
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    base_out_dir = Path(args.base_out_dir).expanduser()
    base_out_dir.mkdir(parents=True, exist_ok=True)

    if not input_dir.is_dir():
        raise SystemExit(f"Input directory not found: {input_dir}")

    allowed_episodes = _parse_episodes_filter(str(args.episodes or ""))
    available_stems = _discover_bag_stems(input_dir=input_dir, allowed=allowed_episodes)
    if not available_stems:
        raise SystemExit("No matching .bag episodes found for sweep.")

    cfgs = expand_grid(grid_json=str(args.grid_json or ""))
    print(f"[sweep] input={input_dir} episodes={len(available_stems)} configs={len(cfgs)}")

    result_rows: List[Dict[str, Any]] = []
    for cfg in cfgs:
        row = run_single_config(
            cfg=cfg,
            input_dir=input_dir,
            base_out_dir=base_out_dir,
            episodes_arg=str(args.episodes or ""),
        )
        result_rows.append(row)

    sweep_summary_path = base_out_dir / "sweep_summary.csv"
    pd.DataFrame(result_rows).to_csv(sweep_summary_path, index=False)
    print(f"[sweep] wrote summary: {sweep_summary_path}")

    write_html_report(
        base_out_dir=base_out_dir,
        cfgs=cfgs,
        report_episodes=choose_report_episodes(
            available_stems=(
                collect_available_episode_names(result_rows=result_rows) or available_stems
            )
        ),
    )
    print(f"[sweep] wrote report: {base_out_dir / 'report' / 'index.html'}")

    top3 = rank_top_configs(result_rows, topk=3)
    if top3:
        print("Top-3 configs (semantic-separation ranking):")
        for rank, row in enumerate(top3, start=1):
            print(
                f"{rank}. cfg_{int(row['cfg_id']):02d}_{row['shortname']} "
                f"rank={float(row['rank_score_semantic']):.3f} "
                f"burst_share_med={float(row['median_burst_energy_share']):.3f} "
                f"context_ratio={float(row['mean_context_energy_ratio']):.3f} "
                f"windup<0.1={float(row['ratio_windup_lt_0p1']):.3f} "
                f"follow<0.1={float(row['ratio_follow_lt_0p1']):.3f} "
                f"burst<0.1={float(row['ratio_burst_lt_0p1']):.3f} "
                f"burst>2.5={float(row['ratio_burst_gt_2p5']):.3f} "
                f"outdir={row['cfg_outdir']}"
            )
    else:
        print("Top-3 configs: no successful configs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
