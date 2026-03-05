List topics in one bag:
```bash
python tools/burst_detect_ros1.py --bag /data/ep01.bag --list-topics
```

Tune on 1 episode:
```bash
python tools/burst_detect_ros1.py --bag /data/ep01.bag --joint-topic /joint_states --outdir burst_out_tune
```

Batch run:
```bash
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_all --joint-topic /joint_states
```

Threshold mode examples:
```bash
# percentile (legacy)
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_pct --threshold-mode percentile --merge-gap 0.12

# event-centered (recommended): snap-driven split + 3-phase intent fields
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_event \
  --detection-mode event_centered \
  --event-keep-topk 1 --event-rank-by peak_speed --event-force-primary \
  --event-attach-gap 0.18 --event-attach-peak-ratio 0.62 \
  --score-norm-base p90 --dtau-weight 0.5 \
  --event-speed-alpha 0.4 --event-below-frames 3 \
  --snap-peak-pct 85 --snap-min-speed-norm 0.6 --snap-min-gap 0.15 \
  --snap-window 0.35 --shape-dct-k 4 \
  --merge-gap 0.12

# normalized + fixed threshold
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_norm_fixed \
  --threshold-mode norm_fixed --norm-base p90 --on-abs 2.0 --off-abs 1.3 --merge-gap 0.12

# normalized + fixed threshold + peak strength filter
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_norm_fixed_peak \
  --threshold-mode norm_fixed --norm-base p90 --on-abs 2.0 --off-abs 1.3 \
  --peak-min-norm 3.0 --merge-gap 0.12

# fill short holes only when v_norm in hole stays high
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_fill_holes \
  --threshold-mode norm_fixed --norm-base p90 --on-abs 1.1 --off-abs 0.83 \
  --merge-mode fill_holes --hole-min-norm 0.5 --merge-gap 0.12

# global threshold JSON
python tools/burst_batch_ros1.py --input-dir /data/episodes --out-dir burst_out_global \
  --threshold-mode global_percentile --global-stats /data/global_stats.json --merge-gap 0.12
```

Recommended run command:
```bash
python tools/burst_batch_ros1.py \
  --input-dir /data/episodes \
  --out-dir burst_out_all_event \
  --detection-mode event_centered \
  --event-keep-topk 1 --event-rank-by peak_speed --event-force-primary \
  --event-attach-gap 0.18 --event-attach-peak-ratio 0.62 \
  --score-norm-base p90 --dtau-weight 0.5 \
  --event-speed-alpha 0.4 --event-below-frames 3 \
  --snap-peak-pct 85 --snap-min-speed-norm 0.6 --snap-min-gap 0.15 \
  --snap-window 0.35 --shape-dct-k 4 \
  --peak-min-norm 3.0 \
  --merge-gap 0.12 \
  --uniform-speed-axis
```

Output layout (single + batch):
- CSV files: `OUTDIR/csv/<episode>.csv`
- Plots: `OUTDIR/plots/<episode>.png`
- Batch summary: `OUTDIR/summary.csv`

`event_centered` outputs extra per-burst columns in CSV:
- snap center: `snap_time_s`, `snap_score`
- phases: `windup_*`, `snap_*`, `follow_*`
- intent vector: `z_amp_speed`, `z_amp_acc`, `z_dur`, `z_dir_unit`, `z_shape_dct`

方案对比（碎片化 + 阈值漂移）:

1. `merge_gap`（已实现）
- 优点：能显著减少反复跨阈值造成的碎片化短段。
- 缺点：`merge_gap` 过大可能把本应分开的动作合并。
- 成本：低。

2. `v_norm + fixed threshold`（已实现，`norm_fixed`）
- 优点：跨 episode 口径更一致，慢 episode 不会因相对分位数被过度标注。
- 缺点：需要调 `on_abs/off_abs`。
- 成本：低。

3. `global_percentile + global_stats.json`（已实现读取）
- 优点：整批数据共享同一阈值标准，可复现实验。
- 缺点：需要先离线统计并维护全局 JSON。
- 成本：中。

4. burst 内最小峰值/最小能量过滤（建议）
- 优点：进一步抑制弱噪声和短促伪 burst。
- 缺点：会增加 1~2 个阈值需要调参。
- 成本：低到中。

5. 周期性/零交叉过滤（建议）
- 优点：有助于区分甩动类运动与单次快速到位动作。
- 缺点：实现复杂、解释成本高。
- 成本：中。

Global stats JSON (v1) schema example:
```json
{
  "version": 1,
  "signal_space": "speed_norm",
  "on_th": 2.0,
  "off_th": 1.3,
  "norm_base": "p90",
  "norm_eps": 1e-6
}
```
