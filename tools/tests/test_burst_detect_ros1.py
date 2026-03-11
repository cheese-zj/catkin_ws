#!/usr/bin/env python3
"""Unit tests for burst detection utilities."""

from __future__ import annotations

import json
import sys
import unittest
from unittest import mock
from pathlib import Path

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import burst_detect_ros1 as detector


class MergeBurstsTest(unittest.TestCase):
    def test_merge_when_gap_less_than_threshold(self):
        bursts = [
            detector.BurstWindow(0.0, 1.0),
            detector.BurstWindow(1.05, 2.0),
        ]
        merged = detector.merge_bursts(bursts, merge_gap=0.1)
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.0, merged[0].start_s)
        self.assertAlmostEqual(2.0, merged[0].end_s)

    def test_do_not_merge_when_gap_equals_threshold(self):
        bursts = [
            detector.BurstWindow(0.0, 1.0),
            detector.BurstWindow(1.1, 2.0),
        ]
        merged = detector.merge_bursts(bursts, merge_gap=0.1)
        self.assertEqual(2, len(merged))

    def test_chain_merge(self):
        bursts = [
            detector.BurstWindow(0.0, 1.0),
            detector.BurstWindow(1.05, 2.0),
            detector.BurstWindow(2.08, 3.0),
        ]
        merged = detector.merge_bursts(bursts, merge_gap=0.1)
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.0, merged[0].start_s)
        self.assertAlmostEqual(3.0, merged[0].end_s)

    def test_overlap_always_merge(self):
        bursts = [
            detector.BurstWindow(0.0, 1.2),
            detector.BurstWindow(1.0, 2.0),
        ]
        merged = detector.merge_bursts(bursts, merge_gap=0.0)
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.0, merged[0].start_s)
        self.assertAlmostEqual(2.0, merged[0].end_s)

    def test_empty_and_single_input(self):
        self.assertEqual([], detector.merge_bursts([], merge_gap=0.1))

        single = [detector.BurstWindow(0.2, 0.4)]
        merged = detector.merge_bursts(single, merge_gap=0.1)
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.2, merged[0].start_s)
        self.assertAlmostEqual(0.4, merged[0].end_s)

    def test_unsorted_input_is_handled(self):
        bursts = [
            detector.BurstWindow(2.0, 3.0),
            detector.BurstWindow(0.0, 1.0),
            detector.BurstWindow(1.05, 1.91),
        ]
        merged = detector.merge_bursts(bursts, merge_gap=0.1)
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.0, merged[0].start_s)
        self.assertAlmostEqual(3.0, merged[0].end_s)


class PeakFilterTest(unittest.TestCase):
    def setUp(self):
        self.t = np.asarray([0.0, 0.5, 1.0, 1.5, 2.0], dtype=float)
        self.speed = np.asarray([0.1, 0.3, 0.6, 1.2, 0.2], dtype=float)
        self.speed_norm = np.asarray([0.5, 1.0, 2.2, 3.3, 0.8], dtype=float)
        self.windows = [
            detector.BurstWindow(0.0, 1.0),
            detector.BurstWindow(1.0, 2.0),
        ]

    def test_peak_min_norm_filters_weak_window(self):
        filtered = detector.filter_bursts_by_peak(
            t=self.t,
            speed=self.speed,
            speed_norm=self.speed_norm,
            windows=self.windows,
            peak_min_raw=0.0,
            peak_min_norm=3.0,
        )
        self.assertEqual(1, len(filtered))
        self.assertAlmostEqual(1.0, filtered[0].start_s)
        self.assertAlmostEqual(2.0, filtered[0].end_s)

    def test_peak_min_raw_filters_weak_window(self):
        filtered = detector.filter_bursts_by_peak(
            t=self.t,
            speed=self.speed,
            speed_norm=self.speed_norm,
            windows=self.windows,
            peak_min_raw=1.0,
            peak_min_norm=0.0,
        )
        self.assertEqual(1, len(filtered))
        self.assertAlmostEqual(1.0, filtered[0].start_s)
        self.assertAlmostEqual(2.0, filtered[0].end_s)

    def test_zero_thresholds_keep_all(self):
        filtered = detector.filter_bursts_by_peak(
            t=self.t,
            speed=self.speed,
            speed_norm=self.speed_norm,
            windows=self.windows,
            peak_min_raw=0.0,
            peak_min_norm=0.0,
        )
        self.assertEqual(2, len(filtered))


class FillHolesMergeTest(unittest.TestCase):
    def test_short_high_signal_hole_merges(self):
        t = np.asarray([0.0, 0.5, 1.0, 1.04, 1.5, 2.0], dtype=float)
        hole_signal = np.asarray([1.0, 1.0, 1.0, 0.7, 1.0, 1.0], dtype=float)
        bursts = [detector.BurstWindow(0.0, 1.0), detector.BurstWindow(1.05, 2.0)]
        merged = detector.merge_bursts_fill_holes(
            t=t,
            hole_signal_norm=hole_signal,
            bursts=bursts,
            merge_gap=0.1,
            hole_min_norm=0.5,
            hole_quantile=0.0,
        )
        self.assertEqual(1, len(merged))
        self.assertAlmostEqual(0.0, merged[0].start_s)
        self.assertAlmostEqual(2.0, merged[0].end_s)

    def test_short_low_signal_hole_does_not_merge(self):
        t = np.asarray([0.0, 0.5, 1.0, 1.04, 1.5, 2.0], dtype=float)
        hole_signal = np.asarray([1.0, 1.0, 1.0, 0.3, 1.0, 1.0], dtype=float)
        bursts = [detector.BurstWindow(0.0, 1.0), detector.BurstWindow(1.05, 2.0)]
        merged = detector.merge_bursts_fill_holes(
            t=t,
            hole_signal_norm=hole_signal,
            bursts=bursts,
            merge_gap=0.1,
            hole_min_norm=0.5,
            hole_quantile=0.0,
        )
        self.assertEqual(2, len(merged))

    def test_long_hole_does_not_merge(self):
        t = np.asarray([0.0, 0.5, 1.0, 1.2, 1.6, 2.0], dtype=float)
        hole_signal = np.asarray([1.0, 1.0, 1.0, 0.9, 0.9, 1.0], dtype=float)
        bursts = [detector.BurstWindow(0.0, 1.0), detector.BurstWindow(1.2, 2.0)]
        merged = detector.merge_bursts_fill_holes(
            t=t,
            hole_signal_norm=hole_signal,
            bursts=bursts,
            merge_gap=0.1,
            hole_min_norm=0.5,
            hole_quantile=0.0,
        )
        self.assertEqual(2, len(merged))


class EventCenteredTest(unittest.TestCase):
    def test_select_speed_peak_idx_energy_prefers_wide_hump(self):
        t = np.linspace(0.0, 1.2, 13, dtype=float)
        speed = np.asarray([0.1, 0.1, 0.2, 0.3, 4.2, 0.4, 1.8, 2.6, 2.8, 2.7, 0.4, 0.2, 0.1], dtype=float)
        local_idx = np.arange(3, 10, dtype=np.int64)

        idx_max = detector._select_speed_peak_idx(
            t=t,
            speed=speed,
            local_idx=local_idx,
            mode="max",
            window_s=0.2,
        )
        idx_energy = detector._select_speed_peak_idx(
            t=t,
            speed=speed,
            local_idx=local_idx,
            mode="energy",
            window_s=0.25,
        )
        self.assertEqual(4, idx_max)  # narrow spike
        self.assertIn(idx_energy, (7, 8, 9))  # broader hump

    def test_resolve_windup_start_from_rest(self):
        t = np.linspace(0.0, 1.0, 11, dtype=float)
        speed = np.asarray([0.2, 0.2, 0.2, 0.25, 0.2, 0.18, 0.5, 0.9, 1.4, 2.0, 1.5], dtype=float)
        burst_start = 0.8
        start = detector._resolve_windup_start_from_rest(
            t=t,
            speed=speed,
            burst_start_s=burst_start,
            fallback_start_s=0.6,
            rest_speed_th=0.3,
            rest_frames=3,
            max_lookback_s=0.6,
            min_gap_s=0.0,
        )
        # Nearest rest segment before acceleration ends around t=0.5.
        self.assertAlmostEqual(0.5, start, places=6)

    def test_resolve_follow_end_from_rest(self):
        t = np.linspace(0.0, 1.2, 13, dtype=float)
        speed = np.asarray(
            [0.2, 0.3, 0.6, 1.0, 1.6, 2.1, 1.4, 0.9, 0.5, 0.25, 0.2, 0.2, 0.2],
            dtype=float,
        )
        burst_end = 0.7
        follow_end = detector._resolve_follow_end_from_rest(
            t=t,
            speed=speed,
            burst_end_s=burst_end,
            fallback_end_s=1.1,
            rest_speed_th=0.3,
            rest_frames=3,
            max_lookahead_s=0.5,
            min_gap_s=0.0,
        )
        # First stable rest run starts at t=0.9.
        self.assertAlmostEqual(0.9, follow_end, places=6)

    def test_resolve_energy_burst_span_contains_snap(self):
        t = np.linspace(0.0, 1.0, 11, dtype=float)
        speed = np.asarray([0.1, 0.2, 0.3, 0.8, 1.2, 2.0, 1.5, 1.0, 0.4, 0.2, 0.1], dtype=float)
        burst_start, burst_end, search_start, search_end, total, covered = detector._resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=0.5,
            search_left_s=0.5,
            search_right_s=0.5,
            energy_rho=0.9,
        )
        self.assertLessEqual(burst_start, 0.5)
        self.assertGreaterEqual(burst_end, 0.5)
        self.assertLessEqual(search_start, burst_start)
        self.assertGreaterEqual(search_end, burst_end)
        self.assertGreater(total, 0.0)
        self.assertGreaterEqual(covered, 0.9 * total - 1e-8)

    def test_resolve_energy_burst_span_prefers_wide_packet_over_spike(self):
        t = np.linspace(0.0, 1.2, 13, dtype=float)
        speed = np.asarray([0.1, 0.1, 0.2, 0.4, 4.0, 0.3, 1.8, 2.4, 2.6, 2.4, 0.4, 0.2, 0.1], dtype=float)
        burst_start, burst_end, _s0, _s1, _total, _covered = detector._resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=0.8,
            search_left_s=0.6,
            search_right_s=0.4,
            energy_rho=0.85,
        )
        self.assertLessEqual(burst_start, 0.8)
        self.assertGreaterEqual(burst_end, 0.8)
        # Should include broad packet around 0.7~0.9, not collapse to spike around 0.4.
        self.assertGreater(burst_end - burst_start, 0.15)
        self.assertLess(burst_start, 0.75)

    def test_resolve_energy_burst_span_multipeak_coverage(self):
        t = np.linspace(0.0, 1.4, 15, dtype=float)
        speed = np.asarray([0.1, 0.2, 0.4, 1.6, 2.0, 1.2, 0.5, 0.4, 1.3, 1.9, 1.5, 0.6, 0.2, 0.1, 0.1], dtype=float)
        burst_start, burst_end, _s0, _s1, total, covered = detector._resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=0.9,
            search_left_s=0.8,
            search_right_s=0.5,
            energy_rho=0.9,
        )
        self.assertLessEqual(burst_start, 0.9)
        self.assertGreaterEqual(burst_end, 0.9)
        self.assertGreater(total, 0.0)
        self.assertGreaterEqual(covered, 0.9 * total - 1e-8)

    def test_attach_strong_neighbor_events(self):
        primary = detector.EventWindow(
            window=detector.BurstWindow(7.45, 7.72),
            snap_idx=10,
            speed_peak_idx=10,
            snap_score=6.5,
            score_th=1.0,
            speed_peak=4.8,
            speed_rel_th=1.9,
        )
        neighbor_strong = detector.EventWindow(
            window=detector.BurstWindow(7.87, 8.30),
            snap_idx=11,
            speed_peak_idx=11,
            snap_score=5.7,
            score_th=1.0,
            speed_peak=3.9,  # ratio ~= 0.81 of primary
            speed_rel_th=1.5,
        )
        neighbor_weak = detector.EventWindow(
            window=detector.BurstWindow(7.00, 7.20),
            snap_idx=9,
            speed_peak_idx=9,
            snap_score=4.0,
            score_th=1.0,
            speed_peak=2.0,  # ratio ~= 0.42, should not attach
            speed_rel_th=0.8,
        )

        merged = detector.attach_strong_neighbor_events(
            primary=primary,
            candidates=[primary, neighbor_strong, neighbor_weak],
            attach_gap_s=0.2,
            attach_peak_ratio=0.7,
        )
        self.assertAlmostEqual(7.45, merged.window.start_s)
        self.assertAlmostEqual(8.30, merged.window.end_s)

    def test_select_top_event_windows_ranking(self):
        events = [
            detector.EventWindow(
                window=detector.BurstWindow(0.0, 1.0),
                snap_idx=1,
                speed_peak_idx=1,
                snap_score=10.0,
                score_th=1.0,
                speed_peak=2.0,
                speed_rel_th=0.8,
            ),
            detector.EventWindow(
                window=detector.BurstWindow(2.0, 3.0),
                snap_idx=2,
                speed_peak_idx=2,
                snap_score=8.0,
                score_th=1.0,
                speed_peak=5.0,
                speed_rel_th=1.5,
            ),
        ]
        top_by_score = detector.select_top_event_windows(events=events, keep_topk=1, rank_by="snap_score")
        self.assertEqual(1, len(top_by_score))
        self.assertAlmostEqual(10.0, top_by_score[0].snap_score)

        top_by_speed = detector.select_top_event_windows(events=events, keep_topk=1, rank_by="peak_speed")
        self.assertEqual(1, len(top_by_speed))
        self.assertAlmostEqual(5.0, top_by_speed[0].speed_peak)

    def test_event_centered_merges_close_multi_peaks(self):
        t = np.linspace(0.0, 1.0, 21, dtype=float)
        speed = np.asarray(
            [
                0.1,
                0.1,
                0.2,
                0.4,
                0.9,
                1.5,
                2.0,
                2.4,
                2.7,
                2.5,
                2.6,
                2.2,
                1.7,
                1.1,
                0.6,
                0.3,
                0.2,
                0.1,
                0.1,
                0.1,
                0.1,
            ],
            dtype=float,
        )
        speed_norm, _ = detector.normalize_speed(speed=speed, norm_base="p90", norm_eps=1e-6)
        burst_score = np.asarray(
            [
                0.1,
                0.2,
                0.3,
                0.4,
                0.5,
                1.1,
                1.9,
                2.8,
                3.5,
                2.1,
                3.3,
                2.0,
                1.2,
                0.8,
                0.4,
                0.2,
                0.1,
                0.1,
                0.1,
                0.1,
                0.1,
            ],
            dtype=float,
        )

        events = detector.detect_event_centered_windows(
            t=t,
            speed=speed,
            speed_norm=speed_norm,
            burst_score=burst_score,
            min_dur=0.08,
            pad=0.0,
            merge_gap=0.2,
            event_speed_alpha=0.4,
            event_below_frames=2,
            snap_peak_pct=80.0,
            snap_peak_min_score=1.0,
            snap_min_speed_norm=0.2,
            snap_min_gap_s=0.0,
            snap_peak_search_s=0.1,
            snap_time_mode="score_peak",
            snap_speed_peak_mode="max",
            snap_speed_peak_window_s=0.2,
        )
        self.assertEqual(1, len(events))
        self.assertLess(events[0].window.start_s, 0.4)
        self.assertGreater(events[0].window.end_s, 0.6)

    def test_build_table_has_phase_and_intent_columns(self):
        t = np.linspace(0.0, 1.0, 11, dtype=float)
        t_abs = t + 100.0
        speed = np.asarray([0.1, 0.2, 0.6, 1.2, 1.8, 2.0, 1.7, 1.0, 0.5, 0.2, 0.1], dtype=float)
        acc = np.asarray([0.3, 0.4, 0.8, 1.5, 2.1, 2.6, 2.0, 1.2, 0.7, 0.4, 0.2], dtype=float)
        dtau = np.asarray([0.1, 0.2, 0.3, 0.6, 1.0, 1.4, 1.1, 0.7, 0.4, 0.2, 0.1], dtype=float)
        qdot = np.column_stack([speed, 0.5 * speed, -0.25 * speed])
        burst_score = acc + dtau
        window = detector.BurstWindow(0.2, 0.8)
        event = detector.EventWindow(
            window=window,
            snap_idx=5,
            speed_peak_idx=5,
            snap_score=float(burst_score[5]),
            score_th=1.0,
            speed_peak=float(speed[5]),
            speed_rel_th=0.8,
        )
        th = detector.ThresholdInfo(
            mode="norm_fixed",
            signal_space="speed_norm",
            norm_base="p90",
            norm_scale=1.0,
            norm_eps=1e-6,
            on_th_signal=2.0,
            off_th_signal=1.3,
            on_th_speed_equiv=2.0,
            off_th_speed_equiv=1.3,
        )

        df = detector.build_bursts_table(
            t=t,
            speed=speed,
            acc=acc,
            dtau=dtau,
            qdot=qdot,
            burst_score=burst_score,
            windows=[window],
            threshold_info=th,
            merge_gap_s=0.12,
            merge_mode="gap",
            hole_min_norm=0.5,
            hole_quantile=0.0,
            peak_min_norm=0.0,
            peak_min_raw=0.0,
            peak_filter_norm_base="p90",
            peak_filter_norm_scale=1.0,
            peak_filter_norm_eps=1e-6,
            detection_mode="event_centered",
            score_norm_base="p90",
            score_norm_eps=1e-6,
            dtau_weight=1.0,
            event_speed_alpha=0.4,
            event_below_frames=3,
            snap_peak_pct=90.0,
            snap_peak_min_score=0.0,
            snap_min_speed_norm=0.4,
            snap_min_gap_s=0.15,
            snap_peak_search_s=0.2,
            snap_time_mode="score_peak",
            snap_speed_peak_mode="max",
            snap_speed_peak_window_s=0.2,
            burst_energy_rho=0.8,
            burst_energy_left_s=0.5,
            burst_energy_right_s=0.5,
            shape_dct_k=4,
            event_keep_topk=1,
            event_rank_by="snap_score",
            event_force_primary=True,
            event_attach_gap_s=0.18,
            event_attach_peak_ratio=0.7,
            windup_rest_frames=0,
            windup_rest_speed_scale=1.0,
            windup_rest_lookback_s=2.5,
            windup_rest_min_gap_s=0.0,
            follow_rest_frames=8,
            follow_rest_speed_scale=1.3,
            follow_rest_lookahead_s=2.5,
            follow_rest_min_gap_s=0.15,
            event_windows=[event],
            t_abs=t_abs,
        )

        self.assertEqual(1, len(df))
        row = df.iloc[0]
        self.assertIn("snap_time_s", df.columns)
        self.assertIn("windup_duration_s", df.columns)
        self.assertIn("burst_duration_s", df.columns)
        self.assertIn("energy_burst_share", df.columns)
        self.assertIn("z_dir_unit", df.columns)
        self.assertNotIn("snap_duration_s", df.columns)
        self.assertAlmostEqual(float(row["windup_end_s"]), float(row["burst_start_s"]), places=8)
        self.assertAlmostEqual(float(row["follow_start_s"]), float(row["burst_end_s"]), places=8)
        self.assertGreater(float(row["burst_duration_s"]), 0.0)
        self.assertGreaterEqual(float(row["follow_end_s"]), float(row["follow_start_s"]))
        self.assertGreaterEqual(float(row["energy_burst_share"]), 0.0)
        self.assertLessEqual(float(row["energy_burst_share"]), 1.0)
        self.assertAlmostEqual(float(row["z_dur"]), float(row["burst_duration_s"]), places=8)
        z_dir = json.loads(str(row["z_dir_unit"]))
        self.assertEqual(3, len(z_dir))
        self.assertAlmostEqual(1.0, float(np.linalg.norm(np.asarray(z_dir))), places=5)

    def test_phase_bounds_mutual_exclusive_windup_burst_follow(self):
        t = np.linspace(0.0, 2.0, 41, dtype=float)
        speed = np.exp(-0.5 * ((t - 1.0) / 0.25) ** 2)
        burst_start, burst_end, _s0, _s1, _total, _covered = detector._resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=1.0,
            search_left_s=1.0,
            search_right_s=1.0,
            energy_rho=0.9,
        )
        windup_start = detector._resolve_windup_start_from_rest(
            t=t,
            speed=speed,
            burst_start_s=burst_start,
            fallback_start_s=max(float(t[0]), burst_start - 0.8),
            rest_speed_th=0.2,
            rest_frames=4,
            max_lookback_s=1.0,
            min_gap_s=0.0,
        )
        follow_end = detector._resolve_follow_end_from_rest(
            t=t,
            speed=speed,
            burst_end_s=burst_end,
            fallback_end_s=min(float(t[-1]), burst_end + 0.8),
            rest_speed_th=0.2,
            rest_frames=4,
            max_lookahead_s=1.0,
            min_gap_s=0.0,
        )
        windup_end = burst_start
        follow_start = burst_end
        self.assertLessEqual(windup_start, windup_end)
        self.assertLessEqual(follow_start, follow_end)
        self.assertAlmostEqual(windup_end, burst_start, places=8)
        self.assertAlmostEqual(follow_start, burst_end, places=8)

    def test_z_features_are_computed_on_burst_span(self):
        t = np.linspace(0.0, 1.0, 11, dtype=float)
        speed = np.asarray([0.1, 0.2, 0.6, 1.2, 1.8, 2.0, 1.7, 1.0, 0.5, 0.2, 0.1], dtype=float)
        burst_start, burst_end, _s0, _s1, _total, _covered = detector._resolve_energy_burst_span(
            t=t,
            speed=speed,
            snap_time_s=0.5,
            search_left_s=0.5,
            search_right_s=0.5,
            energy_rho=0.8,
        )
        idx = detector._indices_in_span(t=t, start_s=burst_start, end_s=burst_end, fallback_idx=5)
        z_dur = float(max(0.0, burst_end - burst_start))
        z_amp = float(np.max(speed[idx]))
        self.assertGreater(z_dur, 0.0)
        self.assertGreater(z_amp, 1.0)

    def test_removed_cli_args_fail_fast(self):
        with mock.patch.object(sys, "argv", ["burst_detect_ros1.py", "--bag", "dummy.bag", "--ramp-frames", "4"]):
            with self.assertRaises(SystemExit):
                detector.parse_args()
        with mock.patch.object(sys, "argv", ["burst_detect_ros1.py", "--bag", "dummy.bag", "--snap-window", "0.35"]):
            with self.assertRaises(SystemExit):
                detector.parse_args()


if __name__ == "__main__":
    unittest.main()
