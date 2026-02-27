#!/usr/bin/env python3
"""Unit tests for burst detection utilities."""

from __future__ import annotations

import sys
import unittest
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
        )
        self.assertEqual(2, len(merged))


if __name__ == "__main__":
    unittest.main()
