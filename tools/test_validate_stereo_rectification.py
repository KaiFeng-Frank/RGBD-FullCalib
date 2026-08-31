#!/usr/bin/env python3
"""Offline tests for the delivered-IR stereo validation tool."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np


MODULE_PATH = Path(__file__).with_name("validate_stereo_rectification.py")
SPEC = importlib.util.spec_from_file_location("validate_stereo_rectification", MODULE_PATH)
assert SPEC and SPEC.loader
stereo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stereo)


class StereoRectificationValidationTest(unittest.TestCase):
    def test_match_is_tag_id_and_canonical_corner_stable(self):
        left = {
            7: np.array([[70, 0], [71, 1], [72, 2], [73, 3]], dtype=float),
            2: np.array([[20, 0], [21, 1], [22, 2], [23, 3]], dtype=float),
        }
        right = {
            2: left[2] + [100, 0.25],
            7: left[7] + [100, -0.5],
            9: np.zeros((4, 2)),
        }
        lpts, rpts, keys = stereo.match_markers(left, right)
        self.assertEqual(keys[0], {"tag_id": 2, "corner_index": 0})
        self.assertEqual(keys[-1], {"tag_id": 7, "corner_index": 3})
        np.testing.assert_allclose(lpts[:4, 1] - rpts[:4, 1], -0.25)
        np.testing.assert_allclose(lpts[4:, 1] - rpts[4:, 1], 0.5)

    def test_duplicate_tag_is_rejected(self):
        corners = [np.zeros((1, 4, 2)), np.ones((1, 4, 2))]
        with self.assertRaisesRegex(ValueError, "duplicate"):
                stereo.marker_map(corners, np.array([[3], [3]]))

    def test_non_target_dictionary_ids_are_ignored(self):
        corners = [np.zeros((1, 4, 2)), np.ones((1, 4, 2))]
        got = stereo.marker_map(corners, np.array([[3], [99]]), stereo.TARGET_TAG_IDS)
        self.assertEqual(list(got), [3])

    def test_summary_keeps_outlier_and_uses_linear_percentile(self):
        dy = np.array([-0.25, 0.5, 1.0, 3.0])
        got = stereo.summarize_vertical(dy)
        self.assertEqual(got["count"], 4)
        self.assertAlmostEqual(got["signed_median_px"], 0.75)
        self.assertAlmostEqual(got["abs_vertical_px"]["median"], 0.75)
        self.assertAlmostEqual(got["abs_vertical_px"]["p95"], float(np.percentile(np.abs(dy), 95)))
        self.assertEqual(got["abs_vertical_px"]["max"], 3.0)
        self.assertEqual(got["fraction_abs_vertical_gt_2px"], 0.25)

    def test_grid_coverage(self):
        points = np.array([[10, 10], [500, 10], [1000, 10],
                           [10, 300], [500, 300], [1000, 600]], dtype=float)
        got = stereo.grid_coverage(points, 1280, 720)
        self.assertEqual(got["covered_cell_count"], 6)
        self.assertEqual(got["covered_row_count"], 3)
        self.assertEqual(got["covered_column_count"], 3)

    @staticmethod
    def _support(pairs=12, corners=600, cells=6, rows=2, columns=2):
        return {
            "pairs_with_common_tags": pairs,
            "matched_corners": corners,
            "coverage": {
                "covered_cell_count": cells,
                "covered_row_count": rows,
                "covered_column_count": columns,
            },
        }

    def test_pass_requires_all_four_frozen_gates(self):
        metrics = stereo.summarize_vertical(np.zeros(600))
        got = stereo.evaluate_acceptance(self._support(), metrics)
        self.assertEqual(got["status"], "passed")
        self.assertEqual(got["pass_numeric"], 1)
        self.assertEqual(len(got["gate_checks"]), 4)

    def test_outlier_failure_is_not_deleted(self):
        # Seven of 600 points above 2 px means 1.167%, just beyond the 1% gate.
        metrics = stereo.summarize_vertical(np.r_[np.zeros(593), np.full(7, 2.1)])
        got = stereo.evaluate_acceptance(self._support(), metrics)
        self.assertEqual(got["status"], "failed")
        failed = {row["id"] for row in got["gate_checks"] if not row["passed"]}
        self.assertIn("fraction_abs_vertical_gt_2px", failed)

    def test_insufficient_support_can_never_pass(self):
        metrics = stereo.summarize_vertical(np.zeros(599))
        got = stereo.evaluate_acceptance(self._support(corners=599), metrics)
        self.assertEqual(got["status"], "insufficient")
        self.assertEqual(got["pass_numeric"], 0)

    def test_frozen_pair_set_rejects_missing_and_extra_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            image = np.zeros((8, 8), dtype=np.uint8)
            for stem in ("0000", "0001"):
                cv2.imwrite(str(root / f"{stem}.png"), image)
                cv2.imwrite(str(root / f"{stem}_r.png"), image)
            pairs = stereo.discover_pairs(root, ("0000", "0001"))
            self.assertEqual([row[0] for row in pairs], ["0000", "0001"])
            (root / "0001.png").unlink()
            with self.assertRaisesRegex(ValueError, "frozen evaluation set changed"):
                stereo.discover_pairs(root, ("0000", "0001"))
            cv2.imwrite(str(root / "0001.png"), image)
            cv2.imwrite(str(root / "0002_r.png"), image)
            with self.assertRaisesRegex(ValueError, r"right_extra=\['0002'\]"):
                stereo.discover_pairs(root, ("0000", "0001"))

    def test_y8_guard_rejects_implicit_color_or_16bit_conversion(self):
        valid = np.zeros((720, 1280), dtype=np.uint8)
        self.assertIs(stereo.require_y8(valid, "0000", "left", 1280, 720), valid)
        with self.assertRaisesRegex(ValueError, "not raw 1280x720 Y8"):
            stereo.require_y8(np.zeros((720, 1280, 3), dtype=np.uint8),
                              "0000", "left", 1280, 720)
        with self.assertRaisesRegex(ValueError, "not raw 1280x720 Y8"):
            stereo.require_y8(np.zeros((720, 1280), dtype=np.uint16),
                              "0000", "left", 1280, 720)

    def test_calibration_and_evaluation_sets_must_differ(self):
        with tempfile.TemporaryDirectory() as tmp:
            same = Path(tmp)
            with self.assertRaisesRegex(ValueError, "must be independent"):
                stereo.validate(same, same, stereo.DEFAULT_CALIBRATION_CHAIN,
                                stereo.DEFAULT_FACTORY, stereo.DEFAULT_BAG)

    def test_copied_calibration_images_are_not_an_independent_holdout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            evaluation = root / "evaluation"
            calibration = root / "calibration"
            evaluation.mkdir()
            calibration.mkdir()
            image = np.zeros((8, 8), dtype=np.uint8)
            for stem in stereo.EXPECTED_STEMS:
                for suffix in (".png", "_r.png"):
                    cv2.imwrite(str(evaluation / f"{stem}{suffix}"), image)
                    cv2.imwrite(str(calibration / f"{stem}{suffix}"), image)
            with self.assertRaisesRegex(ValueError, "overlap the calibration set"):
                stereo.validate(evaluation, calibration, stereo.DEFAULT_CALIBRATION_CHAIN,
                                stereo.DEFAULT_FACTORY, stereo.DEFAULT_BAG)

    def test_copied_subset_is_rejected_even_if_calibration_has_extra_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            evaluation = root / "evaluation"
            calibration = root / "calibration"
            evaluation.mkdir()
            calibration.mkdir()
            for index, stem in enumerate(stereo.EXPECTED_STEMS):
                for suffix, offset in ((".png", 0), ("_r.png", 1)):
                    image = np.full((8, 8), index * 2 + offset, dtype=np.uint8)
                    cv2.imwrite(str(evaluation / f"{stem}{suffix}"), image)
                    cv2.imwrite(str(calibration / f"{stem}{suffix}"), image)
            cv2.imwrite(str(calibration / "extra.png"), np.full((8, 8), 255, dtype=np.uint8))
            with self.assertRaisesRegex(ValueError, "overlap the calibration set"):
                stereo.validate(evaluation, calibration, stereo.DEFAULT_CALIBRATION_CHAIN,
                                stereo.DEFAULT_FACTORY, stereo.DEFAULT_BAG)

    def test_insufficient_zero_match_plot_is_still_emitted(self):
        metrics = stereo.summarize_vertical([])
        result = {
            "metrics": metrics,
            "validation": {"status": "insufficient"},
        }
        plot_data = {
            "dy": np.empty(0),
            "midpoints": np.empty((0, 2)),
            "per_pair": [{"stem": "0000", "p95_abs_vertical_px": None}],
        }
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "empty.png"
            stereo.write_plot(output, result, plot_data)
            self.assertTrue(output.is_file())
            self.assertGreater(output.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
