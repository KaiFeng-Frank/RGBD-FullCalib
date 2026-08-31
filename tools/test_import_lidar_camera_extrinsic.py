#!/usr/bin/env python3
"""Regression tests for the LiDAR-camera extrinsic draft importer."""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from import_lidar_camera_extrinsic import (  # noqa: E402
    ImportFailure,
    _atomic_json_write,
    _max_abs_delta,
    apply_transform,
    build_draft,
    invert_se3,
    load_strict_json,
    transform_from_upstream_vector,
)


class TransformMathTests(unittest.TestCase):
    def test_known_translation_and_nonunit_quaternion(self):
        transform, normalized, original_norm = transform_from_upstream_vector(
            [1, 2, 3, 0, 0, 0, 2])
        self.assertEqual(normalized, [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(original_norm, 2.0)
        self.assertEqual(transform, [
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.assertEqual(invert_se3(transform), [
            [1.0, 0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0, -2.0],
            [0.0, 0.0, 1.0, -3.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def test_direction_is_camera_to_lidar_then_inverted(self):
        half = math.sqrt(0.5)
        forward, _, _ = transform_from_upstream_vector(
            [1, 2, 3, 0, 0, half, half])
        inverse = invert_se3(forward)
        p_camera = [1.0, 0.0, 0.0]
        p_lidar = apply_transform(forward, p_camera)
        self.assertSequenceAlmostEqual(p_lidar, [1.0, 3.0, 3.0])
        self.assertSequenceAlmostEqual(apply_transform(inverse, p_lidar), p_camera)

    def assertSequenceAlmostEqual(self, actual, expected, places=12):
        self.assertEqual(len(actual), len(expected))
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=places)

    def test_double_inverse(self):
        transform, _, _ = transform_from_upstream_vector(
            [-0.42, 1.1, 0.03, 0.2, -0.3, 0.1, 0.8])
        self.assertLess(_max_abs_delta(invert_se3(invert_se3(transform)), transform),
                        1.0e-12)

    def test_invalid_values_are_rejected(self):
        invalid = (
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, float("nan"), 0, 0, 1],
            [0, 0, float("inf"), 0, 0, 0, 1],
            [False, 0, 0, 0, 0, 0, 1],
            [0, 0, "zero", 0, 0, 0, 1],
        )
        for vector in invalid:
            with self.subTest(vector=vector), self.assertRaises(ImportFailure):
                transform_from_upstream_vector(vector)


class DraftTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.calib = self.root / "calib.json"
        self.raw = self.root / "recording.db3"
        self.raw.write_bytes(b"calibration-data")
        self.upstream = {
            "version": "test-version",
            "command": "calibrate --test",
            "results": {
                "T_lidar_camera": [1, 2, 3, 0, 0, 0, 2],
                "score": 0.125,
            },
        }
        self.calib.write_text(json.dumps(self.upstream), encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def test_draft_preserves_provenance_but_has_no_validation(self):
        doc = build_draft(
            self.upstream,
            upstream_path=self.calib,
            rig_id="rig-01",
            mount_session_id="mount-01",
            lidar_serial="LIDAR123",
            camera_serial="CAMERA456",
            source_paths=[self.raw],
            solver_version="v1",
            solver_commit="abc123",
            solver_command="solver input",
        )
        self.assertEqual(doc["status"], "draft")
        self.assertNotIn("validation", doc)
        self.assertEqual(doc["frame_convention"]["equation"],
                         "p_camera = T_camera_lidar * p_lidar")
        self.assertEqual(doc["import_provenance"]
                         ["upstream_T_lidar_camera_vector_raw_xyzw"],
                         [1, 2, 3, 0, 0, 0, 2])
        self.assertEqual(doc["method_metadata"]["upstream_top_level_metadata"],
                         {"version": "test-version", "command": "calibrate --test"})
        self.assertEqual(doc["method_metadata"]["upstream_results_metadata"],
                         {"score": 0.125})
        self.assertEqual({item["role"] for item in doc["source_data"]},
                         {"solver_output", "calibration"})
        self.assertTrue(all(len(item["sha256"]) == 64 for item in doc["source_data"]))
        self.assertEqual(doc["devices"][0]["serial"], "LIDAR123")
        self.assertEqual(doc["mount_session_id"], "mount-01")
        self.assertEqual(doc["devices"][1]["serial"], "CAMERA456")
        self.assertEqual(doc["result"]["T_camera_lidar"][0][3], -1.0)

    def test_missing_transform_is_rejected(self):
        with self.assertRaises(ImportFailure):
            build_draft(
                {"results": {}}, upstream_path=self.calib, rig_id="rig",
                lidar_serial="l", camera_serial="c")

    def test_strict_json_rejects_duplicate_and_nonfinite(self):
        duplicate = self.root / "duplicate.json"
        duplicate.write_text('{"results":{},"results":{}}', encoding="utf-8")
        with self.assertRaises(ImportFailure):
            load_strict_json(duplicate)
        nonfinite = self.root / "nonfinite.json"
        nonfinite.write_text('{"x":NaN}', encoding="utf-8")
        with self.assertRaises(ImportFailure):
            load_strict_json(nonfinite)

    def test_force_cannot_replace_a_validated_result_with_a_draft(self):
        protected = self.root / "protected.json"
        protected.write_text(json.dumps({"status": "validated", "result": {}}),
                             encoding="utf-8")
        original = protected.read_bytes()
        with self.assertRaisesRegex(ImportFailure, "validated result"):
            _atomic_json_write(protected, {"status": "draft"}, force=True)
        self.assertEqual(protected.read_bytes(), original)

    def test_cli_selftest(self):
        script = Path(__file__).with_name("import_lidar_camera_extrinsic.py")
        run = subprocess.run(
            [sys.executable, str(script), "--selftest"],
            text=True, capture_output=True, check=False)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(json.loads(run.stdout)["status"], "passed")


if __name__ == "__main__":
    unittest.main()
