#!/usr/bin/env python3
"""Pure-NumPy tests for LiDAR-D435i gyro time alignment."""
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibrate_lidar_camera_timesync as timesync


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def motion(t):
    t = np.asarray(t, dtype=np.float64)
    pulse1 = 0.55 * np.exp(-0.5 * ((t - 5.2) / 0.45) ** 2)
    pulse2 = -0.42 * np.exp(-0.5 * ((t - 13.7) / 0.7) ** 2)
    pulse3 = 0.48 * np.exp(-0.5 * ((t - 20.1) / 0.38) ** 2)
    return np.column_stack((
        0.35 * np.sin(0.73 * t) + 0.12 * np.sin(2.31 * t + 0.2) + pulse1,
        0.28 * np.cos(0.47 * t + 0.4) + 0.16 * np.sin(1.61 * t) + pulse2,
        0.31 * np.sin(0.91 * t - 0.7) + 0.11 * np.cos(2.77 * t) + pulse3,
    ))


def synthetic(offset_s=0.0374, noise=0.0005):
    rng = np.random.default_rng(20260901)
    livox_t = np.arange(0.0, 27.0, 1.0 / 199.5)
    d435_t = np.arange(0.0, 27.0, 1.0 / 201.3) + 0.0017
    rotation = axis_angle([0.3, -0.7, 0.4], 0.83)
    bias = np.array([0.012, -0.008, 0.004])
    livox_w = motion(livox_t)
    # A D435i sample stamped t corresponds to physical Livox time t-offset.
    d435_w = motion(d435_t - offset_s) @ rotation.T + bias
    livox_w += rng.normal(0.0, noise, livox_w.shape)
    d435_w += rng.normal(0.0, noise, d435_w.shape)
    return livox_t, livox_w, d435_t, d435_w, rotation, bias


class EstimatorTests(unittest.TestCase):
    def test_recovers_positive_offset_rotation_and_bias(self):
        lt, lw, ct, cw, rotation, bias = synthetic()
        fit = timesync.estimate_time_offset(
            lt, lw, ct, cw, min_offset_s=-0.1, max_offset_s=0.1,
            coarse_step_s=0.001, refine_factor=20, min_samples=300)
        self.assertAlmostEqual(fit.offset_s, 0.0374, delta=0.00015)
        angle_error = math.acos(np.clip(
            (np.trace(fit.rotation @ rotation.T) - 1.0) / 2.0, -1.0, 1.0))
        self.assertLess(angle_error, math.radians(0.08))
        np.testing.assert_allclose(fit.bias, bias, atol=2.0e-4)
        self.assertGreater(np.linalg.det(fit.rotation), 0.999999)
        self.assertLess(fit.rmse_rad_s, 0.002)

    def test_sign_convention_recovers_negative_offset(self):
        lt, lw, ct, cw, _, _ = synthetic(offset_s=-0.0246)
        fit = timesync.estimate_time_offset(
            lt, lw, ct, cw, min_offset_s=-0.08, max_offset_s=0.08,
            coarse_step_s=0.001, refine_factor=20, min_samples=300)
        self.assertAlmostEqual(fit.offset_s, -0.0246, delta=0.00015)

    def test_three_independent_segment_refits(self):
        lt, lw, ct, cw, _, _ = synthetic()
        full, segments = timesync.analyze_three_segments(
            lt, lw, ct, cw, min_offset_s=-0.1, max_offset_s=0.1,
            coarse_step_s=0.001, refine_factor=20, min_samples=250)
        self.assertEqual(len(segments), 3)
        self.assertAlmostEqual(full.offset_s, 0.0374, delta=0.00015)
        for item in segments:
            self.assertAlmostEqual(item["offset_ms"], 37.4, delta=0.25)
            self.assertGreater(item["sample_count"], 250)

    def test_low_excitation_is_rejected(self):
        t = np.arange(0.0, 4.0, 0.005)
        w = np.tile([0.01, -0.02, 0.03], (len(t), 1))
        with self.assertRaisesRegex(timesync.TimesyncError, "excitation"):
            timesync.estimate_time_offset(
                t, w, t, w, min_offset_s=-0.02, max_offset_s=0.02,
                coarse_step_s=0.002, min_samples=100,
                min_excitation_rad_s=0.01)

    def test_duplicate_timestamps_are_collapsed(self):
        times = np.array([2.0, 1.0, 1.0, 3.0])
        gyro = np.array([[2.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                         [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
        got_t, got_w = timesync._prepare_series(times, gyro, "test")
        np.testing.assert_array_equal(got_t, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(got_w[0], [1.0, 0.0, 0.0])


class ProvenanceTests(unittest.TestCase):
    def test_directory_hash_binds_names_and_content(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "metadata.yaml").write_text("one", encoding="utf-8")
            (root / "data.db3").write_bytes(b"two")
            first, names = timesync.bag_sha256(root)
            second, _ = timesync.bag_sha256(root)
            self.assertEqual(first, second)
            self.assertEqual(names, ["data.db3", "metadata.yaml"])
            (root / "data.db3").write_bytes(b"changed")
            changed, _ = timesync.bag_sha256(root)
            self.assertNotEqual(first, changed)

    def test_document_records_required_convention_and_fields(self):
        lt, lw, ct, cw, _, _ = synthetic()
        full, segments = timesync.analyze_three_segments(
            lt, lw, ct, cw, min_offset_s=-0.1, max_offset_s=0.1,
            coarse_step_s=0.002, refine_factor=20, min_samples=200)
        document = timesync.build_document(
            bag=Path("fixture"), bag_digest="a" * 64,
            bag_files=["metadata.yaml", "fixture.db3"],
            d435i_serial="D435", mid360s_serial="MID", rig_id="rig",
            livox_topic=timesync.DEFAULT_LIVOX_TOPIC,
            d435i_topic=timesync.DEFAULT_D435I_TOPIC,
            stream_metadata={}, full=full, segments=segments,
            search={"segments": 3})
        encoded = json.dumps(document, allow_nan=False)
        self.assertTrue(encoded)
        self.assertEqual(document["schema"], timesync.SCHEMA)
        self.assertEqual(document["time_convention"]["equation"],
                         "t_d435i = t_livox + offset")
        self.assertIn("R_d435gyro_livoximu", document["result"])
        self.assertEqual(len(document["three_segment_refits"]["offsets_ms"]), 3)
        self.assertEqual(document["source_bag"]["bag_sha256"], "a" * 64)


if __name__ == "__main__":
    unittest.main()
