#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_mid360s_imu_poses as recorder


class UnitConversionTests(unittest.TestCase):
    def test_livox_driver_g_is_explicitly_converted_to_si(self):
        got = recorder.accel_g_to_ms2([1.0, -0.5, 0.0])
        np.testing.assert_allclose(got, [9.80665, -4.903325, 0.0], rtol=0, atol=1e-12)


class StableCollectorTests(unittest.TestCase):
    def test_current_mid360s_defaults_match_audited_half_second_profile(self):
        collector = recorder.StablePoseCollector()
        self.assertEqual(collector.hold_s, 0.5)
        self.assertEqual(collector.min_samples, 60)
        self.assertEqual(collector.direction_drift_limit_deg, 0.8)
        self.assertEqual(recorder.DEFAULT_FRAME, "livox_frame")

    def test_two_stable_direction_separated_poses_are_accepted_once(self):
        rng = np.random.default_rng(4)
        detector = recorder.StablePoseCollector(
            hold_s=0.5,
            min_samples=80,
            min_separation_deg=18.0,
        )
        stamp = 1_000_000_000
        events = []
        for _ in range(150):
            stamp += 5_000_000
            event = detector.ingest(
                stamp,
                np.array([0.0, 0.0, 1.0]) + rng.normal(0, 0.001, 3),
                np.array([0.003, -0.002, 0.001]) + rng.normal(0, 0.0005, 3),
            )
            if event is not None:
                events.append(event)
        self.assertEqual(len(events), 1)
        # A direction change re-arms the detector; the following stationary
        # interval is then recorded as the second pose.
        stamp += 5_000_000
        detector.ingest(stamp, [1.0, 0.0, 0.0], [0.2, 0.0, 0.0])
        for _ in range(150):
            stamp += 5_000_000
            event = detector.ingest(
                stamp,
                np.array([1.0, 0.0, 0.0]) + rng.normal(0, 0.001, 3),
                np.array([0.003, -0.002, 0.001]) + rng.normal(0, 0.0005, 3),
            )
            if event is not None:
                events.append(event)
        self.assertEqual(len(events), 2)
        self.assertEqual(len(detector.windows), 2)
        self.assertGreater(recorder.angle_deg(
            detector.pose_means[0], detector.pose_means[1]), 80.0)

    def test_capture_contains_raw_and_si_samples_without_pickle(self):
        stamps = np.arange(100, dtype=np.int64) * 5_000_000 + 1_000_000_000
        raw = np.tile([0.0, 0.0, 1.0], (100, 1))
        gyro = np.zeros((100, 3))
        window = recorder.StableWindow(
            stamps, raw, recorder.accel_g_to_ms2(raw), gyro, 0.0
        )
        metadata = {
            "schema": recorder.SCHEMA,
            "identity": {"mid360s_serial": "MID", "rig_id": "RIG"},
            "units": {"driver_accelerometer_input": "g"},
        }
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "capture.npz"
            recorder.save_capture(path, [window], metadata)
            with np.load(path, allow_pickle=False) as data:
                self.assertIn("sample_accel_raw_g", data.files)
                self.assertIn("sample_accel_ms2", data.files)
                loaded = json.loads(str(data["metadata_json"].reshape(()).item()))
                self.assertEqual(loaded["identity"]["mid360s_serial"], "MID")
                np.testing.assert_allclose(
                    data["sample_accel_ms2"],
                    data["sample_accel_raw_g"] * recorder.STANDARD_GRAVITY,
                )
            with self.assertRaises(FileExistsError):
                recorder.save_capture(path, [window], metadata)


if __name__ == "__main__":
    unittest.main()
