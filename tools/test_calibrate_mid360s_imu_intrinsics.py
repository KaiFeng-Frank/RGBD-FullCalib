#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibrate_mid360s_imu_intrinsics as calibration
import record_mid360s_imu_poses as recorder
from imu_intrinsic import build_A


def fibonacci_directions(n: int) -> np.ndarray:
    golden = math.pi * (3.0 - math.sqrt(5.0))
    rows = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        rows.append([radius * math.cos(i * golden), radius * math.sin(i * golden), z])
    return np.asarray(rows)


def synthetic_capture(directions: np.ndarray, seed: int = 20260901) -> tuple[dict, np.ndarray]:
    rng = np.random.default_rng(seed)
    gravity = 9.787673065883645
    true_p = np.array([
        -0.0030, 0.0017, -0.0038,
        0.9996, 1.0065, 1.0071,
        -0.0165, 0.0262, -0.0141,
    ])
    matrix, bias = build_A(true_p)
    gyro_bias = np.array([-0.0071, 0.00052, -0.0102])
    windows = []
    for pose, direction in enumerate(directions):
        true_a = gravity * direction / np.linalg.norm(direction)
        measured = np.linalg.solve(matrix, true_a) + bias
        n = 120
        accel_ms2 = measured + rng.normal(0.0, 0.012, (n, 3))
        raw_g = accel_ms2 / recorder.STANDARD_GRAVITY
        gyro = gyro_bias + rng.normal(0.0, 0.0015, (n, 3))
        stamps = (
            1_700_000_000_000_000_000
            + pose * 2_000_000_000
            + np.arange(n, dtype=np.int64) * 5_000_000
        )
        q = max(2, n // 4)
        drift = recorder.angle_deg(
            accel_ms2[:q].mean(axis=0), accel_ms2[-q:].mean(axis=0)
        )
        windows.append(recorder.StableWindow(stamps, raw_g, accel_ms2, gyro, drift))
    metadata = {
        "schema": recorder.SCHEMA,
        "identity": {
            "mid360s_serial": "ARMDN6B0030122",
            "rig_id": "test_rig",
            "mount_id": "test_mount",
        },
        "source": {"role": "operational_capture", "ros_topic": "/livox/imu"},
        "units": {
            "driver_accelerometer_input": "g",
            "standard_gravity_ms2": recorder.STANDARD_GRAVITY,
        },
    }
    capture = recorder.build_capture_arrays(windows, metadata)
    capture["metadata"] = metadata
    capture["window_summaries"] = json.loads(
        str(capture["window_summaries_json"].reshape(()).item())
    )
    return capture, true_p


class SolverTests(unittest.TestCase):
    def test_pose_means_and_stability_claims_are_recomputed_from_samples(self):
        capture, _ = synthetic_capture(fibonacci_directions(17))
        forged = dict(capture)
        forged["pose_accel_mean_ms2"] = capture["pose_accel_mean_ms2"].copy()
        forged["pose_accel_mean_ms2"][0, 0] += 1.0
        with self.assertRaisesRegex(calibration.CalibrationError, "raw-sample mean"):
            calibration.validate_capture(forged)

        short = dict(capture)
        short["metadata"] = dict(capture["metadata"])
        short["metadata"]["stable_detector"] = {
            "hold_s": 0.5,
            "min_samples": 121,
            "min_separation_deg": 18.0,
            "gyro_mean_limit_deg_s": 4.0,
            "gyro_std_limit_deg_s": 2.5,
            "accel_std_limit_ms2": 0.35,
            "direction_drift_limit_deg": 0.8,
        }
        with self.assertRaisesRegex(calibration.CalibrationError, "sample/duration"):
            calibration.validate_capture(short)

    def test_well_covered_data_recovers_model_and_passes_holdout(self):
        capture, true_p = synthetic_capture(fibonacci_directions(17))
        document, arrays = calibration.analyze_capture(
            capture,
            gravity_ms2=9.787673065883645,
            minimum_fit_poses=12,
            desired_holdout_poses=3,
        )
        self.assertEqual(document["status"], "accepted")
        self.assertTrue(document["acceptance"]["passed"])
        self.assertEqual(document["validation"]["holdout_pose_count"], 3)
        self.assertEqual(document["observability"]["jacobian_rank"], 9)
        self.assertLess(
            document["observability"]["jacobian_condition_column_normalized"], 20.0
        )
        np.testing.assert_allclose(arrays["full_parameters"][:6], true_p[:6], atol=0.003)
        np.testing.assert_allclose(arrays["full_parameters"][6:], true_p[6:], atol=0.02)
        self.assertEqual(
            document["frame_convention"]["accel_equation"], calibration.ACCEL_EQUATION
        )
        self.assertEqual(
            document["stationary_noise"]["noise_density_method"],
            "short_window_white_noise",
        )
        self.assertEqual(
            document["stationary_noise"]["allan_characterization"], "not_performed"
        )
        self.assertEqual(len(document["stationary_noise"]["gyro_noise_density_rad_s_sqrt_hz"]), 3)

    def test_one_sided_orientations_are_rejected(self):
        phi = np.linspace(0.0, 2.0 * np.pi, 15, endpoint=False)
        directions = np.column_stack((0.15 * np.cos(phi), 0.15 * np.sin(phi), np.ones(15)))
        capture, _ = synthetic_capture(directions)
        document, _ = calibration.analyze_capture(
            capture,
            gravity_ms2=9.787673065883645,
            minimum_fit_poses=12,
            desired_holdout_poses=3,
        )
        self.assertEqual(document["status"], "rejected")
        reasons = " ".join(document["acceptance"]["rejection_reasons"])
        self.assertTrue("one-sided" in reasons or "both signs" in reasons or "rank" in reasons)

    def test_less_than_twelve_fit_poses_is_refused(self):
        capture, _ = synthetic_capture(fibonacci_directions(11))
        with self.assertRaisesRegex(calibration.CalibrationError, "at least 12"):
            calibration.analyze_capture(
                capture,
                gravity_ms2=9.787673065883645,
                minimum_fit_poses=12,
                desired_holdout_poses=3,
            )


class OfflineSegmentationTests(unittest.TestCase):
    def test_best_stationary_window_is_selected_per_direction(self):
        rng = np.random.default_rng(7)
        raw_parts = []
        gyro_parts = []
        stamp_parts = []
        stamp = 1_700_000_000_000_000_000
        for direction in ([0, 0, 1], [1, 0, 0], [0, -1, 0]):
            # A brief motion separates plateaus.
            n_move = 50
            stamp_parts.append(stamp + np.arange(n_move) * 5_000_000)
            raw_parts.append(rng.normal(0, 0.2, (n_move, 3)))
            gyro_parts.append(rng.normal(0, 0.4, (n_move, 3)))
            stamp += n_move * 5_000_000
            n = 180
            stamp_parts.append(stamp + np.arange(n) * 5_000_000)
            raw_parts.append(np.asarray(direction) + rng.normal(0, 0.002, (n, 3)))
            gyro_parts.append(np.array([0.005, 0.0, -0.004]) + rng.normal(0, 0.002, (n, 3)))
            stamp += n * 5_000_000
        windows = calibration.extract_stable_windows_from_series(
            np.concatenate(stamp_parts).astype(np.int64),
            np.concatenate(raw_parts),
            np.concatenate(gyro_parts),
            hold_s=0.5,
            min_samples=60,
            min_separation_deg=18.0,
        )
        self.assertEqual(len(windows), 3)
        means = [w.accel_mean_ms2 for w in windows]
        self.assertGreater(max(calibration.angle_deg(means[0], x) for x in means[1:]), 80.0)


class OutputSafetyTests(unittest.TestCase):
    def test_results_directory_is_refused_and_existing_analysis_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.assertRaisesRegex(calibration.CalibrationError, "analysis only"):
                calibration._refuse_formal_results_path(root / "results" / "model.json")
            document = {"status": "accepted"}
            arrays = {"x": np.arange(3)}
            out_json = root / "analysis" / "model.json"
            out_npz = root / "analysis" / "model.npz"
            calibration.write_analysis(out_json, out_npz, document, arrays)
            with self.assertRaises(FileExistsError):
                calibration.write_analysis(out_json, out_npz, document, arrays)


if __name__ == "__main__":
    unittest.main()
