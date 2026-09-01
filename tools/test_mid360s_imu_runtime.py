#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mid360s_imu_runtime as runtime  # noqa: E402
from promote_mid360s_imu import ACCEL_EQUATION, OPERATIONAL_SCHEMA  # noqa: E402


def formal_result() -> dict:
    return {
        "schema_version": 1,
        "task_id": "mid360s_imu",
        "status": "operational",
        "local_schema": OPERATIONAL_SCHEMA,
        "created_utc": "2026-09-01T00:00:00Z",
        "rig_id": "rig",
        "mount_session_id": "mount",
        "devices": [{"role": "lidar", "model": "Livox MID-360S", "serial": "MID"}],
        "method": "multi-orientation static-gravity fit",
        "source_data": [{
            "role": "operational_capture",
            "path": "data/capture.npz",
            "sha256": "0" * 64,
        }],
        "frame_convention": {
            "frame": "mid360s_imu_frame",
            "accel_equation": ACCEL_EQUATION,
        },
        "result": {
            "accel_input_unit": "g",
            "accel_output_unit": "m/s^2",
            "accel_unit_scale_ms2_per_g": 9.80665,
            "T_misalignment": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "accel_scale": [1.01, 0.99, 1.02],
            "accel_misalignment_rad": [0.0, 0.0, 0.0],
            "accel_correction_matrix": [[1.01, 0.0, 0.0], [0.0, 0.99, 0.0], [0.0, 0.0, 1.02]],
            "accel_bias_ms2": [0.1, -0.2, 0.3],
            "gyro_bias_rad_s": [0.01, -0.02, 0.03],
            "pose_count": 15,
            "fit_pose_count": 12,
            "holdout_pose_count": 3,
            "accel_fit_residual_rms_ms2": 0.01,
            "accel_holdout_residual_rms_ms2": 0.02,
            "gyro_static_residual_rms_rad_s": 0.003,
            "gravity_reference_ms2": 9.79,
            "imu_sample_rate_hz": 200.0,
            "noise_window_duration_s": 0.5,
            "accel_noise_density_ms2_sqrt_hz": [0.01, 0.02, 0.03],
            "gyro_noise_density_rad_s_sqrt_hz": [0.001, 0.002, 0.003],
            "noise_density_method": "short_window_white_noise",
            "allan_characterization": "not_performed",
            "jacobian_rank": 9,
            "jacobian_parameter_count": 9,
            "jacobian_condition_column_normalized": 2.0,
        },
    }


class RuntimeMathTests(unittest.TestCase):
    def test_raw_g_matrix_bias_and_gyro_bias_only(self):
        calibration = runtime.calibration_from_document(formal_result())
        raw_accel = np.array([1.0, -0.5, 0.25])
        raw_gyro = np.array([0.5, -0.4, 0.3])
        accel, gyro = runtime.correct_measurement(raw_accel, raw_gyro, calibration)
        expected_accel = calibration.accel_matrix @ (
            9.80665 * raw_accel - calibration.accel_bias_ms2
        )
        np.testing.assert_allclose(accel, expected_accel, rtol=0, atol=1e-12)
        # No gyro scale or misalignment is applied: only the observable bias.
        np.testing.assert_allclose(
            gyro, raw_gyro - calibration.gyro_bias_rad_s, rtol=0, atol=1e-12
        )

    def test_covariance_is_density_squared_times_sample_rate(self):
        calibration = runtime.calibration_from_document(formal_result())
        np.testing.assert_allclose(
            np.diag(calibration.accel_covariance),
            np.square([0.01, 0.02, 0.03]) * 200.0,
        )
        np.testing.assert_allclose(
            np.diag(calibration.gyro_covariance),
            np.square([0.001, 0.002, 0.003]) * 200.0,
        )
        self.assertEqual(
            runtime.covariance_row_major(calibration.accel_covariance)[1], 0.0
        )

    def test_missing_noise_or_wrong_identity_fails_closed(self):
        document = formal_result()
        document["result"].pop("gyro_noise_density_rad_s_sqrt_hz")
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "gyro_noise_density"):
            runtime.calibration_from_document(document)
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "expected_serial"):
            runtime.calibration_from_document(
                formal_result(), expected_serial="ANOTHER"
            )

    def test_nonfinite_sample_is_rejected(self):
        calibration = runtime.calibration_from_document(formal_result())
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "NaN or Inf"):
            runtime.correct_measurement([1.0, float("nan"), 0.0], [0, 0, 0], calibration)

    def test_strict_result_loader_rejects_duplicate_json_key(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "result.json"
            path.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
            with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "duplicate"):
                runtime.load_runtime_calibration(path)

    def test_raw_lidar_frame_is_relabelled_as_distinct_imu_origin(self):
        self.assertEqual(
            runtime.output_frame_for_sample(
                "livox_frame", "livox_frame", "mid360s_imu_frame"
            ),
            "mid360s_imu_frame",
        )
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "frame_id mismatch"):
            runtime.output_frame_for_sample(
                "unexpected_frame", "livox_frame", "mid360s_imu_frame"
            )
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "output frame"):
            runtime.output_frame_for_sample("livox_frame", "livox_frame", "")
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "must be mid360s_imu_frame"):
            runtime.output_frame_for_sample(
                "livox_frame", "livox_frame", "base_link"
            )
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "must be livox_frame"):
            runtime.output_frame_for_sample(
                "base_link", "base_link", "mid360s_imu_frame"
            )

    def test_unphysical_matrix_and_orientation_claim_fail_closed(self):
        document = formal_result()
        document["result"]["accel_correction_matrix"] = [
            [1000.0, 0.0, 0.0],
            [0.0, 1000.0, 0.0],
            [0.0, 0.0, 1000.0],
        ]
        with self.assertRaisesRegex(runtime.RuntimeCalibrationError, "operational validation"):
            runtime.calibration_from_document(document)
        self.assertEqual(
            runtime.unavailable_orientation_covariance(),
            [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        document = formal_result()
        document["result"]["gyro_bias_rad_s"] = [1.0e300] * 3
        document["result"]["accel_noise_density_ms2_sqrt_hz"] = [1.0e308] * 3
        document["result"]["imu_sample_rate_hz"] = 1.0e308
        with self.assertRaisesRegex(
            runtime.RuntimeCalibrationError, "static-bias|noise density|sample_rate"
        ):
            runtime.calibration_from_document(document)

    def test_ros_arguments_are_forwarded_past_the_cli_checker(self):
        with mock.patch.object(runtime, "run_ros_node", return_value=0) as run:
            status = runtime.main([
                "--ros-args", "-p", "project_root:=/tmp/project"
            ])
        self.assertEqual(status, 0)
        run.assert_called_once_with(
            ["--ros-args", "-p", "project_root:=/tmp/project"]
        )


if __name__ == "__main__":
    unittest.main()
