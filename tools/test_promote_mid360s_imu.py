#!/usr/bin/env python3
from __future__ import annotations

import copy
import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import calibrate_mid360s_imu_intrinsics as calibration  # noqa: E402
import promote_mid360s_imu as promotion  # noqa: E402
from test_calibrate_mid360s_imu_intrinsics import (  # noqa: E402
    fibonacci_directions,
    synthetic_capture,
)


IDENTITY = {
    "mid360s_serial": "MID-TEST-001",
    "rig_id": "rig-test",
    "mount_id": "mount-test",
}


def build_fixture(root: Path) -> dict[str, Path]:
    (root / "data" / "lidar_camera_extrinsic").mkdir(parents=True)
    (root / "results").mkdir()
    (root / "work").mkdir()
    manifest = {
        "schema": promotion.MANIFEST_SCHEMA,
        "rig_id": IDENTITY["rig_id"],
        "mount_session_id": IDENTITY["mount_id"],
        "mid360s_serial": IDENTITY["mid360s_serial"],
        "d435i_serial": "D435-TEST-001",
    }
    manifest_path = root / promotion.DEFAULT_MANIFEST
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    capture, _ = synthetic_capture(fibonacci_directions(17))
    metadata = copy.deepcopy(capture["metadata"])
    metadata["identity"] = dict(IDENTITY)
    metadata["source"].update({
        "role": "operational_capture",
        "ros_topic": "/livox/imu",
        "frame_id": "livox_frame",
        "message_type": "sensor_msgs/msg/Imu",
    })
    metadata["units"].update({
        "stored_accelerometer_raw": "g",
        "stored_accelerometer_si": "m/s^2",
        "conversion": "accel_ms2 = accel_driver_g * 9.80665",
        "standard_gravity_ms2": 9.80665,
    })
    metadata["stable_detector"] = {
        "hold_s": 0.5,
        "min_samples": 60,
        "min_separation_deg": 18.0,
        "gyro_mean_limit_deg_s": 4.0,
        "gyro_std_limit_deg_s": 2.5,
        "accel_std_limit_ms2": 0.35,
        "direction_drift_limit_deg": 0.8,
    }
    capture["metadata_json"] = np.asarray(
        json.dumps(metadata, ensure_ascii=False, sort_keys=True)
    )
    capture_path = root / "data" / "capture.npz"
    with capture_path.open("xb") as stream:
        np.savez_compressed(
            stream,
            **{key: value for key, value in capture.items()
               if isinstance(value, np.ndarray)},
        )

    loaded = calibration.load_capture_npz(capture_path)
    gravity = 9.787673065883645
    analysis, arrays = calibration.analyze_capture(
        loaded,
        gravity_ms2=gravity,
        minimum_fit_poses=12,
        desired_holdout_poses=3,
    )
    analysis["source"]["inputs"] = [{
        "path": "data/capture.npz",
        "sha256": calibration.sha256_path(capture_path),
    }]
    analysis["source"]["expected_ros_frame"] = "livox_frame"
    analysis["gravity_reference"] = {
        "value_ms2": gravity,
        "method": "explicit_override",
        "latitude_deg": None,
        "altitude_m": None,
    }
    arrays["analysis_json"] = np.asarray(
        json.dumps(analysis, ensure_ascii=False, sort_keys=True)
    )
    analysis_path = root / "work" / "analysis.json"
    analysis_npz_path = root / "work" / "analysis.npz"
    analysis_path.write_text(
        json.dumps(analysis, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    with analysis_npz_path.open("xb") as stream:
        np.savez_compressed(stream, **arrays)
    return {
        "manifest": manifest_path,
        "capture": capture_path,
        "analysis": analysis_path,
        "analysis_npz": analysis_npz_path,
        "output": root / promotion.DEFAULT_OUTPUT,
    }


class PromotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mid360s-promote-test-")
        self.root = Path(self.temp.name)
        self.paths = build_fixture(self.root)

    def tearDown(self):
        self.temp.cleanup()

    def promote(self, **overrides):
        arguments = {
            "project_root": self.root,
            "manifest_path": Path(promotion.DEFAULT_MANIFEST),
            "analysis_path": Path("work/analysis.json"),
            "analysis_npz_path": Path("work/analysis.npz"),
            "output_path": Path(promotion.DEFAULT_OUTPUT),
            "expected_frame": "livox_frame",
            "expected_serial": IDENTITY["mid360s_serial"],
            "expected_rig_id": IDENTITY["rig_id"],
            "expected_mount_id": IDENTITY["mount_id"],
        }
        arguments.update(overrides)
        return promotion.promote(**arguments)

    def rewrite_analysis(self, mutate):
        document = promotion.load_strict_json(self.paths["analysis"])
        mutate(document)
        self.paths["analysis"].write_text(
            json.dumps(document, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        with np.load(self.paths["analysis_npz"], allow_pickle=False) as source:
            arrays = {name: np.asarray(source[name]).copy() for name in source.files}
        arrays["analysis_json"] = np.asarray(json.dumps(document, sort_keys=True))
        self.paths["analysis_npz"].unlink()
        with self.paths["analysis_npz"].open("xb") as stream:
            np.savez_compressed(stream, **arrays)

    def test_success_is_portable_viewer_done_and_exclusive(self):
        summary = self.promote()
        self.assertEqual(summary["viewer_summary"], "done")
        document = promotion.load_strict_json(self.paths["output"])
        self.assertEqual(document["rig_id"], IDENTITY["rig_id"])
        encoded = self.paths["output"].read_text(encoding="utf-8")
        self.assertNotIn(str(self.root), encoded)
        self.assertEqual(
            [row["path"] for row in document["source_data"]],
            ["data/capture.npz", "work/analysis.json", "work/analysis.npz"],
        )
        before = self.paths["output"].read_bytes()
        with self.assertRaisesRegex(promotion.PromotionError, "overwrite"):
            self.promote()
        self.assertEqual(self.paths["output"].read_bytes(), before)

    def test_verify_existing_recomputes_without_replacing(self):
        self.promote()
        before = self.paths["output"].read_bytes()
        summary = self.promote(verify_existing=True)
        self.assertIn("matches new analysis", summary["verification"])
        self.assertEqual(self.paths["output"].read_bytes(), before)

        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            status = promotion.main([
                "--project-root", str(self.root),
                "--manifest", promotion.DEFAULT_MANIFEST,
                "--analysis", "work/analysis.json",
                "--analysis-npz", "work/analysis.npz",
                "--output", promotion.DEFAULT_OUTPUT,
                "--expected-frame", "livox_frame",
                "--expected-serial", IDENTITY["mid360s_serial"],
                "--expected-rig-id", IDENTITY["rig_id"],
                "--expected-mount-id", IDENTITY["mount_id"],
                "--verify-existing",
            ])
        self.assertEqual(status, 0, stdout.getvalue())
        self.assertIn('"verification"', stdout.getvalue())
        self.assertEqual(self.paths["output"].read_bytes(), before)

    def test_verify_existing_requires_complete_analysis_provenance(self):
        self.promote()
        document = promotion.load_strict_json(self.paths["output"])
        document["source_data"] = [
            row
            for row in document["source_data"]
            if row["role"] == "operational_capture"
        ]
        self.paths["output"].write_text(
            json.dumps(document, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(
            promotion.PromotionError, "exactly one accepted_analysis"
        ):
            self.promote(verify_existing=True)

    def test_preflight_cli_emits_manifest_identity(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            status = promotion.main([
                "--project-root", str(self.root),
                "--manifest", promotion.DEFAULT_MANIFEST,
                "--preflight",
            ])
        self.assertEqual(status, 0)
        self.assertEqual(
            stdout.getvalue().splitlines(),
            [IDENTITY["rig_id"], IDENTITY["mount_id"], IDENTITY["mid360s_serial"], "D435-TEST-001"],
        )

    def test_rejected_analysis_and_weakened_gate_fail_closed(self):
        self.rewrite_analysis(lambda doc: doc.update(status="rejected"))
        with self.assertRaisesRegex(promotion.PromotionError, "not accepted"):
            self.promote()
        self.assertFalse(self.paths["output"].exists())

        self.paths = build_fixture(self.root / "second")
        old_root, self.root = self.root, self.root / "second"
        try:
            self.rewrite_analysis(
                lambda doc: doc["observability"].update(jacobian_rank=8)
            )
            with self.assertRaisesRegex(promotion.PromotionError, "full rank"):
                self.promote()
            self.assertFalse(self.paths["output"].exists())
        finally:
            self.root = old_root

    def test_manifest_identity_and_source_hash_are_recomputed(self):
        with self.assertRaisesRegex(promotion.PromotionError, "expected mid360s_serial"):
            self.promote(expected_serial="OTHER-SERIAL")
        self.assertFalse(self.paths["output"].exists())
        with self.paths["capture"].open("ab") as stream:
            stream.write(b"mutation")
        with self.assertRaisesRegex(promotion.PromotionError, "hash mismatch"):
            self.promote()
        self.assertFalse(self.paths["output"].exists())

    def test_npz_numeric_payload_must_match_json(self):
        with np.load(self.paths["analysis_npz"], allow_pickle=False) as source:
            arrays = {name: np.asarray(source[name]).copy() for name in source.files}
        arrays["full_residual_ms2"][0] += 1.0
        self.paths["analysis_npz"].unlink()
        with self.paths["analysis_npz"].open("xb") as stream:
            np.savez_compressed(stream, **arrays)
        with self.assertRaisesRegex(
            promotion.PromotionError, "elementwise|does not reproduce"
        ):
            self.promote()
        self.assertFalse(self.paths["output"].exists())

    def test_npz_residual_sign_cannot_hide_behind_absolute_statistics(self):
        with np.load(self.paths["analysis_npz"], allow_pickle=False) as source:
            arrays = {name: np.asarray(source[name]).copy() for name in source.files}
        for name in (
            "full_residual_ms2",
            "training_residual_ms2",
            "holdout_residual_ms2",
        ):
            arrays[name] *= -1.0
        self.paths["analysis_npz"].unlink()
        with self.paths["analysis_npz"].open("xb") as stream:
            np.savez_compressed(stream, **arrays)
        with self.assertRaisesRegex(promotion.PromotionError, "elementwise"):
            self.promote()
        self.assertFalse(self.paths["output"].exists())


if __name__ == "__main__":
    unittest.main()
