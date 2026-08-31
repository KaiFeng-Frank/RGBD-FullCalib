#!/usr/bin/env python3
"""Regression tests for capture-set integrity in the LiDAR-camera wrapper."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "calibrate_lidar_camera.sh"
CAMCHAIN = PROJECT / "data" / "cam_rgb-camchain.yaml"
IMAGE = "/camera/camera/color/image_raw/compressed"
RAW_IMAGE = "/camera/camera/color/image_raw"
CAMERA_INFO = "/camera/camera/color/camera_info"
POINTS = "/livox/lidar"
D435_SERIAL = "947122070908"
LIDAR_SERIAL = "ARMDN6B0030122"
RIG_ID = "mid360s-d435i-01"
MOUNT_SESSION = "fixture-mount-session"


class CaptureAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.work = Path(self.temp.name) / "work"
        self.bags = self.work / "bags"
        self.bag = self.bags / "scene01"
        self.bag.mkdir(parents=True)
        self._write_fixture()

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def _canonical_json(path: Path, document: dict) -> None:
        path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2,
                       sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8")

    def _publisher_witness(self) -> dict:
        types = {
            RAW_IMAGE: "sensor_msgs/msg/Image",
            IMAGE: "sensor_msgs/msg/CompressedImage",
            CAMERA_INFO: "sensor_msgs/msg/CameraInfo",
            POINTS: "sensor_msgs/msg/PointCloud2",
        }
        return {
            topic: {
                "publisher_count": 1,
                "node_name": "fixture_publisher",
                "node_namespace": "/",
                "gid": f"fixture-gid-{index}",
                "topic_type": message_type,
            }
            for index, (topic, message_type) in enumerate(types.items())
        }

    def _write_fixture(self) -> None:
        topics = {
            IMAGE: "sensor_msgs/msg/CompressedImage",
            CAMERA_INFO: "sensor_msgs/msg/CameraInfo",
            POINTS: "sensor_msgs/msg/PointCloud2",
        }
        counts = {IMAGE: 450, CAMERA_INFO: 15, POINTS: 150}
        metadata = {
            "rosbag2_bagfile_information": {
                "storage_identifier": "sqlite3",
                "duration": {"nanoseconds": 15_000_000_000},
                "topics_with_message_count": [
                    {
                        "topic_metadata": {"name": topic, "type": message_type},
                        "message_count": counts[topic],
                    }
                    for topic, message_type in topics.items()
                ],
            },
        }
        self._canonical_json(self.bag / "metadata.yaml", metadata)
        (self.bag / "scene01_0.db3").write_bytes(b"immutable bag fixture")
        (self.bag / "rosbag_info.txt").write_text(
            "fixture: ros2 bag info witness\n", encoding="utf-8")
        manifest = {
            "schema": "d435i_calib/lidar_camera_capture/v1",
            "scene": "scene01",
            "duration_seconds_recorded": 15.0,
            "rigid_mount_confirmed": True,
            "static_during_capture_confirmed": True,
            "rig_id": RIG_ID,
            "mount_session_id": MOUNT_SESSION,
            "storage_id": "sqlite3",
            "topics": topics,
            "message_counts": counts,
            "publisher_witness": self._publisher_witness(),
            "d435i": {
                "serial_expected": D435_SERIAL,
                "serial_observed": D435_SERIAL,
            },
            "mid360s": {"serial_observed": LIDAR_SERIAL},
            "preprocess_camera_model": {
                "source_sha256": hashlib.sha256(CAMCHAIN.read_bytes()).hexdigest(),
            },
        }
        self._canonical_json(self.bag / "capture_manifest.json", manifest)
        self._canonical_json(self.work / "capture_session.json", {
            "schema": "d435i_calib/lidar_camera_mount_session/v1",
            "rig_id": RIG_ID,
            "mount_session_id": MOUNT_SESSION,
            "d435i_serial": D435_SERIAL,
            "mid360s_serial": LIDAR_SERIAL,
        })
        self._refresh_sums()

    def _refresh_sums(self) -> None:
        rows = []
        for path in sorted(self.bag.iterdir()):
            if path.is_file() and path.name != "SHA256SUMS":
                rows.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  ./{path.name}")
        (self.bag / "SHA256SUMS").write_text("\n".join(rows) + "\n",
                                                   encoding="utf-8")

    def _run_function(self, function: str) -> subprocess.CompletedProcess[str]:
        env = dict(os.environ)
        env.update({
            "LIDAR_CAMERA_WORK_DIR": str(self.work),
            "LIDAR_CAMERA_CAMCHAIN": str(CAMCHAIN),
        })
        return subprocess.run(
            ["bash", "--noprofile", "--norc", "-c",
             'source "$1"; "$2"', "audit-test", str(SCRIPT), function],
            cwd=PROJECT, env=env, text=True, capture_output=True, check=False)

    def test_complete_capture_set_is_canonical_and_identity_bound(self):
        run = self._run_function("audit_capture_bags")
        self.assertEqual(run.returncode, 0, run.stderr)
        document = json.loads(run.stdout)
        self.assertEqual(document["rig_id"], RIG_ID)
        self.assertEqual(document["mount_session_id"], MOUNT_SESSION)
        self.assertEqual(document["d435i_serial"], D435_SERIAL)
        self.assertEqual(document["mid360s_serial"], LIDAR_SERIAL)
        self.assertEqual([row["scene"] for row in document["captures"]],
                         ["scene01"])

    def test_mutating_bag_after_freeze_is_rejected(self):
        audit = self._run_function("audit_capture_bags")
        self.assertEqual(audit.returncode, 0, audit.stderr)
        preprocessed = self.work / "preprocessed"
        preprocessed.mkdir()
        self._canonical_json(preprocessed / "SOURCE_BAGS.json",
                             json.loads(audit.stdout))
        (self.bag / "scene01_0.db3").write_bytes(b"mutated after preprocess")
        run = self._run_function("verify_frozen_capture_bags")
        self.assertNotEqual(run.returncode, 0)
        self.assertIn("SHA-256 mismatch", run.stderr)

    def test_partial_scene_directory_is_not_silently_ignored(self):
        (self.bags / "partial-scene").mkdir()
        run = self._run_function("audit_capture_bags")
        self.assertNotEqual(run.returncode, 0)
        self.assertIn("incomplete/partial capture", run.stderr)

    def test_bad_publisher_witness_is_rejected_even_with_fresh_hashes(self):
        manifest_path = self.bag / "capture_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["publisher_witness"][POINTS]["publisher_count"] = 2
        self._canonical_json(manifest_path, manifest)
        self._refresh_sums()
        run = self._run_function("audit_capture_bags")
        self.assertNotEqual(run.returncode, 0)
        self.assertIn("invalid unique-publisher witness", run.stderr)


if __name__ == "__main__":
    unittest.main()
