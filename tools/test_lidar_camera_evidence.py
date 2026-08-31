#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import os
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_camera_evidence as evidence


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "calibrate_lidar_camera.sh"
LAUNCHER = PROJECT / "start_d435_color.sh"
D435_SERIAL = "fixture-d435-open-source"
LIDAR_SERIAL = "fixture-livox-open-source"


def header(frame: str, ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        frame_id=frame,
        stamp=SimpleNamespace(sec=ns // 1_000_000_000,
                              nanosec=ns % 1_000_000_000),
    )


def camera_info(frame: str, ns: int, width: int, height: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(frame, ns), width=width, height=height,
        distortion_model="plumb_bob", d=[0.0] * 5,
        k=[500.0, 0.0, width / 2, 0.0, 500.0, height / 2, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[500.0, 0.0, width / 2, 0.0,
           0.0, 500.0, height / 2, 0.0,
           0.0, 0.0, 1.0, 0.0],
    )


def sample(topic: str, index: int, ns: int, frame: str, message=None,
           bag_delta_ns: int = 20_000_000) -> evidence.TimedSample:
    return evidence.TimedSample(
        topic=topic, index=index, header_ns=ns,
        bag_ns=ns + bag_delta_ns, frame_id=frame,
        message=message if message is not None else SimpleNamespace(),
    )


class StreamIntegrityTests(unittest.TestCase):
    def test_legacy_scene06_and_scene07_explicitly_fail_for_missing_depth(self):
        layout = evidence.TopicLayout()
        root = PROJECT / "data/lidar_camera_extrinsic_supplement/bags"
        checked = 0
        for scene in ("scene06", "scene07"):
            bag = root / scene
            if not (bag / "metadata.yaml").is_file():
                continue
            checked += 1
            with self.subTest(scene=scene), self.assertRaisesRegex(
                    evidence.EvidenceError, "missing required depth topic"):
                evidence.require_topic_types(
                    evidence.metadata_topic_types(bag), layout)
        if root.is_dir():
            self.assertEqual(checked, 2)

    def test_stale_sample_is_recorded_not_retimed(self):
        topic = "/rgb"
        good = sample(topic, 0, 1_000_000_000, evidence.COLOR_FRAME)
        stale = sample(topic, 1, 2_000_000_000, evidence.COLOR_FRAME,
                       bag_delta_ns=2_000_000_000)
        kept, removed = evidence.filter_stream(
            [good, stale], evidence.COLOR_FRAME, 250_000_000)
        self.assertEqual(kept, [good])
        self.assertEqual(removed[0]["header_ns"], stale.header_ns)
        self.assertEqual(removed[0]["bag_ns"], stale.bag_ns)

    def test_wrong_frame_and_nonmonotonic_stamps_fail(self):
        with self.assertRaisesRegex(evidence.EvidenceError, "wrong frame"):
            evidence.filter_stream(
                [sample("/rgb", 0, 1_000_000_000, "wrong")],
                evidence.COLOR_FRAME, 250_000_000)
        with self.assertRaisesRegex(evidence.EvidenceError, "non-monotonic"):
            evidence.filter_stream([
                sample("/rgb", 0, 2_000_000_000, evidence.COLOR_FRAME),
                sample("/rgb", 1, 1_900_000_000, evidence.COLOR_FRAME,
                       bag_delta_ns=200_000_000),
            ], evidence.COLOR_FRAME, 250_000_000)

    def test_nearest_header_sync_uses_no_invented_timestamp(self):
        layout = evidence.TopicLayout("/rgb", "/ci", "/depth", "/di", "/lidar")
        base = 10_000_000_000
        streams = {
            "/rgb": [sample("/rgb", 0, base, evidence.COLOR_FRAME)],
            "/ci": [sample("/ci", 0, base, evidence.COLOR_FRAME)],
            "/depth": [sample("/depth", 0, base + 2_000_000, evidence.DEPTH_FRAME)],
            "/di": [sample("/di", 0, base + 2_000_000, evidence.DEPTH_FRAME)],
            "/lidar": [sample("/lidar", 0, base + 12_000_000, evidence.LIDAR_FRAME)],
        }
        got = evidence.synchronize(
            streams, layout, 5_000_000, 20_000_000, 5_000_000, 1)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].depth.header_ns, base + 2_000_000)
        self.assertEqual(got[0].lidar.header_ns, base + 12_000_000)


class PayloadRoundTripTests(unittest.TestCase):
    @staticmethod
    def metric_provenance() -> dict:
        return {
            "source_tree_sha256": "a" * 64,
            "device_identity": {
                "d435i_serial": D435_SERIAL,
                "mid360s_serial": LIDAR_SERIAL,
            },
            "factory_metric_geometry": evidence.factory_metric_geometry({
                "serial": D435_SERIAL,
                "depth_scale_mm": 1.0000000474974513,
                "T_ir1_to_rgb": {
                    "t_mm": [14.8, -0.1, 0.3],
                    "R": [[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0]],
                },
            }),
        }

    @staticmethod
    def rgb_message(ns: int) -> SimpleNamespace:
        image = np.zeros((720, 1280, 3), np.uint8)
        image[100:120, 200:230] = (10, 80, 240)
        ok, encoded = cv2.imencode(".jpg", image)
        if not ok:
            raise AssertionError("OpenCV JPEG fixture failed")
        return SimpleNamespace(
            header=header(evidence.COLOR_FRAME, ns),
            format="jpeg", data=encoded.tobytes())

    @staticmethod
    def depth_message(ns: int) -> SimpleNamespace:
        depth = np.zeros((480, 848), np.uint16)
        depth[20:40, 30:60] = 1234
        return SimpleNamespace(
            header=header(evidence.DEPTH_FRAME, ns),
            height=480, width=848, encoding="16UC1", is_bigendian=0,
            step=848 * 2, data=depth.tobytes())

    @staticmethod
    def lidar_message(ns: int) -> SimpleNamespace:
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
            SimpleNamespace(name="intensity", offset=12, datatype=7, count=1),
            SimpleNamespace(name="tag", offset=16, datatype=2, count=1),
            SimpleNamespace(name="line", offset=17, datatype=2, count=1),
            SimpleNamespace(name="timestamp", offset=18, datatype=8, count=1),
        ]
        raw = bytearray(52)
        struct.pack_into("<ffffBBd", raw, 0, 1.0, 2.0, 3.0, 4.0, 5, 6, 0.001)
        struct.pack_into("<ffffBBd", raw, 26, -1.0, -2.0, 2.5, 8.0, 7, 8, 0.002)
        return SimpleNamespace(
            header=header(evidence.LIDAR_FRAME, ns),
            height=1, width=2, fields=fields, is_bigendian=False,
            point_step=26, row_step=52, data=bytes(raw), is_dense=True)

    def fixture_tuple(self, ordinal: int) -> evidence.SyncTuple:
        base = 20_000_000_000 + ordinal * 100_000_000
        rgb_msg = self.rgb_message(base)
        depth_msg = self.depth_message(base + 1_000_000)
        lidar_msg = self.lidar_message(base + 10_000_000)
        ci = camera_info(evidence.COLOR_FRAME, base, 1280, 720)
        di = camera_info(evidence.DEPTH_FRAME, base + 1_000_000, 848, 480)
        return evidence.SyncTuple(
            sample("/rgb", ordinal, base, evidence.COLOR_FRAME, rgb_msg),
            sample("/ci", ordinal, base, evidence.COLOR_FRAME, ci),
            sample("/depth", ordinal, base + 1_000_000,
                   evidence.DEPTH_FRAME, depth_msg),
            sample("/di", ordinal, base + 1_000_000,
                   evidence.DEPTH_FRAME, di),
            sample("/lidar", ordinal, base + 10_000_000,
                   evidence.LIDAR_FRAME, lidar_msg),
        )

    def test_evidence_preserves_organized_pixels_stamps_shapes_and_cloud_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "calibration" / "evidence" / "fixture"
            document = evidence.write_evidence(
                output, [self.fixture_tuple(0), self.fixture_tuple(1)],
                "calibration", self.metric_provenance(), [],
                {"max_lidar_rgb_header_delta_ms": 20.0})
            self.assertEqual(document["tuple_count"], 2)
            self.assertEqual(document["status"],
                             "evidence_only_not_a_calibration_result")
            self.assertEqual(document["devices"], {
                "d435i_serial": D435_SERIAL,
                "mid360s_serial": LIDAR_SERIAL,
            })
            frame = document["frames"][0]
            self.assertEqual(frame["rgb"]["shape_hwc"], [720, 1280, 3])
            self.assertEqual(frame["depth"]["shape_hw"], [480, 848])
            self.assertAlmostEqual(frame["depth"]["millimetres_per_unit"],
                                   1.0000000474974513)
            self.assertEqual(
                document["provenance"]["factory_metric_geometry"]
                ["depth_to_color_extrinsic"]["source_key"],
                "T_ir1_to_rgb")
            self.assertEqual(
                document["provenance"]["factory_metric_geometry"]
                ["depth_to_color_extrinsic"]["rotation_storage_in_factory_json"],
                "R contains three consecutive matrix columns")
            self.assertEqual(frame["lidar"]["frame_id"], evidence.LIDAR_FRAME)
            self.assertEqual(frame["delta_ns"]["lidar_minus_rgb"], 10_000_000)

            with np.load(output / frame["depth"]["file"], allow_pickle=False) as z:
                self.assertEqual(z["depth_u16"].shape, (480, 848))
                self.assertEqual(int(z["depth_u16"][20, 30]), 1234)
                self.assertEqual(int(z["source_linear_index"][479, 847]),
                                 480 * 848 - 1)
            with np.load(output / frame["lidar"]["file"], allow_pickle=False) as z:
                np.testing.assert_allclose(z["xyz"], [[1, 2, 3], [-1, -2, 2.5]])
                np.testing.assert_array_equal(z["point_index"], [0, 1])
                self.assertEqual(len(z["raw_data"]), 52)
            self.assertTrue((output / "manifest.json").is_file())
            self.assertTrue((output / "SHA256SUMS").is_file())

    def test_depth_padding_is_respected_and_lineage_is_row_major(self):
        width, height, step = 3, 2, 8
        raw = bytearray(height * step)
        struct.pack_into("<HHH", raw, 0, 1, 2, 3)
        struct.pack_into("<HHH", raw, step, 4, 5, 6)
        msg = SimpleNamespace(width=width, height=height, step=step,
                              encoding="16UC1", is_bigendian=0, data=raw)
        depth, lineage = evidence.organized_depth(msg)
        np.testing.assert_array_equal(depth, [[1, 2, 3], [4, 5, 6]])
        np.testing.assert_array_equal(lineage, [[0, 1, 2], [3, 4, 5]])

    def test_factory_depth_scale_is_metric_and_fails_closed(self):
        factory = {
            "serial": D435_SERIAL,
            "depth_scale_mm": 2.5,
            "T_ir1_to_rgb": {
                "t_mm": [1, 2, 3],
                # Outer groups are the columns of a +90-degree Z rotation.
                "R": [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],
            },
        }
        geometry = evidence.factory_metric_geometry(factory)
        self.assertEqual(geometry["depth_scale"]["metres_per_unit"], 0.0025)
        self.assertEqual(
            geometry["depth_to_color_extrinsic"]["rotation_row_major"],
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        factory["depth_scale_mm"] = float("nan")
        with self.assertRaisesRegex(evidence.EvidenceError, "finite number"):
            evidence.factory_metric_geometry(factory)


class RoleAndIdentityTests(unittest.TestCase):
    def valid_documents(self, bag: Path, role: str = "calibration"):
        camchain = bag.parent.parent.parent / "camchain.yaml"
        factory = bag.parent.parent.parent / "factory.json"
        camchain.write_text("fixture\n", encoding="utf-8")
        factory.write_text(json.dumps({
            "serial": D435_SERIAL,
            "depth_scale_mm": 1.0,
            "T_ir1_to_rgb": {
                "t_mm": [0.0, 0.0, 0.0],
                "R": [[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]],
            },
        }), encoding="utf-8")
        role_doc = {
            "schema": evidence.ROLE_SCHEMA, "scene": bag.name, "role": role,
            "registered_utc": "2026-09-01T00:00:00Z", "rig_id": "rig",
            "mount_session_id": "mount",
            "d435i_serial": D435_SERIAL,
            "mid360s_serial": LIDAR_SERIAL,
        }
        role_path = bag / "capture_role.json"
        role_path.write_text(json.dumps(role_doc), encoding="utf-8")
        manifest = {
            "schema": evidence.CAPTURE_SCHEMA, "scene": bag.name, "role": role,
            "recorded_utc": "2026-09-01T00:00:01Z", "rig_id": "rig",
            "mount_session_id": "mount",
            "d435i_serial": D435_SERIAL,
            "mid360s_serial": LIDAR_SERIAL,
            "rigid_mount_confirmed": True,
            "static_during_capture_confirmed": True,
            "topics": evidence.TopicLayout().types(),
            "frames": {"color": evidence.COLOR_FRAME,
                       "depth": evidence.DEPTH_FRAME,
                       "lidar": evidence.LIDAR_FRAME},
            "profiles": {
                "color": {"resolution": [1280, 720], "fps": 30,
                          "transport": "jpeg"},
                "depth": {"resolution": [848, 480], "fps": 30,
                          "encoding": "16UC1"},
            },
            "role_registration_sha256": evidence.sha256_file(role_path),
            "camchain_sha256": evidence.sha256_file(camchain),
            "factory_params_sha256": evidence.sha256_file(factory),
        }
        witness_types = dict(evidence.TopicLayout().types())
        witness_types[evidence.DEFAULT_IMAGE_TOPIC.removesuffix("/compressed")] = \
            evidence.TYPE_IMAGE
        manifest["publisher_witness"] = {
            topic: {"publisher_count": 1, "topic_type": message_type,
                    "node_name": "fixture", "gid": f"gid-{index}"}
            for index, (topic, message_type) in enumerate(witness_types.items())
        }
        return manifest, role_doc, camchain, factory

    def test_manifest_is_bound_to_same_camera_role_frame_and_hashes(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Path(temp) / "calibration" / "bags" / "sceneC"
            bag.mkdir(parents=True)
            manifest, role_doc, camchain, factory = self.valid_documents(bag)
            evidence.validate_capture_manifest(
                bag, manifest, role_doc, "calibration", evidence.TopicLayout(),
                camchain, factory, D435_SERIAL, LIDAR_SERIAL)
            manifest["d435i_serial"] = "another-camera"
            with self.assertRaisesRegex(evidence.EvidenceError, "differs from CLI"):
                evidence.validate_capture_manifest(
                    bag, manifest, role_doc, "calibration", evidence.TopicLayout(),
                    camchain, factory, D435_SERIAL, LIDAR_SERIAL)

    def test_role_directory_cannot_be_relabelled(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Path(temp) / "calibration" / "bags" / "sceneH"
            bag.mkdir(parents=True)
            manifest, role_doc, camchain, factory = self.valid_documents(
                bag, role="holdout")
            with self.assertRaisesRegex(evidence.EvidenceError, "physically isolated"):
                evidence.validate_capture_manifest(
                    bag, manifest, role_doc, "holdout", evidence.TopicLayout(),
                    camchain, factory, D435_SERIAL, LIDAR_SERIAL)


class ShellWorkflowTests(unittest.TestCase):
    def test_shell_syntax_and_isolated_paths(self):
        syntax = subprocess.run(["bash", "-n", str(SCRIPT)], cwd=PROJECT,
                                text=True, capture_output=True, check=False)
        self.assertEqual(syntax.returncode, 0, syntax.stderr)
        with tempfile.TemporaryDirectory() as temp:
            env = dict(os.environ)
            env["LIDAR_CAMERA_WORK_DIR"] = temp
            run = subprocess.run(
                ["bash", "--noprofile", "--norc", "-c",
                 'source "$1"; rgbd_bag_path calibration s; rgbd_bag_path holdout s',
                 "test", str(SCRIPT)], cwd=PROJECT, env=env,
                text=True, capture_output=True, check=False)
            self.assertEqual(run.returncode, 0, run.stderr)
            lines = run.stdout.splitlines()
            self.assertIn("/calibration/bags/s", lines[0])
            self.assertIn("/holdout/bags/s", lines[1])
            self.assertNotEqual(lines[0], lines[1])

    def test_bad_role_fails_before_any_hardware_probe(self):
        run = subprocess.run(
            ["bash", str(SCRIPT), "record-rgbd", "sceneX",
             "--role", "invalid", "--rigid-mounted"],
            cwd=PROJECT, text=True, capture_output=True, check=False)
        self.assertEqual(run.returncode, 2)
        self.assertIn("requires --role calibration|holdout", run.stderr)

    def test_role_is_written_before_capture_and_cannot_be_changed(self):
        with tempfile.TemporaryDirectory() as temp:
            env = dict(os.environ)
            env["LIDAR_CAMERA_WORK_DIR"] = temp
            command = r'''
source "$1"
ACTUAL_D435_SERIAL=fixture-d435-shell
ACTUAL_LIDAR_SERIAL=fixture-livox-shell
MOUNT_SESSION_ID=fixture-mount
role_file="$(reserve_rgbd_role sceneR calibration)" || exit
python3 - "$role_file" <<'PY'
import json,sys
d=json.load(open(sys.argv[1], encoding="utf-8"))
print(d["role"], d["d435i_serial"], d["mid360s_serial"])
PY
reserve_rgbd_role sceneR holdout
'''
            run = subprocess.run(
                ["bash", "--noprofile", "--norc", "-c", command,
                 "test", str(SCRIPT)], cwd=PROJECT, env=env,
                text=True, capture_output=True, check=False)
            self.assertEqual(run.returncode, 2)
            self.assertIn(
                "calibration fixture-d435-shell fixture-livox-shell", run.stdout)
            self.assertIn("already frozen", run.stderr)

    def test_launcher_rejects_non_boolean_depth_mode_before_usb_probe(self):
        env = dict(os.environ)
        env["D435I_EXPECTED_SERIAL"] = D435_SERIAL
        env["D435I_USB_SERIAL"] = "fixture-usb-descriptor"
        env["D435I_ENABLE_DEPTH"] = "maybe"
        run = subprocess.run(["bash", str(LAUNCHER)], cwd=PROJECT, env=env,
                             text=True, capture_output=True, check=False)
        self.assertEqual(run.returncode, 2)
        self.assertIn("D435I_ENABLE_DEPTH must be true or false", run.stderr)


if __name__ == "__main__":
    unittest.main()
