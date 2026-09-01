#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
PROJECT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import mid360s_imu_pipeline_node as pipeline_node  # noqa: E402
import promote_mid360s_imu as promotion  # noqa: E402
from test_promote_mid360s_imu import IDENTITY, build_fixture  # noqa: E402


def parameters(root: Path) -> dict:
    return {
        "project_root": str(root),
        "manifest": promotion.DEFAULT_MANIFEST,
        "work_dir": "work/runs",
        "output": promotion.DEFAULT_OUTPUT,
        "topic": "/livox/imu",
        "frame": "livox_frame",
        "mid360s_serial": IDENTITY["mid360s_serial"],
        "rig_id": IDENTITY["rig_id"],
        "mount_session_id": IDENTITY["mount_id"],
        "inputs": ["data/capture.npz"],
        "fit_poses": 12,
        "holdout_poses": 3,
        "live_hold_s": 0.5,
        "bag_hold_s": 0.5,
        "min_samples": 60,
        "min_separation_deg": 18.0,
        "latitude_deg": 22.3,
        "altitude_m": 30.0,
        "python": sys.executable,
        "verify_existing": False,
    }


class PipelineCommandTests(unittest.TestCase):
    def test_input_list_is_kept_as_literal_argv_without_a_shell(self):
        with tempfile.TemporaryDirectory(prefix="mid360s-command-") as temp:
            executable = Path(temp) / "pipeline.sh"
            executable.write_text("#!/bin/sh\n", encoding="utf-8")
            values = parameters(Path(temp))
            dangerous = "--python"
            values["inputs"] = [dangerous, "/tmp/attacker-python"]
            values["verify_existing"] = True
            command = pipeline_node.build_pipeline_command(executable, values)
            input_pairs = [
                command[index:index + 2]
                for index, value in enumerate(command)
                if value == "--input"
            ]
            self.assertEqual(
                input_pairs,
                [["--input", dangerous], ["--input", "/tmp/attacker-python"]],
            )
            self.assertEqual(command[-1], "--verify-existing")
            self.assertNotIn("-c", command)

    def test_nonfinite_parameters_fail_closed(self):
        with tempfile.TemporaryDirectory(prefix="mid360s-command-") as temp:
            executable = Path(temp) / "pipeline.sh"
            executable.write_text("#!/bin/sh\n", encoding="utf-8")
            for name, value in (
                ("latitude_deg", math.nan),
                ("altitude_m", math.inf),
                ("live_hold_s", math.nan),
            ):
                with self.subTest(name=name):
                    values = parameters(Path(temp))
                    values[name] = value
                    with self.assertRaises(pipeline_node.PipelineParameterError):
                        pipeline_node.build_pipeline_command(executable, values)


class PipelineIntegrationTests(unittest.TestCase):
    @unittest.skipUnless(
        Path("/opt/ros/jazzy/setup.bash").is_file()
        and Path("/usr/bin/python3").is_file(),
        "ROS 2 Jazzy is required for the parameter-file regression",
    )
    def test_ros_yaml_live_input_sentinel_is_initialized(self):
        with tempfile.TemporaryDirectory(prefix="mid360s-ros-params-") as temp:
            root = Path(temp)
            (root / "data" / "lidar_camera_extrinsic").mkdir(parents=True)
            (root / "results").mkdir()
            (root / promotion.DEFAULT_MANIFEST).write_text("{}\n", encoding="utf-8")
            (root / promotion.DEFAULT_OUTPUT).write_text("{}\n", encoding="utf-8")
            script = (
                'set +u; source "$1"; set -u; exec "$2" -B "$3" '
                '--ros-args --params-file "$4" -p "project_root:=$5"'
            )
            completed = subprocess.run(
                [
                    "bash", "-c", script, "bash",
                    "/opt/ros/jazzy/setup.bash",
                    "/usr/bin/python3",
                    str(TOOLS / "mid360s_imu_pipeline_node.py"),
                    str(PROJECT / "config" / "mid360s_imu_calibration.yaml"),
                    str(root),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 2)
            combined = completed.stdout + completed.stderr
            self.assertIn("starting fail-closed", combined)
            self.assertIn("refusing to overwrite", combined)
            self.assertNotIn("not initialized", combined)

    def test_offline_end_to_end_and_safe_resume(self):
        with tempfile.TemporaryDirectory(prefix="mid360s-pipeline-") as temp:
            root = Path(temp)
            paths = build_fixture(root)
            command = [
                str(PROJECT / "calibrate_mid360s_imu.sh"),
                "--project-root", str(root),
                "--manifest", promotion.DEFAULT_MANIFEST,
                "--work-dir", "work/runs",
                "--output", promotion.DEFAULT_OUTPUT,
                "--inputs", "data/capture.npz",
                "--python", sys.executable,
                "--serial", IDENTITY["mid360s_serial"],
                "--rig-id", IDENTITY["rig_id"],
                "--mount-id", IDENTITY["mount_id"],
            ]
            environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
            first = subprocess.run(
                command,
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )
            self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
            self.assertTrue(paths["output"].is_file())
            self.assertIn("viewer summary mid360s_imu=done", first.stdout)
            before = paths["output"].read_bytes()
            self.assertNotIn(str(root).encode(), before)

            overwrite = subprocess.run(
                command,
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )
            self.assertNotEqual(overwrite.returncode, 0)
            self.assertIn("refusing to overwrite", overwrite.stderr)
            self.assertEqual(paths["output"].read_bytes(), before)

            resumed = subprocess.run(
                command + ["--verify-existing"],
                text=True,
                capture_output=True,
                env=environment,
                check=False,
            )
            self.assertEqual(resumed.returncode, 0, resumed.stdout + resumed.stderr)
            self.assertIn("verified existing", resumed.stdout)
            self.assertEqual(paths["output"].read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
