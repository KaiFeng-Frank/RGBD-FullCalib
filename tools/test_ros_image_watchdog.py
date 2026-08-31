#!/usr/bin/env python3
"""Deterministic tests for D435 color-stream disconnect supervision."""

from __future__ import annotations

import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest


TOOLS = Path(__file__).resolve().parent
PROJECT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

from ros_image_watchdog import (  # noqa: E402
    ImageHeartbeat,
    SustainedAbsence,
    image_is_valid,
    image_stamp_ns,
    observe_image,
    positive_seconds,
    usb_serial_present,
)


def image(height=1, width=2, step=6, data=bytes(6), stamp_ns=1):
    stamp = SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return SimpleNamespace(
        height=height,
        width=width,
        step=step,
        data=data,
        header=SimpleNamespace(stamp=stamp),
    )


class ImageHeartbeatTests(unittest.TestCase):
    def test_first_frame_has_hard_twenty_second_deadline(self):
        state = ImageHeartbeat(10.0, startup_timeout=20.0, loss_timeout=5.0)
        self.assertEqual(state.status(29.999), "starting")
        self.assertTrue(observe_image(state, image(stamp_ns=1), 29.999))

        late = ImageHeartbeat(10.0, startup_timeout=20.0, loss_timeout=5.0)
        self.assertEqual(late.status(30.0), "startup-timeout")
        self.assertFalse(observe_image(late, image(stamp_ns=1), 30.0))
        self.assertIsNone(late.last_frame_at)

    def test_active_stream_has_hard_five_second_deadline(self):
        state = ImageHeartbeat(0.0, startup_timeout=20.0, loss_timeout=5.0)
        self.assertTrue(observe_image(state, image(stamp_ns=1), 1.0))
        self.assertEqual(state.status(5.999), "healthy")
        self.assertTrue(observe_image(state, image(stamp_ns=2), 5.999))
        self.assertEqual(state.status(10.998), "healthy")
        self.assertEqual(state.status(10.999), "stream-timeout")
        self.assertFalse(observe_image(state, image(stamp_ns=3), 11.0))
        self.assertEqual(state.last_frame_at, 5.999)

    def test_invalid_images_never_arm_or_refresh_stream(self):
        state = ImageHeartbeat(0.0, startup_timeout=20.0, loss_timeout=5.0)
        invalid = (
            object(),
            image(height=0),
            image(width=0),
            image(step=0),
            image(width=2, step=1, data=bytes(2)),
            image(data=b""),
            image(height=2, width=2, step=6, data=bytes(11)),
            image(height="bad"),
            SimpleNamespace(height=1, width=1, step=1, data=None),
            SimpleNamespace(height=1, width=1, step=1, data=b"x"),
            image(stamp_ns=-1),
        )
        for message in invalid:
            self.assertFalse(observe_image(state, message, 1.0), message)
        self.assertIsNone(state.last_frame_at)

        self.assertTrue(observe_image(state, image(stamp_ns=10), 2.0))
        self.assertFalse(observe_image(
            state, image(data=b"", stamp_ns=11), 4.0))
        self.assertEqual(state.last_frame_at, 2.0)
        self.assertEqual(state.status(7.0), "stream-timeout")

    def test_complete_or_padded_payload_is_valid(self):
        self.assertTrue(image_is_valid(image(height=2, width=2, step=6,
                                             data=bytes(12))))
        self.assertTrue(image_is_valid(image(height=2, width=2, step=6,
                                             data=bytes(16))))

    def test_ros_timestamp_must_be_well_formed_and_advance(self):
        self.assertEqual(image_stamp_ns(image(stamp_ns=2_000_000_003)),
                         2_000_000_003)
        malformed = image()
        malformed.header.stamp.nanosec = 1_000_000_000
        self.assertIsNone(image_stamp_ns(malformed))
        malformed.header.stamp.nanosec = True
        self.assertIsNone(image_stamp_ns(malformed))

        state = ImageHeartbeat(0.0, startup_timeout=20.0, loss_timeout=5.0)
        self.assertTrue(observe_image(state, image(stamp_ns=100), 1.0))
        self.assertFalse(observe_image(state, image(stamp_ns=100), 2.0))
        self.assertFalse(observe_image(state, image(stamp_ns=99), 3.0))
        self.assertEqual(state.last_frame_at, 1.0)
        self.assertTrue(observe_image(state, image(stamp_ns=101), 4.0))
        self.assertEqual(state.last_frame_at, 4.0)

    def test_invalid_timing_configuration_is_rejected(self):
        for value in (0.0, -1.0, math.inf, -math.inf, math.nan):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ImageHeartbeat(0.0, value, 5.0)
                with self.assertRaises(ValueError):
                    ImageHeartbeat(0.0, 20.0, value)

    def test_non_monotonic_observation_is_rejected(self):
        state = ImageHeartbeat(10.0, startup_timeout=20.0, loss_timeout=5.0)
        with self.assertRaises(ValueError):
            state.status(9.0)
        self.assertTrue(observe_image(state, image(), 12.0))
        with self.assertRaises(ValueError):
            state.status(11.0)

    def test_cli_seconds_must_be_positive_and_finite(self):
        self.assertEqual(positive_seconds("0.25"), 0.25)
        for value in ("0", "-1", "nan", "inf", "-inf"):
            with self.subTest(value=value):
                with self.assertRaises(Exception):
                    positive_seconds(value)

    def test_sustained_usb_absence_has_three_second_deadline(self):
        state = SustainedAbsence(timeout=3.0)
        self.assertFalse(state.observe(True, 0.0))
        self.assertFalse(state.observe(False, 1.0))
        self.assertFalse(state.observe(False, 3.999))
        self.assertTrue(state.observe(False, 4.0))

    def test_brief_usb_absence_does_not_accumulate(self):
        state = SustainedAbsence(timeout=3.0)
        self.assertFalse(state.observe(False, 1.0))
        self.assertFalse(state.observe(True, 2.0))
        self.assertFalse(state.observe(False, 3.0))
        self.assertFalse(state.observe(False, 5.999))
        self.assertTrue(state.observe(False, 6.0))
        with self.assertRaises(ValueError):
            state.observe(True, 5.0)

    def test_usb_serial_scan_is_exact(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "1-1").mkdir()
            (root / "1-1" / "serial").write_text("947122070908\n",
                                                   encoding="utf-8")
            (root / "2-1").mkdir()
            (root / "2-1" / "serial").write_text("other\n",
                                                   encoding="utf-8")
            self.assertTrue(usb_serial_present("947122070908", root))
            self.assertFalse(usb_serial_present("94712207090", root))
            self.assertFalse(usb_serial_present("", root))


class LauncherPreflightTests(unittest.TestCase):
    def test_missing_expected_serial_exits_before_ros_is_started(self):
        launcher = PROJECT / "start_d435_color.sh"
        missing_serial = f"test-serial-that-cannot-exist-{os.getpid()}-{time.time_ns()}"
        with tempfile.TemporaryDirectory() as temp_dir:
            marker = Path(temp_dir) / "ros2-was-called"
            fake_ros2 = Path(temp_dir) / "ros2"
            fake_ros2.write_text(
                f"#!/usr/bin/env bash\ntouch {marker!s}\nexit 99\n",
                encoding="utf-8",
            )
            fake_ros2.chmod(0o755)
            env = dict(os.environ)
            env["D435I_EXPECTED_SERIAL"] = missing_serial
            env["PATH"] = f"{temp_dir}:{env.get('PATH', '')}"
            completed = subprocess.run(
                ["bash", str(launcher)],
                cwd=PROJECT,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=5,
            )
            ros_was_called = marker.exists()

        self.assertEqual(completed.returncode, 10, completed.stderr)
        self.assertIn(missing_serial, completed.stderr)
        self.assertIn("No ROS process was started", completed.stderr)
        self.assertFalse(ros_was_called)


if __name__ == "__main__":
    unittest.main()
