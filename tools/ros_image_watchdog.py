#!/usr/bin/env python3
"""Exit when a ROS Image topic never starts or stops producing valid frames."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace


STARTUP_TIMEOUT_EXIT = 20
STREAM_TIMEOUT_EXIT = 21
DEVICE_MISSING_EXIT = 22
USB_CHECK_INTERVAL = 0.5


@dataclass
class ImageHeartbeat:
    """Pure stream-timing state, deliberately independent from ROS."""

    started_at: float
    startup_timeout: float
    loss_timeout: float
    last_frame_at: float | None = None
    last_message_stamp_ns: int | None = None

    def __post_init__(self) -> None:
        values = (self.started_at, self.startup_timeout, self.loss_timeout)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("heartbeat times must be finite")
        if self.startup_timeout <= 0 or self.loss_timeout <= 0:
            raise ValueError("heartbeat timeouts must be positive")
        if self.last_frame_at is not None:
            if (not math.isfinite(self.last_frame_at)
                    or self.last_frame_at < self.started_at):
                raise ValueError("last frame time is invalid")
        if self.last_message_stamp_ns is not None:
            if (isinstance(self.last_message_stamp_ns, bool)
                    or not isinstance(self.last_message_stamp_ns, int)
                    or self.last_message_stamp_ns < 0):
                raise ValueError("last Image timestamp is invalid")
            if self.last_frame_at is None:
                raise ValueError("Image timestamp requires a received frame")

    def status(self, now: float) -> str:
        self._check_now(now)
        if self.last_frame_at is None:
            if now - self.started_at >= self.startup_timeout:
                return "startup-timeout"
            return "starting"
        if now - self.last_frame_at >= self.loss_timeout:
            return "stream-timeout"
        return "healthy"

    def observe(self, now: float, message_stamp_ns: int) -> bool:
        """Record an on-time, timestamp-advancing frame."""

        self._check_now(now)
        if (isinstance(message_stamp_ns, bool)
                or not isinstance(message_stamp_ns, int)
                or message_stamp_ns < 0):
            return False
        if self.status(now) in ("startup-timeout", "stream-timeout"):
            return False
        if (self.last_message_stamp_ns is not None
                and message_stamp_ns <= self.last_message_stamp_ns):
            return False
        self.last_frame_at = now
        self.last_message_stamp_ns = message_stamp_ns
        return True

    def _check_now(self, now: float) -> None:
        if not math.isfinite(now) or now < self.started_at:
            raise ValueError("observation time is invalid")
        if self.last_frame_at is not None and now < self.last_frame_at:
            raise ValueError("observation time moved backwards")


@dataclass
class SustainedAbsence:
    """Report only a continuous physical-device absence."""

    timeout: float
    absent_since: float | None = None
    last_observed_at: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("absence timeout must be positive and finite")
        if self.absent_since is not None and not math.isfinite(self.absent_since):
            raise ValueError("absence start time must be finite")
        if (self.last_observed_at is not None
                and not math.isfinite(self.last_observed_at)):
            raise ValueError("device observation time must be finite")
        if (self.absent_since is not None and self.last_observed_at is not None
                and self.absent_since > self.last_observed_at):
            raise ValueError("absence starts after the last observation")

    def observe(self, present: bool, now: float) -> bool:
        if not math.isfinite(now):
            raise ValueError("device observation time must be finite")
        if self.last_observed_at is not None and now < self.last_observed_at:
            raise ValueError("device observation time moved backwards")
        self.last_observed_at = now
        if present:
            self.absent_since = None
            return False
        if self.absent_since is None:
            self.absent_since = now
            return False
        return now - self.absent_since >= self.timeout


def usb_serial_present(
    expected_serial: str,
    sysfs_root: Path = Path("/sys/bus/usb/devices"),
) -> bool:
    """Scan the kernel USB inventory for one exact serial number."""

    if not expected_serial or "\n" in expected_serial or "\r" in expected_serial:
        return False
    try:
        serial_files = sysfs_root.glob("*/serial")
        for serial_file in serial_files:
            try:
                observed = serial_file.read_text(encoding="utf-8").rstrip("\r\n")
            except (OSError, UnicodeError):
                continue
            if observed == expected_serial:
                return True
    except OSError:
        return False
    return False


def image_stamp_ns(message: object) -> int | None:
    """Return a well-formed ROS builtin_interfaces/Time value in nanoseconds."""

    try:
        sec = getattr(getattr(message, "header"), "stamp").sec
        nanosec = getattr(getattr(message, "header"), "stamp").nanosec
    except AttributeError:
        return None
    if (isinstance(sec, bool) or not isinstance(sec, int)
            or isinstance(nanosec, bool) or not isinstance(nanosec, int)):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def image_is_valid(message: object) -> bool:
    """Require a ROS timestamp, dimensions, and a complete non-empty payload."""

    try:
        height = int(getattr(message, "height"))
        width = int(getattr(message, "width"))
        step = int(getattr(message, "step"))
        payload_size = len(getattr(message, "data"))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    if height <= 0 or width <= 0 or step < width:
        return False
    return payload_size >= height * step and image_stamp_ns(message) is not None


def observe_image(
    state: ImageHeartbeat, message: object, now: float
) -> bool:
    """Count only a structurally valid sensor_msgs/Image as a heartbeat."""

    if not image_is_valid(message):
        return False
    stamp_ns = image_stamp_ns(message)
    assert stamp_ns is not None
    return state.observe(now, stamp_ns)


def positive_seconds(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def one_line_serial(value: str) -> str:
    if not value or "\n" in value or "\r" in value:
        raise argparse.ArgumentTypeError("must be one non-empty line")
    return value


def run_self_test() -> int:
    state = ImageHeartbeat(100.0, startup_timeout=20.0, loss_timeout=5.0)
    assert state.status(119.999) == "starting"
    assert state.status(120.0) == "startup-timeout"
    assert not observe_image(
        state,
        SimpleNamespace(
            height=1,
            width=1,
            step=1,
            data=b"x",
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=1, nanosec=0)
            ),
        ),
        120.0,
    )

    state = ImageHeartbeat(100.0, startup_timeout=20.0, loss_timeout=5.0)
    empty = SimpleNamespace(height=720, width=1280, step=3840, data=b"")
    assert not observe_image(state, empty, 101.0)
    frame = SimpleNamespace(
        height=1,
        width=2,
        step=6,
        data=bytes(6),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
    )
    assert observe_image(state, frame, 101.0)
    assert state.status(105.999) == "healthy"
    assert state.status(106.0) == "stream-timeout"
    assert not observe_image(state, frame, 106.0)

    device = SustainedAbsence(timeout=3.0)
    assert not device.observe(True, 0.0)
    assert not device.observe(False, 1.0)
    assert not device.observe(False, 3.999)
    assert device.observe(False, 4.0)

    print("ros_image_watchdog self-test: PASS")
    return 0


def run_ros_watchdog(
    topic: str,
    expected_serial: str,
    startup_timeout: float,
    loss_timeout: float,
    device_loss_timeout: float,
) -> int:
    # Imports stay lazy so the state machine and --self-test need no ROS setup.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    # The watchdog CLI owns argv; no custom option may leak into ROS parsing.
    rclpy.init(args=[])
    node = rclpy.create_node("d435_color_disconnect_watchdog")
    state = ImageHeartbeat(time.monotonic(), startup_timeout, loss_timeout)
    device_absence = SustainedAbsence(device_loss_timeout)
    last_usb_check = -math.inf
    first_frame = True

    def on_image(message: Image) -> None:
        nonlocal first_frame
        if not observe_image(state, message, time.monotonic()):
            return
        if first_frame:
            print(
                f"D435 color watchdog armed on {topic}.",
                file=sys.stderr,
                flush=True,
            )
            first_frame = False

    subscription = node.create_subscription(
        Image, topic, on_image, qos_profile_sensor_data
    )
    # Retain an explicit reference for rclpy implementations that use weak refs.
    assert subscription is not None

    try:
        while rclpy.ok():
            now = time.monotonic()
            if now - last_usb_check >= USB_CHECK_INTERVAL:
                present = usb_serial_present(expected_serial)
                last_usb_check = now
                if device_absence.observe(present, now):
                    print(
                        f"D435 USB serial {expected_serial} has been absent for "
                        f"{device_loss_timeout:g}s.",
                        file=sys.stderr,
                        flush=True,
                    )
                    return DEVICE_MISSING_EXIT
            rclpy.spin_once(node, timeout_sec=0.25)
            status = state.status(time.monotonic())
            if status == "startup-timeout":
                print(
                    f"No valid sensor_msgs/Image received on {topic} for "
                    f"{startup_timeout:g}s during startup.",
                    file=sys.stderr,
                    flush=True,
                )
                return STARTUP_TIMEOUT_EXIT
            if status == "stream-timeout":
                print(
                    f"No valid sensor_msgs/Image received on {topic} for "
                    f"{loss_timeout:g}s after the stream was active.",
                    file=sys.stderr,
                    flush=True,
                )
                return STREAM_TIMEOUT_EXIT
        return 2
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument(
        "--expected-serial", type=one_line_serial, default="947122070908"
    )
    parser.add_argument("--startup-timeout", type=positive_seconds, default=20.0)
    parser.add_argument("--loss-timeout", type=positive_seconds, default=5.0)
    parser.add_argument(
        "--device-loss-timeout", type=positive_seconds, default=3.0
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    return run_ros_watchdog(
        args.topic,
        args.expected_serial,
        args.startup_timeout,
        args.loss_timeout,
        args.device_loss_timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())
