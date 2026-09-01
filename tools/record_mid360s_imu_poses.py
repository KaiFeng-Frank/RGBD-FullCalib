#!/usr/bin/env python3
"""Continuously capture stable MID-360S IMU poses from ROS 2.

The Livox ROS driver publishes the MID-360(S) accelerometer values in ``g``
even though ``sensor_msgs/Imu`` conventionally uses m/s^2.  This recorder
therefore preserves the driver values and explicitly converts them with
``STANDARD_GRAVITY = 9.80665`` before any calibration work.

The operator only has to put the rigid sensor assembly down, wait for the
automatic check mark, and change its orientation.  There are no per-pose
files or scene commands.  By default 15 distinct stable orientations are
captured: 12 for fitting and 3 that the solver can reserve for validation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np


STANDARD_GRAVITY = 9.80665
DEFAULT_TOPIC = "/livox/imu"
DEFAULT_FRAME = "livox_frame"
SCHEMA = "mid360s_imu_stable_poses/v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def accel_g_to_ms2(accel_g: np.ndarray | Iterable[float]) -> np.ndarray:
    """Convert the raw Livox-driver accelerometer unit to SI."""
    return np.asarray(accel_g, dtype=np.float64) * STANDARD_GRAVITY


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    den = float(np.linalg.norm(a) * np.linalg.norm(b))
    if den <= 0.0:
        return 180.0
    return math.degrees(math.acos(float(np.clip(a @ b / den, -1.0, 1.0))))


@dataclass(frozen=True)
class StableWindow:
    stamp_ns: np.ndarray
    accel_raw_g: np.ndarray
    accel_ms2: np.ndarray
    gyro_rad_s: np.ndarray
    direction_drift_deg: float

    @property
    def accel_mean_ms2(self) -> np.ndarray:
        return self.accel_ms2.mean(axis=0)

    def summary(self, pose_index: int) -> dict:
        return {
            "pose_index": int(pose_index),
            "start_stamp_ns": int(self.stamp_ns[0]),
            "end_stamp_ns": int(self.stamp_ns[-1]),
            "duration_s": float((self.stamp_ns[-1] - self.stamp_ns[0]) * 1e-9),
            "sample_count": int(len(self.stamp_ns)),
            "accel_mean_raw_g": self.accel_raw_g.mean(axis=0).tolist(),
            "accel_mean_ms2": self.accel_ms2.mean(axis=0).tolist(),
            "accel_std_ms2": self.accel_ms2.std(axis=0, ddof=1).tolist(),
            "accel_norm_mean_ms2": float(np.linalg.norm(self.accel_ms2, axis=1).mean()),
            "gyro_mean_rad_s": self.gyro_rad_s.mean(axis=0).tolist(),
            "gyro_std_rad_s": self.gyro_rad_s.std(axis=0, ddof=1).tolist(),
            "gyro_norm_mean_deg_s": float(
                np.degrees(np.linalg.norm(self.gyro_rad_s, axis=1)).mean()
            ),
            "direction_drift_deg": float(self.direction_drift_deg),
        }


class StablePoseCollector:
    """Streaming, ROS-independent stable-window detector used by ``main``."""

    def __init__(
        self,
        *,
        hold_s: float = 0.5,
        min_samples: int = 60,
        min_separation_deg: float = 18.0,
        gyro_mean_limit_deg_s: float = 4.0,
        gyro_std_limit_deg_s: float = 2.5,
        accel_std_limit_ms2: float = 0.35,
        direction_drift_limit_deg: float = 0.8,
    ) -> None:
        if hold_s <= 0.0 or min_samples < 8:
            raise ValueError("hold_s and min_samples must be positive")
        self.hold_s = float(hold_s)
        self.min_samples = int(min_samples)
        self.min_separation_deg = float(min_separation_deg)
        self.gyro_mean_limit_deg_s = float(gyro_mean_limit_deg_s)
        self.gyro_std_limit_deg_s = float(gyro_std_limit_deg_s)
        self.accel_std_limit_ms2 = float(accel_std_limit_ms2)
        self.direction_drift_limit_deg = float(direction_drift_limit_deg)
        # 500 Hz leaves margin above the nominal 200 Hz stream.
        self._samples: deque[tuple[int, np.ndarray, np.ndarray]] = deque(
            maxlen=max(self.min_samples * 4, int(math.ceil(self.hold_s * 500.0)) + 8)
        )
        self.windows: list[StableWindow] = []
        self._armed = True
        self.last_state = "warming_up"
        self.last_metrics: dict[str, float] = {}

    @property
    def pose_means(self) -> list[np.ndarray]:
        return [w.accel_mean_ms2 for w in self.windows]

    def _window(self) -> StableWindow | None:
        if len(self._samples) < self.min_samples:
            return None
        end_ns = self._samples[-1][0]
        start_ns = end_ns - int(round(self.hold_s * 1e9))
        rows = [row for row in self._samples if row[0] >= start_ns]
        if len(rows) < self.min_samples:
            return None
        stamps = np.asarray([r[0] for r in rows], dtype=np.int64)
        if (stamps[-1] - stamps[0]) * 1e-9 < self.hold_s * 0.92:
            return None
        raw = np.asarray([r[1] for r in rows], dtype=np.float64)
        gyro = np.asarray([r[2] for r in rows], dtype=np.float64)
        accel = accel_g_to_ms2(raw)
        q = max(2, len(rows) // 4)
        drift = angle_deg(accel[:q].mean(axis=0), accel[-q:].mean(axis=0))
        return StableWindow(stamps, raw, accel, gyro, drift)

    def ingest(
        self,
        stamp_ns: int,
        accel_raw_g: np.ndarray | Iterable[float],
        gyro_rad_s: np.ndarray | Iterable[float],
    ) -> StableWindow | None:
        """Add one sample and return a newly accepted window, if any."""
        stamp_ns = int(stamp_ns)
        accel_raw_g = np.asarray(accel_raw_g, dtype=np.float64).reshape(3)
        gyro_rad_s = np.asarray(gyro_rad_s, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(accel_raw_g)) or not np.all(np.isfinite(gyro_rad_s)):
            self.last_state = "non_finite_sample"
            return None
        if self._samples and stamp_ns <= self._samples[-1][0]:
            # A clock restart must not mix samples into one stable window.
            self._samples.clear()
            self.last_state = "timestamp_reset"
        self._samples.append((stamp_ns, accel_raw_g.copy(), gyro_rad_s.copy()))

        current_accel = accel_g_to_ms2(accel_raw_g)
        if not self._armed and self.windows:
            moved = angle_deg(current_accel, self.windows[-1].accel_mean_ms2)
            if moved >= max(5.0, self.min_separation_deg * 0.55):
                latest = self._samples[-1]
                self._samples.clear()
                self._samples.append(latest)
                self._armed = True
                self.last_state = "moved_rearming"
            else:
                self.last_state = "change_direction"
            return None

        window = self._window()
        if window is None:
            self.last_state = "warming_up"
            return None

        gyro_norm_mean = float(np.degrees(np.linalg.norm(window.gyro_rad_s, axis=1)).mean())
        gyro_std_norm = float(np.degrees(np.linalg.norm(window.gyro_rad_s.std(axis=0, ddof=1))))
        accel_std_norm = float(np.linalg.norm(window.accel_ms2.std(axis=0, ddof=1)))
        accel_norm = float(np.linalg.norm(window.accel_mean_ms2))
        self.last_metrics = {
            "gyro_norm_mean_deg_s": gyro_norm_mean,
            "gyro_std_norm_deg_s": gyro_std_norm,
            "accel_std_norm_ms2": accel_std_norm,
            "direction_drift_deg": window.direction_drift_deg,
            "accel_mean_norm_ms2": accel_norm,
        }

        if not 6.0 <= accel_norm <= 13.0:
            self.last_state = "accel_out_of_range"
        elif gyro_norm_mean > self.gyro_mean_limit_deg_s:
            self.last_state = "moving_gyro_mean"
        elif gyro_std_norm > self.gyro_std_limit_deg_s:
            self.last_state = "moving_gyro_std"
        elif accel_std_norm > self.accel_std_limit_ms2:
            self.last_state = "moving_accel_std"
        elif window.direction_drift_deg > self.direction_drift_limit_deg:
            self.last_state = "moving_direction_drift"
        else:
            sep = min(
                (angle_deg(window.accel_mean_ms2, p) for p in self.pose_means),
                default=180.0,
            )
            self.last_metrics["nearest_pose_deg"] = float(sep)
            if sep < self.min_separation_deg:
                self.last_state = "duplicate_orientation"
            else:
                self.windows.append(window)
                self._armed = False
                self.last_state = "accepted"
                return window
        return None

    def config(self) -> dict:
        return {
            "hold_s": self.hold_s,
            "min_samples": self.min_samples,
            "min_separation_deg": self.min_separation_deg,
            "gyro_mean_limit_deg_s": self.gyro_mean_limit_deg_s,
            "gyro_std_limit_deg_s": self.gyro_std_limit_deg_s,
            "accel_std_limit_ms2": self.accel_std_limit_ms2,
            "direction_drift_limit_deg": self.direction_drift_limit_deg,
        }


def build_capture_arrays(windows: list[StableWindow], metadata: dict) -> dict[str, np.ndarray]:
    """Build a non-pickled NPZ payload with both summaries and raw windows."""
    if not windows:
        raise ValueError("no stable windows captured")
    summaries = [w.summary(i) for i, w in enumerate(windows)]
    sample_pose_index = np.concatenate([
        np.full(len(w.stamp_ns), i, dtype=np.int32) for i, w in enumerate(windows)
    ])
    return {
        "pose_accel_mean_ms2": np.asarray([w.accel_ms2.mean(axis=0) for w in windows]),
        "pose_accel_std_ms2": np.asarray([w.accel_ms2.std(axis=0, ddof=1) for w in windows]),
        "pose_accel_mean_raw_g": np.asarray([w.accel_raw_g.mean(axis=0) for w in windows]),
        "pose_gyro_mean_rad_s": np.asarray([w.gyro_rad_s.mean(axis=0) for w in windows]),
        "pose_gyro_std_rad_s": np.asarray([w.gyro_rad_s.std(axis=0, ddof=1) for w in windows]),
        "pose_start_ns": np.asarray([w.stamp_ns[0] for w in windows], dtype=np.int64),
        "pose_end_ns": np.asarray([w.stamp_ns[-1] for w in windows], dtype=np.int64),
        "pose_sample_count": np.asarray([len(w.stamp_ns) for w in windows], dtype=np.int32),
        "sample_pose_index": sample_pose_index,
        "sample_stamp_ns": np.concatenate([w.stamp_ns for w in windows]),
        "sample_accel_raw_g": np.concatenate([w.accel_raw_g for w in windows]),
        "sample_accel_ms2": np.concatenate([w.accel_ms2 for w in windows]),
        "sample_gyro_rad_s": np.concatenate([w.gyro_rad_s for w in windows]),
        "metadata_json": np.asarray(json.dumps(metadata, ensure_ascii=False, sort_keys=True)),
        "window_summaries_json": np.asarray(
            json.dumps(summaries, ensure_ascii=False, sort_keys=True)
        ),
    }


def save_capture(path: Path, windows: list[StableWindow], metadata: dict) -> None:
    path = Path(path)
    if path.suffix.lower() != ".npz":
        raise ValueError("capture output must end in .npz")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        # Exclusive creation also protects the check/write race when two
        # recorders are accidentally pointed at the same output.
        with path.open("xb") as stream:
            np.savez_compressed(stream, **build_capture_arrays(windows, metadata))
    except FileExistsError as exc:
        raise FileExistsError(
            f"refusing to overwrite existing capture: {path}"
        ) from exc
    except Exception:
        # This invocation exclusively created the path, so removing its own
        # incomplete file cannot affect a pre-existing capture.
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="MID-360S IMU multi-orientation capture; put it down and change direction after each check mark"
    )
    ap.add_argument("-o", "--out", required=True, type=Path, help="new capture .npz (never overwritten)")
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument(
        "--frame", default=DEFAULT_FRAME,
        help="required sensor_msgs/Imu header.frame_id",
    )
    ap.add_argument("--mid360s-serial", required=True)
    ap.add_argument("--rig-id", required=True)
    ap.add_argument("--mount-id", default="")
    ap.add_argument("--fit-poses", type=int, default=12)
    ap.add_argument("--holdout-poses", type=int, default=3)
    # The mounted unit's three audited recordings produced reliable 100–101
    # sample windows at about 196 Hz.  A 0.5 s dwell with a 60-sample floor
    # matches that observed stream while still requiring the full time span.
    ap.add_argument("--hold", type=float, default=0.5)
    ap.add_argument("--min-samples", type=int, default=60)
    ap.add_argument("--min-sep", type=float, default=18.0)
    # MID-360S static samples have visibly more point-to-point jitter than the
    # D435i.  Direction drift plus the full hold interval still reject motion;
    # these defaults were chosen from the device's actual 200 Hz stream.
    ap.add_argument("--gyro-mean-thr", type=float, default=4.0, help="deg/s")
    ap.add_argument("--gyro-std-thr", type=float, default=2.5, help="deg/s")
    ap.add_argument("--accel-std-thr", type=float, default=0.35, help="m/s^2")
    ap.add_argument("--direction-drift-thr", type=float, default=0.8, help="degrees")
    args = ap.parse_args(argv)
    if args.fit_poses < 12:
        ap.error("--fit-poses must be at least 12")
    if not 0 <= args.holdout_poses <= 4:
        ap.error("--holdout-poses must be in [0, 4]")
    for name in ("topic", "frame", "mid360s_serial", "rig_id", "mount_id"):
        if name == "mount_id" and not getattr(args, name).strip():
            ap.error("--mount-id is required for a current-rig capture")
        if name != "mount_id" and not getattr(args, name).strip():
            ap.error(f"--{name.replace('_', '-')} must be non-empty")
    if args.out.exists():
        ap.error(f"refusing to overwrite existing capture: {args.out}")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        print("ROS 2 Python packages are required; source the ROS and Livox workspaces first.", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 2

    collector = StablePoseCollector(
        hold_s=args.hold,
        min_samples=args.min_samples,
        min_separation_deg=args.min_sep,
        gyro_mean_limit_deg_s=args.gyro_mean_thr,
        gyro_std_limit_deg_s=args.gyro_std_thr,
        accel_std_limit_ms2=args.accel_std_thr,
        direction_drift_limit_deg=args.direction_drift_thr,
    )
    target = args.fit_poses + args.holdout_poses
    started_utc = utc_now()
    received = 0
    fallback_stamp_count = 0
    latest_event: StableWindow | None = None
    fatal_error: str | None = None
    observed_frames: set[str] = set()

    rclpy.init(args=None)
    node = rclpy.create_node("mid360s_imu_pose_recorder")

    def on_imu(msg: Imu) -> None:
        nonlocal received, fallback_stamp_count, latest_event, fatal_error
        observed_frame = str(msg.header.frame_id).strip()
        observed_frames.add(observed_frame)
        if observed_frame != args.frame:
            fatal_error = (
                f"{args.topic} frame_id mismatch: expected {args.frame!r}, "
                f"observed {observed_frame!r}"
            )
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = time.time_ns()
            fallback_stamp_count += 1
        raw_g = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        received += 1
        event = collector.ingest(stamp_ns, raw_g, gyro)
        if event is not None:
            latest_event = event

    node.create_subscription(Imu, args.topic, on_imu, qos_profile_sensor_data)
    print(f"MID-360S IMU: {args.topic}  serial={args.mid360s_serial}")
    print(f"Target: {args.fit_poses} fit + {args.holdout_poses} validation = {target} orientations")
    print("Place the rigid rig down. It records automatically; after each check mark, change direction.")
    print("Cover both signs of X/Y/Z and add tilted orientations. Ctrl-C saves a partial capture.\n")

    last_print = 0.0
    interrupted = False
    try:
        while rclpy.ok() and len(collector.windows) < target:
            rclpy.spin_once(node, timeout_sec=0.1)
            if fatal_error is not None:
                break
            now = time.monotonic()
            if latest_event is not None:
                idx = len(collector.windows)
                s = latest_event.summary(idx - 1)
                a = s["accel_mean_ms2"]
                print(
                    f"\r  OK {idx}/{target}  a=[{a[0]:+7.3f},{a[1]:+7.3f},{a[2]:+7.3f}] "
                    f"|a|={s['accel_norm_mean_ms2']:.4f} m/s^2{' ' * 10}"
                )
                if idx < target:
                    print("     -> change direction, then put it down again")
                latest_event = None
                last_print = now
            elif now - last_print >= 0.5:
                m = collector.last_metrics
                detail = ""
                if m:
                    detail = (
                        f"  gyro={m.get('gyro_norm_mean_deg_s', float('nan')):.2f} deg/s"
                        f"  accel_std={m.get('accel_std_norm_ms2', float('nan')):.3f}"
                    )
                print(
                    f"\r  {len(collector.windows)}/{target}  {collector.last_state}{detail}{' ' * 16}",
                    end="",
                    flush=True,
                )
                last_print = now
    except KeyboardInterrupt:
        interrupted = True
        print("\nInterrupted; saving captured stable windows.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if fatal_error is not None:
        print(fatal_error, file=sys.stderr)
        print("No capture written after frame identity failure.", file=sys.stderr)
        return 2

    if not collector.windows:
        print("No stable pose was captured; no file written.", file=sys.stderr)
        return 2

    metadata = {
        "schema": SCHEMA,
        "capture_started_utc": started_utc,
        "capture_ended_utc": utc_now(),
        "identity": {
            "mid360s_serial": args.mid360s_serial,
            "rig_id": args.rig_id,
            "mount_id": args.mount_id,
        },
        "source": {
            "role": "operational_capture",
            "ros_topic": args.topic,
            "frame_id": args.frame,
            "observed_frame_ids": sorted(observed_frames),
            "message_type": "sensor_msgs/msg/Imu",
            "header_stamp_fallback_count": fallback_stamp_count,
            "received_message_count": received,
        },
        "units": {
            "driver_accelerometer_input": "g",
            "stored_accelerometer_raw": "g",
            "stored_accelerometer_si": "m/s^2",
            "gyroscope": "rad/s",
            "conversion": "accel_ms2 = accel_driver_g * 9.80665",
            "standard_gravity_ms2": STANDARD_GRAVITY,
        },
        "capture_plan": {
            "fit_pose_target": args.fit_poses,
            "holdout_pose_target": args.holdout_poses,
            "total_pose_target": target,
            "captured_pose_count": len(collector.windows),
            "interrupted": interrupted,
        },
        "stable_detector": collector.config(),
    }
    try:
        save_capture(args.out, collector.windows, metadata)
    except (FileExistsError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print(f"\nCapture -> {args.out}")
    print(f"Stable orientations: {len(collector.windows)}; raw samples: "
          f"{sum(len(w.stamp_ns) for w in collector.windows)}")
    if len(collector.windows) < args.fit_poses:
        print(f"Need at least {args.fit_poses} orientations before solving.", file=sys.stderr)
        return 2
    if len(collector.windows) < target:
        print("Fit minimum reached, but the requested validation holdout is incomplete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
