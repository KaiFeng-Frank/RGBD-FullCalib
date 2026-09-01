#!/usr/bin/env python3
"""Solve an operational MID-360S built-in IMU calibration.

Input is normally the NPZ produced by ``record_mid360s_imu_poses.py``.  A ROS
2 bag directory containing ``/livox/imu`` is also accepted and is segmented by
the same stable-window detector.  The command writes *analysis* JSON and NPZ;
it deliberately refuses paths inside a ``results`` directory so a reviewed
promotion step remains explicit.

Accelerometer model (the Livox driver input is in g)::

    a_corrected_ms2 = T_misalignment * diag(accel_scale)
                      * (9.80665 * a_raw_g - accel_bias_ms2)

Only stationary data are used.  Gyroscope scale needs a calibrated rate table
and is not claimed here; stationary poses identify its zero-rate bias.  Noise
density is a short-window white-noise estimate, not a long-duration Allan
deviation / bias-instability characterization.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_intrinsic import build_A, local_gravity, residual  # noqa: E402
from record_mid360s_imu_poses import (  # noqa: E402
    DEFAULT_TOPIC,
    SCHEMA as CAPTURE_SCHEMA,
    STANDARD_GRAVITY,
    StablePoseCollector,
    StableWindow,
    angle_deg,
    build_capture_arrays,
)


SCHEMA = "mid360s_imu_intrinsics_analysis/v1"
INTENDED_LOCAL_SCHEMA = "d435i_calib/mid360s_imu_operational/v1"
ACCEL_EQUATION = (
    "a_corrected_ms2 = T_misalignment * diag(accel_scale) * "
    "(9.80665 * a_raw_g - accel_bias_ms2)"
)


class CalibrationError(RuntimeError):
    pass


@dataclass(frozen=True)
class AccelFit:
    params: np.ndarray
    matrix: np.ndarray
    bias_ms2: np.ndarray
    residual_ms2: np.ndarray
    jacobian: np.ndarray
    success: bool
    message: str


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def sha256_path(path: Path) -> str:
    path = Path(path)
    h = hashlib.sha256()
    if path.is_file():
        with path.open("rb") as f:
            for block in iter(lambda: f.read(1024 * 1024), b""):
                h.update(block)
        return h.hexdigest()
    if not path.is_dir():
        raise FileNotFoundError(path)
    for item in sorted(p for p in path.rglob("*") if p.is_file()):
        rel = item.relative_to(path).as_posix().encode("utf-8")
        h.update(len(rel).to_bytes(8, "little"))
        h.update(rel)
        h.update(item.stat().st_size.to_bytes(8, "little"))
        with item.open("rb") as f:
            for block in iter(lambda: f.read(1024 * 1024), b""):
                h.update(block)
    return h.hexdigest()


def _json_scalar(value: np.ndarray) -> dict:
    text = str(np.asarray(value).reshape(()).item())
    parsed = json.loads(text)
    if not isinstance(parsed, dict):
        raise CalibrationError("capture metadata_json is not an object")
    return parsed


def load_capture_npz(path: Path) -> dict:
    path = Path(path)
    try:
        with np.load(path, allow_pickle=False) as z:
            required = {
                "pose_accel_mean_ms2",
                "pose_gyro_mean_rad_s",
                "sample_pose_index",
                "sample_stamp_ns",
                "sample_accel_raw_g",
                "sample_accel_ms2",
                "sample_gyro_rad_s",
                "metadata_json",
                "window_summaries_json",
            }
            missing = sorted(required.difference(z.files))
            if missing:
                raise CalibrationError(f"capture is missing arrays: {', '.join(missing)}")
            capture = {key: np.asarray(z[key]).copy() for key in z.files}
    except (OSError, ValueError) as exc:
        raise CalibrationError(f"cannot read capture NPZ: {exc}") from exc
    capture["metadata"] = _json_scalar(capture["metadata_json"])
    try:
        summaries = json.loads(str(capture["window_summaries_json"].reshape(()).item()))
    except (ValueError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"invalid window_summaries_json: {exc}") from exc
    if not isinstance(summaries, list):
        raise CalibrationError("window_summaries_json is not a list")
    capture["window_summaries"] = summaries
    validate_capture(capture)
    return capture


def validate_capture(capture: dict) -> None:
    means = np.asarray(capture["pose_accel_mean_ms2"], dtype=np.float64)
    gyro_means = np.asarray(capture["pose_gyro_mean_rad_s"], dtype=np.float64)
    sample_pose = np.asarray(capture["sample_pose_index"])
    sample_t = np.asarray(capture["sample_stamp_ns"])
    sample_a_g = np.asarray(capture["sample_accel_raw_g"], dtype=np.float64)
    sample_a = np.asarray(capture["sample_accel_ms2"], dtype=np.float64)
    sample_g = np.asarray(capture["sample_gyro_rad_s"], dtype=np.float64)
    if means.ndim != 2 or means.shape[1] != 3 or gyro_means.shape != means.shape:
        raise CalibrationError("pose means must have shape (N, 3)")
    n_samples = len(sample_pose)
    if (
        sample_pose.shape != (n_samples,)
        or sample_pose.dtype.kind not in "iu"
        or sample_t.shape != (n_samples,)
        or sample_t.dtype.kind not in "iu"
        or sample_a_g.shape != (n_samples, 3)
    ):
        raise CalibrationError("raw sample arrays have inconsistent shape")
    if sample_a.shape != (n_samples, 3) or sample_g.shape != (n_samples, 3):
        raise CalibrationError("SI sample arrays have inconsistent shape")
    numeric_arrays = (means, gyro_means, sample_a_g, sample_a, sample_g)
    if n_samples == 0 or any(not np.all(np.isfinite(array)) for array in numeric_arrays):
        raise CalibrationError("capture contains no finite samples")
    if np.min(sample_pose) < 0 or np.max(sample_pose) >= len(means):
        raise CalibrationError("sample_pose_index is outside the pose array")
    conversion_error = np.max(np.abs(sample_a - sample_a_g * STANDARD_GRAVITY))
    if conversion_error > 1e-8:
        raise CalibrationError(
            "stored SI acceleration does not equal raw driver g times 9.80665"
        )
    observed_poses = np.unique(sample_pose).tolist()
    if observed_poses != list(range(len(means))):
        raise CalibrationError("every declared pose must have raw samples")
    for pose in range(len(means)):
        mask = sample_pose == pose
        if int(np.count_nonzero(mask)) < 8:
            raise CalibrationError(f"pose {pose} has fewer than 8 raw samples")
        stamps = sample_t[mask].astype(np.int64, copy=False)
        if np.any(np.diff(stamps) <= 0):
            raise CalibrationError(
                f"pose {pose} timestamps must be strictly increasing"
            )
        computed_accel = sample_a[mask].mean(axis=0)
        computed_gyro = sample_g[mask].mean(axis=0)
        if not np.allclose(
            means[pose], computed_accel, rtol=1e-12, atol=1e-10
        ):
            raise CalibrationError(
                f"pose_accel_mean_ms2[{pose}] is not the raw-sample mean"
            )
        if not np.allclose(
            gyro_means[pose], computed_gyro, rtol=1e-12, atol=1e-12
        ):
            raise CalibrationError(
                f"pose_gyro_mean_rad_s[{pose}] is not the raw-sample mean"
            )
    summaries = capture.get("window_summaries")
    if summaries is not None:
        if not isinstance(summaries, list) or len(summaries) != len(means):
            raise CalibrationError("window summaries must contain one row per pose")
        for pose, summary in enumerate(summaries):
            if not isinstance(summary, dict) or summary.get("pose_index") != pose:
                raise CalibrationError("window summary pose indices are not canonical")
            mask = sample_pose == pose
            stamps = sample_t[mask].astype(np.int64, copy=False)
            raw = sample_a_g[mask]
            accel = sample_a[mask]
            gyro = sample_g[mask]
            q = max(2, len(stamps) // 4)
            recomputed = {
                "start_stamp_ns": int(stamps[0]),
                "end_stamp_ns": int(stamps[-1]),
                "duration_s": float((stamps[-1] - stamps[0]) * 1e-9),
                "sample_count": int(len(stamps)),
                "accel_mean_raw_g": raw.mean(axis=0),
                "accel_mean_ms2": accel.mean(axis=0),
                "accel_std_ms2": accel.std(axis=0, ddof=1),
                "accel_norm_mean_ms2": float(np.linalg.norm(accel, axis=1).mean()),
                "gyro_mean_rad_s": gyro.mean(axis=0),
                "gyro_std_rad_s": gyro.std(axis=0, ddof=1),
                "gyro_norm_mean_deg_s": float(
                    np.degrees(np.linalg.norm(gyro, axis=1)).mean()
                ),
                "direction_drift_deg": angle_deg(
                    accel[:q].mean(axis=0), accel[-q:].mean(axis=0)
                ),
            }
            for field in ("start_stamp_ns", "end_stamp_ns", "sample_count"):
                if summary.get(field) != recomputed[field]:
                    raise CalibrationError(
                        f"window summary {pose} {field} differs from raw samples"
                    )
            for field in (
                "duration_s",
                "accel_norm_mean_ms2",
                "gyro_norm_mean_deg_s",
                "direction_drift_deg",
            ):
                declared = summary.get(field)
                if (
                    isinstance(declared, bool)
                    or not isinstance(declared, (int, float))
                    or not math.isfinite(float(declared))
                    or not math.isclose(
                        float(declared),
                        float(recomputed[field]),
                        rel_tol=1e-10,
                        abs_tol=1e-10,
                    )
                ):
                    raise CalibrationError(
                        f"window summary {pose} {field} differs from raw samples"
                    )
            for field in (
                "accel_mean_raw_g",
                "accel_mean_ms2",
                "accel_std_ms2",
                "gyro_mean_rad_s",
                "gyro_std_rad_s",
            ):
                try:
                    declared = np.asarray(summary.get(field), dtype=np.float64)
                except (TypeError, ValueError) as exc:
                    raise CalibrationError(
                        f"window summary {pose} {field} is not a vector3"
                    ) from exc
                if declared.shape != (3,) or not np.allclose(
                    declared,
                    recomputed[field],
                    rtol=1e-10,
                    atol=1e-10,
                ):
                    raise CalibrationError(
                        f"window summary {pose} {field} differs from raw samples"
                    )
    metadata = capture.get("metadata", {})
    schema = metadata.get("schema")
    if schema not in (CAPTURE_SCHEMA, None):
        raise CalibrationError(f"unsupported capture schema: {schema}")
    units = metadata.get("units", {})
    if units and units.get("driver_accelerometer_input") != "g":
        raise CalibrationError("capture does not declare the Livox driver acceleration unit as g")
    detector = metadata.get("stable_detector")
    if detector is not None:
        if not isinstance(detector, dict):
            raise CalibrationError("stable_detector metadata must be an object")

        def detector_number(name: str) -> float:
            value = detector.get(name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise CalibrationError(f"stable_detector.{name} must be finite")
            return float(value)

        minimum_samples = detector.get("min_samples")
        if (
            isinstance(minimum_samples, bool)
            or not isinstance(minimum_samples, int)
            or minimum_samples < 8
        ):
            raise CalibrationError("stable_detector.min_samples must be an integer >= 8")
        hold_s = detector_number("hold_s")
        separation = detector_number("min_separation_deg")
        limits = {
            "gyro_norm_mean_deg_s": detector_number("gyro_mean_limit_deg_s"),
            "gyro_std_norm_deg_s": detector_number("gyro_std_limit_deg_s"),
            "accel_std_norm_ms2": detector_number("accel_std_limit_ms2"),
            "direction_drift_deg": detector_number("direction_drift_limit_deg"),
        }
        for pose in range(len(means)):
            mask = sample_pose == pose
            stamps = sample_t[mask]
            accel = sample_a[mask]
            gyro = sample_g[mask]
            duration = float((stamps[-1] - stamps[0]) * 1e-9)
            if len(stamps) < minimum_samples or duration < hold_s * 0.90:
                raise CalibrationError(
                    f"pose {pose} does not satisfy stable_detector sample/duration gate"
                )
            q = max(2, len(stamps) // 4)
            metrics = {
                "gyro_norm_mean_deg_s": float(
                    np.degrees(np.linalg.norm(gyro, axis=1)).mean()
                ),
                "gyro_std_norm_deg_s": float(
                    np.degrees(np.linalg.norm(gyro.std(axis=0, ddof=1)))
                ),
                "accel_std_norm_ms2": float(
                    np.linalg.norm(accel.std(axis=0, ddof=1))
                ),
                "direction_drift_deg": angle_deg(
                    accel[:q].mean(axis=0), accel[-q:].mean(axis=0)
                ),
            }
            if not 6.0 <= float(np.linalg.norm(accel.mean(axis=0))) <= 13.0:
                raise CalibrationError(f"pose {pose} acceleration norm is not gravity-like")
            for metric, limit in limits.items():
                if metrics[metric] > limit:
                    raise CalibrationError(
                        f"pose {pose} exceeds stable_detector {metric} gate"
                    )
        for first in range(len(means)):
            for second in range(first + 1, len(means)):
                if angle_deg(means[first], means[second]) < separation:
                    raise CalibrationError(
                        "capture contains poses closer than stable_detector separation"
                    )


def capture_from_rosbag(
    path: Path,
    *,
    topic: str,
    frame: str,
    serial: str,
    rig_id: str,
    mount_id: str,
    collector_kwargs: dict | None = None,
) -> dict:
    """Read a ROS 2 bag and segment it with the live recorder's detector."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise CalibrationError(
            "ROS 2 bag input needs rosbag2_py/rclpy; source the ROS environment"
        ) from exc

    detector = {
        "hold_s": 0.5,
        "min_samples": 60,
        "min_separation_deg": 18.0,
        "gyro_mean_limit_deg_s": 4.0,
        "gyro_std_limit_deg_s": 2.5,
        "accel_std_limit_ms2": 0.35,
        "direction_drift_limit_deg": 0.8,
    }
    detector.update(collector_kwargs or {})
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="", output_serialization_format=""
    )
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:
        raise CalibrationError(f"cannot open ROS 2 bag {path}: {exc}") from exc
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise CalibrationError(f"bag has no {topic}; topics: {sorted(topic_types)}")
    if topic_types[topic] != "sensor_msgs/msg/Imu":
        raise CalibrationError(f"{topic} has unexpected type {topic_types[topic]}")
    msg_cls = get_message(topic_types[topic])
    received = 0
    fallback_stamp_count = 0
    observed_frames: set[str] = set()
    first_bag_ns = None
    last_bag_ns = None
    stamp_rows = []
    accel_rows = []
    gyro_rows = []
    while reader.has_next():
        name, raw, bag_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(raw, msg_cls)
        observed_frame = str(msg.header.frame_id).strip()
        observed_frames.add(observed_frame)
        if observed_frame != frame:
            raise CalibrationError(
                f"{topic} frame_id mismatch in {path}: "
                f"expected {frame!r}, observed {observed_frame!r}"
            )
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(bag_ns)
            fallback_stamp_count += 1
        stamp_rows.append(stamp_ns)
        accel_rows.append(
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        )
        gyro_rows.append(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        )
        first_bag_ns = int(bag_ns) if first_bag_ns is None else first_bag_ns
        last_bag_ns = int(bag_ns)
        received += 1
    windows = extract_stable_windows_from_series(
        np.asarray(stamp_rows, dtype=np.int64),
        np.asarray(accel_rows, dtype=np.float64),
        np.asarray(gyro_rows, dtype=np.float64),
        **detector,
    )
    if not windows:
        raise CalibrationError("no stable, direction-separated window was found in the bag")
    metadata = {
        "schema": CAPTURE_SCHEMA,
        "capture_started_utc": None,
        "capture_ended_utc": None,
        "identity": {
            "mid360s_serial": serial,
            "rig_id": rig_id,
            "mount_id": mount_id,
        },
        "source": {
            "role": "operational_capture",
            "ros_topic": topic,
            "frame_id": frame,
            "observed_frame_ids": sorted(observed_frames),
            "message_type": "sensor_msgs/msg/Imu",
            "received_message_count": received,
            "header_stamp_fallback_count": fallback_stamp_count,
            "bag_first_record_stamp_ns": first_bag_ns,
            "bag_last_record_stamp_ns": last_bag_ns,
        },
        "units": {
            "driver_accelerometer_input": "g",
            "stored_accelerometer_raw": "g",
            "stored_accelerometer_si": "m/s^2",
            "conversion": "accel_ms2 = accel_driver_g * 9.80665",
            "standard_gravity_ms2": STANDARD_GRAVITY,
        },
        "capture_plan": {
            "captured_pose_count": len(windows),
            "segmented_from_rosbag": True,
        },
        "stable_detector": detector,
    }
    payload = build_capture_arrays(windows, metadata)
    payload["metadata"] = metadata
    payload["window_summaries"] = json.loads(
        str(payload["window_summaries_json"].reshape(()).item())
    )
    validate_capture(payload)
    return payload


def extract_stable_windows_from_series(
    stamp_ns: np.ndarray,
    accel_raw_g: np.ndarray,
    gyro_rad_s: np.ndarray,
    *,
    hold_s: float = 0.5,
    min_samples: int = 60,
    min_separation_deg: float = 18.0,
    gyro_mean_limit_deg_s: float = 4.0,
    gyro_std_limit_deg_s: float = 2.5,
    accel_std_limit_ms2: float = 0.35,
    direction_drift_limit_deg: float = 0.8,
) -> list[StableWindow]:
    """Select the best stable window for each gravity direction in a recording.

    Offline selection can inspect all candidate windows and is deliberately
    different from the causal live detector: candidates are ranked by
    stationarity before direction de-duplication.  This prevents a transition
    edge from masking a quieter window later on the same platform pose.
    """
    stamps = np.asarray(stamp_ns, dtype=np.int64)
    raw = np.asarray(accel_raw_g, dtype=np.float64)
    gyro = np.asarray(gyro_rad_s, dtype=np.float64)
    if stamps.ndim != 1 or raw.shape != (len(stamps), 3) or gyro.shape != raw.shape:
        raise CalibrationError("ROS bag IMU arrays have inconsistent shape")
    if len(stamps) < min_samples:
        return []
    order = np.argsort(stamps, kind="stable")
    stamps, raw, gyro = stamps[order], raw[order], gyro[order]
    keep = np.concatenate(([True], np.diff(stamps) > 0))
    stamps, raw, gyro = stamps[keep], raw[keep], gyro[keep]
    dt = np.diff(stamps).astype(np.float64) * 1e-9
    dt = dt[dt > 0.0]
    if not len(dt):
        return []
    sample_rate = 1.0 / float(np.median(dt))
    n = max(int(min_samples), int(round(hold_s * sample_rate)))
    if len(stamps) < n:
        return []
    step = max(5, n // 8)
    accel = raw * STANDARD_GRAVITY
    limits = np.array(
        [gyro_mean_limit_deg_s, gyro_std_limit_deg_s,
         accel_std_limit_ms2, direction_drift_limit_deg],
        dtype=np.float64,
    )
    candidates: list[tuple[float, int, StableWindow]] = []
    for start in range(0, len(stamps) - n + 1, step):
        stop = start + n
        if (stamps[stop - 1] - stamps[start]) * 1e-9 < hold_s * 0.90:
            continue
        a = accel[start:stop]
        w = gyro[start:stop]
        mean_a = a.mean(axis=0)
        if not 6.0 <= np.linalg.norm(mean_a) <= 13.0:
            continue
        q = max(3, n // 4)
        drift = angle_deg(a[:q].mean(axis=0), a[-q:].mean(axis=0))
        metrics = np.array(
            [
                np.degrees(np.linalg.norm(w, axis=1)).mean(),
                np.degrees(np.linalg.norm(w.std(axis=0, ddof=1))),
                np.linalg.norm(a.std(axis=0, ddof=1)),
                drift,
            ],
            dtype=np.float64,
        )
        if np.any(metrics > limits):
            continue
        window = StableWindow(
            stamps[start:stop].copy(),
            raw[start:stop].copy(),
            a.copy(),
            w.copy(),
            float(drift),
        )
        score = float(np.sum(metrics / limits))
        candidates.append((score, start, window))
    candidates.sort(key=lambda item: (item[0], item[1]))
    selected: list[tuple[int, StableWindow]] = []
    for _, start, window in candidates:
        if all(
            angle_deg(window.accel_mean_ms2, prior.accel_mean_ms2) >= min_separation_deg
            for _, prior in selected
        ):
            selected.append((start, window))
    selected.sort(key=lambda item: item[0])
    return [window for _, window in selected]


def merge_captures(captures: list[dict], minimum_separation_deg: float) -> dict:
    """Merge sessions and discard repeated gravity directions deterministically."""
    if not captures:
        raise CalibrationError("no captures to merge")
    if len(captures) == 1:
        return captures[0]
    identities = [c.get("metadata", {}).get("identity", {}) for c in captures]
    serials = {item.get("mid360s_serial") for item in identities if item.get("mid360s_serial")}
    rigs = {item.get("rig_id") for item in identities if item.get("rig_id")}
    mounts = {item.get("mount_id") for item in identities if item.get("mount_id")}
    frames = {
        c.get("metadata", {}).get("source", {}).get("frame_id")
        for c in captures
        if c.get("metadata", {}).get("source", {}).get("frame_id")
    }
    if len(serials) > 1 or len(rigs) > 1 or len(mounts) > 1:
        raise CalibrationError(
            "refusing to merge captures from different device, rig, or mount identities"
        )
    if len(frames) > 1:
        raise CalibrationError("refusing to merge captures with different ROS frame_id values")

    windows: list[StableWindow] = []
    directions: list[np.ndarray] = []
    session_counts: list[dict] = []
    for session_index, capture in enumerate(captures):
        means = np.asarray(capture["pose_accel_mean_ms2"], dtype=np.float64)
        pose_index = np.asarray(capture["sample_pose_index"], dtype=np.int32)
        kept_here = 0
        for pose in range(len(means)):
            direction = means[pose]
            if directions and min(angle_deg(direction, prior) for prior in directions) < minimum_separation_deg:
                continue
            mask = pose_index == pose
            order = np.argsort(np.asarray(capture["sample_stamp_ns"])[mask])
            stamps = np.asarray(capture["sample_stamp_ns"])[mask][order].astype(np.int64)
            raw = np.asarray(capture["sample_accel_raw_g"])[mask][order].astype(np.float64)
            accel = np.asarray(capture["sample_accel_ms2"])[mask][order].astype(np.float64)
            gyro = np.asarray(capture["sample_gyro_rad_s"])[mask][order].astype(np.float64)
            if len(stamps) < 8:
                continue
            q = max(2, len(stamps) // 4)
            drift = angle_deg(accel[:q].mean(axis=0), accel[-q:].mean(axis=0))
            windows.append(StableWindow(stamps, raw, accel, gyro, drift))
            directions.append(direction)
            kept_here += 1
        session_counts.append(
            {
                "session_index": session_index,
                "candidate_pose_count": int(len(means)),
                "unique_pose_count_added": kept_here,
                "capture_metadata": capture.get("metadata", {}),
            }
        )
    if not windows:
        raise CalibrationError("capture merge left no unique stable orientation")
    first_identity = next((item for item in identities if item), {})
    metadata = {
        "schema": CAPTURE_SCHEMA,
        "identity": first_identity,
        "source": {
            "role": "operational_capture",
            "merged_session_count": len(captures),
            "frame_id": next(iter(frames), None),
            "sessions": session_counts,
        },
        "units": {
            "driver_accelerometer_input": "g",
            "stored_accelerometer_raw": "g",
            "stored_accelerometer_si": "m/s^2",
            "conversion": "accel_ms2 = accel_driver_g * 9.80665",
            "standard_gravity_ms2": STANDARD_GRAVITY,
        },
        "capture_plan": {
            "captured_pose_count": len(windows),
            "merged": True,
            "minimum_cross_session_separation_deg": minimum_separation_deg,
        },
    }
    payload = build_capture_arrays(windows, metadata)
    payload["metadata"] = metadata
    payload["window_summaries"] = json.loads(
        str(payload["window_summaries_json"].reshape(()).item())
    )
    validate_capture(payload)
    return payload


def fit_accelerometer(measurements_ms2: np.ndarray, gravity_ms2: float) -> AccelFit:
    meas = np.asarray(measurements_ms2, dtype=np.float64)
    if meas.ndim != 2 or meas.shape[1] != 3 or len(meas) < 10:
        raise CalibrationError("accelerometer fit needs at least 10 three-axis poses")
    p0 = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    lower = np.array([-0.20, -0.20, -0.20, 0.80, 0.80, 0.80, -2.0, -2.0, -2.0])
    upper = np.array([+0.20, +0.20, +0.20, 1.20, 1.20, 1.20, +2.0, +2.0, +2.0])
    # A small bounded Levenberg-Marquardt implementation keeps this command
    # usable in the system ROS Python, where SciPy is not installed.  The
    # device corrections are close to identity, so the physical p0 is also a
    # strong initialization.  Soft-L1 IRLS prevents one imperfect placement
    # from steering the ellipsoid.
    def numeric_jacobian(p: np.ndarray) -> np.ndarray:
        columns = []
        for j in range(len(p)):
            step = 2e-6 * max(1.0, abs(float(p[j])))
            plus = p.copy(); plus[j] += step
            minus = p.copy(); minus[j] -= step
            columns.append(
                (residual(plus, meas, gravity_ms2) - residual(minus, meas, gravity_ms2))
                / (2.0 * step)
            )
        return np.column_stack(columns)

    p = p0.copy()
    damping = 1e-3
    success = False
    message = "maximum iterations reached"
    f_scale = 0.02
    for iteration in range(500):
        r = residual(p, meas, gravity_ms2)
        jac = numeric_jacobian(p)
        weights = 1.0 / np.sqrt(1.0 + (r / f_scale) ** 2)
        root_w = np.sqrt(weights)
        jw = jac * root_w[:, None]
        rw = r * root_w
        hessian = jw.T @ jw
        gradient = jw.T @ rw
        diagonal = np.maximum(np.diag(hessian), 1e-10)
        try:
            delta = np.linalg.solve(
                hessian + damping * np.diag(diagonal), -gradient
            )
        except np.linalg.LinAlgError:
            damping *= 10.0
            if damping > 1e14:
                message = "singular normal equations"
                break
            continue
        candidate = np.clip(p + delta, lower, upper)
        old_cost = float(np.sum(2.0 * f_scale ** 2 * (np.sqrt(1.0 + (r / f_scale) ** 2) - 1.0)))
        new_r = residual(candidate, meas, gravity_ms2)
        new_cost = float(np.sum(2.0 * f_scale ** 2 * (np.sqrt(1.0 + (new_r / f_scale) ** 2) - 1.0)))
        if new_cost < old_cost:
            p = candidate
            damping = max(damping / 3.0, 1e-12)
            if np.linalg.norm(delta) < 1e-11 * (1.0 + np.linalg.norm(p)):
                success = True
                message = f"converged in {iteration + 1} iterations"
                break
            if abs(old_cost - new_cost) < 1e-15 * (1.0 + old_cost):
                success = True
                message = f"cost converged in {iteration + 1} iterations"
                break
        else:
            damping *= 10.0
            if damping > 1e14:
                message = "damping overflow without a decreasing step"
                break
    final_jac = numeric_jacobian(p)
    matrix, bias = build_A(p)
    return AccelFit(
        params=p.copy(),
        matrix=matrix,
        bias_ms2=np.asarray(bias).copy(),
        residual_ms2=residual(p, meas, gravity_ms2),
        jacobian=final_jac,
        success=success,
        message=message,
    )


def coverage_metrics(measurements_ms2: np.ndarray) -> dict:
    meas = np.asarray(measurements_ms2, dtype=np.float64)
    directions = meas / np.linalg.norm(meas, axis=1, keepdims=True)
    eig = np.linalg.eigvalsh(directions.T @ directions / len(directions))
    axis_min = directions.min(axis=0)
    axis_max = directions.max(axis=0)
    dots = np.clip(directions @ directions.T, -1.0, 1.0)
    np.fill_diagonal(dots, 1.0)
    angles = np.degrees(np.arccos(dots))
    upper = angles[np.triu_indices(len(directions), 1)]
    octants = {
        tuple(int(x >= 0.0) for x in row)
        for row in directions
        if np.all(np.abs(row) > 0.05)
    }
    return {
        "scatter_eigenvalues": eig.tolist(),
        "scatter_condition": float(eig[-1] / max(eig[0], np.finfo(float).eps)),
        "mean_direction_norm": float(np.linalg.norm(directions.mean(axis=0))),
        "axis_min": axis_min.tolist(),
        "axis_max": axis_max.tolist(),
        "axis_span": (axis_max - axis_min).tolist(),
        "minimum_pair_angle_deg": float(upper.min()) if len(upper) else 0.0,
        "octant_count": len(octants),
    }


def normalized_jacobian_condition(jacobian: np.ndarray) -> tuple[int, float, float]:
    jac = np.asarray(jacobian, dtype=np.float64)
    singular_raw = np.linalg.svd(jac, compute_uv=False)
    raw_condition = float(singular_raw[0] / max(singular_raw[-1], np.finfo(float).eps))
    norms = np.linalg.norm(jac, axis=0)
    normalized = jac / np.maximum(norms, np.finfo(float).eps)
    singular = np.linalg.svd(normalized, compute_uv=False)
    tol = max(normalized.shape) * np.finfo(float).eps * singular[0]
    rank = int(np.sum(singular > tol))
    condition = float(singular[0] / max(singular[-1], np.finfo(float).eps))
    return rank, condition, raw_condition


def _split_score(means: np.ndarray, fit_indices: np.ndarray, holdout_indices: np.ndarray) -> float:
    cov = coverage_metrics(means[fit_indices])
    eig0 = cov["scatter_eigenvalues"][0]
    span = min(cov["axis_span"])
    score = 12.0 * eig0 + 0.6 * min(span, 2.0) - 1.2 * cov["mean_direction_norm"]
    if len(holdout_indices) >= 2:
        u = means[holdout_indices]
        u = u / np.linalg.norm(u, axis=1, keepdims=True)
        pair = np.degrees(np.arccos(np.clip(u @ u.T, -1.0, 1.0)))
        score += 0.002 * pair[np.triu_indices(len(u), 1)].min()
    return float(score)


def choose_validation_split(
    measurements_ms2: np.ndarray,
    *,
    minimum_fit_poses: int = 12,
    desired_holdout_poses: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(measurements_ms2)
    if n < minimum_fit_poses:
        raise CalibrationError(f"need at least {minimum_fit_poses} stable orientations; got {n}")
    h = min(max(int(desired_holdout_poses), 0), 4, n - minimum_fit_poses)
    all_indices = np.arange(n, dtype=np.int32)
    if h <= 0:
        return all_indices, np.empty(0, dtype=np.int32)
    best = None
    best_score = -float("inf")
    for combo in itertools.combinations(range(n), h):
        hold = np.asarray(combo, dtype=np.int32)
        mask = np.ones(n, dtype=bool)
        mask[hold] = False
        fit = all_indices[mask]
        score = _split_score(measurements_ms2, fit, hold)
        if score > best_score:
            best_score = score
            best = (fit, hold)
    assert best is not None
    return best


def residual_stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(values)),
        "rms_ms2": float(np.sqrt(np.mean(values ** 2))) if len(values) else None,
        "median_abs_ms2": float(np.median(np.abs(values))) if len(values) else None,
        "max_abs_ms2": float(np.max(np.abs(values))) if len(values) else None,
    }


def estimate_stationary_noise(capture: dict, final_fit: AccelFit) -> dict:
    pose_index = np.asarray(capture["sample_pose_index"], dtype=np.int32)
    stamp_ns = np.asarray(capture["sample_stamp_ns"], dtype=np.int64)
    accel = np.asarray(capture["sample_accel_ms2"], dtype=np.float64)
    gyro = np.asarray(capture["sample_gyro_rad_s"], dtype=np.float64)
    rates = []
    durations = []
    accel_parts = []
    gyro_parts = []
    for pose in np.unique(pose_index):
        mask = pose_index == pose
        t = np.sort(stamp_ns[mask])
        dt = np.diff(t).astype(np.float64) * 1e-9
        dt = dt[dt > 0.0]
        if len(dt):
            rates.append(1.0 / float(np.median(dt)))
            durations.append(float((t[-1] - t[0]) * 1e-9))
        corrected = (accel[mask] - final_fit.bias_ms2) @ final_fit.matrix.T
        accel_parts.append(corrected - corrected.mean(axis=0))
        gyro_parts.append(gyro[mask] - gyro[mask].mean(axis=0))
    if not rates:
        raise CalibrationError("sample timestamps do not identify a positive IMU sample rate")
    sample_rate = float(np.median(rates))
    accel_residual = np.concatenate(accel_parts)
    gyro_residual = np.concatenate(gyro_parts)
    accel_std = accel_residual.std(axis=0, ddof=1)
    gyro_std = gyro_residual.std(axis=0, ddof=1)
    accel_density = accel_std / math.sqrt(sample_rate)
    gyro_density = gyro_std / math.sqrt(sample_rate)
    gyro_bias = gyro.mean(axis=0)
    gyro_static_residual = gyro - gyro_bias
    return {
        "sample_rate_hz": sample_rate,
        "window_duration_s_median": float(np.median(durations)) if durations else 0.0,
        "accel_sample_std_ms2": accel_std.tolist(),
        "gyro_sample_std_rad_s": gyro_std.tolist(),
        "accel_noise_density_ms2_sqrt_hz": accel_density.tolist(),
        "gyro_noise_density_rad_s_sqrt_hz": gyro_density.tolist(),
        "gyro_bias_rad_s": gyro_bias.tolist(),
        "gyro_static_residual_rms_rad_s": float(
            np.sqrt(np.mean(np.sum(gyro_static_residual ** 2, axis=1)))
        ),
        "noise_density_method": "short_window_white_noise",
        "allan_characterization": "not_performed",
        "interpretation": (
            "Within-pose demeaned short stationary windows; this estimates the white-noise "
            "floor only and is not an Allan bias-instability/random-walk measurement."
        ),
    }


def _acceptance_reasons(
    full_fit: AccelFit,
    fit_coverage: dict,
    full_coverage: dict,
    jac_rank: int,
    jac_condition: float,
    train_stats: dict,
    validation_stats: dict,
    holdout_count: int,
) -> list[str]:
    reasons: list[str] = []
    if not full_fit.success:
        reasons.append(f"optimizer did not converge: {full_fit.message}")
    if jac_rank < 9:
        reasons.append(f"accelerometer Jacobian rank is {jac_rank}, expected 9")
    if not np.isfinite(jac_condition) or jac_condition > 200.0:
        reasons.append(f"normalized accelerometer Jacobian condition is {jac_condition:.3g} > 200")
    for label, coverage in (("fit", fit_coverage), ("full", full_coverage)):
        if coverage["scatter_eigenvalues"][0] < 0.075:
            reasons.append(f"{label} orientation scatter minimum eigenvalue is below 0.075")
        if min(coverage["axis_span"]) < 1.15:
            reasons.append(f"{label} orientations do not cover both signs of every axis")
        if coverage["mean_direction_norm"] > 0.60:
            reasons.append(f"{label} orientations are one-sided (mean direction > 0.60)")
    if train_stats["rms_ms2"] is None or train_stats["rms_ms2"] > 0.08:
        reasons.append("fit gravity-norm residual RMS exceeds 0.08 m/s^2")
    if holdout_count:
        if validation_stats["rms_ms2"] is None or validation_stats["rms_ms2"] > 0.15:
            reasons.append("holdout gravity-norm residual RMS exceeds 0.15 m/s^2")
        if validation_stats["max_abs_ms2"] is None or validation_stats["max_abs_ms2"] > 0.35:
            reasons.append("holdout maximum gravity-norm residual exceeds 0.35 m/s^2")
    else:
        reasons.append("no independent holdout pose; capture at least 13 total orientations")
    p = full_fit.params
    if np.any((p[3:6] < 0.85) | (p[3:6] > 1.15)):
        reasons.append("estimated accelerometer scale is outside [0.85, 1.15]")
    if np.max(np.abs(p[:3])) > 0.10:
        reasons.append("estimated non-orthogonality exceeds 0.10 rad")
    if np.max(np.abs(p[6:9])) > 1.0:
        reasons.append("estimated accelerometer bias exceeds 1.0 m/s^2")
    return reasons


def analyze_capture(
    capture: dict,
    *,
    gravity_ms2: float,
    minimum_fit_poses: int = 12,
    desired_holdout_poses: int = 3,
) -> tuple[dict, dict[str, np.ndarray]]:
    validate_capture(capture)
    means = np.asarray(capture["pose_accel_mean_ms2"], dtype=np.float64)
    fit_indices, holdout_indices = choose_validation_split(
        means,
        minimum_fit_poses=minimum_fit_poses,
        desired_holdout_poses=desired_holdout_poses,
    )
    training_fit = fit_accelerometer(means[fit_indices], gravity_ms2)
    train_res = residual(training_fit.params, means[fit_indices], gravity_ms2)
    holdout_res = residual(training_fit.params, means[holdout_indices], gravity_ms2)
    full_fit = fit_accelerometer(means, gravity_ms2)
    full_res = residual(full_fit.params, means, gravity_ms2)
    rank, jac_condition, raw_jac_condition = normalized_jacobian_condition(full_fit.jacobian)
    fit_coverage = coverage_metrics(means[fit_indices])
    full_coverage = coverage_metrics(means)
    train_stats = residual_stats(train_res)
    validation_stats = residual_stats(holdout_res)
    full_stats = residual_stats(full_res)
    noise = estimate_stationary_noise(capture, full_fit)
    reasons = _acceptance_reasons(
        full_fit,
        fit_coverage,
        full_coverage,
        rank,
        jac_condition,
        train_stats,
        validation_stats,
        len(holdout_indices),
    )
    p = full_fit.params
    metadata = capture.get("metadata", {})
    summaries = capture.get("window_summaries", [])
    model = {
        "frame": "mid360s_imu_frame",
        "accel_input_unit": "g",
        "accel_output_unit": "m/s^2",
        "driver_g_to_ms2_scale": STANDARD_GRAVITY,
        "accel_equation": ACCEL_EQUATION,
        "T_misalignment": [
            [1.0, -float(p[0]), float(p[1])],
            [0.0, 1.0, -float(p[2])],
            [0.0, 0.0, 1.0],
        ],
        "accel_scale": p[3:6].tolist(),
        "accel_misalignment_rad": p[:3].tolist(),
        "accel_bias_ms2": p[6:9].tolist(),
        "accel_correction_matrix": full_fit.matrix.tolist(),
        "gyro_bias_rad_s": noise["gyro_bias_rad_s"],
        "gravity_reference_ms2": float(gravity_ms2),
    }
    document = {
        "schema": SCHEMA,
        "intended_local_schema_after_review": INTENDED_LOCAL_SCHEMA,
        "status": "accepted" if not reasons else "rejected",
        "scope": "operational_analysis_only_not_formal_result",
        "generated_at_utc": utc_now(),
        "source": {
            "role": "operational_capture",
            "capture_metadata": metadata,
            "stable_window_summaries": summaries,
        },
        "frame_convention": {
            "frame": "mid360s_imu_frame",
            "accel_equation": ACCEL_EQUATION,
        },
        "result": model,
        "validation": {
            "method": "independent_orientation_holdout",
            "pose_count": int(len(means)),
            "fit_pose_count": int(len(fit_indices)),
            "holdout_pose_count": int(len(holdout_indices)),
            "fit_indices": fit_indices.tolist(),
            "holdout_indices": holdout_indices.tolist(),
            "fit_residual": train_stats,
            "holdout_residual": validation_stats,
            "full_refit_residual": full_stats,
            "training_model_parameters": training_fit.params.tolist(),
        },
        "observability": {
            "parameter_count": 9,
            "jacobian_rank": rank,
            "jacobian_condition_column_normalized": jac_condition,
            "jacobian_condition_raw": raw_jac_condition,
            "fit_orientation_coverage": fit_coverage,
            "full_orientation_coverage": full_coverage,
        },
        "stationary_noise": noise,
        "acceptance": {
            "passed": not reasons,
            "rejection_reasons": reasons,
            "policy": {
                "minimum_fit_poses": minimum_fit_poses,
                "desired_holdout_poses": desired_holdout_poses,
                "minimum_jacobian_rank": 9,
                "maximum_normalized_jacobian_condition": 200.0,
                "maximum_fit_residual_rms_ms2": 0.08,
                "maximum_holdout_residual_rms_ms2": 0.15,
                "maximum_holdout_residual_abs_ms2": 0.35,
            },
        },
        "limitations": {
            "gyro_scale_and_misalignment": "not_observable_without_known-rate excitation",
            "allan_bias_instability_and_random_walk": "not_performed; requires a long stationary run",
            "noise_claim": "short-window white-noise density only",
        },
    }
    arrays = {
        "pose_accel_mean_ms2": means,
        "fit_indices": fit_indices,
        "holdout_indices": holdout_indices,
        "full_parameters": full_fit.params,
        "full_correction_matrix": full_fit.matrix,
        "full_residual_ms2": full_res,
        "training_parameters": training_fit.params,
        "training_residual_ms2": train_res,
        "holdout_residual_ms2": holdout_res,
        "gyro_bias_rad_s": np.asarray(noise["gyro_bias_rad_s"]),
        "accel_noise_density_ms2_sqrt_hz": np.asarray(
            noise["accel_noise_density_ms2_sqrt_hz"]
        ),
        "gyro_noise_density_rad_s_sqrt_hz": np.asarray(
            noise["gyro_noise_density_rad_s_sqrt_hz"]
        ),
        "analysis_json": np.asarray(json.dumps(document, ensure_ascii=False, sort_keys=True)),
    }
    return document, arrays


def _refuse_formal_results_path(path: Path) -> None:
    if "results" in {part.lower() for part in path.resolve().parts}:
        raise CalibrationError(
            f"refusing formal results path {path}; this tool writes analysis only"
        )


def write_analysis(json_path: Path, npz_path: Path, document: dict, arrays: dict) -> None:
    json_path = Path(json_path)
    npz_path = Path(npz_path)
    _refuse_formal_results_path(json_path)
    _refuse_formal_results_path(npz_path)
    for path in (json_path, npz_path):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite existing analysis: {path}")
    if json_path.suffix.lower() != ".json" or npz_path.suffix.lower() != ".npz":
        raise CalibrationError("analysis outputs must use .json and .npz suffixes")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation closes the race between the preflight and write.
    with json_path.open("x", encoding="utf-8") as f:
        json.dump(document, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")
    try:
        with npz_path.open("xb") as f:
            np.savez_compressed(f, **arrays)
    except Exception:
        # Do not leave a one-sided analysis pair.
        try:
            json_path.unlink()
        except OSError:
            pass
        raise


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Solve MID-360S IMU operational intrinsics")
    ap.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="one or more stable-pose .npz files / ROS 2 bag directories",
    )
    ap.add_argument("--out-json", required=True, type=Path, help="new analysis JSON, outside results/")
    ap.add_argument("--out-npz", type=Path, help="new analysis NPZ; defaults beside JSON")
    ap.add_argument("--lat", type=float, default=22.3)
    ap.add_argument("--alt", type=float, default=30.0)
    ap.add_argument("--gravity", type=float, default=None, help="override local gravity m/s^2")
    ap.add_argument("--minimum-fit-poses", type=int, default=12)
    ap.add_argument("--holdout-poses", type=int, default=3)
    ap.add_argument("--topic", default=DEFAULT_TOPIC, help="ROS bag input only")
    ap.add_argument(
        "--frame", default="livox_frame",
        help="required sensor_msgs/Imu header.frame_id for ROS bag input",
    )
    ap.add_argument("--mid360s-serial", default="", help="required for ROS bag input")
    ap.add_argument("--rig-id", default="", help="required for ROS bag input")
    ap.add_argument("--mount-id", default="", help="ROS bag input identity")
    ap.add_argument("--hold", type=float, default=0.5, help="ROS bag stable window seconds")
    ap.add_argument("--min-sep", type=float, default=18.0, help="ROS bag pose separation degrees")
    args = ap.parse_args(argv)
    if args.out_npz is None:
        args.out_npz = args.out_json.with_suffix(".npz")
    if not 2 <= args.holdout_poses <= 4:
        ap.error("--holdout-poses must be 2..4")
    if args.minimum_fit_poses < 12:
        ap.error("--minimum-fit-poses must be at least 12")
    for path in (args.out_json, args.out_npz):
        try:
            _refuse_formal_results_path(path)
        except CalibrationError as exc:
            ap.error(str(exc))
        if path.exists():
            ap.error(f"refusing to overwrite existing analysis: {path}")
    if any(path.is_dir() for path in args.inputs) and (not args.mid360s_serial or not args.rig_id):
        ap.error("ROS bag input requires --mid360s-serial and --rig-id")
    if not args.frame.strip():
        ap.error("--frame must be non-empty")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        captures = []
        for input_path in args.inputs:
            if input_path.is_dir():
                captures.append(capture_from_rosbag(
                    input_path,
                    topic=args.topic,
                    frame=args.frame,
                    serial=args.mid360s_serial,
                    rig_id=args.rig_id,
                    mount_id=args.mount_id,
                    collector_kwargs={
                        "hold_s": args.hold,
                        "min_separation_deg": args.min_sep,
                    },
                ))
            else:
                captures.append(load_capture_npz(input_path))
        capture = merge_captures(captures, args.min_sep)
        gravity = args.gravity if args.gravity is not None else local_gravity(args.lat, args.alt)
        document, arrays = analyze_capture(
            capture,
            gravity_ms2=gravity,
            minimum_fit_poses=args.minimum_fit_poses,
            desired_holdout_poses=args.holdout_poses,
        )
        document["source"]["inputs"] = [
            {"path": path.as_posix(), "sha256": sha256_path(path)}
            for path in args.inputs
        ]
        # Bag inputs are checked against this frame message-by-message.  For a
        # legacy NPZ without frame metadata this remains an explicit operator
        # assertion carried into the promotion boundary.
        document["source"]["expected_ros_frame"] = args.frame
        document["gravity_reference"] = {
            "value_ms2": float(gravity),
            "method": "explicit_override" if args.gravity is not None else "WGS84_normal_gravity",
            "latitude_deg": None if args.gravity is not None else args.lat,
            "altitude_m": None if args.gravity is not None else args.alt,
        }
        # Refresh the embedded document after adding source provenance.
        arrays["analysis_json"] = np.asarray(
            json.dumps(document, ensure_ascii=False, sort_keys=True)
        )
        write_analysis(args.out_json, args.out_npz, document, arrays)
    except (CalibrationError, FileExistsError, OSError, ValueError) as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 2

    result = document["result"]
    val = document["validation"]
    obs = document["observability"]
    noise = document["stationary_noise"]
    print(f"Analysis status: {document['status'].upper()}")
    print(f"  poses {val['fit_pose_count']} fit + {val['holdout_pose_count']} holdout")
    print(
        f"  residual RMS fit={val['fit_residual']['rms_ms2']:.5f}, "
        f"holdout={val['holdout_residual']['rms_ms2']:.5f} m/s^2"
    )
    print(
        f"  scale={np.round(result['accel_scale'], 7)}  "
        f"accel_bias={np.round(result['accel_bias_ms2'], 6)} m/s^2"
    )
    print(f"  gyro_bias={np.round(result['gyro_bias_rad_s'], 8)} rad/s")
    print(
        f"  rank={obs['jacobian_rank']}/9  "
        f"normalized condition={obs['jacobian_condition_column_normalized']:.2f}"
    )
    print(
        f"  short-window white noise @ {noise['sample_rate_hz']:.2f} Hz; "
        "Allan characterization not performed"
    )
    if not document["acceptance"]["passed"]:
        for reason in document["acceptance"]["rejection_reasons"]:
            print(f"  REJECT: {reason}")
    print(f"Analysis JSON -> {args.out_json}")
    print(f"Analysis NPZ  -> {args.out_npz}")
    return 0 if document["acceptance"]["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
