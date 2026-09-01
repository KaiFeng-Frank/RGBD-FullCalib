#!/usr/bin/env python3
"""Rotation-only Livox scan deskew using per-point times and IMU gyros.

The module deliberately has no ROS dependency.  A transport adapter supplies
``CustomMsg.timebase``, each point's ``offset_time`` and stamped gyroscope
samples.  Points are expressed in the LiDAR/IMU frame and are rotated to the
end-of-scan frame with

    p_end = R_end.T @ R(t_point) @ p_point

where ``R`` maps the sensor frame at a given time into an arbitrary common
frame.  The arbitrary initial orientation cancels in the relative rotation.

This is rotation-only deskew.  When a calibrated ``T_lidar_imu`` is supplied,
the deterministic motion of the LiDAR origin around the IMU lever arm is also
removed.  Platform translation still needs velocity, gravity and calibrated
accelerometer units, none of which are invented here.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import operator
import threading
from typing import Iterable

import numpy as np


class DeskewError(RuntimeError):
    """Base class for fail-closed deskew errors."""


class DeskewInputError(DeskewError, ValueError):
    """The point, timestamp or IMU input is malformed."""


class ImuCoverageError(DeskewError):
    """The IMU ring does not bracket the requested scan interval."""


@dataclass(frozen=True)
class DeskewResult:
    """Deskewed points and the exact time interval used to produce them."""

    points: np.ndarray
    scan_start_ns: int
    scan_end_ns: int
    reference_time_ns: int
    source_count: int
    output_count: int
    imu_samples_used: int
    lever_arm_applied: bool


def _integer_ns(value: object, name: str) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise DeskewInputError(f"{name} must be an integer nanosecond stamp")
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise DeskewInputError(
            f"{name} must be an integer nanosecond stamp") from exc
    if result < np.iinfo(np.int64).min or result > np.iinfo(np.int64).max:
        raise DeskewInputError(f"{name} is outside int64 range")
    return int(result)


def _gyro_vector(value: object) -> np.ndarray:
    gyro = np.asarray(value, dtype=np.float64)
    if gyro.shape != (3,):
        raise DeskewInputError("angular_velocity_rad_s must have shape (3,)")
    if not np.isfinite(gyro).all():
        raise DeskewInputError("angular_velocity_rad_s must be finite")
    return gyro


class ImuRing:
    """Bounded, thread-safe ring of strictly increasing gyro samples.

    Timestamps must use the same nanosecond clock as the Livox point timebase.
    Strict ordering is enforced at insertion so a later deskew cannot silently
    interpolate across a duplicate or time reversal.
    """

    def __init__(self, max_samples: int = 4096):
        if isinstance(max_samples, bool) or not isinstance(
                max_samples, (int, np.integer)) or int(max_samples) < 2:
            raise ValueError("max_samples must be an integer >= 2")
        self.max_samples = int(max_samples)
        self._samples: deque[tuple[int, np.ndarray]] = deque(
            maxlen=self.max_samples)
        self._lock = threading.RLock()

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def add(self, timestamp_ns: int, angular_velocity_rad_s: object) -> None:
        """Append one stamped angular-velocity sample."""
        self.extend((timestamp_ns,), (angular_velocity_rad_s,))

    def extend(self, timestamps_ns: Iterable[int],
               angular_velocities_rad_s: Iterable[object]) -> None:
        """Atomically validate and append multiple samples."""
        timestamps_raw = list(timestamps_ns)
        gyros_raw = list(angular_velocities_rad_s)
        if len(timestamps_raw) != len(gyros_raw):
            raise DeskewInputError("IMU timestamps and gyros have different lengths")
        if not timestamps_raw:
            return
        timestamps = [
            _integer_ns(value, f"IMU timestamp[{index}]")
            for index, value in enumerate(timestamps_raw)
        ]
        gyros = [_gyro_vector(value).copy() for value in gyros_raw]
        if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
            raise DeskewInputError("IMU timestamps must be strictly increasing")

        with self._lock:
            if self._samples and timestamps[0] <= self._samples[-1][0]:
                raise DeskewInputError(
                    "IMU timestamps must be newer than the ring's last sample")
            self._samples.extend(zip(timestamps, gyros))

    def covers(self, start_ns: int, end_ns: int) -> bool:
        """Return whether samples bracket both ends of ``[start_ns, end_ns]``."""
        try:
            self.window(start_ns, end_ns)
        except (DeskewInputError, ImuCoverageError):
            return False
        return True

    def window(self, start_ns: int, end_ns: int) -> tuple[np.ndarray, np.ndarray]:
        """Return the minimal sample window bracketing an interval.

        One sample at or before the start and one at or after the end are
        included.  This permits interpolation at both scan boundaries.
        """
        start = _integer_ns(start_ns, "start_ns")
        end = _integer_ns(end_ns, "end_ns")
        if end < start:
            raise DeskewInputError("end_ns must be >= start_ns")
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 2:
            raise ImuCoverageError("at least two IMU samples are required")

        timestamps = np.fromiter(
            (sample[0] for sample in samples), dtype=np.int64,
            count=len(samples))
        left = int(np.searchsorted(timestamps, start, side="right") - 1)
        right = int(np.searchsorted(timestamps, end, side="left"))
        if left < 0 or right >= len(timestamps):
            available = f"[{int(timestamps[0])}, {int(timestamps[-1])}]"
            raise ImuCoverageError(
                f"IMU samples {available} do not bracket [{start}, {end}]")
        # A zero-duration scan on an exact sample still needs a second sample
        # to define interpolation.  Include the adjacent one deterministically.
        if right == left:
            if right + 1 < len(timestamps):
                right += 1
            elif left > 0:
                left -= 1
            else:  # Defensive; len(samples) >= 2 makes this unreachable.
                raise ImuCoverageError("cannot form a two-sample IMU window")
        gyros = np.stack([samples[index][1]
                          for index in range(left, right + 1)]).astype(
                              np.float64, copy=False)
        return timestamps[left:right + 1].copy(), gyros.copy()


def _skew_batch(vectors: np.ndarray) -> np.ndarray:
    result = np.zeros((len(vectors), 3, 3), dtype=np.float64)
    x, y, z = vectors[:, 0], vectors[:, 1], vectors[:, 2]
    result[:, 0, 1] = -z
    result[:, 0, 2] = y
    result[:, 1, 0] = z
    result[:, 1, 2] = -x
    result[:, 2, 0] = -y
    result[:, 2, 1] = x
    return result


def _so3_exp_batch(rotation_vectors: np.ndarray) -> np.ndarray:
    """Vectorized SO(3) exponential map for an ``(N, 3)`` array."""
    vectors = np.asarray(rotation_vectors, dtype=np.float64)
    if vectors.ndim != 2 or vectors.shape[1] != 3:
        raise DeskewInputError("rotation vectors must have shape (N, 3)")
    theta2 = np.einsum("ni,ni->n", vectors, vectors)
    theta = np.sqrt(theta2)
    small = theta2 < 1e-12
    a = np.empty(len(vectors), dtype=np.float64)
    b = np.empty(len(vectors), dtype=np.float64)
    # Stable Taylor series around zero.
    a[small] = 1.0 - theta2[small] / 6.0 + theta2[small] ** 2 / 120.0
    b[small] = 0.5 - theta2[small] / 24.0 + theta2[small] ** 2 / 720.0
    a[~small] = np.sin(theta[~small]) / theta[~small]
    b[~small] = (1.0 - np.cos(theta[~small])) / theta2[~small]
    skew = _skew_batch(vectors)
    identity = np.broadcast_to(np.eye(3, dtype=np.float64), skew.shape).copy()
    return identity + a[:, None, None] * skew + b[:, None, None] * (skew @ skew)


def _integrate_imu_orientations(timestamps_ns: np.ndarray,
                                gyros: np.ndarray) -> np.ndarray:
    """Integrate body-frame gyros at their sample stamps using trapezoids."""
    count = len(timestamps_ns)
    orientations = np.empty((count, 3, 3), dtype=np.float64)
    orientations[0] = np.eye(3, dtype=np.float64)
    deltas_s = np.diff(timestamps_ns).astype(np.float64) * 1e-9
    if np.any(deltas_s <= 0.0):
        raise DeskewInputError("IMU timestamps must be strictly increasing")
    increments = 0.5 * (gyros[:-1] + gyros[1:]) * deltas_s[:, None]
    rotations = _so3_exp_batch(increments)
    for index, rotation in enumerate(rotations, start=1):
        orientations[index] = orientations[index - 1] @ rotation
    return orientations


def _orientations_at(timestamps_ns: np.ndarray, gyros: np.ndarray,
                     orientations: np.ndarray,
                     query_ns: np.ndarray) -> np.ndarray:
    """Interpolate gyro and integrate from the preceding IMU sample."""
    query = np.asarray(query_ns, dtype=np.int64)
    if query.ndim != 1:
        raise DeskewInputError("orientation query times must be one-dimensional")
    if len(query) == 0:
        return np.empty((0, 3, 3), dtype=np.float64)
    if np.any(query < timestamps_ns[0]) or np.any(query > timestamps_ns[-1]):
        raise ImuCoverageError("orientation query lies outside the IMU window")

    segment = np.searchsorted(timestamps_ns, query, side="right") - 1
    segment = np.clip(segment, 0, len(timestamps_ns) - 2)
    t0 = timestamps_ns[segment]
    t1 = timestamps_ns[segment + 1]
    span = (t1 - t0).astype(np.float64)
    elapsed_ns = (query - t0).astype(np.float64)
    alpha = elapsed_ns / span
    gyro0 = gyros[segment]
    gyro_query = gyro0 + alpha[:, None] * (gyros[segment + 1] - gyro0)
    increments = 0.5 * (gyro0 + gyro_query) * (elapsed_ns * 1e-9)[:, None]
    return orientations[segment] @ _so3_exp_batch(increments)


def _selection_indices(indices: object, count: int) -> np.ndarray:
    if indices is None:
        return np.arange(count, dtype=np.int64)
    if isinstance(indices, slice):
        return np.arange(count, dtype=np.int64)[indices]
    selected = np.asarray(indices)
    if selected.ndim != 1:
        raise DeskewInputError("indices must be one-dimensional")
    if selected.dtype.kind == "b":
        if len(selected) != count:
            raise DeskewInputError("boolean indices must match the point count")
        return np.flatnonzero(selected).astype(np.int64)
    if selected.dtype.kind not in "iu":
        raise DeskewInputError("indices must be integers, a boolean mask, or a slice")
    selected = selected.astype(np.int64, copy=False)
    if np.any(selected < 0) or np.any(selected >= count):
        raise DeskewInputError("indices contain an out-of-range point index")
    return selected


def _lidar_imu_transform(value: object) -> tuple[np.ndarray, np.ndarray, bool]:
    """Validate ``p_lidar = T_lidar_imu * p_imu`` and return R, t."""
    if value is None:
        return (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64),
                False)
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise DeskewInputError("T_lidar_imu must be a finite 4x4 transform")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0),
                       rtol=0.0, atol=1e-9):
        raise DeskewInputError("T_lidar_imu must have a rigid homogeneous row")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0.0,
                        atol=1e-7) or
            not np.isclose(np.linalg.det(rotation), 1.0, rtol=0.0,
                           atol=1e-7)):
        raise DeskewInputError("T_lidar_imu rotation must be proper orthonormal")
    translation = transform[:3, 3].copy()
    return rotation.copy(), translation, bool(np.linalg.norm(translation) > 0.0)


def rotation_deskew_to_scan_end(
        points_xyz: object, offset_time_ns: object, timebase_ns: int,
        imu: ImuRing, *, indices: object = None,
        T_lidar_imu: object = None) -> DeskewResult:
    """Rotate selected Livox points into the full scan's end frame.

    ``offset_time_ns`` must describe every row of ``points_xyz``.  The scan end
    is calculated from the complete offset array before optional ``indices``
    are applied, so display downsampling cannot change the reference pose.
    Selected points must be finite.  Non-selected invalid geometry is allowed,
    which lets a caller pass its existing finite-point index filter directly.

    ``T_lidar_imu`` follows ``p_lidar = T_lidar_imu * p_imu``.  Its rotation
    maps IMU gyros into the LiDAR axes; its translation enables the known
    lever-arm component of rotation compensation.  It does not estimate or
    compensate platform translation.

    The function fails closed with :class:`ImuCoverageError` unless the IMU ring
    brackets both the minimum and maximum per-point timestamp.
    """
    if not isinstance(imu, ImuRing):
        raise DeskewInputError("imu must be an ImuRing")
    points = np.asarray(points_xyz)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewInputError("points_xyz must have shape (N, 3)")
    if points.dtype.kind not in "fiu":
        raise DeskewInputError("points_xyz must be numeric")
    count = len(points)
    if count == 0:
        raise DeskewInputError("cannot deskew an empty scan")

    offsets_raw = np.asarray(offset_time_ns)
    if offsets_raw.ndim != 1 or len(offsets_raw) != count:
        raise DeskewInputError("offset_time_ns must have one value per point")
    if offsets_raw.dtype.kind not in "iu" or offsets_raw.dtype.kind == "b":
        raise DeskewInputError("offset_time_ns must contain integer nanoseconds")
    if offsets_raw.dtype.kind == "i" and np.any(offsets_raw < 0):
        raise DeskewInputError("offset_time_ns cannot be negative")
    # uint64 values above int64 max must not wrap during conversion.
    if offsets_raw.dtype.kind == "u" and np.any(
            offsets_raw > np.iinfo(np.int64).max):
        raise DeskewInputError("offset_time_ns is outside int64 range")
    offsets = offsets_raw.astype(np.int64, copy=False)

    base = _integer_ns(timebase_ns, "timebase_ns")
    minimum_offset = int(np.min(offsets))
    maximum_offset = int(np.max(offsets))
    scan_start = base + minimum_offset
    scan_end = base + maximum_offset
    if scan_start < np.iinfo(np.int64).min or scan_end > np.iinfo(np.int64).max:
        raise DeskewInputError("absolute point timestamps are outside int64 range")

    selected = _selection_indices(indices, count)
    selected_points = np.asarray(points[selected], dtype=np.float64)
    if not np.isfinite(selected_points).all():
        raise DeskewInputError("selected points_xyz must be finite")

    rotation_lidar_imu, translation_lidar_imu, lever_arm_applied = (
        _lidar_imu_transform(T_lidar_imu))
    imu_times, gyros_imu = imu.window(scan_start, scan_end)
    gyros_lidar = gyros_imu @ rotation_lidar_imu.T
    imu_orientations = _integrate_imu_orientations(imu_times, gyros_lidar)
    point_times = base + offsets[selected]
    point_orientations = _orientations_at(
        imu_times, gyros_lidar, imu_orientations, point_times)
    end_orientation = _orientations_at(
        imu_times, gyros_lidar, imu_orientations,
        np.asarray([scan_end], dtype=np.int64))[0]
    relative = np.swapaxes(end_orientation, 0, 1)[None, :, :] @ point_orientations
    centered = selected_points - translation_lidar_imu
    output = (np.einsum("nij,nj->ni", relative, centered) +
              translation_lidar_imu)

    return DeskewResult(
        points=np.ascontiguousarray(output),
        scan_start_ns=scan_start,
        scan_end_ns=scan_end,
        reference_time_ns=scan_end,
        source_count=count,
        output_count=len(selected),
        imu_samples_used=len(imu_times),
        lever_arm_applied=lever_arm_applied,
    )


__all__ = [
    "DeskewError",
    "DeskewInputError",
    "ImuCoverageError",
    "DeskewResult",
    "ImuRing",
    "rotation_deskew_to_scan_end",
]
