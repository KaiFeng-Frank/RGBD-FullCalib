#!/usr/bin/env python3
"""Vendor-neutral ROS 2 point-cloud source.

The normal path consumes ``sensor_msgs/msg/PointCloud2``.  Livox
``CustomMsg`` is also accepted when the driver overlay is sourced, so the
viewer can attach to an already-running MID-360 without forcing a second
driver or throwing away its per-point timing at the publisher boundary.

Livox ``CustomMsg`` scans can be rotation-deskewed before the display
downsampling step.  The source retains every point's ``offset_time`` long
enough to interpolate the device IMU, then emits one cloud referenced to the
scan end.  Ring/tag fields and covariance are still not carried by the browser
protocol.
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

from .base import Source
try:
    from ..livox_deskew import (ImuCoverageError, ImuRing,
                                rotation_deskew_to_scan_end)
except ImportError:  # ``server.py`` imports ``sources`` as a top-level package.
    from livox_deskew import (ImuCoverageError, ImuRing,
                              rotation_deskew_to_scan_end)


POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2'
LIVOX_CUSTOM = 'livox_ros_driver2/msg/CustomMsg'
SUPPORTED_TYPES = (POINTCLOUD2, LIVOX_CUSTOM)


def _absolute_topic(topic: str) -> str:
    topic = topic.strip()
    if topic in ('', 'auto') or topic.startswith(('/', '~')):
        return topic or 'auto'
    return '/' + topic


def _point_topics(node) -> list[tuple[str, str]]:
    out = []
    for name, types in node.get_topic_names_and_types():
        if node.count_publishers(name) < 1:
            continue
        for msg_type in SUPPORTED_TYPES:
            if msg_type in types:
                out.append((name, msg_type))
                break
    return sorted(out)


def discover_point_topics(timeout: float = 1.0) -> list[tuple[str, str]]:
    """Return currently advertised supported point streams."""
    import rclpy

    owned = not rclpy.ok()
    if owned:
        rclpy.init(args=[])
    node = rclpy.create_node(f'pointcloud_viewer_discovery_{os.getpid()}')
    try:
        deadline = time.monotonic() + max(0.0, timeout)
        found = []
        while time.monotonic() <= deadline:
            found = _point_topics(node)
            if found:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        return found
    finally:
        node.destroy_node()
        if owned and rclpy.ok():
            rclpy.shutdown()


def _resolve_axes(requested: str, frame_id: str) -> str:
    if requested != 'auto':
        return requested
    # REP-103 camera optical frames are x-right, y-down, z-forward.  Other
    # ROS frames (including Livox) are treated as x-forward, y-left, z-up.
    return 'optical' if 'optical' in frame_id.lower() else 'ros'


def _to_view_axes(xyz: np.ndarray, axes: str) -> np.ndarray:
    """Map source coordinates to WebGL's x-right, y-up, z-back convention."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if axes == 'viewer':
        return np.ascontiguousarray(xyz)
    out = np.empty_like(xyz)
    if axes == 'optical':
        out[:, 0] = xyz[:, 0]
        out[:, 1] = -xyz[:, 1]
        out[:, 2] = -xyz[:, 2]
    elif axes == 'ros':
        out[:, 0] = -xyz[:, 1]
        out[:, 1] = xyz[:, 2]
        out[:, 2] = -xyz[:, 0]
    else:
        raise ValueError(f'unknown axes mode: {axes}')
    return out


def _header_time(msg) -> float:
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    if stamp is not None:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if value > 0:
            return value
    return time.time()


def _message_stamp_ns(msg, msg_type: str) -> int | None:
    """Extract a device/header stamp so replayed clouds cannot fake liveness."""
    if msg_type == LIVOX_CUSTOM:
        try:
            timebase = int(getattr(msg, 'timebase', 0))
        except (TypeError, ValueError):
            return None
        if timebase > 0:
            return timebase
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    if stamp is None:
        return None
    try:
        sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def _structured_points(msg) -> np.ndarray:
    """Create a structured view that also respects organized-cloud row padding."""
    from sensor_msgs_py.point_cloud2 import dtype_from_fields

    width, height = int(msg.width), int(msg.height)
    point_step = int(msg.point_step)
    dtype = dtype_from_fields(msg.fields, point_step=point_step if point_step > 0 else None)
    if width <= 0 or height <= 0:
        return np.empty(0, dtype=dtype)
    if point_step <= 0:
        raise ValueError('PointCloud2 point_step must be positive for a non-empty cloud')
    row_step = int(msg.row_step) or width * point_step
    minimum_row_step = width * point_step
    if row_step < minimum_row_step:
        raise ValueError(
            f'PointCloud2 row_step too small: {row_step} < {minimum_row_step}')
    required = (height - 1) * row_step + width * point_step
    if len(msg.data) < required:
        raise ValueError(
            f'PointCloud2 payload too short: {len(msg.data)} < {required} bytes')
    points = np.ndarray(
        shape=(height, width), dtype=dtype, buffer=msg.data,
        strides=(row_step, point_step))
    points = points.reshape(-1)
    if bool(sys.byteorder != 'little') != bool(msg.is_bigendian):
        points = points.byteswap()
    return points


def _packed_rgb(values: np.ndarray) -> np.ndarray | None:
    values = np.ascontiguousarray(values)
    if values.dtype.kind == 'f' and values.dtype.itemsize == 4:
        packed = values.astype(np.float32, copy=False).view(np.uint32)
    elif values.dtype.kind in 'ui' and values.dtype.itemsize <= 4:
        packed = values.astype(np.uint32, copy=False)
    else:
        return None
    return np.column_stack(((packed >> 16) & 255,
                            (packed >> 8) & 255,
                            packed & 255)).astype(np.uint8)


class Ros2Points(Source):
    name = 'ros2'

    def __init__(self, on_frame, topic='auto', max_points=250_000,
                 axes='auto', topic_timeout=10.0, deskew=True,
                 imu_topic='/livox/imu', time_offset_s=0.0,
                 T_lidar_imu=None):
        super().__init__(on_frame)
        self.topic = _absolute_topic(topic)
        self.max_points = max(1, int(max_points))
        self.axes_requested = axes
        self.topic_timeout = float(topic_timeout)
        self.deskew_requested = bool(deskew)
        self.imu_topic = _absolute_topic(imu_topic)
        self.time_offset_s = float(time_offset_s)
        if not math.isfinite(self.time_offset_s):
            raise ValueError('time_offset_s must be finite')
        self.T_lidar_imu = T_lidar_imu
        self._imu = ImuRing(max_samples=4096)
        self._pending_custom = None
        self._deskew_dropped = 0
        self._deskew_applied = False
        self._deskew_last = None
        self._meta = None
        self._seq = 0
        self._intensity_limits = None
        self._last_stamp_ns = None
        self._saw_unstamped = False

    def meta(self):
        return self._meta

    def _advances_stream(self, nonempty: bool, stamp_ns: int | None) -> bool:
        if not nonempty:
            return False
        if stamp_ns is None:
            if self._saw_unstamped:
                return False
            self._saw_unstamped = True
            return True
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            return False
        self._last_stamp_ns = stamp_ns
        return True

    def _choose_topic(self, node) -> tuple[str, str]:
        deadline = time.monotonic() + self.topic_timeout
        last = []
        while not self._stop.is_set() and time.monotonic() <= deadline:
            last = _point_topics(node)
            if self.topic != 'auto':
                matches = [x for x in last if x[0] == self.topic]
                if matches:
                    return matches[0]
            elif len(last) == 1:
                return last[0]
            elif len(last) > 1:
                choices = ', '.join(f'{n} [{t}]' for n, t in last)
                raise RuntimeError(
                    f'发现多个点云话题，请用 --topic 指定: {choices}')
            time.sleep(0.1)
        choices = ', '.join(f'{n} [{t}]' for n, t in last) or '无'
        wanted = '任一唯一点云话题' if self.topic == 'auto' else self.topic
        raise RuntimeError(
            f'等待 {wanted} 超时；当前可用点云话题: {choices}')

    def _normalise_intensity(self, values: np.ndarray) -> np.ndarray:
        values = np.asarray(values, dtype=np.float32)
        finite = values[np.isfinite(values)]
        if not len(finite):
            return np.zeros(len(values), np.float32)
        lo, hi = np.percentile(finite, (1.0, 99.0))
        if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 1e-9:
            lo, hi = float(np.min(finite)), float(np.max(finite))
        if hi - lo < 1e-9:
            return np.full(len(values), 0.5, np.float32)
        if self._intensity_limits is None:
            self._intensity_limits = [float(lo), float(hi)]
        else:
            # Slow adaptation prevents the display from flickering when a single
            # high-reflectivity target enters the scan.
            a = 0.08
            self._intensity_limits[0] = (1 - a) * self._intensity_limits[0] + a * float(lo)
            self._intensity_limits[1] = (1 - a) * self._intensity_limits[1] + a * float(hi)
        lo, hi = self._intensity_limits
        normalised = np.clip((values - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
        normalised[~np.isfinite(normalised)] = 0.0
        return normalised.astype(np.float32)

    def _pointcloud2_arrays(self, msg):
        points = _structured_points(msg)
        names = set(points.dtype.names or ())
        if not {'x', 'y', 'z'} <= names:
            raise ValueError(
                f'PointCloud2 must contain x/y/z fields; got {sorted(names)}')

        x = np.asarray(points['x'], dtype=np.float32)
        y = np.asarray(points['y'], dtype=np.float32)
        z = np.asarray(points['z'], dtype=np.float32)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        valid &= (np.abs(x) + np.abs(y) + np.abs(z)) > 1e-9
        idx = np.flatnonzero(valid)
        if len(idx) > self.max_points:
            idx = idx[::math.ceil(len(idx) / self.max_points)]
        xyz = np.column_stack((x[idx], y[idx], z[idx])).astype(np.float32)

        lower = {n.lower(): n for n in names}
        intensity = None
        for candidate in ('intensity', 'reflectivity', 'reflectance', 'i'):
            if candidate in lower:
                intensity = self._normalise_intensity(points[lower[candidate]][idx])
                break

        rgb = None
        for candidate in ('rgb', 'rgba'):
            if candidate in lower:
                rgb = _packed_rgb(points[lower[candidate]][idx])
                break
        if rgb is None and all(k in lower for k in ('r', 'g', 'b')):
            rgb = np.column_stack(tuple(
                np.asarray(points[lower[k]][idx], dtype=np.uint8)
                for k in ('r', 'g', 'b')))
        if rgb is None:
            for prefix in ('rgb', 'rgba'):
                channels = tuple(f'{prefix}_{i}' for i in range(3))
                if all(k in lower for k in channels):
                    rgb = np.column_stack(tuple(
                        np.asarray(points[lower[k]][idx], dtype=np.uint8)
                        for k in channels))
                    break
        return xyz, intensity, rgb, [f.name for f in msg.fields]

    def _custom_arrays(self, msg, *, apply_deskew=False):
        reported = int(getattr(msg, 'point_num', 0))
        total = min(reported if reported > 0 else len(msg.points), len(msg.points))
        if total <= 0:
            return (np.empty((0, 3), np.float32), None, None,
                    ['x', 'y', 'z', 'reflectivity', 'offset_time', 'tag', 'line'],
                    int(getattr(msg, 'timebase', 0)))
        xyz = np.empty((total, 3), np.float32)
        intensity = np.empty(total, np.float32)
        offsets = np.empty(total, np.uint32)
        for i in range(total):
            p = msg.points[i]
            xyz[i] = (p.x, p.y, p.z)
            intensity[i] = p.reflectivity
            offsets[i] = p.offset_time
        valid = np.isfinite(xyz).all(axis=1)
        valid &= np.abs(xyz).sum(axis=1) > 1e-9
        selected = np.flatnonzero(valid)
        if len(selected) > self.max_points:
            selected = selected[::math.ceil(len(selected) / self.max_points)]

        reference_ns = int(getattr(msg, 'timebase', 0))
        if apply_deskew:
            result = rotation_deskew_to_scan_end(
                xyz, offsets, reference_ns, self._imu, indices=selected,
                T_lidar_imu=self.T_lidar_imu)
            output = result.points.astype(np.float32)
            reference_ns = result.reference_time_ns
            self._deskew_applied = True
            self._deskew_last = result
        else:
            output = xyz[selected]
        return (output, self._normalise_intensity(intensity[selected]), None,
                ['x', 'y', 'z', 'reflectivity', 'offset_time', 'tag', 'line'],
                reference_ns)

    def _emit(self, msg, msg_type: str):
        frame_id = getattr(getattr(msg, 'header', None), 'frame_id', '') or ''
        axes = _resolve_axes(self.axes_requested, frame_id)
        if msg_type == POINTCLOUD2:
            xyz, intensity, rgb, fields = self._pointcloud2_arrays(msg)
            stamp = _header_time(msg)
        else:
            try:
                xyz, intensity, rgb, fields, reference_ns = self._custom_arrays(
                    msg, apply_deskew=self.deskew_requested)
            except ImuCoverageError:
                return False
            stamp = (reference_ns * 1e-9 if reference_ns > 0
                     else _header_time(msg)) + self.time_offset_s
        advances_stream = self._advances_stream(
            bool(len(xyz)), _message_stamp_ns(msg, msg_type))
        xyz = _to_view_axes(xyz, axes)

        # Empty clouds are legal during filter warm-up or complete no-return
        # scans. Wait for the first non-empty cloud before freezing metadata and
        # auto-fit; after startup, an empty frame is forwarded to clear the view.
        if not len(xyz) and self._meta is None:
            return

        if self._meta is None:
            label = 'Livox ROS 2' if msg_type == LIVOX_CUSTOM else 'ROS 2 PointCloud2'
            if len(xyz):
                lo, hi = np.percentile(xyz, (2.0, 98.0), axis=0)
                center = ((lo + hi) * 0.5).tolist()
                span = float(np.max(hi - lo))
            else:
                center, span = [0.0, 0.0, -3.0], 5.0
            self._meta = dict(
                source=label, kind='point_stream', topic=self.topic,
                message_type=msg_type, frame_id=frame_id or '(empty)',
                fields=fields, has_intensity=intensity is not None,
                has_color=rgb is not None, axes_input=axes,
                axes_view='x-right, y-up, z-back', max_points=self.max_points,
                point_count_raw=(int(getattr(msg, 'point_num', 0))
                                 if msg_type == LIVOX_CUSTOM
                                 else int(msg.width) * int(msg.height)),
                qos='best_effort / volatile / keep_last(1)',
                recommended_max_range=30.0 if 'livox' in label.lower() else 20.0,
                view_center=center, view_distance=max(3.0, min(100.0, span * 1.2)),
            )
            if msg_type == LIVOX_CUSTOM:
                self._meta.update(
                    deskew=dict(
                        applied=self._deskew_applied,
                        mode='rotation_only' if self._deskew_applied else 'off',
                        reference='scan_end' if self._deskew_applied else 'timebase',
                        imu_topic=self.imu_topic if self._deskew_applied else None,
                        per_point_time_field='offset_time',
                        lever_arm_applied=(
                            bool(self._deskew_last.lever_arm_applied)
                            if self._deskew_last is not None else False),
                        dropped_before_coverage=self._deskew_dropped,
                    ),
                    time_offset_s=self.time_offset_s,
                )
        self.on_frame('points', dict(
            seq=self._seq, t=stamp, xyz=xyz, intensity=intensity, rgb=rgb,
            _counts_as_freshness=advances_stream))
        self._seq += 1
        return True

    def _run(self):
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)

        owned = not rclpy.ok()
        if owned:
            # Never feed server.py's own CLI flags to rclpy.
            rclpy.init(args=[])
        node = rclpy.create_node(f'universal_pointcloud_viewer_{os.getpid()}')
        try:
            try:
                self.topic, msg_type = self._choose_topic(node)
            except Exception:
                if self._stop.is_set() or not rclpy.ok():
                    return
                raise
            if msg_type == POINTCLOUD2:
                from sensor_msgs.msg import PointCloud2
                msg_cls = PointCloud2
            else:
                try:
                    from livox_ros_driver2.msg import CustomMsg
                except ImportError as exc:
                    raise RuntimeError(
                        'Livox CustomMsg topic found, but livox_ros_driver2 overlay '
                        'is not sourced') from exc
                msg_cls = CustomMsg

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            if msg_type == LIVOX_CUSTOM and self.deskew_requested:
                from sensor_msgs.msg import Imu

                def try_pending():
                    if self._pending_custom is None:
                        return
                    pending = self._pending_custom
                    if self._emit(pending, msg_type):
                        self._pending_custom = None

                def on_imu(msg):
                    stamp = msg.header.stamp
                    timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    gyro = msg.angular_velocity
                    self._imu.add(timestamp_ns, (gyro.x, gyro.y, gyro.z))
                    try_pending()

                def on_custom(msg):
                    if self._pending_custom is not None:
                        self._deskew_dropped += 1
                    self._pending_custom = msg
                    try_pending()

                node.create_subscription(Imu, self.imu_topic, on_imu, qos)
                node.create_subscription(msg_cls, self.topic, on_custom, qos)
            else:
                node.create_subscription(
                    msg_cls, self.topic, lambda msg: self._emit(msg, msg_type), qos)
            print(f'[ROS 2] 订阅 {self.topic} [{msg_type}]  QoS=best_effort/volatile')
            if msg_type == LIVOX_CUSTOM and self.deskew_requested:
                print(f'[ROS 2] 旋转 deskew: {self.imu_topic} + offset_time -> scan_end')
            try:
                while not self._stop.is_set() and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                # SIGINT lets rclpy shut the context down before asyncio's
                # finally block asks this source thread to stop.
                pass
        finally:
            node.destroy_node()
            if owned and rclpy.ok():
                rclpy.shutdown()
