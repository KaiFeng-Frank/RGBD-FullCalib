#!/usr/bin/env python3
"""D435i / ROS 2 / Mid-360 通用实时点云查看器 —— 服务端。

  python server.py                    # 真机
  python server.py --source synthetic # 合成场景(相机被占用时也能开发/验证)
  python server.py --source synthetic-points
  python server.py --source ros2 --topic /points
  然后浏览器打开 http://localhost:8080

丢帧而非排队:
  源线程只把"最新一帧"写进 latest,广播协程按自己的节奏取。
  客户端慢下来时它看到的是当前画面,而不是一条越拖越长的历史队列 ——
  实时查看器里,迟到的正确帧不如准时的近似帧。
"""
import argparse
import asyncio
import functools
import hashlib
import http.server
import json
import os
import socketserver
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import protocol as P
import calib_summary

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXTRINSIC_RESULT = os.path.join(ROOT, 'results', 'mid360s_d435i_extrinsic.json')
EXTRINSIC_TASK_ID = 'mid360s_d435i_ext'
EXTRINSIC_EQUATION = 'p_camera = T_camera_lidar * p_lidar'
TIMESYNC_RESULT = os.path.join(ROOT, 'results', 'mid360s_d435i_timesync.json')
TIMESYNC_LOCAL_SCHEMA = 'd435i_calib/lidar_camera_timesync_operational/v1'
TIMESYNC_EQUATION = (
    't_d435i_depth = t_livox_scan + '
    'time_offset_lidar_to_d435_depth_ms / 1000'
)
LIDAR_IMU_RESULT = os.path.join(ROOT, 'results', 'mid360s_lidar_imu.json')
LIDAR_IMU_LOCAL_SCHEMA = 'd435i_calib/mid360s_lidar_imu_operational/v1'
LIDAR_IMU_EQUATION = 'p_lidar = T_lidar_imu * p_imu'


def _strict_json(path):
    """Read JSON without accepting NaN/Inf or duplicate keys."""
    with open(path, encoding='utf-8') as stream:
        return _strict_json_text(stream.read())


def _strict_json_text(text):
    """Parse JSON text without accepting NaN/Inf or duplicate keys."""
    def reject_constant(value):
        raise ValueError(f'non-finite JSON constant is forbidden: {value}')

    def unique_object(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ValueError(f'duplicate JSON key is forbidden: {key}')
            out[key] = value
        return out

    return json.loads(text, parse_constant=reject_constant,
                      object_pairs_hook=unique_object)


def _device_serials(doc):
    devices = doc.get('devices')
    if not isinstance(devices, list) or len(devices) != 2:
        raise ValueError('devices must contain exactly lidar and rgbd identities')
    by_role = {}
    for item in devices:
        if not isinstance(item, dict):
            raise ValueError('each devices item must be an object')
        role = item.get('role')
        serial = item.get('serial')
        if role in by_role or role not in ('lidar', 'rgbd'):
            raise ValueError('devices roles must be unique and exactly lidar/rgbd')
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError(f'devices[{role!r}].serial must be non-empty')
        by_role[role] = serial.strip()
    if set(by_role) != {'lidar', 'rgbd'}:
        raise ValueError('devices roles must be exactly lidar/rgbd')
    return by_role


def require_preview_device_serial(preview, role, observed):
    """Bind a loaded transform to the physical device producing this stream."""
    key = {'lidar': 'lidar_serial', 'rgbd': 'camera_serial'}.get(role)
    if key is None:
        raise ValueError(f'unknown extrinsic device role: {role}')
    expected = preview.get(key)
    if not isinstance(observed, str) or not observed.strip():
        raise ValueError(f'online {role} serial is unavailable')
    if observed.strip() != expected:
        raise ValueError(
            f'extrinsic {role} serial={expected!r}, online serial={observed.strip()!r}')


def parse_livox_device_info(payload):
    """Parse the transient-local device identity published by Driver2."""
    begin, end = payload.find('{'), payload.rfind('}')
    if begin < 0 or end < begin:
        raise ValueError('Livox device_info contains no JSON object')
    doc = _strict_json_text(payload[begin:end + 1])
    if not isinstance(doc, dict) or doc.get('schema') != 'livox_ros_driver2/device_info/v1':
        raise ValueError('unsupported Livox device_info schema')
    serial = doc.get('serial_number')
    if not isinstance(serial, str) or not serial.strip():
        raise ValueError('Livox device_info serial_number is missing')
    return {
        'serial': serial.strip(),
        'lidar_ip': str(doc.get('lidar_ip', '')).strip(),
        'device_info_schema': doc['schema'],
    }


def read_livox_device_info(topic='/livox/device_info'):
    """Read one retained identity sample without guessing identity from IP."""
    try:
        run = subprocess.run(
            ['ros2', 'topic', 'echo', topic, '--field', 'data', '--once',
             '--timeout', '6', '--qos-reliability', 'reliable',
             '--qos-durability', 'transient_local'],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=9, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(f'cannot read {topic}: {exc}') from exc
    if run.returncode != 0 or not run.stdout.strip():
        raise ValueError(
            f'cannot read retained Livox identity from {topic}: '
            f'{run.stderr.strip() or "no message"}')
    return parse_livox_device_info(run.stdout)


def _checked_camera_lidar_matrix(doc):
    """Return a finite SE(3) whose declared direction is LiDAR -> color camera."""
    if doc.get('task_id') != EXTRINSIC_TASK_ID:
        raise ValueError(f'task_id must be {EXTRINSIC_TASK_ID}')
    frame = doc.get('frame_convention')
    if (not isinstance(frame, dict) or frame.get('from') != 'livox_frame' or
            frame.get('to') != 'camera_color_optical_frame' or
            frame.get('equation') != EXTRINSIC_EQUATION):
        raise ValueError(
            'frame_convention must declare livox_frame -> '
            f'camera_color_optical_frame and "{EXTRINSIC_EQUATION}"')
    result = doc.get('result')
    matrix = np.asarray(result.get('T_camera_lidar') if isinstance(result, dict)
                        else None, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('result.T_camera_lidar must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-10):
        raise ValueError('T_camera_lidar has an invalid homogeneous row')
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-8) or
            not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-8)):
        raise ValueError('T_camera_lidar rotation is not a proper orthonormal matrix')
    return matrix


def load_extrinsic_preview(draft_path=None, local_path=None):
    """Load a registry result, an explicit legacy draft, or a local calibration.

    Merely placing a JSON file in ``results/`` never enables an overlay.  The
    canonical result still passes the project registry.  Open-source users can
    instead pass their own ``status=operational`` result with ``--extrinsic``;
    it is checked for transform direction, SE(3), frames and device identities,
    then labelled LOCAL without requiring this workstation's audit registry.
    """
    if draft_path is not None and local_path is not None:
        raise ValueError('choose only one of --extrinsic and --extrinsic-draft')
    explicit_draft = draft_path is not None
    explicit_local = local_path is not None
    if explicit_draft or explicit_local:
        selected = draft_path if explicit_draft else local_path
        path = os.path.abspath(os.path.expanduser(selected))
    else:
        path = EXTRINSIC_RESULT
    if not os.path.exists(path):
        if explicit_draft:
            raise ValueError(f'explicit extrinsic draft does not exist: {path}')
        return {
            'available': False, 'status': 'none',
            'reason': 'validated extrinsic result is not present',
            'expected_path': os.path.relpath(path, ROOT),
        }
    try:
        with open(path, 'rb') as stream:
            raw = stream.read()
        doc = _strict_json(path)
        if not isinstance(doc, dict):
            raise ValueError('JSON root must be an object')
        matrix = _checked_camera_lidar_matrix(doc)
        serials = _device_serials(doc)
        rig_id = doc.get('rig_id')
        if not isinstance(rig_id, str) or not rig_id.strip():
            raise ValueError('rig_id must be non-empty')
        mount_session_id = doc.get('mount_session_id')
        if not isinstance(mount_session_id, str) or not mount_session_id.strip():
            raise ValueError('mount_session_id must be non-empty')
        if explicit_draft:
            if str(doc.get('status', '')).lower() != 'draft':
                raise ValueError('an explicit draft preview must have status=draft')
            if doc.get('draft_schema') != 'd435i_calib/lidar_camera_extrinsic_draft/v1':
                raise ValueError('unsupported or missing draft_schema')
            status = 'draft'
        elif explicit_local:
            if str(doc.get('status', '')).lower() != 'operational':
                raise ValueError('an explicit local calibration must have status=operational')
            if doc.get('local_schema') != 'd435i_calib/lidar_camera_extrinsic_local/v1':
                raise ValueError('unsupported or missing local_schema')
            status = 'operational'
        else:
            # Reuse the frozen project registry so status text alone can never
            # promote a file into the live overlay.
            import lidar_calib
            task = next(item for item in lidar_calib.TASKS
                        if item['id'] == EXTRINSIC_TASK_ID)
            lifecycle, reasons = lidar_calib._validate(task, doc)
            if lifecycle != 'done':
                raise ValueError('registry validation failed: ' + '; '.join(reasons))
            status = 'validated'
        return {
            'available': True,
            'status': status,
            'label': {'draft': 'DRAFT', 'operational': 'LOCAL'}.get(
                status, 'VALIDATED'),
            'task_id': EXTRINSIC_TASK_ID,
            'rig_id': rig_id.strip(),
            'mount_session_id': mount_session_id.strip(),
            'lidar_serial': serials['lidar'],
            'camera_serial': serials['rgbd'],
            'path': path if explicit_draft else os.path.relpath(path, ROOT),
            'sha256': hashlib.sha256(raw).hexdigest(),
            'from_frame': 'livox_frame',
            'to_frame': 'camera_color_optical_frame',
            'equation': EXTRINSIC_EQUATION,
            'units': 'metres',
            'T_camera_lidar': matrix.tolist(),
        }
    except Exception as exc:
        if explicit_draft or explicit_local:
            kind = 'draft' if explicit_draft else 'local calibration'
            raise ValueError(f'invalid explicit extrinsic {kind} {path}: {exc}') from exc
        return {
            'available': False, 'status': 'invalid',
            'reason': str(exc), 'expected_path': os.path.relpath(path, ROOT),
        }


def load_timesync_result(path=None, extrinsic_preview=None):
    """Load this rig's LiDAR-scan -> D435i-depth clock correction.

    The caller always gets a finite ``time_offset_s``.  Missing, malformed or
    mismatched results fail closed to zero instead of silently applying an
    offset from another rig or reversing the timestamp equation.
    """
    explicit = path is not None
    selected = (os.path.abspath(os.path.expanduser(path))
                if explicit else TIMESYNC_RESULT)
    display_path = (selected if explicit else os.path.relpath(selected, ROOT))
    fallback = {
        'available': False,
        'applied': False,
        'status': 'missing',
        'path': display_path,
        'time_offset_s': 0.0,
        'time_offset_ms': 0.0,
        'equation': TIMESYNC_EQUATION,
    }
    if not os.path.exists(selected):
        fallback['reason'] = f'timesync result is not present: {display_path}'
        return fallback
    try:
        with open(selected, 'rb') as stream:
            raw = stream.read()
        doc = _strict_json_text(raw.decode('utf-8'))
        if not isinstance(doc, dict):
            raise ValueError('JSON root must be an object')
        if str(doc.get('status', '')).lower() != 'operational':
            raise ValueError('status must be operational')
        if doc.get('local_schema') != TIMESYNC_LOCAL_SCHEMA:
            raise ValueError(
                f'local_schema must be {TIMESYNC_LOCAL_SCHEMA}')
        result = doc.get('result')
        if not isinstance(result, dict):
            raise ValueError('result must be an object')
        if result.get('time_offset_lidar_to_d435_depth_convention') != TIMESYNC_EQUATION:
            raise ValueError(
                'result.time_offset_lidar_to_d435_depth_convention has the '
                'wrong direction')
        offset_ms = result.get('time_offset_lidar_to_d435_depth_ms')
        if (isinstance(offset_ms, bool) or
                not isinstance(offset_ms, (int, float)) or
                not np.isfinite(float(offset_ms))):
            raise ValueError(
                'result.time_offset_lidar_to_d435_depth_ms must be finite')
        offset_ms = float(offset_ms)
        # A seconds-as-milliseconds mistake would visibly tear the overlay.
        # This is a format safety bound, not a calibration acceptance gate.
        if abs(offset_ms) > 1000.0:
            raise ValueError('depth-clock offset exceeds the 1000 ms safety bound')
        serials = _device_serials(doc)
        rig_id = doc.get('rig_id')
        mount_session_id = doc.get('mount_session_id')
        if not isinstance(rig_id, str) or not rig_id.strip():
            raise ValueError('rig_id must be non-empty')
        if not isinstance(mount_session_id, str) or not mount_session_id.strip():
            raise ValueError('mount_session_id must be non-empty')
        rig_id = rig_id.strip()
        mount_session_id = mount_session_id.strip()
        if extrinsic_preview and extrinsic_preview.get('available'):
            expected = extrinsic_preview
            checks = (
                ('rig_id', rig_id, expected.get('rig_id')),
                ('mount_session_id', mount_session_id,
                 expected.get('mount_session_id')),
                ('lidar serial', serials['lidar'],
                 expected.get('lidar_serial')),
                ('rgbd serial', serials['rgbd'],
                 expected.get('camera_serial')),
            )
            for label, observed, wanted in checks:
                if observed != wanted:
                    raise ValueError(
                        f'timesync {label}={observed!r} does not match '
                        f'extrinsic {wanted!r}')
        return {
            'available': True,
            'applied': True,
            'status': 'operational',
            'path': display_path,
            'sha256': hashlib.sha256(raw).hexdigest(),
            'rig_id': rig_id,
            'mount_session_id': mount_session_id,
            'lidar_serial': serials['lidar'],
            'camera_serial': serials['rgbd'],
            'time_offset_ms': offset_ms,
            'time_offset_s': offset_ms / 1000.0,
            'equation': TIMESYNC_EQUATION,
        }
    except Exception as exc:
        fallback['status'] = 'invalid'
        fallback['reason'] = str(exc)
        return fallback


def load_lidar_imu_result(extrinsic_preview=None):
    """Load the MID-360S built-in IMU pose used by rotational deskew.

    Identity is the fail-closed fallback: deskew may still use the measured
    angular velocity, but it must not invent a lever arm from an invalid or
    differently mounted result.
    """
    selected = LIDAR_IMU_RESULT
    display_path = os.path.relpath(selected, ROOT)
    fallback = {
        'available': False,
        'applied': False,
        'status': 'missing',
        'path': display_path,
        'equation': LIDAR_IMU_EQUATION,
        'T_lidar_imu': np.eye(4).tolist(),
    }
    if not os.path.exists(selected):
        fallback['reason'] = f'LiDAR-IMU result is not present: {display_path}'
        return fallback
    try:
        with open(selected, 'rb') as stream:
            raw = stream.read()
        doc = _strict_json_text(raw.decode('utf-8'))
        if not isinstance(doc, dict):
            raise ValueError('JSON root must be an object')
        if str(doc.get('status', '')).lower() != 'operational':
            raise ValueError('status must be operational')
        if doc.get('local_schema') != LIDAR_IMU_LOCAL_SCHEMA:
            raise ValueError(f'local_schema must be {LIDAR_IMU_LOCAL_SCHEMA}')
        frame = doc.get('frame_convention')
        if (not isinstance(frame, dict) or
                frame.get('from') != 'mid360s_imu_frame' or
                frame.get('to') != 'livox_frame' or
                frame.get('equation') != LIDAR_IMU_EQUATION):
            raise ValueError(
                'frame_convention must declare mid360s_imu_frame -> '
                f'livox_frame and "{LIDAR_IMU_EQUATION}"')
        result = doc.get('result')
        matrix = np.asarray(
            result.get('T_lidar_imu') if isinstance(result, dict) else None,
            dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('result.T_lidar_imu must be a finite 4x4 matrix')
        if not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-10):
            raise ValueError('T_lidar_imu has an invalid homogeneous row')
        rotation = matrix[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-8) or
                not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-8)):
            raise ValueError('T_lidar_imu rotation is not proper orthonormal')
        devices = doc.get('devices')
        if not isinstance(devices, list):
            raise ValueError('devices must be a list')
        lidars = [item for item in devices
                  if isinstance(item, dict) and item.get('role') == 'lidar']
        if len(lidars) != 1 or len(devices) != 1:
            raise ValueError('devices must contain exactly one lidar identity')
        serial = lidars[0].get('serial')
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError('lidar serial must be non-empty')
        serial = serial.strip()
        rig_id = doc.get('rig_id')
        mount_session_id = doc.get('mount_session_id')
        if not isinstance(rig_id, str) or not rig_id.strip():
            raise ValueError('rig_id must be non-empty')
        if not isinstance(mount_session_id, str) or not mount_session_id.strip():
            raise ValueError('mount_session_id must be non-empty')
        rig_id = rig_id.strip()
        mount_session_id = mount_session_id.strip()
        if extrinsic_preview and extrinsic_preview.get('available'):
            expected = extrinsic_preview
            checks = (
                ('rig_id', rig_id, expected.get('rig_id')),
                ('mount_session_id', mount_session_id,
                 expected.get('mount_session_id')),
                ('lidar serial', serial, expected.get('lidar_serial')),
            )
            for label, observed, wanted in checks:
                if observed != wanted:
                    raise ValueError(
                        f'LiDAR-IMU {label}={observed!r} does not match '
                        f'extrinsic {wanted!r}')
        return {
            'available': True,
            'applied': True,
            'status': 'operational',
            'path': display_path,
            'sha256': hashlib.sha256(raw).hexdigest(),
            'rig_id': rig_id,
            'mount_session_id': mount_session_id,
            'lidar_serial': serial,
            'equation': LIDAR_IMU_EQUATION,
            'T_lidar_imu': matrix.tolist(),
        }
    except Exception as exc:
        fallback['status'] = 'invalid'
        fallback['reason'] = str(exc)
        return fallback


def transform_livox_viewer_to_camera_viewer(xyz, transform):
    """Apply ``p_camera = T_camera_lidar * p_livox`` to viewer-wire points.

    ``Ros2Points`` puts a REP-103 Livox point ``[x forward,y left,z up]`` on
    the wire as ``[-y,z,-x]``.  The returned points use the color optical
    viewer convention ``[x right,-y down,-z forward]``.  All translations and
    point coordinates are metres.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'xyz must have shape (N,3), got {xyz.shape}')
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('transform must be 4x4')
    livox = np.empty((len(xyz), 3), dtype=np.float64)
    livox[:, 0] = -xyz[:, 2]
    livox[:, 1] = -xyz[:, 0]
    livox[:, 2] = xyz[:, 1]
    camera = livox @ matrix[:3, :3].T + matrix[:3, 3]
    out = np.empty((len(camera), 3), dtype=np.float32)
    out[:, 0] = camera[:, 0]
    out[:, 1] = -camera[:, 1]
    out[:, 2] = -camera[:, 2]
    return out


def load_thermal_model():
    """读温漂补偿模型。没有就返回 None —— 界面上会显示"未标定",不影响其它功能。"""
    p = os.path.join(ROOT, 'results', 'thermal_model.json')
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return json.load(f)
    except Exception:
        return None


def thermal_correction(model, T):
    """给定当前 ASIC 温度,算出各项补偿量。

    模型存的是"测量值随温度怎么变",补偿要反向施加:
    深度实测随温度变小 -> 补偿时要加回去。
    """
    if not model or T is None or not np.isfinite(T):
        return None
    dT = float(T) - float(model.get('T_ref_C', 25.0))
    out = {'T': float(T), 'T_ref': float(model.get('T_ref_C', 25.0)), 'dT': dT}
    if 'depth_ppm_per_C' in model:
        out['depth_ppm'] = -model['depth_ppm_per_C'] * dT
        out['depth_mm_at_ref'] = -model['depth_m_per_C'] * dT * 1000.0
    pp = model.get('principal_px_per_C')
    if pp:
        out['dcx_px'] = -pp['cx'] * dT
        out['dcy_px'] = -pp['cy'] * dT
    if 'focal_ppm_per_C' in model:
        out['focal_ppm'] = -model['focal_ppm_per_C'] * dT
    gb = model.get('gyro_bias_deg_s_per_C')
    if gb:
        out['gyro_bias'] = {k: -v * dT for k, v in gb.items()}
    ab = model.get('accel_bias_ms2_per_C')
    if ab:
        out['accel_bias'] = {k: -v * dT for k, v in ab.items()}
    return out


def load_calibration():
    """优先用我们自己标的内参 —— 出厂彩色内参畸变全 0,拿来做反投影会让点云边缘外扩。"""
    p = os.path.join(ROOT, 'data', 'cam_rgb-camchain.yaml')
    if not os.path.exists(p):
        return None
    out = {}
    for line in open(p):
        line = line.strip()
        for key in ('intrinsics', 'distortion_coeffs', 'resolution'):
            if line.startswith(key + ':'):
                out[key] = json.loads(line.split(':', 1)[1].strip())
    return out if 'intrinsics' in out else None


class Hub:
    def __init__(self):
        self.clients = set()
        self.latest = {}
        self.lock = threading.Lock()
        self.loop = None
        self.wake = None
        self.meta = None
        self.src = None
        self.thermal_model = None
        self.counts = {}
        self.t0 = time.time()
        self.last_frame_monotonic = None
        self.point_transform = None

    def on_frame(self, kind, payload):
        """由源线程调用。只留最新一帧。"""
        with self.lock:
            self.latest[kind] = payload
            self.counts[kind] = self.counts.get(kind, 0) + 1
            # Sources may forward an empty/auxiliary frame for display without
            # allowing it to impersonate a live point/depth stream.
            if payload.get('_counts_as_freshness', True):
                self.last_frame_monotonic = time.monotonic()
        if self.loop and self.wake:
            try:
                self.loop.call_soon_threadsafe(self.wake.set)
            except RuntimeError:
                pass

    def take(self):
        with self.lock:
            got, self.latest = self.latest, {}
            return got

    def stats(self):
        el = max(time.time() - self.t0, 1e-9)
        with self.lock:
            out = {'streams': {k: dict(count=v, fps=round(v / el, 2))
                               for k, v in self.counts.items()}}
        temps = {}
        if self.src is not None and hasattr(self.src, 'temperatures'):
            try:
                temps = self.src.temperatures()
            except Exception:
                temps = {}
        if temps:
            out['temps'] = temps
            corr = thermal_correction(self.thermal_model, temps.get('asic'))
            if corr:
                out['thermal'] = corr
        out['has_thermal_model'] = self.thermal_model is not None
        return out


async def broadcast_loop(hub, max_fps, stream_timeout=5.0):
    min_dt = 1.0 / max_fps if max_fps > 0 else 0.0
    last_stats = 0.0

    def check_source():
        if hub.src is None:
            return
        if hub.src.error:
            raise RuntimeError(f'数据源运行失败: {hub.src.error}') from hub.src.error
        if not hub.src.is_alive() and not hub.src.stop_requested():
            raise RuntimeError('数据源线程已意外停止')
        if stream_timeout > 0:
            with hub.lock:
                last_frame = hub.last_frame_monotonic
            if (last_frame is not None and
                    time.monotonic() - last_frame >= stream_timeout):
                age = time.monotonic() - last_frame
                raise RuntimeError(
                    f'数据源连续 {age:.1f} s 无帧，按断开处理并退出')

    while True:
        check_source()
        try:
            await asyncio.wait_for(hub.wake.wait(), timeout=0.5)
        except asyncio.TimeoutError:
            # A dead source cannot wake the hub. Periodic monitoring turns a
            # frozen last frame into an explicit process failure.
            continue
        hub.wake.clear()
        check_source()
        t = time.time()
        got = hub.take()
        if not hub.clients:
            if min_dt:
                await asyncio.sleep(min_dt)
            continue
        msgs = []
        if 'depth' in got:
            d = got['depth']
            msgs.append(P.pack_depth(d['seq'], d['t'], d['arr'], d['scale']))
        if 'color' in got:
            c = got['color']
            msgs.append(P.pack_color(c['seq'], c['t'], c['w'], c['h'], c['jpeg']))
        for key, lz in (('ir', 1), ('ir_clean', 0)):
            if key in got:
                i = got[key]
                msgs.append(P.pack_ir(i['seq'], i['t'], i['w'], i['h'], i['jpeg'],
                                      i.get('laser', lz)))
        if 'points' in got:
            p = got['points']
            xyz = p['xyz']
            if hub.point_transform is not None:
                xyz = transform_livox_viewer_to_camera_viewer(
                    xyz, hub.point_transform)
            msgs.append(P.pack_points(p['seq'], p['t'], xyz,
                                      p.get('intensity'), p.get('rgb')))
        if t - last_stats > 1.0:
            msgs.append(P.pack_stats(hub.stats()))
            last_stats = t
        if msgs and hub.clients:
            # Send to all clients concurrently and bound the whole batch per
            # client.  One browser that stops reading must never block the
            # source-disconnect watchdog in this same coroutine.
            async def send_batch(ws):
                for message in msgs:
                    await ws.send(message)

            clients = list(hub.clients)
            outcomes = await asyncio.gather(
                *(asyncio.wait_for(send_batch(ws), timeout=1.0)
                  for ws in clients),
                return_exceptions=True)
            for ws, outcome in zip(clients, outcomes):
                if isinstance(outcome, BaseException):
                    hub.clients.discard(ws)
        if min_dt:
            await asyncio.sleep(min_dt)


async def handler(ws, hub):
    hub.clients.add(ws)
    peer = getattr(ws, 'remote_address', None)
    print(f'[ws] 客户端接入 {peer}  (共 {len(hub.clients)})')
    try:
        if hub.meta:
            m = dict(hub.meta)
            if getattr(hub, 'meta_calib', None):
                m['calib'] = hub.meta_calib
            await asyncio.wait_for(ws.send(P.pack_meta(m)), timeout=1.0)
        async for _ in ws:
            pass
    except Exception:
        pass
    finally:
        hub.clients.discard(ws)
        print(f'[ws] 客户端断开 {peer}  (共 {len(hub.clients)})')


def serve_http(port, extrinsic_preview=None):
    class H(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def do_GET(self):
            if self.path.startswith('/extrinsic.json'):
                body = json.dumps(
                    extrinsic_preview or {
                        'available': False, 'status': 'none',
                        'reason': 'overlay mode was not requested',
                    }, ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers(); self.wfile.write(body); return
            if self.path.startswith('/calib.json'):
                try:
                    import calib_summary
                    body = json.dumps(calib_summary.collect(), ensure_ascii=False).encode()
                except Exception as e:
                    body = json.dumps({'error': str(e)}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers(); self.wfile.write(body); return
            if self.path.split('?')[0] in ('/', ''):
                self.path = '/viewer.html'
            return super().do_GET()

        def log_message(self, *a):
            pass

    socketserver.TCPServer.allow_reuse_address = True
    srv = socketserver.TCPServer(('0.0.0.0', port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


async def main_async(args):
    hub = Hub()
    hub.loop = asyncio.get_running_loop()
    hub.wake = asyncio.Event()
    extrinsic_preview = load_extrinsic_preview(
        args.extrinsic_draft, args.extrinsic)
    timesync_result = {
        'available': False, 'applied': False, 'status': 'not_requested',
        'time_offset_s': 0.0, 'time_offset_ms': 0.0,
        'equation': TIMESYNC_EQUATION,
    }
    lidar_imu_result = {
        'available': False, 'applied': False, 'status': 'not_requested',
        'equation': LIDAR_IMU_EQUATION, 'T_lidar_imu': np.eye(4).tolist(),
    }
    if args.overlay_role == 'lidar':
        timesync_result = load_timesync_result(
            args.timesync, extrinsic_preview=extrinsic_preview)
        lidar_imu_result = load_lidar_imu_result(
            extrinsic_preview=extrinsic_preview)
        if args.no_deskew and lidar_imu_result['available']:
            lidar_imu_result = dict(lidar_imu_result)
            lidar_imu_result['applied'] = False
            lidar_imu_result['reason'] = 'deskew was explicitly disabled'
        if timesync_result['available']:
            print(
                f"[timesync] 已应用 {timesync_result['time_offset_ms']:+.6f} ms "
                f"({TIMESYNC_EQUATION})  <- {timesync_result['path']}")
        else:
            print(
                '[timesync] 未应用时移，安全回退到 offset=0.000000 ms: '
                f"{timesync_result.get('reason', 'result unavailable')}")
        if lidar_imu_result['available'] and not args.no_deskew:
            lever_mm = np.asarray(
                lidar_imu_result['T_lidar_imu'], dtype=np.float64)[:3, 3] * 1000.0
            print('[LiDAR-IMU] 已应用 T_lidar_imu，IMU 杠杆臂 '
                  f'[{lever_mm[0]:+.2f}, {lever_mm[1]:+.2f}, '
                  f'{lever_mm[2]:+.2f}] mm')
        elif lidar_imu_result['available']:
            print('[LiDAR-IMU] 结果已载入；deskew 已关闭，杠杆臂未应用')
        else:
            print('[LiDAR-IMU] 未应用杠杆臂，安全回退到 identity: '
                  f"{lidar_imu_result.get('reason', 'result unavailable')}")

    if args.source == 'd435i':
        from sources.d435i import D435i
        calib = load_calibration()
        if calib:
            if args.align:
                print('[calib] 叠加模式保留 librealsense align 的实际彩色几何；'
                      '不用未重采样的自标定 K 替换它')
            else:
                print(f"[calib] 使用自标定彩色内参 fx={calib['intrinsics'][0]:.2f} "
                      f"cx={calib['intrinsics'][2]:.2f}  "
                      "(出厂畸变全 0,不可用于反投影)")
        src = D435i(hub.on_frame, jpeg_quality=args.jpeg,
                    align_to_color=args.align, calib_intrinsics=calib,
                    emitter_alternate=args.alt_emitter,
                    expected_serial=(extrinsic_preview.get('camera_serial')
                                     if args.overlay_role == 'rgbd' and
                                     extrinsic_preview.get('available') else None))
    elif args.source == 'synthetic':
        from sources.synthetic import Synthetic
        src = Synthetic(hub.on_frame, fps=args.fps)
    elif args.source == 'synthetic-points':
        from sources.synthetic_points import SyntheticPoints
        src = SyntheticPoints(hub.on_frame, fps=args.fps)
    elif args.source == 'ros2':
        from sources.ros2_points import Ros2Points
        src = Ros2Points(hub.on_frame, topic=args.topic,
                         max_points=args.max_points, axes=args.axes,
                         topic_timeout=args.topic_timeout,
                         deskew=not args.no_deskew,
                         time_offset_s=timesync_result['time_offset_s'],
                         T_lidar_imu=lidar_imu_result['T_lidar_imu'])
        if args.overlay_role == 'lidar':
            print('[deskew] ' + ('旋转补偿已开启（Livox offset_time + IMU '
                                  '→ scan_end）'
                                if not args.no_deskew else '已显式关闭'))
    else:
        print(f'未知源 {args.source}'); sys.exit(1)

    hub.src = src
    hub.thermal_model = load_thermal_model()
    if hub.thermal_model:
        m = hub.thermal_model
        print(f"[温漂] 已载入模型 (T_ref={m.get('T_ref_C')}C, "
              f"来自 {m.get('source')}, {m.get('duration_h', 0):.1f}h)")
    else:
        print("[温漂] 未找到 results/thermal_model.json —— 界面显示'未标定'")
    src.start()
    http_srv = None
    try:
        deadline = time.monotonic() + args.source_timeout
        while time.monotonic() <= deadline:       # 等源把 meta 准备好
            if src.error:
                raise RuntimeError(f'数据源启动失败: {src.error}') from src.error
            if not src.is_alive() and not src.stop_requested():
                raise RuntimeError('数据源在首帧前已意外停止')
            if src.meta():
                break
            await asyncio.sleep(0.05)
        hub.meta = src.meta()
        if not hub.meta:
            raise RuntimeError(f'数据源在 {args.source_timeout:g}s 内未初始化')

        if args.overlay_role == 'rgbd':
            if args.source != 'd435i':
                raise RuntimeError('overlay role rgbd requires --source d435i')
            if not hub.meta.get('aligned'):
                raise RuntimeError(
                    'RGB-D overlay requires depth aligned to '
                    'camera_color_optical_frame (--align)')
            depth_meta = hub.meta.get('depth') or {}
            if depth_meta.get('frame_id') != 'camera_color_optical_frame':
                raise RuntimeError(
                    'aligned RGB-D depth metadata is not in '
                    'camera_color_optical_frame')
            if extrinsic_preview.get('available'):
                require_preview_device_serial(
                    extrinsic_preview, 'rgbd', hub.meta.get('serial'))
                hub.meta = dict(hub.meta)
                hub.meta['rig_id'] = extrinsic_preview['rig_id']
                hub.meta['mount_session_id'] = extrinsic_preview['mount_session_id']
        elif args.overlay_role == 'lidar':
            if args.source != 'ros2':
                raise RuntimeError('overlay role lidar requires --source ros2')
            if extrinsic_preview.get('available'):
                if hub.meta.get('frame_id') != 'livox_frame':
                    raise RuntimeError(
                        'extrinsic overlay expects LiDAR input frame_id=livox_frame; '
                        f"got {hub.meta.get('frame_id')!r}")
                if hub.meta.get('axes_input') != 'ros':
                    raise RuntimeError(
                        'extrinsic overlay expects REP-103 ROS LiDAR axes; '
                        f"got {hub.meta.get('axes_input')!r}")
                livox_identity = read_livox_device_info()
                require_preview_device_serial(
                    extrinsic_preview, 'lidar', livox_identity['serial'])
                hub.point_transform = extrinsic_preview['T_camera_lidar']
                hub.meta = dict(hub.meta)
                hub.meta.update(
                    input_frame_id='livox_frame',
                    frame_id='camera_color_optical_frame',
                    axes_input='camera_color_optical',
                    axes_view='x-right, y-up, z-back',
                    units='metres',
                    transform_equation=EXTRINSIC_EQUATION,
                    serial=livox_identity['serial'],
                    lidar_ip=livox_identity['lidar_ip'],
                    device_info_schema=livox_identity['device_info_schema'],
                    rig_id=extrinsic_preview['rig_id'],
                    mount_session_id=extrinsic_preview['mount_session_id'],
                )
            hub.meta = dict(hub.meta)
            hub.meta['timesync'] = timesync_result
            hub.meta['lidar_imu'] = lidar_imu_result

        if args.overlay_role:
            hub.meta = dict(hub.meta)
            hub.meta['overlay_role'] = args.overlay_role
            hub.meta['extrinsic_preview'] = extrinsic_preview

        if args.source in ('d435i', 'synthetic'):
            try:
                hub.meta_calib = calib_summary.collect()
                counts = hub.meta_calib['counts']
                print(f"[标定] 载入 {counts['done']} 项标定结果")
            except Exception as e:
                hub.meta_calib = None
                print(f"[标定] 汇总失败: {e}")
        else:
            # Never label the repository's D435i verdicts as if they belonged
            # to an arbitrary ROS/LiDAR stream.
            hub.meta_calib = None

        if hub.meta.get('kind') == 'point_stream':
            print(f"[源] {hub.meta['source']}  {hub.meta.get('topic', '')}  "
                  f"frame={hub.meta.get('frame_id', '?')}  "
                  f"fields={','.join(hub.meta.get('fields', []))}")
        else:
            d = hub.meta['depth']
            print(f"[源] {hub.meta['source']}  depth {d['width']}x{d['height']}  "
                  f"fx={d['fx']:.2f} fy={d['fy']:.2f} "
                  f"cx={d['cx']:.2f} cy={d['cy']:.2f}")
            if hub.meta.get('usb'):
                print(f"[源] USB {hub.meta['usb']}"
                      + ("   ⚠ USB2 限制了帧率"
                         if hub.meta['usb'].startswith('2') else ""))

        if not args.no_http:
            http_srv = serve_http(args.http, extrinsic_preview)
            print(f'\n  打开浏览器:  http://localhost:{args.http}\n')
        else:
            print(f'\n  WebSocket only: ws://localhost:{args.ws}\n')

        import websockets
        async with websockets.serve(functools.partial(handler, hub=hub),
                                    '0.0.0.0', args.ws, max_size=None,
                                    compression=None):
            await broadcast_loop(hub, args.max_fps, args.stream_timeout)
    finally:
        src.stop()
        if http_srv is not None:
            http_srv.shutdown()
            http_srv.server_close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='d435i',
                    choices=['d435i', 'synthetic', 'synthetic-points', 'ros2'])
    ap.add_argument('--http', type=int, default=8080)
    ap.add_argument('--ws', type=int, default=9002)
    ap.add_argument('--no-http', action='store_true',
                    help='只启动 WebSocket；双源工作台的第二后端使用')
    ap.add_argument('--jpeg', type=int, default=75)
    ap.add_argument('--fps', type=int, default=10, help='合成源的帧率')
    ap.add_argument('--max-fps', type=float, default=30, help='广播上限')
    ap.add_argument('--topic', default='auto',
                    help='ROS 2 点云话题；auto 仅在唯一候选时自动选择')
    ap.add_argument('--max-points', type=int, default=250_000,
                    help='每帧最多传给浏览器的点数')
    ap.add_argument('--axes', choices=['auto', 'ros', 'optical', 'viewer'],
                    default='auto', help='输入坐标约定')
    ap.add_argument('--topic-timeout', type=float, default=10.0,
                    help='等待 ROS 2 话题的秒数')
    ap.add_argument('--source-timeout', type=float, default=15.0,
                    help='等待数据源首帧的秒数')
    ap.add_argument('--stream-timeout', type=float, default=5.0,
                    help='首帧后连续无帧的自动退出秒数；0 表示显式禁用')
    ap.add_argument('--list-topics', action='store_true',
                    help='列出当前 ROS 2 点云话题后退出')
    ap.add_argument('--align', action='store_true', help='把深度对齐到彩色视角')
    ap.add_argument('--alt-emitter', action='store_true',
                    help='发射器交替帧:散斑帧供深度,干净帧供 VIO/靶标')
    ap.add_argument('--overlay-role', choices=['rgbd', 'lidar'],
                    help='双源几何叠加中的后端角色；单源查看器不应设置')
    ap.add_argument('--extrinsic-draft',
                    help='显式启用的 DRAFT 外参 JSON；省略时只读取 registry '
                         '验证通过的标准结果路径')
    ap.add_argument('--extrinsic',
                    help='本地 operational 外参 JSON；仅校验 SE(3)、方向、'
                         '坐标系与设备身份，不依赖本仓库验收注册表')
    ap.add_argument('--timesync',
                    help='显式指定 operational 雷视时移 JSON；仅用于 '
                         '--overlay-role lidar。省略时自动读取当前 rig 结果')
    ap.add_argument('--no-deskew', action='store_true',
                    help='显式关闭 Livox 基于 offset_time + IMU 的旋转 deskew')
    args = ap.parse_args()
    if args.source_timeout <= 0:
        ap.error('--source-timeout must be > 0')
    if args.stream_timeout < 0:
        ap.error('--stream-timeout must be >= 0')
    if args.extrinsic_draft and args.extrinsic:
        ap.error('choose only one of --extrinsic and --extrinsic-draft')
    if (args.extrinsic_draft or args.extrinsic) and not args.overlay_role:
        ap.error('--extrinsic/--extrinsic-draft is only valid with --overlay-role')
    if args.timesync and args.overlay_role != 'lidar':
        ap.error('--timesync is only valid with --overlay-role lidar')
    if args.no_deskew and args.source != 'ros2':
        ap.error('--no-deskew is only valid with --source ros2')
    if args.list_topics:
        from sources.ros2_points import discover_point_topics
        topics = discover_point_topics(timeout=args.topic_timeout)
        if topics:
            for name, msg_type in topics:
                print(f'{name}\t{msg_type}')
        else:
            print('未发现 PointCloud2 或 Livox CustomMsg 点云话题')
        return
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print('\n退出')
    except (RuntimeError, ValueError) as e:
        print(f'\n错误: {e}', file=sys.stderr)
        raise SystemExit(2)


if __name__ == '__main__':
    main()
