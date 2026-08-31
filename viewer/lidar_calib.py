#!/usr/bin/env python3
"""MID-360S calibration task registry and lifecycle projection.

Results have two explicit lanes.  A provenance-bound ``operational`` artifact
is exposed as LOCAL when its device and mount identities match the current rig;
an independent acceptance block promotes the same task to VALIDATED.  Both are
results, while missing or malformed artifacts remain planning items
(``pending`` or ``rework``).

Required VALIDATED result envelope (JSON)::

    {
      "schema_version": 1,
      "task_id": "mid360s_d435i_ext",
      "status": "validated",
      "devices": [{"role": "lidar", "model": "Livox Mid-360S", "serial": "..."}],
      "rig_id": "mid360s-d435i-01",
      "created_utc": "2026-08-31T...Z",
      "method": "...",
      "source_data": [{"role": "validation", "path": "...", "sha256": "..."}],
      "frame_convention": {"from": "...", "to": "..."},
      "result": {"...": "..."},
      "summary": {"界面行名": "界面显示值"},
      "validation": {
        "status": "passed",
        "checks": [{"id": "registered_gate_id", "value": 1.0,
                    "status": "passed", "detail": "..."}]
      }
    }

``summary`` is optional; when absent, scalar values from ``result`` are shown.
The provenance and frame fields are deliberately mandatory so a result from a
different unit/rig, or an ambiguous naked 4x4 matrix, cannot silently graduate.
"""
from __future__ import annotations

import json
import hashlib
import math
import os
import re
from datetime import datetime


def _slam(online, online_note, impact, impact_note):
    return dict(online=online, online_note=online_note,
                impact=impact, impact_note=impact_note)


TASKS = [
    dict(
        id='mid360s_health', device='MID-360S', scope='sensor',
        name='MID-360S 测距与点云健康验收', stage='LiDAR · 本体健康',
        result_file='results/mid360s_health.json', cost='A/B 两轮 + 冷热稳态',
        why='厂家已标定束角与测距链；用户侧只验收本机在多距离、材质、入射角和核心温度下是否健康，不重解束角或距离修正模型。',
        how='冻结 10-cell 逐因素协议后采完全独立的 A/B 两轮；外部真值平面算逐射线误差，固定时空角格算覆盖率，并把真值与 bootstrap 不确定度计入上界。',
        slam=_slam('offline', '仅离线健康验收；运行时可按有效率/残差门控坏帧。',
                   'high', '测距系统偏置会直接烘进地图；随机噪声可平均，跨材质/角度偏置不能。')),
    dict(
        id='mid360s_imu', device='MID-360S IMU', scope='sensor',
        name='MID-360S IMU 单位、噪声与内参', stage='LiDAR · IMU',
        result_file='results/mid360s_imu.json', cost='2–3 小时静置 + 多姿态',
        why='MID IMU 数值不能移植 D435i；当前 Driver2 还把 g 单位原样写进 ROS linear_acceleration，必须先固定 SI 转换。',
        how='审计 g→m/s²、做 ≥2 h Allan、12–18 姿态 accel bias/scale/非正交、静止 gyro bias，并在独立姿态复测。',
        slam=_slam('partial', '零偏可在线；噪声密度、标度和非正交应离线标定并保持与部署前处理一致。',
                   'high', '单位错会让重力尺度错约 9.8 倍；噪声和零偏决定初始化及视觉失效窗稳定性。')),
    dict(
        id='mid360s_lidar_imu', device='MID-360S', scope='cross-sensor',
        name='LiDAR–内置 IMU 外参、时移与 deskew', stage='LiDAR–IMU',
        result_file='results/mid360s_lidar_imu.json', cost='动态 6DoF 采集 + 独立复测',
        why='同为 livox_frame 是驱动约定，不是零杠杆臂证明；逐点运动补偿依赖正确的 R/t、时移和 offset_time。',
        how='以官方机械值为初值，充分激励 roll/pitch/yaw 与平移，联合求 T_lidar_imu 和时移；用留出包比较 deskew 前后墙厚/P95。',
        slam=_slam('online', '部分 LIO 可在线精化旋转/时移；离线可靠初值与独立 deskew 验收仍不可省。',
                   'high', '旋转误差随距离放大；0.5° 在 20 m 约造成 17 cm 涂抹。')),
    dict(
        id='mid360s_d435i_ext', device='MID-360S + D435i', scope='cross-sensor',
        name='LiDAR–D435i 6DoF 外参', stage='雷视外参',
        result_file='results/mid360s_d435i_extrinsic.json',
        cost='5 组 × 10–15 s 静态联合数据 + 多起点一致性',
        why='彩色点云、语义投影和雷视融合都依赖直接的 T_camera_lidar；直接求解可避免多段外参链式传播误差。',
        how='刚性固定两传感器，在富几何/纹理环境采五组静态联合数据，直接做稠密 RGB-D↔LiDAR 配准与多起点一致性检查，固结坐标方向和 device/mount 身份后生成 LOCAL；VALIDATED 层级增加独立验收数据。',
        slam=_slam('partial', '当前 rig 已有离线外参；在线精修可作为运行时增强，并继续使用明确的 from/to 坐标定义。',
                   'high', '旋转 0.1° 已约等于 1.5 px；近场 10 mm 平移也会造成数像素错位。')),
    dict(
        id='mid360s_d435i_td', device='MID-360S + D435i', scope='cross-sensor',
        name='LiDAR–D435i 时间偏移与漂移', stage='雷视时间同步',
        result_file='results/mid360s_d435i_timesync.json', cost='动态采集 + 长记录',
        why='MID 与 D435i 不共享硬件时钟；只减一个常数可能掩盖温度/负载引起的线性漂移。',
        how='记录两侧设备时间、host arrival 与每点 timebase+offset_time；动态估 offset，长记录拟合 t_L=a·t_C+b 并独立复测。',
        slam=_slam('online', '主流融合器可在线估常数 td；若存在漂移则需要时钟仿射模型或更强同步。',
                   'high', '5 ms×2 rad/s 在本机焦距下约 8.8 px，是运动雷视错位的一阶项。')),
    dict(
        id='rig_base', device='MID-360S + D435i', scope='rig',
        name='LiDAR/相机到 base_link 安装外参', stage='Rig · base_link',
        result_file='results/rig_base_extrinsics.json', cost='上机器人时必做',
        why='仅有雷视相对外参仍不能把估计位姿放到机器人控制点；安装杠杆臂会影响导航与运动补偿。',
        how='定义唯一 base_link，记录 T_base_lidar/T_base_camera 的方向和单位，做刚体、闭环及装拆复测。',
        slam=_slam('offline', '安装几何通常离线冻结；在线外参只适合有充分激励且明确建模的系统。',
                   'high', 'frame 方向或杠杆臂错误会把所有正确传感器结果一致地变换到错误机体位姿。')),
    dict(
        id='mid360s_validation', device='MID-360S + D435i', scope='rig',
        name='雷视–IMU 独立留出数据闭环验收', stage='Rig · 最终验收',
        result_file='results/mid360s_validation.json', cost='第二份完全独立数据包',
        why='求解残差只能证明拟合了训练数据；必须用未参与求解的数据确认外参、时间和 deskew 能跨采集复现。',
        how='冻结前述结果，用独立 bag 检查投影边缘、平面 P95、动态双墙、两次解散布和 frame 三角闭环，并保存输入/结果哈希。',
        slam=_slam('offline', '这是参数是否可冻结的离线判决门，不由在线估计器替代。',
                   'high', '没有留出验收，错误外参和错误时移可能在同一优化里互相补偿而静默通过。')),
]

# The viewer exposes this rig's current LiDAR deliverable.  The larger catalog
# remains separately addressable for applications that activate those stages.
ACTIVE_TASK_IDS = ('mid360s_d435i_ext',)

# The canonical result is the independent-acceptance VALIDATED lane.  The
# sibling ``.local.json`` is the active, device/mount-bound LOCAL lane.
LOCAL_EXTRINSIC_RESULT = 'results/mid360s_d435i_extrinsic.local.json'
CURRENT_RIG_MANIFEST = 'data/lidar_camera_extrinsic/capture_session.json'
LOCAL_EXTRINSIC_SCHEMA = 'd435i_calib/lidar_camera_extrinsic_local/v1'
CURRENT_RIG_SCHEMA = 'd435i_calib/lidar_camera_mount_session/v1'
EXTRINSIC_EQUATION = 'p_camera = T_camera_lidar * p_lidar'


def _gate(field, op, limit, label, unit=''):
    return dict(id=field, field=field, op=op, limit=limit,
                label=label, unit=unit)


# These are pre-registered project acceptance gates, not values supplied by a
# result file. Change them only before collecting the corresponding dataset.
SPECS = {
    'mid360s_health': dict(
        roles=('lidar',), frames=('livox_frame', 'range_target_frame'),
        source_roles=('calibration', 'validation'),
        required=dict(
            distance_targets_m='distance_coverage', material_count='integer',
            incidence_bin_count='integer', condition_count='integer',
            range_bias_observed_max_mm='number',
            range_p95_observed_max_mm='number',
            repeat_bias_delta_observed_max_mm='number',
            spatiotemporal_coverage_ratio='ratio'),
        literals=dict(
            metric_definition_version='mid360s_health/v1',
            truth_plane_convention='n dot x = D in livox_frame; beam error = measured range - external ray-plane intersection range',
            coverage_definition='10 usable 1 s windows after pre-roll; fixed 0.5 deg angular grid; occupied by >=1 finite nonzero return within 150 mm of external truth plane',
            primary_return_policy='all finite nonzero returns inside the external truth polygon; tag and reflectivity are reported but never used to hide primary range errors',
            core_temperature_source='SDK core_temp / 100 degC'),
        depends_on=(),
        gates=(
            _gate('range_bias_ucb95_mm', 'le', 20.0, '测距偏差 95% 上界', 'mm'),
            _gate('range_p95_ucb95_mm', 'le', 40.0, '测距 P95 的 95% 上界', 'mm'),
            _gate('spatiotemporal_coverage_ratio', 'ge', 0.90, '时空角格覆盖率'),
            _gate('repeat_bias_delta_ucb95_mm', 'le', 10.0, '独立 A/B 偏差差的 95% 上界', 'mm'),
            _gate('truth_range_u95_max_mm', 'le', 5.0, '外部真值最大 U95', 'mm'),
            _gate('material_count', 'ge', 4, '受控材质数量'),
            _gate('incidence_bin_count', 'ge', 3, '入射角分档数量'),
            _gate('condition_count', 'ge', 10, 'A/B 匹配条件数量'),
            _gate('core_temperature_span_C', 'ge', 10.0, '核心温度跨度', '°C'),
            _gate('frame_drop_ratio', 'le', 0.01, '点云掉帧比例'),
            _gate('status_fault_count', 'le', 0, '设备诊断故障样本数'))),
    'mid360s_imu': dict(
        roles=('lidar',), frames=('mid360s_imu_frame', 'si_body_frame'),
        source_roles=('calibration', 'validation'), depends_on=(),
        required=dict(accel_bias_ms2='vector3', accel_scale='positive_vector3',
                      accel_misalignment_rad='vector3',
                      gyro_noise_density_rad_s_sqrt_hz='positive',
                      accel_noise_density_ms2_sqrt_hz='positive',
                      pose_count='integer', accel_unit_scale_ms2_per_g='positive'),
        literals=dict(accel_input_unit='g', accel_output_unit='m/s^2'),
        derived=(dict(field='accel_unit_scale_error_ms2_per_g',
                      source='accel_unit_scale_ms2_per_g', formula='abs_delta',
                      reference=9.80665),),
        gates=(
            _gate('accel_unit_scale_error_ms2_per_g', 'le', 0.01,
                  'g→m/s² 转换误差', 'm/s²/g'),
            _gate('allan_duration_h', 'ge', 2.0, 'Allan 静置时长', 'h'),
            _gate('pose_count', 'ge', 12, '静态姿态数'),
            _gate('static_gravity_error_ms2', 'le', 0.15, '静止重力误差', 'm/s²'),
            _gate('holdout_gravity_error_ms2', 'le', 0.15, '留出姿态重力误差', 'm/s²'),
            _gate('static_gyro_bias_norm_rad_s', 'le', 0.02, '静止陀螺零偏范数', 'rad/s'))),
    'mid360s_lidar_imu': dict(
        roles=('lidar',), frames=('livox_frame', 'mid360s_imu_frame'),
        source_roles=('calibration', 'validation'),
        required=dict(T_imu_lidar='matrix4', deskew_improvement_ratio='ratio',
                      offset_time_valid_ratio='ratio',
                      time_offset_lidar_to_imu_ms='number'),
        literals=dict(time_offset_convention=
                      't_imu = t_lidar + time_offset_lidar_to_imu_ms / 1000'),
        depends_on=('mid360s_health', 'mid360s_imu'),
        gates=(
            _gate('holdout_wall_p95_mm', 'le', 30.0, '留出集 deskew 墙厚 P95', 'mm'),
            _gate('deskew_improvement_ratio', 'ge', 0.20, 'deskew 改善比例'),
            _gate('repeat_rotation_delta_deg', 'le', 0.20, '重复解旋转差', 'deg'),
            _gate('repeat_translation_delta_mm', 'le', 10.0, '重复解平移差', 'mm'),
            _gate('repeat_time_offset_delta_ms', 'le', 0.20, '重复解时移差', 'ms'),
            _gate('offset_time_valid_ratio', 'ge', 0.99, '逐点时间有效比例'))),
    'mid360s_d435i_ext': dict(
        roles=('lidar', 'rgbd'),
        frames=('livox_frame', 'camera_color_optical_frame'),
        source_roles=('calibration', 'validation'),
        required=dict(T_camera_lidar='matrix4', pose_count='integer'),
        # The rigid LiDAR-camera transform is directly observable from static
        # image/point-cloud pairs; range health remains a separate workstream.
        depends_on=(),
        gates=(
            _gate('pose_count', 'ge', 5, '有效静态联合录包数'),
            _gate('holdout_projection_p95_px', 'le', 3.0, '留出集投影误差 P95', 'px'),
            _gate('repeat_rotation_delta_deg', 'le', 0.20, '重复解旋转差', 'deg'),
            _gate('repeat_translation_delta_mm', 'le', 10.0, '重复解平移差', 'mm'))),
    'mid360s_d435i_td': dict(
        roles=('lidar', 'rgbd'), frames=('d435i_clock', 'livox_clock'),
        source_roles=('calibration', 'validation'),
        required=dict(clock_model_a='positive', clock_model_b_ms='number'),
        literals=dict(clock_equation=
                      't_livox = clock_model_a * t_d435i + clock_model_b_ms / 1000'),
        derived=(dict(field='clock_scale_error_ppm_abs', source='clock_model_a',
                      formula='abs_minus_one_ppm'),),
        depends_on=('mid360s_health',),
        gates=(
            _gate('record_duration_min', 'ge', 20.0, '长记录时长', 'min'),
            _gate('holdout_residual_p95_ms', 'le', 1.0, '留出集时间残差 P95', 'ms'),
            _gate('repeat_offset_delta_ms', 'le', 1.0, '重复估计 offset 差', 'ms'),
            _gate('clock_scale_error_ppm_abs', 'le', 200.0, '时钟尺度偏差绝对值', 'ppm'))),
    'rig_base': dict(
        roles=('lidar', 'rgbd'),
        frames=dict(T_base_lidar=('livox_frame', 'base_link'),
                    T_base_camera=('camera_color_optical_frame', 'base_link')),
        source_roles=('calibration', 'validation'),
        required=dict(T_base_lidar='matrix4', T_base_camera='matrix4'),
        depends_on=('mid360s_d435i_ext',),
        gates=(
            _gate('closure_translation_mm', 'le', 5.0, '坐标闭环平移误差', 'mm'),
            _gate('closure_rotation_deg', 'le', 0.20, '坐标闭环旋转误差', 'deg'),
            _gate('remount_translation_mm', 'le', 10.0, '装拆复测平移差', 'mm'),
            _gate('remount_rotation_deg', 'le', 0.50, '装拆复测旋转差', 'deg'))),
    'mid360s_validation': dict(
        roles=('lidar', 'rgbd'),
        frames=('livox_frame', 'camera_color_optical_frame'),
        source_roles=('validation',),
        required=dict(upstream_sha256='hash_map'),
        depends_on=('mid360s_health', 'mid360s_imu', 'mid360s_lidar_imu',
                    'mid360s_d435i_ext', 'mid360s_d435i_td', 'rig_base'),
        gates=(
            _gate('projection_p95_px', 'le', 3.0, '最终投影误差 P95', 'px'),
            _gate('deskew_wall_p95_mm', 'le', 30.0, '最终 deskew 墙厚 P95', 'mm'),
            _gate('clock_residual_p95_ms', 'le', 1.0, '最终时间残差 P95', 'ms'),
            _gate('frame_closure_translation_mm', 'le', 5.0, '最终闭环平移误差', 'mm'),
            _gate('frame_closure_rotation_deg', 'le', 0.20, '最终闭环旋转误差', 'deg'),
            _gate('repeat_extrinsic_rotation_deg', 'le', 0.20, '最终重复外参旋转差', 'deg'),
            _gate('repeat_extrinsic_translation_mm', 'le', 10.0, '最终重复外参平移差', 'mm'))),
}

_MODEL_PATTERNS = {
    'lidar': re.compile(r'(?=.*livox)(?=.*mid[- _]?360s?)', re.I),
    'rgbd': re.compile(r'(?=.*(?:intel|realsense))(?=.*d435i)', re.I),
}

_task_ids = [task['id'] for task in TASKS]
_result_files = [task['result_file'] for task in TASKS]
if (len(_task_ids) != len(set(_task_ids)) or
        len(_result_files) != len(set(_result_files)) or
        set(_task_ids) != set(SPECS)):
    raise RuntimeError('MID-360S calibration registry has duplicate or missing specs')


_PASS = {'ok', 'pass', 'passed'}
_FAIL = {'warn', 'warning', 'bad', 'fail', 'failed', 'error'}
_HASH_RE = re.compile(r'[0-9a-fA-F]{64}')
_OP_SYMBOL = {'le': '≤', 'ge': '≥'}


def _number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _finite_scalars(value):
    """Reject NaN/Inf anywhere in numeric result/summary trees."""
    if isinstance(value, bool) or value is None:
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    if isinstance(value, dict):
        return all(_finite_scalars(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_scalars(v) for v in value)
    return True


def _matrix4(value):
    if (not isinstance(value, list) or len(value) != 4 or
            any(not isinstance(row, list) or len(row) != 4 for row in value) or
            any(not _number(x) for row in value for x in row)):
        return False
    if any(abs(float(value[3][i]) - (1.0 if i == 3 else 0.0)) > 1e-6
           for i in range(4)):
        return False
    r = [[float(value[i][j]) for j in range(3)] for i in range(3)]
    rigid_tol = 1e-5
    for i in range(3):
        if abs(sum(x * x for x in r[i]) - 1.0) > rigid_tol:
            return False
        for j in range(i):
            if abs(sum(r[i][k] * r[j][k] for k in range(3))) > rigid_tol:
                return False
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    return abs(det - 1.0) <= rigid_tol


def _valid_kind(value, kind):
    if kind == 'number':
        return _number(value)
    if kind == 'positive':
        return _number(value) and value > 0
    if kind == 'integer':
        return _number(value) and float(value).is_integer() and value >= 0
    if kind == 'ratio':
        return _number(value) and 0 <= value <= 1
    if kind == 'distance_coverage':
        return (isinstance(value, list) and all(_number(x) and x > 0 for x in value)
                and all(any(abs(float(x) - target) <= 0.05 for x in value)
                        for target in (1.0, 3.0, 5.0, 10.0)))
    if kind in ('vector3', 'positive_vector3'):
        return (isinstance(value, list) and len(value) == 3 and
                all(_number(x) for x in value) and
                (kind == 'vector3' or all(x > 0 for x in value)))
    if kind == 'matrix4':
        return _matrix4(value)
    if kind == 'hash_map':
        return (isinstance(value, dict) and bool(value) and
                all(isinstance(k, str) and _HASH_RE.fullmatch(str(v))
                    for k, v in value.items()))
    raise ValueError(f'unknown result schema kind: {kind}')


def _valid_utc(value):
    if not isinstance(value, str) or not value.endswith('Z'):
        return False
    try:
        parsed = datetime.fromisoformat(value[:-1] + '+00:00')
        return parsed.tzinfo is not None and parsed.utcoffset().total_seconds() == 0
    except (TypeError, ValueError):
        return False


def _derived_value(rule, result):
    source = result.get(rule['source'])
    if not _number(source):
        return None
    if rule['formula'] == 'abs_delta':
        return abs(float(source) - float(rule['reference']))
    if rule['formula'] == 'abs_minus_one_ppm':
        return abs(float(source) - 1.0) * 1e6
    raise ValueError(f"unknown derived formula: {rule['formula']}")


def _gate_pass(gate, value):
    if not _number(value) or value < 0:
        return False
    return value <= gate['limit'] if gate['op'] == 'le' else value >= gate['limit']


def _display(value):
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_display(v) for v in value) + ']'
    if isinstance(value, dict):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def _rows(doc):
    summary = doc.get('summary')
    if isinstance(summary, dict):
        return [[str(k), _display(v)] for k, v in summary.items()]
    if isinstance(summary, list):
        out = []
        for row in summary:
            if isinstance(row, (list, tuple)) and len(row) == 2:
                out.append([str(row[0]), _display(row[1])])
        if out:
            return out
    result = doc.get('result') if isinstance(doc.get('result'), dict) else {}
    return [[str(k), _display(v)] for k, v in result.items()
            if not isinstance(v, (dict, list))][:8]


def _checks(task, doc):
    spec = SPECS[task['id']]
    result = doc.get('result') if isinstance(doc.get('result'), dict) else {}
    validation = doc.get('validation') if isinstance(doc.get('validation'), dict) else {}
    raw_checks = validation.get('checks') if isinstance(validation.get('checks'), list) else []
    by_id = {x.get('id'): x for x in raw_checks if isinstance(x, dict)}
    out = []
    for gate in spec['gates']:
        value = result.get(gate['field'])
        check = by_id.get(gate['id'], {})
        declared = str(check.get('status', '')).lower()
        passed = _gate_pass(gate, value) and declared in _PASS
        unit = f" {gate['unit']}" if gate['unit'] else ''
        detail = (f"实测 {_display(value)}{unit}，完成门 "
                  f"{_OP_SYMBOL[gate['op']]} {_display(gate['limit'])}{unit}")
        if check.get('detail'):
            detail += f"；{check['detail']}"
        out.append([gate['label'], detail, 'ok' if passed else 'bad'])
    return out


def _validate(task, doc):
    """Validate the registered schema and recompute every completion gate."""
    spec = SPECS[task['id']]
    missing, failed = [], []
    if doc.get('schema_version') != 1:
        missing.append('schema_version 必须为 1')
    if doc.get('task_id') != task['id']:
        missing.append(f"task_id 必须为 {task['id']}")
    if str(doc.get('status', '')).lower() != 'validated':
        missing.append('status 尚未标记 validated')

    devices = doc.get('devices')
    if not isinstance(devices, list) or not devices:
        missing.append('devices 必须是非空列表')
        devices = []
    roles = [x.get('role') for x in devices if isinstance(x, dict)]
    if len(roles) != len(devices) or len(roles) != len(set(roles)):
        missing.append('devices 角色缺失或重复')
    if set(roles) != set(spec['roles']):
        missing.append(f"devices 角色必须为 {','.join(spec['roles'])}")
    for i, device in enumerate(devices):
        if not isinstance(device, dict):
            continue
        role = device.get('role')
        model, serial = device.get('model'), device.get('serial')
        if role in _MODEL_PATTERNS and (not isinstance(model, str) or
                                        not _MODEL_PATTERNS[role].search(model)):
            missing.append(f'devices[{i}].model 与 {role} 不匹配')
        if not isinstance(serial, str) or not serial.strip():
            missing.append(f'devices[{i}].serial 缺失')

    if not isinstance(doc.get('rig_id'), str) or not doc['rig_id'].strip():
        missing.append('rig_id 缺失')
    if task['id'] == 'mid360s_d435i_ext' and (
            not isinstance(doc.get('mount_session_id'), str) or
            not doc['mount_session_id'].strip()):
        missing.append('mount_session_id 缺失')
    if not _valid_utc(doc.get('created_utc')):
        missing.append('created_utc 必须是 UTC ISO-8601 时间')
    if not isinstance(doc.get('method'), str) or len(doc['method'].strip()) < 4:
        missing.append('method 缺失或过短')

    sources = doc.get('source_data')
    if not isinstance(sources, list) or not sources:
        missing.append('source_data 必须是非空列表')
        sources = []
    source_roles, seen_sources, hashes_by_role = set(), set(), {}
    for i, src in enumerate(sources):
        if not isinstance(src, dict):
            missing.append(f'source_data[{i}] 必须为 object')
            continue
        role, path, digest = src.get('role'), src.get('path'), str(src.get('sha256', ''))
        if not isinstance(role, str) or not role:
            missing.append(f'source_data[{i}].role 缺失')
        else:
            source_roles.add(role)
        if not isinstance(path, str) or not path:
            missing.append(f'source_data[{i}].path 缺失')
        if not _HASH_RE.fullmatch(digest):
            missing.append(f'source_data[{i}].sha256 必须为 64 位十六进制')
        key = (role, path)
        if key in seen_sources:
            missing.append(f'source_data[{i}] 重复')
        seen_sources.add(key)
        hashes_by_role.setdefault(role, set()).add(digest.lower())
    absent_roles = set(spec['source_roles']) - source_roles
    if absent_roles:
        missing.append('source_data 缺少角色: ' + ','.join(sorted(absent_roles)))
    if (hashes_by_role.get('calibration', set()) &
            hashes_by_role.get('validation', set())):
        missing.append('calibration 与 validation 不得引用同一数据哈希')

    frames = doc.get('frame_convention')
    expected_frames = spec['frames']
    if isinstance(expected_frames, dict):
        valid_frames = isinstance(frames, dict) and set(frames) == set(expected_frames)
        if valid_frames:
            for transform, (source_frame, target_frame) in expected_frames.items():
                item = frames.get(transform)
                if (not isinstance(item, dict) or item.get('from') != source_frame or
                        item.get('to') != target_frame):
                    valid_frames = False
                    break
        if not valid_frames:
            expected_text = '；'.join(
                f'{name}: {pair[0]} → {pair[1]}' for name, pair in expected_frames.items())
            missing.append('frame_convention 必须为 ' + expected_text)
    else:
        expected_from, expected_to = expected_frames
        if (not isinstance(frames, dict) or frames.get('from') != expected_from or
                frames.get('to') != expected_to):
            missing.append(f'frame_convention 必须为 {expected_from} → {expected_to}')

    result = doc.get('result')
    if not isinstance(result, dict) or not result:
        missing.append('result 必须是非空 object')
        result = {}
    elif not _finite_scalars(result):
        missing.append('result 含 NaN/Inf')
    for field, kind in spec['required'].items():
        if not _valid_kind(result.get(field), kind):
            missing.append(f'result.{field} 不符合 {kind}')
    for field, expected in spec.get('literals', {}).items():
        if result.get(field) != expected:
            missing.append(f'result.{field} 必须为 {expected}')
    for rule in spec.get('derived', ()):
        expected = _derived_value(rule, result)
        actual = result.get(rule['field'])
        if (expected is None or not _number(actual) or
                not math.isclose(float(actual), expected, rel_tol=1e-9, abs_tol=1e-9)):
            missing.append(f"result.{rule['field']} 与 {rule['source']} 不一致")
    if 'upstream_sha256' in spec['required']:
        upstream = result.get('upstream_sha256', {})
        if isinstance(upstream, dict) and set(upstream) != set(spec['depends_on']):
            missing.append('result.upstream_sha256 必须覆盖全部上游任务且不能多项')
    if doc.get('summary') is not None and (not isinstance(doc['summary'], (dict, list)) or
                                            not _finite_scalars(doc['summary'])):
        missing.append('summary 必须是有限值组成的 object/list')

    validation = doc.get('validation')
    if not isinstance(validation, dict):
        missing.append('validation 必须为 object')
        validation = {}
    declared_overall = str(validation.get('status', '')).lower()
    if declared_overall not in _PASS | _FAIL:
        missing.append('validation.status 必须为 passed/failed')
    elif declared_overall not in _PASS:
        failed.append('validation.status 未通过')
    checks = validation.get('checks')
    if not isinstance(checks, list):
        missing.append('validation.checks 必须为列表')
        checks = []
    check_ids = [x.get('id') for x in checks if isinstance(x, dict)]
    expected_ids = [x['id'] for x in spec['gates']]
    if len(check_ids) != len(checks) or len(check_ids) != len(set(check_ids)):
        missing.append('validation.checks 含无效项或重复 id')
    if set(check_ids) != set(expected_ids):
        missing.append('validation.checks 必须恰好覆盖预注册 gate id')
    by_id = {x.get('id'): x for x in checks if isinstance(x, dict)}
    for gate in spec['gates']:
        value = result.get(gate['field'])
        if not _number(value):
            missing.append(f"result.{gate['field']} 必须是有限数值")
            continue
        check = by_id.get(gate['id'])
        if not isinstance(check, dict):
            continue
        check_value = check.get('value')
        if (not _number(check_value) or
                not math.isclose(float(check_value), float(value), rel_tol=1e-9, abs_tol=1e-12)):
            missing.append(f"validation.checks[{gate['id']}].value 与 result 不一致")
            continue
        declared = str(check.get('status', '')).lower()
        if declared not in _PASS | _FAIL:
            missing.append(f"validation.checks[{gate['id']}].status 非法")
            continue
        if not _gate_pass(gate, value):
            failed.append(f"{gate['label']} 未过预注册阈值")
        if declared not in _PASS:
            failed.append(f"{gate['label']} 的独立核查未通过")

    if missing:
        return 'pending', missing
    if failed:
        return 'rework', list(dict.fromkeys(failed))
    return 'done', []


def _current_rig(root):
    """Load the capture manifest that identifies the currently mounted rig."""
    path = os.path.join(root, CURRENT_RIG_MANIFEST)
    if not os.path.exists(path):
        return None, [f'当前 rig 清单不存在: {CURRENT_RIG_MANIFEST}']
    try:
        with open(path, encoding='utf-8') as stream:
            doc = json.load(stream)
    except Exception as exc:
        return None, [f'当前 rig 清单不可解析: {type(exc).__name__}: {exc}']
    if not isinstance(doc, dict):
        return None, ['当前 rig 清单顶层必须是 object']
    reasons = []
    if doc.get('schema') != CURRENT_RIG_SCHEMA:
        reasons.append(f'当前 rig 清单 schema 必须为 {CURRENT_RIG_SCHEMA}')
    for field in ('rig_id', 'mount_session_id', 'd435i_serial', 'mid360s_serial'):
        if not isinstance(doc.get(field), str) or not doc[field].strip():
            reasons.append(f'当前 rig 清单 {field} 缺失')
    return (None, reasons) if reasons else (doc, [])


def _validate_operational_extrinsic(task, doc, current_rig):
    """Validate a device/mount-bound LOCAL operational transform.

    This lane checks identity, provenance, frame direction and SE(3).  The
    separate VALIDATED lane adds independent acceptance evidence.
    """
    missing = []
    if task['id'] != 'mid360s_d435i_ext':
        return 'pending', ['LOCAL operational 只适用于雷视外参任务']
    if doc.get('schema_version') != 1:
        missing.append('schema_version 必须为 1')
    if doc.get('task_id') != task['id']:
        missing.append(f"task_id 必须为 {task['id']}")
    if str(doc.get('status', '')).lower() != 'operational':
        missing.append('LOCAL status 必须为 operational')
    if doc.get('local_schema') != LOCAL_EXTRINSIC_SCHEMA:
        missing.append(f'local_schema 必须为 {LOCAL_EXTRINSIC_SCHEMA}')

    devices = doc.get('devices')
    if not isinstance(devices, list) or len(devices) != 2:
        missing.append('devices 必须恰好包含 lidar 与 rgbd')
        devices = []
    by_role = {}
    for i, device in enumerate(devices):
        if not isinstance(device, dict):
            missing.append(f'devices[{i}] 必须为 object')
            continue
        role = device.get('role')
        if role not in ('lidar', 'rgbd') or role in by_role:
            missing.append('devices 角色必须唯一且恰好为 lidar,rgbd')
            continue
        by_role[role] = device
        model, serial = device.get('model'), device.get('serial')
        if (not isinstance(model, str) or
                not _MODEL_PATTERNS[role].search(model)):
            missing.append(f'devices[{i}].model 与 {role} 不匹配')
        if not isinstance(serial, str) or not serial.strip():
            missing.append(f'devices[{i}].serial 缺失')
    if set(by_role) != {'lidar', 'rgbd'}:
        missing.append('devices 角色必须唯一且恰好为 lidar,rgbd')

    for field in ('rig_id', 'mount_session_id'):
        if not isinstance(doc.get(field), str) or not doc[field].strip():
            missing.append(f'{field} 缺失')
    if current_rig:
        expected = {
            'rig_id': current_rig['rig_id'],
            'mount_session_id': current_rig['mount_session_id'],
        }
        for field, value in expected.items():
            if doc.get(field) != value:
                missing.append(f'{field} 与当前 rig 清单不一致')
        serials = {
            'lidar': current_rig['mid360s_serial'],
            'rgbd': current_rig['d435i_serial'],
        }
        for role, serial in serials.items():
            if role in by_role and by_role[role].get('serial') != serial:
                missing.append(f'{role} serial 与当前 rig 清单不一致')

    if not _valid_utc(doc.get('created_utc')):
        missing.append('created_utc 必须是 UTC ISO-8601 时间')
    if not isinstance(doc.get('method'), str) or len(doc['method'].strip()) < 4:
        missing.append('method 缺失或过短')

    frames = doc.get('frame_convention')
    if (not isinstance(frames, dict) or frames.get('from') != 'livox_frame' or
            frames.get('to') != 'camera_color_optical_frame' or
            frames.get('equation') != EXTRINSIC_EQUATION):
        missing.append(
            'frame_convention 必须声明 livox_frame → '
            f'camera_color_optical_frame 且 equation={EXTRINSIC_EQUATION}')

    result = doc.get('result')
    if not isinstance(result, dict) or not result:
        missing.append('result 必须是非空 object')
        result = {}
    elif not _finite_scalars(result):
        missing.append('result 含 NaN/Inf')
    if not _matrix4(result.get('T_camera_lidar')):
        missing.append('result.T_camera_lidar 必须为有限刚体 4x4 变换')
    pose_count = result.get('pose_count')
    if (not _valid_kind(pose_count, 'integer') or pose_count < 1):
        missing.append('result.pose_count 必须为正整数')
    if doc.get('summary') is not None and (
            not isinstance(doc['summary'], (dict, list)) or
            not _finite_scalars(doc['summary'])):
        missing.append('summary 必须是有限值组成的 object/list')

    sources = doc.get('source_data')
    if not isinstance(sources, list) or not sources:
        missing.append('source_data 必须是非空列表')
        sources = []
    seen = set()
    for i, source in enumerate(sources):
        if not isinstance(source, dict):
            missing.append(f'source_data[{i}] 必须为 object')
            continue
        role, path = source.get('role'), source.get('path')
        digest = str(source.get('sha256', ''))
        if not isinstance(role, str) or not role.strip():
            missing.append(f'source_data[{i}].role 缺失')
        if not isinstance(path, str) or not path.strip():
            missing.append(f'source_data[{i}].path 缺失')
        if not _HASH_RE.fullmatch(digest):
            missing.append(f'source_data[{i}].sha256 必须为 64 位十六进制')
        key = (role, path, digest.lower())
        if key in seen:
            missing.append(f'source_data[{i}] 重复')
        seen.add(key)
    return ('pending', list(dict.fromkeys(missing))) if missing else ('done', [])


def _completion_text(task):
    spec = SPECS[task['id']]
    return '；'.join(
        f"{g['label']} {_OP_SYMBOL[g['op']]} {_display(g['limit'])}"
        + (f" {g['unit']}" if g['unit'] else '') for g in spec['gates'])


def _todo(task, lifecycle='pending', progress='尚无结果产物', checks=None):
    return dict(
        id=task['id'], device=task['device'], scope=task['scope'],
        name=task['name'], stage=task['stage'], lifecycle=lifecycle,
        why=task['why'], how=task['how'], cost=task['cost'],
        output=task['result_file'], completion=_completion_text(task),
        progress=progress, checks=checks or [], slam=task['slam'])


def _stage(task, doc, digest, source=None, quality='ok'):
    is_local = quality == 'local'
    if is_local:
        note = str(doc.get('note') or
                   f"LOCAL · 当前 rig operational 外参；五场景稠密配准与多起点一致性已完成；方法: {doc.get('method', '—')}。")
        rows = [['质量', 'LOCAL'], ['状态', 'operational · 当前 rig']] + _rows(doc)
        checks = []
    else:
        note = str(doc.get('note') or
                   f"{task['name']} 已通过预注册阈值与独立核查；方法: {doc.get('method', '—')}。")
        rows = _rows(doc)
        checks = _checks(task, doc)
    return dict(
        id=task['id'], device=task['device'], scope=task['scope'],
        name=task['name'], stage=task['stage'], status='done', quality=quality,
        quality_label='LOCAL' if is_local else 'VALIDATED',
        source=source or task['result_file'], artifact_sha256=digest,
        rows=rows, note=note, checks=checks, slam=task['slam'])


def collect_lidar(root):
    records = {}
    for task in TASKS:
        path = os.path.join(root, task['result_file'])
        rec = dict(task=task, doc=None, digest=None, lifecycle='pending',
                   reasons=['尚无结果产物'], source=task['result_file'],
                   quality='ok')
        records[task['id']] = rec
        if os.path.exists(path):
            try:
                with open(path, 'rb') as f:
                    raw = f.read()
                doc = json.loads(raw.decode('utf-8'))
                if not isinstance(doc, dict):
                    raise ValueError('顶层必须是 JSON object')
                rec['doc'] = doc
                rec['digest'] = hashlib.sha256(raw).hexdigest()
                rec['lifecycle'], rec['reasons'] = _validate(task, doc)
            except Exception as exc:
                rec['reasons'] = [f'产物不可解析: {type(exc).__name__}: {exc}']

        # Canonical VALIDATED output takes precedence.  Otherwise a matching
        # current-rig LOCAL artifact fills the result card as operational.
        if task['id'] == 'mid360s_d435i_ext' and rec['lifecycle'] != 'done':
            local_path = os.path.join(root, LOCAL_EXTRINSIC_RESULT)
            if os.path.exists(local_path):
                try:
                    with open(local_path, 'rb') as f:
                        raw = f.read()
                    doc = json.loads(raw.decode('utf-8'))
                    if not isinstance(doc, dict):
                        raise ValueError('顶层必须是 JSON object')
                    current_rig, rig_reasons = _current_rig(root)
                    lifecycle, reasons = _validate_operational_extrinsic(
                        task, doc, current_rig)
                    reasons = rig_reasons + reasons
                    if not reasons and lifecycle == 'done':
                        rec.update(
                            doc=doc, digest=hashlib.sha256(raw).hexdigest(),
                            lifecycle='done', reasons=[],
                            source=LOCAL_EXTRINSIC_RESULT, quality='local')
                    else:
                        rec['lifecycle'] = 'pending'
                        rec['reasons'] = [
                            'LOCAL 不可用: ' + '；'.join(dict.fromkeys(reasons))]
                except Exception as exc:
                    rec['lifecycle'] = 'pending'
                    rec['reasons'] = [
                        f'LOCAL 产物不可解析: {type(exc).__name__}: {exc}']

    # A result identifies the rig, but does not get to redefine it per task.
    candidates = [x for x in records.values() if x['lifecycle'] == 'done']
    rig_ids = {x['doc']['rig_id'] for x in candidates}
    if len(rig_ids) > 1:
        for rec in candidates:
            rec['lifecycle'] = 'rework'
            rec['reasons'].append('已完成产物的 rig_id 互相冲突')
    for role in _MODEL_PATTERNS:
        serials = set()
        for rec in candidates:
            serials.update(str(x['serial']) for x in rec['doc']['devices']
                           if x.get('role') == role)
        if len(serials) > 1:
            for rec in candidates:
                if any(x.get('role') == role for x in rec['doc']['devices']):
                    rec['lifecycle'] = 'rework'
                    rec['reasons'].append(f'已完成产物的 {role} serial 互相冲突')

    # Dependencies are projected after individual gates and identity checks.
    # Re-collecting after an upstream result appears can promote a dependent.
    changed = True
    while changed:
        changed = False
        for task_id, rec in records.items():
            if rec['lifecycle'] != 'done':
                continue
            blockers = [x for x in SPECS[task_id]['depends_on']
                        if records[x]['lifecycle'] != 'done']
            if blockers:
                rec['lifecycle'] = 'pending'
                rec['reasons'] = ['上游任务未完成: ' + ','.join(blockers)]
                changed = True

    # The final holdout artifact is cryptographically bound to the exact
    # upstream JSON files it validated, so replacing a parameter reopens it.
    final = records['mid360s_validation']
    if final['lifecycle'] == 'done':
        deps = SPECS['mid360s_validation']['depends_on']
        expected = {x: records[x]['digest'] for x in deps}
        supplied = final['doc']['result']['upstream_sha256']
        final_sources = {x['sha256'].lower() for x in final['doc']['source_data']}
        upstream_sources = {
            x['sha256'].lower() for task_id in deps
            for x in records[task_id]['doc']['source_data']
        }
        reasons = []
        if final_sources & upstream_sources:
            reasons.append('最终留出数据与上游求解/验收数据哈希重合')
        if supplied != expected:
            reasons.append('upstream_sha256 与当前上游结果文件不一致')
        if reasons:
            final['lifecycle'] = 'rework'
            final['reasons'] = reasons

    stages, pending = [], []
    for task in TASKS:
        rec = records[task['id']]
        if rec['lifecycle'] == 'done':
            stages.append(_stage(
                task, rec['doc'], rec['digest'], source=rec['source'],
                quality=rec['quality']))
        else:
            pending.append(_todo(
                task, lifecycle=rec['lifecycle'],
                progress='；'.join(rec['reasons']),
                checks=_checks(task, rec['doc']) if rec['doc'] else []))

    projected = [x['id'] for x in stages] + [x['id'] for x in pending]
    if (len(projected) != len(_task_ids) or len(projected) != len(set(projected)) or
            set(projected) != set(_task_ids) or
            {x['id'] for x in stages} & {x['id'] for x in pending}):
        raise AssertionError('LiDAR calibration lifecycle projection is not a partition')
    return stages, pending
