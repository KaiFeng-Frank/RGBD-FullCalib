#!/usr/bin/env python3
"""收集已发布的标定产物,汇总成一份给界面消费的 JSON。

缺失或不可用的已发布产物只进入内部完整性记录,不投影为未来计划。
"""
import json
import os
import re
import sys
import time

# 判决行不再手写:全部来自规则引擎(verdicts/rules_d435i.yaml)。
# GUI 卡片、CLI 报告、REPORT.md 同一事实源 —— 改判决=改规则文件,不改代码。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from .lidar_calib import (ACTIVE_TASK_IDS, collect_lidar,
                              lidar_references)
except ImportError:  # server.py is also supported as a direct script in viewer/
    from lidar_calib import ACTIVE_TASK_IDS, collect_lidar, lidar_references

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_RULES = os.path.join(ROOT, 'verdicts', 'rules_d435i.yaml')


def _current_verdicts():
    """Evaluate on every collect so a new result appears without a restart."""
    try:
        from verdicts.engine import evaluate, for_gui
        return for_gui(evaluate(_RULES))
    except Exception as exc:
        print(f'[calib_summary] 判决引擎不可用: {exc}')
        return {}


def _read(p):
    fp = os.path.join(ROOT, p)
    if not os.path.exists(fp):
        return None
    with open(fp, encoding='utf-8') as stream:
        return stream.read()


def _yaml_block(txt, cam):
    if not txt or f'{cam}:' not in txt:
        return None
    seg = txt.split(f'{cam}:')[1]
    for nxt in ('cam0:', 'cam1:', 'cam2:'):
        if nxt in seg and nxt != f'{cam}:':
            seg = seg.split(nxt)[0]
    return seg


def _nums(block, key):
    if not block:
        return None
    m = re.search(rf'{key}:\s*\[([^\]]+)\]', block)
    return [float(x) for x in m.group(1).split(',')] if m else None


def _T(block):
    if not block or 'T_cn_cnm1:' not in block:
        return None
    rows = re.findall(r'-\s*\[([^\]]+)\]', block.split('T_cn_cnm1:')[1])[:4]
    return [[float(x) for x in r.split(',')] for r in rows] if len(rows) == 4 else None


# 每个标定量族的 SLAM 视角:能否在线标定 + 对 SLAM 的影响定级。
# 定量依据 = IMPACT_ANALYSIS.md 的实测传播 + 本机对照实验;
# "公认"限于主流 VIO/SLAM 的标准实践(VINS-Mono/ORB-SLAM3 类:
# 外参旋转/时移/IMU 零偏在线估计是标配,内参与深度链离线)。
SLAM_NOTES = {
    'rgb': dict(
        online='offline', online_note='仅离线 — 主流 VIO 假设内参固定;在线自标定属研究方向,非公认实践',
        impact='high',
        impact_note='所有视觉约束的底座:焦距误差 1% ≈ 三角化深度误差 1%;'
                    '本机出厂零畸变模型在视野边缘造成 10 px 量级位移(实测)'),
    'ir': dict(
        online='offline', online_note='仅离线 — 深度由 ASIC 用烧录参数计算,运行时不可改',
        impact='high',
        impact_note='深度尺度之源:δz/z = δb/b;基线偏 0.16% = 4 m 处 6.4 mm,直接进建图全局尺度'),
    'depth': dict(
        online='partial', online_note='部分 — 噪声权重可在线自适应;pixel-locking 校正表与多径门控只能离线定',
        impact='split',
        impact_note='随机噪声:中(权重错只降精度,σ 4.1 mm@1m vs 65 mm@4m,不加权则远点主导)· '
                    '多径:高(建图,+25 mm/22 cm 结构化,融合平均不掉)· '
                    'pixel-locking:低-中(0.23%@1m→0.93%@4m,确定性可查表校正)'),
    'camimu': dict(
        online='online', online_note='可在线(公认)— 旋转与时移是 VINS-Mono/ORB-SLAM3 的标准在线状态;'
                    '本机判决:旋转/时移可冻结,平移必须在线(杠杆臂 2~3 cm 不可观,三解散布 26%)',
        impact='high',
        impact_note='旋转偏 1° → 重力泄漏 0.17 m/s² 进加速度计;时移跨会话 0.46 ms 在 ω=2 rad/s 下 ≈ 0.8 px'),
    'rgbir': dict(
        online='offline', online_note='仅离线(光度一致性精修属研究级,非标配)',
        impact='mid',
        impact_note='只影响跨模态对齐(上色/语义投影),px 级,不进位姿链;本机与出厂差 0.023 mm,非问题'),
    'imuintr': dict(
        online='partial', online_note='零偏在线是所有 VIO 标配;标度/非正交不在线(可观测性差,需专门激励)',
        impact='low',
        impact_note='对照实验实测:校正后 cam-IMU 残差几乎不动(动态下被在线零偏状态吸收);'
                    '收益在静置初始化:重力方向偏差 0.5°'),
    'thermal': dict(
        online='partial', online_note='分通道 — IMU 零偏温漂被 VIO 在线零偏估计天然吸收(公认);'
                    '深度尺度温漂无对应在线状态,只能离线建模挂 ASIC 温度',
        impact='split',
        impact_note='深度尺度:高(冷热 −0.48% = 4 m 处 19 mm,单次建图即双层墙;图内定位无状态可吸收)· '
                    'IMU 零偏:低(在线吸收)· 焦距:低(视野边缘 1.6 px)'),
}


def collect():
    st = []
    verdicts = _current_verdicts()

    # --- 阶段 1: RGB 内参 ---
    t = _read('data/cam_rgb-camchain.yaml')
    b = _yaml_block(t, 'cam0')
    i = _nums(b, 'intrinsics'); dd = _nums(b, 'distortion_coeffs')
    st.append(dict(
        id='rgb', name='RGB 内参与畸变', stage='阶段 1',
        status='done' if i else 'pending',
        source='data/cam_rgb-camchain.yaml',
        rows=([['fx / fy', f'{i[0]:.3f} / {i[1]:.3f}'],
               ['cx / cy', f'{i[2]:.3f} / {i[3]:.3f}'],
               ['k1 / k2', f'{dd[0]:+.5f} / {dd[1]:+.5f}'],
               ['p1 / p2', f'{dd[2]:+.5f} / {dd[3]:+.5f}'],
               ['分辨率', '1280 × 720']] if i else []),
        note='出厂畸变系数全为 0(未提供),边缘实测有 10 px 量级位移;'
             'cx 与出厂值吻合到 0.13 px,是标定可信度的直接证据。',
        checks=verdicts.get('rgb', []), device='D435i', scope='sensor'))

    # --- 阶段 2: IR 双目 ---
    t = _read('data/cam_ir-camchain.yaml')
    b0 = _yaml_block(t, 'cam0'); b1 = _yaml_block(t, 'cam1')
    i0 = _nums(b0, 'intrinsics'); T = _T(b1)
    base = (sum(x * x for x in [T[0][3], T[1][3], T[2][3]]) ** 0.5 * 1000) if T else None
    stereo_validation_text = _read('results/stereo_rectification_validation.json')
    stereo_validation = json.loads(stereo_validation_text) if stereo_validation_text else None
    ir_rows = ([['fx / fy', f'{i0[0]:.3f} / {i0[1]:.3f}'],
                ['cx / cy', f'{i0[2]:.3f} / {i0[3]:.3f}'],
                ['立体基线', f'{base:.3f} mm'],
                ['分辨率', '1280 × 720']] if i0 else [])
    stereo_note = ''
    if stereo_validation:
        validation = stereo_validation['validation']
        support = stereo_validation['support']
        metrics = stereo_validation['metrics']
        absolute = metrics['abs_vertical_px']
        status_label = {
            'passed': 'PASS', 'failed': 'FAIL', 'insufficient': 'INSUFFICIENT',
        }.get(validation['status'], validation['status'].upper())
        px = lambda value: '—' if value is None else f'{value:.3f}'
        tail = metrics['fraction_abs_vertical_gt_2px']
        ir_rows.extend([
            ['原样 IR 硬件校正验收', f'{status_label}（当前 RGB-D 非阻塞）'],
            ['独立支持',
             f"{support['pairs_with_common_tags']}/{support['pairs_total']} 对 · "
            f"{support['matched_corners']} corners · "
             f"{support['coverage']['covered_cell_count']}/9 格"],
            ['|Δy| median / P95 / P99',
             f"{px(absolute['median'])} / {px(absolute['p95'])} / {px(absolute['p99'])} px"],
            ['|Δy| > 2 px', '—' if tail is None else f'{tail * 100:.2f}%'],
        ])
        if validation['status'] == 'insufficient':
            stereo_note = (f"独立验收只有 {support['matched_corners']} 个共同角点"
                           f"(<600 冻结下限)，结论只能是 INSUFFICIENT；"
                           '已观测尾部残差仍原样保留，未删帧、未改阈值。')
    st.append(dict(
        id='ir', name='IR 双目内参与基线', stage='阶段 2',
        status='done' if i0 else 'pending',
        source=('data/cam_ir-camchain.yaml + results/stereo_rectification_validation.json'
                if stereo_validation else 'data/cam_ir-camchain.yaml'),
        rows=ir_rows,
        note='IR 经 ASIC 硬件去畸变,k1 仅 −0.011(RGB 是 +0.110);'
             'fx≠fy 说明出厂给的"严格相等"是 rectification 的产物,非物理真实。'
             + stereo_note,
        checks=verdicts.get('ir', []), device='D435i', scope='sensor'))

    # --- 阶段 3: 深度质量 ---
    j = _read('results/depth_check.json')
    d3 = json.loads(j) if j else None
    fit = d3['rounds'][1]['fit'] if d3 else None
    st.append(dict(
        id='depth', name='深度噪声模型', stage='阶段 3',
        status='done' if d3 else 'pending',
        source='results/depth_check.json',
        rows=([['视差噪声', f"{fit['disparity_noise_px']:.4f} px"],
               ['目标不平整度', f"{fit['target_flatness_mm']:.2f} mm"],
               ['模型', 'σ² = (a·z²)² + flatness²'],
               ['1 m / 2 m / 4 m', ' / '.join(
                   f"{d3['extrapolation_mm'][k]:.1f}" for k in ('1.0', '2.0', '4.0')) + ' mm']]
              if d3 else []),
        note='误差随 z² 增长,不是线性。0.45 m 以内测的主要是目标本身有多平。'
             '深度噪声在空间上相关(10~20 px 斑块),小邻域平均无法降噪。',
        checks=verdicts.get('depth', []), device='D435i', scope='sensor'))

    # --- 阶段 4: cam-IMU ---
    t = _read('data/camimu-camchain-imucam.yaml')
    b = _yaml_block(t, 'cam0')
    T = _T(b) if b else None
    ts = re.search(r'timeshift_cam_imu:\s*([\d.eE+-]+)', b).group(1) if b and 'timeshift' in b else None
    tr = None
    if b and 'T_cam_imu:' in b:
        rows = re.findall(r'-\s*\[([^\]]+)\]', b.split('T_cam_imu:')[1])[:4]
        if len(rows) == 4:
            Tm = [[float(x) for x in r.split(',')] for r in rows]
            tr = [Tm[k][3] * 1000 for k in range(3)]
    st.append(dict(
        id='camimu', name='cam–IMU 外参与时间偏移', stage='阶段 4',
        status='done' if tr else 'pending',
        source='data/camimu-camchain-imucam.yaml',
        rows=([['平移 (imu→cam)', f'[{tr[0]:+.2f}, {tr[1]:+.2f}, {tr[2]:+.2f}] mm'],
               ['时间偏移', f'{float(ts)*1000:.4f} ms' if ts else '—'],
               ['IMU 噪声参数', 'results/allan_imu.yaml(3 h Allan)']] if tr else []),
        note='旋转与时间偏移可冻结使用;平移不可信 —— 三次独立解散布在 24.6~31.7 mm'
             '(该量本身仅 26 mm),因 IMU 距光心仅 2~3 cm、杠杆臂过短。'
             '已做对照实验排除"IMU 内参未标"这一解释:用六面静置法标出的 T·K·b 校正加速度'
             '后重跑,accel 归一化残差仅从 6.42 降到 5.96(7%),平移依旧不收敛 —— '
             '残差与散布的主因不在加速度计标度/非正交,而在杠杆臂信噪比与工况振动。'
             '结论:平移交给 VIO 在线估计,不要冻结这个数。',
        checks=verdicts.get('camimu', []), device='D435i', scope='cross-sensor'))

    # --- RGB-IR 外参 ---
    t = _read('data/cam_trio-camchain.yaml')
    b1 = _yaml_block(t, 'cam1'); b2 = _yaml_block(t, 'cam2')
    T10 = _T(b1); T21 = _T(b2)
    tr2 = None
    if T10 and T21:
        import numpy as np
        M = np.array(T21) @ np.array(T10)
        tr2 = [M[k][3] * 1000 for k in range(3)]
    st.append(dict(
        id='rgbir', name='RGB ↔ IR 外参', stage='三目标定',
        status='done' if tr2 else 'pending',
        source='data/cam_trio-camchain.yaml',
        rows=([['IR左 → RGB 平移', f'[{tr2[0]:+.3f}, {tr2[1]:+.3f}, {tr2[2]:+.3f}] mm'],
               ['与出厂 |t| 差', '0.023 mm'],
               ['与出厂旋转差', '0.287°']] if tr2 else []),
        note='必须在板子静止时采集:RGB 与 IR 是独立 sensor、无硬件同步,'
             '运动中时间戳差 44 ms,错位会被优化器塞进外参(曾导致 y 差 3 mm、z 差 7.8 mm)。'
             '静止后偏差降到 0.1 ms。',
        checks=verdicts.get('rgbir', []), device='D435i', scope='cross-sensor'))

    # --- IMU 内参 ---
    j = _read('results/imu_intrinsic.json')
    ii = json.loads(j) if j else None
    st.append(dict(
        id='imuintr', name='加速度计内参', stage='IMU 标定',
        status='done' if ii else 'pending',
        source='results/imu_intrinsic.json',
        rows=([['标度因子', ' / '.join(f'{v:.6f}' for v in ii['scale'])],
               ['非正交角', ' / '.join(f'{v*57.2958:+.3f}°' for v in ii['misalign_rad'])],
               ['零偏', ' / '.join(f'{v:+.4f}' for v in ii['bias_ms2']) + ' m/s²'],
               ['当地重力', f"{ii['gravity_local']:.5f} m/s²"],
               ['姿态数', str(ii['n_poses'])]] if ii else []),
        note='Kalibr 的 IMU 模型假设标度因子为 1、三轴正交,该假设在本机不成立。'
             '主要误差是 z–x 轴非正交 2.83°,它在固定姿态下伪装成 0.7% 的标度偏差 —— '
             '单姿态数据无法分离 bias/scale/非正交。',
        checks=verdicts.get('imuintr', []), device='D435i IMU', scope='sensor'))

    # --- 温漂 ---
    j = _read('results/thermal_model.json')
    tm = json.loads(j) if j else None
    st.append(dict(
        id='thermal', name='温漂模型', stage='误差建模',
        status='done' if tm else 'pending',
        source='results/thermal_model.json',
        rows=([['基准温度', f"{tm['T_ref_C']:.0f} °C(范围 {tm['T_range_C'][0]:.0f}~{tm['T_range_C'][1]:.0f})"],
               ['深度尺度', f"{tm['depth_ppm_per_C']:+.1f} ppm/°C  (R²={tm['depth_r2']:.2f})"],
               ['陀螺零偏 x', f"{tm['gyro_bias_deg_s_per_C']['x']:+.5f} °/s/°C  (R²={tm['gyro_bias_r2']['x']:.2f})"],
               ['加速度零偏 z', f"{tm['accel_bias_ms2_per_C']['z']:+.6f} m/s²/°C  (R²={tm['accel_bias_r2']['z']:.2f})"],
               ['主点漂移', '不显著(R²<0.1)—— 可从误差清单划掉']] if tm else []),
        note='Allan 方差要求恒温(测随机噪声),温漂要求升温(测趋势),两者要求相反。'
             '加速度计温漂是陀螺的 14 倍(13°C 跨度分别相当于 Allan 1 秒噪声的 79 倍和 5.3 倍)。'
             '深度温漂 −372 ppm/°C 是铝合金线胀系数的 16 倍,主因不是机械膨胀。',
        checks=verdicts.get('thermal', []), device='D435i', scope='sensor'))

    # 供其他设备或不同部署条件选用的方法参考。这些条目不是
    # 当前 rig 的未完成任务，也不进入 pending/rework 或完成数。
    references = [
        dict(
            id='d435i_reflectivity', device='RGB-D',
            name='材质反射率受控检查',
            applies='新 RGB-D 设备、特殊材质环境，或需要制定无效深度门限时',
            why='镜面、黑色与半透明材质会改变散斑有效性；受控并排采集可区分材质效应与场景几何混杂。',
            how='同距离摆放黑/白/镜面/半透明样品，用 tools/reflectivity_validity.py 分析。',
            output='材质 × 距离的有效率、偏差与门控建议',
            cost='参考工作量：约 30 分钟',
            slam=dict(
                online='offline',
                online_note='传感器特性需离线建模；运行时可按 IR 亮度门控。',
                impact='mid',
                impact_note='无效像素可门控；镜面虚像属于假几何风险，应按目标材质实测。')),
        dict(
            id='d435i_timeshift_thermal', device='RGB-D + IMU',
            name='时间偏移温度稳定性',
            applies='冻结时间偏移且需要跨温度运行的视觉惯性设备',
            why='在线 td 可吸收常数时移；不含 td 状态的冻结部署可参考温度稳定性建模。',
            how='在不同 ASIC 温度下重复 cam–IMU 标定，拟合 t_shift(T) 并留出复测。',
            output='t_shift(T) 拟合及适用温区',
            cost='参考工作量：数小时',
            slam=dict(
                online='online',
                online_note='td 在线估计是 VINS 类常用状态；离线温度模型面向冻结部署。',
                impact='low',
                impact_note='0.46 ms 在 ω=2 rad/s 下约为 0.8 px，应结合目标运动工况判断。')),
        dict(
            id='d435i_gyro_scale', device='IMU',
            name='陀螺仪标度因子',
            applies='含长时间视觉失效窗、纯惯导段，或具备已知角速度基准时',
            why='陀螺仪 scale 需要已知旋转激励，不能由一般静态姿态数据分离。',
            how='使用转台，或以经核查的视觉旋转作为参考，并用独立旋转复测。',
            output='三轴 gyro scale 与独立旋转复测',
            cost='参考条件：转台或视觉角度基准',
            slam=dict(
                online='offline',
                online_note='标度与真实旋转强耦合，需已知激励离线求解。',
                impact='low',
                impact_note='scale 偏 0.5% 会使 90° 纯积分旋转累积约 0.45° 偏差。')),
        dict(
            id='d435i_rolling_shutter', device='RGB 相机',
            name='卷帘快门 line delay',
            applies='以卷帘快门 RGB 做 VIO，或存在快速运动与逐行曝光剪切时',
            why='line delay 会把快速运动映射为逐行特征位移；全局快门追踪链不需要此项。',
            how='使用转台或闪光灯估计行延迟，并用快速运动图像复测。',
            output='line delay 及快速运动复测',
            cost='参考条件：转台或闪光灯',
            slam=dict(
                online='offline',
                online_note='RS-aware VIO 存在研究性在线估计，工程上通常离线处理。',
                impact='low',
                impact_note='使用全局快门 IR 追踪时不进入位姿链；使用 RGB 追踪时按运动速度评估。')),
    ]
    references.extend(lidar_references())

    for stage in st:
        if stage['id'] in SLAM_NOTES:
            stage['slam'] = SLAM_NOTES[stage['id']]

    stages = []
    integrity = []
    for stage in st:
        if stage['status'] == 'done':
            states = {c[2] for c in stage.get('checks', [])}
            stage['quality'] = ('bad' if 'bad' in states else
                                'warn' if 'warn' in states else 'ok')
            stages.append(stage)
        else:
            integrity.append(dict(
                id=stage['id'], source=stage['source'], status='missing',
                detail='已发布标定产物缺失'))

    lidar_stages, lidar_pending = collect_lidar(ROOT)
    lidar_stages = [x for x in lidar_stages if x['id'] in ACTIVE_TASK_IDS]
    lidar_pending = [x for x in lidar_pending if x['id'] in ACTIVE_TASK_IDS]
    stages.extend(lidar_stages)
    integrity.extend(dict(
        id=item['id'], source=item.get('output', ''),
        status='invalid' if item.get('lifecycle') == 'rework' else 'missing',
        detail=item.get('progress', '已发布标定产物不可用'))
        for item in lidar_pending)

    stage_ids = {x['id'] for x in stages}
    integrity_ids = {x['id'] for x in integrity}
    if (len(stage_ids) != len(stages) or
            len(integrity_ids) != len(integrity) or stage_ids & integrity_ids):
        raise AssertionError('calibration result/integrity projections overlap or duplicate')
    counts = dict(done=len(stages), pending=0, rework=0, total=len(stages))
    return dict(stages=stages, references=references, pending=[], counts=counts,
                integrity=integrity, generated_at=time.time())


if __name__ == '__main__':
    print(json.dumps(collect(), ensure_ascii=False, indent=2)[:1500])
