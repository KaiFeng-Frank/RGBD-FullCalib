#!/usr/bin/env python3
"""温漂采集:从冷启动录到热平衡,同步记录 IMU、深度平面距离与内部温度。

与 Allan 方差采集的关键区别:
  Allan  要求恒温 —— 测的是去趋势后的随机噪声
  温漂   要求升温 —— 测的正是那条趋势本身
所以必须冷启动(相机断电静置 ≥30 min 降到接近室温),否则一上来就在热平衡上,
温度不变,自变量没有变化范围,什么也拟合不出来。

存储策略:IMU 原始全存(零偏要靠滑窗均值算),深度只存每个采样点的平面拟合结果
(原始深度图 720 帧就要 586MB,而我们只需要"到平面的距离"这一个标量)。

用法:
  python record_thermal.py -o data/thermal.npz -t 10800 --interval 10
"""
import argparse
import os
import queue
import sys
import time

import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import importlib.util
_spec = importlib.util.spec_from_file_location(
    'cd_', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'check_depth.py'))
_cd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_cd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('-t', '--seconds', type=float, default=10800)
    ap.add_argument('--interval', type=float, default=10.0, help='深度/温度采样间隔(秒)')
    ap.add_argument('--width', type=int, default=848)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--depth-fps', type=int, default=6, help='深度帧率;越低发热越少')
    ap.add_argument('--roi', type=float, default=0.5)
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--alt-emitter', action='store_true',
                    help='发射器交替帧(推荐):散斑帧供深度、干净帧供靶标,'
                         '不必再分时手动开关,温度曲线也更平滑')
    ap.add_argument('--target', action='store_true',
                    help='视野里有 AprilGrid 时开启:同步记录角点像素坐标,'
                         '板与相机均不动则角点应恒定,其漂移即主点/焦距温漂')
    args = ap.parse_args()

    if os.path.exists(args.out) and not args.force:
        print(f"{args.out} 已存在,加 --force 覆盖"); sys.exit(1)

    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        print('没有检测到相机'); sys.exit(1)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.depth_fps)
    if args.target:
        # IR1 与 depth 同坐标系,直接用它看靶标,不必再开彩色流(少一路发热)
        cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height,
                          rs.format.y8, args.depth_fps)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    q = queue.Queue(maxsize=16384)
    pipe = rs.pipeline()
    prof = pipe.start(cfg, lambda f: q.put(f))
    dev = prof.get_device()
    dsens = dev.first_depth_sensor()
    for s in dev.query_sensors():
        if s.supports(rs.option.global_time_enabled):
            s.set_option(rs.option.global_time_enabled, 1)
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 1)   # 深度需要发射器;它也是主要热源之一
        if args.alt_emitter and s.supports(rs.option.emitter_on_off):
            s.set_option(rs.option.emitter_on_off, 1)
            print('[emitter] 交替帧模式:散斑帧供深度,干净帧供靶标')
    emit_sens = None
    for _s in dev.query_sensors():
        if _s.supports(rs.option.emitter_enabled):
            emit_sens = _s
    scale = dsens.get_depth_scale()
    intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    W, H = intr.width, intr.height
    rw, rh = int(W * args.roi), int(H * args.roi)
    roi = ((W - rw) // 2, (H - rh) // 2, (W + rw) // 2, (H + rh) // 2)

    def temps():
        out = {}
        for o, k in ((rs.option.asic_temperature, 'asic'),
                     (rs.option.projector_temperature, 'proj')):
            try:
                out[k] = float(dsens.get_option(o)) if dsens.supports(o) else np.nan
            except Exception:
                out[k] = np.nan
        return out

    t0 = time.time()
    t_start = temps()
    print(f"depth {W}x{H}@{args.depth_fps}  ROI {roi}  采样间隔 {args.interval}s")
    print(f"起始温度 ASIC {t_start['asic']:.1f}C  Projector {t_start['proj']:.1f}C")
    if t_start['asic'] > 40:
        print("  ⚠ 起始温度偏高 —— 相机可能没有充分冷却,温漂曲线的低温段会缺失")
    print(f"时长 {args.seconds/3600:.1f} 小时。相机必须全程静止不动。\n")

    det = None
    if args.target:
        import cv2
        _s2 = importlib.util.spec_from_file_location(
            'cap_', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capture.py'))
        _cap = importlib.util.module_from_spec(_s2); _s2.loader.exec_module(_cap)
        det = _cap.make_detector()
        print("靶标跟踪已开启:记录 tag 角点像素坐标(板与相机须全程不动)")
    tgt = []              # 每行: t_rel + 36 个 tag 的 (cx, cy),未检出为 nan

    imu_t, imu_g, imu_a = [], [], []
    acc_buf = []          # (t, xyz) 用于把 accel 插值到 gyro 时刻
    pend = []             # 等 accel 追上来的 gyro 样本
    slow = []             # 每个采样点一行
    depth_buf = []
    ir_last = [None]
    next_sample = t0 + args.interval

    try:
        while time.time() - t0 < args.seconds:
            try:
                f = q.get(timeout=1.0)
            except queue.Empty:
                continue
            if f.is_motion_frame():
                mf = f.as_motion_frame()
                t = mf.get_timestamp() / 1000.0
                d = mf.get_motion_data()
                if mf.get_profile().stream_type() == rs.stream.accel:
                    acc_buf.append((t, (d.x, d.y, d.z)))
                    if len(acc_buf) > 512:
                        acc_buf = acc_buf[-256:]
                else:
                    # gyro 时刻常常跑在 accel 缓冲的最新时刻之后(两个流独立到达),
                    # 直接判"超范围就丢"会丢掉绝大多数样本 —— 实测 90s 只剩 14 条。
                    # 正确做法是挂起等待,等 accel 追上来再插值。
                    pend.append((t, (d.x, d.y, d.z)))
                if pend and acc_buf:
                    ts = np.array([x[0] for x in acc_buf])
                    keep = []
                    for tg, g in pend:
                        if tg < ts[0]:
                            continue                      # accel 还没覆盖到,只发生在开头
                        if tg > ts[-1]:
                            keep.append((tg, g)); continue  # accel 还没到,下轮再试
                        i = int(np.searchsorted(ts, tg))
                        t1, a1 = acc_buf[i - 1]; t2, a2 = acc_buf[i]
                        w = 0.0 if t2 == t1 else (tg - t1) / (t2 - t1)
                        a = tuple(a1[k] + w * (a2[k] - a1[k]) for k in range(3))
                        imu_t.append(tg); imu_g.append(g); imu_a.append(a)
                    pend = keep
                continue

            fs = f.as_frameset()
            df = fs.get_depth_frame() if fs else (f.as_depth_frame() if f.is_depth_frame() else None)
            if df:
                depth_buf.append(np.asanyarray(df.get_data()).copy())
                if len(depth_buf) > 12:
                    depth_buf.pop(0)
            if det is not None and fs:
                irf = fs.get_infrared_frame(1)
                if irf:
                    laser = 1
                    try:
                        MD = rs.frame_metadata_value.frame_laser_power_mode
                        if irf.supports_frame_metadata(MD):
                            laser = int(irf.get_frame_metadata(MD))
                    except Exception:
                        pass
                    # 交替模式下只留干净帧;常开模式下留全部(靠后面分时关发射器)
                    if (not args.alt_emitter) or laser == 0:
                        ir_last[0] = np.asanyarray(irf.get_data()).copy()

            now = time.time()
            if now >= next_sample and len(depth_buf) >= 8:
                next_sample = now + args.interval
                st, _, _, _ = _cd.analyze(depth_buf[-8:], intr, scale, roi)
                tp = temps()
                if det is not None and not args.alt_emitter:
                    # 常开模式下的退路:采样点短暂关掉发射器拍一帧干净 IR。
                    # 交替模式已经天然提供干净帧,不用走这条。
                    try:
                        emit_sens.set_option(rs.option.emitter_enabled, 0)
                        t_off = time.time()
                        ir_clean = None
                        while time.time() - t_off < 1.2:
                            try:
                                f2 = q.get(timeout=0.5)
                            except queue.Empty:
                                continue
                            if f2.is_motion_frame():
                                continue
                            fs2 = f2.as_frameset()
                            if fs2:
                                i2 = fs2.get_infrared_frame(1)
                                if i2:
                                    ir_clean = np.asanyarray(i2.get_data()).copy()
                    finally:
                        emit_sens.set_option(rs.option.emitter_enabled, 1)
                    ir_last[0] = ir_clean if ir_clean is not None else ir_last[0]
                if det is not None and ir_last[0] is not None:
                    # 逐 tag 记录中心,而不是所有角点的全局均值:
                    # 每次检测到的 tag 子集会变(22/23/24个),全局均值会因此跳几十像素,
                    # 把亚像素级的主点漂移完全淹没。分析时只取全程都在的 tag。
                    c, ids, _ = det.detectMarkers(ir_last[0])
                    row = np.full(1 + 36 * 2, np.nan)
                    row[0] = now - t0
                    n_seen = 0
                    if ids is not None:
                        for cs, tid in zip(c, ids.flatten()):
                            tid = int(tid)
                            if 0 <= tid < 36:
                                m = cs.reshape(4, 2).mean(axis=0)
                                row[1 + tid * 2] = m[0]
                                row[2 + tid * 2] = m[1]
                                n_seen += 1
                    tgt.append(row)
                slow.append((now - t0, tp['asic'], tp['proj'],
                             st.get('dist', np.nan), st.get('plane_rms', np.nan),
                             st.get('valid_ratio', np.nan), st.get('tilt_deg', np.nan)))
                el = now - t0
                print(f"\r  {el/60:6.1f}/{args.seconds/60:.0f} min   "
                      f"ASIC {tp['asic']:5.1f}C  Proj {tp['proj']:5.1f}C   "
                      f"平面 {st.get('dist', float('nan')):.4f}m  "
                      f"RMS {st.get('plane_rms', float('nan'))*1000:5.2f}mm   "
                      f"tag {int(np.isfinite(tgt[-1][1::2]).sum()) if tgt else 0}/36   "
                      f"IMU {len(imu_t)}   ", end='', flush=True)
    except KeyboardInterrupt:
        print("\n[中断] 收尾中...")
    finally:
        pipe.stop()

    slow = np.array(slow, dtype=np.float64)
    np.savez_compressed(
        args.out,
        imu_t=np.array(imu_t, np.float64),
        imu_g=np.array(imu_g, np.float32),
        imu_a=np.array(imu_a, np.float32),
        slow=slow,
        slow_cols=np.array(['t_rel', 'asic_C', 'proj_C', 'plane_dist_m',
                            'plane_rms_m', 'valid_ratio', 'tilt_deg']),
        target=np.array(tgt, dtype=np.float64) if tgt else np.zeros((0, 73)),
        target_cols=np.array(['t_rel'] + [f'tag{i}_{a}' for i in range(36) for a in 'xy']),
        depth_scale=scale, roi=np.array(roi),
        intr=np.array([intr.fx, intr.fy, intr.ppx, intr.ppy]),
        resolution=np.array([W, H]),
    )
    print(f"\n\n写入 {args.out}")
    print(f"  IMU {len(imu_t)} 样本   慢采样 {len(slow)} 点"
          + (f"   靶标 {len(tgt)} 点" if tgt else ""))
    if len(slow):
        print(f"  温度范围 ASIC {slow[:,1].min():.1f} -> {slow[:,1].max():.1f} C "
              f"(跨度 {slow[:,1].max()-slow[:,1].min():.1f} C)")
        if slow[:,1].max() - slow[:,1].min() < 8:
            print("  ⚠ 温度跨度不足 8C,温漂拟合会很不可靠 —— 需要更彻底的冷启动")


if __name__ == '__main__':
    main()
