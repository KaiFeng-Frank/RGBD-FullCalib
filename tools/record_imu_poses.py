#!/usr/bin/env python3
"""多姿态静置采集,给加速度计内参标定用。

用法:把相机摆成不同姿态,每个姿态放稳不动,工具自动检测静止并记录一个姿态。
需要至少 9 个姿态(9 个未知数),建议 12~18 个:
  六个面各朝下 6 个 + 若干倾斜姿态。

判静止的依据是陀螺:静置时角速度真值恒为 0,比用加速度判可靠得多
(加速度在不同姿态下本来就不同,没法拿它当"没动"的判据)。
"""
import argparse
import os
import sys
import time

import numpy as np
import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--poses', type=int, default=12, help='要采多少个姿态')
    ap.add_argument('--hold', type=float, default=2.0, help='每个姿态需静止多少秒')
    ap.add_argument('--gyro-thr', type=float, default=4.0,
                    help='静止判据:角速度 deg/s。手持通常 1~3,靠着东西更低;手抖是高频的,窗口内平均即可压掉')
    ap.add_argument('--min-sep', type=float, default=15.0,
                    help='与已记录姿态的最小夹角(度),防止重复采同一个朝向')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    if os.path.exists(args.out) and not args.force:
        print(f"{args.out} 已存在,加 --force"); sys.exit(1)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipe = rs.pipeline(); prof = pipe.start(cfg)
    for s in prof.get_device().query_sensors():
        if s.supports(rs.option.global_time_enabled):
            s.set_option(rs.option.global_time_enabled, 1)

    print(f"目标 {args.poses} 个姿态,每个需静止 {args.hold:.0f} 秒")
    print("把相机摆成不同朝向(六个面各朝下 + 若干倾斜),放稳后自动记录。\n")

    poses = []          # 每个姿态的加速度均值
    pose_g = []         # 对应的陀螺均值(用于核查)
    buf_a, buf_g = [], []
    still_since = None
    last_msg = 0.0
    try:
        while len(poses) < args.poses:
            fs = pipe.wait_for_frames(5000)
            for f in fs:
                mf = f.as_motion_frame()
                if not mf:
                    continue
                d = mf.get_motion_data()
                if mf.get_profile().stream_type() == rs.stream.accel:
                    buf_a.append((d.x, d.y, d.z))
                else:
                    buf_g.append((d.x, d.y, d.z))
            if len(buf_a) > 500: buf_a = buf_a[-500:]
            if len(buf_g) > 500: buf_g = buf_g[-500:]
            if len(buf_g) < 100 or len(buf_a) < 100:
                continue

            w = np.degrees(np.linalg.norm(np.array(buf_g[-100:]), axis=1)).mean()
            now = time.time()
            if w < args.gyro_thr:
                if still_since is None:
                    still_since = now
                held = now - still_since
                if held >= args.hold:
                    a = np.array(buf_a[-int(250 * args.hold * 0.6):]).mean(axis=0)
                    u = a / np.linalg.norm(a)
                    sep = min([np.degrees(np.arccos(np.clip(u @ (p / np.linalg.norm(p)), -1, 1)))
                               for p in poses], default=180.0)
                    if sep >= args.min_sep:
                        poses.append(a); pose_g.append(np.array(buf_g[-100:]).mean(axis=0))
                        print(f"\r  ✓ 姿态 {len(poses)}/{args.poses}  "
                              f"a=[{a[0]:+7.3f},{a[1]:+7.3f},{a[2]:+7.3f}]  "
                              f"|a|={np.linalg.norm(a):.4f}  与最近姿态夹角 {sep:.0f}°"
                              + " " * 8)
                        still_since = None
                        buf_a, buf_g = [], []
                        print("     -> 换下一个姿态")
                    else:
                        if now - last_msg > 2:
                            print(f"\r  与已有姿态太接近({sep:.0f}°<{args.min_sep:.0f}°),换个朝向"
                                  + " " * 20, end='', flush=True)
                            last_msg = now
                        still_since = now      # 重新计时
                elif now - last_msg > 0.5:
                    print(f"\r  静止中 {held:.1f}/{args.hold:.0f}s   |w|={w:.2f}°/s"
                          + " " * 20, end='', flush=True)
                    last_msg = now
            else:
                if still_since is not None and now - last_msg > 0.5:
                    print(f"\r  移动中 |w|={w:6.1f}°/s   已采 {len(poses)}/{args.poses}"
                          + " " * 12, end='', flush=True)
                    last_msg = now
                still_since = None
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        pipe.stop()

    if len(poses) < 9:
        print(f"\n只采到 {len(poses)} 个姿态,少于 9 个未知数,无法求解")
        if not poses: return
    P = np.array(poses)
    np.savez_compressed(args.out, pose_accel=P, pose_gyro=np.array(pose_g))
    n = np.linalg.norm(P, axis=1)
    print(f"\n\n写入 {args.out}:{len(P)} 个姿态")
    print(f"  |a| 范围 {n.min():.4f} ~ {n.max():.4f}   均值 {n.mean():.4f}  std {n.std():.4f}")
    print(f"  (标定前 |a| 在不同姿态下的离散程度,就是标度因子/非正交的直接体现)")


if __name__ == '__main__':
    main()
