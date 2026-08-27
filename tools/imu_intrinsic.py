#!/usr/bin/env python3
"""加速度计内参标定(六面静置法)。

物理约束:静置时 |a_true| 必须恒等于当地重力,与姿态无关。
实测这台 D435i 的 |a| = 9.864,当地重力 9.7936,偏高 0.72% —— 说明标度因子不是 1。

模型(imu_tk 的形式):
    a_true = T · K · (a_meas − b)
      K = diag(kx, ky, kz)      标度因子
      T = [[1, -ayz, azy],      非正交修正(小角度近似,上三角 3 参数)
           [0,   1, -azx],
           [0,   0,   1  ]]
      b = (bx, by, bz)          零偏
    共 9 个未知数。

每个静置姿态给 1 个约束(|a_true| = g),所以至少 9 个姿态;
实际用 12~18 个姿态(六个面 + 若干倾斜)才稳。

Kalibr 的 imu.yaml 只有 noise_density / random_walk,它假设 K=I、T=I。
这份标定补的正是那个假设。
"""
import argparse
import json
import os
import sys

import numpy as np


def build_A(p):
    """p = [ayz, azy, azx, kx, ky, kz, bx, by, bz] -> (T·K, b)"""
    ayz, azy, azx, kx, ky, kz = p[:6]
    T = np.array([[1.0, -ayz, azy],
                  [0.0, 1.0, -azx],
                  [0.0, 0.0, 1.0]])
    K = np.diag([kx, ky, kz])
    return T @ K, p[6:9]


def residual(p, meas, g):
    M, b = build_A(p)
    corrected = (meas - b) @ M.T
    return np.linalg.norm(corrected, axis=1) - g


def solve(meas, g, verbose=True):
    from scipy.optimize import least_squares
    p0 = np.array([0, 0, 0, 1, 1, 1, 0, 0, 0], float)
    r = least_squares(residual, p0, args=(meas, g), method='lm', max_nfev=20000)
    M, b = build_A(r.x)
    res = residual(r.x, meas, g)
    if verbose:
        print(f"  优化收敛 {r.success}   残差 RMS {np.sqrt((res**2).mean())*1000:.3f} mm/s²")
    return r.x, M, b, res


def local_gravity(lat_deg, alt_m=0.0):
    """WGS84 重力公式 —— 用当地值而非 9.80665,否则 scale 会吸收纬度差异。"""
    s = np.sin(np.radians(lat_deg)) ** 2
    g = 9.7803267715 * (1 + 0.0052790414 * s + 0.0000232718 * s * s)
    return g - 3.086e-6 * alt_m


def report(p, M, b, res, g):
    ayz, azy, azx, kx, ky, kz = p[:6]
    print(f"\n  标度因子   kx={kx:.6f}  ky={ky:.6f}  kz={kz:.6f}")
    print(f"             偏离 1 的量 {(kx-1)*100:+.3f}% {(ky-1)*100:+.3f}% {(kz-1)*100:+.3f}%")
    print(f"  非正交角   ayz={np.degrees(ayz):+.4f}°  azy={np.degrees(azy):+.4f}°  "
          f"azx={np.degrees(azx):+.4f}°")
    print(f"  零偏       bx={b[0]:+.5f}  by={b[1]:+.5f}  bz={b[2]:+.5f}  m/s²")
    print(f"  当地重力   {g:.5f} m/s²")
    print(f"  标定后残差 最大 {np.abs(res).max()*1000:.2f} mm/s²  "
          f"RMS {np.sqrt((res**2).mean())*1000:.2f} mm/s²")


def extract_poses_from_sweep(npz, win_s=1.5, gyro_thr=4.0, sep_deg=12.0,
                             fs_a=250.0, verbose=True):
    """从连续扫描数据里挑出朝向散开的稳定片段。

    不要求绝对静止:手持时角速度通常 1~4 °/s,手抖是高频的,窗口内平均即可压掉。
    真正要排除的是缓慢漂移 —— 所以除了角速度小,还要求窗口内加速度方向本身稳定
    (首尾朝向夹角小)。最后按朝向贪心去重,优先保留最稳的窗口。
    """
    ta = npz['accel_t']; A = npz['accel'].astype(np.float64)
    tg = npz['gyro_t'];  G = np.degrees(npz['gyro'].astype(np.float64))
    wmag = np.linalg.norm(G, axis=1)
    w_at_a = np.interp(ta, tg, wmag)

    n = int(win_s * fs_a)
    step = max(1, n // 4)
    cands = []
    for i in range(0, len(A) - n, step):
        seg = A[i:i + n]
        wseg = w_at_a[i:i + n]
        if wseg.mean() > gyro_thr:
            continue
        m = seg.mean(axis=0)
        nm = np.linalg.norm(m)
        if nm < 5 or nm > 15:
            continue
        u = m / nm
        h1 = seg[:n // 4].mean(axis=0); h2 = seg[-n // 4:].mean(axis=0)
        drift = np.degrees(np.arccos(np.clip(
            (h1 @ h2) / (np.linalg.norm(h1) * np.linalg.norm(h2)), -1, 1)))
        score = wseg.mean() + drift * 2.0        # 越小越稳
        cands.append((score, u, m, drift, wseg.mean()))
    if not cands:
        return np.zeros((0, 3)), []
    cands.sort(key=lambda x: x[0])
    kept, info = [], []
    for sc, u, m, drift, wm in cands:
        if all(np.degrees(np.arccos(np.clip(u @ (k / np.linalg.norm(k)), -1, 1))) >= sep_deg
               for k in kept):
            kept.append(m); info.append((np.linalg.norm(m), wm, drift))
    if verbose:
        print(f"  候选稳定窗口 {len(cands)} 个 -> 朝向去重后 {len(kept)} 个姿态")
        print(f"  {'#':>3} {'|a|':>9} {'角速度':>9} {'朝向漂移':>9}")
        for k, (nm, wm, dr) in enumerate(info[:20]):
            print(f"  {k+1:>3} {nm:>9.4f} {wm:>7.2f}°/s {dr:>8.2f}°")
    return np.array(kept), info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', help='record_imu_poses.py 采集的多姿态静置数据')
    ap.add_argument('--lat', type=float, default=22.3, help='当地纬度(默认香港)')
    ap.add_argument('--alt', type=float, default=30.0, help='海拔 m')
    ap.add_argument('--sweep', action='store_true',
                    help='输入是 record_imu_sweep.py 的连续数据,自动提取姿态')
    ap.add_argument('--gyro-thr', type=float, default=4.0, help='sweep: 角速度上限 deg/s')
    ap.add_argument('--sep', type=float, default=12.0, help='sweep: 朝向去重夹角(度)')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                   'results', 'imu_intrinsic.json')

    d = np.load(args.npz, allow_pickle=True)
    if args.sweep or 'pose_accel' not in d:
        print("从连续扫描数据提取姿态 ...")
        meas, _ = extract_poses_from_sweep(d, gyro_thr=args.gyro_thr, sep_deg=args.sep)
        print()
    else:
        meas = d['pose_accel']
    g = local_gravity(args.lat, args.alt)
    print(f"姿态数 {len(meas)}   当地重力 {g:.5f} m/s² (纬度 {args.lat}°, 海拔 {args.alt}m)")
    if len(meas) < 9:
        print("⚠ 姿态少于 9 个,9 个未知数解不稳"); 
    raw = np.linalg.norm(meas, axis=1)
    print(f"标定前 |a| = {raw.mean():.5f} ± {raw.std():.5f}   偏差 {(raw.mean()/g-1)*100:+.3f}%")

    p, M, b, res = solve(meas, g)
    report(p, M, b, res, g)

    model = dict(scale=[float(p[3]), float(p[4]), float(p[5])],
                 misalign_rad=[float(p[0]), float(p[1]), float(p[2])],
                 bias_ms2=[float(x) for x in b],
                 M=[[float(x) for x in row] for row in M],
                 gravity_local=float(g), lat=args.lat, alt=args.alt,
                 n_poses=int(len(meas)),
                 residual_rms_ms2=float(np.sqrt((res**2).mean())),
                 note='a_true = M @ (a_meas - bias);  M = T·K')
    with open(out, 'w') as f:
        json.dump(model, f, indent=2, ensure_ascii=False)
    print(f"\n模型 -> {out}")


if __name__ == '__main__':
    main()
