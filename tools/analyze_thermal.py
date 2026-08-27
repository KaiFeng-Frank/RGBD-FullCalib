#!/usr/bin/env python3
"""温漂分析:从升温曲线里拟合出零偏与深度的温度系数。

零偏怎么取:静置时陀螺的真值恒为 0,所以滑窗均值直接就是零偏;
加速度计取三轴均值(姿态固定,重力投影是常数,随温度的变化即零偏漂移)。

这跟 Allan 方差的处理方向相反 —— Allan 要去掉趋势看随机部分,
这里要的正是趋势本身。

用法: python analyze_thermal.py data/thermal.npz
"""
import argparse
import os
import sys

import numpy as np


def fit(T, y, deg=1):
    m = np.isfinite(T) & np.isfinite(y)
    if m.sum() < 5:
        return None
    c = np.polyfit(T[m], y[m], deg)
    pred = np.polyval(c, T[m])
    ss_res = ((y[m] - pred) ** 2).sum()
    ss_tot = ((y[m] - y[m].mean()) ** 2).sum()
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return c, r2, float(np.sqrt(ss_res / m.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz')
    ap.add_argument('--window', type=float, default=30.0, help='零偏滑窗半宽(秒)')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                   'results', 'thermal')

    d = np.load(args.npz, allow_pickle=True)
    it, ig, ia = d['imu_t'], d['imu_g'].astype(np.float64), d['imu_a'].astype(np.float64)
    slow = d['slow']
    if len(slow) < 5:
        print('慢采样点太少'); sys.exit(1)
    irel = it - it[0]
    T = slow[:, 1]                       # ASIC 温度
    trel = slow[:, 0]
    dur = trel[-1] - trel[0]
    print(f"IMU {len(it)} 样本 {irel[-1]/3600:.2f} 小时   慢采样 {len(slow)} 点")
    print(f"ASIC 温度 {T.min():.1f} -> {T.max():.1f} C  (跨度 {T.max()-T.min():.1f} C)")
    if T.max() - T.min() < 8:
        print("⚠ 温度跨度不足,拟合不可靠")

    # ---- 每个温度采样点对应的 IMU 零偏 ----
    gb = np.full((len(slow), 3), np.nan)
    ab = np.full((len(slow), 3), np.nan)
    for i, tr in enumerate(trel):
        m = np.abs(irel - tr) < args.window
        if m.sum() < 100:
            continue
        gb[i] = ig[m].mean(axis=0)
        ab[i] = ia[m].mean(axis=0)

    print("\n" + "=" * 74)
    print("陀螺零偏温度系数   (静置时真值为 0,均值即零偏)")
    print("=" * 74)
    print(f"{'轴':>6} {'零偏@最低温':>14} {'零偏@最高温':>14} {'温度系数':>18} {'R²':>7}")
    gyro_coef = {}
    for i, ax in enumerate('xyz'):
        r = fit(T, np.degrees(gb[:, i]))
        if not r: continue
        c, r2, rms = r
        lo, hi = np.polyval(c, T.min()), np.polyval(c, T.max())
        gyro_coef[ax] = c[0]
        print(f"  gyro{ax} {lo:>12.4f}   {hi:>12.4f}   {c[0]:>12.5f} °/s/°C {r2:>7.3f}")
    print(f"\n  对比 Allan 噪声密度 gyro 1.987e-4 rad/s/√Hz = {np.degrees(1.987e-4):.4f} °/s/√Hz")

    print("\n" + "=" * 74)
    print("加速度计零偏温度系数   (姿态固定,重力投影为常数)")
    print("=" * 74)
    print(f"{'轴':>6} {'@最低温':>13} {'@最高温':>13} {'温度系数':>20} {'R²':>7}")
    for i, ax in enumerate('xyz'):
        r = fit(T, ab[:, i])
        if not r: continue
        c, r2, rms = r
        lo, hi = np.polyval(c, T.min()), np.polyval(c, T.max())
        print(f"  acc{ax}  {lo:>12.5f} {hi:>12.5f}   {c[0]:>12.6f} m/s²/°C {r2:>7.3f}")
    gnorm = np.linalg.norm(ab, axis=1)
    r = fit(T, gnorm)
    if r:
        c, r2, _ = r
        print(f"\n  |a| {np.polyval(c,T.min()):.5f} -> {np.polyval(c,T.max()):.5f} m/s²  "
              f"系数 {c[0]:+.6f} /°C  R²={r2:.3f}   (标准重力 9.80665)")

    print("\n" + "=" * 74)
    print("深度温漂   (固定平面,距离随温度的变化)")
    print("=" * 74)
    dist = slow[:, 3]
    r = fit(T, dist)
    if r:
        c, r2, rms = r
        lo, hi = np.polyval(c, T.min()), np.polyval(c, T.max())
        print(f"  平面距离 {lo:.5f} -> {hi:.5f} m   变化 {(hi-lo)*1000:+.2f} mm")
        print(f"  温度系数 {c[0]*1000:+.4f} mm/°C   相对 {c[0]/np.nanmean(dist)*1e6:+.1f} ppm/°C   R²={r2:.3f}")
        print(f"\n  深度 z = B·f/disparity,故 Δz/z 等于基线的相对变化;")
        print(f"  推得基线温度系数 {c[0]/np.nanmean(dist)*50.148*1000:+.4f} µm/°C  (基线 50.148mm)")
        print(f"  参考:铝合金线胀系数 ~23 ppm/°C")
    rms_ = slow[:, 4]
    r2_ = fit(T, rms_ * 1000)
    if r2_:
        c, r2, _ = r2_
        print(f"\n  平面拟合残差 {np.polyval(c,T.min()):.3f} -> {np.polyval(c,T.max()):.3f} mm  "
              f"系数 {c[0]:+.4f} mm/°C  R²={r2:.3f}")


    # ---- 靶标:主点漂移 vs 焦距漂移 ----
    tgt = d['target'] if 'target' in d else np.zeros((0, 73))
    intr = d['intr']; fx, fy, cx0, cy0 = [float(v) for v in intr]
    pp_coef = None
    if len(tgt) >= 5 and tgt.shape[1] >= 73:
        xy = tgt[:, 1:].reshape(len(tgt), 36, 2)
        # 要求 100% 可见太严:实测每帧检出中位 24 个,但子集在变,
        # 全程都在的只剩 8 个。放宽到 >=90% 可见并对缺失帧插值,可用 tag 翻倍。
        seen = np.isfinite(xy[:, :, 0])
        common = seen.mean(axis=0) >= 0.90
        for k in np.where(common)[0]:
            for c_ in range(2):
                col = xy[:, k, c_]
                m = np.isfinite(col)
                if m.sum() >= 2 and (~m).any():
                    xy[~m, k, c_] = np.interp(tgt[~m, 0], tgt[m, 0], col[m])
        print("\n" + "=" * 74)
        print("靶标像素漂移   (板与相机固定,位移即内参漂移)")
        print("=" * 74)
        print(f"  全程可见的 tag {int(common.sum())}/36")
        if common.sum() >= 6:
            P = xy[:, common, :]                    # (N, K, 2)
            base = P[0]
            # 位移场分解:Δu = Δcx + (Δf/f)·(u-cx) ,Δv = Δcy + (Δf/f)·(v-cy)
            #   平移分量 -> 主点漂移;径向缩放分量 -> 焦距漂移
            dcx = np.full(len(P), np.nan); dcy = np.full(len(P), np.nan)
            dfr = np.full(len(P), np.nan); res = np.full(len(P), np.nan)
            for i in range(len(P)):
                du = (P[i] - base).ravel()
                ru = base[:, 0] - cx0; rv = base[:, 1] - cy0
                A = np.zeros((2 * len(base), 3))
                A[0::2, 0] = 1.0; A[0::2, 2] = ru
                A[1::2, 1] = 1.0; A[1::2, 2] = rv
                sol, *_ = np.linalg.lstsq(A, du, rcond=None)
                dcx[i], dcy[i], dfr[i] = sol
                res[i] = np.sqrt(((A @ sol - du) ** 2).mean())
            # 与温度的关系
            tT = np.interp(tgt[:, 0], trel, T)
            print(f"\n  {'量':>22} {'@最低温':>11} {'@最高温':>11} {'温度系数':>20} {'R²':>7}")
            for name, y, unit, scale_ in [
                    ('主点漂移 Δcx', dcx, 'px/°C', 1.0),
                    ('主点漂移 Δcy', dcy, 'px/°C', 1.0),
                    ('焦距相对变化 Δf/f', dfr * 1e6, 'ppm/°C', 1.0)]:
                r = fit(tT, y)
                if not r: continue
                c, r2, _ = r
                lo, hi = np.polyval(c, tT.min()), np.polyval(c, tT.max())
                print(f"  {name:>22} {lo:>11.4f} {hi:>11.4f} {c[0]:>13.5f} {unit:>7} {r2:>7.3f}")
                if name.endswith('Δcx'):
                    pp_coef = float(c[0])
            print(f"\n  拟合残差(模型解释不了的部分) 中位 {np.nanmedian(res):.4f} px")
            print(f"  焦距漂移换算:Δf/f 每度 {np.polyfit(tT, dfr*1e6, 1)[0]:.2f} ppm -> "
                  f"fx {fx:.1f} 每度变 {np.polyfit(tT, dfr, 1)[0]*fx*1e3:.4f} milli-px")
            print(f"  参考:主点漂移若达 0.5 px,4m 处横向定位误差约 "
                  f"{0.5/fx*4*1000:.1f} mm")
        else:
            print("  全程可见 tag 不足 6 个,无法分解位移场")

    # ---- 出图 ----
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import font_manager as fm
    import matplotlib.pyplot as plt
    try:
        fm.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
        plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except Exception:
        pass
    fig, ax = plt.subplots(3, 3, figsize=(16.5, 12.5), dpi=110)
    ax[0][0].plot(trel / 60, T, lw=1.6, label='ASIC')
    ax[0][0].plot(trel / 60, slow[:, 2], lw=1.2, label='Projector')
    ax[0][0].set_xlabel('时间 [min]'); ax[0][0].set_ylabel('温度 [°C]')
    ax[0][0].set_title('升温曲线'); ax[0][0].legend(fontsize=8); ax[0][0].grid(alpha=.25)
    for i, axn in enumerate('xyz'):
        ax[0][1].scatter(T, np.degrees(gb[:, i]), s=7, alpha=.6, label=f'gyro{axn}')
        rr = fit(T, np.degrees(gb[:, i]))
        if rr: ax[0][1].plot(np.sort(T), np.polyval(rr[0], np.sort(T)), lw=1.2)
    ax[0][1].set_xlabel('ASIC 温度 [°C]'); ax[0][1].set_ylabel('陀螺零偏 [°/s]')
    ax[0][1].set_title('陀螺零偏 vs 温度'); ax[0][1].legend(fontsize=8); ax[0][1].grid(alpha=.25)
    for i, axn in enumerate('xyz'):
        ax[0][2].scatter(T, ab[:, i], s=7, alpha=.6, label=f'acc{axn}')
        rr = fit(T, ab[:, i])
        if rr: ax[0][2].plot(np.sort(T), np.polyval(rr[0], np.sort(T)), lw=1.2)
    ax[0][2].set_xlabel('ASIC 温度 [°C]'); ax[0][2].set_ylabel('加速度均值 [m/s²]')
    ax[0][2].set_title('加速度计 vs 温度'); ax[0][2].legend(fontsize=8); ax[0][2].grid(alpha=.25)
    ax[1][0].scatter(T, dist * 1000, s=8, alpha=.7, color='#e45756')
    rr = fit(T, dist * 1000)
    if rr: ax[1][0].plot(np.sort(T), np.polyval(rr[0], np.sort(T)), lw=1.6, color='k')
    ax[1][0].set_xlabel('ASIC 温度 [°C]'); ax[1][0].set_ylabel('平面距离 [mm]')
    ax[1][0].set_title('深度温漂'); ax[1][0].grid(alpha=.25)
    ax[1][1].scatter(T, rms_ * 1000, s=8, alpha=.7, color='#4c78a8')
    ax[1][1].set_xlabel('ASIC 温度 [°C]'); ax[1][1].set_ylabel('平面残差 RMS [mm]')
    ax[1][1].set_title('深度噪声 vs 温度'); ax[1][1].grid(alpha=.25)
    ax[1][2].plot(trel / 60, dist * 1000, lw=1.4, color='#e45756')
    ax[1][2].set_xlabel('时间 [min]'); ax[1][2].set_ylabel('平面距离 [mm]')
    ax[1][2].set_title('深度随时间(与升温曲线对照)'); ax[1][2].grid(alpha=.25)
    if len(tgt) >= 5 and tgt.shape[1] >= 73 and 'common' in dir() and common.sum() >= 6:
        tT = np.interp(tgt[:, 0], trel, T)
        ax[2][0].scatter(tT, dcx, s=8, alpha=.7, label='Δcx')
        ax[2][0].scatter(tT, dcy, s=8, alpha=.7, label='Δcy')
        for y in (dcx, dcy):
            rr = fit(tT, y)
            if rr: ax[2][0].plot(np.sort(tT), np.polyval(rr[0], np.sort(tT)), lw=1.2)
        ax[2][0].set_xlabel('ASIC 温度 [°C]'); ax[2][0].set_ylabel('主点漂移 [px]')
        ax[2][0].set_title('主点漂移 vs 温度'); ax[2][0].legend(fontsize=8); ax[2][0].grid(alpha=.25)
        ax[2][1].scatter(tT, dfr * 1e6, s=8, alpha=.7, color='#54a24b')
        rr = fit(tT, dfr * 1e6)
        if rr: ax[2][1].plot(np.sort(tT), np.polyval(rr[0], np.sort(tT)), lw=1.4, color='k')
        ax[2][1].set_xlabel('ASIC 温度 [°C]'); ax[2][1].set_ylabel('Δf/f [ppm]')
        ax[2][1].set_title('焦距相对变化 vs 温度'); ax[2][1].grid(alpha=.25)
        ax[2][2].plot(tgt[:, 0] / 60, res, lw=1.2, color='#b279a2')
        ax[2][2].set_xlabel('时间 [min]'); ax[2][2].set_ylabel('位移场拟合残差 [px]')
        ax[2][2].set_title('模型解释不了的部分'); ax[2][2].grid(alpha=.25)
    else:
        for k in range(3): ax[2][k].axis('off')
    plt.tight_layout()
    png = out + '.png'
    os.makedirs(os.path.dirname(png), exist_ok=True)
    plt.savefig(png, bbox_inches='tight')
    print(f"\n图 -> {png}")

    import json
    rec = dict(duration_h=float(dur / 3600), n_imu=int(len(it)), n_slow=int(len(slow)),
               T_range=[float(T.min()), float(T.max())],
               gyro_coef_deg_s_per_C={k: float(v) for k, v in gyro_coef.items()})
    with open(out + '.json', 'w') as f:
        json.dump(rec, f, indent=2, ensure_ascii=False)
    print(f"数值 -> {out}.json")

    # ---- 机器可读的补偿模型:给 viewer / SLAM 直接消费 ----
    def coef(y, x=T):
        r = fit(x, y)
        return (float(r[0][0]), float(r[0][1]), float(r[1])) if r else (0.0, 0.0, 0.0)

    T_ref = float(np.nanmin(T))
    model = {'T_ref_C': T_ref,
             'T_range_C': [float(T.min()), float(T.max())],
             'source': os.path.basename(args.npz),
             'duration_h': float(dur / 3600),
             'note': '一次项系数;补偿量 = 系数 x (T - T_ref)'}
    kg, r2g = {}, {}
    for i, ax in enumerate('xyz'):
        k, b, r2 = coef(np.degrees(gb[:, i])); kg[ax] = k; r2g[ax] = r2
    model['gyro_bias_deg_s_per_C'] = kg
    model['gyro_bias_r2'] = r2g
    ka, r2a = {}, {}
    for i, ax in enumerate('xyz'):
        k, b, r2 = coef(ab[:, i]); ka[ax] = k; r2a[ax] = r2
    model['accel_bias_ms2_per_C'] = ka
    model['accel_bias_r2'] = r2a
    kd, bd, r2d = coef(dist)
    model['depth_m_per_C'] = kd
    model['depth_ppm_per_C'] = float(kd / np.nanmean(dist) * 1e6) if np.nanmean(dist) else 0.0
    model['depth_r2'] = r2d
    model['depth_ref_dist_m'] = float(np.nanmean(dist))
    if pp_coef is not None:
        tT = np.interp(tgt[:, 0], trel, T)
        kx, _, r2x = coef(dcx, tT); ky, _, r2y = coef(dcy, tT)
        kf, _, r2f = coef(dfr * 1e6, tT)
        model['principal_px_per_C'] = {'cx': kx, 'cy': ky}
        model['principal_r2'] = {'cx': r2x, 'cy': r2y}
        model['focal_ppm_per_C'] = kf
        model['focal_r2'] = r2f
    mp = os.path.join(os.path.dirname(out), 'thermal_model.json')
    with open(mp, 'w') as f:
        json.dump(model, f, indent=2, ensure_ascii=False)
    print(f"补偿模型 -> {mp}   (viewer 会自动读取)")


if __name__ == '__main__':
    main()
