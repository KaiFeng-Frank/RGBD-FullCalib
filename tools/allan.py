#!/usr/bin/env python3
"""IMU Allan 方差分析 —— 从静置数据算出 Kalibr 需要的四个噪声参数。

Allan 偏差曲线 σ(τ) 在 log-log 图上由几段不同斜率拼成,每段对应一种噪声:

    斜率 -1/2   白噪声          gyro: 角度随机游走 ARW    acc: 速度随机游走 VRW
                                这就是 Kalibr 的 noise_density
    斜率  0     零偏不稳定性     曲线最低点,B = σ_min / 0.664
    斜率 +1/2   随机游走         gyro: 角速率随机游走 RRW  acc: 加速度随机游走
                                这就是 Kalibr 的 random_walk

读法:短 τ 段是"传感器有多吵",长 τ 段是"零偏漂得有多快"。
VIO 里前者影响单帧观测权重,后者决定 bias 状态需要多快地被重新估计。

用法: python allan.py data/imu_allan.bag -o results/allan
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import TS
from rosbags.rosbag1 import Reader


def load(bag, topic='/imu0'):
    t, g, a = [], [], []
    with Reader(bag) as r:
        for conn, ts, raw in r.messages():
            if conn.topic != topic:
                continue
            m = TS.deserialize_ros1(raw, conn.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    return np.array(t), np.array(g), np.array(a)


def allan_dev(x, fs, taus):
    """重叠式 Allan 偏差。x: (N,) 速率量(rad/s 或 m/s^2)"""
    N = len(x)
    theta = np.cumsum(x) / fs           # 积分成角度/速度
    out_tau, out_dev = [], []
    for tau in taus:
        m = int(round(tau * fs))
        if m < 1 or 2 * m >= N:
            continue
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        av = (d ** 2).sum() / (2 * (tau ** 2) * len(d))
        out_tau.append(m / fs)
        out_dev.append(np.sqrt(av))
    return np.array(out_tau), np.array(out_dev)


def fit_allan_model(tau, dev, T):
    """两步拟合 σ²(τ) = N²/τ + B'² + K²τ/3,返回 (N, B, K)。

    为什么分两步:三个参数同时拟合时,B'(常数项)会把随机游走项吃掉 ——
    在有限的 τ 范围内 B'² 和 K²τ/3 高度相关,优化器很容易给出 K≈0 的解。
    而 N 在短 τ 段几乎不受另外两项污染,可以先单独定死,大幅降低耦合。
    """
    from scipy.optimize import curve_fit
    var = dev ** 2

    # --- 步骤 1:短 τ 段定 N。那里 σ ≈ N/√τ,另两项可忽略 ---
    m = tau <= max(4.0 / (len(tau) and 1), min(1.0, T / 3600))
    m = tau <= 1.0 if (tau <= 1.0).sum() >= 5 else tau <= np.percentile(tau, 20)
    lt, ld = np.log10(tau[m]), np.log10(dev[m])
    sl, ic = np.polyfit(lt, ld, 1)
    N = 10 ** ic                       # 直线在 τ=1 处的值

    # --- 步骤 2:扣掉白噪声,用剩余拟合 B' 和 K ---
    rest = var - N ** 2 / tau
    # 只用「白噪声已不再主导」的点:rest 至少占白噪声项的 30%
    keep = (rest > 0.3 * N ** 2 / tau) & (tau > 1.0) & (tau < T / 3)
    if keep.sum() < 6:
        return float(N), None, None

    def model(x, logB, logK):
        return np.log10(10 ** (2 * logB) + (10 ** (2 * logK)) * x / 3.0)

    p0 = [np.log10(max(np.sqrt(rest[keep]).min(), 1e-13)),
          np.log10(max(np.sqrt(rest[keep][-1] * 3 / tau[keep][-1]), 1e-13))]
    try:
        popt, _ = curve_fit(model, tau[keep], np.log10(rest[keep]), p0=p0, maxfev=40000)
        B, K = 10 ** popt[0], 10 ** popt[1]
    except Exception:
        return float(N), None, None
    return float(N), float(B), float(K)


def analyze_axis(x, fs, taus, label):
    tau, dev = allan_dev(x, fs, taus)
    T = len(x) / fs
    res = dict(tau=tau, dev=dev)
    N, B, K = fit_allan_model(tau, dev, T)
    res['N'] = N
    res['K'] = K
    res['B_fit'] = B if B else dev.min() / 0.664
    i = int(np.argmin(dev))
    res['B'] = dev[i] / 0.664
    res['B_tau'] = tau[i]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'allan'))
    ap.add_argument('--topic', default='/imu0')
    args = ap.parse_args()

    print(f'读取 {args.bag} ...')
    t, g, a = load(args.bag, args.topic)
    if len(t) < 1000:
        print('样本太少'); sys.exit(1)
    dt = np.diff(t)
    fs = 1.0 / np.median(dt)
    T = t[-1] - t[0]
    print(f'  {len(t)} 样本   {T/3600:.2f} 小时   {fs:.2f} Hz')
    print(f'  采样间隔 p99 {np.percentile(dt,99)*1000:.3f}ms  最大 {dt.max()*1000:.1f}ms  '
          f'大间隔(>3x) {int((dt>3*np.median(dt)).sum())}')
    gap = dt > 3 * np.median(dt)
    if gap.sum() > len(t) * 0.001:
        print('  ⚠ 数据有明显断流,Allan 结果可能失真')
    n = np.linalg.norm(a, axis=1)
    print(f'  |a| = {n.mean():.4f} ± {n.std():.4f} m/s^2')
    if n.std() > 0.3:
        print('  ⚠ 加速度波动大 —— 采集期间相机可能被碰到了')

    taus = np.logspace(np.log10(2.0 / fs), np.log10(T / 3), 120)
    res = {}
    for nm, data, unit in [('gyro', g, 'rad/s'), ('accel', a, 'm/s^2')]:
        for i, ax in enumerate('xyz'):
            res[f'{nm}_{ax}'] = analyze_axis(data[:, i], fs, taus, f'{nm}{ax}')

    print('\n' + '=' * 74)
    print(f"{'':>9} {'噪声密度 N':>16} {'随机游走 K':>18} {'零偏不稳定 B':>16} {'@τ':>7}")
    print('=' * 74)
    summary = {}
    for nm, unit_n, unit_k in [('gyro', 'rad/s/√Hz', 'rad/s²/√Hz'),
                               ('accel', 'm/s²/√Hz', 'm/s³/√Hz')]:
        Ns, Ks = [], []
        for ax in 'xyz':
            r = res[f'{nm}_{ax}']
            N, K, B = r.get('N'), r.get('K'), r.get('B')
            if N: Ns.append(N)
            if K: Ks.append(K)
            print(f"{nm+ax:>9} {N:>16.3e} {K:>18.3e} {B:>16.3e} {r['B_tau']:>6.1f}s"
                  if N and K else f"{nm+ax:>9}  拟合失败")
        summary[nm] = (float(np.mean(Ns)), float(np.mean(Ks)))
        print(f"{'  平均':>9} {summary[nm][0]:>16.3e} {summary[nm][1]:>18.3e}"
              f"      ({unit_n} / {unit_k})")

    # Kalibr imu.yaml
    gn, gw = summary['gyro']; an, aw = summary['accel']
    yaml_path = args.out + '_imu.yaml'
    os.makedirs(os.path.dirname(yaml_path) or '.', exist_ok=True)
    with open(yaml_path, 'w') as f:
        f.write(f"""#Accelerometers
accelerometer_noise_density: {an:.6e}   #Noise density (continuous-time)
accelerometer_random_walk:   {aw:.6e}   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     {gn:.6e}   #Noise density (continuous-time)
gyroscope_random_walk:       {gw:.6e}   #Bias random walk

rostopic:                    {args.topic}
update_rate:                 {fs:.1f}

# 由 {os.path.basename(args.bag)} 的 {T/3600:.2f} 小时静置数据算出。
# 注意:Kalibr 官方建议把 Allan 算出的值放大 5~10 倍再用于标定,
# 因为真实工况下的振动和温漂远超实验室静置条件。
""")
    print(f'\nKalibr 噪声参数 -> {yaml_path}')

    # 出图
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
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.6), dpi=115)
    for ax_, nm, unit in [(axes[0], 'gyro', 'rad/s'), (axes[1], 'accel', 'm/s²')]:
        for c, axl in zip(['#4c78a8', '#f58518', '#54a24b'], 'xyz'):
            r = res[f'{nm}_{axl}']
            ax_.loglog(r['tau'], r['dev'], color=c, lw=1.6, label=f'{nm}{axl}')
        r = res[f'{nm}_x']
        tt = np.logspace(np.log10(r['tau'].min()), np.log10(r['tau'].max()), 100)
        if 'N' in r:
            ax_.loglog(tt, r['N'] / np.sqrt(tt), '--', color='#aaa', lw=1,
                       label=r'$N/\sqrt{\tau}$  白噪声')
            ax_.loglog(tt, r['K'] * np.sqrt(tt / 3), ':', color='#aaa', lw=1.2,
                       label=r'$K\sqrt{\tau/3}$  随机游走')
            full = np.sqrt(r['N']**2 / tt + r['B_fit']**2 + r['K']**2 * tt / 3)
            ax_.loglog(tt, full, '-', color='#c00', lw=1.0, alpha=0.7,
                       label='三参数模型拟合')
        ax_.set_xlabel(r'簇时间 $\tau$ [s]')
        ax_.set_ylabel(f'Allan 偏差 [{unit}]')
        ax_.set_title(f'{nm}  Allan 偏差   ({T/3600:.2f} h @ {fs:.0f} Hz)')
        ax_.grid(True, which='both', alpha=0.25)
        ax_.legend(fontsize=8)
    plt.tight_layout()
    png = args.out + '.png'
    plt.savefig(png, bbox_inches='tight')
    print(f'图 -> {png}')


if __name__ == '__main__':
    main()
