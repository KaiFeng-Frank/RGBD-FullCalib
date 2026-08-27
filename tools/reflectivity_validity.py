#!/usr/bin/env python3
"""材质/反射率 → 深度有效性:用 IR 亮度当反射率代理,不摆材质阵列。

原理:
  黑色吸光、镜面反光、半透明都表现为"回到 IR 相机的散斑能量异常"——
  IR 亮度就是这个能量的直接读数。所以不需要摆一排材质样品:任何有明暗
  差异的场景,按 IR 亮度分箱统计深度的有效率和时域噪声,就是这项实验。

  诚实声明:亮度与距离/入射角相关,这里测的是**场景内预测力**不是纯材质
  因果 —— 但门控应用要的恰恰是预测力:IR 亮度是外部信号(不依赖深度残差),
  按它做 0/1 成员资格弃权,正合"判决信号必须外部"的纪律。

产出:有效率/噪声 vs 亮度曲线 + 建议门限(两端:欠曝失效 + 过曝失效)。
"""
import argparse
import json
import os

import numpy as np


def analyze(D, Imean, nb=14, valid_lo=0.15):
    """D: (F,H,W) 深度栈;Imean: (H,W) IR 均值。"""
    valid_frac = (D > valid_lo).mean(0)
    # 时域噪声:仅统计在 ≥80% 帧里有效的像素(断续有效的像素 std 无意义)
    stable = valid_frac >= 0.8
    Dm = np.where(D > valid_lo, D, np.nan)
    tstd = np.nanstd(Dm, 0)
    zmed = np.nanmedian(Dm, 0)
    edges = np.quantile(Imean, np.linspace(0, 1, nb + 1))
    rows = []
    for i in range(nb):
        m = (Imean >= edges[i]) & (Imean < edges[i + 1])
        if m.sum() < 500:
            continue
        ms = m & stable
        rows.append(dict(
            ir_lo=float(edges[i]), ir_hi=float(edges[i + 1]),
            n=int(m.sum()),
            valid_pct=float(100 * valid_frac[m].mean()),
            tnoise_mm=float(np.nanmedian(tstd[ms]) * 1000) if ms.sum() > 200 else None,
            z_med=float(np.nanmedian(zmed[ms])) if ms.sum() > 200 else None))
    return rows


def thresholds(rows, ok_pct=90.0):
    """从曲线找建议门限 —— 但先做混杂检验,不单峰就拒绝。

    单场景观察数据里,亮度可能主要编码"哪类物体"(镜面地板 vs 平坦墙)而
    不是反射率:那时曲线中段塌陷、两端反而高,任何门限都是把场景结构误当
    材质规律。宁可输出"需要受控摆位",不给一个能用但错误的数字。
    实测案例:瓷砖地板反光区(IR>180,13% 散斑饱和)有效率 44%,
    而暗端远墙 93% —— 曲线完全倒挂。
    """
    v = [r['valid_pct'] for r in rows]
    mid = max(range(len(v)), key=lambda i: v[i])
    # 单峰性:峰值到两端应大体单调下降;中途回升 >8% 视为混杂
    def monotone(seq):
        worst = 0.0
        low = seq[0]
        for x in seq[1:]:
            low = min(low, x)
            worst = max(worst, x - low)
        return worst
    bump = max(monotone(v[mid::-1][::-1][::-1]), monotone(v[mid:]))
    if bump > 8.0:
        return None, None, (f'曲线非单峰(回升 {bump:.0f}%):亮度在本场景编码物体'
                            f'类别而非反射率,单因子门限无效 —— 需受控摆位'
                            f'(同距离多材质并排)')
    lo = hi = None
    for i in range(mid, -1, -1):
        if v[i] < ok_pct:
            lo = rows[i]['ir_hi']; break
    for i in range(mid, len(rows)):
        if v[i] < ok_pct:
            hi = rows[i]['ir_lo']; break
    return lo, hi, None


def selftest():
    print("自测:合成已知失效关系,验证曲线与门限恢复")
    rng = np.random.default_rng(0)
    H, W, F = 120, 160, 30
    I = np.tile(np.linspace(10, 240, W, dtype=np.float32), (H, 1))
    D = np.full((F, H, W), 1.5, np.float32) + rng.normal(0, 0.004, (F, H, W))
    # 注入:亮度<40 有效率 50%;>200 有效率 30%;中段 100%
    drop_dark = (I < 40); drop_bright = (I > 200)
    for f in range(F):
        m = rng.random((H, W))
        D[f][drop_dark & (m < 0.5)] = 0
        D[f][drop_bright & (m < 0.7)] = 0
    rows = analyze(D, I)
    lo, hi, why = thresholds(rows)
    ok = why is None and lo is not None and hi is not None and 25 < lo < 60 and 180 < hi < 220
    print(f"门限恢复: 暗端 <{lo:.0f}  亮端 >{hi:.0f}  " + ("✓" if ok else "✗"))
    # 混杂 case:中段人工塌陷 -> 必须拒绝
    for f in range(F):
        m = rng.random((H, W))
        D[f][((I > 100) & (I < 140)) & (m < 0.6)] = 0
    _, _, why2 = thresholds(analyze(D, I))
    ok2 = why2 is not None
    print("混杂检验: " + ("✓ 正确拒绝 —— " + why2[:40] if ok2 else "✗ 没拒绝"))
    print("自测" + ("通过" if ok and ok2 else "失败"))
    return ok and ok2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--npz', help='含 D(帧栈) 与 Imean 的 npz')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results', 'reflectivity_validity.json'))
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest() else 1)
    z = np.load(a.npz)
    D = z['D'].astype(np.float32); Imean = z['Imean']
    rows = analyze(D, Imean)
    lo, hi, why = thresholds(rows)
    print(f"{'IR 亮度':>14} {'有效率':>7} {'时域噪声':>9} {'距离中位':>9} {'像素':>8}")
    for r in rows:
        tn = f"{r['tnoise_mm']:.1f}mm" if r['tnoise_mm'] else '   --'
        zm = f"{r['z_med']:.2f}m" if r['z_med'] else '  --'
        print(f"{r['ir_lo']:6.0f}~{r['ir_hi']:6.0f} {r['valid_pct']:6.1f}% {tn:>9} {zm:>9} {r['n']:8d}")
    if why:
        print(f"\n✗ 不给门限:{why}")
    else:
        print(f"\n建议成员资格门限: IR 亮度 {'< %.0f 弃权' % lo if lo else '暗端无失效'}"
              f"{';  > %.0f 弃权' % hi if hi else ';  亮端无失效'}")
    json.dump(dict(rows=rows, gate_dark=lo, gate_bright=hi, rejected=why,
                   note='场景内预测力,非纯材质因果;IR 亮度=外部门控信号'),
              open(a.out, 'w'), indent=2, ensure_ascii=False)
    print(f"写入 {a.out}")


if __name__ == '__main__':
    main()
