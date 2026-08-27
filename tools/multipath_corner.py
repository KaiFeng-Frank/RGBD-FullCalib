#!/usr/bin/env python3
"""多径干扰定量:凹角(墙脚/墙角)处的虚深轮廓。

原理:
  发射的散斑打到面 A 再弹到面 B 才回来,光程变长 → 凹角附近深度被测远
  ("虚深"),点云把直角撑圆。它是**结构化**误差:融合平均不掉,ICP 会被
  它拉偏。要的产出不是"有没有",而是轮廓:虚深多大、伸进多远 ——
  这直接变成成员资格门(交线 r cm 内弃权)。

方法:
  双平面 RANSAC → 解析交线 → 每个平面内点算 (到交线距离 r, 深度方向符号
  残差 δz);δz = ((p−c)·n)/(ray·n),>0 = 测远。按 r 分箱看轮廓。
  平面拟合用 r>30 cm 的点重估(靠角的点已被多径污染,让它们参与拟合会把
  基准面拉歪,虚深被自己吃掉一半)。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_nonlinearity import deproject, fit_plane  # noqa


def line_of(n1, c1, n2, c2):
    w = np.cross(n1, n2); w /= np.linalg.norm(w)
    A = np.stack([n1, n2]); b = np.array([n1 @ c1, n2 @ c2])
    x0 = np.linalg.lstsq(A, b, rcond=None)[0]
    return x0, w


def profile(P, thr_far=0.30, nb=12, rmax=0.6):
    """返回 dict:两平面各自的 (r 分箱, δz 均值 mm, SE, n) + 夹角。"""
    n1, c1, in1 = fit_plane(P)
    P2 = P[~in1]
    n2, c2, in2 = fit_plane(P2)
    ang = float(np.degrees(np.arccos(np.clip(abs(float(n1 @ n2)), 0, 1))))
    if ang < 45:
        return None
    x0, w = line_of(n1, c1, n2, c2)

    def per_plane(Q, n, c):
        # 到交线距离
        v = Q - x0
        r = np.linalg.norm(v - (v @ w)[:, None] * w[None, :], axis=1)
        # 用远离角的点重估平面基准(防污染)
        far = r > thr_far
        if far.sum() > 2000:
            cc = Q[far].mean(0)
            nn = np.linalg.svd(Q[far] - cc, full_matrices=False)[2][-1]
            if nn @ n < 0:
                nn = -nn
            n, c = nn, cc
        rays = Q / np.linalg.norm(Q, axis=1, keepdims=True)
        sens = rays @ n
        ok = np.abs(sens) > 0.15
        dz = ((Q[ok] - c) @ n) / sens[ok]
        r = r[ok]
        edges = np.linspace(0, rmax, nb + 1)
        rows = []
        for i in range(nb):
            m = (r >= edges[i]) & (r < edges[i + 1])
            if m.sum() < 150:
                continue
            rows.append([float((edges[i] + edges[i + 1]) / 2 * 100),      # cm
                         float(dz[m].mean() * 1000),                      # mm
                         float(dz[m].std() / np.sqrt(m.sum()) * 1000),
                         int(m.sum())])
        return rows

    return dict(angle_deg=ang,
                plane1=per_plane(P[in1], n1, c1),
                plane2=per_plane(P2[in2], n2, c2))


def summarize(pr):
    """虚深幅度(<8cm 段相对 >30cm 基线)与影响半径(降到 2mm 处)。"""
    out = {}
    for k in ('plane1', 'plane2'):
        rows = pr[k]
        base = np.mean([m for r, m, s, n in rows if r > 30]) if any(r > 30 for r, m, s, n in rows) else 0.0
        near = [m - base for r, m, s, n in rows if r < 8]
        amp = float(np.mean(near)) if near else None
        # 影响半径 = 从交线向外的**连续**超限段;远处地面不平会让个别远箱
        # 超限,用"最远超限箱"会把半径虚报成半米
        radius = None
        for r, m, s, n in rows:
            if abs(m - base) > 2.0:
                radius = float(r)
            else:
                break
        out[k] = dict(amp_mm=amp, radius_cm=radius, baseline_mm=float(base))
    return out


def selftest():
    print("自测:合成 90° 墙角,注入/不注入虚深各验一次")
    rng = np.random.default_rng(0)
    H, W = 480, 848
    FX, FY = 424.75, 428.82
    CX, CY = 418.4, 243.6
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    # 墙 x=1.0(画面右半),地 y=0.8(下半);逐像素取先命中的面
    with np.errstate(divide='ignore'):
        z_wall = 1.0 * FX / (u - CX)
        z_floor = 0.8 * FY / (v - CY)
    z_wall[(u - CX) <= 5] = 9e9
    z_floor[(v - CY) <= 5] = 9e9
    d_true = np.minimum(z_wall, z_floor)
    d_true[(d_true < 0.3) | (d_true > 4.5)] = 0
    ok_all = True
    for A_mm, tau_cm, lbl in [(0.0, 8, '零注入'), (15.0, 8, '注入 15mm/τ=8cm')]:
        d = d_true.copy()
        m = d > 0
        # 到交线(x=1,y=0.8 的直线,方向 z)的 3D 距离
        X = (u - CX) / FX * d; Y = (v - CY) / FY * d
        r = np.sqrt((X - 1.0) ** 2 + (Y - 0.8) ** 2)
        d[m] += A_mm / 1000 * np.exp(-r[m] / (tau_cm / 100))
        d[m] += rng.normal(0, 0.004 * (d[m]) ** 2 / 1.0)
        P, _ = deproject(d, fx=FX, fy=FY, cx=CX, cy=CY)
        pr = profile(P)
        if pr is None:
            print("  !! 双平面失败"); return False
        sm = summarize(pr)
        a1 = sm['plane1']['amp_mm']; a2 = sm['plane2']['amp_mm']
        print(f"  {lbl}: 夹角 {pr['angle_deg']:.0f}°  虚深幅度 平面1 {a1:+.1f} / 平面2 {a2:+.1f} mm")
        if A_mm == 0:
            good = abs(a1) < 3 and abs(a2) < 3
            print("    " + ("✓ 零注入不虚报" if good else "✗ 假阳性"))
        else:
            good = 0.5 * A_mm < max(a1, a2) < 1.6 * A_mm
            print("    " + (f"✓ 恢复幅度(真值 {A_mm})" if good else "✗ 恢复失败"))
        ok_all = ok_all and good
    print("自测" + ("通过" if ok_all else "失败"))
    return ok_all


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--npy', help='深度图 npy')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results', 'multipath_corner.json'))
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest() else 1)
    d = np.load(a.npy)
    P, _ = deproject(d)
    pr = profile(P)
    if pr is None:
        print("场景里没有 >45° 双平面"); return
    sm = summarize(pr)
    print(f"双平面夹角 {pr['angle_deg']:.0f}°\n")
    for k in ('plane1', 'plane2'):
        print(f"{k}   (r=到交线距离)")
        print(f"{'r (cm)':>8} {'δz (mm)':>9} {'SE':>6} {'点数':>8}")
        for r, m, s, n in pr[k]:
            print(f"{r:8.1f} {m:9.2f} {s:6.2f} {n:8d}")
        print(f"  → 虚深幅度(<8cm) {sm[k]['amp_mm']:+.1f} mm,影响半径 ~{sm[k]['radius_cm']} cm\n")
    json.dump(dict(**pr, summary=sm), open(a.out, 'w'), indent=2, ensure_ascii=False)
    print(f"写入 {a.out}")


if __name__ == '__main__':
    main()
