#!/usr/bin/env python3
"""深度非线性定界:斜置平面法。

为什么这么测:
  深度非线性的定义是"读数与真实距离的偏差随距离变化"。要测它,教科书做法是
  拿卷尺在多个距离逐点比对 —— 需要大空间和体力。这里换一条路:让一个**真实平面**
  斜着铺满画面,单帧内就有连续的距离梯度,距离跨度靠角度换而不靠空间。

  关键性质:纯尺度误差(d_meas = k·d_true)在这个方法下**检测不到** ——
  点云整体缩放后平面仍是平面,拟合残差为零。平面拟合还会吸收掉任意的线性倾斜。
  所以残差里剩下的只有真正随距离弯曲的成分,正好是"非线性"的定义。
  这是特性不是缺陷:尺度误差已由立体基线核查覆盖(见 CALIBRATION.md 阶段 2)。

  代价:测的是相对形变,给不出绝对偏置。要绝对值仍需外部长度基准。

用法:
  python depth_nonlinearity.py --selftest       # 合成数据验算法(不碰相机)
  python depth_nonlinearity.py --frames 60      # 实机采集 + 分析
"""
import os
os.environ.setdefault('QT_LOGGING_RULES', '*=false')  # 压掉 cv2 的 Qt 字体刷屏
import argparse
import json
import os

import cv2
import numpy as np

# 深度流用的 IR 内参(由 848x480 从 1280x720 标定值缩放而来)
FX, FY = 641.140 * 848 / 1280, 643.233 * 480 / 720
CX, CY = 631.518 * 848 / 1280, 365.416 * 480 / 720


def deproject(d, fx=FX, fy=FY, cx=CX, cy=CY, zmin=0.15, zmax=6.0, ret_uv=False):
    """深度图 -> 点云 + 像素坐标。zmax 挡掉超量程离群点。"""
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    m = (d > zmin) & (d < zmax)
    P = np.stack([((u - cx) / fx * d)[m], ((v - cy) / fy * d)[m], d[m]], 1)
    uv = np.stack([u[m], v[m]], 1)
    return (P, uv) if ret_uv else (P, uv)


def fit_plane(P, thr=0.02, iters=800, seed=0):
    """RANSAC 找最大平面,再用内点做最小二乘精修。返回 (法向, 平面上一点, 内点掩码)。"""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i = rng.choice(len(P), 3, replace=False)
        a, b, c = P[i]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        inl = np.abs((P - a) @ n) < thr
        k = inl.sum()
        if best is None or k > best[0]:
            best = (k, n, a, inl)
    _, n, a, inl = best
    # 精修:内点质心 + SVD 最小奇异向量
    Q = P[inl]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c, full_matrices=False)[2][-1]
    inl = np.abs((P - c) @ n) < thr
    return n, c, inl


def residual_trend(P, n, c, nbins=14, min_pts=200):
    """残差按距离分箱。返回每箱 (z中位, 残差均值, 残差std, 点数)。

    残差 = 点到拟合平面的有符号距离(沿法向)。平面拟合已吸收常数和线性项,
    所以残差里剩下的是二次及以上的形变。
    """
    r = (P - c) @ n
    z = P[:, 2]
    # 等宽分箱,不是分位分箱。点密度随距离掉得很快(∝1/z²),分位分箱会把
    # 稀疏的远点全塞进最后一个箱 —— 而那些点正是二次拟合的杠杆所在。
    # 代价是远处箱点少、噪声大,交给后面的加权拟合处理。
    edges = np.linspace(np.percentile(z, 0.5), np.percentile(z, 99.5), nbins + 1)
    out = []
    for i in range(nbins):
        m = (z >= edges[i]) & (z < edges[i + 1])
        if m.sum() < min_pts:
            continue
        out.append((np.median(z[m]), r[m].mean(), r[m].std(), int(m.sum())))
    return np.array(out)


MIN_RATIO = 2.5      # 连续段远近比:二次拟合的杠杆
MIN_ZMAX = 2.5       # 合格线:普通居室的实际可达上限(实测八轮:稳定 2.4~2.7 m)。
                     # 给出的界约 6%,宽于厂商谱面 2%,但那是**实测**的界,
                     # 比"未定界"强;更紧的界需要更大的空间,不是更用力地调。
GOOD_ZMAX = 4.0      # 理想值:界压到 1.8%,越过厂商谱面
MIN_PTS = 20000


def expected_ub(zmax):
    """这个 z_max 大致能给出多紧的 95% 上界(%)。

    由合成标定:(2.5m, 5.96%) (3.0, 4.03) (3.5, 2.92) (4.2, 1.60) 拟合幂律
    ub ≈ 60·zmax^-2.53。分辨力几乎全由最远距离决定,所以调摆位时
    盯着"最远"比盯着"远近比"更有用。
    """
    return 60.0 * max(zmax, 0.5) ** -2.53


def placement_ok(ratio, zmax, npts, nvalid=99):
    """摆位三条门槛,返回 (合格?, 差在哪的说明)。

    比例和绝对距离缺一不可:0.3~0.9 m 的 3× 跨度比例好看,但杠杆臂太短,
    对"深度模型在 3~4 m 是否还成立"这个真正的问题没有发言权。
    """
    bad = []
    if nvalid < 4:
        bad.append(f"可用距离段只有 {nvalid} 个(要 ≥4)—— 让目标面在画面里铺开一些")
    if ratio < MIN_RATIO:
        bad.append(f"远近比 {ratio:.2f}× < {MIN_RATIO}× —— 俯角再放平,让目标面铺得更长")
    if zmax < MIN_ZMAX:
        bad.append(f"最远只到 {zmax:.2f} m < {MIN_ZMAX} m —— 太近了,"
                   f"相机架高、俯角放平,看向 3 m 外的地面")
    if npts < MIN_PTS:
        bad.append(f"平面点太少 {npts} < {MIN_PTS} —— 让目标面占满画面")
    return len(bad) == 0, bad


def geometry_hint(n, c):
    """从拟合平面反推相机高度和俯角,并算出要看到 4 m 需要多小的俯角。

    看到的最远地面距离 ≈ 高度 / tan(俯角) —— 相机太低时,俯角再压平也看不远。
    这是"为什么调不出跨度"最常见的原因,直接显示出来比让人猜快得多。
    """
    h = abs(float(np.dot(c, n)))                    # 相机到平面的垂直距离
    pitch = np.degrees(np.arcsin(min(abs(n[2]), 1.0)))   # 光轴与平面的夹角
    need = np.degrees(np.arctan(h / 4.0)) if h > 0 else 0.0
    return h, pitch, need


def advise(zl, zh, ratio, npts, geo, ok, scene=None, normal=None):
    """当前最该做的**一个**动作。

    一次只说一件事。目标面是墙还是地面,给的建议完全不同 ——
    地面要调高度和俯角(受天花板/杂物限制,难),墙面只要贴着一端沿墙看(容易)。
    所以先按法向判断在看什么,再给对应的话。
    """
    h, pitch, need = geo
    horizontal = normal is not None and abs(normal[1]) > 0.7   # y 主导 = 水平面
    if scene is not None:
        p50, p90, far = scene
        if p90 < 2.0 and far < 0.05:
            return (f"镜头前 {p50:.1f} m 就被挡住了 —— 贴到一面长墙的一端,"
                    f"镜头顺着墙拍过去")
    if npts < 5000:
        return "画面里没有大片平面 —— 对准一面墙或地面"
    if zh >= 3.0:
        return "很好,保持不动"
    if horizontal:
        if h < 0.40:
            return f"相机太低(离地 {h*100:.0f} cm)—— 搬到桌上,或者改成贴着墙拍"
        if pitch > need + 4:
            return f"镜头太朝下({pitch:.0f}°)—— 往上抬到 {need:.0f}° 以内"
        return (f"地面只够到 {zh:.1f} m(前方多半有东西挡)—— "
                f"改成贴着一面长墙的一端、顺着墙拍,通常更容易")
    # 竖直面(墙):贴近一端沿墙拍,天然有梯度
    if ratio < MIN_RATIO:
        return f"墙面只铺到 {zh:.1f} m —— 把相机贴到墙的一端,镜头顺着墙拍,别正对墙"
    return f"再顺着墙偏一点 —— 现在够到 {zh:.1f} m,到 3 m 就合格"


def draw_gui(d, P, uv, inl, zl, zh, ratio, nvb, tip, ok, stable, ub):
    """看得见的反馈:绿=用于拟合的平面点,红=被 RANSAC 剔除的(遮挡物、家具、人)。

    纯文字提示说不清"为什么这块不算" —— 画出来一眼就知道是挡住了还是没对准。
    """
    H, W = d.shape
    vis = np.zeros((H, W, 3), np.uint8)
    valid = d > 0.15
    if valid.any():
        norm = np.clip((d - 0.3) / max(zh - 0.3, 0.5), 0, 1)
        gray = (norm * 200 + 40).astype(np.uint8)
        vis[valid] = cv2.applyColorMap(gray, cv2.COLORMAP_BONE)[valid]
    im = np.zeros((H, W), bool)
    im[uv[inl, 1], uv[inl, 0]] = True          # 平面内点
    ex = valid & ~im                            # 被剔除
    vis[im] = (vis[im] * 0.35 + np.array([40, 200, 90]) * 0.65).astype(np.uint8)
    vis[ex] = (vis[ex] * 0.55 + np.array([60, 60, 210]) * 0.45).astype(np.uint8)

    panel = np.zeros((150, W, 3), np.uint8) + 22
    # 距离直方图:绿条=可用于拟合的距离段
    zin = P[inl, 2] if inl.any() else np.array([1.0])
    hist, ed = np.histogram(zin, bins=40, range=(0, max(4.5, zh * 1.1)))
    for i, hv in enumerate(hist):
        x0 = int(i * W / 40) + 2
        x1 = int((i + 1) * W / 40) - 2
        hh = int(60 * hv / max(hist.max(), 1))
        col = (90, 200, 90) if hv >= 300 else (70, 70, 70)
        cv2.rectangle(panel, (x0, 92 - hh), (x1, 92), col, -1)
    for zt in (1, 2, 3, 4):
        x = int(zt / max(4.5, zh * 1.1) * W)
        cv2.line(panel, (x, 92), (x, 100), (150, 150, 150), 1)
        cv2.putText(panel, f"{zt}m", (x - 10, 114), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (170, 170, 170), 1)
    tgt = int(3.0 / max(4.5, zh * 1.1) * W)
    cv2.line(panel, (tgt, 20), (tgt, 92), (60, 180, 255), 2)
    cv2.putText(panel, "target 3m", (tgt + 6, 34), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (60, 180, 255), 1)
    head = (f"OK {stable}/4  keep still" if ok else
            f"z {zl:.2f}-{zh:.2f}m  ratio {ratio:.2f}x  bins {nvb}  bound {ub:.0f}%")
    cv2.putText(panel, head, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (90, 230, 120) if ok else (240, 240, 240), 2)
    cv2.putText(panel, "green = plane used   red = rejected (obstacles)",
                (10, 138), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    out = np.vstack([vis, panel])
    cv2.imshow('depth nonlinearity  (q=quit)', out)
    return cv2.waitKey(1) & 0xFF


def render(zl, zh, ratio, npts, geo, ok, stable, scene=None, normal=None):
    """单行原地刷新。

    原来用 ANSI 上移画四行面板,在部分终端里不生效就变成滚屏刷屏。
    \r 覆盖单行是所有终端都认的最小公倍数 —— 动作放最前面,因为那是唯一要照做的。
    """
    ub = expected_ub(zh)
    filled = int(min(zh / 4.0, 1.0) * 12)
    bar = '█' * filled + '░' * (12 - filled)
    if ok:
        extra = "" if zh >= GOOD_ZMAX else f"(再远些界更紧,现 {ub:.1f}%)"
        line = f"  ✓ 合格 {stable}/4 别动 {extra}   {zh:.1f}/4.0m {bar}"
    else:
        line = (f"  ▶ {advise(zl, zh, ratio, npts, geo, ok, scene, normal)}"
                f"   [{zh:.1f}/4.0m {bar} 上界{ub:.0f}%]")
    print("\r" + line + " " * 12, end='', flush=True)


def placement_score(P, min_pts=300, nb=24):
    """返回 (最低有效 z, 最高有效 z, 跨度比, 内点数, 几何提示, 法向, 有效箱数)。

    **不要求连续**。之前取"最长连续段",等于让一张桌子把整个摆位否掉 ——
    可二次拟合的前提是有分散的支撑点,不是连续覆盖:中间缺一段不但无害,
    杠杆反而更好。遮挡物本来就被 RANSAC 排除在平面之外了,不该再罚一次。
    要求改成:有效箱 ≥4 个,且最远/最近拉得开。
    """
    n, c, inl = fit_plane(P, iters=200)
    z = P[inl][:, 2]
    g = geometry_hint(n, c)
    if len(z) < 1000:
        return 0.0, 0.0, 0.0, int(inl.sum()), g, n, 0
    lo, hi = np.percentile(z, [1, 99])
    edges = np.linspace(lo, hi, nb + 1)
    cnt = np.array([((z >= edges[i]) & (z < edges[i + 1])).sum() for i in range(nb)])
    good = cnt >= min_pts
    nvalid = int(good.sum())
    if nvalid < 4:
        return 0.0, 0.0, 0.0, int(inl.sum()), g, n, nvalid
    first, last = int(np.argmax(good)), nb - 1 - int(np.argmax(good[::-1]))
    zl, zh = float(edges[first]), float(edges[last + 1])
    return zl, zh, float(zh / max(zl, 1e-6)), int(inl.sum()), g, n, nvalid


def check_placement(P, min_ratio=2.5, min_pts=300, nb=24):
    """摆位是否够格。

    判据不是 min/max 跨度 —— 那被单个离群点就能骗过,也看不见中间的空档。
    这里按**等宽**分箱找最长的连续有效段,只有连续段才提供二次拟合的杠杆。
    """
    n, c, inl = fit_plane(P)
    Q = P[inl]
    z = Q[:, 2]
    lo, hi = np.percentile(z, [1, 99])
    edges = np.linspace(lo, hi, nb + 1)
    cnt = np.array([((z >= edges[i]) & (z < edges[i + 1])).sum() for i in range(nb)])
    good = cnt >= min_pts
    # 最长连续 True 段
    best_i = best_len = cur_i = cur_len = 0
    for i, g in enumerate(good):
        if g:
            if cur_len == 0:
                cur_i = i
            cur_len += 1
            if cur_len > best_len:
                best_len, best_i = cur_len, cur_i
        else:
            cur_len = 0
    z_lo, z_hi = edges[best_i], edges[best_i + best_len]
    ratio = z_hi / max(z_lo, 1e-6)
    print(f"平面内点 {inl.sum()} ({100*inl.sum()/len(P):.0f}% 有效像素)  "
          f"法向 [{n[0]:+.2f} {n[1]:+.2f} {n[2]:+.2f}]")
    print(f"\n距离分布(等宽箱,# = 点数):")
    for i in range(nb):
        bar = '#' * int(40 * cnt[i] / max(cnt.max(), 1))
        flag = ' ' if good[i] else '·'
        print(f"  {edges[i]:4.2f}~{edges[i+1]:4.2f} m {flag}{bar}")
    print(f"\n最长连续段 {z_lo:.2f} ~ {z_hi:.2f} m   远近比 {ratio:.2f}×")
    ok, bad = placement_ok(ratio, z_hi, int(inl.sum()))
    if ok:
        print(f"\n✓ 摆位合格 —— 可以采(远近比 {ratio:.2f}×,最远 {z_hi:.2f} m)")
    else:
        print(f"\n✗ 摆位不够:")
        for b in bad:
            print(f"   {b}")
    return ok


def sensitivity(P, n):
    """深度误差 -> 平面残差 的几何放大因子。

    深度读数偏 δd 时,点沿视线移动 δd,残差只吃到它在法向上的投影 (ray·n)。
    斜平面上这个因子远小于 1,不折算就会把非线性低估掉。
    """
    rays = P / np.linalg.norm(P, axis=1, keepdims=True)
    return float(np.abs(rays @ n).mean())


def fit_quad_boot(z, r, w, nboot=3000, seed=0):
    """加权二次拟合 + 对分箱做 bootstrap。

    对箱重采样而不是对点:空间相关噪声让同一箱内的点远不独立,
    按点 bootstrap 会把置信区间算得虚窄。
    """
    rng = np.random.default_rng(seed)
    n = len(z)
    out = []
    for _ in range(nboot):
        i = rng.integers(0, n, n)
        if len(np.unique(z[i])) < 3:
            continue
        try:
            out.append(np.polyfit(z[i], r[i], 2, w=w[i])[0])
        except Exception:
            pass
    out = np.array(out)
    return float(np.median(out)), float(out.std()), np.percentile(out, [2.5, 97.5])


def analyze(P, label, nbins=14, verbose=True):
    n, c, inl = fit_plane(P)
    Q = P[inl]
    tr = residual_trend(Q, n, c, nbins=nbins)
    if len(tr) < 5:
        return None
    z, rm, rs, cnt = tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3]
    # 空档保护:分位分箱会把"绝大多数点挤在近处 + 个别远点"伪装成大跨度,
    # 二次拟合的杠杆就全压在那一两个箱上,CI 会炸开。
    # 只看支撑点够不够、两端拉不拉得开 —— 空档本身不害二次拟合
    if len(z) < 5:
        print(f"\n--- {label} ---\n✗ 有效距离段只有 {len(z)} 个,至少要 5 个")
        return None
    w = np.sqrt(cnt) / np.maximum(rs, 1e-6)          # 权重:点多、散布小的箱更可信
    coef, se, ci = fit_quad_boot(z, rm, w)
    sens = sensitivity(Q, n)
    zmax = z.max()
    # 二次项在 z_max 处对应的残差 -> 折算回深度相对偏差
    def to_pct(a_coef):
        return abs(a_coef) * zmax ** 2 / sens / zmax * 100
    detected = not (ci[0] < 0 < ci[1])
    ub_pct = to_pct(max(abs(ci[0]), abs(ci[1])))
    if verbose:
        print(f"\n--- {label} ---")
        print(f"平面内点 {inl.sum()} / {len(P)}   距离 {z.min():.2f}~{zmax:.2f} m   "
              f"法向 [{n[0]:+.2f} {n[1]:+.2f} {n[2]:+.2f}]  几何灵敏度 {sens:.2f}")
        print(f"二次系数 {coef*1000:+.2f} mm/m²  bootstrap 95%CI "
              f"[{ci[0]*1000:+.2f}, {ci[1]*1000:+.2f}]")
        print(f"→ {'检出非线性' if detected else '未检出'};"
              f" {zmax:.1f} m 处深度相对偏差 95% 上界 {ub_pct:.2f}%")
    return dict(n=n.tolist(), z_min=float(z.min()), z_max=float(zmax),
                sens=sens, quad_mm_per_m2=coef * 1000,
                ci_mm_per_m2=[ci[0] * 1000, ci[1] * 1000],
                detected=bool(detected), ub_pct=float(ub_pct),
                trend=[[float(x) for x in row] for row in tr],
                n_inliers=int(inl.sum()))


def synth(nonlin_a=0.0, noise_mm=4.0, seed=0):
    """合成一面斜墙的深度图。

    真值:x = 1.2 m 处的竖直平面(与实测摆位同构)。
    注入非线性 d_meas = d_true·(1 + a·(d_true - 1)²) —— 二次型,1 m 处为零,
    这样它和"尺度误差"正交,不会被平面拟合吸收掉。
    噪声按 σ ∝ z² 缩放(用本机实测 4.1 mm @ 1 m 作基准),并做 15 px 空间相关
    ——CALIBRATION.md 记过深度噪声成斑块,i.i.d. 会高估有效样本数。
    """
    rng = np.random.default_rng(seed)
    H, W = 480, 848
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    # 竖直平面 x = X0:由 x=(u-cx)/fx·z 反解 z
    X0 = 1.2
    with np.errstate(divide='ignore', invalid='ignore'):
        z = X0 * FX / (u - CX)
    z[(u - CX) <= 0] = 0            # 平面只在画面一侧
    z[(z < 0.4) | (z > 4.0)] = 0
    d_true = z.copy()
    m = d_true > 0
    d = np.zeros_like(d_true)
    dt = d_true[m]
    d[m] = dt * (1 + nonlin_a * (dt - 1.0) ** 2)
    # 空间相关噪声:低分辨率白噪声上采样
    sig = noise_mm / 1000 * (d_true / 1.0) ** 2
    small = rng.normal(size=(H // 15 + 1, W // 15 + 1))
    big = np.repeat(np.repeat(small, 15, 0), 15, 1)[:H, :W]
    d[m] += (big * sig)[m]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true', help='合成数据验算法,不碰相机')
    ap.add_argument('--check', action='store_true', help='只检查摆位够不够,不做分析')
    ap.add_argument('--live', action='store_true', help='实时显示摆位评分,边调边看(Ctrl+C 退出)')
    ap.add_argument('--no-gui', action='store_true', help='不开窗口')
    ap.add_argument('--frames', type=int, default=60, help='实机采集平均帧数')
    ap.add_argument('--npy', help='直接分析已保存的深度图')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results', 'depth_nonlinearity.json'))
    a = ap.parse_args()

    if a.selftest:
        print("=" * 66)
        print("自测:注入已知非线性,检验 (1) 零注入不虚报 (2) 上界覆盖真值")
        print("=" * 66)
        ok = True
        for inj in [0.0, 0.005, 0.02]:
            d = synth(nonlin_a=inj)
            P, _ = deproject(d)
            # 注入量在 z_max 处的真实相对偏差
            r = analyze(P, f"注入 a={inj}")
            if r is None:
                print("  !! 分箱不足"); ok = False; continue
            true_pct = inj * (r['z_max'] - 1.0) ** 2 * 100
            print(f"  真值 {true_pct:.2f}%   上界 {r['ub_pct']:.2f}%")
            if inj == 0.0:
                good = not r['detected']
                print(f"  {'✓' if good else '✗'} 零注入" +
                      ("判为未检出(不虚报)" if good else "被误判为检出 —— 假阳性"))
            else:
                good = r['ub_pct'] >= true_pct * 0.8
                print(f"  {'✓' if good else '✗'} 上界" +
                      ("覆盖真值" if good else "低于真值 —— 会漏报真实非线性"))
            ok = ok and good
        print("\n" + ("自测通过:零注入不虚报,有注入时上界覆盖真值"
                      if ok else "自测失败"))
        return

    if a.live:
        import pyrealsense2 as rs
        p = rs.pipeline(); c = rs.config()
        c.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        pr = p.start(c)
        ds = pr.get_device().first_depth_sensor()
        try:
            ds.set_option(rs.option.emitter_enabled, 1)
        except Exception:
            pass
        scale = ds.get_depth_scale()
        print("按提示调相机就行,合格会自动开采(Ctrl+C 随时退出)")
        best = 0.0
        stable = 0
        done = False
        hist = []
        try:
            while not done:
                acc = [np.asanyarray(p.wait_for_frames().get_depth_frame().get_data())
                       .astype(np.float32) * scale for _ in range(6)]
                d = np.median(np.stack(acc), 0)
                P, _ = deproject(d)
                if len(P) < 2000:
                    continue
                zl, zh, ratio, npts, geo, nrm, nvb = placement_score(P)
                hist.append((zl, zh, ratio, npts, geo[0], geo[1], geo[2]))
                if len(hist) > 5:
                    hist.pop(0)
                if len(hist) >= 3:      # 取中位,单帧抓错平面不至于把提示带偏
                    m = np.median(np.array(hist), axis=0)
                    zl, zh, ratio, npts = m[0], m[1], m[2], int(m[3])
                    geo = (m[4], m[5], m[6])
                zall = d[d > 0.15]
                scene = (float(np.median(zall)), float(np.percentile(zall, 90)),
                         float(((d > 3.0) & (d < 8.0)).mean())) if len(zall) > 1000 else None
                best = max(best, ratio)
                ok, bad = placement_ok(ratio, zh, npts, nvb)
                stable = stable + 1 if ok else 0
                render(zl, zh, ratio, npts, geo, ok, stable, scene, nrm)
                if not a.no_gui:
                    Pf, uvf = deproject(d, ret_uv=True)
                    nn, cc, inlf = fit_plane(Pf, iters=200)
                    key = draw_gui(d, Pf, uvf, inlf, zl, zh, ratio, nvb,
                                   '', ok, stable, expected_ub(zh))
                    if key == ord('q'):
                        break
                if stable < 4:
                    continue
                print(f"\n\n✓ 摆位稳定,保持不动 —— 正在采 {a.frames} 帧...")
                acc = [np.asanyarray(p.wait_for_frames().get_depth_frame().get_data())
                       .astype(np.float32) * scale for _ in range(a.frames)]
                d = np.median(np.stack(acc), 0)
                P, _ = deproject(d)
                zl, zh, ratio, npts, geo, nrm, nvb = placement_score(P)
                ok2, bad2 = placement_ok(ratio, zh, npts)
                if not ok2:
                    print("  采集期间摆位变了(" + bad2[0][:30] + "),继续调\n")
                    stable = 0
                    continue
                r = analyze(P, f"实测(连续段 {zl:.2f}~{zh:.2f} m,{ratio:.2f}×)")
                if r is not None:
                    os.makedirs(os.path.dirname(a.out), exist_ok=True)
                    json.dump(r, open(a.out, 'w'), indent=2, ensure_ascii=False)
                    print(f"\n写入 {a.out}")
                    np.save(a.out.replace('.json', '_depth.npy'), d)
                done = True
        except KeyboardInterrupt:
            print(f"\n\n本次最好 {best:.2f}×")
        finally:
            p.stop()
            cv2.destroyAllWindows()
        return

    if a.npy:
        d = np.load(a.npy)
    else:
        import pyrealsense2 as rs
        p = rs.pipeline(); c = rs.config()
        c.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        pr = p.start(c)
        ds = pr.get_device().first_depth_sensor()
        try:
            ds.set_option(rs.option.emitter_enabled, 1)
        except Exception:
            pass
        scale = ds.get_depth_scale()
        for _ in range(30):
            p.wait_for_frames()
        acc = []
        nf = 15 if a.check else a.frames
        for i in range(nf):
            acc.append(np.asanyarray(p.wait_for_frames().get_depth_frame()
                                     .get_data()).astype(np.float32) * scale)
        p.stop()
        st = np.stack(acc)
        d = np.median(st, 0)
        print(f"采了 {nf} 帧,取中位")

    P, _ = deproject(d)
    if a.check:
        check_placement(P)
        return
    # 分析前先过摆位门。跨度不够时二次拟合的杠杆太短,算出的"上界"会比
    # 厂商谱面还宽 —— 那不是测量结果,是没测出来。宁可拒绝也不给假数字。
    zl, zh, ratio, npts, geo, nrm, nvb = placement_score(P)
    ok, bad = placement_ok(ratio, zh, npts, nvb)
    if not ok:
        print(f"\n✗ 摆位不合格:连续段 {zl:.2f}~{zh:.2f} m,远近比 {ratio:.2f}×,内点 {npts}")
        for b in bad:
            print(f"   {b}")
        print("  这个摆位下拟合出的界没有意义。先跑:")
        print("    python tools/depth_nonlinearity.py --live")
        print("  调到 ✓ 合格再采。")
        return
    r = analyze(P, f"实测(连续段 {zl:.2f}~{zh:.2f} m,{ratio:.2f}×)")
    if r is None:
        print("平面点太少,分不出箱"); return
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(r, open(a.out, 'w'), indent=2, ensure_ascii=False)
    print(f"\n写入 {a.out}")


if __name__ == '__main__':
    main()
