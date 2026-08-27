#!/usr/bin/env python3
"""阶段 3:深度质量与 depth->color 对齐核查。

这一步跟前面不同 —— 它不是"标定",是"核查"。
D400 的深度外参出厂就烧在设备里,而且 depth 图本身定义在左 IR 相机坐标系
(check_device.py 已经验证过:IR左->COLOR 与 DEPTH->COLOR 的外参完全相同)。
所以没什么可标的。真正会咬人的是另外三件事:

  1. 深度噪声有多大 —— 决定你的点云能不能用
  2. 平面是不是真的被测成平面 —— 系统性弯曲说明深度有畸变
  3. 深度值是不是有恒定偏置 —— 整体测远或测近

用法:
  python check_depth.py                 # 实时预览,空格采样,q 退出
  python check_depth.py --shots 3       # 采够 3 组自动结束
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pyrealsense2 as rs


def pick_profile(dev, stream, fmt, idx=0):
    cands = []
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            try:
                vp = p.as_video_stream_profile()
            except Exception:
                continue
            if p.stream_type() == stream and p.stream_index() == idx and p.format() == fmt:
                cands.append((vp.width(), vp.height(), p.fps()))
    if not cands:
        return None
    for res in [(848, 480), (640, 480), (1280, 720)]:
        got = [c for c in cands if (c[0], c[1]) == res]
        if got:
            return max(got, key=lambda c: min(c[2], 30))
    return max(cands, key=lambda c: (c[0] * c[1], c[2]))


def fit_plane_ransac(P, thresh=0.006, iters=200, rng=None):
    """P: (N,3) 点云。返回 (n, d, inlier_mask),平面为 n·x + d = 0,|n|=1"""
    rng = rng or np.random.default_rng(0)
    best = (None, None, None, -1)
    N = len(P)
    if N < 50:
        return None, None, None
    for _ in range(iters):
        i = rng.choice(N, 3, replace=False)
        a, b, c = P[i]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        d = -n @ a
        r = np.abs(P @ n + d)
        cnt = int((r < thresh).sum())
        if cnt > best[3]:
            best = (n, d, r < thresh, cnt)
    n, d, m, _ = best
    if n is None:
        return None, None, None
    # 用 inlier 做最小二乘精修
    Q = P[m]
    ctr = Q.mean(0)
    _, _, vt = np.linalg.svd(Q - ctr, full_matrices=False)
    n = vt[2] / np.linalg.norm(vt[2])
    d = -n @ ctr
    return n, d, np.abs(P @ n + d) < thresh


def analyze(depth_frames, intr, scale, roi):
    """depth_frames: list of (H,W) uint16。返回统计字典"""
    x0, y0, x1, y1 = roi
    stack = np.stack(depth_frames).astype(np.float32) * scale
    stack[stack <= 0] = np.nan
    sub = stack[:, y0:y1, x0:x1]

    valid_ratio = float(np.isfinite(sub).mean())
    import warnings
    with np.errstate(invalid='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # 整列全 NaN 的像素属正常
        mean_d = np.nanmean(sub, axis=0)
        temporal_std = np.nanstd(sub, axis=0)

    ys, xs = np.mgrid[y0:y1, x0:x1]
    z = mean_d
    ok = np.isfinite(z) & (z > 0.1)
    X = (xs[ok] - intr.ppx) / intr.fx * z[ok]
    Y = (ys[ok] - intr.ppy) / intr.fy * z[ok]
    P = np.stack([X, Y, z[ok]], 1)

    out = dict(valid_ratio=valid_ratio, n_pts=len(P),
               temporal_std=float(np.nanmedian(temporal_std)),
               dist=float(np.nanmedian(z[ok])))
    if len(P) < 200:
        out['ok'] = False
        return out, None, None, ok

    n, d, inl = fit_plane_ransac(P)
    if n is None:
        out['ok'] = False
        return out, None, None, ok
    resid = P @ n + d
    out.update(ok=True,
               inlier_ratio=float(inl.mean()),
               plane_rms=float(np.sqrt((resid[inl] ** 2).mean())),
               plane_p95=float(np.percentile(np.abs(resid[inl]), 95)),
               tilt_deg=float(np.degrees(np.arccos(min(1.0, abs(n[2]))))))
    # 残差图(用于看系统性弯曲)
    rmap = np.full(z.shape, np.nan, np.float32)
    rmap[ok] = resid
    return out, rmap, n, ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shots', type=int, default=0, help='采够几组自动结束(0=手动)')
    ap.add_argument('--avg', type=int, default=30, help='每组平均多少帧')
    ap.add_argument('--roi', type=float, default=0.5, help='中心 ROI 占画面比例')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'depth_check'))
    args = ap.parse_args()

    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        print('没有检测到相机'); sys.exit(1)
    dev = ctx.query_devices()[0]
    dp = pick_profile(dev, rs.stream.depth, rs.format.z16)
    cp = pick_profile(dev, rs.stream.color, rs.format.bgr8)
    if dp is None:
        print('没有可用的 depth 流'); sys.exit(1)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, dp[0], dp[1], rs.format.z16, dp[2])
    if cp:
        cfg.enable_stream(rs.stream.color, cp[0], cp[1], rs.format.bgr8, cp[2])
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    ds = prof.get_device().first_depth_sensor()
    scale = ds.get_depth_scale()
    # 深度核查必须开发射器 —— 这跟前面标定相反,深度就是靠散斑算出来的
    for s in prof.get_device().query_sensors():
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 1)
    intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    W, H = intr.width, intr.height
    rw, rh = int(W * args.roi), int(H * args.roi)
    roi = ((W - rw) // 2, (H - rh) // 2, (W + rw) // 2, (H + rh) // 2)
    print(f'depth {W}x{H}@{dp[2]}   深度单位 {scale*1000:.3f} mm/LSB')
    print(f'ROI {roi}   每组平均 {args.avg} 帧')
    print('\n对准目标,[空格]采样  [q]结束\n')

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    win = 'depth check   空格=采样  q=退出'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    buf, shots = [], []
    try:
        while True:
            fs = pipe.wait_for_frames()
            df = fs.get_depth_frame()
            if not df:
                continue
            dimg = np.asanyarray(df.get_data())
            buf.append(dimg.copy())
            if len(buf) > args.avg:
                buf.pop(0)

            vis = cv2.applyColorMap(
                cv2.convertScaleAbs(dimg, alpha=255.0 / max(1, dimg.max())), cv2.COLORMAP_TURBO)
            vis[dimg == 0] = (40, 40, 40)
            x0, y0, x1, y1 = roi
            sub = dimg[y0:y1, x0:x1]
            vr = float((sub > 0).mean())
            dmed = float(np.median(sub[sub > 0]) * scale) if (sub > 0).any() else 0.0
            cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 255, 255), 2)

            if vr < 0.6:
                tip, col = 'ROI 内深度缺失严重 -> 目标反光/太远/太暗', (0, 80, 255)
            elif dmed < 0.25 or dmed > 3.0:
                tip, col = f'距离 {dmed:.2f}m 超出建议范围 0.3-2.0m', (0, 165, 255)
            else:
                tip, col = f'OK  距离 {dmed:.2f}m', (0, 255, 0)
            cv2.rectangle(vis, (0, H - 52), (W, H), (0, 0, 0), -1)
            cv2.putText(vis, f'有效 {vr*100:5.1f}%   {tip}', (10, H - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            cv2.putText(vis, f'已采 {len(shots)} 组   缓冲 {len(buf)}/{args.avg}', (10, H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord(' ') and len(buf) >= args.avg:
                st, rmap, nrm, okmask = analyze(buf, intr, scale, roi)
                shots.append((st, rmap))
                if st.get('ok'):
                    print(f"[{len(shots)}] 距离 {st['dist']:.3f}m  有效 {st['valid_ratio']*100:.1f}%  "
                          f"平面RMS {st['plane_rms']*1000:6.2f}mm  p95 {st['plane_p95']*1000:6.2f}mm  "
                          f"时域噪声 {st['temporal_std']*1000:5.2f}mm  倾角 {st['tilt_deg']:.1f}deg")
                else:
                    print(f"[{len(shots)}] 有效点太少,无法拟合平面(有效 {st['valid_ratio']*100:.1f}%)")
                if args.shots and len(shots) >= args.shots:
                    break
    finally:
        pipe.stop(); cv2.destroyAllWindows()

    if not shots:
        print('\n没有采样。'); return
    good = [s for s in shots if s[0].get('ok')]
    if not good:
        print('\n所有采样都无法拟合平面 —— 换一个哑光、非反光的平整目标'); return

    print('\n' + '=' * 66)
    print('汇总')
    print('=' * 66)
    for i, (st, _) in enumerate(good, 1):
        rel = st['plane_rms'] / st['dist'] * 100
        print(f"  [{i}] {st['dist']:.3f}m   平面RMS {st['plane_rms']*1000:6.2f}mm "
              f"({rel:.3f}% of range)   时域 {st['temporal_std']*1000:5.2f}mm   "
              f"inlier {st['inlier_ratio']*100:.1f}%")
    # 数值落盘 —— 只出图会让结果留在终端里丢掉
    import json
    rec = dict(depth_resolution=[int(W), int(H)], depth_scale_m=float(scale),
               roi=[int(v) for v in roi], avg_frames=int(args.avg),
               shots=[{k: (float(v) if isinstance(v, (int, float, np.floating)) else v)
                       for k, v in st.items()} for st, _ in good])
    with open(args.out + '.json', 'w') as f:
        json.dump(rec, f, indent=2, ensure_ascii=False)
    print(f'数值 -> {args.out}.json')

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
    n = len(good)
    fig, axes = plt.subplots(2, n, figsize=(5.2 * n, 8), dpi=110, squeeze=False)
    for i, (st, rmap) in enumerate(good):
        ax = axes[0][i]
        v = np.nanpercentile(np.abs(rmap), 98)
        im = ax.imshow(rmap * 1000, cmap='RdBu_r', vmin=-v * 1000, vmax=v * 1000)
        ax.set_title(f"{st['dist']:.2f} m  平面残差 [mm]")
        ax.set_xticks([]); ax.set_yticks([])
        plt.colorbar(im, ax=ax, fraction=0.046)
        ax = axes[1][i]
        r = rmap[np.isfinite(rmap)] * 1000
        ax.hist(r, bins=80, color='#4c78a8')
        ax.axvline(0, color='k', lw=0.8)
        ax.set_title(f"RMS {st['plane_rms']*1000:.2f}mm   p95 {st['plane_p95']*1000:.2f}mm")
        ax.set_xlabel('残差 [mm]')
    plt.tight_layout()
    png = args.out + '.png'
    plt.savefig(png, bbox_inches='tight')
    print(f'\n图 -> {png}')
    print('\n看残差图:随机噪点=正常;出现同心圆/条纹/整片偏色=深度有系统性畸变')


if __name__ == '__main__':
    main()
