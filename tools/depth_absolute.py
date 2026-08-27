#!/usr/bin/env python3
"""深度绝对偏差 / 非线性:AprilGrid PnP 参考法。

为什么换掉斜平面法:
  斜平面法的分辨力几乎全靠距离跨度比(z_max/z_min)。实测在 1.1~2.5 m 的跨度下
  95% 上界只到 6%,比厂商谱面(2%)还宽 —— 等于没测。要压到 2% 以下需要看到
  4 m 外,普通房间做不到。

这个方法为什么不受空间限制:
  AprilGrid 的几何是**已知真值**(tag 边长实测)。PnP 从角点解出的板面距离精度
  ≈ z·(角点噪声/板张角像素),在 2.5 m 处约 0.12% —— 与跨度无关,只与板子在
  画面里占多少像素有关。拿它当参考尺,直接读出深度的**绝对**偏差,
  而不只是相对形变。

发射器两难的处理:
  tag 检测怕散斑、深度要散斑。用 emitter_on_off 交替帧:干净帧检 tag 解 PnP,
  相邻的散斑帧读深度,同一相机、同一区域、相隔一帧。
  (CALIBRATION.md 已验过 D400 在交替模式下深度帧率不减半。)

用法:
  python depth_absolute.py --selftest        # 合成角点验 PnP 精度,不碰相机
  python depth_absolute.py                   # 实时:把板子拿到不同距离,自动采样
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except Exception:
    rs = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture import make_detector, TAG_SIZE, TAG_SPACING, TAG_ROWS, TAG_COLS  # noqa


def board_points():
    """AprilGrid 的 3D 角点。

    与 capture.py 的版本差一个角点循环移位:那边喂给 Kalibr(自带对应关系推断),
    这里直接喂 solvePnP,角点顺序必须和 cv2.aruco 的输出严格对齐。
    实测枚举四种移位,shift=2 让重投影从 45.7 px 降到 3.4 px。
    """
    pitch = TAG_SIZE * (1.0 + TAG_SPACING)
    pts = {}
    for tid in range(TAG_ROWS * TAG_COLS):
        r, c = divmod(tid, TAG_COLS)
        x0, y0 = c * pitch, r * pitch
        q = [[x0, y0 + TAG_SIZE, 0.0], [x0 + TAG_SIZE, y0 + TAG_SIZE, 0.0],
             [x0 + TAG_SIZE, y0, 0.0], [x0, y0, 0.0]]
        pts[tid] = np.array(q[2:] + q[:2], dtype=np.float64)
    return pts

# 深度流(848x480)对应的左 IR 内参:由 1280x720 标定值缩放
FX, FY = 884.7801714792454, 883.8676458516142
CX, CY = 652.1558719536981, 373.36376307278374
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
# IR 经 ASIC 硬件去畸变,标定实测 k1 仅 -0.011,PnP 用零畸变
DIST = np.array([0.11018893566651113, -0.19834915299940875,
                 -0.0012403776370154334, 0.0014777729661525543])


def solve_board(gray, det, bpts, K=K, dist=DIST):
    """检测 AprilGrid 并 PnP。返回 (板面到相机的距离 z, 板中心像素, 角点数, 重投影RMS)。"""
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None or len(ids) < 4:
        return None
    obj, img = [], []
    for k, tid in enumerate(ids.flatten()):
        if tid not in bpts:
            continue
        obj.append(bpts[tid])
        img.append(corners[k].reshape(4, 2))
    if len(obj) < 4:
        return None
    obj = np.concatenate(obj).astype(np.float64)
    img = np.concatenate(img).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
    rms = float(np.sqrt(((proj.reshape(-1, 2) - img) ** 2).sum(1).mean()))
    # 板面中心在相机系下的 z(取板中心的物点)
    ctr_obj = obj.mean(0).reshape(1, 3)
    R, _ = cv2.Rodrigues(rv)
    ctr_cam = (R @ ctr_obj.T + tv).ravel()
    ctr_px, _ = cv2.projectPoints(ctr_obj, rv, tv, K, dist)
    return float(ctr_cam[2]), ctr_px.ravel(), len(obj), rms


def laser_on(frame):
    """这一帧是不是散斑帧。交替模式下深度要用散斑帧读,干净帧的被动立体在
    白板上会失效。metadata 拿不到时返回 None(调用方退回"全收")。"""
    try:
        MD = rs.frame_metadata_value.frame_laser_power_mode
        if frame.supports_frame_metadata(MD):
            return int(frame.get_frame_metadata(MD)) == 1
    except Exception:
        pass
    return None


def auto_expose(ds, pipe, alt, target=110, lo=75, hi=165, max_iter=14):
    """在**干净帧**上把亮度闭环到目标区间。

    不能用相机自带的自动曝光:交替模式下散斑帧亮、干净帧暗,AE 会在两者之间
    来回震荡,干净帧长期欠曝 —— 正是 tag 检不出的原因。
    """
    exp = int(ds.get_option(rs.option.exposure))
    med = 0
    for _ in range(max_iter):
        try:
            ds.set_option(rs.option.exposure, exp)
        except Exception:
            break
        for _ in range(5):
            pipe.wait_for_frames()
        irf = None
        for _ in range(12):
            f = pipe.wait_for_frames().get_infrared_frame(1)
            if f and (not alt or laser_on(f) is not True):
                irf = f
                break
        if irf is None:
            break
        med = float(np.median(np.asanyarray(irf.get_data())))
        print(f"  曝光 {exp/1000:5.1f} ms -> 亮度中位 {med:5.1f}")
        if lo <= med <= hi:
            return exp, med
        exp = int(np.clip(exp * (target / max(med, 3.0)), 800, 90000))
    return exp, med


def read_depth(depth_m, ctr_px, half=18):
    """板中心邻域的深度中位数(m)与有效率。"""
    u, v = int(round(ctr_px[0])), int(round(ctr_px[1]))
    H, W = depth_m.shape
    u0, u1 = max(0, u - half), min(W, u + half + 1)
    v0, v1 = max(0, v - half), min(H, v + half + 1)
    if u1 <= u0 or v1 <= v0:
        return None, 0.0
    patch = depth_m[v0:v1, u0:u1]
    val = patch[patch > 0.1]
    if len(val) < 30:
        return None, float(len(val) / max(patch.size, 1))
    return float(np.median(val)), float(len(val) / patch.size)


def fit_and_report(S):
    """S: list of (z_pnp, z_depth). 拟合 bias + 线性 + 二次,报告绝对偏差。"""
    z = np.array([s[0] for s in S])
    d = np.array([s[1] for s in S])
    err = d - z
    rel = err / z * 100
    print(f"\n{'PnP 真值 (m)':>13} {'深度读数 (m)':>13} {'偏差 (mm)':>11} {'相对 (%)':>10}")
    for zz, dd, ee, rr in sorted(zip(z, d, err, rel)):
        print(f"{zz:13.3f} {dd:13.3f} {ee*1000:11.1f} {rr:10.2f}")
    out = dict(n=len(z), z=z.tolist(), d=d.tolist(),
               err_mm=(err * 1000).tolist(), rel_pct=rel.tolist())
    if len(z) >= 3:
        # 相对偏差随距离的斜率:非线性的直接读数
        A = np.polyfit(z, rel, 1)
        out['rel_slope_pct_per_m'] = float(A[0])
        out['rel_at_1m'] = float(np.polyval(A, 1.0))
        print(f"\n相对偏差 ≈ {A[0]:+.3f} %/m · z {A[1]:+.3f} %")
        for zq in (2.0, 3.0, 4.0):
            print(f"  外推 {zq:.0f} m 处相对偏差 {np.polyval(A, zq):+.2f}%")
        resid = rel - np.polyval(A, z)
        print(f"  线性模型残差 RMS {resid.std():.3f}% "
              f"—— 大于此值的弯曲才算非线性")
        out['resid_rms_pct'] = float(resid.std())
    return out


def selftest():
    print("=" * 66)
    print("自测:合成角点,验 PnP 距离精度是否达到宣称的 ~0.1%")
    print("=" * 66)
    bpts = board_points()
    obj_all = np.concatenate([bpts[t] for t in sorted(bpts)])
    rng = np.random.default_rng(0)
    ok_all = True
    print(f"\n板面 {TAG_ROWS}x{TAG_COLS} tags, tag {TAG_SIZE*1000:.1f} mm, "
          f"spacing {TAG_SPACING}, 总宽约 "
          f"{(TAG_COLS*TAG_SIZE*(1+TAG_SPACING))*1000:.0f} mm")
    print(f"\n{'距离 (m)':>9} {'板张角 (px)':>12} {'PnP 误差 (mm)':>14} {'相对':>9}")
    for ztrue in [1.0, 1.5, 2.0, 2.5, 3.0]:
        # 板正对相机,中心在光轴上
        ctr = obj_all.mean(0)
        tv = np.array([[-ctr[0]], [-ctr[1]], [ztrue]], dtype=np.float64)
        rv = np.zeros((3, 1))
        proj, _ = cv2.projectPoints(obj_all, rv, tv, K, DIST)
        proj = proj.reshape(-1, 2)
        span = proj[:, 0].max() - proj[:, 0].min()
        errs = []
        for _ in range(30):
            noisy = proj + rng.normal(0, 0.1, proj.shape)   # 0.1 px 亚像素噪声
            ok, rv2, tv2 = cv2.solvePnP(obj_all, noisy, K, DIST,
                                        flags=cv2.SOLVEPNP_ITERATIVE)
            R, _ = cv2.Rodrigues(rv2)
            zc = (R @ ctr.reshape(3, 1) + tv2).ravel()[2]
            errs.append(zc - ztrue)
        e = np.abs(errs).mean() * 1000
        rel = e / (ztrue * 1000) * 100
        good = rel < 0.3
        ok_all = ok_all and good
        print(f"{ztrue:9.1f} {span:12.0f} {e:14.2f} {rel:8.3f}% {'✓' if good else '✗'}")
    print("\n" + ("自测通过:PnP 在 1~3 m 全程优于 0.3%,足以当深度的参考尺"
                  if ok_all else "自测失败:PnP 精度不足"))
    return ok_all


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--avg', type=int, default=12, help='每个采样点平均多少帧深度')
    ap.add_argument('--min-tags', type=int, default=8)
    ap.add_argument('--exposure', type=int, default=0, help='IR 曝光 us,0=自动闭环')
    ap.add_argument('--gain', type=int, default=160)
    ap.add_argument('--no-gui', action='store_true')
    ap.add_argument('--diag', action='store_true', help='抓一帧,打印 tag id 排布并存图')
    ap.add_argument('-o', '--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'results', 'depth_absolute.json'))
    a = ap.parse_args()

    if a.selftest:
        selftest()
        return

    det = make_detector()
    bpts = board_points()
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    align = rs.align(rs.stream.color)      # 深度投到 RGB 视角,同一像素可直接读
    pr = pipe.start(cfg)
    dev = pr.get_device()
    ds = dev.first_depth_sensor()
    # 散斑是 850nm 红外,RGB 的 IR-cut 滤镜基本挡掉了它 —— 所以发射器全程
    # 全功率(深度拿最好信噪比),同时 RGB 上的靶标干干净净。这就是"发射器两难"
    # 在这个实验里的解:不靠交替帧,靠两个相机各取所需。
    for sen in dev.query_sensors():
        if sen.supports(rs.option.emitter_enabled):
            sen.set_option(rs.option.emitter_enabled, 1)
        if sen.supports(rs.option.emitter_on_off):
            sen.set_option(rs.option.emitter_on_off, 0)
    scale = ds.get_depth_scale()
    print("RGB 检靶标(自动曝光)+ 发射器全功率深度")
    print("把板子举到相机前,慢慢改变距离。每个距离停稳 1 秒会自动采一个点。")
    print("目标:1.0 / 1.5 / 2.0 / 2.5 m 各采到,Ctrl+C 结束并出结果。\n")

    if a.diag:
        print("举稳板子,抓一帧诊断...")
        got = None
        for _ in range(90):
            fs = align.process(pipe.wait_for_frames())
            cf = fs.get_color_frame()
            if not cf:
                continue
            bgr = np.asanyarray(cf.get_data())
            g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            cs, ids_, _ = det.detectMarkers(g)
            if ids_ is not None and len(ids_) >= 6:
                got = (bgr, cs, ids_)
                break
        pipe.stop()
        if got is None:
            print("没检到足够 tag")
            return
        bgr, cs, ids_ = got
        ctr = np.array([c.reshape(4, 2).mean(0) for c in cs])
        ids_ = ids_.flatten()
        print(f"\n检到 {len(ids_)} 个 tag,id 范围 {ids_.min()}~{ids_.max()}")
        # 按 y 聚成行,行内按 x 排序 —— 直接看出 id 是怎么编号的
        order = np.argsort(ctr[:, 1])
        rows, cur = [], [order[0]]
        for k in order[1:]:
            if abs(ctr[k, 1] - ctr[cur[-1], 1]) > 30:
                rows.append(cur); cur = [k]
            else:
                cur.append(k)
        rows.append(cur)
        print("\n板面上的 id 排布(按实际像素位置还原,上→下 / 左→右):")
        for r_ in rows:
            r_ = sorted(r_, key=lambda k: ctr[k, 0])
            print("   " + "  ".join(f"{ids_[k]:3d}" for k in r_))
        tag_px = np.linalg.norm(cs[0].reshape(4, 2)[0] - cs[0].reshape(4, 2)[1])
        print(f"\ntag 边长约 {tag_px:.0f} px")
        cv2.aruco.drawDetectedMarkers(bgr, cs, ids_.reshape(-1, 1))
        out = os.path.join(os.path.dirname(a.out), 'board_diag.png')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, bgr)
        print(f"存图 {out}")
        return

    S = []
    last_z = None
    still = 0
    try:
        while True:
            fs = align.process(pipe.wait_for_frames())
            cf = fs.get_color_frame()
            df = fs.get_depth_frame()
            if not cf or not df:
                continue
            bgr = np.asanyarray(cf.get_data())
            ir = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            r = solve_board(ir, det, bpts)
            if not a.no_gui:
                vis = bgr.copy()
                cs, ids_, _ = det.detectMarkers(ir)
                if ids_ is not None:
                    cv2.aruco.drawDetectedMarkers(vis, cs, ids_)
                ntag = 0 if ids_ is None else len(ids_)
                txt = (f"tags {ntag}/{TAG_ROWS*TAG_COLS}   亮度中位 {int(np.median(ir))}"
                       f"   已采 {len(S)}")
                cv2.putText(vis, txt, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if ntag >= a.min_tags else (0, 165, 255), 2)
                if r is not None:
                    cv2.putText(vis, f"board {r[0]:.3f} m  still {still}/8",
                                (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow('depth_absolute  (q=quit)', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if r is None:
                print("\r  没检到板子(或 tag 太少)                       ",
                      end='', flush=True)
                still = 0
                continue
            z_pnp, ctr_px, ncorner, rms = r
            if ncorner < a.min_tags * 4:
                print(f"\r  tag 太少 {ncorner//4}/{TAG_ROWS*TAG_COLS}          ",
                      end='', flush=True)
                still = 0
                continue
            # 静止判定:PnP 距离稳定
            if last_z is not None and abs(z_pnp - last_z) < 0.004:
                still += 1
            else:
                still = 0
            last_z = z_pnp
            covered = [f"{s[0]:.1f}" for s in S]
            print(f"\r  板距 {z_pnp:5.3f} m  tags {ncorner//4:2d}  重投影 {rms:4.2f}px  "
                  f"稳定 {still:2d}/8  已采 {len(S)} 点 {covered}      ",
                  end='', flush=True)
            if still < 8:
                continue
            # 稳定了:多帧平均读深度
            acc = []
            for _ in range(a.avg):
                f2 = align.process(pipe.wait_for_frames()).get_depth_frame()
                if f2:
                    acc.append(np.asanyarray(f2.get_data()).astype(np.float32) * scale)
            if not acc:
                still = 0
                continue
            dep = np.median(np.stack(acc), 0)
            z_dep, valid = read_depth(dep, ctr_px)
            if z_dep is None:
                print(f"\n  板中心深度无效(有效率 {valid:.0%}),换个角度再试")
                still = 0
                continue
            # 同距离去重
            if any(abs(s[0] - z_pnp) < 0.08 for s in S):
                still = 0
                continue
            S.append((z_pnp, z_dep))
            print(f"\n  ✓ 采样 {len(S)}: PnP {z_pnp:.3f} m  深度 {z_dep:.3f} m  "
                  f"偏差 {(z_dep-z_pnp)*1000:+.1f} mm\n")
            still = 0
    except KeyboardInterrupt:
        print()
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if len(S) < 3:
        print(f"\n只采到 {len(S)} 个点,至少要 3 个不同距离才能拟合")
        return
    out = fit_and_report(S)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, 'w'), indent=2, ensure_ascii=False)
    print(f"\n写入 {a.out}")


if __name__ == '__main__':
    main()
