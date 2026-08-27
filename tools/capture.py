#!/usr/bin/env python3
"""AprilGrid 采集器:实时检测 + 姿态覆盖引导 + 直接写 ROS1 bag。

为什么不是"随便拍几十张":
  标定的病态方向来自姿态单一。如果所有帧里板子都正对相机、都在画面中央、
  距离都差不多,那么 fx 和 "板子到相机的距离" 这两个量在数值上几乎无法区分
  (焦距-距离歧义),优化器会给你一个重投影误差极小、但焦距系统性偏掉的解。
  破解办法只有一个:让板子在画面里跑遍各个位置,并且倾斜到大角度。

  所以这个工具不按快门计数,它按"覆盖格子"计数。

用法:
  python capture.py --stream color -o data/cam_rgb.bag
  python capture.py --stream ir    -o data/cam_ir.bag
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import BagWriter

TAG_ROWS, TAG_COLS = 6, 6
TAG_SIZE = 0.0352
TAG_SPACING = 0.3

GRID = 3           # 画面分 3x3 区域统计覆盖
TILT_BINS = [0, 15, 30, 45, 90]   # 倾角分桶(度)
NEED_PER_CELL = 2  # 每个画面区域至少要有几帧
NEED_PER_TILT = 4  # 每个倾角桶至少要有几帧
MIN_TAGS = 12      # 一帧至少检测到多少个 tag 才算数(共 36 个)


def make_detector():
    """AprilTag 检测器。

    默认的 adaptiveThreshWinSize 是 3~23 step 10,即只试 3/13/23 三个窗口。
    自适应二值化的窗口必须与 tag 单格尺度相当:36h11 是 8x8 格,IR 下 tag 约
    48px 时每格才 6px,窗口 13/23 会把相邻黑白格一起平均掉,tag 内部图案糊没。
    实测同一张 IR 图:默认 2/36,改成 3~15 step 2 后 29/36。
    代价只是多试几个窗口(检测耗时 3ms -> 约 7ms),完全吃得下。
    """
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 15
    p.adaptiveThreshWinSizeStep = 2
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def board_points():
    """AprilGrid 的 3D 角点。tag id 行优先递增,每 tag 四角序为 OpenCV aruco 的
    左上->右上->右下->左下。板面为 z=0 平面。"""
    pitch = TAG_SIZE * (1.0 + TAG_SPACING)
    pts = {}
    for tid in range(TAG_ROWS * TAG_COLS):
        r, c = divmod(tid, TAG_COLS)
        x0, y0 = c * pitch, r * pitch
        pts[tid] = np.array([
            [x0,            y0 + TAG_SIZE, 0.0],
            [x0 + TAG_SIZE, y0 + TAG_SIZE, 0.0],
            [x0 + TAG_SIZE, y0,            0.0],
            [x0,            y0,            0.0],
        ], dtype=np.float64)
    return pts


def open_camera(stream, width, height, fps, exposure_us=0, gain=64,
                color_exposure_us=0, color_gain=0):
    """stream: color | ir | stereo(IR 双目)"""
    pipe = rs.pipeline()
    cfg = rs.config()
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        print("没有检测到相机"); sys.exit(1)
    dev = ctx.query_devices()[0]

    # 从设备真实支持的组合里挑,避免 USB2 下 "Couldn't resolve requests"
    avail = []
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            try:
                vp = p.as_video_stream_profile()
            except Exception:
                continue
            st, idx = p.stream_type(), p.stream_index()
            if stream == 'color' and st == rs.stream.color and p.format() == rs.format.bgr8:
                avail.append((vp.width(), vp.height(), p.fps()))
            if stream in ('ir', 'stereo', 'trio') and st == rs.stream.infrared and idx == 1 \
                    and p.format() == rs.format.y8:
                avail.append((vp.width(), vp.height(), p.fps()))
    if not avail:
        print(f"设备不支持 {stream} 流"); sys.exit(1)

    exact = [a for a in avail if (a[0], a[1]) == (width, height) and a[2] == fps]
    if exact:
        w, h, f = exact[0]
    else:
        # 优先保帧率(采集手感),其次保分辨率
        w, h, f = max(avail, key=lambda a: (min(a[2], 30), a[0] * a[1]))
        print(f"⚠ 请求 {width}x{height}@{fps} 不可用,改用 {w}x{h}@{f}")

    if stream == 'color':
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
    else:
        cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, f)
        if stream in ('stereo', 'trio'):
            cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, f)
        if stream == 'trio':
            # RGB 视场(69x42)比 IR(87x58)窄,板子要按 RGB 的视野摆
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
    prof = pipe.start(cfg)

    for s in prof.get_device().query_sensors():
        if s.supports(rs.option.emitter_on_off):
            # RealSense 的 option 是设备级持久的:别的程序开过交替模式会一直留着,
            # 导致这里一半帧是暗的干净帧。标定不需要交替,显式关掉。
            s.set_option(rs.option.emitter_on_off, 0)
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 0)   # 散斑会毁掉 IR 上的 tag 检测
        if s.supports(rs.option.global_time_enabled):
            s.set_option(rs.option.global_time_enabled, 1)
        # 曝光:标定是手持采集,自动曝光会为了亮度把曝光时间拉到帧周期上限
        # (15fps 时可达 66ms),tag 的黑白边缘糊成灰带,检测直接归零。
        # 锁死短曝光、用增益补亮度 —— 噪点比运动模糊好对付得多。
        is_ir_sensor = ('Stereo' in s.get_info(rs.camera_info.name) or s.is_depth_sensor())
        # IR 与彩色对同一光源的响应差一个数量级(IR 只吃环境光里的近红外成分,
        # 彩色吃全部可见光),必须分别设曝光,否则一个欠曝一个全白。
        want = True if stream == 'trio' else (
            is_ir_sensor if stream in ('ir', 'stereo') else (not is_ir_sensor))
        exp_v = exposure_us if is_ir_sensor else (color_exposure_us or exposure_us)
        gain_v = gain if is_ir_sensor else (color_gain or gain)
        if exp_v and want and s.supports(rs.option.exposure):
            try:
                if s.supports(rs.option.enable_auto_exposure):
                    s.set_option(rs.option.enable_auto_exposure, 0)
                rng = s.get_option_range(rs.option.exposure)
                e = float(min(max(exp_v, rng.min), rng.max))
                s.set_option(rs.option.exposure, e)
                if s.supports(rs.option.gain):
                    gr = s.get_option_range(rs.option.gain)
                    g = float(min(max(gain_v, gr.min), gr.max))
                    s.set_option(rs.option.gain, g)
                    print(f"  [{s.get_info(rs.camera_info.name)}] 曝光 {e/1000:.2f}ms 增益 {g:.0f}")
            except Exception as ex:
                print(f"  曝光设置失败: {ex}")

    sp = (prof.get_stream(rs.stream.color) if stream == 'color'
          else prof.get_stream(rs.stream.infrared, 1))
    if stream in ('stereo', 'trio'):
        e = prof.get_stream(rs.stream.infrared, 1).get_extrinsics_to(
            prof.get_stream(rs.stream.infrared, 2))
        print(f"  出厂立体基线 {abs(e.translation[0])*1000:.2f} mm "
              f"(标定完拿这个数对一下,差太多说明采集有问题)")
    intr = sp.as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    return pipe, K, (w, h), f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stream', choices=['color', 'ir', 'stereo', 'trio'], default='color')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=15,
                    help='传感器帧率。标定只往 bag 写 4Hz,30fps 纯属浪费带宽和 CPU')
    ap.add_argument('--min-move', type=float, default=0.04,
                    help='与已存帧的最小位姿差异(米);太小会存进一堆几乎重复的帧')
    ap.add_argument('--topic', default=None)
    ap.add_argument('--exposure', type=int, default=0,
                    help='手动曝光(微秒)。手持采集务必设短,如 8000(8ms);0=自动')
    ap.add_argument('--gain', type=int, default=64,
                    help='增益,配合短曝光补亮度')
    ap.add_argument('--color-exposure', type=int, default=0,
                    help='彩色相机的曝光(微秒);trio 模式下与 IR 分开设,0=沿用 --exposure')
    ap.add_argument('--color-gain', type=int, default=0, help='彩色相机增益,0=沿用 --gain')
    ap.add_argument('--resume', action='store_true',
                    help='在已有的 _frames 目录上续采,保留此前的覆盖度')
    ap.add_argument('--force', action='store_true', help='覆盖已存在的 bag')
    ap.add_argument('--no-gui', action='store_true',
                    help='不开窗口(自动化测试用;正常采集需要看引导所以别加)')
    ap.add_argument('--seconds', type=float, default=0,
                    help='跑够 N 秒自动结束(配合 --no-gui 做无人值守测试)')
    ap.add_argument('--settle', type=int, default=0,
                    help='要求板子静止 N 帧后才采样(trio 必须用:RGB 与 IR 无硬件同步,'
                         '实测时间戳差 44ms,运动中三路会错位并把误差塞进外参)')
    ap.add_argument('--settle-px', type=float, default=0.6,
                    help='判定静止的角点位移阈值(像素/帧)')
    args = ap.parse_args()
    topic = args.topic or ('/cam2/image_raw' if args.stream == 'color' else '/cam0/image_raw')
    topic2 = '/cam1/image_raw'      # stereo 时的右目

    # 输出路径的问题必须在采集之前暴露 —— 采完才发现写不了,代价是重采一次
    if os.path.exists(args.out):
        if args.force or args.resume:
            os.remove(args.out)
            print(f"[覆盖] 已删除旧 bag {args.out}")
            if args.force and not args.resume:
                # frames 目录也要清,否则上一轮的 PNG 会跟这一轮混在一起
                import glob
                fd = os.path.splitext(args.out)[0] + '_frames'
                old = glob.glob(os.path.join(fd, '*.png'))
                for f in old:
                    os.remove(f)
                if old:
                    print(f"[覆盖] 已清空 {fd} 的 {len(old)} 张旧 PNG")
        else:
            print(f"错误:{args.out} 已存在。")
            print("  --force  覆盖   |   --resume  在已有帧上续采")
            sys.exit(1)

    pipe, K, (W, H), FPS = open_camera(args.stream, args.width, args.height, args.fps,
                                       exposure_us=args.exposure, gain=args.gain,
                                       color_exposure_us=args.color_exposure,
                                       color_gain=args.color_gain)
    dist0 = np.zeros(5)          # 出厂 coeffs 全 0,位姿估计用零畸变足够(只为判多样性)
    bpts = board_points()

    det = make_detector()

    cell_hit = np.zeros((GRID, GRID), dtype=int)
    tilt_hit = np.zeros(len(TILT_BINS) - 1, dtype=int)
    kept = []           # (t, img)
    kept_r = []         # stereo/trio 右目
    kept_c = []         # trio 彩色
    kept_pose = []      # tvec

    # 保底:每存一帧同时落盘 PNG。写 bag 万一出问题,不必重采。
    frames_dir = os.path.splitext(args.out)[0] + '_frames'
    os.makedirs(frames_dir, exist_ok=True)
    idx_path = os.path.join(frames_dir, 'times.txt')

    both_ok = [0]
    if args.resume and os.path.exists(idx_path):
        print("续采:载入已有帧并重算覆盖度 ...")
        for line in open(idx_path):
            if not line.strip():
                continue
            nm, tt = line.split()
            im = cv2.imread(os.path.join(frames_dir, nm),
                            cv2.IMREAD_COLOR if args.stream == 'color' else cv2.IMREAD_GRAYSCALE)
            if im is None:
                continue
            g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
            cs, ii, _ = det.detectMarkers(g)
            if ii is None or len(ii) < MIN_TAGS:
                kept.append((float(tt), im)); continue
            o, q = [], []
            for c_, tid in zip(cs, ii.flatten()):
                if tid in bpts:
                    o.append(bpts[tid]); q.append(c_.reshape(4, 2))
            o = np.concatenate(o).astype(np.float64); q = np.concatenate(q).astype(np.float64)
            okp, rv, tv = cv2.solvePnP(o, q, K, dist0, flags=cv2.SOLVEPNP_ITERATIVE)
            R, _ = cv2.Rodrigues(rv)
            tl = np.degrees(np.arccos(min(1.0, abs(R[2, 2]))))
            ctr = q.mean(axis=0)
            gx = min(GRID - 1, int(ctr[0] / W * GRID)); gy = min(GRID - 1, int(ctr[1] / H * GRID))
            tb = max(0, min(len(tilt_hit) - 1, int(np.digitize(tl, TILT_BINS) - 1)))
            cell_hit[gy, gx] += 1; tilt_hit[tb] += 1
            kept.append((float(tt), im)); kept_pose.append(tv.ravel().copy())
            # trio/stereo:右目与彩色也要一并载入,否则写 bag 时三路对不齐
            rp = os.path.join(frames_dir, nm.replace('.png', '_r.png'))
            if os.path.exists(rp):
                ri = cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
                if ri is not None:
                    kept_r.append((float(tt), ri))
            cp = os.path.join(frames_dir, nm.replace('.png', '_c.png'))
            if os.path.exists(cp):
                ci = cv2.imread(cp, cv2.IMREAD_COLOR)
                if ci is not None:
                    kept_c.append((float(tt), ci))
            # 共视统计也要从历史帧重算,否则续采后显示成 0/N 误导判断
            if os.path.exists(rp):
                ri2 = cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
                if ri2 is not None:
                    _, i2r, _ = det.detectMarkers(ri2)
                    if i2r is not None and len(i2r) >= MIN_TAGS:
                        both_ok[0] += 1
        print(f"  载入 {len(kept)} 帧"
              + (f" (右目 {len(kept_r)}, 彩色 {len(kept_c)})" if kept_r or kept_c else "")
              + f"\n  九宫格:\n{cell_hit}\n  倾角桶: {tilt_hit.tolist()}")

    times_fp = open(idx_path, 'a' if args.resume else 'w')

    dt_ms = []
    prev_pts = [None]     # 上一帧 {tag_id: 中心},用于静止判定
    still_cnt = [0]

    def keep(t, img, tvec, img2=None, img3=None):
        i = len(kept)   # 续采时自然接在已有帧之后
        kept.append((t, img)); kept_pose.append(tvec)
        png_fast = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]
        cv2.imwrite(os.path.join(frames_dir, f'{i:04d}.png'), img, png_fast)
        if img3 is not None:
            cv2.imwrite(os.path.join(frames_dir, f'{i:04d}_c.png'), img3, png_fast)
            kept_c.append((t, img3))
        if img2 is not None:
            cv2.imwrite(os.path.join(frames_dir, f'{i:04d}_r.png'), img2, png_fast)
            kept_r.append((t, img2))
            # 双目标定要求两个相机同时看到板子,只看左目会采到一堆废帧
            c2, i2, _ = det.detectMarkers(img2 if img2.ndim == 2
                                          else cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY))
            if i2 is not None and len(i2) >= MIN_TAGS:
                both_ok[0] += 1
        times_fp.write(f'{i:04d}.png {t:.6f}\n'); times_fp.flush()

    print(f"\n流={args.stream}  {W}x{H}@{FPS}  ->  {args.out}   topic={topic}")
    print("把板子举起来,让绿框跑遍九宫格,并且大角度倾斜。")
    print("  [空格] 强制存一帧    [q] 结束并写 bag\n")

    miss = 0
    win = 'AprilGrid capture  —  空格=强制存  q=结束'
    if not args.no_gui:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, min(1280, W), min(720, H))
    t_begin = time.time()

    try:
        while True:
            try:
                fs = pipe.wait_for_frames(5000)
                miss = 0
            except Exception as e:
                # 保存帧时的 PNG 编码会占住 CPU,偶尔把取帧饿超时。
                # 这不该让整次采集连同已采数据一起崩掉 —— 跳过继续。
                miss += 1
                print(f"\n  [取帧超时 {miss}/10] {str(e)[:50]}")
                if miss >= 10:
                    print("  连续 10 次取不到帧,结束采集(已采数据保留)")
                    break
                continue
            f = fs.get_color_frame() if args.stream == 'color' else fs.get_infrared_frame(1)
            if not f:
                continue
            t = f.get_timestamp() / 1000.0
            img = np.asanyarray(f.get_data())
            img2 = None; img3 = None
            if args.stream == 'trio':
                f3 = fs.get_color_frame()
                if not f3:
                    continue
                img3 = np.asanyarray(f3.get_data())
                # RGB 与 IR 是独立 sensor,无硬件同步。记录偏差,超标时提醒放慢动作。
                dt_ms.append(abs(f3.get_timestamp() / 1000.0 - t) * 1000.0)
            if args.stream in ('stereo', 'trio'):
                f2 = fs.get_infrared_frame(2)
                if not f2:
                    continue
                img2 = np.asanyarray(f2.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
            vis = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            corners, ids, _ = det.detectMarkers(gray)
            n_tag = 0 if ids is None else len(ids)
            status, color = f"tags {n_tag}/36", (0, 0, 255)
            why = (f"tag 只有 {n_tag},不足 {MIN_TAGS} —— 走近些/别太斜/检查光照"
                   if n_tag < MIN_TAGS else "")
            rvec = tvec = None

            if n_tag >= MIN_TAGS:
                obj, imgp = [], []
                for cs, i in zip(corners, ids.flatten()):
                    if i in bpts:
                        obj.append(bpts[i]); imgp.append(cs.reshape(4, 2))
                obj = np.concatenate(obj).astype(np.float64)
                imgp = np.concatenate(imgp).astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist0,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    R, _ = cv2.Rodrigues(rvec)
                    # 板面法向与光轴夹角 = 倾角。正对时 0°,越大越好(破解焦距-距离歧义)
                    tilt = np.degrees(np.arccos(min(1.0, abs(R[2, 2]))))
                    ctr = imgp.mean(axis=0)
                    gx = min(GRID - 1, int(ctr[0] / W * GRID))
                    gy = min(GRID - 1, int(ctr[1] / H * GRID))
                    tb = int(np.digitize(tilt, TILT_BINS) - 1)
                    tb = max(0, min(len(tilt_hit) - 1, tb))

                    # --- 静止判定:按 tag id 匹配共同点比位移 ---
                    # 不能用"角点数组长度相同"当前提:每帧检出的 tag 数会波动
                    # (30/31/32),一变就判成无法比较,still_cnt 永远归零。
                    if args.settle:
                        cur = {int(t_): cs.reshape(4, 2).mean(axis=0)
                               for cs, t_ in zip(corners, ids.flatten())}
                        prev = prev_pts[0]
                        shared = [k for k in cur if prev and k in prev]
                        if len(shared) >= 6:
                            mv = float(np.mean([np.linalg.norm(cur[k] - prev[k])
                                                for k in shared]))
                        else:
                            mv = 1e9
                        prev_pts[0] = cur
                        still_cnt[0] = still_cnt[0] + 1 if mv < args.settle_px else 0
                        settled = still_cnt[0] >= args.settle
                    else:
                        settled = True
                        mv = 0.0

                    far = all(np.linalg.norm(tvec.ravel() - p) > args.min_move for p in kept_pose)
                    need = cell_hit[gy, gx] < NEED_PER_CELL or tilt_hit[tb] < NEED_PER_TILT
                    # 不保存时必须说清原因,否则用户只看到"没反应"
                    if not settled:
                        why = (f"等待静止 {still_cnt[0]}/{args.settle} 帧"
                               f"(当前位移 {mv:.2f}px,需 <{args.settle_px}px)")
                    elif not far:
                        why = f"位姿与已存帧太近(<{args.min_move*100:.0f}cm) 换个位置"
                    elif not need:
                        why = (f"格[{gy},{gx}]={cell_hit[gy,gx]}/{NEED_PER_CELL} 且 "
                               f"{TILT_BINS[tb]}-{TILT_BINS[tb+1]}deg={tilt_hit[tb]}/{NEED_PER_TILT} 都已满")
                    else:
                        why = ""
                    if far and need and settled:
                        keep(t, img.copy(), tvec.ravel().copy(),
                             img2.copy() if img2 is not None else None,
                             img3.copy() if img3 is not None else None)
                        cell_hit[gy, gx] += 1; tilt_hit[tb] += 1
                        color = (0, 255, 0)
                    else:
                        color = (0, 200, 200)
                    inside = (imgp[:, 0] > 2).all() and (imgp[:, 0] < W - 3).all() and \
                             (imgp[:, 1] > 2).all() and (imgp[:, 1] < H - 3).all()
                    full = n_tag == 36 and inside
                    st_tag = (f"  静止{still_cnt[0]}/{args.settle}" if args.settle else "")
                    status = (f"tags {n_tag}/36 {'完整' if full else '部分出画->退远些'}{st_tag}   "
                              f"dist {np.linalg.norm(tvec):.2f}m   tilt {tilt:4.1f}deg")
                    if not full:
                        color = (0, 165, 255)
                    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                    cv2.drawFrameAxes(vis, K, dist0, rvec, tvec, TAG_SIZE * 2)

            # ---- 覆盖度叠加 ----
            for gy in range(GRID):
                for gx in range(GRID):
                    x0, y0 = int(gx * W / GRID), int(gy * H / GRID)
                    x1, y1 = int((gx + 1) * W / GRID), int((gy + 1) * H / GRID)
                    hit = cell_hit[gy, gx]
                    c = (0, 180, 0) if hit >= NEED_PER_CELL else (60, 60, 60)
                    cv2.rectangle(vis, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), c, 1)
                    cv2.putText(vis, f"{hit}/{NEED_PER_CELL}", (x0 + 8, y0 + 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)

            bar = " ".join(f"{TILT_BINS[i]}-{TILT_BINS[i+1]}deg:{tilt_hit[i]}/{NEED_PER_TILT}"
                           for i in range(len(tilt_hit)))
            done = (cell_hit >= NEED_PER_CELL).all() and (tilt_hit >= NEED_PER_TILT).all()
            cv2.rectangle(vis, (0, H - 58), (W, H), (0, 0, 0), -1)
            cv2.putText(vis, status, (10, H - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            extra = f"   stereo {both_ok[0]}/{len(kept)}" if args.stream == 'stereo' else ""
            cv2.putText(vis, f"kept {len(kept)}{extra}   {bar}", (10, H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if done else (200, 200, 200), 1)
            if why:
                cv2.rectangle(vis, (0, H - 84), (W, H - 58), (0, 0, 0), -1)
                cv2.putText(vis, "未存: " + why, (10, H - 64),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
            if done:
                cv2.putText(vis, "COVERAGE OK - press q", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if args.no_gui:
                if args.seconds and time.time() - t_begin > args.seconds:
                    print('\n[无头模式] 时间到,结束采集')
                    break
                k = 255
            else:
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord(' ') and rvec is not None:
                keep(t, img.copy(), tvec.ravel().copy(),
                     img2.copy() if img2 is not None else None,
                     img3.copy() if img3 is not None else None)
                print(f"  强制保存,共 {len(kept)} 帧")
    finally:
        pipe.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()
        times_fp.close()

    if not kept:
        print("\n一帧都没采到,bag 未写入。"); return

    enc = 'bgr8' if args.stream == 'color' else 'mono8'
    try:
        with BagWriter(args.out) as bag:
            for i, (t, img) in enumerate(kept):
                bag.write_image(topic, t, img, enc)
                if kept_r and i < len(kept_r):
                    bag.write_image(topic2, kept_r[i][0], kept_r[i][1], enc)
                if kept_c and i < len(kept_c):
                    bag.write_image('/cam2/image_raw', kept_c[i][0], kept_c[i][1], 'bgr8')
        print(f"\n写入 {args.out}:{len(kept)} 帧  topic={topic}"
              + (f" + {len(kept_r)} 帧 {topic2}" if kept_r else "")
              + (f" + {len(kept_c)} 帧 /cam2/image_raw" if kept_c else ""))
        if dt_ms:
            import statistics as _st
            print(f"  RGB↔IR 时间戳偏差 中位 {_st.median(dt_ms):.1f} ms  最大 {max(dt_ms):.1f} ms"
                  + ("   (两个 sensor 无硬件同步,采集时动作要慢)" if _st.median(dt_ms) > 20 else ""))
        if args.stream in ('stereo', 'trio'):
            print(f"  双目共视 {both_ok[0]}/{len(kept)} 帧 "
                  + ("✓" if both_ok[0] > len(kept) * 0.7 else
                     "⚠ 偏低 —— 板子要放在两个相机都能看全的位置"))
    except Exception as e:
        print(f"\n写 bag 失败: {e}")
        print(f"但原始帧已落盘在 {frames_dir},可用以下命令重建 bag,无需重采:")
        print(f"  python tools/imgs2bag.py {frames_dir} -o {args.out} --topic {topic}")
        raise
    print(f"  画面覆盖:\n{cell_hit}")
    print(f"  倾角分布:{dict(zip([f'{TILT_BINS[i]}-{TILT_BINS[i+1]}' for i in range(len(tilt_hit))], tilt_hit.tolist()))}")
    if not ((cell_hit >= NEED_PER_CELL).all() and (tilt_hit >= NEED_PER_TILT).all()):
        print("  ⚠ 覆盖未达标,标定结果可能有系统性偏差(尤其焦距)")


if __name__ == '__main__':
    main()
