#!/usr/bin/env python3
"""D435i -> ROS1 bag 采集器(喂给 Kalibr,全程不需要安装 ROS1)。

三种模式:
  cam       阶段1+2:IR左/IR右/RGB 三目,低帧率,标内参与相机间外参
  cam_imu   阶段4  :IR双目 + IMU,标 T_cam_imu 与时间偏移
  imu       阶段4前置:只录 IMU,静置数小时,给 Allan 方差用

用法:
  python record.py cam      -o data/cam.bag     -t 180
  python record.py cam_imu  -o data/camimu.bag  -t 90
  python record.py imu      -o data/imu.bag     -t 10800
"""
import argparse
import sys
import threading
import queue
import time

import os

import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import BagWriter



class GyroClockImu:
    """把 librealsense 分开的 accel / gyro 流合成单条 IMU 消息流。

    D435i 的加速度计和陀螺是两个独立传感器,采样率不同(accel 250Hz / gyro 200Hz)
    且时间戳不对齐。Kalibr 要的是每条消息同时带 w 和 a。

    这里以 gyro 为主时钟(它频率低、且旋转是外参可观测性的主要来源),
    对 accel 做线性插值。反过来做(accel 为主)也可以,但 gyro 插值会
    平滑掉高频角速度,而高频角速度正是标 t_shift 的信号来源。
    """

    def __init__(self):
        self.a_buf = []          # [(t, np.array(3))]
        self.pending_gyro = []

    def push_accel(self, t, a):
        self.a_buf.append((t, np.asarray(a, dtype=np.float64)))
        if len(self.a_buf) > 512:
            self.a_buf = self.a_buf[-256:]

    def push_gyro(self, t, g):
        self.pending_gyro.append((t, np.asarray(g, dtype=np.float64)))

    def drain(self):
        """吐出所有能被 accel 前后夹住(可插值)的 gyro 样本。"""
        out = []
        keep = []
        for tg, g in self.pending_gyro:
            if not self.a_buf or tg < self.a_buf[0][0]:
                continue                      # accel 还没覆盖到,丢弃(只发生在开头)
            if tg > self.a_buf[-1][0]:
                keep.append((tg, g))          # accel 还没到,等下一轮
                continue
            ts = np.array([t for t, _ in self.a_buf])
            i = int(np.searchsorted(ts, tg))
            t1, a1 = self.a_buf[i - 1]
            t2, a2 = self.a_buf[i]
            w = 0.0 if t2 == t1 else (tg - t1) / (t2 - t1)
            out.append((tg, g, a1 + w * (a2 - a1)))
        self.pending_gyro = keep
        return out


def available(dev, stream, fmt, idx=0):
    """设备在当前链路下真正支持的 (w,h,fps)。USB2 下 IR2 整个流都不存在,
    照 USB3 的假设写死会直接 Couldn\'t resolve requests。"""
    out = []
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            try:
                vp = p.as_video_stream_profile()
            except Exception:
                continue
            if p.stream_type() == stream and p.stream_index() == idx and p.format() == fmt:
                out.append((vp.width(), vp.height(), p.fps()))
    return out


def _fit(cands, width, height, fps):
    if not cands:
        return None
    exact = [c for c in cands if (c[0], c[1]) == (width, height) and c[2] == fps]
    if exact:
        return exact[0]
    same = [c for c in cands if (c[0], c[1]) == (width, height)]
    if same:
        return max(same, key=lambda c: c[2])
    return max(cands, key=lambda c: (min(c[2], fps), c[0] * c[1]))


def configure_streams(cfg, mode, width, height, fps, use_color=False):
    """返回实际启用的图像流。缺什么就不启用什么,而不是硬失败。"""
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        raise RuntimeError("没有检测到 RealSense 设备")
    dev = ctx.query_devices()[0]
    enabled = []
    if mode == "cam_imu" and use_color:
        c = _fit(available(dev, rs.stream.color, rs.format.bgr8), width, height, fps)
        if not c:
            raise RuntimeError("没有可用的彩色流")
        cfg.enable_stream(rs.stream.color, c[0], c[1], rs.format.bgr8, c[2])
        enabled.append(("/cam2/image_raw", 0, c))
    elif mode in ("cam", "cam_imu"):
        for idx, topic in ((1, "/cam0/image_raw"), (2, "/cam1/image_raw")):
            c = _fit(available(dev, rs.stream.infrared, rs.format.y8, idx), width, height, fps)
            if c:
                cfg.enable_stream(rs.stream.infrared, idx, c[0], c[1], rs.format.y8, c[2])
                enabled.append((topic, idx, c))
            elif idx == 2:
                print("  该链路没有 Infrared[2](USB2 会砍掉它)-> 退化为单目 + IMU")
    if mode == "cam":
        c = _fit(available(dev, rs.stream.color, rs.format.bgr8), width, height, fps)
        if c:
            cfg.enable_stream(rs.stream.color, c[0], c[1], rs.format.bgr8, c[2])
            enabled.append(("/cam2/image_raw", 0, c))
    if mode in ("cam_imu", "imu"):
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    return enabled


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['cam', 'cam_imu', 'imu'])
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('-t', '--seconds', type=float, default=120.0)
    ap.add_argument('--width', type=int, default=848)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30, help='传感器采集帧率')
    ap.add_argument('--bag-hz', type=float, default=None,
                    help='写入 bag 的图像帧率;默认 cam=4, cam_imu=20')
    ap.add_argument('--force', action='store_true', help='覆盖已存在的 bag')
    ap.add_argument('--exposure', type=int, default=0,
                    help='手动曝光(微秒)。cam_imu 激励运动务必设短,如 5000;0=自动')
    ap.add_argument('--gain', type=int, default=100)
    ap.add_argument('--color', action='store_true',
                    help='cam_imu 用彩色相机(推荐:内参已标定,且不依赖红外照明)')
    ap.add_argument('--emitter', action='store_true',
                    help='保持红外发射器开启(标定时应关闭,默认已关)')
    args = ap.parse_args()

    if os.path.exists(args.out):
        if args.force:
            os.remove(args.out); print(f"[覆盖] 已删除旧 bag {args.out}")
        else:
            print(f"错误:{args.out} 已存在。加 --force 覆盖。"); sys.exit(1)

    bag_hz = args.bag_hz or {'cam': 4.0, 'cam_imu': 20.0, 'imu': 0.0}[args.mode]
    min_dt = (1.0 / bag_hz) if bag_hz > 0 else 0.0

    pipe = rs.pipeline()
    cfg = rs.config()
    enabled = configure_streams(cfg, args.mode, args.width, args.height,
                                args.fps, use_color=args.color)

    q = queue.Queue(maxsize=8192)
    prof = pipe.start(cfg, lambda f: q.put((f, time.time())))
    dev = prof.get_device()

    # --- 采集期的传感器设置 ---
    for s in dev.query_sensors():
        nm = s.get_info(rs.camera_info.name)
        is_ir = ('Stereo' in nm or s.is_depth_sensor())
        if s.supports(rs.option.global_time_enabled):
            s.set_option(rs.option.global_time_enabled, 1)   # 统一到主机时钟
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 1 if args.emitter else 0)
            print(f"[emitter] {'开' if args.emitter else '关'}"
                  f"  <- IR 图上的散斑会破坏 AprilTag 检测,标定必须关")
        # cam-IMU 采集要做剧烈激励运动,自动曝光会为了亮度把曝光拉长,
        # tag 边缘糊掉之后 Kalibr 提不到角点 —— 比标定内参时更要命。
        want = is_ir if not args.color else (not is_ir)
        if args.exposure and want and s.supports(rs.option.exposure):
            try:
                if s.supports(rs.option.enable_auto_exposure):
                    s.set_option(rs.option.enable_auto_exposure, 0)
                r = s.get_option_range(rs.option.exposure)
                e = float(min(max(args.exposure, r.min), r.max))
                s.set_option(rs.option.exposure, e)
                if s.supports(rs.option.gain):
                    gr = s.get_option_range(rs.option.gain)
                    g = float(min(max(args.gain, gr.min), gr.max))
                    s.set_option(rs.option.gain, g)
                print(f"[曝光]    {nm}: {e/1000:.1f}ms  增益 {args.gain}")
            except Exception as ex:
                print(f"[曝光]    设置失败 {ex}")

    topics = {}
    for topic, idx, c in enabled:
        topics[topic] = ('sensor_msgs/msg/Image', topic.split('/')[1])
        print(f"  启用 {topic:18s} {c[0]}x{c[1]}@{c[2]}")
    if args.mode in ('cam_imu', 'imu'):
        topics['/imu0'] = ('sensor_msgs/msg/Imu', 'imu4')

    print(f"\n模式={args.mode}  时长={args.seconds}s  "
          f"图像 {args.width}x{args.height}@{args.fps}Hz -> bag {bag_hz}Hz")
    print(f"topics: {list(topics)}")
    print("Ctrl-C 可提前结束并正常收尾\n")

    # 温度日志:cam-IMU 采集若从冷启动开始,窗口内温度会变几度,
    # 这是事后做温漂补偿验证的唯一依据(bag 本身存不了温度)。
    temp_sens = None
    for _s in dev.query_sensors():
        if _s.supports(rs.option.asic_temperature):
            temp_sens = _s
    temp_log = []
    next_temp = time.time()

    fuse = GyroClockImu()
    last_img_t = {}
    n_img = {k: 0 for k in topics if 'image' in k}
    n_imu = 0
    t0 = time.time()

    with BagWriter(args.out) as bag:
        try:
            while time.time() - t0 < args.seconds:
                try:
                    f, _ = q.get(timeout=1.0)
                except queue.Empty:
                    continue

                if temp_sens is not None and time.time() >= next_temp:
                    next_temp = time.time() + 2.0
                    try:
                        temp_log.append((time.time() - t0,
                                         float(temp_sens.get_option(rs.option.asic_temperature)),
                                         float(temp_sens.get_option(rs.option.projector_temperature))
                                         if temp_sens.supports(rs.option.projector_temperature) else float('nan')))
                    except Exception:
                        pass

                if f.is_motion_frame():
                    mf = f.as_motion_frame()
                    t = mf.get_timestamp() / 1000.0
                    d = mf.get_motion_data()
                    v = (d.x, d.y, d.z)
                    if mf.get_profile().stream_type() == rs.stream.accel:
                        fuse.push_accel(t, v)
                    else:
                        fuse.push_gyro(t, v)
                    for tg, g, a in fuse.drain():
                        bag.write_imu('/imu0', tg, g, a)
                        n_imu += 1
                    if args.mode == 'imu' and n_imu % 400 == 0 and n_imu:
                        el = time.time() - t0
                        print(f"\r  {el/60:7.1f} / {args.seconds/60:.0f} min   "
                              f"imu={n_imu}   {n_imu/max(el,1e-9):.1f} Hz   ", end='', flush=True)
                    continue

                if args.mode == 'imu':
                    continue

                # pipeline callback 在混合流(30Hz 图像 + 200Hz IMU)下并不保证交付
                # frameset:有时把 IR1/IR2 打包成 frameset,有时分成两个独立 video frame。
                # 只处理 frameset 会把后一种整批丢掉 —— 实测出现过一帧不剩的情况。
                batch = []
                fs = f.as_frameset()
                if fs:
                    batch = [fs[i] for i in range(fs.size())]
                else:
                    batch = [f]

                for vf in batch:
                    try:
                        pr = vf.get_profile()
                        st, idx = pr.stream_type(), pr.stream_index()
                    except Exception:
                        continue
                    if st == rs.stream.infrared and idx == 1:
                        topic, enc = '/cam0/image_raw', 'mono8'
                    elif st == rs.stream.infrared and idx == 2:
                        topic, enc = '/cam1/image_raw', 'mono8'
                    elif st == rs.stream.color:
                        topic, enc = '/cam2/image_raw', 'bgr8'
                    else:
                        continue
                    if topic not in topics:
                        continue
                    t = vf.get_timestamp() / 1000.0
                    if t - last_img_t.get(topic, -1e9) < min_dt:
                        continue
                    last_img_t[topic] = t
                    try:
                        arr = np.asanyarray(vf.as_video_frame().get_data())
                    except Exception:
                        continue
                    bag.write_image(topic, t, arr, enc, frame_id=topics[topic][1])
                    n_img[topic] += 1

                el = time.time() - t0
                print(f"\r  {el:6.1f}s / {args.seconds:.0f}s   "
                      + "  ".join(f"{k.split('/')[1]}={v}" for k, v in n_img.items())
                      + (f"   imu={n_imu}" if n_imu else "") + "   ", end='', flush=True)
        except KeyboardInterrupt:
            print("\n[中断] 正在收尾...")
        finally:
            pipe.stop()

    if temp_log:
        import csv
        tp = os.path.splitext(args.out)[0] + '_temp.csv'
        with open(tp, 'w', newline='') as fp:
            w_ = csv.writer(fp); w_.writerow(['t_rel', 'asic_C', 'proj_C'])
            w_.writerows(temp_log)
        dT = temp_log[-1][1] - temp_log[0][1]
        print(f"\n温度日志 -> {tp}   {temp_log[0][1]:.0f} -> {temp_log[-1][1]:.0f} °C (Δ{dT:+.0f})")
        if abs(dT) < 2:
            print("  ⚠ 采集窗口内温度几乎没变 —— 无法用于温漂补偿验证(需冷启动)")

    print(f"\n\n写入完成: {args.out}")
    empty = [k for k, v in n_img.items() if v == 0]
    if empty:
        print(f"  ⚠ 这些图像 topic 一帧都没写入: {empty}")
        print("    多半是 frameset 没组装成功 —— 重跑一次通常就好")
    for k, v in n_img.items():
        print(f"  {k:22s} {v} 帧")
    if n_imu:
        print(f"  {'/imu0':22s} {n_imu} 条")


if __name__ == '__main__':
    main()
