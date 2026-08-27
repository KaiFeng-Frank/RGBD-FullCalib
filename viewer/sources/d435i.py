#!/usr/bin/env python3
"""RealSense D435i 源:稠密深度图 + 彩色图 + IMU。"""
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from .base import Source


def _pick(dev, stream, fmt, idx=0, prefer=((848, 480), (640, 480), (1280, 720)),
          max_fps=30):
    """max_fps: 帧率上限。D435i 的 ASIC 发热与取流负载直接相关,
    848x480@90 会让 ASIC 稳定在 55C+;查看器 30fps 足够(WebSocket 那端也就 28Hz)。"""
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
    ok = [c for c in cands if c[2] <= max_fps] or cands
    for res in prefer:
        got = [c for c in ok if (c[0], c[1]) == res]
        if got:
            return max(got, key=lambda c: c[2])
    return max(ok, key=lambda c: (c[0] * c[1], c[2]))


class D435i(Source):
    name = 'd435i'

    def __init__(self, on_frame, jpeg_quality=75, align_to_color=False,
                 calib_intrinsics=None, with_ir=True, emitter_alternate=False):
        super().__init__(on_frame)
        self.jpeg_quality = jpeg_quality
        self.align_to_color = align_to_color
        self.calib_intrinsics = calib_intrinsics    # 我们自己标的,覆盖出厂值
        self.with_ir = with_ir
        self.emitter_alternate = emitter_alternate
        self._meta = None
        self._pipe = None
        self._prof = None

    def _open(self):
        ctx = rs.context()
        if len(ctx.query_devices()) == 0:
            raise RuntimeError('没有检测到 RealSense 设备')
        dev = ctx.query_devices()[0]
        dp = _pick(dev, rs.stream.depth, rs.format.z16)
        cp = _pick(dev, rs.stream.color, rs.format.bgr8, prefer=((1280, 720), (640, 480)))
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, dp[0], dp[1], rs.format.z16, dp[2])
        ip = _pick(dev, rs.stream.infrared, rs.format.y8, idx=1) if self.with_ir else None
        if ip:
            # IR 原图是唯一能看到散斑的视图,也是诊断多径/材质失效的窗口
            cfg.enable_stream(rs.stream.infrared, 1, ip[0], ip[1], rs.format.y8, ip[2])
        if cp:
            cfg.enable_stream(rs.stream.color, cp[0], cp[1], rs.format.bgr8, cp[2])
        self._pipe = rs.pipeline()
        prof = self._pipe.start(cfg)
        self._prof = prof
        ds = prof.get_device().first_depth_sensor()
        for s in prof.get_device().query_sensors():
            if s.supports(rs.option.emitter_enabled):
                s.set_option(rs.option.emitter_enabled, 1)     # 点云要深度,发射器必开
            if s.supports(rs.option.global_time_enabled):
                s.set_option(rs.option.global_time_enabled, 1)
            if self.emitter_alternate and s.supports(rs.option.emitter_on_off):
                # 交替模式:发射器逐帧开关。散斑帧供深度,干净帧供 VIO/靶标。
                # USB3 下 848x480@90 砍半仍有 45Hz,代价可接受。
                s.set_option(rs.option.emitter_on_off, 1)
                print('  [emitter] 交替模式已开启(散斑帧 / 干净帧 逐帧轮换)', flush=True)

        di = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        ci = (prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
              if cp else None)
        ext = None
        if cp:
            e = prof.get_stream(rs.stream.depth).get_extrinsics_to(prof.get_stream(rs.stream.color))
            ext = dict(rotation=list(e.rotation), translation=list(e.translation))

        self._meta = dict(
            source='d435i', kind='depth_image',
            serial=dev.get_info(rs.camera_info.serial_number),
            firmware=dev.get_info(rs.camera_info.firmware_version),
            usb=dev.get_info(rs.camera_info.usb_type_descriptor),
            depth=dict(width=di.width, height=di.height, fps=dp[2],
                       fx=di.fx, fy=di.fy, cx=di.ppx, cy=di.ppy,
                       coeffs=list(di.coeffs), model=str(di.model)),
            color=(dict(width=ci.width, height=ci.height, fps=cp[2],
                        fx=ci.fx, fy=ci.fy, cx=ci.ppx, cy=ci.ppy,
                        coeffs=list(ci.coeffs), model=str(ci.model)) if ci else None),
            depth_to_color=ext,
            depth_scale=ds.get_depth_scale(),
            aligned=self.align_to_color,
        )
        # 用我们自己标定的彩色内参覆盖出厂值(出厂畸变全 0,不可用)
        if self.calib_intrinsics and self._meta['color']:
            k = self.calib_intrinsics
            if [self._meta['color']['width'], self._meta['color']['height']] == k['resolution']:
                self._meta['color'].update(fx=k['intrinsics'][0], fy=k['intrinsics'][1],
                                           cx=k['intrinsics'][2], cy=k['intrinsics'][3],
                                           coeffs=k['distortion_coeffs'],
                                           model='calibrated_radtan')
                self._meta['color_source'] = 'our_calibration'
        return prof

    def temperatures(self):
        try:
            ds = self._prof.get_device().first_depth_sensor()
            out = {}
            for o, k in ((rs.option.asic_temperature, 'asic'),
                         (rs.option.projector_temperature, 'projector')):
                if ds.supports(o):
                    out[k] = round(float(ds.get_option(o)), 1)
            return out
        except Exception:
            return {}

    def meta(self):
        return self._meta

    def _run(self):
        self._open()
        align = rs.align(rs.stream.color) if self.align_to_color else None
        seq = 0
        enc = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while not self._stop.is_set():
            try:
                fs = self._pipe.wait_for_frames(2000)
            except Exception:
                continue
            if align:
                fs = align.process(fs)
            df = fs.get_depth_frame()
            cf = fs.get_color_frame()
            t = time.time()
            if df:
                self.on_frame('depth', dict(
                    seq=seq, t=df.get_timestamp() / 1000.0,
                    arr=np.asanyarray(df.get_data()),
                    scale=self._meta['depth_scale']))
            irf = fs.get_infrared_frame(1) if self.with_ir else None
            if irf:
                a = np.asanyarray(irf.get_data())
                # 优先用 metadata 判断激光状态;拿不到就用高频能量兜底
                # (实测两类相差 115 倍组内标准差,不可能分错)
                laser = 1
                try:
                    if irf.supports_frame_metadata(rs.frame_metadata_value.frame_laser_power_mode):
                        laser = int(irf.get_frame_metadata(
                            rs.frame_metadata_value.frame_laser_power_mode))
                    elif self.emitter_alternate:
                        laser = 1 if cv2.Laplacian(a, cv2.CV_64F).var() > 300 else 0
                except Exception:
                    pass
                ok, buf = cv2.imencode('.jpg', a, enc)
                if ok:
                    self.on_frame('ir' if laser else 'ir_clean', dict(
                        seq=seq, t=irf.get_timestamp() / 1000.0, laser=laser,
                        w=a.shape[1], h=a.shape[0], jpeg=buf.tobytes()))
            if cf:
                img = np.asanyarray(cf.get_data())
                ok, buf = cv2.imencode('.jpg', img, enc)
                if ok:
                    self.on_frame('color', dict(
                        seq=seq, t=cf.get_timestamp() / 1000.0,
                        w=img.shape[1], h=img.shape[0], jpeg=buf.tobytes()))
            seq += 1
        if self._pipe:
            self._pipe.stop()
