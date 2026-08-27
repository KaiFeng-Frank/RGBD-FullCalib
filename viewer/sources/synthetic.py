#!/usr/bin/env python3
"""合成源:用已知几何造深度图。

用途有两个,都不是"没有相机时凑合":
  1. 相机被别的任务占着时也能开发渲染管线(RealSense 不允许两个进程同时打开设备)
  2. 更重要 —— 它是渲染正确性的**判据**。合成场景的几何是已知的:
     墙就是平面、立方体的棱就是直角。如果点云里墙弯了、直角不直,
     那一定是反投影或内参用错了,而不是"深度传感器就这样"。
     拿真实相机是分不清这两者的。
"""
import time

import cv2
import numpy as np

from .base import Source


class Synthetic(Source):
    name = 'synthetic'

    def __init__(self, on_frame, width=848, height=480, fps=10,
                 fx=423.46, fy=423.46, cx=419.71, cy=243.50):
        super().__init__(on_frame)
        self.W, self.H, self.fps = width, height, fps
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.scale = 0.001

    def temperatures(self):
        """模拟一条指数趋近的升温曲线,便于在没有真机时调试温漂界面。"""
        import time as _t
        if not hasattr(self, '_t0'):
            self._t0 = _t.time()
        el = _t.time() - self._t0
        asic = 30.0 + 25.0 * (1 - pow(2.718281828, -el / 120.0))
        return {'asic': round(asic, 1), 'projector': round(asic - 8.0, 1)}

    def meta(self):
        return dict(
            source='synthetic', kind='depth_image',
            depth=dict(width=self.W, height=self.H, fps=self.fps,
                       fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                       coeffs=[0.0] * 5, model='pinhole'),
            color=dict(width=self.W, height=self.H, fps=self.fps,
                       fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                       coeffs=[0.0] * 5, model='pinhole'),
            depth_to_color=None, depth_scale=self.scale, aligned=True,
            note='合成场景:背墙 2.0m + 地面 + 立方体 + 球。几何已知,可用于验证反投影正确性。')

    def _scene(self, phase):
        """光线步进画一个简单场景。返回 (depth_m, color_bgr)"""
        W, H = self.W, self.H
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        dx = (u - self.cx) / self.fx
        dy = (v - self.cy) / self.fy
        # 归一化视线方向
        n = np.sqrt(dx * dx + dy * dy + 1.0)
        rx, ry, rz = dx / n, dy / n, 1.0 / n

        INF = 1e9
        z = np.full((H, W), INF)
        col = np.zeros((H, W, 3), np.uint8)

        def hit(t, bgr):
            m = (t > 0.15) & (t < z)
            z[m] = t[m]
            col[m] = bgr

        # 背墙 z=2.0
        hit(np.where(rz > 1e-6, 2.0 / rz, INF), (170, 170, 170))
        # 地面 y=+0.6 (相机坐标 y 向下)
        hit(np.where(ry > 1e-6, 0.6 / ry, INF), (120, 150, 120))
        # 左墙 x=-1.2
        hit(np.where(rx < -1e-6, -1.2 / rx, INF), (150, 130, 120))

        # 立方体:轴对齐盒子,slab 求交
        c = np.array([0.25 * np.sin(phase), 0.25, 1.2])
        r = np.array([0.18, 0.18, 0.18])
        lo, hi = c - r, c + r
        d = np.stack([rx, ry, rz])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[:, None, None] - 0.0) / d
            t2 = (hi[:, None, None] - 0.0) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=0)
        tmax = np.nanmin(np.maximum(t1, t2), axis=0)
        box = (tmax >= np.maximum(tmin, 0))
        hit(np.where(box, tmin, INF), (60, 90, 220))

        # 球
        sc = np.array([-0.35, 0.2, 1.5])
        sr = 0.16
        b = rx * sc[0] + ry * sc[1] + rz * sc[2]
        cc = (sc ** 2).sum() - sr ** 2
        disc = b * b - cc
        sph = disc > 0
        ts = np.where(sph, b - np.sqrt(np.maximum(disc, 0)), INF)
        hit(ts, (80, 200, 240))

        # 光线步进得到的是「沿光线距离」range,而深度相机输出的是「z 深度」
        # (垂直于像平面的分量)。两者差一个 rz 因子,画面边缘能差 10% 以上。
        # 不转换的话平面会被渲染成球面 —— 这是深度相机里最经典的混淆。
        z = np.where(z >= INF, 0.0, z * rz)

        # 加一点符合实测的噪声:视差噪声 0.08px -> Δz = z²·Δd/(B·f)
        m = z > 0
        if m.any():
            sigma = (z[m] ** 2) * 0.08 / (0.05 * self.fx)
            z[m] += np.random.normal(0, sigma)
        return z, col

    def _run(self):
        seq = 0
        dt = 1.0 / self.fps
        enc = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        while not self._stop.is_set():
            t0 = time.time()
            zf, col = self._scene(seq * 0.05)
            depth = np.clip(zf / self.scale, 0, 65535).astype(np.uint16)
            self.on_frame('depth', dict(seq=seq, t=t0, arr=depth, scale=self.scale))
            ok, buf = cv2.imencode('.jpg', col, enc)
            if ok:
                self.on_frame('color', dict(seq=seq, t=t0, w=self.W, h=self.H,
                                            jpeg=buf.tobytes()))
            seq += 1
            time.sleep(max(0.0, dt - (time.time() - t0)))
