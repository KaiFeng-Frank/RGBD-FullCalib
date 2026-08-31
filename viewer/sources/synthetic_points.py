#!/usr/bin/env python3
"""Deterministic native point stream for renderer and protocol smoke tests."""
import time

import numpy as np

from .base import Source


class SyntheticPoints(Source):
    name = 'synthetic-points'

    def __init__(self, on_frame, fps=10):
        super().__init__(on_frame)
        self.fps = max(1, int(fps))
        self.xyz, self.intensity, self.rgb = self._make_scene()

    @staticmethod
    def _make_scene():
        rng = np.random.default_rng(360)

        # Viewer coordinates: x right, y up, z back; forward is negative z.
        gx, gz = np.meshgrid(np.linspace(-8, 8, 140), np.linspace(-0.5, -24, 140))
        ground = np.column_stack((gx.ravel(), np.full(gx.size, -1.2), gz.ravel()))

        wx, wy = np.meshgrid(np.linspace(-8, 8, 120), np.linspace(-1.2, 5, 55))
        wall = np.column_stack((wx.ravel(), wy.ravel(), np.full(wx.size, -24.0)))

        # Three vertical posts make orientation and parallax immediately obvious.
        posts = []
        for x, z in ((-3.5, -8.0), (0.0, -12.0), (4.0, -17.0)):
            a = rng.uniform(0, 2 * np.pi, 1600)
            y = rng.uniform(-1.2, 3.0, 1600)
            posts.append(np.column_stack((x + 0.18 * np.cos(a), y,
                                          z + 0.18 * np.sin(a))))
        xyz = np.vstack((ground, wall, *posts)).astype(np.float32)
        xyz += rng.normal(0.0, 0.008, xyz.shape).astype(np.float32)

        distance = np.linalg.norm(xyz, axis=1)
        intensity = np.clip(1.0 - distance / 32.0, 0.05, 1.0).astype(np.float32)
        rgb = np.empty((len(xyz), 3), np.uint8)
        rgb[:, 0] = np.clip(80 + 35 * xyz[:, 1], 30, 255)
        rgb[:, 1] = np.clip(180 - 5 * distance, 35, 220)
        rgb[:, 2] = np.clip(230 - 7 * distance, 40, 240)
        return xyz, intensity, rgb

    def meta(self):
        return dict(
            source='synthetic point stream', kind='point_stream',
            topic='(internal)', message_type='native xyz/intensity/rgb',
            frame_id='synthetic_lidar', fields=['x', 'y', 'z', 'intensity', 'rgb'],
            has_intensity=True, has_color=True, axes_input='viewer',
            axes_view='x-right, y-up, z-back', max_points=len(self.xyz),
            point_count_raw=len(self.xyz), qos='n/a', recommended_max_range=30.0,
            view_center=[0.0, 0.8, -12.0], view_distance=24.0,
            note='Known scene: ground plane, back wall and three vertical posts.')

    def _run(self):
        seq = 0
        dt = 1.0 / self.fps
        while not self._stop.is_set():
            t0 = time.time()
            self.on_frame('points', dict(
                seq=seq, t=t0, xyz=self.xyz,
                intensity=self.intensity, rgb=self.rgb))
            seq += 1
            self._stop.wait(max(0.0, dt - (time.time() - t0)))
