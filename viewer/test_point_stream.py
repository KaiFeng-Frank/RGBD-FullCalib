#!/usr/bin/env python3
import array
import asyncio
import math
import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol
from server import Hub, broadcast_loop
from sources.ros2_points import Ros2Points, _structured_points, _to_view_axes
from sources.synthetic_points import SyntheticPoints


class PointProtocolTest(unittest.TestCase):
    def test_xyz_intensity_rgb_layout(self):
        xyz = np.array([[1, 2, 3], [4, 5, 6]], np.float32)
        intensity = np.array([0.25, 0.75], np.float32)
        rgb = np.array([[255, 10, 20], [30, 40, 50]], np.uint8)
        packet = protocol.pack_points(7, 12.5, xyz, intensity, rgb)
        kind, flags, count, seq, stamp = struct.unpack('>BB2xIId', packet[:20])
        self.assertEqual((kind, flags, count, seq), (protocol.T_POINTS, 3, 2, 7))
        self.assertEqual(stamp, 12.5)
        self.assertEqual(len(packet), 20 + 2 * (12 + 4 + 3))
        np.testing.assert_array_equal(np.frombuffer(packet, '<f4', 6, 20).reshape(-1, 3), xyz)
        np.testing.assert_array_equal(np.frombuffer(packet, '<f4', 2, 44), intensity)
        np.testing.assert_array_equal(np.frombuffer(packet, np.uint8, 6, 52).reshape(-1, 3), rgb)

    def test_shape_validation(self):
        with self.assertRaises(ValueError):
            protocol.pack_points(0, 0, np.zeros((3, 2), np.float32))
        with self.assertRaises(ValueError):
            protocol.pack_points(0, 0, np.zeros((3, 3), np.float32), np.zeros(2))

    def test_synthetic_native_source_contract(self):
        src = SyntheticPoints(lambda *_: None, fps=10)
        meta = src.meta()
        self.assertEqual(meta['kind'], 'point_stream')
        self.assertEqual(src.xyz.shape[1], 3)
        self.assertEqual(len(src.xyz), len(src.intensity))
        self.assertEqual(src.rgb.shape, src.xyz.shape)


class AxisTest(unittest.TestCase):
    def test_ros_and_optical_mapping(self):
        p = np.array([[1.0, 2.0, 3.0]], np.float32)
        np.testing.assert_array_equal(_to_view_axes(p, 'ros'), [[-2, 3, -1]])
        np.testing.assert_array_equal(_to_view_axes(p, 'optical'), [[1, -2, -3]])
        np.testing.assert_array_equal(_to_view_axes(p, 'viewer'), p)

    def test_intensity_normalisation_sanitises_nan(self):
        src = Ros2Points(lambda *_: None)
        got = src._normalise_intensity(np.array([0.0, 1.0, np.nan], np.float32))
        self.assertTrue(np.isfinite(got).all())
        self.assertTrue(((0.0 <= got) & (got <= 1.0)).all())

    def test_repeated_or_empty_cloud_cannot_fake_stream_freshness(self):
        src = Ros2Points(lambda *_: None)
        self.assertTrue(src._advances_stream(True, 100))
        self.assertFalse(src._advances_stream(True, 100))
        self.assertFalse(src._advances_stream(True, 99))
        self.assertFalse(src._advances_stream(False, 101))
        self.assertTrue(src._advances_stream(True, 101))

        unstamped = Ros2Points(lambda *_: None)
        self.assertTrue(unstamped._advances_stream(True, None))
        self.assertFalse(unstamped._advances_stream(True, None))


class SourceFailureTest(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def running_source(error=None):
        return type('RunningSource', (), {
            'error': error,
            'is_alive': lambda _: True,
            'stop_requested': lambda _: False,
        })()

    async def test_runtime_source_failure_stops_broadcast(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = type('FailedSource', (), {
            'error': RuntimeError('decode failed'),
            'is_alive': lambda _: False,
            'stop_requested': lambda _: False,
        })()
        with self.assertRaisesRegex(RuntimeError, 'decode failed'):
            await asyncio.wait_for(broadcast_loop(hub, 30), timeout=1.0)

    async def test_clean_but_unexpected_source_exit_stops_broadcast(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = type('StoppedSource', (), {
            'error': None,
            'is_alive': lambda _: False,
            'stop_requested': lambda _: False,
        })()
        with self.assertRaisesRegex(RuntimeError, '意外停止'):
            await asyncio.wait_for(broadcast_loop(hub, 30), timeout=1.0)

    async def test_live_thread_with_stale_frames_is_treated_as_disconnected(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = self.running_source()
        hub.last_frame_monotonic = __import__('time').monotonic() - 2.0
        with self.assertRaisesRegex(RuntimeError, '无帧.*退出'):
            await asyncio.wait_for(
                broadcast_loop(hub, 30, stream_timeout=0.05), timeout=1.0)

    async def test_recent_frames_do_not_false_trigger_watchdog(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = self.running_source()
        hub.on_frame('points', {'seq': 1})
        task = asyncio.create_task(
            broadcast_loop(hub, 30, stream_timeout=0.5))
        await asyncio.sleep(0.08)
        self.assertFalse(task.done())
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task

    async def test_empty_display_frame_does_not_refresh_stream_health(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = self.running_source()
        hub.on_frame('points', {'seq': 1})
        first_freshness = hub.last_frame_monotonic
        hub.on_frame('points', {
            'seq': 2,
            '_counts_as_freshness': False,
        })
        self.assertEqual(hub.last_frame_monotonic, first_freshness)
        hub.last_frame_monotonic -= 2.0
        with self.assertRaisesRegex(RuntimeError, '无帧.*退出'):
            await asyncio.wait_for(
                broadcast_loop(hub, 30, stream_timeout=0.05), timeout=1.0)

    async def test_source_error_takes_priority_over_stale_frame(self):
        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = self.running_source(RuntimeError('decode wins'))
        hub.last_frame_monotonic = __import__('time').monotonic() - 2.0
        with self.assertRaisesRegex(RuntimeError, 'decode wins'):
            await asyncio.wait_for(
                broadcast_loop(hub, 30, stream_timeout=0.05), timeout=1.0)

    async def test_non_reading_websocket_cannot_block_disconnect_watchdog(self):
        class BlockedClient:
            async def send(self, _message):
                await asyncio.Event().wait()

        hub = Hub()
        hub.wake = asyncio.Event()
        hub.src = self.running_source()
        hub.clients.add(BlockedClient())
        hub.on_frame('points', {
            'seq': 1, 't': 1.0,
            'xyz': np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        })
        hub.wake.set()
        with self.assertRaisesRegex(RuntimeError, '无帧.*退出'):
            await asyncio.wait_for(
                broadcast_loop(hub, 30, stream_timeout=0.05), timeout=2.0)


try:
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError:  # Non-ROS Conda runs still exercise protocol and renderer source.
    PointCloud2 = PointField = None


@unittest.skipIf(PointCloud2 is None, 'ROS 2 sensor_msgs not available in this interpreter')
class PointCloud2DecodeTest(unittest.TestCase):
    @staticmethod
    def field(name, offset, datatype):
        return PointField(name=name, offset=offset, datatype=datatype, count=1)

    def test_padded_organized_cloud_and_optional_fields(self):
        fields = [
            self.field('x', 0, PointField.FLOAT32),
            self.field('y', 4, PointField.FLOAT32),
            self.field('z', 8, PointField.FLOAT32),
            self.field('reflectivity', 12, PointField.UINT8),
            self.field('rgb', 16, PointField.UINT32),
        ]
        width, height, point_step, row_step = 2, 2, 20, 44
        buf = bytearray(row_step * height)
        values = [
            (1.0, 2.0, 3.0, 10, 0xFF1020),
            (0.0, 0.0, 0.0, 20, 0x304050),  # invalid origin
            (4.0, 5.0, 6.0, 30, 0x607080),
            (math.nan, 8.0, 9.0, 40, 0x90A0B0),
        ]
        for i, value in enumerate(values):
            row, col = divmod(i, width)
            struct.pack_into('<fffB3xI', buf, row * row_step + col * point_step, *value)
        msg = PointCloud2(height=height, width=width, fields=fields,
                          is_bigendian=False, point_step=point_step,
                          row_step=row_step, data=array.array('B', buf), is_dense=True)
        src = Ros2Points(lambda *_: None, max_points=100)
        xyz, intensity, rgb, names = src._pointcloud2_arrays(msg)
        np.testing.assert_allclose(xyz, [[1, 2, 3], [4, 5, 6]])
        self.assertEqual(names, ['x', 'y', 'z', 'reflectivity', 'rgb'])
        self.assertEqual(intensity.shape, (2,))
        np.testing.assert_array_equal(rgb, [[255, 16, 32], [96, 112, 128]])

    def test_big_endian_float64(self):
        fields = [
            self.field('x', 0, PointField.FLOAT64),
            self.field('y', 8, PointField.FLOAT64),
            self.field('z', 16, PointField.FLOAT64),
        ]
        payload = struct.pack('>ddd', 1.25, -2.5, 3.75)
        msg = PointCloud2(height=1, width=1, fields=fields,
                          is_bigendian=True, point_step=24, row_step=24,
                          data=array.array('B', payload), is_dense=True)
        decoded = _structured_points(msg)
        self.assertAlmostEqual(float(decoded['x'][0]), 1.25)
        src = Ros2Points(lambda *_: None)
        xyz, intensity, rgb, _ = src._pointcloud2_arrays(msg)
        np.testing.assert_allclose(xyz, [[1.25, -2.5, 3.75]])
        self.assertIsNone(intensity)
        self.assertIsNone(rgb)

    def test_rejects_overlapping_rows(self):
        fields = [
            self.field('x', 0, PointField.FLOAT32),
            self.field('y', 4, PointField.FLOAT32),
            self.field('z', 8, PointField.FLOAT32),
        ]
        msg = PointCloud2(height=2, width=2, fields=fields,
                          is_bigendian=False, point_step=12, row_step=12,
                          data=array.array('B', bytes(48)), is_dense=True)
        with self.assertRaisesRegex(ValueError, 'row_step too small'):
            _structured_points(msg)

    def test_uint8_count3_rgb_field(self):
        fields = [
            self.field('x', 0, PointField.FLOAT32),
            self.field('y', 4, PointField.FLOAT32),
            self.field('z', 8, PointField.FLOAT32),
            PointField(name='rgb', offset=12, datatype=PointField.UINT8, count=3),
        ]
        payload = struct.pack('<fffBBB', 1.0, 2.0, 3.0, 12, 34, 56)
        msg = PointCloud2(height=1, width=1, fields=fields,
                          is_bigendian=False, point_step=15, row_step=15,
                          data=array.array('B', payload), is_dense=True)
        src = Ros2Points(lambda *_: None)
        xyz, intensity, rgb, _ = src._pointcloud2_arrays(msg)
        np.testing.assert_allclose(xyz, [[1, 2, 3]])
        self.assertIsNone(intensity)
        np.testing.assert_array_equal(rgb, [[12, 34, 56]])

    def test_empty_cloud_is_legal_and_does_not_freeze_meta(self):
        fields = [
            self.field('x', 0, PointField.FLOAT32),
            self.field('y', 4, PointField.FLOAT32),
            self.field('z', 8, PointField.FLOAT32),
        ]
        msg = PointCloud2(height=1, width=0, fields=fields,
                          is_bigendian=False, point_step=12, row_step=0,
                          data=array.array('B'), is_dense=True)
        src = Ros2Points(lambda *_: self.fail('startup empty cloud must not emit'))
        xyz, intensity, rgb, _ = src._pointcloud2_arrays(msg)
        self.assertEqual(xyz.shape, (0, 3))
        self.assertIsNone(intensity)
        self.assertIsNone(rgb)
        src.topic = '/empty'
        src._emit(msg, 'sensor_msgs/msg/PointCloud2')
        self.assertIsNone(src.meta())


if __name__ == '__main__':
    unittest.main()
