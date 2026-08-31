#!/usr/bin/env python3
import json
import os
import tempfile
import unittest
from unittest import mock

import numpy as np

import protocol
import server
from lidar_calib import TASKS
from test_calib_state import valid_result

try:
    from sources.d435i import D435i, _check_aligned_meta
except ModuleNotFoundError as exc:
    if exc.name != 'pyrealsense2':
        raise
    D435i = None
    _check_aligned_meta = None


TASK = next(item for item in TASKS if item['id'] == server.EXTRINSIC_TASK_ID)


def validated_document():
    doc = valid_result(TASK)
    doc['frame_convention']['equation'] = server.EXTRINSIC_EQUATION
    return doc


class ExtrinsicLoaderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, 'extrinsic.json')

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, doc):
        with open(self.path, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, allow_nan=False)

    def load_default(self):
        with mock.patch.object(server, 'EXTRINSIC_RESULT', self.path):
            return server.load_extrinsic_preview()

    def test_validated_default_must_pass_full_registry(self):
        self.write(validated_document())
        got = self.load_default()
        self.assertTrue(got['available'])
        self.assertEqual(got['status'], 'validated')
        self.assertEqual(got['equation'], server.EXTRINSIC_EQUATION)
        self.assertEqual(got['units'], 'metres')
        self.assertEqual(got['rig_id'], 'test-rig')
        self.assertEqual(got['mount_session_id'], 'test-mount-session')
        self.assertEqual(got['lidar_serial'], 'LIVOX-0001')
        self.assertEqual(got['camera_serial'], 'D435I-0001')

        doc = validated_document()
        doc['result']['holdout_projection_p95_px'] = 99.0
        # A naked status=validated must not bypass the frozen gate.
        self.write(doc)
        got = self.load_default()
        self.assertFalse(got['available'])
        self.assertEqual(got['status'], 'invalid')
        self.assertIn('registry validation failed', got['reason'])

    def test_draft_is_only_accepted_by_explicit_path(self):
        doc = validated_document()
        doc['status'] = 'draft'
        doc['draft_schema'] = 'd435i_calib/lidar_camera_extrinsic_draft/v1'
        doc.pop('validation')
        self.write(doc)

        self.assertFalse(self.load_default()['available'])
        got = server.load_extrinsic_preview(self.path)
        self.assertTrue(got['available'])
        self.assertEqual(got['status'], 'draft')
        self.assertEqual(got['label'], 'DRAFT')
        self.assertEqual(got['path'], os.path.abspath(self.path))

    def test_operational_local_file_needs_no_machine_registry(self):
        doc = validated_document()
        doc['status'] = 'operational'
        doc['local_schema'] = 'd435i_calib/lidar_camera_extrinsic_local/v1'
        doc.pop('validation')
        self.write(doc)

        got = server.load_extrinsic_preview(local_path=self.path)
        self.assertTrue(got['available'])
        self.assertEqual(got['status'], 'operational')
        self.assertEqual(got['label'], 'LOCAL')

        doc['status'] = 'validated'
        self.write(doc)
        with self.assertRaisesRegex(ValueError, 'status=operational'):
            server.load_extrinsic_preview(local_path=self.path)

    def test_wrong_direction_is_never_previewed(self):
        doc = validated_document()
        doc['frame_convention']['from'] = 'camera_color_optical_frame'
        doc['frame_convention']['to'] = 'livox_frame'
        self.write(doc)
        got = self.load_default()
        self.assertFalse(got['available'])
        self.assertIn('frame_convention', got['reason'])

    def test_missing_or_duplicate_device_identity_is_never_previewed(self):
        doc = validated_document()
        doc['devices'][0]['serial'] = ''
        self.write(doc)
        got = self.load_default()
        self.assertFalse(got['available'])
        self.assertIn('serial', got['reason'])

        doc = validated_document()
        doc['devices'][1]['role'] = 'lidar'
        self.write(doc)
        got = self.load_default()
        self.assertFalse(got['available'])
        self.assertIn('roles', got['reason'])

    def test_loaded_transform_is_bound_to_online_device_serials(self):
        self.write(validated_document())
        got = self.load_default()
        server.require_preview_device_serial(got, 'lidar', 'LIVOX-0001')
        server.require_preview_device_serial(got, 'rgbd', 'D435I-0001')
        with self.assertRaisesRegex(ValueError, 'online serial'):
            server.require_preview_device_serial(got, 'lidar', 'WRONG-LIDAR')
        with self.assertRaisesRegex(ValueError, 'unavailable'):
            server.require_preview_device_serial(got, 'rgbd', '')


class TransformDirectionTest(unittest.TestCase):
    def test_livox_ros_wire_to_color_optical_viewer(self):
        # Ros2Points wire mapping for p_livox=[1,2,3] is [-2,3,-1].
        wire = np.array([[-2.0, 3.0, -1.0]], np.float32)
        transform = np.eye(4)
        transform[:3, 3] = [0.1, 0.2, 0.3]
        got = server.transform_livox_viewer_to_camera_viewer(wire, transform)
        # p_camera=[1.1,2.2,3.3], then optical -> viewer [x,-y,-z].
        np.testing.assert_allclose(got, [[1.1, -2.2, -3.3]], atol=1e-6)

    def test_rotation_is_not_accidentally_inverted(self):
        wire = np.array([[0.0, 0.0, -1.0]], np.float32)  # p_livox=[1,0,0]
        transform = np.array([
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        got = server.transform_livox_viewer_to_camera_viewer(wire, transform)
        np.testing.assert_allclose(got, [[0.0, -1.0, 0.0]], atol=1e-6)


class LivoxIdentityTest(unittest.TestCase):
    def test_retained_device_info_is_strictly_parsed(self):
        payload = ('{"schema":"livox_ros_driver2/device_info/v1",'
                   '"serial_number":"ARMDN6B0030122",'
                   '"lidar_ip":"192.168.1.3"}\n---\n')
        got = server.parse_livox_device_info(payload)
        self.assertEqual(got['serial'], 'ARMDN6B0030122')
        self.assertEqual(got['lidar_ip'], '192.168.1.3')

    def test_bad_or_duplicate_identity_json_is_rejected(self):
        for payload in (
                '{"schema":"wrong","serial_number":"x"}',
                '{"schema":"livox_ros_driver2/device_info/v1"}',
                ('{"schema":"livox_ros_driver2/device_info/v1",'
                 '"serial_number":"a","serial_number":"b"}')):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                server.parse_livox_device_info(payload)


@unittest.skipIf(_check_aligned_meta is None,
                 'pyrealsense2 is unavailable in this interpreter')
class AlignedDepthMetadataTest(unittest.TestCase):
    def test_aligned_depth_and_color_geometry_must_match(self):
        depth = dict(width=1280, height=720, fx=640.0, fy=641.0,
                     cx=639.5, cy=359.5,
                     frame_id='camera_color_optical_frame')
        _check_aligned_meta(depth, dict(depth))
        bad = dict(depth, cx=638.5)
        with self.assertRaisesRegex(RuntimeError, 'intrinsics mismatch'):
            _check_aligned_meta(depth, bad)

    def test_pipeline_is_stopped_when_frame_processing_fails(self):
        class BrokenFrames:
            @staticmethod
            def get_depth_frame():
                raise RuntimeError('simulated unplug during frame processing')

        pipeline = mock.Mock()
        pipeline.wait_for_frames.return_value = BrokenFrames()
        source = D435i(lambda *_: None, align_to_color=False, with_ir=False)

        def fake_open():
            source._pipe = pipeline

        source._open = fake_open
        with self.assertRaisesRegex(RuntimeError, 'simulated unplug'):
            source._run()
        pipeline.stop.assert_called_once_with()


class ProtocolAndMarkupTest(unittest.TestCase):
    def test_meta_protocol_preserves_transform_direction(self):
        packet = protocol.pack_meta({
            'frame_id': 'camera_color_optical_frame',
            'input_frame_id': 'livox_frame',
            'units': 'metres',
            'transform_equation': server.EXTRINSIC_EQUATION,
        })
        self.assertEqual(packet[0], protocol.T_META)
        meta = json.loads(packet[1:].decode())
        self.assertEqual(meta['transform_equation'], server.EXTRINSIC_EQUATION)
        self.assertEqual(meta['units'], 'metres')

    def test_viewer_has_source_toggles_and_unambiguous_draft_warning(self):
        path = os.path.join(os.path.dirname(__file__), 'viewer.html')
        with open(path, encoding='utf-8') as stream:
            html = stream.read()
        self.assertIn('id="showRgbd"', html)
        self.assertIn('id="showLidar"', html)
        self.assertIn('id="draftWatermark"', html)
        self.assertIn('DRAFT · 未标定 · 几何叠加预览', html)
        self.assertIn('p_camera = T_camera_lidar * p_lidar', html)
        self.assertIn('当前 RIG 外参已载入 · 雷视融合', html)
        self.assertIn('双源实时点流 · 当前 rig 外参', html)

    def test_fused_view_keeps_rgbd_images_and_separates_stream_controls(self):
        path = os.path.join(os.path.dirname(__file__), 'viewer.html')
        with open(path, encoding='utf-8') as stream:
            html = stream.read()
        self.assertIn('html.overlay #bottom{display:grid}', html)
        self.assertNotIn('html.overlay #bottom{display:none}', html)
        for control in ('rgbdPsize', 'lidarPsize', 'rgbdStride',
                        'lidarStride', 'rgbdShade', 'lidarShade'):
            self.assertIn(f'id="{control}"', html)
        self.assertIn('OVERLAY?+$("rgbdStride").value', html)
        self.assertIn('OVERLAY?+$("lidarStride").value', html)
        self.assertIn('{depth:0,color:1,height:2,source:3}', html)
        self.assertIn('{depth:0,height:2,source:3,intensity:4}', html)
        self.assertIn('gl.uniform1i(U.uPalette,OVERLAY?1:0)', html)
        self.assertIn('gl.uniform1i(UN.uPalette,OVERLAY?2:0)', html)
        self.assertIn('log(max(d,lo)/lo)/log(hi/lo)', html)
        self.assertIn('深度 · 青→蓝', html)
        self.assertIn('距离 · 黄→红', html)


if __name__ == '__main__':
    unittest.main()
