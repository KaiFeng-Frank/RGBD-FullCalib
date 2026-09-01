#!/usr/bin/env python3
import hashlib
import json
import os
import tempfile
import unittest

from lidar_calib import (ACTIVE_TASK_IDS, CURRENT_RIG_MANIFEST,
                         D435I_DEPTH_IMU_CONVENTION, EXTRINSIC_EQUATION,
                         IMU_CLOCK_CONVENTION, LIDAR_DEPTH_TIME_CONVENTION,
                         LIDAR_IMU_EQUATION, LIDAR_IMU_OPERATIONAL_SCHEMA,
                         LIDAR_IMU_TIME_CONVENTION, LOCAL_EXTRINSIC_RESULT,
                         MID360S_IMU_ACCEL_EQUATION,
                         MID360S_IMU_OPERATIONAL_SCHEMA, SPECS, TASKS,
                         TIMESYNC_OPERATIONAL_SCHEMA, _matrix4,
                         collect_lidar, lidar_references)


IDENTITY4 = [[1, 0, 0, 0], [0, 1, 0, 0],
             [0, 0, 1, 0], [0, 0, 0, 1]]


def _required_value(kind, task):
    if kind == 'matrix4':
        return IDENTITY4
    if kind == 'vector3':
        return [0.01, -0.01, 0.02]
    if kind == 'positive_vector3':
        return [1.0, 1.0, 1.0]
    if kind == 'positive':
        return 0.001
    if kind == 'number':
        return 1.0
    if kind == 'integer':
        return 1
    if kind == 'ratio':
        return 1.0
    if kind == 'distance_coverage':
        return [1.0, 3.0, 5.0, 10.0]
    if kind == 'hash_map':
        return {task_id: 'f' * 64 for task_id in task['spec']['depends_on']}
    raise AssertionError(kind)


def valid_result(task):
    spec = SPECS[task['id']]
    task_with_spec = dict(task, spec=spec)
    result = {field: _required_value(kind, task_with_spec)
              for field, kind in spec['required'].items()}
    result.update(spec.get('literals', {}))
    if 'accel_unit_scale_ms2_per_g' in result:
        result['accel_unit_scale_ms2_per_g'] = 9.80665
    if 'clock_model_a' in result:
        result['clock_model_a'] = 1.0
    checks = []
    for gate in spec['gates']:
        kind = spec['required'].get(gate['field'])
        if kind == 'integer':
            value = int(gate['limit']) + 1
        elif kind == 'ratio':
            value = (gate['limit'] * 0.5 if gate['op'] == 'le'
                     else min(1.0, gate['limit'] + 0.1))
        else:
            value = (gate['limit'] * 0.5 if gate['op'] == 'le'
                     else gate['limit'] + max(abs(gate['limit']) * 0.1, 0.1))
        result[gate['field']] = value
        checks.append(dict(id=gate['id'], value=value, status='passed',
                           detail='unit-test holdout passed'))
    for rule in spec.get('derived', ()):
        if rule['formula'] == 'abs_delta':
            value = abs(result[rule['source']] - rule['reference'])
        elif rule['formula'] == 'abs_minus_one_ppm':
            value = abs(result[rule['source']] - 1.0) * 1e6
        else:
            raise AssertionError(rule['formula'])
        result[rule['field']] = value
        next(x for x in checks if x['id'] == rule['field'])['value'] = value
    models = {'lidar': 'Livox Mid-360S', 'rgbd': 'Intel RealSense D435i'}
    serials = {'lidar': 'LIVOX-0001', 'rgbd': 'D435I-0001'}
    devices = [dict(role=role, model=models[role], serial=serials[role])
               for role in spec['roles']]
    hash_base = ord('c') if task['id'] == 'mid360s_validation' else ord('a')
    sources = [dict(role=role, path=f'data/{task["id"]}-{role}.bag',
                    sha256=chr(hash_base + i) * 64)
               for i, role in enumerate(spec['source_roles'])]
    return {
        'schema_version': 1,
        'task_id': task['id'],
        'status': 'validated',
        'devices': devices,
        'rig_id': 'test-rig',
        'mount_session_id': 'test-mount-session',
        'created_utc': '2026-08-31T12:00:00Z',
        'method': 'unit-test fixture',
        'source_data': sources,
        'frame_convention': (
            {name: {'from': pair[0], 'to': pair[1]}
             for name, pair in spec['frames'].items()}
            if isinstance(spec['frames'], dict)
            else {'from': spec['frames'][0], 'to': spec['frames'][1]}),
        'result': result,
        'summary': {'测试量': 'registered gates passed'},
        'validation': {'status': 'passed', 'checks': checks},
    }


def operational_result(task):
    doc = valid_result(task)
    doc['status'] = 'operational'
    doc['local_schema'] = 'd435i_calib/lidar_camera_extrinsic_local/v1'
    doc['frame_convention']['equation'] = EXTRINSIC_EQUATION
    doc.pop('validation')
    doc['source_data'] = [{
        'role': 'operational_fit',
        'path': 'data/local-fit/scene01.npz',
        'sha256': 'e' * 64,
    }]
    doc['result']['independent_holdout_available'] = False
    return doc


def operational_aux_result(task):
    doc = valid_result(task)
    doc['status'] = 'operational'
    doc.pop('validation')
    doc['source_data'] = [{
        'role': 'operational_capture',
        'path': f'data/{task["id"]}/run1',
        'sha256': 'e' * 64,
    }]
    if task['id'] == 'mid360s_imu':
        doc['local_schema'] = MID360S_IMU_OPERATIONAL_SCHEMA
        doc['frame_convention'] = {
            'frame': 'mid360s_imu_frame',
            'accel_equation': MID360S_IMU_ACCEL_EQUATION,
        }
        doc['result'] = {
            'accel_input_unit': 'g',
            'accel_output_unit': 'm/s^2',
            'accel_unit_scale_ms2_per_g': 9.80665,
            'accel_bias_ms2': [0.01, -0.02, 0.03],
            'accel_scale': [1.001, 0.999, 1.002],
            'accel_misalignment_rad': [0.001, -0.002, 0.003],
            'T_misalignment': [
                [1.0, -0.001, -0.002],
                [0.0, 1.0, -0.003],
                [0.0, 0.0, 1.0],
            ],
            'accel_correction_matrix': [
                [1.001, -0.000999, -0.002004],
                [0.0, 0.999, -0.003006],
                [0.0, 0.0, 1.002],
            ],
            'gyro_bias_rad_s': [0.001, -0.001, 0.0005],
            'pose_count': 15,
            'fit_pose_count': 12,
            'holdout_pose_count': 3,
            'gravity_reference_ms2': 9.78767,
            'accel_fit_residual_rms_ms2': 0.02,
            'accel_holdout_residual_rms_ms2': 0.03,
            'gyro_static_residual_rms_rad_s': 0.002,
            'imu_sample_rate_hz': 200.0,
            'noise_window_duration_s': 30.0,
            'accel_noise_density_ms2_sqrt_hz': [0.02, 0.021, 0.019],
            'gyro_noise_density_rad_s_sqrt_hz': [0.001, 0.0011, 0.0009],
            'noise_density_method': 'short_window_white_noise',
            'allan_characterization': 'not_performed',
        }
    elif task['id'] == 'mid360s_lidar_imu':
        doc['local_schema'] = LIDAR_IMU_OPERATIONAL_SCHEMA
        doc['frame_convention'] = {
            'from': 'mid360s_imu_frame', 'to': 'livox_frame',
            'equation': LIDAR_IMU_EQUATION,
        }
        doc['result'].update({
            'T_lidar_imu': IDENTITY4,
            'time_offset_lidar_to_imu_ms': 0.0,
            'time_offset_convention': LIDAR_IMU_TIME_CONVENTION,
            'deskew_mode': 'rotation_only',
            'per_point_time_field': 'offset_time',
            'offset_time_valid_ratio': 1.0,
        })
    elif task['id'] == 'mid360s_d435i_td':
        doc['local_schema'] = TIMESYNC_OPERATIONAL_SCHEMA
        doc['frame_convention'] = {
            'from': 'livox_scan', 'to': 'd435i_depth',
            'equation': LIDAR_DEPTH_TIME_CONVENTION,
        }
        doc['result'].update({
            'imu_clock_offset_livox_to_d435i_ms': -1.94,
            'd435i_depth_to_imu_timeshift_ms': 4.0494159,
            'time_offset_lidar_to_d435_depth_ms': -5.9894159,
            'imu_clock_offset_convention': IMU_CLOCK_CONVENTION,
            'd435i_depth_to_imu_timeshift_convention':
                D435I_DEPTH_IMU_CONVENTION,
            'time_offset_lidar_to_d435_depth_convention':
                LIDAR_DEPTH_TIME_CONVENTION,
            'segment_offsets_ms': [-1.91, -2.62, -1.56],
            'residual_rmse_rad_s': 0.0184,
            'matched_samples': 12910,
        })
    else:
        raise AssertionError(task['id'])
    return doc


class LidarCalibrationLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        os.makedirs(os.path.join(self.tmp.name, 'results'))

    def tearDown(self):
        self.tmp.cleanup()

    def path(self, task):
        return os.path.join(self.tmp.name, task['result_file'])

    def write(self, task, doc):
        with open(self.path(task), 'w', encoding='utf-8') as f:
            json.dump(doc, f, allow_nan=True, sort_keys=True)

    def write_current_rig(self, doc=None):
        path = os.path.join(self.tmp.name, CURRENT_RIG_MANIFEST)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        manifest = doc or {
            'schema': 'd435i_calib/lidar_camera_mount_session/v1',
            'rig_id': 'test-rig',
            'mount_session_id': 'test-mount-session',
            'd435i_serial': 'D435I-0001',
            'mid360s_serial': 'LIVOX-0001',
        }
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(manifest, stream, sort_keys=True)

    def write_operational(self, task, doc=None):
        path = os.path.join(self.tmp.name, LOCAL_EXTRINSIC_RESULT)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(doc or operational_result(task), stream,
                      allow_nan=True, sort_keys=True)

    def digest(self, task):
        with open(self.path(task), 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()

    def assert_partition(self, stages, pending):
        done = [x['id'] for x in stages]
        todo = [x['id'] for x in pending]
        self.assertEqual(len(done) + len(todo), len(TASKS))
        self.assertEqual(len(done + todo), len(set(done + todo)))
        self.assertFalse(set(done) & set(todo))
        self.assertEqual(set(done) | set(todo), {x['id'] for x in TASKS})

    def write_complete_chain(self, bad_final_hash=False):
        for task in TASKS[:-1]:
            self.write(task, valid_result(task))
        final = valid_result(TASKS[-1])
        final['result']['upstream_sha256'] = {
            task['id']: self.digest(task)
            for task in TASKS[:-1]
        }
        if bad_final_hash:
            final['result']['upstream_sha256'][TASKS[0]['id']] = '0' * 64
        self.write(TASKS[-1], final)

    def test_all_missing_are_todos(self):
        stages, pending = collect_lidar(self.tmp.name)
        self.assertEqual(stages, [])
        self.assertTrue(all(x['lifecycle'] == 'pending' for x in pending))
        self.assert_partition(stages, pending)

    def test_validated_result_moves_exactly_once(self):
        task = TASKS[0]
        self.write(task, valid_result(task))
        stages, pending = collect_lidar(self.tmp.name)
        self.assertEqual([x['id'] for x in stages], [task['id']])
        self.assertNotIn(task['id'], {x['id'] for x in pending})
        self.assert_partition(stages, pending)

    def test_draft_stays_pending(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['status'] = 'draft'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertEqual(item['lifecycle'], 'pending')
        self.assertIn('validated', item['progress'])
        self.assert_partition(stages, pending)

    def test_current_rig_operational_result_is_local_not_todo(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_d435i_ext')
        self.write_current_rig()
        self.write_operational(task)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in stages if x['id'] == task['id'])
        self.assertNotIn(task['id'], {x['id'] for x in pending})
        self.assertEqual(item['quality'], 'local')
        self.assertEqual(item['quality_label'], 'LOCAL')
        self.assertEqual(item['source'], LOCAL_EXTRINSIC_RESULT)
        self.assertEqual(item['rows'][0], ['质量', 'LOCAL'])
        self.assertIn('五场景稠密配准与多起点一致性已完成', item['note'])
        self.assertEqual(item['checks'], [])
        self.assert_partition(stages, pending)

    def test_current_rig_operational_aux_results_skip_generic_gates(self):
        self.write_current_rig()
        for task_id in ('mid360s_imu', 'mid360s_lidar_imu',
                        'mid360s_d435i_td'):
            task = next(item for item in TASKS if item['id'] == task_id)
            self.write(task, operational_aux_result(task))
            stages, pending = collect_lidar(self.tmp.name)
            item = next(x for x in stages if x['id'] == task_id)
            self.assertEqual(item['quality'], 'local')
            self.assertEqual(item['quality_label'], 'LOCAL')
            self.assertEqual(item['source'], task['result_file'])
            self.assertEqual(item['checks'], [])
            # Their generic prerequisites are intentionally absent: this
            # current-rig lane must not be demoted by VALIDATED catalog gates.
            self.assertNotIn(task_id, {x['id'] for x in pending})
            self.assert_partition(stages, pending)

    def test_mid360s_imu_operational_core_and_noise_claim_are_checked(self):
        self.write_current_rig()
        task = next(item for item in TASKS if item['id'] == 'mid360s_imu')
        doc = operational_aux_result(task)
        doc['result']['accel_unit_scale_ms2_per_g'] = 1.0
        doc['result']['pose_count'] = 8
        doc['result']['accel_holdout_residual_rms_ms2'] = float('nan')
        doc['result']['noise_density_method'] = 'allan_deviation'
        doc['result']['allan_characterization'] = '2h_allan'
        doc['result']['allan_duration_h'] = 2.0
        doc['result']['accel_correction_matrix'][0][0] = 2.0
        doc['source_data'][0]['sha256'] = 'not-a-sha256'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('sha256 必须为 64 位十六进制', item['progress'])
        self.assertIn('accel_unit_scale_ms2_per_g 必须为 9.80665',
                      item['progress'])
        self.assertIn('pose_count 必须是 >= 13', item['progress'])
        self.assertIn('result 含 NaN/Inf', item['progress'])
        self.assertIn('noise_density_method 必须为 short_window_white_noise',
                      item['progress'])
        self.assertIn('accel_correction_matrix 与 accel_scale', item['progress'])
        self.assertIn('不能冒充 2 h Allan', item['progress'])
        self.assert_partition(stages, pending)

    def test_aux_operational_result_is_bound_to_current_rig(self):
        self.write_current_rig()
        task = next(item for item in TASKS
                    if item['id'] == 'mid360s_d435i_td')
        doc = operational_aux_result(task)
        doc['mount_session_id'] = 'another-mount'
        doc['devices'][0]['serial'] = 'ANOTHER-LIDAR'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('mount_session_id 与当前 rig', item['progress'])
        self.assertIn('lidar serial 与当前 rig', item['progress'])
        self.assert_partition(stages, pending)

    def test_aux_operational_result_requires_current_rig_manifest(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_imu')
        self.write(task, operational_aux_result(task))
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('当前 rig 清单不存在', item['progress'])
        self.assertNotIn(task['id'], {x['id'] for x in stages})
        self.assert_partition(stages, pending)

    def test_timesync_operational_composition_and_sign_are_checked(self):
        self.write_current_rig()
        task = next(item for item in TASKS
                    if item['id'] == 'mid360s_d435i_td')
        doc = operational_aux_result(task)
        doc['result']['time_offset_lidar_to_d435_depth_ms'] = 5.9894159
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('必须等于 IMU 时钟偏移减去', item['progress'])
        self.assert_partition(stages, pending)

    def test_active_catalog_and_reference_catalog_are_disjoint(self):
        active = set(ACTIVE_TASK_IDS)
        references = lidar_references()
        reference_ids = {item['id'] for item in references}
        all_ids = {item['id'] for item in TASKS}
        self.assertEqual(
            active,
            {'mid360s_d435i_ext', 'mid360s_imu', 'mid360s_lidar_imu',
             'mid360s_d435i_td'},
        )
        self.assertFalse(active & reference_ids)
        self.assertEqual(active | reference_ids, all_ids)
        self.assertTrue(all('applies' in item for item in references))

    def test_operational_result_must_match_current_rig(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_d435i_ext')
        self.write_current_rig()
        doc = operational_result(task)
        doc['mount_session_id'] = 'another-mount'
        doc['devices'][1]['serial'] = 'ANOTHER-D435I'
        self.write_operational(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertEqual(item['lifecycle'], 'pending')
        self.assertIn('mount_session_id 与当前 rig', item['progress'])
        self.assertIn('rgbd serial 与当前 rig', item['progress'])
        self.assert_partition(stages, pending)

    def test_local_status_never_masquerades_as_validated(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_d435i_ext')
        self.write_current_rig()
        doc = operational_result(task)
        doc['status'] = 'validated'
        self.write_operational(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        self.assertNotIn(task['id'], {x['id'] for x in stages})
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('status 必须为 operational', item['progress'])
        self.assert_partition(stages, pending)

    def test_canonical_validated_result_has_priority_over_local(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_d435i_ext')
        self.write_current_rig()
        self.write_operational(task)
        canonical = valid_result(task)
        self.write(task, canonical)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in stages if x['id'] == task['id'])
        self.assertEqual(item['quality'], 'ok')
        self.assertEqual(item['quality_label'], 'VALIDATED')
        self.assertEqual(item['source'], task['result_file'])
        self.assertNotIn(task['id'], {x['id'] for x in pending})
        self.assert_partition(stages, pending)

    def test_lidar_camera_result_requires_mount_session_identity(self):
        task = next(item for item in TASKS if item['id'] == 'mid360s_d435i_ext')
        doc = valid_result(task)
        doc.pop('mount_session_id')
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('mount_session_id', item['progress'])
        self.assert_partition(stages, pending)

    def test_failed_registered_gate_is_rework(self):
        task = TASKS[0]
        doc = valid_result(task)
        gate = SPECS[task['id']]['gates'][0]
        value = gate['limit'] + 1
        doc['result'][gate['field']] = value
        doc['validation']['checks'][0].update(value=value, status='failed')
        doc['validation']['status'] = 'failed'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertEqual(item['lifecycle'], 'rework')
        self.assertIn('未过预注册阈值', item['progress'])
        self.assert_partition(stages, pending)

    def test_nan_result_cannot_graduate(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['result']['range_bias_abs_mm'] = float('nan')
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('NaN/Inf', item['progress'])
        self.assert_partition(stages, pending)

    def test_result_for_another_task_cannot_graduate(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['task_id'] = TASKS[1]['id']
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('task_id', item['progress'])
        self.assert_partition(stages, pending)

    def test_bad_json_is_local_failure(self):
        task = TASKS[0]
        with open(self.path(task), 'w', encoding='utf-8') as f:
            f.write('{broken')
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('不可解析', item['progress'])
        self.assert_partition(stages, pending)

    def test_non_object_result_is_local_pending_not_summary_crash(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['result'] = [1, 2, 3]
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('非空 object', item['progress'])
        self.assert_partition(stages, pending)

    def test_scale_or_shear_matrix_is_not_a_rigid_transform(self):
        scaled = [[1.006, 0, 0, 0], [0, 1.006, 0, 0],
                  [0, 0, 1.006, 0], [0, 0, 0, 1]]
        sheared = [[1, 0.019, 0, 0], [0, 1, 0, 0],
                   [0, 0, 1, 0], [0, 0, 0, 1]]
        self.assertTrue(_matrix4(IDENTITY4))
        self.assertFalse(_matrix4(scaled))
        self.assertFalse(_matrix4(sheared))

    def test_wrong_model_and_frame_are_rejected(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['devices'][0]['model'] = 'WRONG MODEL'
        doc['frame_convention'] = {'from': 'foo', 'to': 'bar'}
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('model', item['progress'])
        self.assertIn('frame_convention', item['progress'])
        self.assert_partition(stages, pending)

    def test_source_data_must_be_independent_typed_list(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['source_data'] = {'not': 'a list'}
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('非空列表', item['progress'])
        self.assert_partition(stages, pending)

        doc = valid_result(task)
        doc['source_data'][1]['sha256'] = doc['source_data'][0]['sha256']
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('不得引用同一数据哈希', item['progress'])

    def test_missing_or_duplicate_gate_id_is_rejected(self):
        task = TASKS[0]
        doc = valid_result(task)
        doc['validation']['checks'].pop()
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('恰好覆盖', item['progress'])

        doc = valid_result(task)
        doc['validation']['checks'].append(dict(doc['validation']['checks'][0]))
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('重复 id', item['progress'])

    def test_capture_protocol_fields_are_required(self):
        health = TASKS[0]
        doc = valid_result(health)
        doc['result']['distance_targets_m'] = [1.0, 3.0, 5.0]
        self.write(health, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == health['id'])
        self.assertIn('distance_coverage', item['progress'])

        imu = TASKS[1]
        doc = valid_result(imu)
        del doc['result']['accel_unit_scale_ms2_per_g']
        self.write(imu, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == imu['id'])
        self.assertIn('accel_unit_scale', item['progress'])

    def test_lidar_imu_requires_explicit_time_offset_and_sign(self):
        for task in TASKS[:2]:
            self.write(task, valid_result(task))
        task = TASKS[2]
        doc = valid_result(task)
        del doc['result']['time_offset_lidar_to_imu_ms']
        doc['result']['time_offset_convention'] = 'opposite sign'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('time_offset_lidar_to_imu_ms', item['progress'])
        self.assertIn('time_offset_convention', item['progress'])

    def test_clock_scale_error_is_recomputed_from_affine_model(self):
        health, task = TASKS[0], TASKS[4]
        self.write(health, valid_result(health))
        doc = valid_result(task)
        doc['result']['clock_model_a'] = 0.9999
        doc['result']['clock_scale_error_ppm_abs'] = 0.0
        check = next(x for x in doc['validation']['checks']
                     if x['id'] == 'clock_scale_error_ppm_abs')
        check['value'] = 0.0
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertEqual(item['lifecycle'], 'pending')
        self.assertIn('不一致', item['progress'])

    def test_rig_base_defines_both_transform_directions(self):
        for task in (TASKS[0], TASKS[3]):
            self.write(task, valid_result(task))
        task = TASKS[5]
        doc = valid_result(task)
        doc['frame_convention']['T_base_camera']['from'] = 'ambiguous_camera'
        self.write(task, doc)
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertIn('T_base_camera', item['progress'])

    def test_final_validation_waits_for_all_upstream_tasks(self):
        task = TASKS[-1]
        self.write(task, valid_result(task))
        stages, pending = collect_lidar(self.tmp.name)
        item = next(x for x in pending if x['id'] == task['id'])
        self.assertEqual(item['lifecycle'], 'pending')
        self.assertIn('上游任务未完成', item['progress'])
        self.assert_partition(stages, pending)

    def test_complete_chain_and_exact_upstream_hashes_graduate(self):
        self.write_complete_chain()
        stages, pending = collect_lidar(self.tmp.name)
        self.assertEqual(len(stages), len(TASKS))
        self.assertEqual(pending, [])
        self.assert_partition(stages, pending)

    def test_changed_upstream_artifact_reopens_final_validation(self):
        self.write_complete_chain(bad_final_hash=True)
        stages, pending = collect_lidar(self.tmp.name)
        final = next(x for x in pending if x['id'] == 'mid360s_validation')
        self.assertEqual(final['lifecycle'], 'rework')
        self.assertIn('upstream_sha256', final['progress'])
        self.assert_partition(stages, pending)

    def test_final_holdout_must_not_reuse_any_upstream_source(self):
        self.write_complete_chain()
        with open(self.path(TASKS[-1]), encoding='utf-8') as f:
            final_doc = json.load(f)
        final_doc['source_data'][0]['sha256'] = 'a' * 64
        self.write(TASKS[-1], final_doc)
        stages, pending = collect_lidar(self.tmp.name)
        final = next(x for x in pending if x['id'] == 'mid360s_validation')
        self.assertEqual(final['lifecycle'], 'rework')
        self.assertIn('数据哈希重合', final['progress'])
        self.assert_partition(stages, pending)

    def test_conflicting_serials_reopen_all_affected_results(self):
        health, imu = TASKS[:2]
        self.write(health, valid_result(health))
        doc = valid_result(imu)
        doc['devices'][0]['serial'] = 'LIVOX-OTHER'
        self.write(imu, doc)
        stages, pending = collect_lidar(self.tmp.name)
        for task_id in (health['id'], imu['id']):
            item = next(x for x in pending if x['id'] == task_id)
            self.assertEqual(item['lifecycle'], 'rework')
            self.assertIn('serial', item['progress'])
        self.assert_partition(stages, pending)


if __name__ == '__main__':
    unittest.main()
