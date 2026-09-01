import math
import unittest

import numpy as np

try:
    from .livox_deskew import (DeskewInputError, ImuCoverageError, ImuRing,
                               rotation_deskew_to_scan_end)
except ImportError:
    from livox_deskew import (DeskewInputError, ImuCoverageError, ImuRing,
                              rotation_deskew_to_scan_end)


MS = 1_000_000


def constant_gyro_ring(start_ns, end_ns, gyro=(0.0, 0.0, 0.0), step_ms=10):
    ring = ImuRing()
    timestamps = list(range(start_ns, end_ns + 1, step_ms * MS))
    if timestamps[-1] != end_ns:
        timestamps.append(end_ns)
    ring.extend(timestamps, [gyro] * len(timestamps))
    return ring


class ImuRingTest(unittest.TestCase):
    def test_ring_evicts_old_samples_and_reports_exact_coverage(self):
        ring = ImuRing(max_samples=3)
        ring.extend((0, 10, 20, 30), ((0, 0, 0),) * 4)
        self.assertEqual(len(ring), 3)
        self.assertFalse(ring.covers(0, 20))
        self.assertTrue(ring.covers(10, 30))
        times, gyros = ring.window(15, 25)
        np.testing.assert_array_equal(times, (10, 20, 30))
        self.assertEqual(gyros.shape, (3, 3))

    def test_non_monotonic_input_is_rejected_without_partial_append(self):
        ring = ImuRing()
        ring.add(100, (0, 0, 0))
        with self.assertRaisesRegex(DeskewInputError, "strictly increasing"):
            ring.extend((110, 105), ((0, 0, 0), (0, 0, 0)))
        self.assertEqual(len(ring), 1)
        ring.add(110, (0, 0, 0))
        with self.assertRaisesRegex(DeskewInputError, "newer"):
            ring.add(110, (0, 0, 0))


class RotationDeskewTest(unittest.TestCase):
    def setUp(self):
        # A realistic epoch catches accidental float conversion of absolute ns.
        self.base = 1_788_118_142_634_378_514

    def test_zero_gyro_is_identity(self):
        points = np.asarray(((1.0, 2.0, 3.0), (-2.0, 0.5, 7.0)))
        offsets = np.asarray((0, 100 * MS), dtype=np.uint32)
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS)
        result = rotation_deskew_to_scan_end(points, offsets, self.base, imu)
        np.testing.assert_allclose(result.points, points, atol=1e-14)
        self.assertEqual(result.scan_start_ns, self.base)
        self.assertEqual(result.scan_end_ns, self.base + 100 * MS)
        self.assertEqual(result.reference_time_ns, result.scan_end_ns)

    def test_constant_yaw_matches_analytic_solution(self):
        offsets = np.asarray((0, 25 * MS, 100 * MS), dtype=np.uint32)
        points = np.tile((1.0, 0.0, 0.0), (len(offsets), 1))
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS,
            gyro=(0.0, 0.0, 1.0))
        result = rotation_deskew_to_scan_end(points, offsets, self.base, imu)
        angles = (offsets.astype(np.float64) - 100 * MS) * 1e-9
        expected = np.column_stack((
            np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        np.testing.assert_allclose(result.points, expected, atol=2e-12)

    def test_unsorted_point_offsets_are_deskewed_in_original_order(self):
        offsets = np.asarray((100, 0, 50, 20), dtype=np.uint32) * MS
        points = np.tile((1.0, 0.0, 0.0), (len(offsets), 1))
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS,
            gyro=(0.0, 0.0, 2.0))
        result = rotation_deskew_to_scan_end(points, offsets, self.base, imu)
        angles = 2.0 * (offsets.astype(np.float64) - 100 * MS) * 1e-9
        expected = np.column_stack((
            np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        np.testing.assert_allclose(result.points, expected, atol=2e-12)
        np.testing.assert_allclose(result.points[0], points[0], atol=2e-12)

    def test_lidar_imu_lever_arm_is_applied_without_claiming_translation(self):
        offsets = np.asarray((0, 100 * MS), dtype=np.uint32)
        points = np.tile((1.0, 0.0, 0.0), (2, 1))
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS,
            gyro=(0.0, 0.0, 1.0))
        transform = np.eye(4)
        transform[:3, 3] = (0.1, -0.05, 0.02)
        result = rotation_deskew_to_scan_end(
            points, offsets, self.base, imu, T_lidar_imu=transform)

        angle = -0.1
        relative = np.asarray(((math.cos(angle), -math.sin(angle), 0.0),
                               (math.sin(angle), math.cos(angle), 0.0),
                               (0.0, 0.0, 1.0)))
        expected_start = relative @ (points[0] - transform[:3, 3]) + transform[:3, 3]
        np.testing.assert_allclose(result.points[0], expected_start, atol=2e-12)
        np.testing.assert_allclose(result.points[1], points[1], atol=2e-12)
        self.assertTrue(result.lever_arm_applied)

    def test_invalid_lidar_imu_transform_fails_closed(self):
        points = np.asarray(((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
        offsets = np.asarray((0, 10 * MS), dtype=np.uint32)
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 20 * MS)
        transform = np.eye(4)
        transform[0, 0] = 2.0
        with self.assertRaisesRegex(DeskewInputError, "proper orthonormal"):
            rotation_deskew_to_scan_end(
                points, offsets, self.base, imu, T_lidar_imu=transform)

    def test_missing_imu_at_either_boundary_fails_closed(self):
        points = np.asarray(((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
        offsets = np.asarray((0, 100 * MS), dtype=np.uint32)
        missing_start = constant_gyro_ring(
            self.base + 1 * MS, self.base + 110 * MS)
        with self.assertRaises(ImuCoverageError):
            rotation_deskew_to_scan_end(
                points, offsets, self.base, missing_start)
        missing_end = constant_gyro_ring(
            self.base - 10 * MS, self.base + 99 * MS)
        with self.assertRaises(ImuCoverageError):
            rotation_deskew_to_scan_end(
                points, offsets, self.base, missing_end)

    def test_selected_nan_is_rejected_but_filter_indices_can_exclude_it(self):
        points = np.asarray(((1.0, 0.0, 0.0),
                             (math.nan, 0.0, 0.0),
                             (0.0, 1.0, 0.0)))
        offsets = np.asarray((0, 100 * MS, 50 * MS), dtype=np.uint32)
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS)
        with self.assertRaisesRegex(DeskewInputError, "finite"):
            rotation_deskew_to_scan_end(points, offsets, self.base, imu)

        result = rotation_deskew_to_scan_end(
            points, offsets, self.base, imu, indices=np.asarray((2, 0)))
        np.testing.assert_array_equal(result.points, points[[2, 0]])
        self.assertEqual(result.source_count, 3)
        self.assertEqual(result.output_count, 2)
        # Reference remains the complete scan end (the excluded NaN row).
        self.assertEqual(result.scan_end_ns, self.base + 100 * MS)

    def test_stride_selection_uses_full_scan_reference(self):
        points = np.tile((1.0, 0.0, 0.0), (6, 1))
        offsets = np.asarray((0, 20, 40, 60, 80, 100), dtype=np.uint32) * MS
        imu = constant_gyro_ring(
            self.base - 10 * MS, self.base + 110 * MS,
            gyro=(0.0, 0.0, 1.0))
        result = rotation_deskew_to_scan_end(
            points, offsets, self.base, imu, indices=slice(None, None, 2))
        self.assertEqual(result.output_count, 3)
        self.assertEqual(result.reference_time_ns, self.base + 100 * MS)
        selected_offsets = offsets[::2]
        angles = (selected_offsets.astype(np.float64) - 100 * MS) * 1e-9
        expected = np.column_stack((
            np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        np.testing.assert_allclose(result.points, expected, atol=2e-12)

    def test_invalid_offsets_and_empty_scan_are_rejected(self):
        imu = constant_gyro_ring(self.base, self.base + 10 * MS)
        with self.assertRaisesRegex(DeskewInputError, "empty"):
            rotation_deskew_to_scan_end(
                np.empty((0, 3)), np.empty(0, np.uint32), self.base, imu)
        with self.assertRaisesRegex(DeskewInputError, "integer nanoseconds"):
            rotation_deskew_to_scan_end(
                np.zeros((1, 3)), np.asarray((0.0,)), self.base, imu)
        with self.assertRaisesRegex(DeskewInputError, "negative"):
            rotation_deskew_to_scan_end(
                np.zeros((1, 3)), np.asarray((-1,)), self.base, imu)


if __name__ == "__main__":
    unittest.main()
