#!/usr/bin/env python3
"""ROS 2 runtime correction for the promoted MID-360S IMU result.

The Livox driver publishes accelerometer values in ``g``.  This node converts
them to SI, applies the promoted accelerometer matrix/bias, subtracts only the
observable static gyroscope bias, and publishes ``sensor_msgs/msg/Imu``.  The
manufacturer-defined axes are aligned, but the IMU and LiDAR origins remain
distinct (their lever arm belongs to ``T_lidar_imu``), so output frame identity
is explicit.  This node does not estimate or apply gyroscope scale/misalignment.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
for _candidate in (_THIS_DIR, _THIS_DIR / "tools"):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))
from promote_mid360s_imu import (  # noqa: E402
    ACCEL_EQUATION,
    OPERATIONAL_SCHEMA,
    STANDARD_GRAVITY,
    TASK_ID,
    PromotionError,
    load_strict_json,
    validate_viewer_document,
)


class RuntimeCalibrationError(ValueError):
    """The formal result cannot safely drive runtime correction."""


@dataclass(frozen=True)
class RuntimeCalibration:
    accel_matrix: np.ndarray
    accel_bias_ms2: np.ndarray
    gyro_bias_rad_s: np.ndarray
    accel_covariance: np.ndarray
    gyro_covariance: np.ndarray
    rig_id: str
    mount_session_id: str
    mid360s_serial: str


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeCalibrationError(f"{label} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeCalibrationError(f"{label} must be finite")
    return value


def _nonempty(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeCalibrationError(f"{label} must be a non-empty string")
    return value.strip()


def _vector3(value: Any, label: str, *, positive: bool = False) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeCalibrationError(f"{label} must contain three numbers")
    result = np.asarray(
        [_number(item, f"{label}[{index}]") for index, item in enumerate(value)],
        dtype=np.float64,
    )
    if positive and np.any(result <= 0.0):
        raise RuntimeCalibrationError(f"{label} entries must be positive")
    return result


def _matrix3(value: Any, label: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeCalibrationError(f"{label} must be a 3x3 matrix")
    rows = [_vector3(row, f"{label}[{index}]") for index, row in enumerate(value)]
    return np.asarray(rows, dtype=np.float64)


def _identity(document: dict[str, Any]) -> tuple[str, str, str]:
    devices = document.get("devices")
    if not isinstance(devices, list) or len(devices) != 1:
        raise RuntimeCalibrationError("formal result must contain exactly one device")
    device = devices[0]
    if not isinstance(device, dict) or device.get("role") != "lidar":
        raise RuntimeCalibrationError("formal result device role must be lidar")
    return (
        _nonempty(document.get("rig_id"), "rig_id"),
        _nonempty(document.get("mount_session_id"), "mount_session_id"),
        _nonempty(device.get("serial"), "devices[0].serial"),
    )


def calibration_from_document(
    document: dict[str, Any],
    *,
    expected_serial: str | None = None,
    expected_rig_id: str | None = None,
    expected_mount_id: str | None = None,
) -> RuntimeCalibration:
    if document.get("schema_version") != 1:
        raise RuntimeCalibrationError("schema_version must be 1")
    if document.get("task_id") != TASK_ID:
        raise RuntimeCalibrationError(f"task_id must be {TASK_ID}")
    if document.get("status") != "operational":
        raise RuntimeCalibrationError("formal result status must be operational")
    if document.get("local_schema") != OPERATIONAL_SCHEMA:
        raise RuntimeCalibrationError(f"local_schema must be {OPERATIONAL_SCHEMA}")
    frame = document.get("frame_convention")
    if not isinstance(frame, dict) or frame.get("frame") != "mid360s_imu_frame":
        raise RuntimeCalibrationError("formal result frame must be mid360s_imu_frame")
    if frame.get("accel_equation") != ACCEL_EQUATION:
        raise RuntimeCalibrationError("formal result acceleration equation is not canonical")
    rig_id, mount_id, serial = _identity(document)
    try:
        validate_viewer_document(
            document,
            {
                "rig_id": rig_id,
                "mount_session_id": mount_id,
                "mid360s_serial": serial,
                # The MID-360S IMU task has no RGB-D role, but the shared
                # viewer manifest shape includes this field.
                "d435i_serial": "not-applicable-to-mid360s-imu-runtime",
            },
        )
    except PromotionError as exc:
        raise RuntimeCalibrationError(
            f"formal result failed operational validation: {exc}"
        ) from exc
    for label, supplied, actual in (
        ("expected_serial", expected_serial, serial),
        ("expected_rig_id", expected_rig_id, rig_id),
        ("expected_mount_id", expected_mount_id, mount_id),
    ):
        if supplied is not None and _nonempty(supplied, label) != actual:
            raise RuntimeCalibrationError(f"{label} does not match formal result")

    result = document.get("result")
    if not isinstance(result, dict):
        raise RuntimeCalibrationError("formal result.result must be an object")
    if result.get("accel_input_unit") != "g" or result.get("accel_output_unit") != "m/s^2":
        raise RuntimeCalibrationError("formal result must declare g input and m/s^2 output")
    scale = _number(
        result.get("accel_unit_scale_ms2_per_g"),
        "result.accel_unit_scale_ms2_per_g",
    )
    if not math.isclose(scale, STANDARD_GRAVITY, rel_tol=0.0, abs_tol=1.0e-12):
        raise RuntimeCalibrationError("g-to-SI scale must be exactly 9.80665")
    if result.get("noise_density_method") != "short_window_white_noise":
        raise RuntimeCalibrationError("unsupported or missing IMU noise-density method")
    if result.get("allan_characterization") != "not_performed":
        raise RuntimeCalibrationError("runtime result must not misrepresent short-window noise as Allan")

    matrix = _matrix3(result.get("accel_correction_matrix"), "result.accel_correction_matrix")
    accel_bias = _vector3(result.get("accel_bias_ms2"), "result.accel_bias_ms2")
    scales = _vector3(result.get("accel_scale"), "result.accel_scale", positive=True)
    misalignment = _vector3(
        result.get("accel_misalignment_rad"), "result.accel_misalignment_rad"
    )
    if np.any((scales < 0.85) | (scales > 1.15)):
        raise RuntimeCalibrationError("result.accel_scale is outside [0.85, 1.15]")
    if np.max(np.abs(misalignment)) > 0.10:
        raise RuntimeCalibrationError(
            "result.accel_misalignment_rad exceeds 0.10 rad"
        )
    if np.max(np.abs(accel_bias)) > 1.0:
        raise RuntimeCalibrationError("result.accel_bias_ms2 exceeds 1.0 m/s^2")
    for field, minimum in (
        ("fit_pose_count", 12),
        ("holdout_pose_count", 3),
        ("jacobian_rank", 9),
        ("jacobian_parameter_count", 9),
    ):
        value = result.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise RuntimeCalibrationError(f"result.{field} must be an integer >= {minimum}")
    if result["jacobian_rank"] != 9 or result["jacobian_parameter_count"] != 9:
        raise RuntimeCalibrationError("runtime requires accelerometer Jacobian rank 9/9")
    if _number(
        result.get("jacobian_condition_column_normalized"),
        "result.jacobian_condition_column_normalized",
    ) > 200.0:
        raise RuntimeCalibrationError("normalized accelerometer Jacobian condition exceeds 200")
    for field, maximum in (
        ("accel_fit_residual_rms_ms2", 0.08),
        ("accel_holdout_residual_rms_ms2", 0.15),
    ):
        value = _number(result.get(field), f"result.{field}")
        if value < 0.0 or value > maximum:
            raise RuntimeCalibrationError(f"result.{field} exceeds {maximum}")
    gyro_bias = _vector3(result.get("gyro_bias_rad_s"), "result.gyro_bias_rad_s")
    if np.max(np.abs(gyro_bias)) > 1.0:
        raise RuntimeCalibrationError(
            "result.gyro_bias_rad_s exceeds the 1 rad/s static-bias bound"
        )
    accel_density = _vector3(
        result.get("accel_noise_density_ms2_sqrt_hz"),
        "result.accel_noise_density_ms2_sqrt_hz",
        positive=True,
    )
    gyro_density = _vector3(
        result.get("gyro_noise_density_rad_s_sqrt_hz"),
        "result.gyro_noise_density_rad_s_sqrt_hz",
        positive=True,
    )
    if np.any(accel_density > 10.0) or np.any(gyro_density > 10.0):
        raise RuntimeCalibrationError("formal result noise density is unphysical")
    sample_rate = _number(result.get("imu_sample_rate_hz"), "result.imu_sample_rate_hz")
    if not 1.0 <= sample_rate <= 10_000.0:
        raise RuntimeCalibrationError(
            "result.imu_sample_rate_hz must be within [1, 10000] Hz"
        )
    # A one-sided white-noise density N [unit/sqrt(Hz)] sampled at f [Hz]
    # maps to per-sample diagonal variance N^2*f.  No unsupported cross-axis
    # correlation is invented.
    accel_covariance = np.diag(accel_density * accel_density * sample_rate)
    gyro_covariance = np.diag(gyro_density * gyro_density * sample_rate)
    if not np.all(np.isfinite(accel_covariance)) or not np.all(
        np.isfinite(gyro_covariance)
    ):
        raise RuntimeCalibrationError("derived runtime covariance is not finite")
    return RuntimeCalibration(
        accel_matrix=matrix,
        accel_bias_ms2=accel_bias,
        gyro_bias_rad_s=gyro_bias,
        accel_covariance=accel_covariance,
        gyro_covariance=gyro_covariance,
        rig_id=rig_id,
        mount_session_id=mount_id,
        mid360s_serial=serial,
    )


def load_runtime_calibration(
    path: Path,
    **expected_identity: str | None,
) -> RuntimeCalibration:
    try:
        document = load_strict_json(Path(path), "formal MID-360S IMU result")
    except PromotionError as exc:
        raise RuntimeCalibrationError(str(exc)) from exc
    return calibration_from_document(document, **expected_identity)


def correct_measurement(
    accel_raw_g: Sequence[float],
    gyro_rad_s: Sequence[float],
    calibration: RuntimeCalibration,
) -> tuple[np.ndarray, np.ndarray]:
    raw_accel = np.asarray(accel_raw_g, dtype=np.float64)
    raw_gyro = np.asarray(gyro_rad_s, dtype=np.float64)
    if raw_accel.shape != (3,) or raw_gyro.shape != (3,):
        raise RuntimeCalibrationError("IMU acceleration and gyro samples must each have shape (3,)")
    if not np.all(np.isfinite(raw_accel)) or not np.all(np.isfinite(raw_gyro)):
        raise RuntimeCalibrationError("IMU sample contains NaN or Inf")
    accel_si = calibration.accel_matrix @ (
        STANDARD_GRAVITY * raw_accel - calibration.accel_bias_ms2
    )
    # Static bias is observable from the calibration.  Gyro scale and
    # misalignment are deliberately not claimed or applied.
    gyro_corrected = raw_gyro - calibration.gyro_bias_rad_s
    return accel_si, gyro_corrected


def covariance_row_major(matrix: np.ndarray) -> list[float]:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise RuntimeCalibrationError("covariance must be a finite 3x3 matrix")
    return matrix.reshape(9).tolist()


def output_frame_for_sample(
    observed_frame: str,
    expected_frame: str,
    output_frame: str,
) -> str:
    """Validate the raw frame and return the distinct calibrated IMU frame."""
    observed = _nonempty(observed_frame, "observed input frame")
    expected = _nonempty(expected_frame, "expected input frame")
    corrected = _nonempty(output_frame, "output frame")
    if expected != "livox_frame":
        raise RuntimeCalibrationError(
            "expected input frame must be livox_frame for the promoted model"
        )
    if observed != expected:
        raise RuntimeCalibrationError(
            f"raw IMU frame_id mismatch: expected {expected!r}, observed {observed!r}"
        )
    if corrected != "mid360s_imu_frame":
        raise RuntimeCalibrationError(
            "output frame must be mid360s_imu_frame; relabelling another frame "
            "without rotating/translating the measurement is forbidden"
        )
    return corrected


def unavailable_orientation_covariance() -> list[float]:
    """sensor_msgs marker for an IMU message with no orientation estimate."""
    return [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def run_ros_node(ros_args: Sequence[str] | None = None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        print(f"ROS 2 Python packages are required: {exc}", file=sys.stderr)
        return 2

    class Mid360sImuRuntimeNode(Node):
        def __init__(self) -> None:
            super().__init__("mid360s_imu_runtime")
            self.declare_parameter("project_root", "")
            self.declare_parameter("input_topic", "/livox/imu")
            self.declare_parameter("output_topic", "/livox/imu_calibrated")
            self.declare_parameter("result_path", "results/mid360s_imu.json")
            self.declare_parameter("expected_frame", "livox_frame")
            self.declare_parameter("output_frame", "mid360s_imu_frame")
            self.declare_parameter("mid360s_serial", "")
            self.declare_parameter("rig_id", "")
            self.declare_parameter("mount_session_id", "")
            root_value = _nonempty(
                self.get_parameter("project_root").value, "project_root"
            )
            if not Path(root_value).is_absolute():
                raise RuntimeCalibrationError(
                    "project_root must be an explicit absolute path"
                )
            root = Path(root_value).resolve()
            if not root.is_dir():
                raise RuntimeCalibrationError(
                    f"project_root is not a directory: {root}"
                )
            result_value = Path(str(self.get_parameter("result_path").value))
            result_path = (
                result_value if result_value.is_absolute() else root / result_value
            ).resolve()
            try:
                result_path.relative_to(root)
            except ValueError as exc:
                raise RuntimeCalibrationError(
                    "result_path must stay inside project_root"
                ) from exc
            required_identity = lambda name: _nonempty(
                self.get_parameter(name).value, name
            )
            self.calibration = load_runtime_calibration(
                result_path,
                expected_serial=required_identity("mid360s_serial"),
                expected_rig_id=required_identity("rig_id"),
                expected_mount_id=required_identity("mount_session_id"),
            )
            self.expected_frame = str(self.get_parameter("expected_frame").value).strip()
            if not self.expected_frame:
                raise RuntimeCalibrationError("expected_frame must be non-empty")
            self.output_frame = str(self.get_parameter("output_frame").value).strip()
            output_frame_for_sample(
                self.expected_frame, self.expected_frame, self.output_frame
            )
            input_topic = _nonempty(self.get_parameter("input_topic").value, "input_topic")
            output_topic = _nonempty(self.get_parameter("output_topic").value, "output_topic")
            if input_topic == output_topic:
                raise RuntimeCalibrationError("input_topic and output_topic must differ")
            self.publisher = self.create_publisher(Imu, output_topic, qos_profile_sensor_data)
            self.subscription = self.create_subscription(
                Imu, input_topic, self._on_imu, qos_profile_sensor_data
            )
            self.get_logger().info(
                f"MID-360S IMU correction {input_topic} -> {output_topic}; "
                f"serial={self.calibration.mid360s_serial}"
            )

        def _on_imu(self, message: Imu) -> None:
            try:
                corrected_frame = output_frame_for_sample(
                    message.header.frame_id,
                    self.expected_frame,
                    self.output_frame,
                )
                accel, gyro = correct_measurement(
                    (
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ),
                    (
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ),
                    self.calibration,
                )
            except RuntimeCalibrationError as exc:
                self.get_logger().fatal(str(exc))
                rclpy.shutdown()
                return
            output = Imu()
            output.header.stamp = message.header.stamp
            output.header.frame_id = corrected_frame
            output.orientation = message.orientation
            # Livox raw IMU messages do not contain an orientation estimate.
            # Mark that explicitly per sensor_msgs/Imu instead of forwarding
            # an all-zero covariance that can be misread as perfect certainty.
            output.orientation_covariance = unavailable_orientation_covariance()
            output.linear_acceleration.x = float(accel[0])
            output.linear_acceleration.y = float(accel[1])
            output.linear_acceleration.z = float(accel[2])
            output.angular_velocity.x = float(gyro[0])
            output.angular_velocity.y = float(gyro[1])
            output.angular_velocity.z = float(gyro[2])
            output.linear_acceleration_covariance = covariance_row_major(
                self.calibration.accel_covariance
            )
            output.angular_velocity_covariance = covariance_row_major(
                self.calibration.gyro_covariance
            )
            self.publisher.publish(output)

    rclpy.init(args=list(ros_args) if ros_args is not None else None)
    try:
        node = Mid360sImuRuntimeNode()
    except (RuntimeCalibrationError, OSError, ValueError) as exc:
        print(f"MID-360S IMU runtime refused to start: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the ROS 2 MID-360S raw-g to calibrated-SI IMU node"
    )
    parser.add_argument(
        "--check-result",
        type=Path,
        help="strictly load a formal result and exit without starting ROS",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args, ros_args = parser.parse_known_args(argv)
    if args.check_result is not None:
        if ros_args:
            parser.error("ROS arguments cannot be combined with --check-result")
        try:
            calibration = load_runtime_calibration(args.check_result)
        except (RuntimeCalibrationError, OSError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(
            f"OK serial={calibration.mid360s_serial} "
            f"rig={calibration.rig_id} mount={calibration.mount_session_id}"
        )
        return 0
    return run_ros_node(ros_args)


if __name__ == "__main__":
    raise SystemExit(main())
