#!/usr/bin/env python3
"""One-shot ROS 2 parameter bridge for calibrate_mid360s_imu.sh."""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


class PipelineParameterError(ValueError):
    pass


def _text(parameters: Mapping[str, Any], name: str, *, required: bool = True) -> str:
    value = parameters.get(name, "")
    if not isinstance(value, str):
        raise PipelineParameterError(f"{name} must be a string")
    value = value.strip()
    if required and not value:
        raise PipelineParameterError(f"{name} must be non-empty")
    return value


def _integer(parameters: Mapping[str, Any], name: str, minimum: int) -> int:
    value = parameters.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise PipelineParameterError(f"{name} must be an integer >= {minimum}")
    return value


def _positive(parameters: Mapping[str, Any], name: str) -> float:
    value = parameters.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise PipelineParameterError(f"{name} must be positive")
    return float(value)


def _finite(parameters: Mapping[str, Any], name: str) -> float:
    value = parameters.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise PipelineParameterError(f"{name} must be a finite number")
    return float(value)


def build_pipeline_command(
    pipeline: Path,
    parameters: Mapping[str, Any],
) -> list[str]:
    pipeline = Path(pipeline).resolve()
    if not pipeline.is_file():
        raise PipelineParameterError(f"pipeline executable does not exist: {pipeline}")
    inputs = parameters.get("inputs", [])
    if not isinstance(inputs, (list, tuple)) or any(
        not isinstance(item, str) or not item.strip() for item in inputs
    ):
        raise PipelineParameterError("inputs must be a list of non-empty path strings")
    project_root = _text(parameters, "project_root")
    if not Path(project_root).is_absolute():
        raise PipelineParameterError(
            "project_root must be an explicit absolute path for ROS deployment"
        )
    command = [
        str(pipeline),
        "--project-root", project_root,
        "--manifest", _text(parameters, "manifest"),
        "--work-dir", _text(parameters, "work_dir"),
        "--output", _text(parameters, "output"),
        "--topic", _text(parameters, "topic"),
        "--frame", _text(parameters, "frame"),
        "--fit-poses", str(_integer(parameters, "fit_poses", 12)),
        "--holdout-poses", str(_integer(parameters, "holdout_poses", 3)),
        "--live-hold", str(_positive(parameters, "live_hold_s")),
        "--bag-hold", str(_positive(parameters, "bag_hold_s")),
        "--min-samples", str(_integer(parameters, "min_samples", 8)),
        "--min-sep", str(_positive(parameters, "min_separation_deg")),
        "--lat", str(_finite(parameters, "latitude_deg")),
        "--alt", str(_finite(parameters, "altitude_m")),
    ]
    optional = (
        ("mid360s_serial", "--serial"),
        ("rig_id", "--rig-id"),
        ("mount_session_id", "--mount-id"),
        ("python", "--python"),
    )
    for name, flag in optional:
        value = _text(parameters, name, required=False)
        if value:
            command.extend((flag, value))
    # Keep every string as the value of a repeatable one-value option.  This
    # prevents a path beginning with ``--`` from being reinterpreted as a
    # pipeline option while retaining a shell-free argv invocation.
    for item in inputs:
        command.extend(("--input", item.strip()))
    verify_existing = parameters.get("verify_existing", False)
    if not isinstance(verify_existing, bool):
        raise PipelineParameterError("verify_existing must be a boolean")
    if verify_existing:
        command.append("--verify-existing")
    return command


def _pipeline_path() -> Path:
    # Source layout: tools/node.py -> project/calibrate....
    source_candidate = Path(__file__).resolve().parents[1] / "calibrate_mid360s_imu.sh"
    if source_candidate.is_file():
        return source_candidate
    # Installed layout: lib/rgbd_fullcalib/node.py and sibling pipeline.
    installed_candidate = Path(__file__).resolve().parent / "calibrate_mid360s_imu.sh"
    if installed_candidate.is_file():
        return installed_candidate
    raise PipelineParameterError("cannot locate calibrate_mid360s_imu.sh")


def main(argv: Sequence[str] | None = None) -> int:
    try:
        import rclpy
        from rclpy.exceptions import ParameterException
        from rclpy.node import Node
    except ImportError as exc:
        print(f"ROS 2 Python packages are required: {exc}", file=sys.stderr)
        return 2

    class PipelineNode(Node):
        def __init__(self) -> None:
            super().__init__("mid360s_imu_pipeline")
            defaults = {
                "project_root": "",
                "manifest": "data/lidar_camera_extrinsic/capture_session.json",
                "work_dir": "data/mid360s_imu_pipeline",
                "output": "results/mid360s_imu.json",
                "topic": "/livox/imu",
                "frame": "livox_frame",
                "mid360s_serial": "",
                "rig_id": "",
                "mount_session_id": "",
                # rclpy cannot infer the element type of an empty array.  A
                # one-empty-string sentinel keeps the ROS parameter typed as
                # STRING_ARRAY and is normalized to [] immediately below.
                "inputs": [""],
                "fit_poses": 12,
                "holdout_poses": 3,
                "live_hold_s": 0.5,
                "bag_hold_s": 0.5,
                "min_samples": 60,
                "min_separation_deg": 18.0,
                "latitude_deg": 22.3,
                "altitude_m": 30.0,
                "python": "",
                "verify_existing": False,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.parameters = {
                name: self.get_parameter(name).value for name in defaults
            }
            if self.parameters["inputs"] == [""]:
                self.parameters["inputs"] = []

    rclpy.init(args=list(argv) if argv is not None else None)
    node = None
    try:
        node = PipelineNode()
        command = build_pipeline_command(_pipeline_path(), node.parameters)
        node.get_logger().info("starting fail-closed MID-360S IMU pipeline")
        completed = subprocess.run(command, shell=False, check=False)
        if completed.returncode != 0:
            node.get_logger().error(
                f"MID-360S IMU pipeline failed with exit code {completed.returncode}"
            )
        return int(completed.returncode)
    except (PipelineParameterError, ParameterException, OSError, ValueError) as exc:
        if node is None:
            print(f"MID-360S IMU pipeline refused to start: {exc}", file=sys.stderr)
        else:
            node.get_logger().error(str(exc))
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
