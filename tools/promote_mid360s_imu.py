#!/usr/bin/env python3
"""Promote an accepted MID-360S IMU analysis into a current-rig result.

The solver intentionally writes analysis artifacts outside ``results/``.  This
tool is the narrow promotion boundary: it rechecks the analysis gates, hashes
the actual inputs, binds every capture identity to the current-rig manifest,
verifies the paired NPZ, builds the operational result, and asks the viewer
registry to accept the exact document before creating it.

Existing outputs are never replaced.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np


ANALYSIS_SCHEMA = "mid360s_imu_intrinsics_analysis/v1"
CAPTURE_SCHEMA = "mid360s_imu_stable_poses/v1"
MANIFEST_SCHEMA = "d435i_calib/lidar_camera_mount_session/v1"
OPERATIONAL_SCHEMA = "d435i_calib/mid360s_imu_operational/v1"
TASK_ID = "mid360s_imu"
STANDARD_GRAVITY = 9.80665
ACCEL_EQUATION = (
    "a_corrected_ms2 = T_misalignment * diag(accel_scale) * "
    "(9.80665 * a_raw_g - accel_bias_ms2)"
)
DEFAULT_MANIFEST = "data/lidar_camera_extrinsic/capture_session.json"
DEFAULT_OUTPUT = "results/mid360s_imu.json"


class PromotionError(ValueError):
    """The analysis is not safe to promote."""


def _reject_constant(token: str) -> None:
    raise PromotionError(f"non-standard/non-finite JSON constant is forbidden: {token}")


def _parse_float(token: str) -> float:
    value = float(token)
    if not math.isfinite(value):
        raise PromotionError(f"non-finite JSON number is forbidden: {token}")
    return value


def _unique_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise PromotionError(f"duplicate JSON key is forbidden: {key}")
        result[key] = value
    return result


def _finite_tree(value: Any, label: str = "document") -> None:
    if isinstance(value, bool) or value is None or isinstance(value, (str, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise PromotionError(f"{label} contains a non-finite number")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _finite_tree(item, f"{label}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_tree(item, f"{label}.{key}")
        return
    raise PromotionError(f"{label} contains an unsupported value type: {type(value).__name__}")


def load_strict_json(path: Path, label: str = "JSON") -> dict[str, Any]:
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            document = json.load(
                stream,
                parse_constant=_reject_constant,
                parse_float=_parse_float,
                object_pairs_hook=_unique_object,
            )
    except PromotionError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PromotionError(f"cannot read strict {label} {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise PromotionError(f"{label} root must be an object")
    _finite_tree(document, label)
    return document


def _strict_json_text(text: str, label: str) -> dict[str, Any]:
    try:
        document = json.loads(
            text,
            parse_constant=_reject_constant,
            parse_float=_parse_float,
            object_pairs_hook=_unique_object,
        )
    except PromotionError:
        raise
    except json.JSONDecodeError as exc:
        raise PromotionError(f"invalid {label}: {exc}") from exc
    if not isinstance(document, dict):
        raise PromotionError(f"{label} root must be an object")
    _finite_tree(document, label)
    return document


def _nonempty(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PromotionError(f"{label} must be a non-empty string")
    value = value.strip()
    if any(character in value for character in "\r\n\t"):
        raise PromotionError(f"{label} must not contain control whitespace")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PromotionError(f"{label} must be a finite JSON number")
    value = float(value)
    if not math.isfinite(value):
        raise PromotionError(f"{label} must be finite")
    return value


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise PromotionError(f"{label} must be an integer")
    return value


def _dict(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PromotionError(f"{label} must be an object")
    return value


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise PromotionError(f"{label} must be a list")
    return value


def _vector(value: Any, length: int, label: str, *, positive: bool = False) -> list[float]:
    rows = _list(value, label)
    if len(rows) != length:
        raise PromotionError(f"{label} must contain exactly {length} numbers")
    converted = [_number(item, f"{label}[{index}]") for index, item in enumerate(rows)]
    if positive and any(item <= 0.0 for item in converted):
        raise PromotionError(f"{label} entries must be positive")
    return converted


def _matrix3(value: Any, label: str) -> list[list[float]]:
    rows = _list(value, label)
    if len(rows) != 3:
        raise PromotionError(f"{label} must be a 3x3 matrix")
    return [_vector(row, 3, f"{label}[{index}]") for index, row in enumerate(rows)]


def _utc(value: Any, label: str) -> str:
    value = _nonempty(value, label)
    if not value.endswith("Z"):
        raise PromotionError(f"{label} must be an ISO-8601 UTC timestamp ending in Z")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise PromotionError(f"{label} is not a valid ISO-8601 timestamp") from exc
    if parsed.tzinfo is None or parsed.utcoffset().total_seconds() != 0.0:
        raise PromotionError(f"{label} must be UTC")
    return value


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def _resolve_under_root(
    value: Path | str,
    root: Path,
    label: str,
    *,
    must_exist: bool = True,
) -> tuple[Path, str]:
    raw = Path(value)
    path = (raw if raw.is_absolute() else root / raw).resolve()
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise PromotionError(f"{label} must stay inside project root {root}: {value}") from exc
    if must_exist and not path.exists():
        raise PromotionError(f"{label} does not exist: {path}")
    if path.is_symlink():
        raise PromotionError(f"{label} must not be a symlink: {path}")
    return path, relative.as_posix()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise PromotionError(f"cannot hash file {path}: {exc}") from exc
    return digest.hexdigest()


def sha256_path(path: Path) -> tuple[str, str]:
    """Match calibrate_mid360s_imu_intrinsics.sha256_path exactly."""
    path = Path(path)
    if path.is_symlink():
        raise PromotionError(f"source input must not be a symlink: {path}")
    if path.is_file():
        return sha256_file(path), "sha256-file-v1(bytes)"
    if not path.is_dir():
        raise PromotionError(f"source input does not exist: {path}")
    digest = hashlib.sha256()
    files = sorted(item for item in path.rglob("*") if item.is_file() or item.is_symlink())
    for item in files:
        if item.is_symlink():
            raise PromotionError(f"source input tree contains a symlink: {item}")
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "little"))
        digest.update(relative)
        digest.update(item.stat().st_size.to_bytes(8, "little"))
        with item.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return (
        digest.hexdigest(),
        "sha256-tree-v1(length-prefixed relative path, size, bytes; uint64 little-endian)",
    )


def validate_manifest(
    document: dict[str, Any],
    *,
    expected_serial: str | None = None,
    expected_rig_id: str | None = None,
    expected_mount_id: str | None = None,
) -> dict[str, str]:
    if document.get("schema") != MANIFEST_SCHEMA:
        raise PromotionError(f"manifest schema must be {MANIFEST_SCHEMA}")
    identity = {
        "rig_id": _nonempty(document.get("rig_id"), "manifest.rig_id"),
        "mount_session_id": _nonempty(
            document.get("mount_session_id"), "manifest.mount_session_id"
        ),
        "mid360s_serial": _nonempty(
            document.get("mid360s_serial"), "manifest.mid360s_serial"
        ),
        "d435i_serial": _nonempty(document.get("d435i_serial"), "manifest.d435i_serial"),
    }
    expected = {
        "mid360s_serial": expected_serial,
        "rig_id": expected_rig_id,
        "mount_session_id": expected_mount_id,
    }
    for key, value in expected.items():
        if value is not None and _nonempty(value, f"expected {key}") != identity[key]:
            raise PromotionError(
                f"expected {key} does not match current-rig manifest: "
                f"{value!r} != {identity[key]!r}"
            )
    return identity


def _capture_identity(metadata: dict[str, Any], label: str) -> dict[str, str]:
    if metadata.get("schema") != CAPTURE_SCHEMA:
        raise PromotionError(f"{label}.schema must be {CAPTURE_SCHEMA}")
    raw = _dict(metadata.get("identity"), f"{label}.identity")
    return {
        "mid360s_serial": _nonempty(
            raw.get("mid360s_serial"), f"{label}.identity.mid360s_serial"
        ),
        "rig_id": _nonempty(raw.get("rig_id"), f"{label}.identity.rig_id"),
        "mount_id": _nonempty(raw.get("mount_id"), f"{label}.identity.mount_id"),
    }


def _validate_capture_metadata(
    metadata: dict[str, Any],
    manifest: dict[str, str],
    expected_frame: str,
    label: str,
) -> None:
    got = _capture_identity(metadata, label)
    expected = {
        "mid360s_serial": manifest["mid360s_serial"],
        "rig_id": manifest["rig_id"],
        "mount_id": manifest["mount_session_id"],
    }
    if got != expected:
        raise PromotionError(f"{label}.identity does not match current-rig manifest")
    units = _dict(metadata.get("units"), f"{label}.units")
    if units.get("driver_accelerometer_input") != "g":
        raise PromotionError(f"{label} must declare raw Livox acceleration in g")
    if not math.isclose(
        _number(units.get("standard_gravity_ms2"), f"{label}.units.standard_gravity_ms2"),
        STANDARD_GRAVITY,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PromotionError(f"{label} must use g * 9.80665 for SI conversion")
    conversion = units.get("conversion")
    if conversion != "accel_ms2 = accel_driver_g * 9.80665":
        raise PromotionError(f"{label}.units.conversion must explicitly be g * 9.80665")
    source = _dict(metadata.get("source"), f"{label}.source")
    frame = source.get("frame_id")
    if frame != expected_frame:
        raise PromotionError(
            f"{label}.source.frame_id does not match expected ROS frame: "
            f"{frame!r} != {expected_frame!r}"
        )
    session_rows = _list(source.get("sessions", []), f"{label}.source.sessions")
    if not session_rows:
        detector = _dict(metadata.get("stable_detector"), f"{label}.stable_detector")
        lower_limits = {
            "hold_s": 0.5,
            "min_samples": 60.0,
            "min_separation_deg": 18.0,
        }
        upper_limits = {
            "gyro_mean_limit_deg_s": 4.0,
            "gyro_std_limit_deg_s": 2.5,
            "accel_std_limit_ms2": 0.35,
            "direction_drift_limit_deg": 0.8,
        }
        for field, minimum in lower_limits.items():
            if _number(detector.get(field), f"{label}.stable_detector.{field}") < minimum:
                raise PromotionError(
                    f"{label}.stable_detector.{field} weakens the minimum {minimum}"
                )
        for field, maximum in upper_limits.items():
            if _number(detector.get(field), f"{label}.stable_detector.{field}") > maximum:
                raise PromotionError(
                    f"{label}.stable_detector.{field} weakens the maximum {maximum}"
                )
    if session_rows:
        for index, session in enumerate(session_rows):
            session_doc = _dict(session, f"{label}.source.sessions[{index}]")
            nested = _dict(
                session_doc.get("capture_metadata"),
                f"{label}.source.sessions[{index}].capture_metadata",
            )
            _validate_capture_metadata(
                nested,
                manifest,
                expected_frame,
                f"{label}.source.sessions[{index}].capture_metadata",
            )


def _validate_indices(validation: dict[str, Any]) -> tuple[int, int, int]:
    pose_count = _integer(validation.get("pose_count"), "validation.pose_count")
    fit_count = _integer(validation.get("fit_pose_count"), "validation.fit_pose_count")
    holdout_count = _integer(
        validation.get("holdout_pose_count"), "validation.holdout_pose_count"
    )
    if fit_count < 12:
        raise PromotionError("formal promotion requires at least 12 fit orientations")
    if holdout_count < 3:
        raise PromotionError("formal promotion requires at least 3 independent holdout orientations")
    if pose_count != fit_count + holdout_count:
        raise PromotionError("validation.pose_count must equal fit + holdout counts")
    fit_indices = _list(validation.get("fit_indices"), "validation.fit_indices")
    holdout_indices = _list(validation.get("holdout_indices"), "validation.holdout_indices")
    if len(fit_indices) != fit_count or len(holdout_indices) != holdout_count:
        raise PromotionError("validation index lengths do not match declared counts")
    all_indices = []
    for label, rows in (("fit", fit_indices), ("holdout", holdout_indices)):
        for index, value in enumerate(rows):
            all_indices.append(_integer(value, f"validation.{label}_indices[{index}]"))
    if sorted(all_indices) != list(range(pose_count)):
        raise PromotionError("fit and holdout indices must be disjoint and cover every pose exactly once")
    return pose_count, fit_count, holdout_count


def _validate_analysis_gates(document: dict[str, Any]) -> None:
    validation = _dict(document.get("validation"), "validation")
    _validate_indices(validation)
    policy = _dict(_dict(document.get("acceptance"), "acceptance").get("policy"), "acceptance.policy")
    if _integer(policy.get("minimum_fit_poses"), "acceptance.policy.minimum_fit_poses") < 12:
        raise PromotionError("analysis policy weakens the minimum 12-pose fit gate")
    if _integer(policy.get("desired_holdout_poses"), "acceptance.policy.desired_holdout_poses") < 3:
        raise PromotionError("analysis policy weakens the 3-pose holdout gate")
    if _integer(policy.get("minimum_jacobian_rank"), "acceptance.policy.minimum_jacobian_rank") < 9:
        raise PromotionError("analysis policy weakens the Jacobian rank gate")
    upper_limits = {
        "maximum_normalized_jacobian_condition": 200.0,
        "maximum_fit_residual_rms_ms2": 0.08,
        "maximum_holdout_residual_rms_ms2": 0.15,
        "maximum_holdout_residual_abs_ms2": 0.35,
    }
    for field, maximum in upper_limits.items():
        if _number(policy.get(field), f"acceptance.policy.{field}") > maximum:
            raise PromotionError(f"analysis policy weakens {field} beyond {maximum}")

    fit_stats = _dict(validation.get("fit_residual"), "validation.fit_residual")
    holdout_stats = _dict(validation.get("holdout_residual"), "validation.holdout_residual")
    if _number(fit_stats.get("rms_ms2"), "validation.fit_residual.rms_ms2") > 0.08:
        raise PromotionError("fit residual RMS exceeds 0.08 m/s^2")
    if _number(holdout_stats.get("rms_ms2"), "validation.holdout_residual.rms_ms2") > 0.15:
        raise PromotionError("holdout residual RMS exceeds 0.15 m/s^2")
    if _number(holdout_stats.get("max_abs_ms2"), "validation.holdout_residual.max_abs_ms2") > 0.35:
        raise PromotionError("holdout maximum residual exceeds 0.35 m/s^2")

    observability = _dict(document.get("observability"), "observability")
    if _integer(observability.get("parameter_count"), "observability.parameter_count") != 9:
        raise PromotionError("observability.parameter_count must be 9")
    if _integer(observability.get("jacobian_rank"), "observability.jacobian_rank") != 9:
        raise PromotionError("accelerometer Jacobian must have full rank 9/9")
    if _number(
        observability.get("jacobian_condition_column_normalized"),
        "observability.jacobian_condition_column_normalized",
    ) > 200.0:
        raise PromotionError("normalized Jacobian condition exceeds 200")
    for coverage_name in ("fit_orientation_coverage", "full_orientation_coverage"):
        coverage = _dict(observability.get(coverage_name), f"observability.{coverage_name}")
        eigenvalues = _vector(
            coverage.get("scatter_eigenvalues"),
            3,
            f"observability.{coverage_name}.scatter_eigenvalues",
        )
        span = _vector(
            coverage.get("axis_span"), 3, f"observability.{coverage_name}.axis_span"
        )
        if min(eigenvalues) < 0.075:
            raise PromotionError(f"{coverage_name} scatter minimum eigenvalue is below 0.075")
        if min(span) < 1.15:
            raise PromotionError(f"{coverage_name} does not cover both signs of every axis")
        if _number(
            coverage.get("mean_direction_norm"),
            f"observability.{coverage_name}.mean_direction_norm",
        ) > 0.60:
            raise PromotionError(f"{coverage_name} is one-sided")


def _validate_model(document: dict[str, Any]) -> None:
    frames = _dict(document.get("frame_convention"), "frame_convention")
    if frames != {"frame": "mid360s_imu_frame", "accel_equation": ACCEL_EQUATION}:
        raise PromotionError("analysis frame convention or g-to-SI equation is not canonical")
    result = _dict(document.get("result"), "result")
    literals = {
        "frame": "mid360s_imu_frame",
        "accel_input_unit": "g",
        "accel_output_unit": "m/s^2",
        "accel_equation": ACCEL_EQUATION,
    }
    for field, expected in literals.items():
        if result.get(field) != expected:
            raise PromotionError(f"result.{field} must be {expected!r}")
    if not math.isclose(
        _number(result.get("driver_g_to_ms2_scale"), "result.driver_g_to_ms2_scale"),
        STANDARD_GRAVITY,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PromotionError("result.driver_g_to_ms2_scale must be exactly 9.80665")
    _matrix3(result.get("T_misalignment"), "result.T_misalignment")
    scales = _vector(result.get("accel_scale"), 3, "result.accel_scale", positive=True)
    misalignment = _vector(
        result.get("accel_misalignment_rad"), 3, "result.accel_misalignment_rad"
    )
    biases = _vector(result.get("accel_bias_ms2"), 3, "result.accel_bias_ms2")
    _matrix3(result.get("accel_correction_matrix"), "result.accel_correction_matrix")
    _vector(result.get("gyro_bias_rad_s"), 3, "result.gyro_bias_rad_s")
    if any(not 0.85 <= item <= 1.15 for item in scales):
        raise PromotionError("accelerometer scale is outside [0.85, 1.15]")
    if max(abs(item) for item in misalignment) > 0.10:
        raise PromotionError("accelerometer non-orthogonality exceeds 0.10 rad")
    if max(abs(item) for item in biases) > 1.0:
        raise PromotionError("accelerometer bias exceeds 1.0 m/s^2")


def validate_analysis(
    document: dict[str, Any],
    manifest: dict[str, str],
    expected_frame: str,
) -> None:
    _finite_tree(document, "analysis")
    if document.get("schema") != ANALYSIS_SCHEMA:
        raise PromotionError(f"analysis schema must be {ANALYSIS_SCHEMA}")
    if document.get("intended_local_schema_after_review") != OPERATIONAL_SCHEMA:
        raise PromotionError(f"analysis intended schema must be {OPERATIONAL_SCHEMA}")
    if document.get("scope") != "operational_analysis_only_not_formal_result":
        raise PromotionError("analysis scope is not the solver's analysis-only lane")
    if document.get("status") != "accepted":
        raise PromotionError("refusing to promote an analysis whose status is not accepted")
    _utc(document.get("generated_at_utc"), "analysis.generated_at_utc")
    acceptance = _dict(document.get("acceptance"), "acceptance")
    if acceptance.get("passed") is not True:
        raise PromotionError("analysis acceptance.passed must be true")
    if _list(acceptance.get("rejection_reasons"), "acceptance.rejection_reasons"):
        raise PromotionError("accepted analysis must have no rejection reasons")
    metadata = _dict(
        _dict(document.get("source"), "source").get("capture_metadata"),
        "source.capture_metadata",
    )
    if _dict(document.get("source"), "source").get("expected_ros_frame") != expected_frame:
        raise PromotionError(
            "analysis source.expected_ros_frame does not match the promotion frame"
        )
    _validate_capture_metadata(metadata, manifest, expected_frame, "source.capture_metadata")
    _validate_model(document)
    _validate_analysis_gates(document)


def _verify_analysis_npz(path: Path, analysis: dict[str, Any]) -> None:
    required = {
        "pose_accel_mean_ms2",
        "fit_indices",
        "holdout_indices",
        "full_parameters",
        "full_correction_matrix",
        "full_residual_ms2",
        "training_parameters",
        "training_residual_ms2",
        "holdout_residual_ms2",
        "gyro_bias_rad_s",
        "accel_noise_density_ms2_sqrt_hz",
        "gyro_noise_density_rad_s_sqrt_hz",
        "analysis_json",
    }
    try:
        with np.load(path, allow_pickle=False) as arrays:
            missing = sorted(required.difference(arrays.files))
            if missing:
                raise PromotionError(
                    "analysis NPZ is missing arrays: " + ", ".join(missing)
                )
            unexpected = sorted(set(arrays.files).difference(required))
            if unexpected:
                raise PromotionError(
                    "analysis NPZ has unexpected arrays: " + ", ".join(unexpected)
                )
            embedded_text = str(np.asarray(arrays["analysis_json"]).reshape(()).item())
            values = {
                name: np.asarray(arrays[name]).copy()
                for name in required if name != "analysis_json"
            }
    except PromotionError:
        raise
    except (OSError, ValueError) as exc:
        raise PromotionError(f"cannot read analysis NPZ {path}: {exc}") from exc
    embedded = _strict_json_text(embedded_text, "analysis NPZ analysis_json")
    if embedded != analysis:
        raise PromotionError("analysis NPZ analysis_json does not match the JSON artifact")
    for name, value in values.items():
        if value.dtype.kind not in "iuf":
            raise PromotionError(f"analysis NPZ {name} must have a numeric dtype")
        if not np.all(np.isfinite(value)):
            raise PromotionError(f"analysis NPZ {name} contains NaN or Inf")

    validation = _dict(analysis.get("validation"), "validation")
    model = _dict(analysis.get("result"), "result")
    noise = _dict(analysis.get("stationary_noise"), "stationary_noise")
    pose_count, fit_count, holdout_count = _validate_indices(validation)
    expected_shapes = {
        "pose_accel_mean_ms2": (pose_count, 3),
        "fit_indices": (fit_count,),
        "holdout_indices": (holdout_count,),
        "full_parameters": (9,),
        "full_correction_matrix": (3, 3),
        "full_residual_ms2": (pose_count,),
        "training_parameters": (9,),
        "training_residual_ms2": (fit_count,),
        "holdout_residual_ms2": (holdout_count,),
        "gyro_bias_rad_s": (3,),
        "accel_noise_density_ms2_sqrt_hz": (3,),
        "gyro_noise_density_rad_s_sqrt_hz": (3,),
    }
    for name, shape in expected_shapes.items():
        if values[name].shape != shape:
            raise PromotionError(
                f"analysis NPZ {name} shape is {values[name].shape}, expected {shape}"
            )
    if values["fit_indices"].tolist() != validation["fit_indices"]:
        raise PromotionError("analysis NPZ fit_indices differs from analysis JSON")
    if values["holdout_indices"].tolist() != validation["holdout_indices"]:
        raise PromotionError("analysis NPZ holdout_indices differs from analysis JSON")
    summaries = _list(
        _dict(analysis.get("source"), "source").get("stable_window_summaries"),
        "source.stable_window_summaries",
    )
    if len(summaries) != pose_count:
        raise PromotionError(
            "analysis stable-window summaries must contain one row per pose"
        )
    declared_pose_means = []
    for index, summary in enumerate(summaries):
        row = _dict(summary, f"source.stable_window_summaries[{index}]")
        if _integer(
            row.get("pose_index"),
            f"source.stable_window_summaries[{index}].pose_index",
        ) != index:
            raise PromotionError("analysis stable-window pose indices are not canonical")
        declared_pose_means.append(
            _vector(
                row.get("accel_mean_ms2"),
                3,
                f"source.stable_window_summaries[{index}].accel_mean_ms2",
            )
        )
    if not np.allclose(
        values["pose_accel_mean_ms2"],
        np.asarray(declared_pose_means, dtype=np.float64),
        rtol=1e-12,
        atol=1e-10,
    ):
        raise PromotionError(
            "analysis NPZ pose_accel_mean_ms2 differs from stable-window summaries"
        )

    def same(name: str, expected: Any, *, atol: float = 1.0e-12) -> None:
        expected_array = np.asarray(expected, dtype=np.float64)
        if values[name].shape != expected_array.shape or not np.allclose(
            values[name], expected_array, rtol=1.0e-12, atol=atol
        ):
            raise PromotionError(f"analysis NPZ {name} differs from analysis JSON")

    same(
        "full_parameters",
        list(model["accel_misalignment_rad"])
        + list(model["accel_scale"])
        + list(model["accel_bias_ms2"]),
    )
    same("full_correction_matrix", model["accel_correction_matrix"])
    same("training_parameters", validation["training_model_parameters"])
    same("gyro_bias_rad_s", model["gyro_bias_rad_s"])
    same(
        "accel_noise_density_ms2_sqrt_hz",
        noise["accel_noise_density_ms2_sqrt_hz"],
    )
    same(
        "gyro_noise_density_rad_s_sqrt_hz",
        noise["gyro_noise_density_rad_s_sqrt_hz"],
    )

    def residual_from_parameters(
        parameters: np.ndarray,
        measurements: np.ndarray,
    ) -> np.ndarray:
        p = np.asarray(parameters, dtype=np.float64)
        ayz, azy, azx = p[:3]
        transform = np.asarray(
            [[1.0, -ayz, azy], [0.0, 1.0, -azx], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        matrix = transform @ np.diag(p[3:6])
        corrected = (measurements - p[6:9]) @ matrix.T
        gravity = _number(
            model.get("gravity_reference_ms2"),
            "result.gravity_reference_ms2",
        )
        return np.linalg.norm(corrected, axis=1) - gravity

    pose_means = values["pose_accel_mean_ms2"]
    fit_indices_array = values["fit_indices"].astype(np.int64)
    holdout_indices_array = values["holdout_indices"].astype(np.int64)
    elementwise_residuals = {
        "full_residual_ms2": residual_from_parameters(
            values["full_parameters"], pose_means
        ),
        "training_residual_ms2": residual_from_parameters(
            values["training_parameters"], pose_means[fit_indices_array]
        ),
        "holdout_residual_ms2": residual_from_parameters(
            values["training_parameters"], pose_means[holdout_indices_array]
        ),
    }
    for name, expected in elementwise_residuals.items():
        if not np.allclose(values[name], expected, rtol=1e-12, atol=1e-12):
            raise PromotionError(
                f"analysis NPZ {name} does not match model residuals elementwise"
            )

    def residual_stats(array: np.ndarray) -> dict[str, float]:
        return {
            "rms_ms2": float(np.sqrt(np.mean(array * array))),
            "median_abs_ms2": float(np.median(np.abs(array))),
            "max_abs_ms2": float(np.max(np.abs(array))),
        }

    for array_name, json_name in (
        ("training_residual_ms2", "fit_residual"),
        ("holdout_residual_ms2", "holdout_residual"),
        ("full_residual_ms2", "full_refit_residual"),
    ):
        declared = _dict(validation.get(json_name), f"validation.{json_name}")
        computed = residual_stats(values[array_name])
        for field, actual in computed.items():
            if not math.isclose(
                actual,
                _number(declared.get(field), f"validation.{json_name}.{field}"),
                rel_tol=1.0e-12,
                abs_tol=1.0e-12,
            ):
                raise PromotionError(
                    f"analysis NPZ {array_name} does not reproduce {json_name}.{field}"
                )


def _validate_and_hash_inputs(
    analysis: dict[str, Any], project_root: Path
) -> list[dict[str, str]]:
    source = _dict(analysis.get("source"), "source")
    inputs = _list(source.get("inputs"), "source.inputs")
    if not inputs:
        raise PromotionError("analysis source.inputs must not be empty")
    result = []
    seen_paths: set[str] = set()
    for index, raw in enumerate(inputs):
        entry = _dict(raw, f"source.inputs[{index}]")
        declared_path = _nonempty(entry.get("path"), f"source.inputs[{index}].path")
        path, relative = _resolve_under_root(
            declared_path, project_root, f"source.inputs[{index}].path"
        )
        if relative in seen_paths:
            raise PromotionError(f"duplicate source input path: {relative}")
        seen_paths.add(relative)
        digest, scheme = sha256_path(path)
        declared_digest = _nonempty(
            entry.get("sha256"), f"source.inputs[{index}].sha256"
        ).lower()
        if digest != declared_digest:
            raise PromotionError(
                f"source input hash mismatch for {relative}: {digest} != {declared_digest}"
            )
        result.append(
            {
                "role": "operational_capture",
                "path": relative,
                "sha256": digest,
                "hash_scheme": scheme,
            }
        )
    return result


def recompute_analysis_from_inputs(
    analysis: dict[str, Any],
    project_root: Path,
    manifest: dict[str, str],
    expected_frame: str,
) -> None:
    """Replay the exact inputs through the solver and compare every claim."""
    tools_dir = Path(__file__).resolve().parent
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    try:
        import calibrate_mid360s_imu_intrinsics as calibration
    except ImportError as exc:
        raise PromotionError(f"cannot import MID-360S IMU solver for replay: {exc}") from exc

    source = _dict(analysis.get("source"), "source")
    input_rows = _list(source.get("inputs"), "source.inputs")
    capture_metadata = _dict(source.get("capture_metadata"), "source.capture_metadata")
    sessions = _list(
        _dict(capture_metadata.get("source"), "source.capture_metadata.source").get(
            "sessions", []
        ),
        "source.capture_metadata.source.sessions",
    )
    if len(input_rows) > 1 and len(sessions) != len(input_rows):
        raise PromotionError("merged analysis session count does not match source.inputs")
    captures = []
    for index, raw in enumerate(input_rows):
        entry = _dict(raw, f"source.inputs[{index}]")
        input_path, _ = _resolve_under_root(
            _nonempty(entry.get("path"), f"source.inputs[{index}].path"),
            project_root,
            f"source.inputs[{index}].path",
        )
        if input_path.is_dir():
            if sessions:
                session = _dict(sessions[index], f"source sessions[{index}]")
                metadata = _dict(
                    session.get("capture_metadata"),
                    f"source sessions[{index}].capture_metadata",
                )
            else:
                metadata = capture_metadata
            detector = dict(
                _dict(metadata.get("stable_detector"), f"capture[{index}].stable_detector")
            )
            session_source = _dict(metadata.get("source"), f"capture[{index}].source")
            topic = _nonempty(
                session_source.get("ros_topic"), f"capture[{index}].source.ros_topic"
            )
            captures.append(
                calibration.capture_from_rosbag(
                    input_path,
                    topic=topic,
                    frame=expected_frame,
                    serial=manifest["mid360s_serial"],
                    rig_id=manifest["rig_id"],
                    mount_id=manifest["mount_session_id"],
                    collector_kwargs=detector,
                )
            )
        else:
            captures.append(calibration.load_capture_npz(input_path))

    # A live or offline single-session capture may legitimately predate the
    # merge-only ``capture_plan`` block.  Multiple inputs are always merged by
    # the analyzer and therefore do carry this block.  Keep the replay default
    # identical to the solver CLI without weakening any acceptance gate.
    capture_plan = _dict(
        capture_metadata.get("capture_plan", {}),
        "source.capture_metadata.capture_plan",
    )
    minimum_separation = _number(
        capture_plan.get("minimum_cross_session_separation_deg", 18.0),
        "source.capture_metadata.capture_plan.minimum_cross_session_separation_deg",
    )
    merged = calibration.merge_captures(captures, minimum_separation)
    acceptance = _dict(analysis.get("acceptance"), "acceptance")
    policy = _dict(acceptance.get("policy"), "acceptance.policy")
    gravity_reference = _dict(analysis.get("gravity_reference"), "gravity_reference")
    recomputed, _ = calibration.analyze_capture(
        merged,
        gravity_ms2=_number(
            gravity_reference.get("value_ms2"), "gravity_reference.value_ms2"
        ),
        minimum_fit_poses=_integer(
            policy.get("minimum_fit_poses"), "acceptance.policy.minimum_fit_poses"
        ),
        desired_holdout_poses=_integer(
            policy.get("desired_holdout_poses"), "acceptance.policy.desired_holdout_poses"
        ),
    )
    recomputed["source"]["inputs"] = input_rows
    recomputed["source"]["expected_ros_frame"] = expected_frame
    recomputed["gravity_reference"] = gravity_reference
    # Generation time is the only intentionally non-deterministic field.
    recomputed["generated_at_utc"] = analysis["generated_at_utc"]
    if recomputed != analysis:
        raise PromotionError(
            "replaying source inputs through calibrate_mid360s_imu_intrinsics "
            "does not reproduce the analysis JSON"
        )


def _copy_model_fields(analysis: dict[str, Any]) -> dict[str, Any]:
    model = _dict(analysis.get("result"), "result")
    validation = _dict(analysis.get("validation"), "validation")
    observability = _dict(analysis.get("observability"), "observability")
    full_coverage = _dict(
        observability.get("full_orientation_coverage"),
        "observability.full_orientation_coverage",
    )
    noise = _dict(analysis.get("stationary_noise"), "stationary_noise")
    fit_stats = _dict(validation.get("fit_residual"), "validation.fit_residual")
    holdout_stats = _dict(
        validation.get("holdout_residual"), "validation.holdout_residual"
    )
    full_stats = _dict(
        validation.get("full_refit_residual"), "validation.full_refit_residual"
    )
    return {
        "accel_input_unit": model["accel_input_unit"],
        "accel_output_unit": model["accel_output_unit"],
        "accel_unit_scale_ms2_per_g": model["driver_g_to_ms2_scale"],
        "T_misalignment": model["T_misalignment"],
        "accel_scale": model["accel_scale"],
        "accel_misalignment_rad": model["accel_misalignment_rad"],
        "accel_bias_ms2": model["accel_bias_ms2"],
        "accel_correction_matrix": model["accel_correction_matrix"],
        "gyro_bias_rad_s": model["gyro_bias_rad_s"],
        "gravity_reference_ms2": model["gravity_reference_ms2"],
        "pose_count": validation["pose_count"],
        "fit_pose_count": validation["fit_pose_count"],
        "holdout_pose_count": validation["holdout_pose_count"],
        "accel_fit_residual_rms_ms2": fit_stats["rms_ms2"],
        "accel_holdout_residual_rms_ms2": holdout_stats["rms_ms2"],
        "accel_full_refit_residual_rms_ms2": full_stats["rms_ms2"],
        "imu_sample_rate_hz": noise["sample_rate_hz"],
        "noise_window_duration_s": noise["window_duration_s_median"],
        "accel_noise_density_ms2_sqrt_hz": noise[
            "accel_noise_density_ms2_sqrt_hz"
        ],
        "gyro_noise_density_rad_s_sqrt_hz": noise[
            "gyro_noise_density_rad_s_sqrt_hz"
        ],
        "gyro_static_residual_rms_rad_s": noise[
            "gyro_static_residual_rms_rad_s"
        ],
        "noise_density_method": noise["noise_density_method"],
        "allan_characterization": noise["allan_characterization"],
        "jacobian_rank": observability["jacobian_rank"],
        "jacobian_parameter_count": observability["parameter_count"],
        "jacobian_condition_column_normalized": observability[
            "jacobian_condition_column_normalized"
        ],
        "orientation_scatter_eigenvalues": full_coverage["scatter_eigenvalues"],
        "orientation_minimum_pair_angle_deg": full_coverage[
            "minimum_pair_angle_deg"
        ],
        "orientation_axis_span": full_coverage["axis_span"],
    }


def build_operational_result(
    analysis: dict[str, Any],
    manifest: dict[str, str],
    source_data: list[dict[str, str]],
    *,
    analysis_path: str,
    analysis_sha256: str,
    analysis_npz_path: str,
    analysis_npz_sha256: str,
    created_utc: str | None = None,
) -> dict[str, Any]:
    result = _copy_model_fields(analysis)
    source_data = list(source_data) + [
        {
            "role": "accepted_analysis",
            "path": analysis_path,
            "sha256": analysis_sha256,
            "hash_scheme": "sha256-file-v1(bytes)",
        },
        {
            "role": "accepted_analysis_arrays",
            "path": analysis_npz_path,
            "sha256": analysis_npz_sha256,
            "hash_scheme": "sha256-file-v1(bytes)",
        },
    ]
    return {
        "schema_version": 1,
        "task_id": TASK_ID,
        "status": "operational",
        "local_schema": OPERATIONAL_SCHEMA,
        "created_utc": created_utc or utc_now(),
        "rig_id": manifest["rig_id"],
        "mount_session_id": manifest["mount_session_id"],
        "devices": [
            {
                "role": "lidar",
                "model": "Livox MID-360S",
                "serial": manifest["mid360s_serial"],
            }
        ],
        "method": (
            "Multi-orientation static-gravity fit with explicit Livox g-to-SI "
            "conversion, independent orientation holdout, and per-pose "
            "short-window white-noise estimation"
        ),
        "frame_convention": {
            "frame": "mid360s_imu_frame",
            "accel_equation": ACCEL_EQUATION,
        },
        "source_data": source_data,
        "result": result,
        "summary": {
            "quality": "OPERATIONAL · current MID-360S",
            "coverage": (
                f"{result['pose_count']} orientations: {result['fit_pose_count']} fit + "
                f"{result['holdout_pose_count']} independent holdout; Jacobian rank "
                f"{result['jacobian_rank']}/{result['jacobian_parameter_count']}, "
                f"normalized condition {result['jacobian_condition_column_normalized']:.2f}"
            ),
            "accel_fit": (
                f"fit RMS {result['accel_fit_residual_rms_ms2']:.5f} m/s^2; "
                f"holdout RMS {result['accel_holdout_residual_rms_ms2']:.5f} m/s^2"
            ),
            "accel_scale": "[" + ", ".join(f"{x:.6f}" for x in result["accel_scale"]) + "]",
            "accel_bias": "[" + ", ".join(f"{x:+.5f}" for x in result["accel_bias_ms2"]) + "] m/s^2",
            "gyro_bias": "[" + ", ".join(f"{x:+.6f}" for x in result["gyro_bias_rad_s"]) + "] rad/s",
            "noise": (
                "per-axis short-window white-noise density at "
                f"{result['imu_sample_rate_hz']:.2f} Hz; not an Allan characterization"
            ),
        },
        "note": (
            "当前 MID-360S 的加速度单位换算、加速度计偏置/标度/非正交、"
            "陀螺静态偏置及短窗白噪声参数已固结。"
        ),
    }


def _viewer_modules():
    code_root = Path(__file__).resolve().parents[1]
    if str(code_root) not in sys.path:
        sys.path.insert(0, str(code_root))
    try:
        from viewer.lidar_calib import (  # type: ignore
            TASKS,
            _validate_operational_aux,
            collect_lidar,
        )
    except (ImportError, RuntimeError) as exc:
        raise PromotionError(f"cannot load viewer calibration registry: {exc}") from exc
    return TASKS, _validate_operational_aux, collect_lidar


def validate_viewer_document(document: dict[str, Any], manifest: dict[str, str]) -> None:
    tasks, validator, _ = _viewer_modules()
    task = next(item for item in tasks if item["id"] == TASK_ID)
    lifecycle, reasons = validator(task, document, manifest)
    if lifecycle != "done" or reasons:
        raise PromotionError(
            "viewer registry rejected promoted document: " + "; ".join(reasons)
        )


def verify_viewer_output(project_root: Path) -> None:
    _, _, collect_lidar = _viewer_modules()
    stages, pending = collect_lidar(str(project_root))
    stage = next((item for item in stages if item.get("id") == TASK_ID), None)
    if stage is None:
        pending_row = next((item for item in pending if item.get("id") == TASK_ID), None)
        detail = pending_row.get("progress") if pending_row else "missing from projection"
        raise PromotionError(f"viewer summary did not mark {TASK_ID} done: {detail}")


def verify_existing_result(
    path: Path,
    expected: dict[str, Any],
    manifest_document: dict[str, Any],
    project_root: Path,
) -> dict[str, Any]:
    existing = load_strict_json(path, "existing formal MID-360S IMU result")
    validate_viewer_document(existing, manifest_document)
    identity_fields = (
        "schema_version",
        "task_id",
        "status",
        "local_schema",
        "rig_id",
        "mount_session_id",
        "devices",
        "frame_convention",
    )
    for field in identity_fields:
        if existing.get(field) != expected.get(field):
            raise PromotionError(f"existing formal result {field} differs from new analysis")
    if existing.get("result") != expected.get("result"):
        raise PromotionError("existing formal result core calibration fields differ from new analysis")

    def sources_by_role(
        document: dict[str, Any],
    ) -> dict[str, list[tuple[str, str]]]:
        sources = _list(document.get("source_data"), "source_data")
        rows: dict[str, list[tuple[str, str]]] = {}
        allowed_roles = {
            "operational_capture",
            "accepted_analysis",
            "accepted_analysis_arrays",
        }
        seen_paths: set[str] = set()
        for index, source in enumerate(sources):
            entry = _dict(source, f"source_data[{index}]")
            role = _nonempty(entry.get("role"), f"source_data[{index}].role")
            relative = _nonempty(entry.get("path"), f"source_data[{index}].path")
            declared = _nonempty(
                entry.get("sha256"), f"source_data[{index}].sha256"
            ).lower()
            source_path, canonical_relative = _resolve_under_root(
                relative, project_root, f"source_data[{index}].path"
            )
            actual, _ = sha256_path(source_path)
            if actual != declared:
                raise PromotionError(
                    f"existing formal source hash mismatch for {canonical_relative}"
                )
            if role not in allowed_roles:
                raise PromotionError(
                    f"existing formal result has unexpected source role: {role}"
                )
            if canonical_relative in seen_paths:
                raise PromotionError(
                    f"existing formal result repeats source path: {canonical_relative}"
                )
            seen_paths.add(canonical_relative)
            rows.setdefault(role, []).append((canonical_relative, declared))
        for role in ("accepted_analysis", "accepted_analysis_arrays"):
            if len(rows.get(role, [])) != 1:
                raise PromotionError(
                    f"existing formal result must contain exactly one {role} source"
                )
        if not rows.get("operational_capture"):
            raise PromotionError(
                "existing formal result has no operational_capture source"
            )
        return {role: sorted(values) for role, values in rows.items()}

    existing_sources = sources_by_role(existing)
    existing_captures = existing_sources["operational_capture"]
    expected_captures = sorted(
        (entry["path"], entry["sha256"])
        for entry in expected["source_data"]
        if entry["role"] == "operational_capture"
    )
    if existing_captures != expected_captures:
        raise PromotionError(
            "existing formal operational_capture paths/hashes differ from new analysis"
        )
    verify_viewer_output(project_root)
    return existing


def atomic_exclusive_json(path: Path, document: dict[str, Any]) -> str:
    path = path.resolve()
    if path.exists():
        raise PromotionError(f"refusing to overwrite existing formal result: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(
                document,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise PromotionError(f"refusing to overwrite existing formal result: {path}") from exc
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return sha256_file(path)


def promote(
    *,
    project_root: Path,
    manifest_path: Path,
    analysis_path: Path,
    analysis_npz_path: Path,
    output_path: Path,
    expected_frame: str,
    expected_serial: str | None = None,
    expected_rig_id: str | None = None,
    expected_mount_id: str | None = None,
    verify_existing: bool = False,
) -> dict[str, Any]:
    project_root = Path(project_root).resolve()
    if not project_root.is_dir():
        raise PromotionError(f"project root is not a directory: {project_root}")
    manifest_path, _ = _resolve_under_root(
        manifest_path, project_root, "manifest path"
    )
    analysis_path, analysis_relative = _resolve_under_root(
        analysis_path, project_root, "analysis JSON path"
    )
    analysis_npz_path, npz_relative = _resolve_under_root(
        analysis_npz_path, project_root, "analysis NPZ path"
    )
    output_path, output_relative = _resolve_under_root(
        output_path, project_root, "output path", must_exist=False
    )
    if output_relative != DEFAULT_OUTPUT:
        raise PromotionError(
            f"formal MID-360S IMU output must be {DEFAULT_OUTPUT} under project root"
        )
    if output_path.exists() and not verify_existing:
        raise PromotionError(f"refusing to overwrite existing formal result: {output_path}")
    if verify_existing and not output_path.is_file():
        raise PromotionError(f"--verify-existing requires an existing formal result: {output_path}")
    expected_frame = _nonempty(expected_frame, "expected frame")
    manifest_doc = load_strict_json(manifest_path, "current-rig manifest")
    manifest = validate_manifest(
        manifest_doc,
        expected_serial=expected_serial,
        expected_rig_id=expected_rig_id,
        expected_mount_id=expected_mount_id,
    )
    analysis = load_strict_json(analysis_path, "analysis JSON")
    validate_analysis(analysis, manifest, expected_frame)
    _verify_analysis_npz(analysis_npz_path, analysis)
    source_data = _validate_and_hash_inputs(analysis, project_root)
    recompute_analysis_from_inputs(analysis, project_root, manifest, expected_frame)
    document = build_operational_result(
        analysis,
        manifest,
        source_data,
        analysis_path=analysis_relative,
        analysis_sha256=sha256_file(analysis_path),
        analysis_npz_path=npz_relative,
        analysis_npz_sha256=sha256_file(analysis_npz_path),
    )
    validate_viewer_document(document, manifest_doc)
    if verify_existing:
        verify_existing_result(
            output_path, document, manifest_doc, project_root
        )
        return {
            "status": "ok",
            "task_id": TASK_ID,
            "task_status": "operational",
            "artifact": output_relative,
            "artifact_sha256": sha256_file(output_path),
            "viewer_summary": "done",
            "verification": "existing artifact matches new analysis core and capture hashes",
            "rig_id": manifest["rig_id"],
            "mount_session_id": manifest["mount_session_id"],
            "mid360s_serial": manifest["mid360s_serial"],
            "input_count": len(source_data),
        }
    artifact_sha256 = atomic_exclusive_json(output_path, document)
    try:
        verify_viewer_output(project_root)
    except Exception:
        # This path did not exist before this invocation.  Remove only the
        # exact artifact we just created, and only while its bytes still match.
        if output_path.exists() and sha256_file(output_path) == artifact_sha256:
            output_path.unlink()
        raise
    return {
        "status": "ok",
        "task_id": TASK_ID,
        "task_status": "operational",
        "artifact": output_relative,
        "artifact_sha256": artifact_sha256,
        "viewer_summary": "done",
        "rig_id": manifest["rig_id"],
        "mount_session_id": manifest["mount_session_id"],
        "mid360s_serial": manifest["mid360s_serial"],
        "input_count": len(source_data),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Strictly promote an accepted MID-360S IMU analysis to the "
            "current-rig results/mid360s_imu.json artifact."
        )
    )
    parser.add_argument("--project-root", default=".")
    parser.add_argument("--manifest", default=DEFAULT_MANIFEST)
    parser.add_argument("--analysis")
    parser.add_argument("--analysis-npz")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-frame", default="livox_frame")
    parser.add_argument("--expected-serial")
    parser.add_argument("--expected-rig-id")
    parser.add_argument("--expected-mount-id")
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="validate manifest/expected identity and print four identity lines",
    )
    parser.add_argument(
        "--verify-existing",
        action="store_true",
        help="verify an existing formal result against the new analysis; never write",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        root = Path(args.project_root).resolve()
        manifest_path, _ = _resolve_under_root(
            args.manifest, root, "manifest path"
        )
        manifest = validate_manifest(
            load_strict_json(manifest_path, "current-rig manifest"),
            expected_serial=args.expected_serial,
            expected_rig_id=args.expected_rig_id,
            expected_mount_id=args.expected_mount_id,
        )
        if args.preflight:
            for field in (
                "rig_id",
                "mount_session_id",
                "mid360s_serial",
                "d435i_serial",
            ):
                print(manifest[field])
            return 0
        missing = [name for name in ("analysis", "analysis_npz") if not getattr(args, name)]
        if missing:
            raise PromotionError("missing required arguments: " + ", ".join(missing))
        summary = promote(
            project_root=root,
            manifest_path=Path(args.manifest),
            analysis_path=Path(args.analysis),
            analysis_npz_path=Path(args.analysis_npz),
            output_path=Path(args.output),
            expected_frame=args.expected_frame,
            expected_serial=args.expected_serial,
            expected_rig_id=args.expected_rig_id,
            expected_mount_id=args.expected_mount_id,
            verify_existing=args.verify_existing,
        )
        print(json.dumps(summary, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return 0
    except (PromotionError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {"status": "error", "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
