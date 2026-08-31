#!/usr/bin/env python3
"""Import a direct_visual_lidar_calibration result as an auditable draft.

The upstream JSON stores ``results.T_lidar_camera`` as
``[x, y, z, qx, qy, qz, qw]`` and uses the convention::

    p_lidar = T_lidar_camera * p_camera

This project consumes the inverse convention::

    p_camera = T_camera_lidar * p_lidar

This importer deliberately cannot create a ``validated`` result.  It only
normalizes and checks the upstream quaternion, performs the analytic SE(3)
inverse, and records enough provenance for later independent validation.
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


SCHEMA = "d435i_calib/lidar_camera_extrinsic_draft/v1"
TASK_ID = "mid360s_d435i_ext"
UPSTREAM_KEY = "results.T_lidar_camera"
DEFAULT_OUTPUT = "results/mid360s_d435i_extrinsic.draft.json"
QUATERNION_MIN_NORM = 1.0e-12
SE3_TOLERANCE = 1.0e-10


class ImportFailure(ValueError):
    """Input or provenance is unsuitable for a safe draft import."""


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite_number(value: Any, label: str) -> float:
    if not _is_number(value):
        raise ImportFailure(f"{label} must be a JSON number (booleans are invalid)")
    value = float(value)
    if not math.isfinite(value):
        raise ImportFailure(f"{label} must be finite")
    return value


def _reject_json_constant(token: str) -> None:
    raise ImportFailure(f"non-standard/non-finite JSON constant is forbidden: {token}")


def _unique_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise ImportFailure(f"duplicate JSON key is forbidden: {key}")
        out[key] = value
    return out


def load_strict_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=_reject_json_constant,
                object_pairs_hook=_unique_object,
            )
    except ImportFailure:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ImportFailure(f"cannot read strict JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ImportFailure("upstream calib.json root must be an object")
    return value


def _matmul(a: Sequence[Sequence[float]],
            b: Sequence[Sequence[float]]) -> list[list[float]]:
    if not a or not b or len(a[0]) != len(b):
        raise ImportFailure("incompatible matrix dimensions")
    return [
        [sum(float(a[i][k]) * float(b[k][j]) for k in range(len(b)))
         for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def _max_abs_delta(a: Sequence[Sequence[float]],
                   b: Sequence[Sequence[float]]) -> float:
    if len(a) != len(b) or any(len(x) != len(y) for x, y in zip(a, b)):
        raise ImportFailure("matrix shapes differ")
    return max(abs(float(x) - float(y))
               for row_a, row_b in zip(a, b)
               for x, y in zip(row_a, row_b))


def _identity4() -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _det3(r: Sequence[Sequence[float]]) -> float:
    return (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )


def normalize_quaternion_xyzw(values: Sequence[Any]) -> tuple[list[float], float]:
    """Return a deterministic unit quaternion and the original norm."""
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ImportFailure("quaternion must contain exactly [qx,qy,qz,qw]")
    q = [_finite_number(v, f"quaternion[{i}]") for i, v in enumerate(values)]
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= QUATERNION_MIN_NORM:
        raise ImportFailure("quaternion norm is zero or too small to normalize safely")
    q = [v / norm for v in q]

    # q and -q encode the same rotation.  Canonicalizing the sign makes drafts
    # deterministic across upstream solvers/runs without changing the matrix.
    for component in (q[3], q[0], q[1], q[2]):
        if abs(component) > 1.0e-15:
            if component < 0.0:
                q = [-v for v in q]
            break
    return q, norm


def quaternion_xyzw_to_rotation(q: Sequence[Any]) -> list[list[float]]:
    qn, _ = normalize_quaternion_xyzw(q)
    x, y, z, w = qn
    return [
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ]


def transform_from_upstream_vector(values: Sequence[Any]) -> tuple[
        list[list[float]], list[float], float]:
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ImportFailure(
            f"{UPSTREAM_KEY} must be exactly [x,y,z,qx,qy,qz,qw]")
    translation = [
        _finite_number(values[i], f"{UPSTREAM_KEY}[{i}]") for i in range(3)
    ]
    qn, original_norm = normalize_quaternion_xyzw(values[3:7])
    rotation = quaternion_xyzw_to_rotation(qn)
    transform = [rotation[i] + [translation[i]] for i in range(3)]
    transform.append([0.0, 0.0, 0.0, 1.0])
    validate_se3(transform, "T_lidar_camera")
    return transform, qn, original_norm


def validate_se3(transform: Sequence[Sequence[Any]], label: str = "transform") -> dict[str, float]:
    if (not isinstance(transform, (list, tuple)) or len(transform) != 4 or
            any(not isinstance(row, (list, tuple)) or len(row) != 4
                for row in transform)):
        raise ImportFailure(f"{label} must be a 4x4 matrix")
    t = [[_finite_number(value, f"{label}[{i}][{j}]")
          for j, value in enumerate(row)]
         for i, row in enumerate(transform)]
    bottom_error = max(abs(t[3][j] - (1.0 if j == 3 else 0.0)) for j in range(4))
    if bottom_error > SE3_TOLERANCE:
        raise ImportFailure(f"{label} has an invalid homogeneous bottom row")

    r = [row[:3] for row in t[:3]]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    orthogonality_error = _max_abs_delta(
        _matmul(rt, r), [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)])
    determinant = _det3(r)
    if orthogonality_error > SE3_TOLERANCE:
        raise ImportFailure(
            f"{label} rotation is not orthonormal (max error {orthogonality_error:.3g})")
    if abs(determinant - 1.0) > SE3_TOLERANCE:
        raise ImportFailure(
            f"{label} rotation determinant is {determinant:.17g}, expected +1")
    return {
        "rotation_orthogonality_max_abs": orthogonality_error,
        "rotation_determinant": determinant,
        "homogeneous_row_max_abs_error": bottom_error,
    }


def invert_se3(transform: Sequence[Sequence[Any]]) -> list[list[float]]:
    validate_se3(transform)
    t = [[float(value) for value in row] for row in transform]
    r = [row[:3] for row in t[:3]]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    translation = [t[i][3] for i in range(3)]
    inverse_translation = [
        -sum(rt[i][j] * translation[j] for j in range(3)) for i in range(3)
    ]
    inverse = [rt[i] + [inverse_translation[i]] for i in range(3)]
    inverse.append([0.0, 0.0, 0.0, 1.0])
    validate_se3(inverse, "inverse transform")
    return inverse


def apply_transform(transform: Sequence[Sequence[float]],
                    point: Sequence[float]) -> list[float]:
    if len(point) != 3:
        raise ImportFailure("point must have three coordinates")
    return [sum(float(transform[i][j]) * float(point[j]) for j in range(3))
            + float(transform[i][3]) for i in range(3)]


def sha256_file(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    try:
        with path.open("rb") as stream:
            while True:
                block = stream.read(1024 * 1024)
                if not block:
                    break
                size += len(block)
                digest.update(block)
    except OSError as exc:
        raise ImportFailure(f"cannot hash file {path}: {exc}") from exc
    return digest.hexdigest(), size


def sha256_path(path: Path) -> dict[str, Any]:
    """Hash a file byte-for-byte or a directory as a canonical file tree."""
    path = path.resolve()
    if path.is_symlink():
        raise ImportFailure(f"source-data symlinks are forbidden: {path}")
    if path.is_file():
        digest, size = sha256_file(path)
        return {
            "path": str(path),
            "sha256": digest,
            "hash_kind": "sha256-file-v1",
            "byte_count": size,
            "file_count": 1,
        }
    if not path.is_dir():
        raise ImportFailure(f"source-data path does not exist: {path}")

    files = sorted(item for item in path.rglob("*") if item.is_file() or item.is_symlink())
    digest = hashlib.sha256(b"d435i-calib-tree-sha256-v1\0")
    byte_count = 0
    for item in files:
        if item.is_symlink():
            raise ImportFailure(f"source-data tree contains a symlink: {item}")
        relative = item.relative_to(path).as_posix().encode("utf-8")
        file_digest, size = sha256_file(item)
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(size.to_bytes(16, "big"))
        digest.update(bytes.fromhex(file_digest))
        byte_count += size
    return {
        "path": str(path),
        "sha256": digest.hexdigest(),
        "hash_kind": "sha256-tree-v1",
        "byte_count": byte_count,
        "file_count": len(files),
    }


def _nonempty(value: str, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ImportFailure(f"{label} must be a non-empty string")
    return value.strip()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def build_draft(
    upstream_doc: dict[str, Any],
    *,
    upstream_path: Path,
    rig_id: str,
    mount_session_id: str | None = None,
    lidar_serial: str,
    camera_serial: str,
    lidar_model: str = "Livox Mid-360S",
    camera_model: str = "Intel RealSense D435i",
    source_paths: Sequence[Path] = (),
    solver_version: str | None = None,
    solver_commit: str | None = None,
    solver_command: str | None = None,
    method_note: str | None = None,
    created_utc: str | None = None,
) -> dict[str, Any]:
    """Build, but do not write, a draft project result."""
    results = upstream_doc.get("results")
    if not isinstance(results, dict) or "T_lidar_camera" not in results:
        raise ImportFailure(f"upstream JSON is missing {UPSTREAM_KEY}")
    raw_vector = results["T_lidar_camera"]
    t_lidar_camera, normalized_q, q_norm = transform_from_upstream_vector(raw_vector)
    t_camera_lidar = invert_se3(t_lidar_camera)
    inverse_twice = invert_se3(t_camera_lidar)
    inverse_residual = max(
        _max_abs_delta(_matmul(t_lidar_camera, t_camera_lidar), _identity4()),
        _max_abs_delta(_matmul(t_camera_lidar, t_lidar_camera), _identity4()),
    )
    double_inverse_residual = _max_abs_delta(inverse_twice, t_lidar_camera)
    if max(inverse_residual, double_inverse_residual) > SE3_TOLERANCE:
        raise ImportFailure("SE(3) inverse consistency check failed")

    upstream_path = upstream_path.resolve()
    upstream_hash = sha256_path(upstream_path)
    sources = [dict(upstream_hash, role="solver_output")]
    for source_path in source_paths:
        sources.append(dict(sha256_path(source_path), role="calibration"))

    top_metadata = {key: value for key, value in upstream_doc.items() if key != "results"}
    result_metadata = {key: value for key, value in results.items()
                       if key != "T_lidar_camera"}
    method_metadata = {
        "solver": "direct_visual_lidar_calibration",
        "solver_version": solver_version,
        "solver_commit": solver_commit,
        "solver_command": solver_command,
        "note": method_note,
        "upstream_top_level_metadata": top_metadata,
        "upstream_results_metadata": result_metadata,
    }

    draft = {
        "schema_version": 1,
        "draft_schema": SCHEMA,
        "task_id": TASK_ID,
        "status": "draft",
        "devices": [
            {"role": "lidar", "model": _nonempty(lidar_model, "lidar_model"),
             "serial": _nonempty(lidar_serial, "lidar_serial")},
            {"role": "rgbd", "model": _nonempty(camera_model, "camera_model"),
             "serial": _nonempty(camera_serial, "camera_serial")},
        ],
        "rig_id": _nonempty(rig_id, "rig_id"),
        "created_utc": created_utc or _utc_now(),
        "method": "direct_visual_lidar_calibration; audited convention inversion import",
        "method_metadata": method_metadata,
        "source_data": sources,
        "frame_convention": {
            "from": "livox_frame",
            "to": "camera_color_optical_frame",
            "equation": "p_camera = T_camera_lidar * p_lidar",
            "upstream_equation": "p_lidar = T_lidar_camera * p_camera",
        },
        "result": {
            "T_camera_lidar": t_camera_lidar,
        },
        "import_provenance": {
            "upstream_transform_key": UPSTREAM_KEY,
            "upstream_T_lidar_camera_vector_raw_xyzw": list(raw_vector),
            "upstream_quaternion_original_norm": q_norm,
            "upstream_quaternion_normalized_xyzw": normalized_q,
            "T_lidar_camera_from_normalized_quaternion": t_lidar_camera,
            "T_camera_lidar_analytic_inverse": t_camera_lidar,
            "T_lidar_camera_se3": validate_se3(t_lidar_camera, "T_lidar_camera"),
            "T_camera_lidar_se3": validate_se3(t_camera_lidar, "T_camera_lidar"),
            "inverse_identity_max_abs": inverse_residual,
            "double_inverse_max_abs": double_inverse_residual,
        },
        "summary": {
            "state": "draft; independent validation not run",
            "translation_lidar_to_camera_m": [row[3] for row in t_camera_lidar[:3]],
            "quaternion_input_norm": q_norm,
            "inverse_identity_max_abs": inverse_residual,
            "source_hash_count": len(sources),
        },
        "next_step": (
            "Collect an independent holdout dataset and compute the registered projection, "
            "capture-count, and repeatability gates before producing a validated result."
        ),
    }
    if mount_session_id is not None:
        draft["mount_session_id"] = _nonempty(
            mount_session_id, "mount_session_id")
    # Intentionally no validation object: importing a solver output is not an
    # independent validation and must not manufacture gate values/statuses.
    return draft


def _atomic_json_write(path: Path, document: dict[str, Any], force: bool) -> None:
    path = path.resolve()
    if path.exists() and not force:
        raise ImportFailure(f"output already exists (use --force explicitly): {path}")
    if path.exists() and force:
        try:
            existing = load_strict_json(path)
        except ImportFailure:
            existing = None
        if isinstance(existing, dict) and str(existing.get("status", "")).lower() == "validated":
            raise ImportFailure(
                f"refusing to overwrite a validated result with a draft: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _run_selftest() -> dict[str, Any]:
    # Direction test: +90 deg about z, then translation [1,2,3].
    half = math.sqrt(0.5)
    forward, _, norm = transform_from_upstream_vector(
        [1.0, 2.0, 3.0, 0.0, 0.0, 2.0 * half, 2.0 * half])
    inverse = invert_se3(forward)
    camera_point = [1.0, 0.0, 0.0]
    lidar_point = apply_transform(forward, camera_point)
    recovered = apply_transform(inverse, lidar_point)
    if max(abs(a - b) for a, b in zip(lidar_point, [1.0, 3.0, 3.0])) > 1.0e-12:
        raise AssertionError("forward convention test failed")
    if max(abs(a - b) for a, b in zip(recovered, camera_point)) > 1.0e-12:
        raise AssertionError("inverse convention test failed")
    if abs(norm - 2.0) > 1.0e-12:
        raise AssertionError("quaternion normalization test failed")
    if _max_abs_delta(invert_se3(inverse), forward) > 1.0e-12:
        raise AssertionError("double inverse test failed")

    rejected = 0
    for invalid in (
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, float("inf"), 0, 0, 0, 1],
        [True, 0, 0, 0, 0, 0, 1],
    ):
        try:
            transform_from_upstream_vector(invalid)
        except ImportFailure:
            rejected += 1
    if rejected != 4:
        raise AssertionError("invalid input rejection test failed")
    return {
        "status": "passed",
        "schema": SCHEMA,
        "checks": [
            "quaternion_normalization",
            "known_transform_direction",
            "analytic_inverse",
            "double_inverse",
            "invalid_value_rejection",
        ],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Import direct_visual_lidar_calibration calib.json as a draft "
            "T_camera_lidar result; never marks validation as passed."
        ))
    parser.add_argument("input", nargs="?", help="upstream calib.json")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"draft JSON path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--rig-id")
    parser.add_argument("--mount-session-id")
    parser.add_argument("--lidar-serial")
    parser.add_argument("--camera-serial")
    parser.add_argument("--lidar-model", default="Livox Mid-360S")
    parser.add_argument("--camera-model", default="Intel RealSense D435i")
    parser.add_argument("--source-data", action="append", default=[], metavar="PATH",
                        help="calibration bag/file/directory to hash; repeatable")
    parser.add_argument("--solver-version")
    parser.add_argument("--solver-commit")
    parser.add_argument("--solver-command")
    parser.add_argument("--method-note")
    parser.add_argument("--force", action="store_true",
                        help="explicitly replace an existing draft output")
    parser.add_argument("--selftest", action="store_true",
                        help="run deterministic math/input checks and exit")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.selftest:
            print(json.dumps(_run_selftest(), ensure_ascii=False, sort_keys=True))
            return 0
        missing = [name for name in ("input", "rig_id", "lidar_serial", "camera_serial")
                   if not getattr(args, name)]
        if missing:
            raise ImportFailure("missing required arguments: " + ", ".join(missing))

        input_path = Path(args.input).resolve()
        output_path = Path(args.output).resolve()
        if input_path == output_path:
            raise ImportFailure("input and output paths must differ")
        upstream = load_strict_json(input_path)
        draft = build_draft(
            upstream,
            upstream_path=input_path,
            rig_id=args.rig_id,
            mount_session_id=args.mount_session_id,
            lidar_serial=args.lidar_serial,
            camera_serial=args.camera_serial,
            lidar_model=args.lidar_model,
            camera_model=args.camera_model,
            source_paths=[Path(path) for path in args.source_data],
            solver_version=args.solver_version,
            solver_commit=args.solver_commit,
            solver_command=args.solver_command,
            method_note=args.method_note,
        )
        _atomic_json_write(output_path, draft, args.force)
        output_sha256, output_size = sha256_file(output_path)
        summary = {
            "status": "ok",
            "task_status": "draft",
            "artifact": str(output_path),
            "artifact_sha256": output_sha256,
            "artifact_bytes": output_size,
            "T_camera_lidar": draft["result"]["T_camera_lidar"],
            "inverse_identity_max_abs":
                draft["import_provenance"]["inverse_identity_max_abs"],
            "source_hash_count": len(draft["source_data"]),
            "validation_created": False,
            "warnings": ([] if args.source_data else [
                "No raw calibration dataset was supplied with --source-data; only calib.json is hashed."
            ]),
        }
        print(json.dumps(summary, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return 0
    except (ImportFailure, OSError) as exc:
        print(json.dumps({"status": "error", "error": str(exc)},
                         ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
