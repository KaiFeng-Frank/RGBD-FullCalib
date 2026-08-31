#!/usr/bin/env python3
"""Validate D435i delivered IR1/IR2 epipolar alignment on an independent set.

This is deliberately a validation tool, not a calibration tool.  The primary
metric is measured on the PNG pixels exactly as saved by tools/capture.py.  No
undistort, stereoRectify, remap, fitted transform, or cam_trio calibration is
allowed in the metric path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any, Iterable, Sequence

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_ID = "d435i-delivered-ir-epipolar-v1"
PROTOCOL_DOC = ROOT / "docs" / "STEREO_RECTIFICATION_ACCEPTANCE.md"
DEFAULT_FRAMES = ROOT / "data" / "cam_trio_frames"
DEFAULT_CALIBRATION_FRAMES = ROOT / "data" / "cam_ir_frames"
DEFAULT_CALIBRATION_CHAIN = ROOT / "data" / "cam_ir-camchain.yaml"
DEFAULT_FACTORY = ROOT / "results" / "factory_params.json"
DEFAULT_BAG = ROOT / "data" / "cam_trio.bag"
DEFAULT_OUTPUT = ROOT / "results" / "stereo_rectification_validation.json"
DEFAULT_PLOT = ROOT / "results" / "stereo_rectification_validation.png"
CAPTURE_TOOL = ROOT / "tools" / "capture.py"

EXPECTED_STEMS = tuple(f"{i:04d}" for i in range(16))
EXPECTED_SIZE = (1280, 720)  # width, height
EXPECTED_SERIAL = "947122070908"
EXPECTED_FIRMWARE = "5.12.7.100"
TARGET_TAG_IDS = frozenset(range(36))

SUPPORT_LIMITS = {
    "minimum_pairs_with_common_tags": 12,
    "minimum_matched_corners": 600,
    "minimum_grid_cells": 6,
    "minimum_grid_rows": 2,
    "minimum_grid_columns": 2,
}
PASS_LIMITS = {
    "median_abs_vertical_px": 1.0,
    "p95_abs_vertical_px": 1.5,
    "p99_abs_vertical_px": 2.0,
    "fraction_abs_vertical_gt_2px": 0.01,
}


def make_detector() -> cv2.aruco.ArucoDetector:
    """Build the detector frozen in the v1 acceptance protocol."""
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 15
    p.adaptiveThreshWinSizeStep = 2
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def marker_map(corners: Sequence[np.ndarray], ids: np.ndarray | None,
               allowed_ids: frozenset[int] | None = None) -> dict[int, np.ndarray]:
    """Return tag_id -> canonical OpenCV corner array; reject ambiguous IDs."""
    if ids is None:
        return {}
    flat_ids = np.asarray(ids).reshape(-1)
    if len(corners) != len(flat_ids):
        raise ValueError("detector returned a different number of corners and IDs")
    out: dict[int, np.ndarray] = {}
    for raw_id, raw_corners in zip(flat_ids, corners):
        tag_id = int(raw_id)
        if allowed_ids is not None and tag_id not in allowed_ids:
            continue
        if tag_id in out:
            raise ValueError(f"duplicate detected tag id {tag_id}")
        pts = np.asarray(raw_corners, dtype=np.float64).reshape(-1, 2)
        if pts.shape != (4, 2) or not np.isfinite(pts).all():
            raise ValueError(f"invalid corners for tag {tag_id}")
        out[tag_id] = pts
    return out


def match_markers(left: dict[int, np.ndarray], right: dict[int, np.ndarray]) -> tuple[np.ndarray, np.ndarray, list[dict[str, int]]]:
    """Match only by (tag ID, canonical corner index), never by proximity."""
    lpts: list[np.ndarray] = []
    rpts: list[np.ndarray] = []
    keys: list[dict[str, int]] = []
    for tag_id in sorted(set(left) & set(right)):
        if left[tag_id].shape != (4, 2) or right[tag_id].shape != (4, 2):
            raise ValueError(f"tag {tag_id} does not have exactly four canonical corners")
        for corner_index in range(4):
            lpts.append(left[tag_id][corner_index])
            rpts.append(right[tag_id][corner_index])
            keys.append({"tag_id": tag_id, "corner_index": corner_index})
    if not lpts:
        empty = np.empty((0, 2), dtype=np.float64)
        return empty, empty.copy(), []
    return np.asarray(lpts), np.asarray(rpts), keys


def summarize_vertical(dy: Iterable[float]) -> dict[str, Any]:
    values = np.asarray(list(dy), dtype=np.float64)
    if values.size == 0:
        return {
            "count": 0,
            "signed_median_px": None,
            "abs_vertical_px": {
                "mean": None, "median": None, "p95": None, "p99": None,
                "max": None, "rms": None,
            },
            "fraction_abs_vertical_gt_1px": None,
            "fraction_abs_vertical_gt_2px": None,
        }
    if not np.isfinite(values).all():
        raise ValueError("vertical residuals contain non-finite values")
    absolute = np.abs(values)
    return {
        "count": int(values.size),
        "signed_median_px": float(np.median(values)),
        "abs_vertical_px": {
            "mean": float(np.mean(absolute)),
            "median": float(np.median(absolute)),
            "p95": float(np.percentile(absolute, 95)),
            "p99": float(np.percentile(absolute, 99)),
            "max": float(np.max(absolute)),
            "rms": float(np.sqrt(np.mean(np.square(absolute)))),
        },
        "fraction_abs_vertical_gt_1px": float(np.mean(absolute > 1.0)),
        "fraction_abs_vertical_gt_2px": float(np.mean(absolute > 2.0)),
    }


def grid_coverage(midpoints: np.ndarray, width: int, height: int) -> dict[str, Any]:
    pts = np.asarray(midpoints, dtype=np.float64).reshape(-1, 2)
    if pts.size == 0:
        cells: set[tuple[int, int]] = set()
    else:
        if not np.isfinite(pts).all():
            raise ValueError("grid midpoints contain non-finite values")
        cols = np.clip((pts[:, 0] / width * 3).astype(int), 0, 2)
        rows = np.clip((pts[:, 1] / height * 3).astype(int), 0, 2)
        cells = {(int(row), int(col)) for row, col in zip(rows, cols)}
    covered_rows = sorted({row for row, _ in cells})
    covered_columns = sorted({col for _, col in cells})
    return {
        "grid": "3x3",
        "covered_cells": [[row, col] for row, col in sorted(cells)],
        "covered_cell_count": len(cells),
        "covered_rows": covered_rows,
        "covered_row_count": len(covered_rows),
        "covered_columns": covered_columns,
        "covered_column_count": len(covered_columns),
    }


def evaluate_acceptance(support: dict[str, Any], metrics: dict[str, Any]) -> dict[str, Any]:
    support_checks = [
        _check("pairs_with_common_tags", support["pairs_with_common_tags"], "ge",
               SUPPORT_LIMITS["minimum_pairs_with_common_tags"]),
        _check("matched_corners", support["matched_corners"], "ge",
               SUPPORT_LIMITS["minimum_matched_corners"]),
        _check("grid_cells", support["coverage"]["covered_cell_count"], "ge",
               SUPPORT_LIMITS["minimum_grid_cells"]),
        _check("grid_rows", support["coverage"]["covered_row_count"], "ge",
               SUPPORT_LIMITS["minimum_grid_rows"]),
        _check("grid_columns", support["coverage"]["covered_column_count"], "ge",
               SUPPORT_LIMITS["minimum_grid_columns"]),
    ]
    sufficient = all(row["passed"] for row in support_checks)

    abs_metrics = metrics["abs_vertical_px"]
    gate_checks: list[dict[str, Any]] = []
    if metrics["count"]:
        gate_checks = [
            _check("median_abs_vertical_px", abs_metrics["median"], "le",
                   PASS_LIMITS["median_abs_vertical_px"]),
            _check("p95_abs_vertical_px", abs_metrics["p95"], "le",
                   PASS_LIMITS["p95_abs_vertical_px"]),
            _check("p99_abs_vertical_px", abs_metrics["p99"], "le",
                   PASS_LIMITS["p99_abs_vertical_px"]),
            _check("fraction_abs_vertical_gt_2px",
                   metrics["fraction_abs_vertical_gt_2px"], "le",
                   PASS_LIMITS["fraction_abs_vertical_gt_2px"]),
        ]
    gates_passed = bool(gate_checks) and all(row["passed"] for row in gate_checks)
    status = "passed" if sufficient and gates_passed else ("failed" if sufficient else "insufficient")
    return {
        "status": status,
        "sufficient": sufficient,
        "sufficient_numeric": int(sufficient),
        "passed": status == "passed",
        "pass_numeric": int(status == "passed"),
        "support_checks": support_checks,
        "gate_checks": gate_checks,
    }


def _check(name: str, value: float | int, compare: str, limit: float | int) -> dict[str, Any]:
    if compare == "le":
        passed = float(value) <= float(limit)
    elif compare == "ge":
        passed = float(value) >= float(limit)
    else:
        raise ValueError(f"unknown comparison {compare}")
    return {"id": name, "value": value, "compare": compare, "limit": limit, "passed": passed}


def discover_pairs(frame_dir: Path, expected_stems: Sequence[str] = EXPECTED_STEMS) -> list[tuple[str, Path, Path]]:
    if not frame_dir.is_dir():
        raise FileNotFoundError(f"evaluation directory not found: {frame_dir}")
    actual = sorted(path.stem for path in frame_dir.glob("[0-9][0-9][0-9][0-9].png"))
    actual_right = sorted(
        path.name.removesuffix("_r.png")
        for path in frame_dir.glob("[0-9][0-9][0-9][0-9]_r.png"))
    expected = list(expected_stems)
    if actual != expected or actual_right != expected:
        left_missing = sorted(set(expected) - set(actual))
        left_extra = sorted(set(actual) - set(expected))
        right_missing = sorted(set(expected) - set(actual_right))
        right_extra = sorted(set(actual_right) - set(expected))
        raise ValueError(
            "frozen evaluation set changed; "
            f"left_missing={left_missing}, left_extra={left_extra}, "
            f"right_missing={right_missing}, right_extra={right_extra}")
    pairs = []
    for stem in expected:
        left = frame_dir / f"{stem}.png"
        right = frame_dir / f"{stem}_r.png"
        if not right.is_file():
            raise FileNotFoundError(f"missing frozen right image: {right}")
        pairs.append((stem, left, right))
    return pairs


def require_y8(image: np.ndarray | None, stem: str, side: str,
               width: int, height: int) -> np.ndarray:
    if image is None:
        raise ValueError(f"cannot decode {side} image in pair {stem}")
    if image.ndim != 2 or image.dtype != np.uint8 or image.shape != (height, width):
        raise ValueError(
            f"pair {stem} {side} is not raw {width}x{height} Y8: "
            f"shape={image.shape}, dtype={image.dtype}")
    return image


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _files_digest(paths: Sequence[Path], base: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda p: p.relative_to(base).as_posix()):
        relative = path.relative_to(base).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(bytes.fromhex(_sha256(path)))
    return digest.hexdigest()


def _content_hash_index(paths: Sequence[Path]) -> dict[str, list[str]]:
    """Index exact file content independent of path, for holdout leakage checks."""
    out: dict[str, list[str]] = {}
    for path in paths:
        out.setdefault(_sha256(path), []).append(path.name)
    return out


def _relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def _git_provenance() -> dict[str, Any]:
    def run(*args: str) -> str | None:
        proc = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, check=False)
        return proc.stdout.strip() if proc.returncode == 0 else None
    commit = run("git", "rev-parse", "HEAD")
    dirty = run("git", "status", "--porcelain")
    return {"commit": commit, "worktree_dirty": bool(dirty) if dirty is not None else None}


def validate(frame_dir: Path, calibration_dir: Path, calibration_chain: Path,
             factory_path: Path, bag_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    frame_dir = frame_dir.resolve()
    calibration_dir = calibration_dir.resolve()
    if frame_dir == calibration_dir:
        raise ValueError("evaluation and calibration directories must be independent")
    if not calibration_chain.is_file():
        raise FileNotFoundError(f"calibration camchain provenance not found: {calibration_chain}")
    pairs = discover_pairs(frame_dir)
    evaluation_images = [path for _, left, right in pairs for path in (left, right)]
    calibration_images = sorted(calibration_dir.glob("*.png"))
    if not calibration_images:
        raise ValueError(f"calibration provenance has no PNG images: {calibration_dir}")
    evaluation_image_digest = _files_digest(evaluation_images, frame_dir)
    calibration_image_digest = _files_digest(calibration_images, calibration_dir)
    evaluation_content = _content_hash_index(evaluation_images)
    calibration_content = _content_hash_index(calibration_images)
    duplicate_hashes = sorted(set(evaluation_content) & set(calibration_content))
    if duplicate_hashes:
        examples = [
            {"evaluation": evaluation_content[digest], "calibration": calibration_content[digest]}
            for digest in duplicate_hashes[:3]
        ]
        raise ValueError(f"evaluation images overlap the calibration set: {examples}")
    factory = json.loads(factory_path.read_text(encoding="utf-8"))
    if str(factory.get("serial")) != EXPECTED_SERIAL:
        raise ValueError(f"protocol is frozen to serial {EXPECTED_SERIAL}, got {factory.get('serial')}")
    if str(factory.get("fw")) != EXPECTED_FIRMWARE:
        raise ValueError(
            f"protocol is frozen to firmware {EXPECTED_FIRMWARE}, got {factory.get('fw')}")
    factory_ir = factory.get("ir1_1280x720", {})
    if (factory_ir.get("w"), factory_ir.get("h")) != EXPECTED_SIZE:
        raise ValueError(f"factory IR profile is not frozen {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]}")
    detector = make_detector()

    all_dy: list[float] = []
    all_midpoints: list[np.ndarray] = []
    per_pair: list[dict[str, Any]] = []
    pairs_with_common_tags = 0
    matched_tags = 0
    width, height = EXPECTED_SIZE

    for stem, left_path, right_path in pairs:
        left_image = cv2.imread(str(left_path), cv2.IMREAD_UNCHANGED)
        right_image = cv2.imread(str(right_path), cv2.IMREAD_UNCHANGED)
        left_image = require_y8(left_image, stem, "left", width, height)
        right_image = require_y8(right_image, stem, "right", width, height)

        left_corners, left_ids, _ = detector.detectMarkers(left_image)
        right_corners, right_ids, _ = detector.detectMarkers(right_image)
        left = marker_map(left_corners, left_ids, TARGET_TAG_IDS)
        right = marker_map(right_corners, right_ids, TARGET_TAG_IDS)
        lpts, rpts, keys = match_markers(left, right)
        common_tags = len(keys) // 4
        if common_tags:
            pairs_with_common_tags += 1
            matched_tags += common_tags
        dy = lpts[:, 1] - rpts[:, 1] if len(lpts) else np.empty(0)
        midpoints = (lpts + rpts) / 2.0 if len(lpts) else np.empty((0, 2))
        all_dy.extend(float(x) for x in dy)
        if len(midpoints):
            all_midpoints.append(midpoints)
        pair_metrics = summarize_vertical(dy)
        per_pair.append({
            "stem": stem,
            "left_tags_detected": len(left),
            "right_tags_detected": len(right),
            "common_tags": common_tags,
            "matched_corners": len(keys),
            "signed_median_px": pair_metrics["signed_median_px"],
            "median_abs_vertical_px": pair_metrics["abs_vertical_px"]["median"],
            "p95_abs_vertical_px": pair_metrics["abs_vertical_px"]["p95"],
            "max_abs_vertical_px": pair_metrics["abs_vertical_px"]["max"],
        })

    midpoint_array = np.concatenate(all_midpoints) if all_midpoints else np.empty((0, 2))
    metrics = summarize_vertical(all_dy)
    coverage = grid_coverage(midpoint_array, width, height)
    support = {
        "pairs_total": len(pairs),
        "pairs_processed": len(pairs),
        "pairs_with_common_tags": pairs_with_common_tags,
        "matched_tags": matched_tags,
        "matched_corners": len(all_dy),
        "coverage": coverage,
    }
    decision = evaluate_acceptance(support, metrics)

    times_path = frame_dir / "times.txt"
    calibration_times = calibration_dir / "times.txt"

    from datetime import datetime, timezone
    result = {
        "schema_version": 1,
        "artifact_type": "stereo_rectification_validation",
        "role": "validation_only",
        "is_calibration_parameter": False,
        "slam_blocking": False,
        "claim_scope": "D435i IR1/IR2 frames delivered by firmware/SDK and saved without remap",
        "protocol": {
            "id": PROTOCOL_ID,
            "document": _relative(PROTOCOL_DOC),
            "document_sha256": _sha256(PROTOCOL_DOC),
            "frozen_date": "2026-08-31",
            "support_limits": SUPPORT_LIMITS,
            "pass_limits": PASS_LIMITS,
            "percentile_method": "numpy linear interpolation",
        },
        "device": {
            "model": factory.get("device"),
            "serial": factory.get("serial"),
            "firmware": factory.get("fw"),
        },
        "inputs": {
            "evaluation_set": {
                "path": _relative(frame_dir),
                "role": "independent_holdout",
                "pair_stems": list(EXPECTED_STEMS),
                "image_set_sha256": evaluation_image_digest,
                "times_sha256": _sha256(times_path) if times_path.is_file() else None,
                "bag_path": _relative(bag_path),
                "bag_sha256": _sha256(bag_path) if bag_path.is_file() else None,
            },
            "calibration_set": {
                "path": _relative(calibration_dir),
                "role": "calibration_input_used_only_to_prove_holdout_independence",
                "image_set_sha256": calibration_image_digest,
                "times_sha256": _sha256(calibration_times) if calibration_times.is_file() else None,
                "camchain_path": _relative(calibration_chain),
                "camchain_sha256": _sha256(calibration_chain),
                "camchain_role": "provenance_not_used_for_metric",
            },
            "factory_params": {
                "path": _relative(factory_path),
                "sha256": _sha256(factory_path),
                "role": "device_identity_only",
            },
        },
        "profile": {"width": width, "height": height, "format": "Y8"},
        "method": {
            "detector": "OpenCV DICT_APRILTAG_36h11",
            "accepted_tag_ids": {"minimum": 0, "maximum": 35},
            "adaptive_threshold_windows": {"min": 3, "max": 15, "step": 2},
            "corner_refinement": "CORNER_REFINE_SUBPIX",
            "corner_refinement_window": 5,
            "correspondence": "tag_id + canonical_corner_index",
            "signed_error": "y_IR1 - y_IR2",
            "remap_applied": False,
            "undistort_applied": False,
            "fitted_parameters": False,
            "cam_trio_camchain_used": False,
            "manual_frame_or_corner_rejection": False,
        },
        "support": support,
        "metrics": metrics,
        "validation": decision,
        "per_pair": per_pair,
        "provenance": {
            "tool": _relative(Path(__file__)),
            "tool_sha256": _sha256(Path(__file__)),
            "current_capture_tool_reference": {
                "path": _relative(CAPTURE_TOOL),
                "sha256": _sha256(CAPTURE_TOOL),
                "role": "implementation_reference_not_historical_capture_proof",
            },
            "command": None,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "python": platform.python_version(),
            "opencv": cv2.__version__,
            "numpy": np.__version__,
            "git": _git_provenance(),
        },
    }
    plot_data = {"dy": np.asarray(all_dy), "midpoints": midpoint_array, "per_pair": per_pair}
    return result, plot_data


def write_plot(path: Path, result: dict[str, Any], plot_data: dict[str, Any]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dy = plot_data["dy"]
    mids = plot_data["midpoints"]
    per_pair = plot_data["per_pair"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)

    color_limit = max(2.0, float(np.max(np.abs(dy)))) if dy.size else 2.0
    sc = axes[0].scatter(mids[:, 0], mids[:, 1], c=dy, s=9, cmap="coolwarm",
                         vmin=-color_limit, vmax=color_limit)
    axes[0].invert_yaxis()
    axes[0].set(xlim=(0, EXPECTED_SIZE[0]), ylim=(EXPECTED_SIZE[1], 0),
                xlabel="image x (px)", ylabel="image y (px)", title="Matched corners; color = signed dy")
    fig.colorbar(sc, ax=axes[0], label="y_IR1 - y_IR2 (px)")

    axes[1].hist(np.abs(dy), bins=60, color="#356d9e", alpha=0.85)
    for x, label, color in ((1.0, "median gate", "#2a9d55"),
                            (1.5, "P95 gate", "#e69f00"),
                            (2.0, "P99 gate", "#c43c39")):
        axes[1].axvline(x, color=color, linestyle="--", linewidth=1.4, label=f"{label}: {x:g}px")
    axes[1].set(xlabel="|dy| (px)", ylabel="matched corners", title="Raw delivered IR vertical residual")
    axes[1].legend(fontsize=8)

    values = [row["p95_abs_vertical_px"] if row["p95_abs_vertical_px"] is not None else np.nan
              for row in per_pair]
    axes[2].bar(range(len(values)), values, color="#6c77b5")
    axes[2].axhline(1.5, color="#e69f00", linestyle="--", linewidth=1.4, label="aggregate P95 gate")
    axes[2].set(xticks=range(len(values)), xticklabels=[row["stem"] for row in per_pair],
                xlabel="holdout pair", ylabel="per-pair P95 |dy| (px)", title="No pair removed")
    axes[2].tick_params(axis="x", rotation=70, labelsize=7)
    axes[2].legend(fontsize=8)

    m = result["metrics"]["abs_vertical_px"]
    fmt = lambda value: "—" if value is None else f"{value:.3f}px"
    fig.suptitle(
        f"D435i delivered-IR rectification: {result['validation']['status'].upper()}  "
        f"N={result['metrics']['count']}  median={fmt(m['median'])}  "
        f"P95={fmt(m['p95'])}  P99={fmt(m['p99'])}",
        fontsize=12,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, format="png")
    plt.close(fig)


def _resolve(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate raw delivered D435i stereo IR rectification")
    parser.add_argument("--frames", default=str(DEFAULT_FRAMES))
    parser.add_argument("--calibration-frames", default=str(DEFAULT_CALIBRATION_FRAMES))
    parser.add_argument("--calibration-chain", default=str(DEFAULT_CALIBRATION_CHAIN))
    parser.add_argument("--factory", default=str(DEFAULT_FACTORY))
    parser.add_argument("--bag", default=str(DEFAULT_BAG))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--plot", default=str(DEFAULT_PLOT))
    parser.add_argument("--force", action="store_true", help="replace an existing result explicitly")
    used_argv = list(sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(used_argv)

    output = _resolve(args.output)
    plot = _resolve(args.plot)
    if output.resolve() == plot.resolve():
        parser.error("--output and --plot must be different files")
    if (output.exists() or plot.exists()) and not args.force:
        parser.error("result already exists; use a different path or pass --force explicitly")

    result, plot_data = validate(
        _resolve(args.frames), _resolve(args.calibration_frames),
        _resolve(args.calibration_chain), _resolve(args.factory), _resolve(args.bag))
    result["provenance"]["command"] = [sys.executable, _relative(Path(__file__)), *used_argv]
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary_plot = plot.with_suffix(plot.suffix + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    try:
        write_plot(temporary_plot, result, plot_data)
        os.replace(temporary, output)
        os.replace(temporary_plot, plot)
    finally:
        temporary.unlink(missing_ok=True)
        temporary_plot.unlink(missing_ok=True)

    metrics = result["metrics"]["abs_vertical_px"]
    support = result["support"]
    def display(value: float | None, suffix: str = "") -> str:
        return "—" if value is None else f"{value:.4f}{suffix}"
    print(f"{result['validation']['status'].upper()}: raw delivered IR1/IR2; remap=false")
    print(f"  pairs {support['pairs_with_common_tags']}/{support['pairs_total']}, "
          f"corners {support['matched_corners']}, grid {support['coverage']['covered_cell_count']}/9")
    print(f"  |dy| median {display(metrics['median'], 'px')}, "
          f"P95 {display(metrics['p95'], 'px')}, "
          f"P99 {display(metrics['p99'], 'px')}, max {display(metrics['max'], 'px')}")
    fraction = result["metrics"]["fraction_abs_vertical_gt_2px"]
    print(f"  |dy|>2px {display(None if fraction is None else fraction * 100, '%')}")
    print(f"  JSON: {_relative(output)}")
    print(f"  plot: {_relative(plot)}")
    return {"passed": 0, "failed": 2, "insufficient": 3}[result["validation"]["status"]]


if __name__ == "__main__":
    sys.exit(main())
