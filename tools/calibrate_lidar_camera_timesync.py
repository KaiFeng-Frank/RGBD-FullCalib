#!/usr/bin/env python3
"""Offline MID-360S <-> D435i effective timestamp-offset estimator.

The estimator aligns the two gyroscopes.  For each candidate offset it
interpolates the D435i gyro at ``t_livox + offset`` and jointly fits a proper
3-D rotation plus a constant bias using Wahba/Kabsch.  The public convention
is therefore unambiguous::

    t_d435i = t_livox + offset

Only NumPy is imported by the estimation core.  ROS 2 modules are loaded
lazily when a bag is read, so the math can be regression-tested without ROS.
This command writes only the explicit ``--output`` path; it never promotes an
analysis into the project's formal ``results/`` directory by itself.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


SCHEMA = "d435i_calib/lidar_camera_timesync/v1"
TIME_EQUATION = "t_d435i = t_livox + offset"
MATCH_EQUATION = (
    "omega_d435i(t_livox + offset) = "
    "R_d435gyro_livoximu * omega_livox(t_livox) + bias"
)
DEFAULT_LIVOX_TOPIC = "/livox/imu"
DEFAULT_D435I_TOPIC = "/camera/camera/gyro/sample"


class TimesyncError(RuntimeError):
    """The input cannot support a defensible time-offset estimate."""


@dataclass(frozen=True)
class OffsetFit:
    offset_s: float
    rotation: np.ndarray
    bias: np.ndarray
    rmse_rad_s: float
    median_rad_s: float
    p95_rad_s: float
    sample_count: int
    livox_excitation_rad_s: float
    d435i_excitation_rad_s: float
    singular_values: np.ndarray

    def json_result(self) -> dict[str, Any]:
        return {
            "offset_ms": float(round(self.offset_s * 1000.0, 9)),
            "R_d435gyro_livoximu": self.rotation.tolist(),
            "bias_d435gyro_minus_rotated_livoximu_rad_s": self.bias.tolist(),
            "residual_rmse_rad_s": float(self.rmse_rad_s),
            "residual_median_rad_s": float(self.median_rad_s),
            "residual_p95_rad_s": float(self.p95_rad_s),
            "sample_count": int(self.sample_count),
            "livox_excitation_rms_rad_s": float(self.livox_excitation_rad_s),
            "d435i_excitation_rms_rad_s": float(self.d435i_excitation_rad_s),
            "wahba_singular_values": self.singular_values.tolist(),
        }


def _finite_float(value: float, label: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise TimesyncError(f"{label} must be finite")
    return value


def _prepare_series(times: np.ndarray, gyro: np.ndarray, label: str,
                    minimum: int = 3) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray(times, dtype=np.float64)
    gyro = np.asarray(gyro, dtype=np.float64)
    if times.ndim != 1:
        raise TimesyncError(f"{label} times must be a one-dimensional array")
    if gyro.shape != (len(times), 3):
        raise TimesyncError(
            f"{label} gyro must have shape ({len(times)}, 3), got {gyro.shape}")
    finite = np.isfinite(times) & np.isfinite(gyro).all(axis=1)
    times, gyro = times[finite], gyro[finite]
    if len(times) < minimum:
        raise TimesyncError(
            f"{label} has only {len(times)} finite samples; need at least {minimum}")

    order = np.argsort(times, kind="stable")
    times, gyro = times[order], gyro[order]
    unique, inverse, counts = np.unique(times, return_inverse=True,
                                        return_counts=True)
    if len(unique) != len(times):
        collapsed = np.zeros((len(unique), 3), dtype=np.float64)
        np.add.at(collapsed, inverse, gyro)
        gyro = collapsed / counts[:, None]
        times = unique
    if len(times) < minimum or np.any(np.diff(times) <= 0):
        raise TimesyncError(f"{label} timestamps do not contain enough unique values")
    return times, gyro


def _proper_rotation_and_bias(source: np.ndarray, target: np.ndarray
                              ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit ``target = R @ source + bias`` with det(R)=+1."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise TimesyncError("Wahba inputs must be equal N x 3 arrays")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    x = source - source_mean
    y = target - target_mean
    covariance = x.T @ y / max(1, len(x))
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt = vt.copy()
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    bias = target_mean - rotation @ source_mean
    return rotation, bias, singular_values


def _fit_candidate(livox_times: np.ndarray, livox_gyro: np.ndarray,
                   d435i_times: np.ndarray, d435i_gyro: np.ndarray,
                   offset_s: float, min_samples: int) -> OffsetFit | None:
    query = livox_times + offset_s
    valid = (query >= d435i_times[0]) & (query <= d435i_times[-1])
    if int(np.count_nonzero(valid)) < min_samples:
        return None
    source = livox_gyro[valid]
    q = query[valid]
    target = np.column_stack([
        np.interp(q, d435i_times, d435i_gyro[:, axis])
        for axis in range(3)
    ])
    rotation, bias, singular_values = _proper_rotation_and_bias(source, target)
    predicted = source @ rotation.T + bias
    residual_norm = np.linalg.norm(target - predicted, axis=1)
    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)
    return OffsetFit(
        offset_s=float(offset_s),
        rotation=rotation,
        bias=bias,
        rmse_rad_s=float(np.sqrt(np.mean(residual_norm ** 2))),
        median_rad_s=float(np.median(residual_norm)),
        p95_rad_s=float(np.percentile(residual_norm, 95.0)),
        sample_count=len(source),
        livox_excitation_rad_s=float(
            np.sqrt(np.mean(np.sum(source_centered ** 2, axis=1)))),
        d435i_excitation_rad_s=float(
            np.sqrt(np.mean(np.sum(target_centered ** 2, axis=1)))),
        singular_values=singular_values,
    )


def _offset_grid(lo: float, hi: float, step: float) -> np.ndarray:
    if not (math.isfinite(lo) and math.isfinite(hi) and math.isfinite(step)):
        raise TimesyncError("offset search limits and step must be finite")
    if hi < lo:
        raise TimesyncError("maximum offset must be >= minimum offset")
    if step <= 0:
        raise TimesyncError("offset search step must be positive")
    count = int(math.floor((hi - lo) / step + 1.0e-9)) + 1
    grid = lo + np.arange(count, dtype=np.float64) * step
    if grid[-1] < hi - step * 1.0e-6:
        grid = np.append(grid, hi)
    return grid


def _best_on_grid(livox_times: np.ndarray, livox_gyro: np.ndarray,
                  d435i_times: np.ndarray, d435i_gyro: np.ndarray,
                  offsets: np.ndarray, min_samples: int) -> OffsetFit:
    best = None
    for candidate in offsets:
        fit = _fit_candidate(livox_times, livox_gyro, d435i_times, d435i_gyro,
                             float(candidate), min_samples)
        if fit is not None and (best is None or fit.rmse_rad_s < best.rmse_rad_s):
            best = fit
    if best is None:
        raise TimesyncError(
            "no candidate offset has enough overlapping gyro samples")
    return best


def estimate_time_offset(livox_times: np.ndarray, livox_gyro: np.ndarray,
                         d435i_times: np.ndarray, d435i_gyro: np.ndarray,
                         *, min_offset_s: float = -0.25,
                         max_offset_s: float = 0.25,
                         coarse_step_s: float = 0.001,
                         refine_factor: int = 20,
                         min_samples: int = 200,
                         min_excitation_rad_s: float = 0.05) -> OffsetFit:
    """Estimate the offset under ``t_d435i = t_livox + offset``.

    A coarse grid is followed by a local fine grid.  The input epochs are
    shifted near zero before interpolation to retain sub-millisecond floating
    point resolution even for Unix-epoch ROS timestamps.
    """
    if refine_factor < 1:
        raise TimesyncError("refine_factor must be >= 1")
    if min_samples < 6:
        raise TimesyncError("min_samples must be >= 6")
    min_excitation_rad_s = _finite_float(
        min_excitation_rad_s, "minimum excitation")
    if min_excitation_rad_s < 0:
        raise TimesyncError("minimum excitation must be non-negative")

    lt, lw = _prepare_series(livox_times, livox_gyro, "Livox", min_samples)
    ct, cw = _prepare_series(d435i_times, d435i_gyro, "D435i", min_samples)
    origin = min(float(lt[0]), float(ct[0]))
    lt, ct = lt - origin, ct - origin

    coarse = _offset_grid(min_offset_s, max_offset_s, coarse_step_s)
    first = _best_on_grid(lt, lw, ct, cw, coarse, min_samples)
    if refine_factor == 1:
        best = first
    else:
        fine_step = coarse_step_s / refine_factor
        fine_lo = max(min_offset_s, first.offset_s - coarse_step_s)
        fine_hi = min(max_offset_s, first.offset_s + coarse_step_s)
        fine = _offset_grid(fine_lo, fine_hi, fine_step)
        best = _best_on_grid(lt, lw, ct, cw, fine, min_samples)

    if min(best.livox_excitation_rad_s,
           best.d435i_excitation_rad_s) < min_excitation_rad_s:
        raise TimesyncError(
            "gyro motion excitation is too low for time-offset estimation: "
            f"Livox={best.livox_excitation_rad_s:.6g} rad/s, "
            f"D435i={best.d435i_excitation_rad_s:.6g} rad/s, "
            f"required={min_excitation_rad_s:.6g} rad/s")
    return best


def analyze_three_segments(livox_times: np.ndarray, livox_gyro: np.ndarray,
                           d435i_times: np.ndarray, d435i_gyro: np.ndarray,
                           **estimate_options: Any
                           ) -> tuple[OffsetFit, list[dict[str, Any]]]:
    """Fit the full recording, then independently refit three time segments."""
    lt, lw = _prepare_series(livox_times, livox_gyro, "Livox")
    ct, cw = _prepare_series(d435i_times, d435i_gyro, "D435i")
    full = estimate_time_offset(lt, lw, ct, cw, **estimate_options)

    start = max(float(lt[0]), float(ct[0] - full.offset_s))
    end = min(float(lt[-1]), float(ct[-1] - full.offset_s))
    if not end > start:
        raise TimesyncError("the fitted streams have no common time interval")
    edges = np.linspace(start, end, 4)
    segments = []
    for index in range(3):
        if index == 2:
            selected = (lt >= edges[index]) & (lt <= edges[index + 1])
        else:
            selected = (lt >= edges[index]) & (lt < edges[index + 1])
        fit = estimate_time_offset(
            lt[selected], lw[selected], ct, cw, **estimate_options)
        segments.append({
            "index": index + 1,
            "livox_start_s": float(edges[index]),
            "livox_end_s": float(edges[index + 1]),
            **fit.json_result(),
        })
    return full, segments


def _stamp_seconds(message: Any) -> float:
    try:
        stamp = message.header.stamp
        value = int(stamp.sec) + int(stamp.nanosec) * 1.0e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise TimesyncError("IMU message has no valid header timestamp") from exc
    if not math.isfinite(value) or value <= 0:
        raise TimesyncError("IMU header timestamp must be finite and positive")
    return value


def read_imu_topics(bag: Path, livox_topic: str = DEFAULT_LIVOX_TOPIC,
                    d435i_topic: str = DEFAULT_D435I_TOPIC
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                               dict[str, Any]]:
    """Read two ``sensor_msgs/msg/Imu`` streams using the ROS 2 bag API."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise TimesyncError(
            "ROS 2 Python modules are unavailable; source /opt/ros/jazzy/setup.bash"
        ) from exc

    bag = Path(bag).expanduser().resolve()
    if not bag.exists():
        raise TimesyncError(f"bag does not exist: {bag}")
    try:
        metadata = rosbag2_py.Info().read_metadata(str(bag), "")
        storage_id = metadata.storage_identifier
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"),
        )
    except Exception as exc:
        raise TimesyncError(f"cannot open ROS 2 bag {bag}: {exc}") from exc

    topic_types = {item.name: item.type
                   for item in reader.get_all_topics_and_types()}
    expected_type = "sensor_msgs/msg/Imu"
    for topic in (livox_topic, d435i_topic):
        actual = topic_types.get(topic)
        if actual != expected_type:
            raise TimesyncError(
                f"{topic} must be {expected_type}, got {actual or 'missing'}")
    try:
        reader.set_filter(rosbag2_py.StorageFilter(
            topics=[livox_topic, d435i_topic]))
    except Exception as exc:
        raise TimesyncError(f"cannot filter bag topics: {exc}") from exc

    message_class = get_message(expected_type)
    streams = {
        livox_topic: {"times": [], "gyro": [], "frames": set(),
                      "bag_minus_header_ms": []},
        d435i_topic: {"times": [], "gyro": [], "frames": set(),
                      "bag_minus_header_ms": []},
    }
    try:
        while reader.has_next():
            topic, payload, bag_ns = reader.read_next()
            message = deserialize_message(payload, message_class)
            stamp = _stamp_seconds(message)
            vector = message.angular_velocity
            gyro = (float(vector.x), float(vector.y), float(vector.z))
            if not all(math.isfinite(value) for value in gyro):
                continue
            stream = streams[topic]
            stream["times"].append(stamp)
            stream["gyro"].append(gyro)
            frame = str(getattr(message.header, "frame_id", "")).strip()
            if frame:
                stream["frames"].add(frame)
            stream["bag_minus_header_ms"].append(
                (int(bag_ns) * 1.0e-9 - stamp) * 1000.0)
    except Exception as exc:
        raise TimesyncError(f"failed while reading {bag}: {exc}") from exc
    finally:
        try:
            reader.close()
        except Exception:
            pass

    arrays = []
    stream_metadata = {}
    for topic in (livox_topic, d435i_topic):
        stream = streams[topic]
        times = np.asarray(stream["times"], dtype=np.float64)
        gyro = np.asarray(stream["gyro"], dtype=np.float64).reshape(-1, 3)
        times, gyro = _prepare_series(times, gyro, topic)
        arrays.extend((times, gyro))
        latency = np.asarray(stream["bag_minus_header_ms"], dtype=np.float64)
        stream_metadata[topic] = {
            "type": expected_type,
            "sample_count": int(len(times)),
            "frame_ids": sorted(stream["frames"]),
            "header_start_s": float(times[0]),
            "header_end_s": float(times[-1]),
            "median_rate_hz": float(1.0 / np.median(np.diff(times))),
            "bag_minus_header_median_ms": float(np.median(latency)),
            "bag_minus_header_abs_p95_ms": float(
                np.percentile(np.abs(latency), 95.0)),
        }
    return (*arrays, stream_metadata)


def bag_sha256(path: Path) -> tuple[str, list[str]]:
    """Hash a bag file or a bag directory, binding names and bytes."""
    path = Path(path).expanduser().resolve()
    if path.is_file():
        files = [path]
        root = path.parent
    elif path.is_dir():
        files = sorted(item for item in path.rglob("*") if item.is_file())
        root = path
    else:
        raise TimesyncError(f"bag path does not exist: {path}")
    if not files:
        raise TimesyncError(f"bag contains no files: {path}")
    digest = hashlib.sha256()
    names = []
    for item in files:
        relative = item.relative_to(root).as_posix()
        names.append(relative)
        encoded = relative.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(item.stat().st_size.to_bytes(8, "big"))
        with item.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest(), names


def _write_new_json(path: Path, document: dict[str, Any]) -> None:
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2,
                      sort_keys=True, allow_nan=False)
            stream.write("\n")
    except FileExistsError as exc:
        raise TimesyncError(f"refusing to overwrite existing output: {path}") from exc


def build_document(*, bag: Path, bag_digest: str, bag_files: list[str],
                   d435i_serial: str, mid360s_serial: str, rig_id: str,
                   livox_topic: str, d435i_topic: str,
                   stream_metadata: dict[str, Any], full: OffsetFit,
                   segments: list[dict[str, Any]], search: dict[str, Any]
                   ) -> dict[str, Any]:
    segment_offsets = [float(item["offset_ms"]) for item in segments]
    return {
        "schema": SCHEMA,
        "status": "analyzed",
        "created_utc": datetime.now(timezone.utc).isoformat(
            timespec="seconds").replace("+00:00", "Z"),
        "rig_id": rig_id,
        "devices": {
            "d435i": {"serial": d435i_serial, "gyro_topic": d435i_topic},
            "mid360s": {"serial": mid360s_serial, "imu_topic": livox_topic},
        },
        "time_convention": {
            "equation": TIME_EQUATION,
            "matching_equation": MATCH_EQUATION,
            "offset_unit": "ms",
            "interpretation": (
                "add offset to a Livox timestamp to express the matching "
                "event on the D435i timestamp axis"),
        },
        "source_bag": {
            "path": str(Path(bag).expanduser().resolve()),
            "bag_sha256": bag_digest,
            "hash_scheme": "sha256-tree-v1(length-prefixed relative path, size, bytes)",
            "files": bag_files,
            "streams": stream_metadata,
        },
        "search": search,
        "result": full.json_result(),
        "three_segment_refits": {
            "segments": segments,
            "offsets_ms": segment_offsets,
            "offset_range_ms": float(max(segment_offsets) - min(segment_offsets)),
            "offset_std_ms": float(np.std(segment_offsets)),
        },
    }


def _topic(value: str) -> str:
    if not value.startswith("/") or any(char.isspace() for char in value):
        raise argparse.ArgumentTypeError(
            "topic must be an absolute ROS name without whitespace")
    return value


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate MID-360S -> D435i effective timestamp offset from the "
            "two gyroscopes in a ROS 2 bag."))
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or file")
    parser.add_argument("--output", type=Path, required=True,
                        help="new JSON analysis path (existing files are refused)")
    parser.add_argument("--d435i-serial", required=True)
    parser.add_argument("--mid360s-serial", required=True)
    parser.add_argument("--rig-id", default="mid360s-d435i-01")
    parser.add_argument("--livox-topic", type=_topic,
                        default=DEFAULT_LIVOX_TOPIC)
    parser.add_argument("--d435i-topic", type=_topic,
                        default=DEFAULT_D435I_TOPIC)
    parser.add_argument("--min-offset-ms", type=float, default=-250.0)
    parser.add_argument("--max-offset-ms", type=float, default=250.0)
    parser.add_argument("--coarse-step-ms", type=float, default=1.0)
    parser.add_argument("--refine-factor", type=int, default=20)
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument("--min-excitation-rad-s", type=float, default=0.05)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    try:
        for label, value in (("D435i serial", args.d435i_serial),
                             ("MID-360S serial", args.mid360s_serial),
                             ("rig ID", args.rig_id)):
            if not str(value).strip() or "\n" in str(value) or "\r" in str(value):
                raise TimesyncError(f"{label} must be one non-empty line")
        search_options = {
            "min_offset_s": args.min_offset_ms / 1000.0,
            "max_offset_s": args.max_offset_ms / 1000.0,
            "coarse_step_s": args.coarse_step_ms / 1000.0,
            "refine_factor": args.refine_factor,
            "min_samples": args.min_samples,
            "min_excitation_rad_s": args.min_excitation_rad_s,
        }
        lt, lw, ct, cw, stream_metadata = read_imu_topics(
            args.bag, args.livox_topic, args.d435i_topic)
        full, segments = analyze_three_segments(
            lt, lw, ct, cw, **search_options)
        digest, bag_files = bag_sha256(args.bag)
        document = build_document(
            bag=args.bag, bag_digest=digest, bag_files=bag_files,
            d435i_serial=args.d435i_serial.strip(),
            mid360s_serial=args.mid360s_serial.strip(),
            rig_id=args.rig_id.strip(), livox_topic=args.livox_topic,
            d435i_topic=args.d435i_topic, stream_metadata=stream_metadata,
            full=full, segments=segments,
            search={
                "min_offset_ms": float(args.min_offset_ms),
                "max_offset_ms": float(args.max_offset_ms),
                "coarse_step_ms": float(args.coarse_step_ms),
                "fine_step_ms": float(
                    args.coarse_step_ms / args.refine_factor),
                "refine_factor": int(args.refine_factor),
                "min_samples": int(args.min_samples),
                "min_excitation_rad_s": float(args.min_excitation_rad_s),
                "segments": 3,
                "fit": "proper_rotation_plus_constant_bias_wahba_kabsch",
            },
        )
        _write_new_json(args.output, document)
    except (TimesyncError, OSError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(json.dumps({
        "status": "analyzed",
        "output": str(Path(args.output).expanduser().resolve()),
        "offset_ms": document["result"]["offset_ms"],
        "segment_offsets_ms": document["three_segment_refits"]["offsets_ms"],
        "residual_rmse_rad_s": document["result"]["residual_rmse_rad_s"],
    }, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
