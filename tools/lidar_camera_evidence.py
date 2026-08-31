#!/usr/bin/env python3
"""Build auditable, per-frame RGB/depth/LiDAR evidence from a ROS 2 bag.

This is deliberately not a calibrator and cannot create a draft or validated
extrinsic.  It preserves the measurements needed by a later, independently
specified evaluator without repeating the lossy ``concatenate(xyz)`` shortcut:

* original message and header timestamps;
* exact JPEG payloads;
* organized uint16 depth plus an explicit source-pixel index image;
* exact PointCloud2 bytes, shape, field layout and point ordinal;
* immutable capture role, device identity and source hashes.

The command fails closed.  In particular, the legacy RGB+LiDAR bags are not
silently accepted as RGB-D evidence when the two depth topics are absent.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

import cv2
import numpy as np
import yaml


SCHEMA = "d435i_calib/lidar_camera_rgbd_evidence/v1"
CAPTURE_SCHEMA = "d435i_calib/lidar_camera_rgbd_capture/v1"
ROLE_SCHEMA = "d435i_calib/lidar_camera_capture_role/v1"
ROLES = ("calibration", "holdout")

DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw/compressed"
DEFAULT_COLOR_INFO_TOPIC = "/camera/camera/color/camera_info"
DEFAULT_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEFAULT_DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info"
DEFAULT_POINTS_TOPIC = "/livox/lidar"

TYPE_COMPRESSED = "sensor_msgs/msg/CompressedImage"
TYPE_IMAGE = "sensor_msgs/msg/Image"
TYPE_INFO = "sensor_msgs/msg/CameraInfo"
TYPE_POINTS = "sensor_msgs/msg/PointCloud2"

COLOR_FRAME = "camera_color_optical_frame"
DEPTH_FRAME = "camera_depth_optical_frame"
LIDAR_FRAME = "livox_frame"

SHA_LINE = re.compile(r"^([0-9a-f]{64}) [ *](\./[^/]+)$")


class EvidenceError(ValueError):
    """The input cannot support an auditable RGB-D/LiDAR evidence set."""


@dataclass(frozen=True)
class TopicLayout:
    image: str = DEFAULT_IMAGE_TOPIC
    color_info: str = DEFAULT_COLOR_INFO_TOPIC
    depth: str = DEFAULT_DEPTH_TOPIC
    depth_info: str = DEFAULT_DEPTH_INFO_TOPIC
    points: str = DEFAULT_POINTS_TOPIC

    def types(self) -> dict[str, str]:
        return {
            self.image: TYPE_COMPRESSED,
            self.color_info: TYPE_INFO,
            self.depth: TYPE_IMAGE,
            self.depth_info: TYPE_INFO,
            self.points: TYPE_POINTS,
        }


@dataclass(frozen=True)
class TimedSample:
    topic: str
    index: int
    bag_ns: int
    header_ns: int
    frame_id: str
    message: Any


@dataclass(frozen=True)
class SyncTuple:
    rgb: TimedSample
    color_info: TimedSample
    depth: TimedSample
    depth_info: TimedSample
    lidar: TimedSample


def _strict_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise EvidenceError(f"duplicate JSON key is forbidden: {key}")
        out[key] = value
    return out


def load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                object_pairs_hook=_strict_object,
                parse_constant=lambda token: (_ for _ in ()).throw(
                    EvidenceError(f"non-finite JSON token is forbidden: {token}")),
            )
    except EvidenceError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f"cannot read strict JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise EvidenceError(f"JSON root must be an object: {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def tree_sha256(path: Path) -> tuple[str, int, int]:
    if not path.is_dir() or path.is_symlink():
        raise EvidenceError(f"source bag must be a real directory: {path}")
    digest = hashlib.sha256(b"d435i-rgbd-evidence-source-tree-v1\0")
    byte_count = 0
    files = 0
    for item in sorted(path.rglob("*")):
        if item.is_symlink():
            raise EvidenceError(f"source tree contains a symlink: {item}")
        if not item.is_file():
            continue
        rel = item.relative_to(path).as_posix().encode("utf-8")
        size = item.stat().st_size
        digest.update(len(rel).to_bytes(8, "big"))
        digest.update(rel)
        digest.update(size.to_bytes(16, "big"))
        digest.update(bytes.fromhex(sha256_file(item)))
        byte_count += size
        files += 1
    return digest.hexdigest(), byte_count, files


def verify_sha256sums(bag: Path) -> None:
    sums = bag / "SHA256SUMS"
    if not sums.is_file() or sums.is_symlink():
        raise EvidenceError(f"capture SHA256SUMS is missing or unsafe: {sums}")
    listed: dict[str, str] = {}
    for line in sums.read_text(encoding="utf-8").splitlines():
        match = SHA_LINE.fullmatch(line)
        if not match:
            raise EvidenceError(f"malformed SHA256SUMS line: {line!r}")
        name = match.group(2)[2:]
        if name in listed:
            raise EvidenceError(f"duplicate SHA256SUMS entry: {name}")
        listed[name] = match.group(1)
    actual = {
        item.name for item in bag.iterdir()
        if item.is_file() and item.name != "SHA256SUMS"
    }
    if set(listed) != actual:
        raise EvidenceError(
            f"SHA256SUMS file set mismatch: listed={sorted(listed)}, actual={sorted(actual)}")
    for name, expected in listed.items():
        if sha256_file(bag / name) != expected:
            raise EvidenceError(f"SHA-256 mismatch: {name}")


def metadata_topic_types(bag: Path) -> dict[str, str]:
    path = bag / "metadata.yaml"
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream)
        info = root["rosbag2_bagfile_information"]
        rows = info["topics_with_message_count"]
        return {
            str(row["topic_metadata"]["name"]):
                str(row["topic_metadata"]["type"])
            for row in rows
        }
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise EvidenceError(f"cannot parse rosbag metadata {path}: {exc}") from exc


def require_topic_types(actual: dict[str, str], layout: TopicLayout) -> None:
    expected = layout.types()
    absent = [name for name in expected if name not in actual]
    depth_absent = [name for name in (layout.depth, layout.depth_info) if name not in actual]
    if depth_absent:
        raise EvidenceError(
            "missing required depth topic(s); legacy RGB+LiDAR bags are not valid "
            "RGB-D evidence: " + ", ".join(depth_absent))
    if absent:
        raise EvidenceError("missing required topic(s): " + ", ".join(absent))
    wrong = [
        f"{name}={actual[name]} (expected {message_type})"
        for name, message_type in expected.items()
        if actual[name] != message_type
    ]
    if wrong:
        raise EvidenceError("wrong topic type(s): " + "; ".join(wrong))


def _iso_utc(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise EvidenceError(f"{label} must be an ISO-8601 UTC string ending in Z")
    try:
        return datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise EvidenceError(f"invalid {label}: {value!r}") from exc


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise EvidenceError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise EvidenceError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise EvidenceError(f"{label} must be a finite number")
    return result


def _serial(value: Any, label: str) -> str:
    result = str(value).strip()
    if not result or any(character in result for character in "\r\n"):
        raise EvidenceError(f"{label} must be one non-empty line")
    return result


def factory_metric_geometry(factory: dict[str, Any]) -> dict[str, Any]:
    """Freeze the metric depth scale and the IR1/depth-to-RGB convention.

    ``dump_factory.py`` calls ``ir1.get_extrinsics_to(color)`` and groups the
    returned nine-value ``rs2_extrinsics.rotation`` sequence three values at a
    time. Librealsense defines that sequence as column-major, so the groups in
    ``factory_params.json`` are matrix columns, not logical matrix rows. Both
    the unmodified source groups and the derived row-major matrix are retained
    below to make a later projection implementation unambiguous.
    """
    factory_serial = _serial(factory.get("serial", ""), "factory D435i serial")

    scale_mm = _finite_number(factory.get("depth_scale_mm"),
                              "factory depth_scale_mm")
    if scale_mm <= 0:
        raise EvidenceError("factory depth_scale_mm must be positive")

    transform = factory.get("T_ir1_to_rgb")
    if not isinstance(transform, dict):
        raise EvidenceError("factory T_ir1_to_rgb must be an object")
    translation_value = transform.get("t_mm")
    columns_value = transform.get("R")
    if not isinstance(translation_value, list) or len(translation_value) != 3:
        raise EvidenceError("factory T_ir1_to_rgb.t_mm must contain three values")
    if (not isinstance(columns_value, list) or len(columns_value) != 3 or
            any(not isinstance(column, list) or len(column) != 3
                for column in columns_value)):
        raise EvidenceError("factory T_ir1_to_rgb.R must be a 3x3 column-major array")
    translation_mm = [
        _finite_number(value, f"factory T_ir1_to_rgb.t_mm[{index}]")
        for index, value in enumerate(translation_value)
    ]
    rotation_columns = [
        [
            _finite_number(value, f"factory T_ir1_to_rgb.R[{column}][{row}]")
            for row, value in enumerate(values)
        ]
        for column, values in enumerate(columns_value)
    ]
    rotation_row_major = np.asarray(rotation_columns, dtype=np.float64).T
    if (not np.allclose(rotation_row_major.T @ rotation_row_major, np.eye(3),
                        rtol=0.0, atol=1e-4) or
            not math.isclose(float(np.linalg.det(rotation_row_major)), 1.0,
                             rel_tol=0.0, abs_tol=1e-4)):
        raise EvidenceError("factory T_ir1_to_rgb.R is not a proper rotation")

    return {
        "d435i_serial": factory_serial,
        "depth_scale": {
            "source_key": "depth_scale_mm",
            "stored_dtype": "uint16",
            "millimetres_per_unit": scale_mm,
            "metres_per_unit": scale_mm / 1000.0,
            "conversion": "z_m = depth_u16 * metres_per_unit",
        },
        "depth_to_color_extrinsic": {
            "source_key": "T_ir1_to_rgb",
            "source_api_call": "ir1.get_extrinsics_to(color)",
            "from_frame": DEPTH_FRAME,
            "from_factory_sensor": "infrared stream 1 (IR1/depth optical geometry)",
            "to_frame": COLOR_FRAME,
            "direction": "IR1/depth optical coordinates to RGB/color optical coordinates",
            "equation": "p_color_mm = R_row_major @ p_depth_mm + translation_mm",
            "translation_mm": translation_mm,
            "rotation_source_layout": (
                "librealsense rs2_extrinsics.rotation is column-major"
            ),
            "rotation_storage_in_factory_json": (
                "R contains three consecutive matrix columns"
            ),
            "rotation_column_major_chunks": rotation_columns,
            "rotation_row_major": rotation_row_major.tolist(),
        },
    }


def validate_capture_manifest(
    bag: Path,
    manifest: dict[str, Any],
    role_doc: dict[str, Any],
    expected_role: str,
    layout: TopicLayout,
    camchain: Path,
    factory_params: Path,
    expected_d435_serial: str,
    expected_lidar_serial: str,
) -> dict[str, Any]:
    expected_d435_serial = _serial(expected_d435_serial, "CLI D435i serial")
    expected_lidar_serial = _serial(expected_lidar_serial, "CLI MID-360S serial")
    if expected_role not in ROLES:
        raise EvidenceError(f"unsupported expected role: {expected_role!r}")
    if manifest.get("schema") != CAPTURE_SCHEMA:
        raise EvidenceError(
            f"unsupported capture schema {manifest.get('schema')!r}; an RGB-D capture is required")
    if role_doc.get("schema") != ROLE_SCHEMA:
        raise EvidenceError("capture_role.json has an unsupported schema")
    role = manifest.get("role")
    if role != expected_role or role_doc.get("role") != expected_role:
        raise EvidenceError(
            f"capture role mismatch: manifest={role!r}, registration={role_doc.get('role')!r}, "
            f"expected={expected_role!r}")
    scene = str(manifest.get("scene", ""))
    if not scene or role_doc.get("scene") != scene or bag.name != scene:
        raise EvidenceError("scene identity differs across bag, manifest and role registration")

    normalized_parts = {part.lower() for part in bag.resolve().parts}
    if expected_role not in normalized_parts:
        raise EvidenceError(
            f"{expected_role} capture is not physically isolated under a '{expected_role}' directory")

    for doc, label in ((manifest, "manifest"), (role_doc, "role registration")):
        d435 = doc.get("d435i_serial")
        lidar = doc.get("mid360s_serial")
        if d435 != expected_d435_serial:
            raise EvidenceError(
                f"{label} D435i serial {d435!r} differs from CLI "
                f"{expected_d435_serial!r}")
        if lidar != expected_lidar_serial:
            raise EvidenceError(
                f"{label} MID-360S serial {lidar!r} differs from CLI "
                f"{expected_lidar_serial!r}")
    identity_keys = ("rig_id", "mount_session_id", "d435i_serial", "mid360s_serial")
    for key in identity_keys:
        if manifest.get(key) != role_doc.get(key) or not str(manifest.get(key, "")).strip():
            raise EvidenceError(f"capture role registration disagrees on {key}")

    if manifest.get("topics") != layout.types():
        raise EvidenceError("capture manifest topic/type map differs from the requested five-topic layout")
    frames = manifest.get("frames")
    if frames != {"color": COLOR_FRAME, "depth": DEPTH_FRAME, "lidar": LIDAR_FRAME}:
        raise EvidenceError(f"capture frame declaration is wrong: {frames!r}")
    profiles = manifest.get("profiles")
    expected_profiles = {
        "color": {"resolution": [1280, 720], "fps": 30, "transport": "jpeg"},
        "depth": {"resolution": [848, 480], "fps": 30, "encoding": "16UC1"},
    }
    if profiles != expected_profiles:
        raise EvidenceError(f"capture profiles differ from the frozen setup: {profiles!r}")
    if (manifest.get("rigid_mount_confirmed") is not True or
            manifest.get("static_during_capture_confirmed") is not True):
        raise EvidenceError("rigid/static capture confirmation is missing")
    witness = manifest.get("publisher_witness")
    raw_topic = layout.image.removesuffix("/compressed")
    witness_types = dict(layout.types())
    witness_types[raw_topic] = TYPE_IMAGE
    if not isinstance(witness, dict) or set(witness) != set(witness_types):
        raise EvidenceError("publisher witness does not cover raw RGB plus all five recorded topics")
    for topic, message_type in witness_types.items():
        item = witness.get(topic)
        if (not isinstance(item, dict) or item.get("publisher_count") != 1 or
                item.get("topic_type") != message_type or
                not str(item.get("node_name", "")).strip() or
                not str(item.get("gid", "")).strip()):
            raise EvidenceError(f"invalid unique-publisher witness for {topic}")

    registered = _iso_utc(role_doc.get("registered_utc"), "registered_utc")
    recorded = _iso_utc(manifest.get("recorded_utc"), "recorded_utc")
    if registered > recorded:
        raise EvidenceError("capture role was registered after recording started")
    if manifest.get("role_registration_sha256") != sha256_file(bag / "capture_role.json"):
        raise EvidenceError("capture_role.json hash differs from the capture manifest")

    if manifest.get("camchain_sha256") != sha256_file(camchain):
        raise EvidenceError("capture camchain hash differs from the supplied RGB calibration")
    if manifest.get("factory_params_sha256") != sha256_file(factory_params):
        raise EvidenceError("capture factory-parameter hash differs from the supplied device record")
    factory = load_json(factory_params)
    geometry = factory_metric_geometry(factory)
    if geometry["d435i_serial"] != expected_d435_serial:
        raise EvidenceError(
            "factory D435i serial differs from manifest/role/CLI identity")
    return geometry


def stamp_ns(message: Any) -> int:
    try:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as exc:
        raise EvidenceError("message lacks a valid ROS header timestamp") from exc


def read_rosbag(bag: Path, layout: TopicLayout) -> dict[str, list[TimedSample]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise EvidenceError(
            "ROS 2 Python modules are unavailable; source /opt/ros/jazzy/setup.bash") from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    advertised = {item.name: item.type for item in reader.get_all_topics_and_types()}
    require_topic_types(advertised, layout)
    wanted = layout.types()
    classes = {name: get_message(message_type) for name, message_type in wanted.items()}
    result: dict[str, list[TimedSample]] = {name: [] for name in wanted}
    counters = {name: 0 for name in wanted}
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(raw, classes[topic])
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        result[topic].append(TimedSample(
            topic=topic,
            index=counters[topic],
            bag_ns=int(bag_ns),
            header_ns=stamp_ns(message),
            frame_id=frame_id,
            message=message,
        ))
        counters[topic] += 1
    return result


def filter_stream(
    samples: Sequence[TimedSample],
    expected_frame: str,
    max_receive_lag_ns: int,
) -> tuple[list[TimedSample], list[dict[str, Any]]]:
    if not samples:
        raise EvidenceError("a required stream contains no messages")
    kept: list[TimedSample] = []
    stale: list[dict[str, Any]] = []
    previous = -1
    for sample in sorted(samples, key=lambda item: item.bag_ns):
        lag = sample.bag_ns - sample.header_ns
        if sample.header_ns <= 0 or abs(lag) > max_receive_lag_ns:
            stale.append({
                "topic": sample.topic,
                "index": sample.index,
                "bag_ns": sample.bag_ns,
                "header_ns": sample.header_ns,
                "receive_minus_header_ns": lag,
            })
            continue
        if sample.frame_id != expected_frame:
            raise EvidenceError(
                f"wrong frame on {sample.topic}[{sample.index}]: "
                f"{sample.frame_id!r}, expected {expected_frame!r}")
        if sample.header_ns <= previous:
            raise EvidenceError(
                f"non-monotonic header timestamps on {sample.topic}: "
                f"{sample.header_ns} after {previous}")
        previous = sample.header_ns
        kept.append(sample)
    if not kept:
        raise EvidenceError("all messages in a required stream were stale")
    return kept, stale


def _nearest(samples: Sequence[TimedSample], target_ns: int) -> TimedSample:
    stamps = [sample.header_ns for sample in samples]
    pos = bisect.bisect_left(stamps, target_ns)
    candidates = []
    if pos < len(samples):
        candidates.append(samples[pos])
    if pos:
        candidates.append(samples[pos - 1])
    if not candidates:
        raise EvidenceError("cannot find a nearest message in an empty stream")
    return min(candidates, key=lambda sample: (abs(sample.header_ns - target_ns), sample.header_ns))


def synchronize(
    streams: dict[str, Sequence[TimedSample]],
    layout: TopicLayout,
    max_camera_delta_ns: int,
    max_lidar_delta_ns: int,
    max_info_delta_ns: int,
    min_tuples: int,
) -> list[SyncTuple]:
    tuples: list[SyncTuple] = []
    used: set[tuple[int, int, int]] = set()
    for lidar in streams[layout.points]:
        rgb = _nearest(streams[layout.image], lidar.header_ns)
        depth = _nearest(streams[layout.depth], rgb.header_ns)
        color_info = _nearest(streams[layout.color_info], rgb.header_ns)
        depth_info = _nearest(streams[layout.depth_info], depth.header_ns)
        if abs(lidar.header_ns - rgb.header_ns) > max_lidar_delta_ns:
            continue
        if abs(depth.header_ns - rgb.header_ns) > max_camera_delta_ns:
            continue
        if abs(color_info.header_ns - rgb.header_ns) > max_info_delta_ns:
            continue
        if abs(depth_info.header_ns - depth.header_ns) > max_info_delta_ns:
            continue
        identity = (rgb.index, depth.index, lidar.index)
        if identity in used:
            continue
        used.add(identity)
        tuples.append(SyncTuple(rgb, color_info, depth, depth_info, lidar))
    if len(tuples) < min_tuples:
        raise EvidenceError(
            f"only {len(tuples)} synchronized tuples satisfy frozen timing gates; "
            f"at least {min_tuples} are required")
    return tuples


def decode_jpeg(message: Any) -> tuple[bytes, tuple[int, int]]:
    fmt = str(getattr(message, "format", "")).lower()
    if "jpeg" not in fmt and "jpg" not in fmt:
        raise EvidenceError(f"compressed RGB format is not JPEG: {fmt!r}")
    payload = bytes(message.data)
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise EvidenceError("compressed RGB payload cannot be decoded as a color JPEG")
    return payload, (int(image.shape[0]), int(image.shape[1]))


def organized_depth(message: Any) -> tuple[np.ndarray, np.ndarray]:
    if str(getattr(message, "encoding", "")) != "16UC1":
        raise EvidenceError(
            f"depth encoding must be exactly 16UC1, got {getattr(message, 'encoding', None)!r}")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step < width * 2:
        raise EvidenceError(f"invalid depth shape/step: {width}x{height}, step={step}")
    raw = memoryview(message.data)
    if len(raw) < height * step:
        raise EvidenceError("depth payload is shorter than height*step")
    endian = ">u2" if bool(message.is_bigendian) else "<u2"
    rows = [
        np.frombuffer(raw[row * step:row * step + width * 2], dtype=endian, count=width)
        for row in range(height)
    ]
    depth = np.stack(rows).astype(np.uint16, copy=False)
    # Explicit lineage: source_linear_index[row, col] == row * width + col.
    lineage = np.arange(height * width, dtype=np.uint32).reshape(height, width)
    return depth, lineage


def camera_info(message: Any) -> dict[str, Any]:
    width, height = int(message.width), int(message.height)
    k = [float(value) for value in message.k]
    d = [float(value) for value in message.d]
    r = [float(value) for value in message.r]
    p = [float(value) for value in message.p]
    if width <= 0 or height <= 0 or len(k) != 9 or len(r) != 9 or len(p) != 12:
        raise EvidenceError("CameraInfo has invalid dimensions or matrix lengths")
    if not all(math.isfinite(value) for value in k + d + r + p):
        raise EvidenceError("CameraInfo contains a non-finite value")
    return {
        "width": width,
        "height": height,
        "distortion_model": str(message.distortion_model),
        "d": d,
        "k": k,
        "r": r,
        "p": p,
    }


POINT_DTYPES = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2",
    5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


def pointcloud_payload(message: Any) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    height, width = int(message.height), int(message.width)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if height <= 0 or width <= 0 or point_step <= 0 or row_step < width * point_step:
        raise EvidenceError("PointCloud2 has invalid shape or strides")
    fields = []
    by_name = {}
    for field in message.fields:
        item = {
            "name": str(field.name), "offset": int(field.offset),
            "datatype": int(field.datatype), "count": int(field.count),
        }
        if item["name"] in by_name:
            raise EvidenceError(f"PointCloud2 has duplicate field {item['name']!r}")
        if item["datatype"] not in POINT_DTYPES or item["count"] <= 0:
            raise EvidenceError(f"unsupported PointCloud2 field: {item}")
        fields.append(item)
        by_name[item["name"]] = item
    for name in ("x", "y", "z", "intensity"):
        if name not in by_name:
            raise EvidenceError(f"PointCloud2 is missing required field {name!r}")
    for name in ("x", "y", "z"):
        if by_name[name]["datatype"] != 7 or by_name[name]["count"] != 1:
            raise EvidenceError(f"PointCloud2 {name} must be scalar FLOAT32")

    raw = np.frombuffer(bytes(message.data), dtype=np.uint8).copy()
    if len(raw) < height * row_step:
        raise EvidenceError("PointCloud2 payload is shorter than height*row_step")
    endian = ">" if bool(message.is_bigendian) else "<"
    xyz_rows = []
    for row in range(height):
        start = row * row_step
        view = memoryview(message.data)[start:start + width * point_step]
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [by_name[name]["offset"] for name in ("x", "y", "z")],
            "itemsize": point_step,
        })
        record = np.frombuffer(view, dtype=dtype, count=width)
        xyz_rows.append(np.column_stack((record["x"], record["y"], record["z"])))
    xyz = np.concatenate(xyz_rows).astype(np.float32, copy=False)
    arrays = {
        "raw_data": raw,
        "xyz": xyz,
        "point_index": np.arange(height * width, dtype=np.uint32),
        "valid_xyz": np.isfinite(xyz).all(axis=1),
    }
    meta = {
        "height": height, "width": width,
        "point_step": point_step, "row_step": row_step,
        "is_bigendian": bool(message.is_bigendian),
        "is_dense": bool(message.is_dense), "fields": fields,
    }
    return arrays, meta


def _write_npz(path: Path, **arrays: np.ndarray) -> None:
    np.savez_compressed(path, **arrays)


def write_evidence(
    output: Path,
    tuples: Sequence[SyncTuple],
    role: str,
    provenance: dict[str, Any],
    stale: Sequence[dict[str, Any]],
    timing: dict[str, float | int],
) -> dict[str, Any]:
    device_identity = provenance.get("device_identity")
    if not isinstance(device_identity, dict):
        raise EvidenceError("provenance is missing device identity")
    d435_serial = _serial(device_identity.get("d435i_serial"),
                          "provenance D435i serial")
    lidar_serial = _serial(device_identity.get("mid360s_serial"),
                           "provenance MID-360S serial")
    metric_geometry = provenance.get("factory_metric_geometry")
    if not isinstance(metric_geometry, dict):
        raise EvidenceError("provenance is missing factory metric depth geometry")
    depth_scale = metric_geometry.get("depth_scale")
    depth_to_color = metric_geometry.get("depth_to_color_extrinsic")
    if (not isinstance(depth_scale, dict) or
            not isinstance(depth_to_color, dict)):
        raise EvidenceError("factory metric depth geometry is incomplete")
    if metric_geometry.get("d435i_serial") != d435_serial:
        raise EvidenceError("factory metric geometry and provenance D435i serial differ")
    scale_mm = _finite_number(depth_scale.get("millimetres_per_unit"),
                              "provenance depth scale")
    scale_m = _finite_number(depth_scale.get("metres_per_unit"),
                             "provenance depth scale")
    if (scale_mm <= 0 or scale_m <= 0 or
            not math.isclose(scale_m, scale_mm / 1000.0,
                             rel_tol=1e-12, abs_tol=0.0)):
        raise EvidenceError("provenance depth scale is inconsistent")
    if (depth_to_color.get("source_key") != "T_ir1_to_rgb" or
            depth_to_color.get("from_frame") != DEPTH_FRAME or
            depth_to_color.get("to_frame") != COLOR_FRAME):
        raise EvidenceError("provenance depth-to-color direction is inconsistent")
    if output.exists() or output.is_symlink():
        raise EvidenceError(f"refusing to overwrite evidence output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.staging.", dir=output.parent))
    try:
        (staging / "rgb").mkdir()
        (staging / "depth").mkdir()
        (staging / "lidar").mkdir()
        frame_rows = []
        color_shape = None
        depth_shape = None
        for ordinal, item in enumerate(tuples):
            stem = f"{ordinal:06d}"
            jpeg, rgb_shape = decode_jpeg(item.rgb.message)
            depth, lineage = organized_depth(item.depth.message)
            color_model = camera_info(item.color_info.message)
            depth_model = camera_info(item.depth_info.message)
            cloud_arrays, cloud_meta = pointcloud_payload(item.lidar.message)
            if rgb_shape != (color_model["height"], color_model["width"]):
                raise EvidenceError("JPEG shape differs from synchronized color CameraInfo")
            if depth.shape != (depth_model["height"], depth_model["width"]):
                raise EvidenceError("organized depth shape differs from depth CameraInfo")
            if rgb_shape != (720, 1280):
                raise EvidenceError(
                    f"RGB shape is {rgb_shape}, expected the calibrated 720x1280 profile")
            if depth.shape != (480, 848):
                raise EvidenceError(
                    f"depth shape is {depth.shape}, expected the frozen 480x848 profile")
            if color_shape is None:
                color_shape, depth_shape = rgb_shape, depth.shape
            if rgb_shape != color_shape or depth.shape != depth_shape:
                raise EvidenceError("image/depth shape changed within one capture")

            rgb_rel = Path("rgb") / f"{stem}.jpg"
            depth_rel = Path("depth") / f"{stem}.npz"
            lidar_rel = Path("lidar") / f"{stem}.npz"
            (staging / rgb_rel).write_bytes(jpeg)
            _write_npz(staging / depth_rel, depth_u16=depth,
                       source_linear_index=lineage,
                       valid_mask=(depth > 0))
            _write_npz(staging / lidar_rel, **cloud_arrays)
            frame_rows.append({
                "ordinal": ordinal,
                "rgb": {
                    "source_message_index": item.rgb.index,
                    "header_ns": item.rgb.header_ns,
                    "bag_ns": item.rgb.bag_ns,
                    "frame_id": item.rgb.frame_id,
                    "shape_hwc": [rgb_shape[0], rgb_shape[1], 3],
                    "file": rgb_rel.as_posix(),
                    "camera_info": color_model,
                },
                "depth": {
                    "source_message_index": item.depth.index,
                    "header_ns": item.depth.header_ns,
                    "bag_ns": item.depth.bag_ns,
                    "frame_id": item.depth.frame_id,
                    "shape_hw": [int(depth.shape[0]), int(depth.shape[1])],
                    "encoding": "16UC1",
                    "stored_dtype": "uint16",
                    "millimetres_per_unit": scale_mm,
                    "metres_per_unit": scale_m,
                    "metric_conversion": "z_m = depth_u16 * metres_per_unit",
                    "file": depth_rel.as_posix(),
                    "pixel_lineage": "source_linear_index[row,col] = row*width+col",
                    "camera_info": depth_model,
                },
                "lidar": {
                    "source_message_index": item.lidar.index,
                    "header_ns": item.lidar.header_ns,
                    "bag_ns": item.lidar.bag_ns,
                    "frame_id": item.lidar.frame_id,
                    "file": lidar_rel.as_posix(),
                    "point_lineage": "point_index is the source PointCloud2 row-major ordinal",
                    **cloud_meta,
                },
                "delta_ns": {
                    "depth_minus_rgb": item.depth.header_ns - item.rgb.header_ns,
                    "lidar_minus_rgb": item.lidar.header_ns - item.rgb.header_ns,
                    "color_info_minus_rgb": item.color_info.header_ns - item.rgb.header_ns,
                    "depth_info_minus_depth": item.depth_info.header_ns - item.depth.header_ns,
                },
            })

        document = {
            "schema": SCHEMA,
            "status": "evidence_only_not_a_calibration_result",
            "created_utc": datetime.now(timezone.utc).isoformat(
                timespec="seconds").replace("+00:00", "Z"),
            "role": role,
            "holdout_policy": (
                "never enters calibration, initialization, seed selection or threshold tuning"
                if role == "holdout" else
                "calibration input; never relabel as holdout"
            ),
            "devices": {
                "d435i_serial": d435_serial,
                "mid360s_serial": lidar_serial,
            },
            "timing_policy": timing,
            "stale_messages_excluded": list(stale),
            "provenance": provenance,
            "tuple_count": len(frame_rows),
            "frames": frame_rows,
        }
        manifest_path = staging / "manifest.json"
        manifest_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2,
                       sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8")
        sums = []
        for path in sorted(item for item in staging.rglob("*") if item.is_file()):
            rel = path.relative_to(staging).as_posix()
            sums.append(f"{sha256_file(path)}  ./{rel}")
        (staging / "SHA256SUMS").write_text("\n".join(sums) + "\n", encoding="utf-8")
        os.replace(staging, output)
        return document
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def build(args: argparse.Namespace) -> dict[str, Any]:
    bag = args.bag.resolve()
    d435_serial = _serial(args.d435i_serial, "CLI D435i serial")
    lidar_serial = _serial(args.mid360s_serial, "CLI MID-360S serial")
    layout = TopicLayout(args.image_topic, args.color_info_topic,
                         args.depth_topic, args.depth_info_topic,
                         args.points_topic)
    # This is deliberately first: legacy bags get one precise, stable failure
    # rather than a misleading capture-schema or ROS import error.
    require_topic_types(metadata_topic_types(bag), layout)
    verify_sha256sums(bag)

    manifest_path = bag / "capture_manifest.json"
    role_path = bag / "capture_role.json"
    manifest = load_json(manifest_path)
    role_doc = load_json(role_path)
    factory_geometry = validate_capture_manifest(
        bag, manifest, role_doc, args.expect_role, layout,
        args.camchain.resolve(), args.factory_params.resolve(),
        d435_serial, lidar_serial)

    raw = read_rosbag(bag, layout)
    expected_frames = {
        layout.image: COLOR_FRAME,
        layout.color_info: COLOR_FRAME,
        layout.depth: DEPTH_FRAME,
        layout.depth_info: DEPTH_FRAME,
        layout.points: LIDAR_FRAME,
    }
    streams: dict[str, list[TimedSample]] = {}
    stale = []
    max_lag_ns = int(round(args.max_receive_lag_ms * 1e6))
    for topic, samples in raw.items():
        streams[topic], removed = filter_stream(
            samples, expected_frames[topic], max_lag_ns)
        stale.extend(removed)
    timing: dict[str, float | int] = {
        "max_receive_header_lag_ms": args.max_receive_lag_ms,
        "max_rgb_depth_header_delta_ms": args.max_camera_delta_ms,
        "max_lidar_rgb_header_delta_ms": args.max_lidar_delta_ms,
        "max_camera_info_header_delta_ms": args.max_info_delta_ms,
        "minimum_tuple_count": args.min_tuples,
        "pairing": "nearest header stamp; no interpolation and no invented timestamps",
    }
    tuples = synchronize(
        streams, layout,
        int(round(args.max_camera_delta_ms * 1e6)),
        int(round(args.max_lidar_delta_ms * 1e6)),
        int(round(args.max_info_delta_ms * 1e6)),
        args.min_tuples,
    )
    tree_hash, byte_count, file_count = tree_sha256(bag)
    provenance = {
        "source_bag": str(bag),
        "source_tree_sha256": tree_hash,
        "source_byte_count": byte_count,
        "source_file_count": file_count,
        "capture_manifest_sha256": sha256_file(manifest_path),
        "capture_role_sha256": sha256_file(role_path),
        "camchain": str(args.camchain.resolve()),
        "camchain_sha256": sha256_file(args.camchain.resolve()),
        "factory_params": str(args.factory_params.resolve()),
        "factory_params_sha256": sha256_file(args.factory_params.resolve()),
        "factory_metric_geometry": factory_geometry,
        "device_identity": {
            "d435i_serial": d435_serial,
            "mid360s_serial": lidar_serial,
        },
        "rig_id": manifest["rig_id"],
        "mount_session_id": manifest["mount_session_id"],
    }
    return write_evidence(
        args.output.resolve(), tuples, args.expect_role, provenance, stale, timing)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def parser() -> argparse.ArgumentParser:
    project = Path(__file__).resolve().parents[1]
    out = argparse.ArgumentParser(description=__doc__)
    sub = out.add_subparsers(dest="command", required=True)
    cmd = sub.add_parser("build", help="validate one five-topic bag and freeze per-frame evidence")
    cmd.add_argument("bag", type=Path)
    cmd.add_argument("--output", required=True, type=Path)
    cmd.add_argument("--expect-role", required=True, choices=ROLES)
    cmd.add_argument("--d435i-serial", required=True,
                     help="online D435i serial, also required in factory/role/manifest")
    cmd.add_argument("--mid360s-serial", required=True,
                     help="observed/overridden Livox serial frozen by the capture session")
    cmd.add_argument("--camchain", type=Path, default=project / "data/cam_rgb-camchain.yaml")
    cmd.add_argument("--factory-params", type=Path, default=project / "results/factory_params.json")
    cmd.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    cmd.add_argument("--color-info-topic", default=DEFAULT_COLOR_INFO_TOPIC)
    cmd.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    cmd.add_argument("--depth-info-topic", default=DEFAULT_DEPTH_INFO_TOPIC)
    cmd.add_argument("--points-topic", default=DEFAULT_POINTS_TOPIC)
    cmd.add_argument("--max-receive-lag-ms", type=positive_float, default=250.0)
    cmd.add_argument("--max-camera-delta-ms", type=positive_float, default=5.0)
    cmd.add_argument("--max-lidar-delta-ms", type=positive_float, default=20.0)
    cmd.add_argument("--max-info-delta-ms", type=positive_float, default=5.0)
    cmd.add_argument("--min-tuples", type=positive_int, default=10)
    return out


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "build":
            document = build(args)
        else:  # pragma: no cover - argparse constrains this branch.
            raise EvidenceError(f"unsupported command: {args.command}")
    except EvidenceError as exc:
        print(f"lidar-camera evidence error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "output": str(args.output.resolve()),
        "role": document["role"],
        "tuple_count": document["tuple_count"],
        "source_tree_sha256": document["provenance"]["source_tree_sha256"],
    }, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
