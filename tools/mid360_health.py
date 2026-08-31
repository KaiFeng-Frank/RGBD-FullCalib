#!/usr/bin/env python3
"""MID-360S raw health preflight and reproducible ROS 2 bag capture.

This tool deliberately stops at acquisition/QC.  It does not estimate range
bias, manufacture device identity or temperature, or write the final
``results/mid360s_health.json`` calibration artifact.

The Livox CustomMsg exposes received point-array entries, not the number of
rays the device attempted to emit.  Consequently all point counters below
refer to *received message slots*; none is a per-ray valid ratio.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from types import SimpleNamespace
from typing import Any, Iterable, Mapping, Sequence


LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
DEVICE_INFO_TOPIC = "/livox/device_info"
DEVICE_STATUS_TOPIC = "/livox/device_status"
RECORD_TOPICS = (
    LIDAR_TOPIC,
    IMU_TOPIC,
    DEVICE_INFO_TOPIC,
    DEVICE_STATUS_TOPIC,
)
EXPECTED_TYPES = {
    LIDAR_TOPIC: "livox_ros_driver2/msg/CustomMsg",
    IMU_TOPIC: "sensor_msgs/msg/Imu",
    DEVICE_INFO_TOPIC: "std_msgs/msg/String",
    DEVICE_STATUS_TOPIC: "std_msgs/msg/String",
}
FORMAL_ROLES = frozenset(("calibration", "validation"))
PRE_ROLL_S = 1.0
WINDOW_DURATION_S = 1.0
REQUIRED_WINDOW_COUNT = 10
ANALYSIS_DURATION_S = WINDOW_DURATION_S * REQUIRED_WINDOW_COUNT
DEFAULT_CAPTURE_DURATION_S = 12.0
MINIMUM_RECORDED_WINDOW_SPAN_S = PRE_ROLL_S + ANALYSIS_DURATION_S
MANIFEST_NAME = "manifest.json"
CHECKSUM_NAME = "SHA256SUMS"
NOT_FINAL_NOTICE = (
    "Raw acquisition/QC only; this is not a final MID-360S health result and "
    "must not be copied to results/mid360s_health.json."
)


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _percentile(values: Sequence[float], percent: float) -> float | None:
    """Linear percentile matching the common (n-1)*q convention."""
    finite = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not finite:
        return None
    position = (len(finite) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    alpha = position - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def _value_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    if not finite:
        return {
            "count": 0,
            "min": None,
            "p05": None,
            "p50": None,
            "p95": None,
            "max": None,
            "mean": None,
        }
    return {
        "count": len(finite),
        "min": min(finite),
        "p05": _percentile(finite, 5.0),
        "p50": _percentile(finite, 50.0),
        "p95": _percentile(finite, 95.0),
        "max": max(finite),
        "mean": sum(finite) / len(finite),
    }


def _histogram_percentiles(
    counts: Sequence[int], percents: Iterable[int]
) -> dict[str, float | None]:
    """Exact linearly interpolated percentiles for an integer histogram."""
    total = sum(int(c) for c in counts)
    if total <= 0:
        return {f"p{p:02d}": None for p in percents}

    def order_statistic(index: int) -> int:
        cumulative = 0
        for value, count in enumerate(counts):
            cumulative += int(count)
            if index < cumulative:
                return value
        return len(counts) - 1

    result: dict[str, float | None] = {}
    for percent in percents:
        position = (total - 1) * percent / 100.0
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        low_value = order_statistic(lower)
        high_value = order_statistic(upper)
        result[f"p{percent:02d}"] = low_value + (
            high_value - low_value
        ) * (position - lower)
    return result


def _message_stamp_s(msg: Any) -> float | None:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    sec = _finite_float(getattr(stamp, "sec", None))
    nanosec = _finite_float(getattr(stamp, "nanosec", None))
    if sec is None or nanosec is None:
        return None
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else None


@dataclass
class RateCounter:
    count: int = 0
    first_arrival_s: float | None = None
    last_arrival_s: float | None = None
    first_header_s: float | None = None
    last_header_s: float | None = None
    previous_header_s: float | None = None
    header_samples: int = 0
    nonmonotonic_header_steps: int = 0

    def add(self, arrival_s: float, header_s: float | None) -> None:
        self.count += 1
        if self.first_arrival_s is None:
            self.first_arrival_s = arrival_s
        self.last_arrival_s = arrival_s
        if header_s is None:
            return
        if self.first_header_s is None:
            self.first_header_s = header_s
        if self.previous_header_s is not None and header_s <= self.previous_header_s:
            self.nonmonotonic_header_steps += 1
        self.previous_header_s = header_s
        self.last_header_s = header_s
        self.header_samples += 1

    @staticmethod
    def _rate(count: int, first: float | None, last: float | None) -> float | None:
        if count < 2 or first is None or last is None or last <= first:
            return None
        return (count - 1) / (last - first)

    def as_dict(self) -> dict[str, Any]:
        arrival_span = None
        if self.first_arrival_s is not None and self.last_arrival_s is not None:
            arrival_span = max(0.0, self.last_arrival_s - self.first_arrival_s)
        header_span = None
        if self.first_header_s is not None and self.last_header_s is not None:
            header_span = self.last_header_s - self.first_header_s
        return {
            "messages": self.count,
            "arrival_rate_hz": self._rate(
                self.count, self.first_arrival_s, self.last_arrival_s
            ),
            "arrival_span_s": arrival_span,
            "header_rate_hz": self._rate(
                self.header_samples, self.first_header_s, self.last_header_s
            ),
            "header_span_s": header_span,
            "header_samples": self.header_samples,
            "nonmonotonic_header_steps": self.nonmonotonic_header_steps,
        }


@dataclass
class LidarAccumulator:
    rate: RateCounter = field(default_factory=RateCounter)
    raw_slots: int = 0
    reported_point_num: int = 0
    analyzed_slots: int = 0
    point_count_mismatch_frames: int = 0
    geometry_valid_points: int = 0
    strict_tag_slots: int = 0
    strict_high_confidence_points: int = 0
    tag_observed_slots: int = 0
    tag_counts: list[int] = field(default_factory=lambda: [0] * 256)
    reflectivity_counts: list[int] = field(default_factory=lambda: [0] * 256)
    reflectivity_observed_points: int = 0
    offset_observed_slots: int = 0
    offset_min_ns: int | None = None
    offset_max_ns: int | None = None
    offset_frame_spans_ns: list[float] = field(default_factory=list)
    processing_errors: list[str] = field(default_factory=list)

    def add(self, msg: Any, arrival_s: float | None = None) -> None:
        arrival_s = time.monotonic() if arrival_s is None else float(arrival_s)
        self.rate.add(arrival_s, _message_stamp_s(msg))
        points = getattr(msg, "points", ())
        raw_count = len(points)
        try:
            reported = max(0, int(getattr(msg, "point_num", 0)))
        except (TypeError, ValueError, OverflowError):
            reported = 0
        self.raw_slots += raw_count
        self.reported_point_num += reported
        if reported != raw_count:
            self.point_count_mismatch_frames += 1
        analyze_count = min(reported, raw_count) if reported > 0 else raw_count
        self.analyzed_slots += analyze_count

        frame_offset_min: int | None = None
        frame_offset_max: int | None = None
        try:
            for index in range(analyze_count):
                point = points[index]
                x = _finite_float(getattr(point, "x", None))
                y = _finite_float(getattr(point, "y", None))
                z = _finite_float(getattr(point, "z", None))
                geometry_valid = (
                    x is not None
                    and y is not None
                    and z is not None
                    and abs(x) + abs(y) + abs(z) > 1e-9
                )
                if geometry_valid:
                    self.geometry_valid_points += 1

                try:
                    tag = int(getattr(point, "tag"))
                except (AttributeError, TypeError, ValueError, OverflowError):
                    tag = -1
                if 0 <= tag <= 255:
                    self.tag_counts[tag] += 1
                    self.tag_observed_slots += 1
                    strict_tag = (tag & 0x3F) == 0
                    if strict_tag:
                        self.strict_tag_slots += 1
                        if geometry_valid:
                            self.strict_high_confidence_points += 1

                if geometry_valid:
                    try:
                        reflectivity = int(getattr(point, "reflectivity"))
                    except (AttributeError, TypeError, ValueError, OverflowError):
                        reflectivity = -1
                    if 0 <= reflectivity <= 255:
                        self.reflectivity_counts[reflectivity] += 1
                        self.reflectivity_observed_points += 1

                try:
                    offset = int(getattr(point, "offset_time"))
                except (AttributeError, TypeError, ValueError, OverflowError):
                    offset = -1
                if offset >= 0:
                    self.offset_observed_slots += 1
                    frame_offset_min = (
                        offset if frame_offset_min is None else min(frame_offset_min, offset)
                    )
                    frame_offset_max = (
                        offset if frame_offset_max is None else max(frame_offset_max, offset)
                    )
                    self.offset_min_ns = (
                        offset if self.offset_min_ns is None else min(self.offset_min_ns, offset)
                    )
                    self.offset_max_ns = (
                        offset if self.offset_max_ns is None else max(self.offset_max_ns, offset)
                    )
        except Exception as exc:  # Preserve a partial report for malformed messages.
            if len(self.processing_errors) < 20:
                self.processing_errors.append(f"{type(exc).__name__}: {exc}")
        if frame_offset_min is not None and frame_offset_max is not None:
            self.offset_frame_spans_ns.append(float(frame_offset_max - frame_offset_min))

    def as_dict(self) -> dict[str, Any]:
        rate = self.rate.as_dict()
        return {
            "frames": self.rate.count,
            "frame_rate_hz": rate["arrival_rate_hz"],
            "rate_detail": rate,
            "raw_slots": self.raw_slots,
            "raw_slots_definition": "len(CustomMsg.points), summed over received frames",
            "reported_point_num": self.reported_point_num,
            "analyzed_slots": self.analyzed_slots,
            "point_count_mismatch_frames": self.point_count_mismatch_frames,
            "geometry_valid_points": self.geometry_valid_points,
            "geometry_valid_definition": "finite x/y/z and not the zero vector",
            "strict_high_confidence_points": self.strict_high_confidence_points,
            "strict_high_confidence_definition": (
                "geometry-valid received slots satisfying (tag & 0x3f) == 0"
            ),
            "strict_tag_slots_including_invalid_geometry": self.strict_tag_slots,
            "tag_observed_slots": self.tag_observed_slots,
            "tag_histogram": {
                f"0x{tag:02x}": count
                for tag, count in enumerate(self.tag_counts)
                if count
            },
            "reflectivity_observed_geometry_points": self.reflectivity_observed_points,
            "reflectivity_percentiles": _histogram_percentiles(
                self.reflectivity_counts, (1, 5, 50, 95, 99)
            ),
            "reflectivity_definition": (
                "raw uint8 reflectivity on geometry-valid received slots"
            ),
            "offset_observed_slots": self.offset_observed_slots,
            "offset_min_ns": self.offset_min_ns,
            "offset_max_ns": self.offset_max_ns,
            "offset_span_ns": _value_summary(self.offset_frame_spans_ns),
            "offset_span_definition": "per-frame max(offset_time)-min(offset_time)",
            "processing_errors": list(self.processing_errors),
            "observability_note": (
                "Counts describe received CustomMsg slots only. The number of emitted "
                "rays is unavailable, so no per-ray valid ratio is computed."
            ),
        }


@dataclass
class ImuAccumulator:
    rate: RateCounter = field(default_factory=RateCounter)
    gyro_norms: list[float] = field(default_factory=list)
    accel_norms: list[float] = field(default_factory=list)
    invalid_vectors: int = 0

    def add(self, msg: Any, arrival_s: float | None = None) -> None:
        arrival_s = time.monotonic() if arrival_s is None else float(arrival_s)
        self.rate.add(arrival_s, _message_stamp_s(msg))
        angular = getattr(msg, "angular_velocity", None)
        linear = getattr(msg, "linear_acceleration", None)
        gyro = tuple(
            _finite_float(getattr(angular, axis, None)) for axis in ("x", "y", "z")
        )
        accel = tuple(
            _finite_float(getattr(linear, axis, None)) for axis in ("x", "y", "z")
        )
        if any(value is None for value in gyro + accel):
            self.invalid_vectors += 1
            return
        self.gyro_norms.append(math.sqrt(sum(float(value) ** 2 for value in gyro)))
        self.accel_norms.append(math.sqrt(sum(float(value) ** 2 for value in accel)))

    def as_dict(self) -> dict[str, Any]:
        rate = self.rate.as_dict()
        return {
            "messages": self.rate.count,
            "rate_hz": rate["arrival_rate_hz"],
            "rate_detail": rate,
            "gyro_norm": _value_summary(self.gyro_norms),
            "gyro_norm_declared_unit": "rad/s (sensor_msgs/Imu contract)",
            "accel_norm": _value_summary(self.accel_norms),
            "accel_norm_unit_note": (
                "Values are reported exactly from linear_acceleration. Verify the "
                "deployed Livox driver performs g-to-m/s^2 conversion before treating "
                "them as SI."
            ),
            "invalid_vector_messages": self.invalid_vectors,
        }


@dataclass
class StringTelemetry:
    messages: int = 0
    first_arrival_s: float | None = None
    last_arrival_s: float | None = None
    last_raw: str | None = None

    def add(self, msg: Any, arrival_s: float | None = None) -> None:
        arrival_s = time.monotonic() if arrival_s is None else float(arrival_s)
        self.messages += 1
        if self.first_arrival_s is None:
            self.first_arrival_s = arrival_s
        self.last_arrival_s = arrival_s
        self.last_raw = str(getattr(msg, "data", ""))

    def as_dict(self) -> dict[str, Any]:
        parsed: Any = None
        payload_format: str | None = None
        if self.last_raw is not None:
            try:
                parsed = json.loads(self.last_raw)
                payload_format = "json"
            except json.JSONDecodeError:
                parsed = self.last_raw
                payload_format = "text"
        return {
            "observed": self.messages > 0,
            "messages": self.messages,
            "payload_format": payload_format,
            "last_payload_verbatim": parsed,
            "interpretation": (
                "Payload is preserved verbatim/JSON-decoded only; no serial number, "
                "temperature, or health state is inferred by this tool."
            ),
        }


def _status_evidence(telemetry: Mapping[str, Any]) -> dict[str, Any]:
    """Extract traceable raw fields without pretending absent telemetry exists."""
    payload = telemetry.get("last_payload_verbatim")
    status: Mapping[str, Any] | None = None
    if isinstance(payload, Mapping):
        nested = payload.get("sdk_status")
        if isinstance(nested, Mapping):
            status = nested
        else:
            sdk_json = payload.get("sdk_json")
            if isinstance(sdk_json, str):
                try:
                    decoded = json.loads(sdk_json)
                except json.JSONDecodeError:
                    decoded = None
                if isinstance(decoded, Mapping):
                    status = decoded

    raw_core_temp: Any = None
    core_temp_c: float | None = None
    serial: Any = None
    diag: Any = None
    hms: Any = None
    if status is not None:
        raw_core_temp = status.get("core_temp")
        numeric_core_temp = _finite_float(raw_core_temp)
        if numeric_core_temp is not None:
            core_temp_c = numeric_core_temp / 100.0
        serial = status.get("sn")
        diag = status.get("lidar_diag_status")
        hms = status.get("hms_code")
    return {
        "source_topic": DEVICE_STATUS_TOPIC,
        "payload_observed": bool(telemetry.get("observed")),
        "core_temp_raw": raw_core_temp,
        "core_temp_scale_c_per_count": 0.01 if raw_core_temp is not None else None,
        "core_temp_c": core_temp_c,
        "core_temp_formula": "core_temp_raw / 100" if raw_core_temp is not None else None,
        "serial_verbatim": serial,
        "lidar_diag_status_raw": diag,
        "hms_code_raw": hms,
        "provenance_note": (
            "All populated fields come from the received sdk_status payload; missing "
            "identity or temperature remains null."
        ),
    }


def _topic_graph(node: Any) -> dict[str, Any]:
    discovered = dict(node.get_topic_names_and_types())
    graph: dict[str, Any] = {}
    for topic in RECORD_TOPICS:
        try:
            publisher_count = len(node.get_publishers_info_by_topic(topic))
        except Exception:
            publisher_count = None
        graph[topic] = {
            "types": sorted(discovered.get(topic, [])),
            "expected_type": EXPECTED_TYPES[topic],
            "publisher_count": publisher_count,
        }
    return graph


def _preflight_verdict(
    lidar: LidarAccumulator,
    imu: ImuAccumulator,
    info: StringTelemetry,
    status: StringTelemetry,
    graph: Mapping[str, Any],
    status_required: bool,
) -> tuple[str, list[str], int]:
    failures: list[str] = []
    degraded: list[str] = []
    if lidar.rate.count == 0:
        failures.append(f"no messages received on {LIDAR_TOPIC}")
    elif lidar.geometry_valid_points == 0:
        failures.append("no geometry-valid points observed")
    elif lidar.strict_high_confidence_points == 0:
        degraded.append("no geometry-valid points passed strict (tag & 0x3f) == 0")
    if imu.rate.count == 0:
        failures.append(f"no messages received on {IMU_TOPIC}")
    if status.messages == 0:
        message = (
            f"no std_msgs/String telemetry received on {DEVICE_STATUS_TOPIC}; "
            "device health cannot be established"
        )
        (failures if status_required else degraded).append(message)
    if info.messages == 0:
        degraded.append(
            f"no std_msgs/String telemetry received on {DEVICE_INFO_TOPIC}; "
            "serial number and device identity remain unknown"
        )
    if lidar.point_count_mismatch_frames:
        degraded.append("CustomMsg point_num differed from len(points) in one or more frames")
    if lidar.processing_errors:
        degraded.append("one or more lidar frames raised a processing error")
    for topic, expected in EXPECTED_TYPES.items():
        types = set(graph.get(topic, {}).get("types", []))
        publisher_count = graph.get(topic, {}).get("publisher_count")
        if publisher_count and types and expected not in types:
            message = f"{topic} is advertised with {sorted(types)}, expected {expected}"
            if topic in (LIDAR_TOPIC, IMU_TOPIC, DEVICE_STATUS_TOPIC):
                failures.append(message)
            else:
                degraded.append(message)
    if failures:
        return "failed", failures + degraded, 3
    if degraded:
        return "degraded", degraded, 2
    return "passed", [], 0


def _run_preflight(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from livox_ros_driver2.msg import CustomMsg
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
    except ImportError as exc:
        print(
            "ROS/Livox Python imports failed. Run through calibrate_mid360_health.sh "
            f"so Jazzy and the Livox overlay are sourced: {exc}",
            file=sys.stderr,
        )
        return 4

    started_utc = _utc_now()
    started = time.monotonic()
    lidar = LidarAccumulator()
    imu = ImuAccumulator()
    info = StringTelemetry()
    status = StringTelemetry()
    callback_errors: list[str] = []
    lidar_window_counts = [0] * REQUIRED_WINDOW_COUNT
    imu_window_counts = [0] * REQUIRED_WINDOW_COUNT
    analysis_started = started + PRE_ROLL_S
    analysis_ends = analysis_started + ANALYSIS_DURATION_S

    def guarded(callback: Any, window_counts: list[int] | None = None) -> Any:
        def invoke(msg: Any) -> None:
            try:
                arrival = time.monotonic()
                if window_counts is not None:
                    window_index = int((arrival - analysis_started) // WINDOW_DURATION_S)
                    if not 0 <= window_index < REQUIRED_WINDOW_COUNT:
                        return
                    window_counts[window_index] += 1
                callback(msg, arrival)
            except Exception as exc:
                if len(callback_errors) < 20:
                    callback_errors.append(f"{type(exc).__name__}: {exc}")

        return invoke

    rclpy.init(args=[])
    node = rclpy.create_node(f"mid360_health_preflight_{os.getpid()}")
    # Device info is static identity/configuration and is expected to be
    # transient-local.  Requesting that durability lets a late preflight read
    # the sample published at driver startup.  Status is periodic/volatile.
    device_info_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions = [
        node.create_subscription(
            CustomMsg,
            LIDAR_TOPIC,
            guarded(lidar.add, lidar_window_counts),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Imu, IMU_TOPIC, guarded(imu.add, imu_window_counts), qos_profile_sensor_data
        ),
        node.create_subscription(
            String, DEVICE_INFO_TOPIC, guarded(info.add), device_info_qos
        ),
        node.create_subscription(
            String, DEVICE_STATUS_TOPIC, guarded(status.add), qos_profile_sensor_data
        ),
    ]
    interrupted = False
    deadline = started + args.duration
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
    except KeyboardInterrupt:
        interrupted = True
    finally:
        observed_ends = time.monotonic()
        graph = _topic_graph(node)
        # Keep subscriptions alive until after the final graph query.
        del subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    status_required = not args.allow_missing_status
    verdict, reasons, exit_code = _preflight_verdict(
        lidar, imu, info, status, graph, status_required
    )
    if callback_errors:
        reasons.append("callback errors were observed")
        if verdict == "passed":
            verdict, exit_code = "degraded", 2
    if interrupted:
        reasons.append("operator interrupted the requested observation window")
        if verdict == "passed":
            verdict, exit_code = "degraded", 2

    completed_windows = min(
        REQUIRED_WINDOW_COUNT,
        max(0, int((min(observed_ends, analysis_ends) - analysis_started) // WINDOW_DURATION_S)),
    )
    if completed_windows < REQUIRED_WINDOW_COUNT:
        reasons.append(
            f"only {completed_windows}/{REQUIRED_WINDOW_COUNT} complete 1 s analysis "
            "windows followed the pre-roll"
        )
        if verdict == "passed":
            verdict, exit_code = "degraded", 2
    elif any(count == 0 for count in lidar_window_counts):
        reasons.append("one or more complete analysis windows contained no lidar frame")
        verdict, exit_code = "failed", 3
    elif any(count == 0 for count in imu_window_counts):
        reasons.append("one or more complete analysis windows contained no IMU message")
        verdict, exit_code = "failed", 3

    window_report = [
        {
            "index": index,
            "start_offset_s": PRE_ROLL_S + index * WINDOW_DURATION_S,
            "end_offset_s": PRE_ROLL_S + (index + 1) * WINDOW_DURATION_S,
            "lidar_frames": lidar_window_counts[index],
            "imu_messages": imu_window_counts[index],
            "clock_complete": index < completed_windows,
        }
        for index in range(REQUIRED_WINDOW_COUNT)
    ]

    info_report = info.as_dict()
    status_report = status.as_dict()

    report = {
        "schema": "mid360s-health-preflight/v1",
        "tool": "tools/mid360_health.py",
        "created_utc": _utc_now(),
        "started_utc": started_utc,
        "requested_duration_s": args.duration,
        "observed_wall_duration_s": time.monotonic() - started,
        "not_final_result": True,
        "notice": NOT_FINAL_NOTICE,
        "topic_graph": graph,
        "lidar": lidar.as_dict(),
        "imu": imu.as_dict(),
        "analysis_protocol": {
            "pre_roll_s": PRE_ROLL_S,
            "window_duration_s": WINDOW_DURATION_S,
            "required_complete_windows": REQUIRED_WINDOW_COUNT,
            "completed_windows": completed_windows,
            "windows": window_report,
        },
        "device_info": info_report,
        "device_status": status_report,
        "device_status_evidence": _status_evidence(status_report),
        "status_policy": "required" if status_required else "missing_is_degraded",
        "verdict": {"level": verdict, "reasons": reasons, "exit_code": exit_code},
        "callback_errors": callback_errors,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return exit_code


def _probe_string_telemetry(timeout_s: float) -> dict[str, Any]:
    """Receive device info/status before recording; never interpret their schema."""
    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String

    info = StringTelemetry()
    status = StringTelemetry()
    rclpy.init(args=[])
    node = rclpy.create_node(f"mid360_health_status_probe_{os.getpid()}")
    device_info_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions = [
        node.create_subscription(String, DEVICE_INFO_TOPIC, info.add, device_info_qos),
        node.create_subscription(
            String, DEVICE_STATUS_TOPIC, status.add, qos_profile_sensor_data
        ),
    ]
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and rclpy.ok():
            if info.messages and status.messages:
                break
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
        graph = _topic_graph(node)
    finally:
        del subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return {
        "probe_duration_s": timeout_s,
        "device_info": info.as_dict(),
        "device_status": status.as_dict(),
        "topic_graph": graph,
    }


def _topic_counts_from_metadata(document: Mapping[str, Any]) -> dict[str, int]:
    root = document.get("rosbag2_bagfile_information", {})
    rows = root.get("topics_with_message_count", []) if isinstance(root, Mapping) else []
    result: dict[str, int] = {}
    for row in rows if isinstance(rows, Sequence) else []:
        if not isinstance(row, Mapping):
            continue
        metadata = row.get("topic_metadata", {})
        if not isinstance(metadata, Mapping):
            continue
        name = metadata.get("name")
        try:
            count = int(row.get("message_count", 0))
        except (TypeError, ValueError, OverflowError):
            continue
        if isinstance(name, str):
            result[name] = count
    return result


def _duration_from_metadata(document: Mapping[str, Any]) -> float | None:
    root = document.get("rosbag2_bagfile_information", {})
    if not isinstance(root, Mapping):
        return None
    duration = root.get("duration", {})
    if not isinstance(duration, Mapping):
        return None
    try:
        nanoseconds = int(duration.get("nanoseconds"))
    except (TypeError, ValueError, OverflowError):
        return None
    return nanoseconds * 1e-9 if nanoseconds >= 0 else None


def _read_bag_metadata(
    output_dir: Path,
) -> tuple[dict[str, int], float | None, str | None]:
    metadata_path = output_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return {}, None, "metadata.yaml is missing"
    try:
        import yaml

        with metadata_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, Mapping):
            return {}, None, "metadata.yaml root is not a mapping"
        return (
            _topic_counts_from_metadata(document),
            _duration_from_metadata(document),
            None,
        )
    except Exception as exc:
        return {}, None, f"could not parse metadata.yaml: {type(exc).__name__}: {exc}"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _artifact_hashes(output_dir: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    excluded = {MANIFEST_NAME, CHECKSUM_NAME}
    for path in sorted(output_dir.rglob("*")):
        if not path.is_file() or path.name in excluded:
            continue
        relative = path.relative_to(output_dir).as_posix()
        if "\n" in relative or "\r" in relative:
            raise ValueError("artifact filenames containing newlines are unsupported")
        result[relative] = _sha256_file(path)
    return result


def _hash_inventory(hashes: Mapping[str, str]) -> str:
    canonical = json.dumps(
        dict(sorted(hashes.items())), separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _write_new_text(path: Path, content: str) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    descriptor = os.open(path, flags, 0o644)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            descriptor = -1
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _signal_process_group(process: subprocess.Popen[Any], signal_number: int) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal_number)
    except ProcessLookupError:
        pass


def _record_command(output_dir: Path) -> list[str]:
    return [
        "ros2",
        "bag",
        "record",
        "--output",
        str(output_dir),
        "--storage",
        "sqlite3",
        "--disable-keyboard-controls",
        "--topics",
        *RECORD_TOPICS,
    ]


def _run_record(args: argparse.Namespace) -> int:
    output_dir = Path(args.out).expanduser().resolve()
    if os.path.lexists(output_dir):
        raise ValueError(f"output path already exists; refusing to overwrite: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    formal = args.role in FORMAL_ROLES
    probe = _probe_string_telemetry(args.status_timeout)
    status_observed = bool(probe["device_status"]["observed"])
    status_types = set(
        probe["topic_graph"][DEVICE_STATUS_TOPIC].get("types", [])
    )
    if formal and not status_observed:
        detail = ""
        if status_types:
            detail = f" (advertised types: {sorted(status_types)})"
        raise RuntimeError(
            f"formal role {args.role!r} requires a received std_msgs/String "
            f"message on {DEVICE_STATUS_TOPIC}{detail}; recording was not started"
        )

    command = _record_command(output_dir)
    started_utc = _utc_now()
    started_monotonic = time.monotonic()
    print("[record] " + " ".join(command), file=sys.stderr, flush=True)
    process = subprocess.Popen(command, start_new_session=True)

    external_signal: int | None = None
    def handle_signal(signal_number: int, _frame: Any) -> None:
        nonlocal external_signal
        if external_signal is None:
            external_signal = signal_number

    for signal_number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_number, handle_signal)

    stop_reason: str | None = None
    graceful_started: float | None = None
    sent_term = False
    sent_kill = False
    try:
        while process.poll() is None:
            now = time.monotonic()
            if external_signal is not None and stop_reason is None:
                stop_reason = f"signal:{signal.Signals(external_signal).name}"
                graceful_started = now
                _signal_process_group(process, signal.SIGINT)
            elif now - started_monotonic >= args.duration and stop_reason is None:
                stop_reason = "duration_elapsed"
                graceful_started = now
                _signal_process_group(process, signal.SIGINT)

            if graceful_started is not None:
                stopping_for = now - graceful_started
                if stopping_for >= args.stop_timeout and not sent_term:
                    sent_term = True
                    _signal_process_group(process, signal.SIGTERM)
                if stopping_for >= args.stop_timeout + 5.0 and not sent_kill:
                    sent_kill = True
                    _signal_process_group(process, signal.SIGKILL)
            time.sleep(0.1)
    finally:
        # Keep our handlers installed through metadata hashing and manifest
        # creation.  A second SIGTERM during finalization must not strand a bag
        # without its evidence manifest.
        if process.poll() is None:
            _signal_process_group(process, signal.SIGINT)
            try:
                process.wait(timeout=args.stop_timeout)
            except subprocess.TimeoutExpired:
                _signal_process_group(process, signal.SIGTERM)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    _signal_process_group(process, signal.SIGKILL)
                    process.wait()

    ended_utc = _utc_now()
    elapsed = time.monotonic() - started_monotonic
    recorder_returncode = int(process.returncode or 0)
    if stop_reason is None:
        stop_reason = "recorder_exited"

    if not output_dir.is_dir():
        raise RuntimeError(
            f"ros2 bag recorder exited with {recorder_returncode} without creating {output_dir}"
        )

    topic_counts, recorded_duration_s, metadata_error = _read_bag_metadata(output_dir)
    reasons: list[str] = []
    for topic in (LIDAR_TOPIC, IMU_TOPIC):
        if topic_counts.get(topic, 0) <= 0:
            reasons.append(f"bag contains no messages for required topic {topic}")
    if formal and topic_counts.get(DEVICE_STATUS_TOPIC, 0) <= 0:
        reasons.append(
            f"formal bag contains no {DEVICE_STATUS_TOPIC} messages despite pre-record probe"
        )
    if formal and (
        recorded_duration_s is None
        or recorded_duration_s < MINIMUM_RECORDED_WINDOW_SPAN_S
    ):
        reasons.append(
            "formal bag duration does not cover the 1 s pre-roll plus ten complete "
            f"1 s windows (recorded={recorded_duration_s!r} s, "
            f"required>={MINIMUM_RECORDED_WINDOW_SPAN_S} s)"
        )
    if metadata_error:
        reasons.append(metadata_error)
    if recorder_returncode != 0:
        reasons.append(f"ros2 bag recorder returned {recorder_returncode}")

    if reasons:
        capture_state = "failed"
    elif topic_counts.get(DEVICE_STATUS_TOPIC, 0) <= 0:
        capture_state = "degraded"
        reasons.append(
            f"smoke bag has no {DEVICE_STATUS_TOPIC}; device health was not recorded"
        )
    elif topic_counts.get(DEVICE_INFO_TOPIC, 0) <= 0:
        capture_state = "degraded"
        reasons.append(
            f"bag has no {DEVICE_INFO_TOPIC}; device identity was not recorded"
        )
    else:
        capture_state = "complete"

    artifact_hashes = _artifact_hashes(output_dir)
    status_report = probe["device_status"]
    manifest = {
        "schema": "mid360s-health-acquisition-manifest/v1",
        "artifact_kind": "raw_acquisition",
        "not_final_result": True,
        "notice": NOT_FINAL_NOTICE,
        "role": args.role,
        "rig_id": args.rig_id,
        "started_utc": started_utc,
        "ended_utc": ended_utc,
        "requested_duration_s": args.duration,
        "observed_process_duration_s": elapsed,
        "recorded_bag_duration_s": recorded_duration_s,
        "analysis_protocol": {
            "pre_roll_s": PRE_ROLL_S,
            "window_duration_s": WINDOW_DURATION_S,
            "required_complete_windows": REQUIRED_WINDOW_COUNT,
            "minimum_recorded_span_s": MINIMUM_RECORDED_WINDOW_SPAN_S,
        },
        "output_directory": str(output_dir),
        "record_topics": list(RECORD_TOPICS),
        "storage": "sqlite3",
        "truth": {
            "plane_perpendicular_distance_m": args.plane_distance_m,
            "uncertainty_mm": args.truth_uncertainty_mm,
            "method": args.truth_method,
        },
        "scene": {
            "material_id": args.material_id,
            "incidence_deg": args.incidence_deg,
            "operator_ambient_temperature_c": args.ambient_temp_c,
            "ambient_temperature_source": (
                "operator_argument" if args.ambient_temp_c is not None else None
            ),
        },
        "pre_record_telemetry_probe": probe,
        "device_status_evidence": _status_evidence(status_report),
        "serial_and_device_temperature_policy": (
            "not synthesized; inspect recorded/verbatim device telemetry only"
        ),
        "recorder": {
            "command": command,
            "returncode": recorder_returncode,
            "stop_reason": stop_reason,
            "graceful_sigint_sent": graceful_started is not None,
            "sigterm_escalation_sent": sent_term,
            "sigkill_escalation_sent": sent_kill,
        },
        "recorded_topic_message_counts": {
            topic: topic_counts.get(topic, 0) for topic in RECORD_TOPICS
        },
        "metadata_error": metadata_error,
        "capture_qc": {"state": capture_state, "reasons": reasons},
        "artifact_sha256": artifact_hashes,
        "artifact_inventory_sha256": _hash_inventory(artifact_hashes),
    }
    manifest_path = output_dir / MANIFEST_NAME
    _write_new_text(
        manifest_path,
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
    )
    checksum_hashes = dict(artifact_hashes)
    checksum_hashes[MANIFEST_NAME] = _sha256_file(manifest_path)
    checksum_text = "".join(
        f"{digest}  {relative}\n"
        for relative, digest in sorted(checksum_hashes.items())
    )
    _write_new_text(output_dir / CHECKSUM_NAME, checksum_text)

    summary = {
        "output": str(output_dir),
        "manifest": str(manifest_path),
        "sha256sums": str(output_dir / CHECKSUM_NAME),
        "capture_state": capture_state,
        "reasons": reasons,
        "not_final_result": True,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if external_signal is not None:
        return 128 + external_signal
    if capture_state == "failed":
        return 6
    return 0


def _positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return value


def _nonnegative_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("must be a finite non-negative number")
    return value


def _finite_number(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("must be a finite number")
    return value


def _incidence(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 90.0:
        raise argparse.ArgumentTypeError("must be in [0, 90] degrees")
    return value


def _nonempty(text: str) -> str:
    value = text.strip()
    if not value:
        raise argparse.ArgumentTypeError("must not be empty")
    return value


def _validate_record_args(args: argparse.Namespace) -> None:
    if args.role not in FORMAL_ROLES:
        return
    if args.duration < DEFAULT_CAPTURE_DURATION_S:
        raise ValueError(
            f"formal role {args.role!r} requires --duration >= "
            f"{DEFAULT_CAPTURE_DURATION_S:g} s to protect a 1 s pre-roll and ten "
            "complete 1 s windows"
        )
    required = {
        "--plane-distance-m": args.plane_distance_m,
        "--truth-uncertainty-mm": args.truth_uncertainty_mm,
        "--truth-method": args.truth_method,
        "--material-id": args.material_id,
        "--incidence-deg": args.incidence_deg,
    }
    missing = [flag for flag, value in required.items() if value is None]
    if missing:
        raise ValueError(
            f"role {args.role!r} requires ground truth/context arguments: "
            + ", ".join(missing)
        )


def _selftest() -> int:
    def point(x: float, y: float, z: float, reflectivity: int, tag: int, offset: int) -> Any:
        return SimpleNamespace(
            x=x,
            y=y,
            z=z,
            reflectivity=reflectivity,
            tag=tag,
            offset_time=offset,
        )

    stamp = SimpleNamespace(sec=10, nanosec=0)
    header = SimpleNamespace(stamp=stamp)
    lidar = LidarAccumulator()
    lidar.add(
        SimpleNamespace(
            header=header,
            point_num=4,
            points=[
                point(1, 0, 0, 10, 0x00, 100),
                point(0, 2, 0, 20, 0x40, 200),
                point(0, 0, 0, 30, 0x00, 300),
                point(float("nan"), 0, 1, 40, 0x01, 400),
            ],
        ),
        arrival_s=1.0,
    )
    lidar.add(
        SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=100_000_000)),
            point_num=1,
            points=[point(0, 0, 3, 30, 0x01, 50)],
        ),
        arrival_s=1.1,
    )
    report = lidar.as_dict()
    assert report["frames"] == 2
    assert report["raw_slots"] == 5
    assert report["geometry_valid_points"] == 3
    assert report["strict_high_confidence_points"] == 2
    assert report["tag_histogram"] == {"0x00": 2, "0x01": 2, "0x40": 1}
    assert report["reflectivity_percentiles"]["p50"] == 20.0
    assert report["offset_span_ns"]["max"] == 300.0
    assert abs(report["frame_rate_hz"] - 10.0) < 1e-9

    imu = ImuAccumulator()
    imu.add(
        SimpleNamespace(
            header=header,
            angular_velocity=SimpleNamespace(x=3.0, y=4.0, z=0.0),
            linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.8),
        ),
        arrival_s=2.0,
    )
    imu_report = imu.as_dict()
    assert imu_report["gyro_norm"]["p50"] == 5.0
    assert imu_report["accel_norm"]["p50"] == 9.8

    metadata = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 12_500_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": LIDAR_TOPIC},
                    "message_count": 12,
                },
                {
                    "topic_metadata": {"name": DEVICE_STATUS_TOPIC},
                    "message_count": 3,
                },
            ]
        }
    }
    assert _topic_counts_from_metadata(metadata) == {
        LIDAR_TOPIC: 12,
        DEVICE_STATUS_TOPIC: 3,
    }
    assert _duration_from_metadata(metadata) == 12.5
    telemetry = {
        "observed": True,
        "last_payload_verbatim": {
            "sdk_status": {
                "sn": "TEST-SN",
                "core_temp": 5906,
                "lidar_diag_status": 0,
                "hms_code": [0, 0],
            }
        },
    }
    evidence = _status_evidence(telemetry)
    assert evidence["core_temp_raw"] == 5906
    assert evidence["core_temp_c"] == 59.06
    assert evidence["serial_verbatim"] == "TEST-SN"
    inventory_a = _hash_inventory({"b": "2", "a": "1"})
    inventory_b = _hash_inventory({"a": "1", "b": "2"})
    assert inventory_a == inventory_b and len(inventory_a) == 64
    print("mid360_health selftest: PASS")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MID-360S raw health preflight and reproducible bag capture"
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="run hardware-free pure-function tests and exit",
    )
    subparsers = parser.add_subparsers(dest="command")

    preflight = subparsers.add_parser(
        "preflight",
        help="observe fixed Livox lidar/IMU/info/status topics and print JSON QC",
    )
    preflight.add_argument(
        "--duration",
        type=_positive_float,
        default=DEFAULT_CAPTURE_DURATION_S,
        help=(
            "total observation seconds (default: 12; first 1 s is pre-roll, "
            "then ten complete 1 s windows are analyzed)"
        ),
    )
    preflight.add_argument(
        "--allow-missing-status",
        action="store_true",
        help=(
            "label missing /livox/device_status as degraded instead of failed; "
            "exit code remains 2 for degraded QC"
        ),
    )

    record = subparsers.add_parser(
        "record", help="record the four fixed evidence topics and write a manifest"
    )
    record.add_argument("--role", required=True, choices=("smoke", "calibration", "validation"))
    record.add_argument("--rig-id", required=True, type=_nonempty)
    record.add_argument(
        "--duration",
        type=_positive_float,
        default=DEFAULT_CAPTURE_DURATION_S,
        help="recording seconds (default: 12; formal roles require >=12)",
    )
    record.add_argument("--out", required=True, help="new rosbag directory; must not exist")
    record.add_argument(
        "--plane-distance-m",
        type=_positive_float,
        help="ground-truth perpendicular distance from lidar origin to plane",
    )
    record.add_argument(
        "--truth-uncertainty-mm",
        type=_nonnegative_float,
        help="uncertainty of the plane-distance ground truth",
    )
    record.add_argument(
        "--truth-method",
        type=_nonempty,
        help="traceable instrument/procedure used to establish ground truth",
    )
    record.add_argument("--material-id", type=_nonempty, help="controlled target material ID")
    record.add_argument(
        "--incidence-deg",
        type=_incidence,
        help="beam-to-plane-normal incidence angle in degrees",
    )
    record.add_argument(
        "--ambient-temp-c",
        type=_finite_number,
        help="optional operator-measured ambient temperature (not device temperature)",
    )
    record.add_argument(
        "--status-timeout",
        type=_nonnegative_float,
        default=5.0,
        help="seconds to wait for info/status before recording (default: 5)",
    )
    record.add_argument(
        "--stop-timeout",
        type=_positive_float,
        default=20.0,
        help="graceful rosbag shutdown timeout before escalation (default: 20)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.selftest:
        if args.command is not None:
            parser.error("--selftest cannot be combined with a command")
        return _selftest()
    if args.command is None:
        parser.error("a command is required (preflight or record), or use --selftest")
    try:
        if args.command == "preflight":
            return _run_preflight(args)
        if args.command == "record":
            _validate_record_args(args)
            return _run_record(args)
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError) as exc:
        print(f"mid360_health: ERROR: {exc}", file=sys.stderr)
        return 5
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
