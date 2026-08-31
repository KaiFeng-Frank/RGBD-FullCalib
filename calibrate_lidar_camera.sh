#!/usr/bin/env bash
# Minimal orchestration for target-less MID-360S <-> D435i calibration.
# The actual solver is the official direct_visual_lidar_calibration Jazzy image.
set -euo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly USER_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"
readonly LIVOX_WS="${LIVOX_WS:-${USER_DIR}/livox_mid360s_ws}"
readonly LIVOX_SETUP="${LIVOX_WS}/install/setup.bash"
readonly LIVOX_CONFIG="${LIVOX_WS}/src/livox_ros_driver2/config/MID360s_config.json"

readonly DVLC_IMAGE="${DVLC_IMAGE:-koide3/direct_visual_lidar_calibration:jazzy}"
readonly CAMCHAIN="${LIDAR_CAMERA_CAMCHAIN:-${PROJECT_DIR}/data/cam_rgb-camchain.yaml}"
readonly FACTORY_PARAMS="${D435I_FACTORY_PARAMS:-${PROJECT_DIR}/results/factory_params.json}"
readonly WORK_DIR="${LIDAR_CAMERA_WORK_DIR:-${PROJECT_DIR}/data/lidar_camera_extrinsic}"
readonly BAGS_DIR="${WORK_DIR}/bags"
readonly PREPROCESSED_DIR="${WORK_DIR}/preprocessed"

readonly CAMERA_NAMESPACE="${D435I_CAMERA_NAMESPACE:-camera}"
readonly CAMERA_NAME="${D435I_CAMERA_NAME:-camera}"
readonly CAMERA_NODE="/${CAMERA_NAMESPACE#/}/${CAMERA_NAME#/}"
readonly RAW_IMAGE_TOPIC="${D435I_RAW_IMAGE_TOPIC:-${CAMERA_NODE}/color/image_raw}"
readonly IMAGE_TOPIC="${LIDAR_CAMERA_IMAGE_TOPIC:-${RAW_IMAGE_TOPIC}/compressed}"
readonly CAMERA_INFO_TOPIC="${LIDAR_CAMERA_INFO_TOPIC:-${CAMERA_NODE}/color/camera_info}"
readonly DEPTH_IMAGE_TOPIC="${LIDAR_CAMERA_DEPTH_IMAGE_TOPIC:-${CAMERA_NODE}/depth/image_rect_raw}"
readonly DEPTH_INFO_TOPIC="${LIDAR_CAMERA_DEPTH_INFO_TOPIC:-${CAMERA_NODE}/depth/camera_info}"
readonly DEVICE_INFO_SERVICE="${D435I_DEVICE_INFO_SERVICE:-${CAMERA_NODE}/device_info}"
readonly POINTS_TOPIC="${LIDAR_CAMERA_POINTS_TOPIC:-/livox/lidar}"
readonly LIVOX_DEVICE_INFO_TOPIC="${LIVOX_DEVICE_INFO_TOPIC:-/livox/device_info}"
readonly CAPTURE_SECONDS=15
readonly RGBD_CAPTURE_SECONDS="${LIDAR_CAMERA_RGBD_CAPTURE_SECONDS:-6}"
readonly RGBD_ROOT="${LIDAR_CAMERA_RGBD_ROOT:-${WORK_DIR}/rgbd_evidence}"
readonly EXPECTED_WIDTH=1280
readonly EXPECTED_HEIGHT=720
readonly EXPECTED_COLOR_PROFILE="1280x720x30"
readonly EXPECTED_DEPTH_WIDTH=848
readonly EXPECTED_DEPTH_HEIGHT=480
readonly EXPECTED_DEPTH_PROFILE="848x480x30"
readonly COLOR_OPTICAL_FRAME="camera_color_optical_frame"
readonly DEPTH_OPTICAL_FRAME="camera_depth_optical_frame"
readonly DEFAULT_RIG_ID="mid360s-d435i-01"
readonly DEFAULT_DRAFT_OUTPUT="${PROJECT_DIR}/results/mid360s_d435i_extrinsic.draft.json"
readonly VALIDATED_OUTPUT="${PROJECT_DIR}/results/mid360s_d435i_extrinsic.json"
readonly CAPTURE_SESSION_FILE="${WORK_DIR}/capture_session.json"

CAMERA_MODEL=""
CAMERA_INTRINSICS=""
CAMERA_DISTORTION=""
CAMERA_RESOLUTION=""
EXPECTED_D435_SERIAL=""
ACTUAL_D435_SERIAL=""
ACTUAL_D435_SERIAL_SOURCE=""
ACTUAL_LIDAR_SERIAL=""
LIDAR_SERIAL_SOURCE=""
PUBLISHER_WITNESS_JSON=""
SOLVER_RUN_IMAGE="${DVLC_IMAGE}"
FROZEN_SOLVER_DIGEST=""
FROZEN_SOLVER_COMMIT=""
CAPTURE_RIG_ID="${LIDAR_CAMERA_RIG_ID:-${DEFAULT_RIG_ID}}"
MOUNT_SESSION_ID=""

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

say() {
  printf '[lidar-camera] %s\n' "$*"
}

usage() {
  cat <<'EOF'
MID-360S <-> D435i external-calibration workflow

Usage:
  ./calibrate_lidar_camera.sh preflight
  ./calibrate_lidar_camera.sh record <scene> [--rigid-mounted]
  ./calibrate_lidar_camera.sh record-rgbd <scene> --role calibration|holdout
                                      [--rigid-mounted]
  ./calibrate_lidar_camera.sh preprocess
  ./calibrate_lidar_camera.sh initial
  ./calibrate_lidar_camera.sh solve
  ./calibrate_lidar_camera.sh view
  ./calibrate_lidar_camera.sh import [--rig-id ID] [--lidar-serial SN]
                                  [--output FILE] [--force]

Publisher helpers (foreground; optional):
  ./calibrate_lidar_camera.sh camera
  ./calibrate_lidar_camera.sh camera-rgbd
  ./calibrate_lidar_camera.sh lidar-points

Typical order:
  # Terminal 1, after stopping any process that owns the D435i:
  ./calibrate_lidar_camera.sh camera
  # Terminal 2, after deliberately stopping the CustomMsg Livox driver:
  ./calibrate_lidar_camera.sh lidar-points
  # Terminal 3:
  ./calibrate_lidar_camera.sh preflight
  ./calibrate_lidar_camera.sh record scene01
  # Repose the whole rigid rig, keep it still, and record scene02 ... scene05+.
  ./calibrate_lidar_camera.sh preprocess
  ./calibrate_lidar_camera.sh initial
  ./calibrate_lidar_camera.sh solve
  ./calibrate_lidar_camera.sh view
  ./calibrate_lidar_camera.sh import

Notes:
  * record is fixed at 15 seconds and never overwrites an existing scene bag.
  * record-rgbd is a separate evidence capture.  It binds the same calibrated
    D435i serial, preregisters calibration/holdout role, records five topics,
    and never feeds its holdout directory to preprocess/solve.
  * Only the JPEG-compressed RGB transport is recorded; raw 1280x720x30 RGB is
    checked live but intentionally excluded from the bag to bound disk use.
  * The rig must remain mechanically rigid; move the whole rig only between bags.
  * preprocess always uses data/cam_rgb-camchain.yaml, not factory CameraInfo
    distortion.  Override paths/topics only with the environment variables
    documented in docs/LIDAR_CAMERA_EXTRINSIC.md.
EOF
}

prepare_ros() {
  [[ -f "${ROS_SETUP}" ]] || die \
    "ROS 2 Jazzy not found. Administrator install command: sudo apt install ros-jazzy-ros-base"

  # Keep Conda's Python/ELF libraries out of the Jazzy CLI process.
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
  unset CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CONDA_EXE _CONDA_ROOT
  unset PYTHONHOME PYTHONPATH GSETTINGS_SCHEMA_DIR GSETTINGS_SCHEMA_DIR_CONDA_BACKUP

  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  if [[ -f "${LIVOX_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${LIVOX_SETUP}"
  fi
  set -u

  command -v ros2 >/dev/null 2>&1 || die "ros2 CLI is unavailable after sourcing ${ROS_SETUP}"
}

require_ros_package() {
  local package="$1" install_hint="$2"
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    die "ROS package '${package}' is missing. Administrator install command: ${install_hint}"
  fi
}

load_camera_evidence() {
  [[ "${IMAGE_TOPIC}" == *compressed* ]] || die \
    "image topic must contain the lowercase token 'compressed' because the upstream ROS 2 preprocessor uses the topic name to select CompressedImage decoding: ${IMAGE_TOPIC}"
  [[ -r "${CAMCHAIN}" ]] || die "self-calibrated RGB camchain not found: ${CAMCHAIN}"
  [[ -r "${FACTORY_PARAMS}" ]] || die "D435i identity file not found: ${FACTORY_PARAMS}"
  command -v /usr/bin/python3 >/dev/null 2>&1 || die "/usr/bin/python3 is required"

  local parsed
  if ! parsed="$(/usr/bin/python3 - "${CAMCHAIN}" <<'PY'
import sys
import math
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    doc = yaml.safe_load(stream)
cam = doc.get("cam0", {}) if isinstance(doc, dict) else {}
if cam.get("camera_model") != "pinhole" or cam.get("distortion_model") != "radtan":
    raise SystemExit("cam0 must be Kalibr pinhole-radtan")
intr = cam.get("intrinsics")
dist = cam.get("distortion_coeffs")
res = cam.get("resolution")
if not isinstance(intr, list) or len(intr) != 4:
    raise SystemExit("cam0.intrinsics must contain fx,fy,cx,cy")
if not isinstance(dist, list) or len(dist) != 4:
    raise SystemExit("cam0.distortion_coeffs must contain k1,k2,p1,p2")
if res != [1280, 720]:
    raise SystemExit(f"cam0 resolution must be 1280x720, got {res!r}")
values = intr + dist
if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
       not math.isfinite(float(value)) for value in values):
    raise SystemExit("cam0 intrinsics/distortion must contain only finite numbers")
if float(intr[0]) <= 0 or float(intr[1]) <= 0:
    raise SystemExit("cam0 fx/fy must be positive")
if not (0 <= float(intr[2]) < res[0] and 0 <= float(intr[3]) < res[1]):
    raise SystemExit("cam0 principal point must lie inside the calibrated image")
fmt = lambda values: ",".join(format(float(v), ".17g") for v in values)
print("plumb_bob")
print(fmt(intr))
# direct_visual_lidar_calibration's plumb_bob CLI expects five values.
# Kalibr radtan estimates four, so the absent k3 term is explicitly constrained to 0.
print(fmt(dist + [0.0]))
print(f"{res[0]}x{res[1]}")
PY
)"; then
    die "failed to parse calibrated RGB parameters from ${CAMCHAIN}"
  fi

  local -a values=()
  mapfile -t values <<<"${parsed}"
  [[ ${#values[@]} -eq 4 ]] || die "unexpected camchain parser output"
  CAMERA_MODEL="${values[0]}"
  CAMERA_INTRINSICS="${values[1]}"
  CAMERA_DISTORTION="${values[2]}"
  CAMERA_RESOLUTION="${values[3]}"

  if ! EXPECTED_D435_SERIAL="$(/usr/bin/python3 - "${FACTORY_PARAMS}" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as stream:
    doc = json.load(stream)
serial = str(doc.get("serial", "")).strip()
if not serial:
    raise SystemExit("serial is missing")
print(serial)
PY
)"; then
    die "failed to read the expected D435i serial from ${FACTORY_PARAMS}"
  fi
}

topic_type() {
  timeout 4s ros2 topic type "$1" 2>/dev/null | head -n 1 | tr -d '\r' || true
}

unique_publisher_witness() {
  /usr/bin/python3 - "$@" <<'PY'
import json
import re
import subprocess
import sys

out = {}
for topic in sys.argv[1:]:
    run = subprocess.run(["ros2", "topic", "info", "-v", topic], text=True,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         timeout=8, check=False)
    if run.returncode != 0:
        raise SystemExit(f"cannot inspect publishers for {topic}: {run.stderr.strip()}")
    count_match = re.search(r"^Publisher count:\s*(\d+)\s*$", run.stdout, re.M)
    count = int(count_match.group(1)) if count_match else -1
    if count != 1:
        raise SystemExit(f"{topic} must have exactly one publisher, observed {count}")
    publisher_section = run.stdout.split("Publisher count:", 1)[1].split(
        "Subscription count:", 1)[0]
    def field(label):
        match = re.search(rf"^{re.escape(label)}:\s*(\S.*?)\s*$",
                          publisher_section, re.M)
        return match.group(1).strip() if match else ""
    witness = {
        "publisher_count": count,
        "node_name": field("Node name"),
        "node_namespace": field("Node namespace"),
        "gid": field("GID"),
        "topic_type": field("Topic type"),
    }
    if not witness["node_name"] or not witness["gid"] or not witness["topic_type"]:
        raise SystemExit(f"could not parse the unique publisher identity for {topic}")
    out[topic] = witness
print(json.dumps(out, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
PY
}

require_topic_type() {
  local topic="$1" expected="$2" actual
  actual="$(topic_type "${topic}")"
  if [[ -z "${actual}" ]]; then
    die "topic is not published: ${topic} (expected ${expected})"
  fi
  if [[ "${actual}" != "${expected}" ]]; then
    if [[ "${topic}" == "${POINTS_TOPIC}" && "${actual}" == "livox_ros_driver2/msg/CustomMsg" ]]; then
      die "${POINTS_TOPIC} is CustomMsg, but the upstream solver requires sensor_msgs/msg/PointCloud2. Deliberately stop the CustomMsg driver, then run './calibrate_lidar_camera.sh lidar-points' in another terminal."
    fi
    die "wrong type on ${topic}: got ${actual}, expected ${expected}"
  fi
  printf '  OK %-43s %s\n' "${topic}" "${actual}"
}

require_message() {
  local topic="$1"
  if ! ros2 topic echo "${topic}" --once --timeout 6 >/dev/null 2>&1; then
    die "${topic} has the right graph type but no message arrived within 6 seconds"
  fi
}

read_numeric_field() {
  local topic="$1" field="$2" output
  output="$(ros2 topic echo "${topic}" --field "${field}" --once --timeout 6 2>/dev/null || true)"
  awk '/^[[:space:]]*[0-9]+[[:space:]]*$/ {
         gsub(/[[:space:]]/, ""); print; exit
       }' <<<"${output}" || true
}

require_camera_parameter() {
  local parameter="$1" expected="$2" output normalized
  output="$(timeout 5s ros2 param get "${CAMERA_NODE}" "${parameter}" 2>/dev/null || true)"
  normalized="$(tr '[:upper:]' '[:lower:]' <<<"${output}")"
  if [[ "${normalized}" != *"${expected,,}"* ]]; then
    die "${CAMERA_NODE} parameter ${parameter} must be ${expected}; got '${output:-unavailable}'. Start the matching profile with './calibrate_lidar_camera.sh camera' or './calibrate_lidar_camera.sh camera-rgbd'."
  fi
}

read_d435_serial() {
  local service_type response parameter_output
  service_type="$(timeout 4s ros2 service type "${DEVICE_INFO_SERVICE}" 2>/dev/null | head -n 1 || true)"
  [[ "${service_type}" == "realsense2_camera_msgs/srv/DeviceInfo" ]] || die \
    "RealSense device-info service is unavailable at ${DEVICE_INFO_SERVICE}; expected realsense2_camera_msgs/srv/DeviceInfo"

  response="$(timeout 8s ros2 service call "${DEVICE_INFO_SERVICE}" "${service_type}" '{}' 2>/dev/null || true)"
  [[ -n "${response}" ]] || die "no response from ${DEVICE_INFO_SERVICE}"
  ACTUAL_D435_SERIAL="$(/usr/bin/python3 - "${response}" <<'PY'
import re
import sys
text = sys.argv[1]
match = re.search(r"serial_number\s*=\s*['\"]([^'\"]+)", text)
if not match:
    match = re.search(r"serial_number:\s*['\"]?([^,'\"}\s]+)", text)
print(match.group(1).strip() if match else "")
PY
)"
  if [[ -n "${ACTUAL_D435_SERIAL}" ]]; then
    ACTUAL_D435_SERIAL_SOURCE="${DEVICE_INFO_SERVICE}"
  else
    # Some mixed ROS binary installations cannot deserialize the optional
    # DeviceInfo service, while standard parameter services and image streams
    # remain healthy.  The driver only reaches this state after librealsense
    # has matched serial_no and opened the device, so retain that explicit
    # parameter as the measured launcher identity witness.
    parameter_output="$(timeout 5s ros2 param get "${CAMERA_NODE}" serial_no 2>/dev/null || true)"
    ACTUAL_D435_SERIAL="$(sed -nE 's/^String value is:[[:space:]]*([^[:space:]]+).*$/\1/p' \
      <<<"${parameter_output}" | head -n 1)"
    ACTUAL_D435_SERIAL_SOURCE="${CAMERA_NODE}:serial_no parameter"
  fi
  [[ -n "${ACTUAL_D435_SERIAL}" ]] || die \
    "could not obtain D435 serial from ${DEVICE_INFO_SERVICE} or ${CAMERA_NODE}:serial_no"
  [[ "${ACTUAL_D435_SERIAL}" == "${EXPECTED_D435_SERIAL}" ]] || die \
    "wrong D435i: online serial=${ACTUAL_D435_SERIAL}, calibrated serial=${EXPECTED_D435_SERIAL}"
}

read_lidar_serial() {
  local actual_type payload observed override
  actual_type="$(topic_type "${LIVOX_DEVICE_INFO_TOPIC}")"
  observed=""
  if [[ -n "${actual_type}" ]]; then
    [[ "${actual_type}" == "std_msgs/msg/String" ]] || die \
      "wrong type on ${LIVOX_DEVICE_INFO_TOPIC}: got ${actual_type}, expected std_msgs/msg/String"
    payload="$(ros2 topic echo "${LIVOX_DEVICE_INFO_TOPIC}" \
      --field data --once --timeout 6 \
      --qos-reliability reliable --qos-durability transient_local 2>/dev/null || true)"
    if [[ -n "${payload}" ]]; then
      observed="$(/usr/bin/python3 - "${payload}" <<'PY'
import json
import sys

text = sys.argv[1]
begin, end = text.find("{"), text.rfind("}")
if begin < 0 or end < begin:
    print("")
else:
    try:
        doc = json.loads(text[begin:end + 1])
    except json.JSONDecodeError:
        print("")
    else:
        print(str(doc.get("serial_number", "")).strip())
PY
)"
    fi
  fi

  override="${MID360S_SERIAL:-}"
  if [[ -n "${observed}" ]]; then
    if [[ -n "${override}" && "${override}" != "${observed}" ]]; then
      die "MID360S_SERIAL=${override} conflicts with ${LIVOX_DEVICE_INFO_TOPIC} serial_number=${observed}"
    fi
    ACTUAL_LIDAR_SERIAL="${observed}"
    LIDAR_SERIAL_SOURCE="${LIVOX_DEVICE_INFO_TOPIC}:serial_number"
  elif [[ -n "${override}" ]]; then
    ACTUAL_LIDAR_SERIAL="${override}"
    LIDAR_SERIAL_SOURCE="MID360S_SERIAL operator override"
  else
    die "MID-360S serial is unavailable from ${LIVOX_DEVICE_INFO_TOPIC}; set MID360S_SERIAL explicitly (never infer identity from IP)"
  fi
}

hardware_preflight() {
  prepare_ros
  load_camera_evidence
  require_ros_package rosbag2 "sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-sqlite3"
  require_ros_package rosbag2_storage_sqlite3 "sudo apt install ros-jazzy-rosbag2-storage-sqlite3"
  require_ros_package realsense2_camera "sudo apt install ros-jazzy-realsense2-camera"
  require_ros_package compressed_image_transport "sudo apt install ros-jazzy-image-transport-plugins"

  say "checking the three recorded topics and the unrecorded raw resolution witness"
  require_topic_type "${RAW_IMAGE_TOPIC}" "sensor_msgs/msg/Image"
  require_topic_type "${IMAGE_TOPIC}" "sensor_msgs/msg/CompressedImage"
  require_topic_type "${CAMERA_INFO_TOPIC}" "sensor_msgs/msg/CameraInfo"
  require_topic_type "${POINTS_TOPIC}" "sensor_msgs/msg/PointCloud2"

  require_message "${RAW_IMAGE_TOPIC}"
  require_message "${IMAGE_TOPIC}"
  require_message "${CAMERA_INFO_TOPIC}"
  require_message "${POINTS_TOPIC}"
  if ! PUBLISHER_WITNESS_JSON="$(unique_publisher_witness \
      "${RAW_IMAGE_TOPIC}" "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" "${POINTS_TOPIC}")"; then
    die "recorded streams do not have a unique publisher; refusing a potentially mixed bag"
  fi

  local image_width image_height info_width info_height point_fields point_frame compressed_format
  image_width="$(read_numeric_field "${RAW_IMAGE_TOPIC}" width)"
  image_height="$(read_numeric_field "${RAW_IMAGE_TOPIC}" height)"
  info_width="$(read_numeric_field "${CAMERA_INFO_TOPIC}" width)"
  info_height="$(read_numeric_field "${CAMERA_INFO_TOPIC}" height)"
  [[ "${image_width}" == "${EXPECTED_WIDTH}" && "${image_height}" == "${EXPECTED_HEIGHT}" ]] || die \
    "RGB Image is ${image_width:-?}x${image_height:-?}; calibrated intrinsics require ${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}"
  [[ "${info_width}" == "${EXPECTED_WIDTH}" && "${info_height}" == "${EXPECTED_HEIGHT}" ]] || die \
    "CameraInfo is ${info_width:-?}x${info_height:-?}; expected ${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}"
  compressed_format="$(ros2 topic echo "${IMAGE_TOPIC}" --field format --once --timeout 6 \
    2>/dev/null | awk 'tolower($0) ~ /jpeg/ { print; exit }' || true)"
  [[ "${compressed_format,,}" == *jpeg* ]] || die \
    "${IMAGE_TOPIC} is not reporting JPEG transport format (got '${compressed_format:-unavailable}')"

  point_fields="$(ros2 topic echo "${POINTS_TOPIC}" --field fields --once --timeout 6 2>/dev/null || true)"
  for field in x y z intensity; do
    grep -Eq "name:[[:space:]]*['\"]?${field}(['\"]|$)|name=['\"]${field}['\"]" \
      <<<"${point_fields}" || die \
      "${POINTS_TOPIC} PointCloud2 is missing required field '${field}'"
  done
  point_frame="$(ros2 topic echo "${POINTS_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  [[ "${point_frame}" == "livox_frame" ]] || die \
    "${POINTS_TOPIC} frame_id must be livox_frame, got '${point_frame:-unavailable}'"

  require_camera_parameter enable_color true
  require_camera_parameter rgb_camera.color_profile "${EXPECTED_COLOR_PROFILE}"
  require_camera_parameter enable_depth false
  require_camera_parameter enable_infra1 false
  require_camera_parameter enable_infra2 false
  require_camera_parameter enable_gyro false
  require_camera_parameter enable_accel false

  read_d435_serial
  read_lidar_serial

  say "hardware preflight passed"
  printf '  D435i serial:       %s (%s; matches %s)\n' \
    "${ACTUAL_D435_SERIAL}" "${ACTUAL_D435_SERIAL_SOURCE}" "${FACTORY_PARAMS}"
  printf '  MID-360S serial:    %s (%s)\n' "${ACTUAL_LIDAR_SERIAL}" "${LIDAR_SERIAL_SOURCE}"
  printf '  RGB stream:         %sx%s @ profile %s\n' "${image_width}" "${image_height}" "${EXPECTED_COLOR_PROFILE}"
  printf '  recorded transport: %s (%s)\n' "${IMAGE_TOPIC}" "${compressed_format}"
  printf '  calibrated model:   %s, %s\n' "${CAMERA_MODEL}" "${CAMERA_RESOLUTION}"
  printf '  calibrated K:       %s\n' "${CAMERA_INTRINSICS}"
  printf '  calibrated D:       %s\n' "${CAMERA_DISTORTION}"
  printf '  LiDAR fields/frame: x,y,z,intensity / %s\n' "${point_frame}"
  printf '  capture duration:   %ss (fixed)\n' "${CAPTURE_SECONDS}"
}

rgbd_hardware_preflight() {
  [[ "${RGBD_CAPTURE_SECONDS}" =~ ^[1-9][0-9]*$ ]] || die \
    "LIDAR_CAMERA_RGBD_CAPTURE_SECONDS must be a positive integer"
  ((RGBD_CAPTURE_SECONDS >= 4 && RGBD_CAPTURE_SECONDS <= 15)) || die \
    "LIDAR_CAMERA_RGBD_CAPTURE_SECONDS must be between 4 and 15 seconds"

  prepare_ros
  load_camera_evidence
  require_ros_package rosbag2 \
    "sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-sqlite3"
  require_ros_package rosbag2_storage_sqlite3 \
    "sudo apt install ros-jazzy-rosbag2-storage-sqlite3"
  require_ros_package realsense2_camera \
    "sudo apt install ros-jazzy-realsense2-camera"
  require_ros_package compressed_image_transport \
    "sudo apt install ros-jazzy-image-transport-plugins"

  say "checking the five RGB-D evidence topics and fixed device identities"
  require_topic_type "${RAW_IMAGE_TOPIC}" "sensor_msgs/msg/Image"
  require_topic_type "${IMAGE_TOPIC}" "sensor_msgs/msg/CompressedImage"
  require_topic_type "${CAMERA_INFO_TOPIC}" "sensor_msgs/msg/CameraInfo"
  require_topic_type "${DEPTH_IMAGE_TOPIC}" "sensor_msgs/msg/Image"
  require_topic_type "${DEPTH_INFO_TOPIC}" "sensor_msgs/msg/CameraInfo"
  require_topic_type "${POINTS_TOPIC}" "sensor_msgs/msg/PointCloud2"

  local topic
  for topic in "${RAW_IMAGE_TOPIC}" "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" \
      "${DEPTH_IMAGE_TOPIC}" "${DEPTH_INFO_TOPIC}" "${POINTS_TOPIC}"; do
    require_message "${topic}"
  done
  if ! PUBLISHER_WITNESS_JSON="$(unique_publisher_witness \
      "${RAW_IMAGE_TOPIC}" "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" \
      "${DEPTH_IMAGE_TOPIC}" "${DEPTH_INFO_TOPIC}" "${POINTS_TOPIC}")"; then
    die "RGB-D evidence streams do not have unique publishers"
  fi

  local image_width image_height color_info_width color_info_height
  local depth_width depth_height depth_info_width depth_info_height
  local compressed_format depth_encoding color_frame color_info_frame
  local depth_frame depth_info_frame point_frame point_fields
  image_width="$(read_numeric_field "${RAW_IMAGE_TOPIC}" width)"
  image_height="$(read_numeric_field "${RAW_IMAGE_TOPIC}" height)"
  color_info_width="$(read_numeric_field "${CAMERA_INFO_TOPIC}" width)"
  color_info_height="$(read_numeric_field "${CAMERA_INFO_TOPIC}" height)"
  depth_width="$(read_numeric_field "${DEPTH_IMAGE_TOPIC}" width)"
  depth_height="$(read_numeric_field "${DEPTH_IMAGE_TOPIC}" height)"
  depth_info_width="$(read_numeric_field "${DEPTH_INFO_TOPIC}" width)"
  depth_info_height="$(read_numeric_field "${DEPTH_INFO_TOPIC}" height)"
  [[ "${image_width}x${image_height}" == "${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}" ]] || die \
    "RGB Image is ${image_width:-?}x${image_height:-?}; expected ${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}"
  [[ "${color_info_width}x${color_info_height}" == "${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}" ]] || die \
    "color CameraInfo is ${color_info_width:-?}x${color_info_height:-?}"
  [[ "${depth_width}x${depth_height}" == "${EXPECTED_DEPTH_WIDTH}x${EXPECTED_DEPTH_HEIGHT}" ]] || die \
    "depth Image is ${depth_width:-?}x${depth_height:-?}; expected ${EXPECTED_DEPTH_WIDTH}x${EXPECTED_DEPTH_HEIGHT}"
  [[ "${depth_info_width}x${depth_info_height}" == "${EXPECTED_DEPTH_WIDTH}x${EXPECTED_DEPTH_HEIGHT}" ]] || die \
    "depth CameraInfo is ${depth_info_width:-?}x${depth_info_height:-?}"

  compressed_format="$(ros2 topic echo "${IMAGE_TOPIC}" --field format --once --timeout 6 \
    2>/dev/null | awk 'tolower($0) ~ /jpeg/ { print; exit }' || true)"
  [[ "${compressed_format,,}" == *jpeg* ]] || die \
    "${IMAGE_TOPIC} is not JPEG transport"
  depth_encoding="$(ros2 topic echo "${DEPTH_IMAGE_TOPIC}" --field encoding \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  [[ "${depth_encoding}" == "16UC1" ]] || die \
    "${DEPTH_IMAGE_TOPIC} encoding must be 16UC1, got '${depth_encoding:-unavailable}'"

  color_frame="$(ros2 topic echo "${RAW_IMAGE_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  color_info_frame="$(ros2 topic echo "${CAMERA_INFO_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  depth_frame="$(ros2 topic echo "${DEPTH_IMAGE_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  depth_info_frame="$(ros2 topic echo "${DEPTH_INFO_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  point_frame="$(ros2 topic echo "${POINTS_TOPIC}" --field header.frame_id \
    --once --timeout 6 2>/dev/null | head -n 1 | tr -d "[:space:]'\"" || true)"
  [[ "${color_frame}" == "${COLOR_OPTICAL_FRAME}" \
      && "${color_info_frame}" == "${COLOR_OPTICAL_FRAME}" ]] || die \
    "RGB/ImageInfo frame must both be ${COLOR_OPTICAL_FRAME}"
  [[ "${depth_frame}" == "${DEPTH_OPTICAL_FRAME}" \
      && "${depth_info_frame}" == "${DEPTH_OPTICAL_FRAME}" ]] || die \
    "depth Image/CameraInfo frame must both be ${DEPTH_OPTICAL_FRAME}"
  [[ "${point_frame}" == "livox_frame" ]] || die \
    "${POINTS_TOPIC} frame_id must be livox_frame, got '${point_frame:-unavailable}'"

  point_fields="$(ros2 topic echo "${POINTS_TOPIC}" --field fields \
    --once --timeout 6 2>/dev/null || true)"
  local field
  for field in x y z intensity; do
    grep -Eq "name:[[:space:]]*['\"]?${field}(['\"]|$)|name=['\"]${field}['\"]" \
      <<<"${point_fields}" || die \
      "${POINTS_TOPIC} PointCloud2 is missing required field '${field}'"
  done

  require_camera_parameter enable_color true
  require_camera_parameter rgb_camera.color_profile "${EXPECTED_COLOR_PROFILE}"
  require_camera_parameter enable_depth true
  require_camera_parameter depth_module.depth_profile "${EXPECTED_DEPTH_PROFILE}"
  require_camera_parameter enable_sync true
  require_camera_parameter enable_infra1 false
  require_camera_parameter enable_infra2 false
  require_camera_parameter enable_gyro false
  require_camera_parameter enable_accel false

  read_d435_serial
  read_lidar_serial
  say "RGB-D hardware preflight passed"
  printf '  same D435i serial: %s (%s)\n' \
    "${ACTUAL_D435_SERIAL}" "${ACTUAL_D435_SERIAL_SOURCE}"
  printf '  MID-360S serial:   %s (%s)\n' \
    "${ACTUAL_LIDAR_SERIAL}" "${LIDAR_SERIAL_SOURCE}"
  printf '  color/depth:       %sx%s / %sx%s @ synchronized 30 Hz\n' \
    "${EXPECTED_WIDTH}" "${EXPECTED_HEIGHT}" \
    "${EXPECTED_DEPTH_WIDTH}" "${EXPECTED_DEPTH_HEIGHT}"
  printf '  evidence duration: %ss (fixed for this invocation)\n' \
    "${RGBD_CAPTURE_SECONDS}"
}

require_docker_image() {
  command -v docker >/dev/null 2>&1 || die \
    "Docker CLI is missing. Administrator install command: sudo apt install docker.io"
  if ! docker info >/dev/null 2>&1; then
    die "Docker daemon is unavailable to this user. Start Docker and grant the current user Docker access; this script never invokes sudo."
  fi
  if ! docker image inspect "${DVLC_IMAGE}" >/dev/null 2>&1; then
    die "official Jazzy image is missing. Install command (no sudo): docker pull ${DVLC_IMAGE}"
  fi
}

current_solver_identity() {
  local image_id repo_digest source_commit
  image_id="$(docker image inspect "${DVLC_IMAGE}" --format '{{.Id}}' 2>/dev/null || true)"
  repo_digest="$(docker image inspect "${DVLC_IMAGE}" \
    --format '{{index .RepoDigests 0}}' 2>/dev/null || true)"
  source_commit="$(docker run --rm --entrypoint git "${DVLC_IMAGE}" \
    -C /root/ros2_ws/src/direct_visual_lidar_calibration rev-parse HEAD 2>/dev/null || true)"
  [[ "${image_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || die \
    "could not resolve immutable image ID for ${DVLC_IMAGE}"
  [[ "${repo_digest}" == *@sha256:* ]] || die \
    "could not resolve immutable repo digest for ${DVLC_IMAGE}"
  [[ "${source_commit}" =~ ^[0-9a-f]{40}$ ]] || die \
    "could not resolve upstream source commit from ${DVLC_IMAGE}"
  IMAGE_REF_VALUE="${DVLC_IMAGE}" IMAGE_ID_VALUE="${image_id}" \
  REPO_DIGEST_VALUE="${repo_digest}" SOURCE_COMMIT_VALUE="${source_commit}" \
    /usr/bin/python3 - <<'PY'
import json
import os
print(json.dumps({
    "schema": "d435i_calib/direct_visual_lidar_calibration_image/v1",
    "image_ref": os.environ["IMAGE_REF_VALUE"],
    "image_id": os.environ["IMAGE_ID_VALUE"],
    "repo_digest": os.environ["REPO_DIGEST_VALUE"],
    "source_commit": os.environ["SOURCE_COMMIT_VALUE"],
}, sort_keys=True, separators=(",", ":")))
PY
}

freeze_solver_identity() {
  local target_dir="${1:-${PREPROCESSED_DIR}}" identity parsed
  local -a values=()
  identity="$(current_solver_identity)"
  SOLVER_IDENTITY_JSON="${identity}" /usr/bin/python3 - \
    "${target_dir}/SOURCE_SOLVER.json" <<'PY'
import json
import os
import sys
document = json.loads(os.environ["SOLVER_IDENTITY_JSON"])
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
    stream.write("\n")
PY
  parsed="$(SOLVER_IDENTITY_JSON="${identity}" /usr/bin/python3 - <<'PY'
import json
import os
doc = json.loads(os.environ["SOLVER_IDENTITY_JSON"])
print(doc["image_id"])
print(doc["repo_digest"])
print(doc["source_commit"])
PY
)"
  mapfile -t values <<<"${parsed}"
  SOLVER_RUN_IMAGE="${values[0]}"
  FROZEN_SOLVER_DIGEST="${values[1]}"
  FROZEN_SOLVER_COMMIT="${values[2]}"
}

verify_frozen_solver_identity() {
  [[ -s "${PREPROCESSED_DIR}/SOURCE_SOLVER.json" ]] || die \
    "frozen solver identity is missing: ${PREPROCESSED_DIR}/SOURCE_SOLVER.json"
  local current parsed
  local -a values=()
  current="$(current_solver_identity)"
  parsed="$(CURRENT_SOLVER_JSON="${current}" /usr/bin/python3 - \
    "${PREPROCESSED_DIR}/SOURCE_SOLVER.json" <<'PY'
import json
import os
import sys
with open(sys.argv[1], "r", encoding="utf-8") as stream:
    frozen = json.load(stream)
current = json.loads(os.environ["CURRENT_SOLVER_JSON"])
identity_keys = ("image_id", "repo_digest", "source_commit")
if any(current.get(key) != frozen.get(key) for key in identity_keys):
    raise SystemExit(
        "DVLC image identity changed after preprocess; set DVLC_IMAGE to the frozen repo_digest")
print(frozen["image_id"])
print(frozen["repo_digest"])
print(frozen["source_commit"])
PY
)" || die "current solver image does not match SOURCE_SOLVER.json"
  mapfile -t values <<<"${parsed}"
  SOLVER_RUN_IMAGE="${values[0]}"
  FROZEN_SOLVER_DIGEST="${values[1]}"
  FROZEN_SOLVER_COMMIT="${values[2]}"
  docker image inspect "${SOLVER_RUN_IMAGE}" >/dev/null 2>&1 || die \
    "frozen solver image is no longer present locally: ${SOLVER_RUN_IMAGE}"
}

find_display() {
  if [[ -n "${DISPLAY:-}" ]]; then
    printf '%s\n' "${DISPLAY}"
    return
  fi
  if [[ -S /tmp/.X11-unix/X1 ]]; then
    printf ':1\n'
    return
  fi
  local socket
  socket="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' 2>/dev/null | sort | head -n 1 || true)"
  if [[ -n "${socket}" ]]; then
    printf ':%s\n' "${socket##*X}"
  fi
  return 0
}

find_xauthority() {
  local candidate
  for candidate in "${XAUTHORITY:-}" \
    "/run/user/$(id -u)/gdm/Xauthority" \
    "${USER_DIR}/.Xauthority"; do
    if [[ -n "${candidate}" && -r "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return
    fi
  done
  return 0
}

docker_gui_args() {
  local host_display host_xauthority
  host_display="$(find_display)"
  host_xauthority="$(find_xauthority)"
  [[ -n "${host_display}" ]] || die "no X display found (DISPLAY is empty and /tmp/.X11-unix has no socket)"
  [[ -n "${host_xauthority}" ]] || die \
    "no readable Xauthority found; set XAUTHORITY (expected /run/user/$(id -u)/gdm/Xauthority on this host)"

  DOCKER_ARGS=(
    run --rm
    --network host
    --ipc host
    -e HOME=/tmp
    -e "HOST_UID=$(id -u)"
    -e "HOST_GID=$(id -g)"
    -e "DISPLAY=${host_display}"
    -e XAUTHORITY=/tmp/host.Xauthority
    -e QT_X11_NO_MITSHM=1
    -e LIBGL_ALWAYS_SOFTWARE=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro
    -v "${host_xauthority}:/tmp/host.Xauthority:ro"
    -v "${PREPROCESSED_DIR}:/tmp/preprocessed:rw"
  )
  # NVIDIA is opt-in: this workstation's Docker need not have that runtime.
  if [[ "${DVLC_USE_NVIDIA:-0}" == "1" ]]; then
    DOCKER_ARGS+=(--gpus all)
  fi
  DOCKER_ARGS+=("${SOLVER_RUN_IMAGE}")
}

docker_run_owned() {
  # The official image entrypoint sources /root/ros2_ws/install/setup.bash, so
  # --user <host uid> fails before ROS starts.  Run with its expected container
  # user, then limit ownership repair to the one writable host mount.
  docker "${DOCKER_ARGS[@]}" bash -c '
    status=0
    "$@" || status=$?
    chown -R "${HOST_UID}:${HOST_GID}" /tmp/preprocessed 2>/dev/null || true
    exit "${status}"
  ' dvlc "$@"
}

confirm_rigid_mount() {
  local acknowledged="$1" duration="${2:-${CAPTURE_SECONDS}}" answer
  if [[ "${acknowledged}" == "1" ]]; then
    return
  fi
  if [[ ! -t 0 ]]; then
    die "non-interactive record requires --rigid-mounted after physically checking the bracket"
  fi
  printf '确认 MID-360S 与 D435i 已锁紧在同一刚性支架上，%s s 内完全静止？ [y/N] ' \
    "${duration}"
  read -r answer
  [[ "${answer}" == "y" || "${answer}" == "Y" ]] || die "record cancelled: rigid/static condition not confirmed"
}

ensure_capture_session() {
  local requested="${LIDAR_CAMERA_MOUNT_SESSION:-}" session
  session="$(REQUESTED_SESSION="${requested}" /usr/bin/python3 - \
    "${CAPTURE_SESSION_FILE}" "${CAPTURE_RIG_ID}" \
    "${ACTUAL_D435_SERIAL}" "${ACTUAL_LIDAR_SERIAL}" <<'PY'
import json
import os
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

path = Path(sys.argv[1])
rig_id, camera_serial, lidar_serial = sys.argv[2:5]
requested = os.environ.get("REQUESTED_SESSION", "").strip()
if path.exists():
    with path.open("r", encoding="utf-8") as stream:
        doc = json.load(stream)
    expected = {"rig_id": rig_id, "d435i_serial": camera_serial,
                "mid360s_serial": lidar_serial}
    for key, value in expected.items():
        if doc.get(key) != value:
            raise SystemExit(f"capture session {key}={doc.get(key)!r}, expected {value!r}")
    session = str(doc.get("mount_session_id", "")).strip()
    if not session:
        raise SystemExit("capture session has no mount_session_id")
    if requested and requested != session:
        raise SystemExit(
            f"requested mount session {requested!r} conflicts with frozen {session!r}")
else:
    session = requested or str(uuid.uuid4())
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {
        "schema": "d435i_calib/lidar_camera_mount_session/v1",
        "rig_id": rig_id,
        "mount_session_id": session,
        "d435i_serial": camera_serial,
        "mid360s_serial": lidar_serial,
        "created_utc": datetime.now(timezone.utc).isoformat(
            timespec="seconds").replace("+00:00", "Z"),
        "physical_policy": (
            "All bags in this work directory belong to one uninterrupted rigid "
            "mount. After loosening/remounting either sensor, use a new work directory."),
    }
    with path.open("x", encoding="utf-8") as stream:
        json.dump(doc, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
print(session)
PY
)" || die \
    "capture mount-session audit failed; after any remount, use a new LIDAR_CAMERA_WORK_DIR"
  [[ -n "${session}" ]] || die "mount_session_id is empty"
  MOUNT_SESSION_ID="${session}"
}

write_capture_manifest() {
  local scene="$1" bag_path="$2" intrinsics_sha="$3" recorded_utc="$4"
  local bag_info_file="${bag_path}/rosbag_info.txt"
  ros2 bag info "${bag_path}" >"${bag_info_file}"

  SCENE="${scene}" BAG_PATH="${bag_path}" INTRINSICS_SHA="${intrinsics_sha}" \
  RECORDED_UTC="${recorded_utc}" EXPECTED_SERIAL="${EXPECTED_D435_SERIAL}" \
  ACTUAL_SERIAL="${ACTUAL_D435_SERIAL}" CAMCHAIN_PATH="${CAMCHAIN}" \
  LIDAR_SERIAL="${ACTUAL_LIDAR_SERIAL}" LIDAR_SERIAL_SOURCE_VALUE="${LIDAR_SERIAL_SOURCE}" \
  RIG_ID_VALUE="${CAPTURE_RIG_ID}" MOUNT_SESSION_VALUE="${MOUNT_SESSION_ID}" \
  PUBLISHER_WITNESS_VALUE="${PUBLISHER_WITNESS_JSON}" \
  IMAGE_TOPIC_VALUE="${IMAGE_TOPIC}" CAMERA_INFO_TOPIC_VALUE="${CAMERA_INFO_TOPIC}" \
  POINTS_TOPIC_VALUE="${POINTS_TOPIC}" CAPTURE_SECONDS_VALUE="${CAPTURE_SECONDS}" \
  CAMERA_INTRINSICS_VALUE="${CAMERA_INTRINSICS}" CAMERA_DISTORTION_VALUE="${CAMERA_DISTORTION}" \
  /usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path
import yaml

bag_path = Path(os.environ["BAG_PATH"])
with (bag_path / "metadata.yaml").open("r", encoding="utf-8") as stream:
    metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
duration_ns = int(metadata["duration"]["nanoseconds"])
message_counts = {
    row["topic_metadata"]["name"]: int(row["message_count"])
    for row in metadata["topics_with_message_count"]
}
expected_topics = [
    os.environ["IMAGE_TOPIC_VALUE"],
    os.environ["CAMERA_INFO_TOPIC_VALUE"],
    os.environ["POINTS_TOPIC_VALUE"],
]
missing = [topic for topic in expected_topics if message_counts.get(topic, 0) <= 0]
if missing:
    raise SystemExit(f"recorded bag has zero messages on: {', '.join(missing)}")
if duration_ns < 12_000_000_000:
    raise SystemExit(f"recorded bag is too short: {duration_ns / 1e9:.3f}s < 12s")
duration_s = duration_ns / 1e9
minimum_counts = {
    os.environ["IMAGE_TOPIC_VALUE"]: max(1, int(duration_s * 20.0)),
    os.environ["CAMERA_INFO_TOPIC_VALUE"]: 1,
    os.environ["POINTS_TOPIC_VALUE"]: max(1, int(duration_s * 7.0)),
}
too_sparse = [
    f"{topic}={message_counts.get(topic, 0)}<{minimum}"
    for topic, minimum in minimum_counts.items()
    if message_counts.get(topic, 0) < minimum
]
if too_sparse:
    raise SystemExit("recorded bag is incomplete/too sparse: " + ", ".join(too_sparse))

manifest = {
    "schema": "d435i_calib/lidar_camera_capture/v1",
    "scene": os.environ["SCENE"],
    "recorded_utc": os.environ["RECORDED_UTC"],
    "duration_seconds_requested": int(os.environ["CAPTURE_SECONDS_VALUE"]),
    "duration_seconds_recorded": duration_ns / 1e9,
    "rigid_mount_confirmed": True,
    "static_during_capture_confirmed": True,
    "rig_id": os.environ["RIG_ID_VALUE"],
    "mount_session_id": os.environ["MOUNT_SESSION_VALUE"],
    "storage_id": "sqlite3",
    "topics": {
        os.environ["IMAGE_TOPIC_VALUE"]: "sensor_msgs/msg/CompressedImage",
        os.environ["CAMERA_INFO_TOPIC_VALUE"]: "sensor_msgs/msg/CameraInfo",
        os.environ["POINTS_TOPIC_VALUE"]: "sensor_msgs/msg/PointCloud2",
    },
    "message_counts": {topic: message_counts[topic] for topic in expected_topics},
    "publisher_witness": json.loads(os.environ["PUBLISHER_WITNESS_VALUE"]),
    "d435i": {
        "serial_expected": os.environ["EXPECTED_SERIAL"],
        "serial_observed": os.environ["ACTUAL_SERIAL"],
        "image_resolution": [1280, 720],
        "color_profile": "1280x720x30",
    },
    "mid360s": {
        "serial_observed": os.environ["LIDAR_SERIAL"],
        "serial_source": os.environ["LIDAR_SERIAL_SOURCE_VALUE"],
    },
    "preprocess_camera_model": {
        "source": os.environ["CAMCHAIN_PATH"],
        "source_sha256": os.environ["INTRINSICS_SHA"],
        "model": "plumb_bob",
        "intrinsics_fx_fy_cx_cy": [float(v) for v in os.environ["CAMERA_INTRINSICS_VALUE"].split(",")],
        "distortion_k1_k2_p1_p2_k3": [float(v) for v in os.environ["CAMERA_DISTORTION_VALUE"].split(",")],
        "policy": "explicit self-calibration; recorded factory CameraInfo is not used as solver intrinsics",
    },
}
with (bag_path / "capture_manifest.json").open("x", encoding="utf-8") as stream:
    json.dump(manifest, stream, indent=2, ensure_ascii=False)
    stream.write("\n")
PY

  (
    cd "${bag_path}"
    find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 \
      | sort -z \
      | xargs -0 sha256sum >SHA256SUMS
  )
}

record_scene() {
  local scene="${1:-}" rigid_ack="${2:-0}"
  [[ -n "${scene}" ]] || die "record requires a scene name"
  [[ "${scene}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die \
    "scene must match [A-Za-z0-9][A-Za-z0-9._-]* (no paths or spaces)"

  local bag_path="${BAGS_DIR}/${scene}"
  [[ ! -e "${bag_path}" ]] || die "refusing to overwrite existing bag: ${bag_path}"

  hardware_preflight
  mkdir -p "${BAGS_DIR}"
  ensure_capture_session
  confirm_rigid_mount "${rigid_ack}"

  local available_kib
  available_kib="$(df -Pk "${BAGS_DIR}" | awk 'NR == 2 {print $4}')"
  [[ "${available_kib}" =~ ^[0-9]+$ ]] || die "could not determine free disk space for ${BAGS_DIR}"
  ((available_kib >= 5 * 1024 * 1024)) || die \
    "less than 5 GiB is free on the capture filesystem; refusing to start a new bag"
  printf '  free disk before capture: %s\n' "$(df -h "${BAGS_DIR}" | awk 'NR == 2 {print $4}')"

  local intrinsics_sha recorded_utc record_status
  intrinsics_sha="$(sha256sum "${CAMCHAIN}" | awk '{print $1}')"
  recorded_utc="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"

  say "recording '${scene}' for ${CAPTURE_SECONDS}s; do not touch the rig"
  printf '  output: %s\n' "${bag_path}"
  sleep 2

  set +e
  timeout --signal=INT --kill-after=8s "${CAPTURE_SECONDS}s" \
    ros2 bag record \
      --storage sqlite3 \
      --output "${bag_path}" \
      --disable-keyboard-controls \
      --max-cache-size 104857600 \
      --custom-data \
        "scene=${scene}" \
        "rigid_mount_confirmed=true" \
        "d435i_serial=${ACTUAL_D435_SERIAL}" \
        "mid360s_serial=${ACTUAL_LIDAR_SERIAL}" \
        "camchain_sha256=${intrinsics_sha}" \
      --topics "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" "${POINTS_TOPIC}"
  record_status=$?
  set -e

  case "${record_status}" in
    0|124|130) ;;
    *) die "ros2 bag record exited with status ${record_status}; partial bag was preserved at ${bag_path}" ;;
  esac
  [[ -f "${bag_path}/metadata.yaml" ]] || die "bag metadata is missing; partial bag preserved at ${bag_path}"

  write_capture_manifest "${scene}" "${bag_path}" "${intrinsics_sha}" "${recorded_utc}"
  say "capture complete: ${bag_path}"
  printf '  actual bag size: %s\n' "$(du -sh "${bag_path}" | awk '{print $1}')"
  printf '  Move/repose the WHOLE rigid rig before the next scene; do not alter the bracket.\n'
}

rgbd_bag_path() {
  local role="$1" scene="$2"
  [[ "${role}" == "calibration" || "${role}" == "holdout" ]] || die \
    "RGB-D role must be calibration or holdout"
  printf '%s/%s/bags/%s\n' "${RGBD_ROOT}" "${role}" "${scene}"
}

rgbd_evidence_path() {
  local role="$1" scene="$2"
  [[ "${role}" == "calibration" || "${role}" == "holdout" ]] || die \
    "RGB-D role must be calibration or holdout"
  printf '%s/%s/evidence/%s\n' "${RGBD_ROOT}" "${role}" "${scene}"
}

reserve_rgbd_role() {
  local scene="$1" role="$2" role_dir role_file other_role candidate
  [[ "${role}" == "calibration" || "${role}" == "holdout" ]] || die \
    "RGB-D role must be calibration or holdout"
  role_dir="${RGBD_ROOT}/roles"
  role_file="${role_dir}/${scene}.json"
  [[ ! -e "${role_file}" ]] || die \
    "scene role is already frozen and cannot be changed: ${role_file}"
  for other_role in calibration holdout; do
    candidate="$(rgbd_bag_path "${other_role}" "${scene}")"
    [[ ! -e "${candidate}" ]] || die \
      "scene already exists under ${other_role}; refusing cross-role reuse: ${candidate}"
    candidate="$(rgbd_evidence_path "${other_role}" "${scene}")"
    [[ ! -e "${candidate}" ]] || die \
      "scene evidence already exists under ${other_role}: ${candidate}"
  done
  mkdir -p "${role_dir}"
  ROLE_FILE="${role_file}" SCENE_VALUE="${scene}" ROLE_VALUE="${role}" \
  RIG_ID_VALUE="${CAPTURE_RIG_ID}" MOUNT_SESSION_VALUE="${MOUNT_SESSION_ID}" \
  D435_SERIAL_VALUE="${ACTUAL_D435_SERIAL}" \
  LIDAR_SERIAL_VALUE="${ACTUAL_LIDAR_SERIAL}" /usr/bin/python3 - <<'PY' || \
    die "failed to freeze RGB-D capture role: ${role_file}"
import json
import os
from datetime import datetime, timezone

document = {
    "schema": "d435i_calib/lidar_camera_capture_role/v1",
    "scene": os.environ["SCENE_VALUE"],
    "role": os.environ["ROLE_VALUE"],
    "registered_utc": datetime.now(timezone.utc).isoformat(
        timespec="seconds").replace("+00:00", "Z"),
    "rig_id": os.environ["RIG_ID_VALUE"],
    "mount_session_id": os.environ["MOUNT_SESSION_VALUE"],
    "d435i_serial": os.environ["D435_SERIAL_VALUE"],
    "mid360s_serial": os.environ["LIDAR_SERIAL_VALUE"],
    "policy": (
        "holdout: never enters calibration, initialization, seed selection, "
        "correspondence threshold tuning, or solver"
        if os.environ["ROLE_VALUE"] == "holdout" else
        "calibration: may enter solver and can never be relabeled as holdout"
    ),
}
with open(os.environ["ROLE_FILE"], "x", encoding="utf-8") as stream:
    json.dump(document, stream, ensure_ascii=False, indent=2,
              sort_keys=True, allow_nan=False)
    stream.write("\n")
PY
  printf '%s\n' "${role_file}"
}

write_rgbd_capture_manifest() {
  local scene="$1" role="$2" bag_path="$3" role_file="$4"
  local intrinsics_sha="$5" factory_sha="$6" recorded_utc="$7"
  local bag_info_file="${bag_path}/rosbag_info.txt"
  ros2 bag info "${bag_path}" >"${bag_info_file}"
  install -m 0444 "${role_file}" "${bag_path}/capture_role.json"

  SCENE="${scene}" ROLE_VALUE="${role}" BAG_PATH="${bag_path}" \
  INTRINSICS_SHA="${intrinsics_sha}" FACTORY_SHA="${factory_sha}" \
  RECORDED_UTC="${recorded_utc}" D435_SERIAL="${ACTUAL_D435_SERIAL}" \
  LIDAR_SERIAL="${ACTUAL_LIDAR_SERIAL}" RIG_ID_VALUE="${CAPTURE_RIG_ID}" \
  MOUNT_SESSION_VALUE="${MOUNT_SESSION_ID}" \
  PUBLISHER_WITNESS_VALUE="${PUBLISHER_WITNESS_JSON}" \
  IMAGE_TOPIC_VALUE="${IMAGE_TOPIC}" COLOR_INFO_TOPIC_VALUE="${CAMERA_INFO_TOPIC}" \
  DEPTH_TOPIC_VALUE="${DEPTH_IMAGE_TOPIC}" DEPTH_INFO_TOPIC_VALUE="${DEPTH_INFO_TOPIC}" \
  POINTS_TOPIC_VALUE="${POINTS_TOPIC}" CAPTURE_SECONDS_VALUE="${RGBD_CAPTURE_SECONDS}" \
  /usr/bin/python3 - <<'PY'
import hashlib
import json
import os
from pathlib import Path
import yaml

bag_path = Path(os.environ["BAG_PATH"])
with (bag_path / "metadata.yaml").open("r", encoding="utf-8") as stream:
    info = yaml.safe_load(stream)["rosbag2_bagfile_information"]
duration_ns = int(info["duration"]["nanoseconds"])
duration_s = duration_ns / 1e9
requested = int(os.environ["CAPTURE_SECONDS_VALUE"])
if duration_s < max(2.0, requested - 2.0):
    raise SystemExit(
        f"RGB-D evidence bag is too short: {duration_s:.3f}s for requested {requested}s")
types = {
    row["topic_metadata"]["name"]: row["topic_metadata"]["type"]
    for row in info["topics_with_message_count"]
}
counts = {
    row["topic_metadata"]["name"]: int(row["message_count"])
    for row in info["topics_with_message_count"]
}
expected = {
    os.environ["IMAGE_TOPIC_VALUE"]: "sensor_msgs/msg/CompressedImage",
    os.environ["COLOR_INFO_TOPIC_VALUE"]: "sensor_msgs/msg/CameraInfo",
    os.environ["DEPTH_TOPIC_VALUE"]: "sensor_msgs/msg/Image",
    os.environ["DEPTH_INFO_TOPIC_VALUE"]: "sensor_msgs/msg/CameraInfo",
    os.environ["POINTS_TOPIC_VALUE"]: "sensor_msgs/msg/PointCloud2",
}
wrong = [f"{topic}={types.get(topic)!r}, expected {kind!r}"
         for topic, kind in expected.items() if types.get(topic) != kind]
if wrong:
    raise SystemExit("recorded five-topic layout is incomplete/wrong: " + "; ".join(wrong))
minimum = {
    os.environ["IMAGE_TOPIC_VALUE"]: max(1, int(duration_s * 15)),
    os.environ["COLOR_INFO_TOPIC_VALUE"]: max(1, int(duration_s * 15)),
    os.environ["DEPTH_TOPIC_VALUE"]: max(1, int(duration_s * 15)),
    os.environ["DEPTH_INFO_TOPIC_VALUE"]: max(1, int(duration_s * 15)),
    os.environ["POINTS_TOPIC_VALUE"]: max(1, int(duration_s * 7)),
}
sparse = [f"{topic}={counts.get(topic, 0)}<{limit}"
          for topic, limit in minimum.items() if counts.get(topic, 0) < limit]
if sparse:
    raise SystemExit("recorded RGB-D bag is incomplete/too sparse: " + ", ".join(sparse))

role_path = bag_path / "capture_role.json"
role_sha = hashlib.sha256(role_path.read_bytes()).hexdigest()
with role_path.open("r", encoding="utf-8") as stream:
    role_doc = json.load(stream)
for key, value in {
    "scene": os.environ["SCENE"], "role": os.environ["ROLE_VALUE"],
    "rig_id": os.environ["RIG_ID_VALUE"],
    "mount_session_id": os.environ["MOUNT_SESSION_VALUE"],
    "d435i_serial": os.environ["D435_SERIAL"],
    "mid360s_serial": os.environ["LIDAR_SERIAL"],
}.items():
    if role_doc.get(key) != value:
        raise SystemExit(f"capture role registration disagrees on {key}")

manifest = {
    "schema": "d435i_calib/lidar_camera_rgbd_capture/v1",
    "scene": os.environ["SCENE"],
    "role": os.environ["ROLE_VALUE"],
    "recorded_utc": os.environ["RECORDED_UTC"],
    "duration_seconds_requested": requested,
    "duration_seconds_recorded": duration_s,
    "rigid_mount_confirmed": True,
    "static_during_capture_confirmed": True,
    "rig_id": os.environ["RIG_ID_VALUE"],
    "mount_session_id": os.environ["MOUNT_SESSION_VALUE"],
    "d435i_serial": os.environ["D435_SERIAL"],
    "mid360s_serial": os.environ["LIDAR_SERIAL"],
    "storage_id": "sqlite3",
    "topics": expected,
    "message_counts": {topic: counts[topic] for topic in expected},
    "frames": {
        "color": "camera_color_optical_frame",
        "depth": "camera_depth_optical_frame",
        "lidar": "livox_frame",
    },
    "profiles": {
        "color": {"resolution": [1280, 720], "fps": 30, "transport": "jpeg"},
        "depth": {"resolution": [848, 480], "fps": 30, "encoding": "16UC1"},
    },
    "publisher_witness": json.loads(os.environ["PUBLISHER_WITNESS_VALUE"]),
    "camchain_sha256": os.environ["INTRINSICS_SHA"],
    "factory_params_sha256": os.environ["FACTORY_SHA"],
    "role_registration_sha256": role_sha,
    "role_policy": role_doc["policy"],
    "evidence_policy": (
        "per-frame stamps, organized depth pixels and PointCloud2 point ordinals "
        "must be retained; this manifest is not a calibration result"
    ),
}
with (bag_path / "capture_manifest.json").open("x", encoding="utf-8") as stream:
    json.dump(manifest, stream, ensure_ascii=False, indent=2,
              sort_keys=True, allow_nan=False)
    stream.write("\n")
PY

  (
    cd "${bag_path}"
    find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 \
      | sort -z \
      | xargs -0 sha256sum >SHA256SUMS
  )
}

record_rgbd_scene() {
  local scene="${1:-}" role="${2:-}" rigid_ack="${3:-0}"
  [[ -n "${scene}" ]] || die "record-rgbd requires a scene name"
  [[ "${scene}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die \
    "scene must match [A-Za-z0-9][A-Za-z0-9._-]* (no paths or spaces)"
  [[ "${role}" == "calibration" || "${role}" == "holdout" ]] || die \
    "record-rgbd requires --role calibration|holdout"

  local bag_path evidence_path other_role candidate
  bag_path="$(rgbd_bag_path "${role}" "${scene}")"
  evidence_path="$(rgbd_evidence_path "${role}" "${scene}")"
  for other_role in calibration holdout; do
    candidate="$(rgbd_bag_path "${other_role}" "${scene}")"
    [[ ! -e "${candidate}" ]] || die "refusing to overwrite/relabel existing bag: ${candidate}"
    candidate="$(rgbd_evidence_path "${other_role}" "${scene}")"
    [[ ! -e "${candidate}" ]] || die "refusing to overwrite/relabel existing evidence: ${candidate}"
  done

  rgbd_hardware_preflight
  mkdir -p "$(dirname "${bag_path}")" "$(dirname "${evidence_path}")"
  ensure_capture_session
  confirm_rigid_mount "${rigid_ack}" "${RGBD_CAPTURE_SECONDS}"

  local available_kib role_file intrinsics_sha factory_sha recorded_utc record_status
  available_kib="$(df -Pk "${RGBD_ROOT}" | awk 'NR == 2 {print $4}')"
  [[ "${available_kib}" =~ ^[0-9]+$ ]] || die \
    "could not determine free disk space for ${RGBD_ROOT}"
  ((available_kib >= 5 * 1024 * 1024)) || die \
    "less than 5 GiB is free; refusing raw depth evidence capture"

  # Role is immutable before rosbag starts.  A failed/partial attempt keeps the
  # reservation as provenance and must use a new scene name on retry.
  role_file="$(reserve_rgbd_role "${scene}" "${role}")" || die \
    "failed to preregister RGB-D scene role"
  intrinsics_sha="$(sha256sum "${CAMCHAIN}" | awk '{print $1}')"
  factory_sha="$(sha256sum "${FACTORY_PARAMS}" | awk '{print $1}')"
  recorded_utc="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"

  say "recording preregistered RGB-D ${role} scene '${scene}' for ${RGBD_CAPTURE_SECONDS}s"
  printf '  role record: %s\n' "${role_file}"
  printf '  bag output:  %s\n' "${bag_path}"
  sleep 2
  set +e
  timeout --signal=INT --kill-after=8s "${RGBD_CAPTURE_SECONDS}s" \
    ros2 bag record \
      --storage sqlite3 \
      --output "${bag_path}" \
      --disable-keyboard-controls \
      --max-cache-size 536870912 \
      --custom-data \
        "scene=${scene}" \
        "role=${role}" \
        "role_registration_sha256=$(sha256sum "${role_file}" | awk '{print $1}')" \
        "rigid_mount_confirmed=true" \
        "d435i_serial=${ACTUAL_D435_SERIAL}" \
        "mid360s_serial=${ACTUAL_LIDAR_SERIAL}" \
        "camchain_sha256=${intrinsics_sha}" \
        "factory_params_sha256=${factory_sha}" \
      --topics "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" \
        "${DEPTH_IMAGE_TOPIC}" "${DEPTH_INFO_TOPIC}" "${POINTS_TOPIC}"
  record_status=$?
  set -e
  case "${record_status}" in
    0|124|130) ;;
    *) die "ros2 bag record exited with status ${record_status}; partial role/bag preserved" ;;
  esac
  [[ -f "${bag_path}/metadata.yaml" ]] || die \
    "RGB-D bag metadata is missing; role reservation and partial bag were preserved"

  write_rgbd_capture_manifest "${scene}" "${role}" "${bag_path}" "${role_file}" \
    "${intrinsics_sha}" "${factory_sha}" "${recorded_utc}"
  /usr/bin/python3 "${PROJECT_DIR}/tools/lidar_camera_evidence.py" build \
    "${bag_path}" --output "${evidence_path}" --expect-role "${role}" \
    --d435i-serial "${ACTUAL_D435_SERIAL}" \
    --mid360s-serial "${ACTUAL_LIDAR_SERIAL}" \
    --camchain "${CAMCHAIN}" --factory-params "${FACTORY_PARAMS}" \
    --image-topic "${IMAGE_TOPIC}" --color-info-topic "${CAMERA_INFO_TOPIC}" \
    --depth-topic "${DEPTH_IMAGE_TOPIC}" --depth-info-topic "${DEPTH_INFO_TOPIC}" \
    --points-topic "${POINTS_TOPIC}"
  say "RGB-D evidence capture complete"
  printf '  immutable role: %s\n' "${role}"
  printf '  source bag:     %s\n' "${bag_path}"
  printf '  evidence:       %s\n' "${evidence_path}"
  printf '  Repose the WHOLE rigid rig; never loosen or remount either sensor.\n'
}

audit_capture_bags() {
  local camchain_sha session_lidar_serial
  if [[ -z "${EXPECTED_D435_SERIAL}" ]]; then
    load_camera_evidence
  fi
  camchain_sha="$(sha256sum "${CAMCHAIN}" | awk '{print $1}')"
  session_lidar_serial="$(/usr/bin/python3 - "${CAPTURE_SESSION_FILE}" \
    "${EXPECTED_D435_SERIAL}" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    doc = json.load(stream)
if doc.get("d435i_serial") != sys.argv[2]:
    raise SystemExit("capture session D435i serial differs from factory_params.json")
serial = str(doc.get("mid360s_serial", "")).strip()
if not serial:
    raise SystemExit("capture session has no MID-360S serial")
print(serial)
PY
)" || die "failed to read frozen device identity from ${CAPTURE_SESSION_FILE}"
  /usr/bin/python3 - "${BAGS_DIR}" "${IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" \
    "${POINTS_TOPIC}" "${RAW_IMAGE_TOPIC}" "${EXPECTED_D435_SERIAL}" "${session_lidar_serial}" \
    "${camchain_sha}" <<'PY'
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import yaml

root = Path(sys.argv[1])
expected_topics = {
    sys.argv[2]: "sensor_msgs/msg/CompressedImage",
    sys.argv[3]: "sensor_msgs/msg/CameraInfo",
    sys.argv[4]: "sensor_msgs/msg/PointCloud2",
}
raw_topic = sys.argv[5]
d435_serial, lidar_serial, camchain_sha = sys.argv[6:9]
if not root.is_dir() or root.is_symlink():
    raise SystemExit(f"capture directory is missing/unsafe: {root}")
bags = sorted(path for path in root.iterdir() if path.is_dir())
if not bags:
    raise SystemExit(f"no rosbag2 scenes found under {root}")

line_re = re.compile(r"^([0-9a-f]{64}) [ *](\./[^/]+)$")
records = []
rig_ids = set()
mount_sessions = set()
for bag in bags:
    if bag.is_symlink():
        raise SystemExit(f"bag directory may not be a symlink: {bag}")
    required = {"metadata.yaml", "capture_manifest.json", "rosbag_info.txt", "SHA256SUMS"}
    absent = sorted(name for name in required if not (bag / name).is_file())
    if absent:
        raise SystemExit(f"{bag.name}: incomplete/partial capture; missing {', '.join(absent)}")

    listed = {}
    for line in (bag / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        match = line_re.fullmatch(line)
        if not match:
            raise SystemExit(f"{bag.name}: malformed SHA256SUMS line: {line!r}")
        name = match.group(2)[2:]
        if name in listed:
            raise SystemExit(f"{bag.name}: duplicate SHA256SUMS entry: {name}")
        listed[name] = match.group(1)
    actual_files = {path.name for path in bag.iterdir()
                    if path.is_file() and path.name != "SHA256SUMS"}
    if set(listed) != actual_files:
        raise SystemExit(
            f"{bag.name}: SHA256SUMS file set mismatch; "
            f"listed={sorted(listed)}, actual={sorted(actual_files)}")
    for name, expected_hash in listed.items():
        digest = hashlib.sha256()
        with (bag / name).open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != expected_hash:
            raise SystemExit(f"{bag.name}: SHA-256 mismatch: {name}")
    tree_digest = hashlib.sha256(b"d435i-calib-capture-tree-v1\0")
    for name, digest_hex in sorted(listed.items()):
        encoded = name.encode("utf-8")
        tree_digest.update(len(encoded).to_bytes(8, "big"))
        tree_digest.update(encoded)
        tree_digest.update(bytes.fromhex(digest_hex))

    with (bag / "capture_manifest.json").open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    with (bag / "metadata.yaml").open("r", encoding="utf-8") as stream:
        info = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    if manifest.get("schema") != "d435i_calib/lidar_camera_capture/v1":
        raise SystemExit(f"{bag.name}: unsupported capture manifest schema")
    if manifest.get("scene") != bag.name:
        raise SystemExit(f"{bag.name}: manifest scene does not match directory")
    if (manifest.get("rigid_mount_confirmed") is not True or
            manifest.get("static_during_capture_confirmed") is not True):
        raise SystemExit(f"{bag.name}: rigid/static confirmation is missing")
    if manifest.get("storage_id") != "sqlite3" or info.get("storage_identifier") != "sqlite3":
        raise SystemExit(f"{bag.name}: storage must be sqlite3")
    if manifest.get("topics") != expected_topics:
        raise SystemExit(f"{bag.name}: manifest topics/types do not match this run")
    rig_id = str(manifest.get("rig_id", "")).strip()
    mount_session = str(manifest.get("mount_session_id", "")).strip()
    if not rig_id or not mount_session:
        raise SystemExit(f"{bag.name}: rig_id/mount_session_id is missing")
    rig_ids.add(rig_id)
    mount_sessions.add(mount_session)
    if manifest.get("d435i", {}).get("serial_expected") != d435_serial or \
            manifest.get("d435i", {}).get("serial_observed") != d435_serial:
        raise SystemExit(f"{bag.name}: wrong D435i identity")
    if manifest.get("mid360s", {}).get("serial_observed") != lidar_serial:
        raise SystemExit(f"{bag.name}: wrong MID-360S identity")
    if manifest.get("preprocess_camera_model", {}).get("source_sha256") != camchain_sha:
        raise SystemExit(f"{bag.name}: RGB camchain hash differs from the current calibration")
    witness = manifest.get("publisher_witness")
    witness_types = dict(expected_topics)
    witness_types[raw_topic] = "sensor_msgs/msg/Image"
    if not isinstance(witness, dict) or set(witness) != set(witness_types):
        raise SystemExit(f"{bag.name}: publisher witness does not cover the four live topics")
    for topic, expected_type in witness_types.items():
        item = witness.get(topic)
        if (not isinstance(item, dict) or item.get("publisher_count") != 1 or
                item.get("topic_type") != expected_type or
                not str(item.get("node_name", "")).strip() or
                not str(item.get("gid", "")).strip()):
            raise SystemExit(f"{bag.name}: invalid unique-publisher witness for {topic}")

    duration_ns = int(info["duration"]["nanoseconds"])
    duration_s = duration_ns / 1e9
    if duration_s < 12.0:
        raise SystemExit(f"{bag.name}: duration {duration_s:.3f}s < 12s")
    if not math.isclose(float(manifest.get("duration_seconds_recorded", -1)),
                        duration_s, rel_tol=0, abs_tol=1e-6):
        raise SystemExit(f"{bag.name}: manifest duration disagrees with metadata.yaml")
    rows = info["topics_with_message_count"]
    metadata_types = {row["topic_metadata"]["name"]: row["topic_metadata"]["type"]
                      for row in rows}
    counts = {row["topic_metadata"]["name"]: int(row["message_count"])
              for row in rows}
    if metadata_types != expected_topics:
        raise SystemExit(f"{bag.name}: bag topics/types do not match this run")
    if manifest.get("message_counts") != {topic: counts[topic] for topic in expected_topics}:
        raise SystemExit(f"{bag.name}: manifest message counts disagree with metadata.yaml")
    minimum = {sys.argv[2]: int(duration_s * 20.0), sys.argv[3]: 1,
               sys.argv[4]: int(duration_s * 7.0)}
    sparse = [f"{topic}={counts.get(topic, 0)}<{limit}"
              for topic, limit in minimum.items() if counts.get(topic, 0) < limit]
    if sparse:
        raise SystemExit(f"{bag.name}: incomplete/too sparse: {', '.join(sparse)}")
    records.append({
        "scene": bag.name,
        "tree_sha256": tree_digest.hexdigest(),
        "manifest_sha256": listed["capture_manifest.json"],
        "metadata_sha256": listed["metadata.yaml"],
        "duration_ns": duration_ns,
        "message_counts": {topic: counts[topic] for topic in sorted(expected_topics)},
    })

if len(rig_ids) != 1 or len(mount_sessions) != 1:
    raise SystemExit(
        f"capture set mixes rig/mount identities: rig={sorted(rig_ids)}, "
        f"mount={sorted(mount_sessions)}")
session_path = root.parent / "capture_session.json"
if not session_path.is_file() or session_path.is_symlink():
    raise SystemExit(f"capture session record is missing/unsafe: {session_path}")
with session_path.open("r", encoding="utf-8") as stream:
    session_doc = json.load(stream)
rig_id = next(iter(rig_ids))
mount_session = next(iter(mount_sessions))
if (session_doc.get("schema") != "d435i_calib/lidar_camera_mount_session/v1" or
        session_doc.get("rig_id") != rig_id or
        session_doc.get("mount_session_id") != mount_session or
        session_doc.get("d435i_serial") != d435_serial or
        session_doc.get("mid360s_serial") != lidar_serial):
    raise SystemExit("capture_session.json disagrees with the captured rig/mount/devices")
session_sha = hashlib.sha256(session_path.read_bytes()).hexdigest()

print(json.dumps({
    "schema": "d435i_calib/lidar_camera_capture_set/v1",
    "topics": expected_topics,
    "d435i_serial": d435_serial,
    "mid360s_serial": lidar_serial,
    "rig_id": rig_id,
    "mount_session_id": mount_session,
    "capture_session_sha256": session_sha,
    "camchain_sha256": camchain_sha,
    "captures": records,
}, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
PY
}

verify_frozen_capture_bags() {
  [[ -s "${PREPROCESSED_DIR}/SOURCE_BAGS.json" ]] || die \
    "frozen capture-set identity is missing: ${PREPROCESSED_DIR}/SOURCE_BAGS.json"
  local current
  if ! current="$(audit_capture_bags)"; then
    die "capture-bag integrity/identity audit failed"
  fi
  CURRENT_CAPTURE_AUDIT="${current}" /usr/bin/python3 - \
    "${PREPROCESSED_DIR}/SOURCE_BAGS.json" <<'PY' || die \
      "capture bags changed after preprocess; refusing stale solve/import provenance"
import json
import os
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    frozen = json.load(stream)
current = json.loads(os.environ["CURRENT_CAPTURE_AUDIT"])
if current != frozen:
    raise SystemExit("current capture-set identity differs from SOURCE_BAGS.json")
PY
}

preprocess_bags() {
  require_docker_image
  load_camera_evidence
  [[ -d "${BAGS_DIR}" ]] || die "no capture directory: ${BAGS_DIR}"

  local bag_count capture_audit staging_dir
  if ! capture_audit="$(audit_capture_bags)"; then
    die "capture-bag integrity/identity audit failed; partial data was preserved and will not be solved"
  fi
  bag_count="$(/usr/bin/python3 -c \
    'import json,sys; print(len(json.loads(sys.argv[1])["captures"]))' \
    "${capture_audit}")"
  [[ ! -e "${PREPROCESSED_DIR}" ]] || die \
    "refusing to overwrite preprocessed data: ${PREPROCESSED_DIR} (move it aside before rerunning)"
  staging_dir="${PREPROCESSED_DIR}.staging.$(date -u +'%Y%m%dT%H%M%SZ').$$"
  [[ ! -e "${staging_dir}" ]] || die "preprocess staging path already exists: ${staging_dir}"
  mkdir -p "${staging_dir}"
  CAPTURE_AUDIT_JSON="${capture_audit}" /usr/bin/python3 - \
    "${staging_dir}/SOURCE_BAGS.json" <<'PY'
import json
import os
import sys

document = json.loads(os.environ["CAPTURE_AUDIT_JSON"])
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True,
              allow_nan=False)
    stream.write("\n")
PY
  freeze_solver_identity "${staging_dir}"

  say "preprocessing ${bag_count} scene(s) with the self-calibrated RGB model"
  ((bag_count >= 5)) || say "warning: upstream recommends 5-10 independently posed static bags; this is a pilot-sized set"
  printf '  camchain: %s\n' "${CAMCHAIN}"
  printf '  K:        %s\n' "${CAMERA_INTRINSICS}"
  printf '  D:        %s\n' "${CAMERA_DISTORTION}"

  DOCKER_ARGS=(
    run --rm
    --network host
    -e HOME=/tmp
    -e "HOST_UID=$(id -u)"
    -e "HOST_GID=$(id -g)"
    -v "${BAGS_DIR}:/tmp/input_bags:ro"
    -v "${staging_dir}:/tmp/preprocessed:rw"
    "${SOLVER_RUN_IMAGE}"
  )
  docker_run_owned \
    ros2 run direct_visual_lidar_calibration preprocess \
      /tmp/input_bags /tmp/preprocessed \
      --camera_info_topic "${CAMERA_INFO_TOPIC}" \
      --image_topic "${IMAGE_TOPIC}" \
      --points_topic "${POINTS_TOPIC}" \
      --intensity_channel intensity \
      --camera_model "${CAMERA_MODEL}" \
      --camera_intrinsics "${CAMERA_INTRINSICS}" \
      --camera_distortion_coeffs "${CAMERA_DISTORTION}"

  /usr/bin/python3 - "${BAGS_DIR}" "${staging_dir}" <<'PY' || \
    die "upstream preprocess did not produce a complete artifact set"
import json
import math
import sys
from pathlib import Path

bags = Path(sys.argv[1])
output = Path(sys.argv[2])
calib = output / "calib.json"
if not calib.is_file() or calib.stat().st_size == 0:
    raise SystemExit("calib.json is missing or empty")
with calib.open("r", encoding="utf-8") as stream:
    doc = json.load(stream, parse_constant=lambda token: (_ for _ in ()).throw(
        ValueError(f"non-finite JSON constant: {token}")))
expected = sorted(path.parent.name for path in bags.glob("*/metadata.yaml"))
reported = sorted(str(name) for name in doc.get("meta", {}).get("bag_names", []))
if reported != expected:
    raise SystemExit(f"calib.json bag_names mismatch: expected={expected}, reported={reported}")
for name in expected:
    for suffix in (".png", ".ply", "_lidar_indices.png", "_lidar_intensities.png"):
        path = output / f"{name}{suffix}"
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"missing/empty preprocess artifact: {path.name}")
print(f"verified {len(expected)} bag(s), calib.json and {4 * len(expected)} derived files")
PY
  sha256sum "${CAMCHAIN}" >"${staging_dir}/SOURCE_CAMCHAIN.sha256"
  mv -- "${staging_dir}" "${PREPROCESSED_DIR}"
  say "preprocessed data ready: ${PREPROCESSED_DIR}"
}

require_preprocessed() {
  [[ -s "${PREPROCESSED_DIR}/calib.json" ]] || die \
    "preprocessed calib.json is missing; run './calibrate_lidar_camera.sh preprocess' first"
}

require_result_transform() {
  local key="$1"
  /usr/bin/python3 - "${PREPROCESSED_DIR}/calib.json" "${key}" <<'PY' || \
    die "calib.json does not contain a valid results.${key}; complete the preceding solver step and save it"
import json
import math
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    doc = json.load(stream, parse_constant=lambda token: (_ for _ in ()).throw(
        ValueError(f"non-finite JSON constant: {token}")))
key = sys.argv[2]
value = doc.get("results", {}).get(key)
if (not isinstance(value, list) or len(value) != 7 or
        any(isinstance(item, bool) or not isinstance(item, (int, float)) or
            not math.isfinite(float(item)) for item in value)):
    raise SystemExit(f"results.{key} must be seven finite numbers [x,y,z,qx,qy,qz,qw]")
if math.sqrt(sum(float(item) ** 2 for item in value[3:])) <= 1e-12:
    raise SystemExit(f"results.{key} has a zero quaternion")
PY
}

run_initial() {
  require_docker_image
  require_preprocessed
  verify_frozen_capture_bags
  verify_frozen_solver_identity
  docker_gui_args
  say "opening upstream manual initial-guess UI"
  docker_run_owned \
    ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed
  require_result_transform init_T_lidar_camera
}

run_solve() {
  require_docker_image
  require_preprocessed
  verify_frozen_capture_bags
  verify_frozen_solver_identity
  require_result_transform init_T_lidar_camera
  # Upstream still initializes GLFW in --background mode, so it needs X11.
  docker_gui_args
  say "running upstream fine registration (background UI mode still uses X11/GLFW)"
  docker_run_owned \
    ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed \
      --background --auto_quit
  require_result_transform T_lidar_camera
  say "solver finished; inspect with './calibrate_lidar_camera.sh view'"
}

run_view() {
  require_docker_image
  require_preprocessed
  verify_frozen_capture_bags
  verify_frozen_solver_identity
  require_result_transform T_lidar_camera
  docker_gui_args
  say "opening upstream result viewer"
  docker_run_owned \
    ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed
}

resolve_import_lidar_serial() {
  local explicit="$1" from_manifests
  from_manifests="$(/usr/bin/python3 - "${BAGS_DIR}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
serials = set()
for path in sorted(root.glob("*/capture_manifest.json")):
    with path.open("r", encoding="utf-8") as stream:
        doc = json.load(stream)
    serial = str(doc.get("mid360s", {}).get("serial_observed", "")).strip()
    if serial:
        serials.add(serial)
if len(serials) > 1:
    raise SystemExit("capture manifests contain conflicting MID-360S serials: " + ", ".join(sorted(serials)))
print(next(iter(serials)) if serials else "")
PY
)" || die "failed to audit MID-360S serials in capture manifests"

  if [[ -n "${explicit}" && -n "${from_manifests}" && "${explicit}" != "${from_manifests}" ]]; then
    die "explicit LiDAR serial ${explicit} conflicts with bag manifests ${from_manifests}"
  fi
  if [[ -n "${from_manifests}" ]]; then
    printf '%s\n' "${from_manifests}"
  elif [[ -n "${explicit}" ]]; then
    printf '%s\n' "${explicit}"
  else
    die "capture manifests have no MID-360S serial; pass --lidar-serial SN or set MID360S_SERIAL"
  fi
}

run_import() {
  local rig_id="${LIDAR_CAMERA_RIG_ID:-${DEFAULT_RIG_ID}}"
  local lidar_serial="${MID360S_SERIAL:-}"
  local output="${LIDAR_CAMERA_DRAFT_OUTPUT:-${DEFAULT_DRAFT_OUTPUT}}"
  local force=0
  while (($#)); do
    case "$1" in
      --rig-id)
        (($# >= 2)) || die "--rig-id requires a value"
        rig_id="$2"
        shift
        ;;
      --lidar-serial)
        (($# >= 2)) || die "--lidar-serial requires a value"
        lidar_serial="$2"
        shift
        ;;
      --output)
        (($# >= 2)) || die "--output requires a value"
        output="$2"
        shift
        ;;
      --force) force=1 ;;
      *) die "unknown import option: $1" ;;
    esac
    shift
  done

  [[ -n "${rig_id//[[:space:]]/}" ]] || die "rig-id must not be empty"
  output="$(realpath -m -- "${output}")"
  [[ "${output}" != "$(realpath -m -- "${VALIDATED_OUTPUT}")" ]] || die \
    "import only creates a draft and may not target the canonical validated result: ${VALIDATED_OUTPUT}"
  [[ -s "${PREPROCESSED_DIR}/calib.json" ]] || die \
    "solver output is missing: ${PREPROCESSED_DIR}/calib.json"
  [[ -d "${BAGS_DIR}" ]] || die "calibration bags are missing: ${BAGS_DIR}"
  [[ -x "${PROJECT_DIR}/tools/import_lidar_camera_extrinsic.py" || \
     -r "${PROJECT_DIR}/tools/import_lidar_camera_extrinsic.py" ]] || die \
    "existing draft importer is missing: ${PROJECT_DIR}/tools/import_lidar_camera_extrinsic.py"
  require_docker_image
  load_camera_evidence
  verify_frozen_capture_bags
  local frozen_capture_identity
  frozen_capture_identity="$(/usr/bin/python3 - \
    "${PREPROCESSED_DIR}/SOURCE_BAGS.json" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as stream:
    doc = json.load(stream)
for key in ("rig_id", "mount_session_id", "d435i_serial", "mid360s_serial"):
    print(str(doc.get(key, "")).strip())
PY
)"
  local -a capture_identity_values=()
  mapfile -t capture_identity_values <<<"${frozen_capture_identity}"
  ((${#capture_identity_values[@]} == 4)) || die "invalid frozen capture identity"
  [[ "${rig_id}" == "${capture_identity_values[0]}" ]] || die \
    "--rig-id ${rig_id} conflicts with captured rig_id ${capture_identity_values[0]}"
  MOUNT_SESSION_ID="${capture_identity_values[1]}"
  [[ -n "${capture_identity_values[2]}" ]] || die \
    "frozen capture set has no D435i serial"
  [[ "${capture_identity_values[2]}" == "${EXPECTED_D435_SERIAL}" ]] || die \
    "frozen capture D435i ${capture_identity_values[2]} conflicts with factory ${EXPECTED_D435_SERIAL}"
  [[ -n "${capture_identity_values[3]}" ]] || die \
    "frozen capture set has no MID-360S serial"
  verify_frozen_solver_identity
  lidar_serial="$(resolve_import_lidar_serial "${lidar_serial}")"
  [[ "${lidar_serial}" == "${capture_identity_values[3]}" ]] || die \
    "MID-360S serial ${lidar_serial} conflicts with frozen capture ${capture_identity_values[3]}"

  local -a command=(
    /usr/bin/python3 "${PROJECT_DIR}/tools/import_lidar_camera_extrinsic.py"
    "${PREPROCESSED_DIR}/calib.json"
    --output "${output}"
    --rig-id "${rig_id}"
    --mount-session-id "${MOUNT_SESSION_ID}"
    --lidar-serial "${lidar_serial}"
    --camera-serial "${capture_identity_values[2]}"
    --source-data "${BAGS_DIR}"
    --source-data "${CAPTURE_SESSION_FILE}"
    --source-data "${PREPROCESSED_DIR}/SOURCE_BAGS.json"
    --source-data "${PREPROCESSED_DIR}/SOURCE_SOLVER.json"
    --source-data "${CAMCHAIN}"
    --solver-version "${FROZEN_SOLVER_DIGEST}"
    --solver-commit "${FROZEN_SOLVER_COMMIT}"
    --solver-command "ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed --background --auto_quit"
    --method-note "Imported by calibrate_lidar_camera.sh; draft only, independent validation still required."
  )
  ((force == 0)) || command+=(--force)

  say "importing upstream T_lidar_camera as a non-validated draft"
  printf '  rig:            %s\n' "${rig_id}"
  printf '  mount session:  %s\n' "${MOUNT_SESSION_ID}"
  printf '  MID-360S serial:%s\n' "${lidar_serial}"
  printf '  D435i serial:   %s (factory/capture matched)\n' "${capture_identity_values[2]}"
  printf '  source data:    %s\n' "${BAGS_DIR}"
  printf '  output:         %s\n' "${output}"
  "${command[@]}"
}

run_camera_publisher() {
  prepare_ros
  load_camera_evidence
  require_ros_package realsense2_camera "sudo apt install ros-jazzy-realsense2-camera"
  require_ros_package compressed_image_transport "sudo apt install ros-jazzy-image-transport-plugins"
  if [[ -n "$(topic_type "${RAW_IMAGE_TOPIC}")" ]]; then
    die "${RAW_IMAGE_TOPIC} is already published; refusing to start a second D435i driver"
  fi
  local launcher="${PROJECT_DIR}/start_d435_color.sh"
  [[ -x "${launcher}" ]] || die \
    "supervised D435 launcher not found or not executable: ${launcher}"
  say "starting supervised D435i ${EXPECTED_D435_SERIAL}: color ${EXPECTED_COLOR_PROFILE} only"
  exec env \
    D435I_CAMERA_NAMESPACE="${CAMERA_NAMESPACE}" \
    D435I_CAMERA_NAME="${CAMERA_NAME}" \
    D435I_EXPECTED_SERIAL="${EXPECTED_D435_SERIAL}" \
    D435I_USB_SERIAL="${D435I_USB_SERIAL:-${D435I_EXPECTED_USB_SERIAL:-${EXPECTED_D435_SERIAL}}}" \
    D435I_COLOR_PROFILE="${EXPECTED_COLOR_PROFILE}" \
    D435I_RAW_IMAGE_TOPIC="${RAW_IMAGE_TOPIC}" \
    "${launcher}"
}

run_camera_rgbd_publisher() {
  prepare_ros
  load_camera_evidence
  require_ros_package realsense2_camera \
    "sudo apt install ros-jazzy-realsense2-camera"
  require_ros_package compressed_image_transport \
    "sudo apt install ros-jazzy-image-transport-plugins"
  if [[ -n "$(topic_type "${RAW_IMAGE_TOPIC}")" ]]; then
    die "${RAW_IMAGE_TOPIC} is already published; refusing to start a second D435i driver"
  fi
  local launcher="${PROJECT_DIR}/start_d435_color.sh"
  [[ -x "${launcher}" ]] || die \
    "supervised D435 launcher not found or not executable: ${launcher}"
  say "starting the same supervised D435i ${EXPECTED_D435_SERIAL}: synchronized color+depth"
  exec env \
    D435I_CAMERA_NAMESPACE="${CAMERA_NAMESPACE}" \
    D435I_CAMERA_NAME="${CAMERA_NAME}" \
    D435I_EXPECTED_SERIAL="${EXPECTED_D435_SERIAL}" \
    D435I_USB_SERIAL="${D435I_USB_SERIAL:-${D435I_EXPECTED_USB_SERIAL:-${EXPECTED_D435_SERIAL}}}" \
    D435I_COLOR_PROFILE="${EXPECTED_COLOR_PROFILE}" \
    D435I_ENABLE_DEPTH=true \
    D435I_DEPTH_PROFILE="${EXPECTED_DEPTH_PROFILE}" \
    D435I_ENABLE_SYNC=true \
    D435I_RAW_IMAGE_TOPIC="${RAW_IMAGE_TOPIC}" \
    "${launcher}"
}

run_lidar_points_publisher() {
  prepare_ros
  require_ros_package livox_ros_driver2 \
    "build ${LIVOX_WS} first: cd ${LIVOX_WS} && colcon build --packages-select livox_ros_driver2 --symlink-install"
  [[ -r "${LIVOX_CONFIG}" ]] || die "MID-360S driver config not found: ${LIVOX_CONFIG}"

  local launcher="${LIVOX_WS}/start_mid360s.sh"
  [[ -x "${launcher}" ]] || die \
    "supervised MID-360S launcher not found or not executable: ${launcher}"

  local current_type
  current_type="$(topic_type "${POINTS_TOPIC}")"
  if [[ "${current_type}" == "sensor_msgs/msg/PointCloud2" ]]; then
    die "${POINTS_TOPIC} is already PointCloud2; refusing to start a second Livox driver"
  elif [[ -n "${current_type}" ]]; then
    die "${POINTS_TOPIC} is already published as ${current_type}. Stop that driver deliberately before switching formats."
  fi

  say "starting supervised MID-360S PointCloud2 publisher (xfer_format=0, 10 Hz)"
  exec env MID360S_TOPIC="${POINTS_TOPIC}" "${launcher}" --pointcloud2
}

main() {
  local command="${1:-help}"
  [[ $# -eq 0 ]] || shift
  case "${command}" in
    preflight)
      [[ $# -eq 0 ]] || die "preflight takes no arguments"
      hardware_preflight
      require_docker_image
      say "official solver image ready: ${DVLC_IMAGE}"
      ;;
    record)
      local scene="${1:-}" rigid_ack=0
      [[ $# -eq 0 ]] || shift
      while (($#)); do
        case "$1" in
          --rigid-mounted) rigid_ack=1 ;;
          *) die "unknown record option: $1" ;;
        esac
        shift
      done
      record_scene "${scene}" "${rigid_ack}"
      ;;
    record-rgbd)
      local scene="${1:-}" rigid_ack=0 role=""
      [[ $# -eq 0 ]] || shift
      while (($#)); do
        case "$1" in
          --rigid-mounted) rigid_ack=1 ;;
          --role)
            shift
            (($#)) || die "--role requires calibration or holdout"
            role="$1"
            ;;
          *) die "unknown record-rgbd option: $1" ;;
        esac
        shift
      done
      record_rgbd_scene "${scene}" "${role}" "${rigid_ack}"
      ;;
    preprocess)
      [[ $# -eq 0 ]] || die "preprocess takes no arguments"
      preprocess_bags
      ;;
    initial)
      [[ $# -eq 0 ]] || die "initial takes no arguments"
      run_initial
      ;;
    solve)
      [[ $# -eq 0 ]] || die "solve takes no arguments"
      run_solve
      ;;
    view)
      [[ $# -eq 0 ]] || die "view takes no arguments"
      run_view
      ;;
    import)
      run_import "$@"
      ;;
    camera)
      [[ $# -eq 0 ]] || die "camera takes no arguments"
      run_camera_publisher
      ;;
    camera-rgbd)
      [[ $# -eq 0 ]] || die "camera-rgbd takes no arguments"
      run_camera_rgbd_publisher
      ;;
    lidar-points)
      [[ $# -eq 0 ]] || die "lidar-points takes no arguments"
      run_lidar_points_publisher
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      usage >&2
      die "unknown command: ${command}"
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
