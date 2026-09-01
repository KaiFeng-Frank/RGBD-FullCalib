#!/usr/bin/env bash
# End-to-end MID-360S IMU capture, solve, strict promotion, and viewer check.
set -euo pipefail

readonly CODE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RECORDER="${CODE_DIR}/tools/record_mid360s_imu_poses.py"
readonly ANALYZER="${CODE_DIR}/tools/calibrate_mid360s_imu_intrinsics.py"
readonly PROMOTER="${CODE_DIR}/tools/promote_mid360s_imu.py"

PROJECT_ROOT="${CODE_DIR}"
MANIFEST="data/lidar_camera_extrinsic/capture_session.json"
WORK_DIR="data/mid360s_imu_pipeline"
OUTPUT="results/mid360s_imu.json"
TOPIC="/livox/imu"
FRAME="livox_frame"
EXPECTED_SERIAL=""
EXPECTED_RIG_ID=""
EXPECTED_MOUNT_ID=""
FIT_POSES=12
HOLDOUT_POSES=3
LIVE_HOLD=0.5
BAG_HOLD=0.5
MIN_SAMPLES=60
MIN_SEP=18.0
LATITUDE=22.3
ALTITUDE=30.0
PYTHON_BIN=""
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
LIVOX_SETUP="${LIVOX_SETUP:-}"
INPUTS=()
VERIFY_EXISTING=0

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

say() {
  printf '[mid360s-imu] %s\n' "$*"
}

usage() {
  cat <<'EOF'
MID-360S IMU end-to-end calibration pipeline

Usage:
  ./calibrate_mid360s_imu.sh [options]
  ./calibrate_mid360s_imu.sh --inputs BAG_OR_NPZ [BAG_OR_NPZ ...] [options]

Without --inputs, the pipeline captures stable poses live from /livox/imu.
With --inputs, it non-interactively reuses existing rosbag2 directories or
capture NPZ files.  Both paths solve into a new analysis pair, recheck fixed
acceptance/observability/holdout gates, exclusively create
results/mid360s_imu.json, and require the viewer registry to report it done.

Options:
  --project-root DIR       Artifact root (default: directory of this script)
  --manifest PATH          Current-rig manifest relative to project root
  --work-dir DIR           Parent for a new exclusive run directory
  --output PATH            Must resolve to results/mid360s_imu.json
  --inputs PATH...         Existing rosbag2 directories and/or capture NPZs
  --input PATH             Repeatable single-input form used by ROS launch
  --topic TOPIC            Raw Livox IMU topic (default: /livox/imu)
  --frame FRAME            Required raw message frame (default: livox_frame)
  --serial SERIAL          Expected MID-360S serial; must match manifest
  --rig-id ID              Expected rig ID; must match manifest
  --mount-id ID            Expected mount session; must match manifest
  --fit-poses N            Minimum fit orientations (default: 12)
  --holdout-poses N        Independent holdout orientations (default: 3)
  --live-hold SEC          Live stable dwell (default: 0.5)
  --bag-hold SEC           rosbag2 segmentation dwell (default: 0.5)
  --min-samples N          Live stable-window sample floor (default: 60)
  --min-sep DEG            Minimum orientation separation (default: 18)
  --lat DEG --alt METRES   Gravity model location (default: 22.3, 30)
  --python PATH            Python interpreter with NumPy and ROS when needed
  --verify-existing        Re-solve and verify an existing formal result;
                           never write or replace it
  -h, --help               Show this help and exit

Safety:
  * The current-rig manifest is checked before capture or analysis.
  * Raw Livox acceleration is always interpreted as g and multiplied by
    exactly 9.80665 before the promoted correction model is applied.
  * Existing analysis, capture, and formal result files are never overwritten.
  * No hardware publisher or experiment process is started or stopped here.
EOF
}

need_value() {
  (($# >= 2)) || die "$1 requires a value"
}

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --project-root)
      need_value "$@"; PROJECT_ROOT="$2"; shift 2
      ;;
    --manifest)
      need_value "$@"; MANIFEST="$2"; shift 2
      ;;
    --work-dir)
      need_value "$@"; WORK_DIR="$2"; shift 2
      ;;
    --output)
      need_value "$@"; OUTPUT="$2"; shift 2
      ;;
    --topic)
      need_value "$@"; TOPIC="$2"; shift 2
      ;;
    --frame)
      need_value "$@"; FRAME="$2"; shift 2
      ;;
    --serial)
      need_value "$@"; EXPECTED_SERIAL="$2"; shift 2
      ;;
    --rig-id)
      need_value "$@"; EXPECTED_RIG_ID="$2"; shift 2
      ;;
    --mount-id)
      need_value "$@"; EXPECTED_MOUNT_ID="$2"; shift 2
      ;;
    --fit-poses)
      need_value "$@"; FIT_POSES="$2"; shift 2
      ;;
    --holdout-poses)
      need_value "$@"; HOLDOUT_POSES="$2"; shift 2
      ;;
    --live-hold)
      need_value "$@"; LIVE_HOLD="$2"; shift 2
      ;;
    --bag-hold)
      need_value "$@"; BAG_HOLD="$2"; shift 2
      ;;
    --min-samples)
      need_value "$@"; MIN_SAMPLES="$2"; shift 2
      ;;
    --min-sep)
      need_value "$@"; MIN_SEP="$2"; shift 2
      ;;
    --lat)
      need_value "$@"; LATITUDE="$2"; shift 2
      ;;
    --alt)
      need_value "$@"; ALTITUDE="$2"; shift 2
      ;;
    --python)
      need_value "$@"; PYTHON_BIN="$2"; shift 2
      ;;
    --verify-existing)
      VERIFY_EXISTING=1; shift
      ;;
    --inputs)
      shift
      ((${#INPUTS[@]} == 0)) || die "--inputs may be supplied only once"
      while (($#)) && [[ "$1" != --* ]]; do
        INPUTS+=("$1")
        shift
      done
      ((${#INPUTS[@]} > 0)) || die "--inputs requires at least one path"
      ;;
    --input)
      need_value "$@"
      INPUTS+=("$2")
      shift 2
      ;;
    --)
      shift
      (($# == 0)) || die "unexpected positional arguments after --"
      ;;
    *)
      die "unknown argument: $1 (use --help)"
      ;;
  esac
done

[[ -r "${RECORDER}" && -r "${ANALYZER}" && -r "${PROMOTER}" ]] || \
  die "pipeline tools are missing beside ${CODE_DIR}"
[[ -n "${TOPIC}" && -n "${FRAME}" ]] || die "--topic and --frame must be non-empty"

if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x "${CODE_DIR}/.venv_ros/bin/python" ]]; then
    PYTHON_BIN="${CODE_DIR}/.venv_ros/bin/python"
  else
    PYTHON_BIN="/usr/bin/python3"
  fi
fi
[[ -x "${PYTHON_BIN}" ]] || die "Python interpreter is not executable: ${PYTHON_BIN}"
"${PYTHON_BIN}" -B -c 'import numpy' >/dev/null 2>&1 || \
  die "${PYTHON_BIN} cannot import NumPy"

PROJECT_ROOT="$(${PYTHON_BIN} -B - "${PROJECT_ROOT}" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
if not root.is_dir():
    raise SystemExit(f"project root is not a directory: {root}")
print(root)
PY
)" || die "invalid --project-root"

resolve_inside_root() {
  "${PYTHON_BIN}" -B - "${PROJECT_ROOT}" "$1" "$2" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
raw = Path(sys.argv[2])
label = sys.argv[3]
path = (raw if raw.is_absolute() else root / raw).resolve()
try:
    relative = path.relative_to(root)
except ValueError:
    raise SystemExit(f"{label} must stay inside project root: {raw}")
print(path)
print(relative.as_posix())
PY
}

declare -a manifest_paths=()
mapfile -t manifest_paths < <(resolve_inside_root "${MANIFEST}" "manifest")
((${#manifest_paths[@]} == 2)) || die "cannot resolve manifest path"
[[ -r "${manifest_paths[0]}" ]] || die "manifest is not readable: ${manifest_paths[0]}"

declare -a output_paths=()
mapfile -t output_paths < <(resolve_inside_root "${OUTPUT}" "output")
((${#output_paths[@]} == 2)) || die "cannot resolve output path"
[[ "${output_paths[1]}" == "results/mid360s_imu.json" ]] || \
  die "formal output must be results/mid360s_imu.json under project root"
if ((VERIFY_EXISTING)); then
  [[ -f "${output_paths[0]}" ]] || \
    die "--verify-existing requires an existing formal result: ${output_paths[0]}"
else
  [[ ! -e "${output_paths[0]}" ]] || \
    die "refusing to overwrite existing formal result: ${output_paths[0]}"
fi

identity_args=()
[[ -z "${EXPECTED_SERIAL}" ]] || identity_args+=(--expected-serial "${EXPECTED_SERIAL}")
[[ -z "${EXPECTED_RIG_ID}" ]] || identity_args+=(--expected-rig-id "${EXPECTED_RIG_ID}")
[[ -z "${EXPECTED_MOUNT_ID}" ]] || identity_args+=(--expected-mount-id "${EXPECTED_MOUNT_ID}")
preflight_text="$(
  "${PYTHON_BIN}" -B "${PROMOTER}" \
    --preflight --project-root "${PROJECT_ROOT}" --manifest "${manifest_paths[1]}" \
    "${identity_args[@]}"
)" || die "current-rig manifest/identity preflight failed"
declare -a identity=()
mapfile -t identity <<<"${preflight_text}"
((${#identity[@]} == 4)) || die "unexpected identity preflight output"
readonly RIG_ID="${identity[0]}"
readonly MOUNT_ID="${identity[1]}"
readonly MID360S_SERIAL="${identity[2]}"
say "preflight OK: serial=${MID360S_SERIAL} rig=${RIG_ID} mount=${MOUNT_ID}"

needs_ros=0
if ((${#INPUTS[@]} == 0)); then
  needs_ros=1
else
  for input in "${INPUTS[@]}"; do
    candidate="${input}"
    [[ "${candidate}" == /* ]] || candidate="${PROJECT_ROOT}/${candidate}"
    if [[ -d "${candidate}" ]]; then
      needs_ros=1
    fi
  done
fi

if ((needs_ros)); then
  if [[ -f "${ROS_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
    if [[ -n "${LIVOX_SETUP}" && -f "${LIVOX_SETUP}" ]]; then
      # shellcheck disable=SC1090
      source "${LIVOX_SETUP}"
    fi
    set -u
  fi
  "${PYTHON_BIN}" -B -c 'import rclpy, rosbag2_py, sensor_msgs' >/dev/null 2>&1 || \
    die "ROS bag/live mode requires rclpy, rosbag2_py, and sensor_msgs in ${PYTHON_BIN}"
fi

declare -a work_paths=()
mapfile -t work_paths < <(resolve_inside_root "${WORK_DIR}" "work directory")
((${#work_paths[@]} == 2)) || die "cannot resolve work directory"
mkdir -p -- "${work_paths[0]}"
run_prefix="${work_paths[0]}/run_$(date -u +%Y%m%dT%H%M%SZ)_"
RUN_DIR="$(mktemp -d "${run_prefix}XXXXXX")"
RUN_REL="${RUN_DIR#"${PROJECT_ROOT}/"}"
[[ "${RUN_REL}" != "${RUN_DIR}" ]] || die "run directory escaped project root"
say "run directory: ${RUN_REL}"

declare -a normalized_inputs=()
if ((${#INPUTS[@]} == 0)); then
  command -v ros2 >/dev/null 2>&1 || die "ros2 CLI is unavailable for live capture"
  observed_type="$(timeout 4s ros2 topic type "${TOPIC}" 2>/dev/null | head -n 1 || true)"
  [[ "${observed_type}" == "sensor_msgs/msg/Imu" ]] || \
    die "${TOPIC} must be published as sensor_msgs/msg/Imu; observed ${observed_type:-nothing}"
  capture_rel="${RUN_REL}/stable_poses.npz"
  say "capturing live stable poses from ${TOPIC} (${FRAME})"
  "${PYTHON_BIN}" -B "${RECORDER}" \
    --out "${PROJECT_ROOT}/${capture_rel}" \
    --topic "${TOPIC}" --frame "${FRAME}" \
    --mid360s-serial "${MID360S_SERIAL}" --rig-id "${RIG_ID}" --mount-id "${MOUNT_ID}" \
    --fit-poses "${FIT_POSES}" --holdout-poses "${HOLDOUT_POSES}" \
    --hold "${LIVE_HOLD}" --min-samples "${MIN_SAMPLES}" --min-sep "${MIN_SEP}"
  normalized_inputs+=("${capture_rel}")
else
  normalized_text="$(${PYTHON_BIN} -B - "${PROJECT_ROOT}" "${INPUTS[@]}" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
for value in sys.argv[2:]:
    raw = Path(value)
    path = (raw if raw.is_absolute() else root / raw).resolve()
    try:
        relative = path.relative_to(root)
    except ValueError:
        raise SystemExit(f"input must stay inside project root: {value}")
    if not path.exists():
        raise SystemExit(f"input does not exist: {path}")
    if path.is_symlink():
        raise SystemExit(f"input must not be a symlink: {path}")
    if not path.is_dir() and path.suffix.lower() != '.npz':
        raise SystemExit(f"input must be a rosbag2 directory or capture NPZ: {path}")
    print(relative.as_posix())
PY
)" || die "input normalization failed"
  mapfile -t normalized_inputs <<<"${normalized_text}"
  ((${#normalized_inputs[@]} == ${#INPUTS[@]})) || die "input normalization count mismatch"
fi

analysis_rel="${RUN_REL}/analysis.json"
analysis_npz_rel="${RUN_REL}/analysis.npz"
say "solving ${#normalized_inputs[@]} capture input(s); raw g is multiplied by exactly 9.80665"
(
  cd -- "${PROJECT_ROOT}"
  "${PYTHON_BIN}" -B "${ANALYZER}" "${normalized_inputs[@]}" \
    --out-json "${analysis_rel}" --out-npz "${analysis_npz_rel}" \
    --topic "${TOPIC}" --frame "${FRAME}" \
    --mid360s-serial "${MID360S_SERIAL}" --rig-id "${RIG_ID}" --mount-id "${MOUNT_ID}" \
    --minimum-fit-poses "${FIT_POSES}" --holdout-poses "${HOLDOUT_POSES}" \
    --hold "${BAG_HOLD}" --min-sep "${MIN_SEP}" \
    --lat "${LATITUDE}" --alt "${ALTITUDE}"
)

say "analysis accepted; entering strict promotion boundary"
promotion_mode=()
((VERIFY_EXISTING == 0)) || promotion_mode+=(--verify-existing)
(
  cd -- "${PROJECT_ROOT}"
  "${PYTHON_BIN}" -B "${PROMOTER}" \
    --project-root . --manifest "${manifest_paths[1]}" \
    --analysis "${analysis_rel}" --analysis-npz "${analysis_npz_rel}" \
    --output "${output_paths[1]}" --expected-frame "${FRAME}" \
    --expected-serial "${MID360S_SERIAL}" --expected-rig-id "${RIG_ID}" \
    --expected-mount-id "${MOUNT_ID}" "${promotion_mode[@]}"
)
if ((VERIFY_EXISTING)); then
  say "verified existing: ${output_paths[1]}; viewer summary mid360s_imu=done"
else
  say "complete: ${output_paths[1]}; viewer summary mid360s_imu=done"
fi
