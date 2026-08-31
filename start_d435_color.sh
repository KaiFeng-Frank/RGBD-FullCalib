#!/usr/bin/env bash
# Supervised D435 ROS 2 publisher.  Color-only remains the safe default; the
# LiDAR-camera evidence path can explicitly add synchronized raw depth.  A
# missing device or dead color heartbeat is a hard failure, and every ROS child
# is contained in one disposable PGID.
set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly WATCHDOG="${PROJECT_DIR}/tools/ros_image_watchdog.py"

readonly CAMERA_NAMESPACE="${D435I_CAMERA_NAMESPACE:-camera}"
readonly CAMERA_NAME="${D435I_CAMERA_NAME:-camera}"
readonly EXPECTED_SERIAL="${D435I_EXPECTED_SERIAL:-}"
readonly EXPECTED_USB_SERIAL="${D435I_USB_SERIAL:-${D435I_EXPECTED_USB_SERIAL:-${EXPECTED_SERIAL}}}"
readonly COLOR_PROFILE="${D435I_COLOR_PROFILE:-1280x720x30}"
readonly ENABLE_DEPTH="${D435I_ENABLE_DEPTH:-false}"
readonly DEPTH_PROFILE="${D435I_DEPTH_PROFILE:-848x480x30}"
readonly ENABLE_SYNC="${D435I_ENABLE_SYNC:-false}"
readonly IMAGE_TOPIC="${D435I_RAW_IMAGE_TOPIC:-/${CAMERA_NAMESPACE#/}/${CAMERA_NAME#/}/color/image_raw}"
readonly STARTUP_TIMEOUT="${D435I_STARTUP_TIMEOUT_SECONDS:-20}"
readonly LOSS_TIMEOUT="${D435I_LOSS_TIMEOUT_SECONDS:-5}"
readonly DEVICE_LOSS_TIMEOUT="${D435I_DEVICE_LOSS_TIMEOUT_SECONDS:-3}"
readonly STOP_GRACE="${D435I_STOP_GRACE_SECONDS:-8}"

launch_pid=""
watchdog_pid=""

usage() {
  cat <<'EOF'
Usage: ./start_d435_color.sh

Environment:
  D435I_CAMERA_NAMESPACE       ROS camera namespace (default: camera)
  D435I_CAMERA_NAME            ROS camera name (default: camera)
  D435I_EXPECTED_SERIAL        required librealsense device serial
  D435I_USB_SERIAL             USB descriptor/ASIC serial used by unplug watchdog
                               (default: D435I_EXPECTED_SERIAL)
  D435I_COLOR_PROFILE          WIDTHxHEIGHTxFPS (default: 1280x720x30)
  D435I_ENABLE_DEPTH           true enables raw depth (default: false)
  D435I_DEPTH_PROFILE          WIDTHxHEIGHTxFPS (default: 848x480x30)
  D435I_ENABLE_SYNC            true synchronizes color/depth frames (default: false)
  D435I_RAW_IMAGE_TOPIC        watchdog topic (derived from namespace/name)

Optional supervision tuning:
  D435I_STARTUP_TIMEOUT_SECONDS  first valid Image deadline (default: 20)
  D435I_LOSS_TIMEOUT_SECONDS     active-stream deadline (default: 5)
  D435I_DEVICE_LOSS_TIMEOUT_SECONDS  USB absence deadline (default: 3)
  D435I_STOP_GRACE_SECONDS       TERM-to-KILL grace, integer s (default: 8)
EOF
}

if (($#)); then
  if [[ "$1" == "--help" && $# -eq 1 ]]; then
    usage
    exit 0
  fi
  usage >&2
  exit 2
fi

if [[ -z "${EXPECTED_SERIAL}" || "${EXPECTED_SERIAL}" == *$'\n'* \
  || "${EXPECTED_SERIAL}" == *$'\r'* ]]; then
  echo "D435I_EXPECTED_SERIAL is required and must be one non-empty line." >&2
  exit 2
fi
if [[ -z "${EXPECTED_USB_SERIAL}" || "${EXPECTED_USB_SERIAL}" == *$'\n'* \
  || "${EXPECTED_USB_SERIAL}" == *$'\r'* ]]; then
  echo "D435I_USB_SERIAL must be one non-empty line." >&2
  exit 2
fi
if [[ ! "${COLOR_PROFILE}" =~ ^[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*$ ]]; then
  echo "D435I_COLOR_PROFILE must be WIDTHxHEIGHTxFPS: ${COLOR_PROFILE}" >&2
  exit 2
fi
if [[ "${ENABLE_DEPTH}" != "true" && "${ENABLE_DEPTH}" != "false" ]]; then
  echo "D435I_ENABLE_DEPTH must be true or false: ${ENABLE_DEPTH}" >&2
  exit 2
fi
if [[ ! "${DEPTH_PROFILE}" =~ ^[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*$ ]]; then
  echo "D435I_DEPTH_PROFILE must be WIDTHxHEIGHTxFPS: ${DEPTH_PROFILE}" >&2
  exit 2
fi
if [[ "${ENABLE_SYNC}" != "true" && "${ENABLE_SYNC}" != "false" ]]; then
  echo "D435I_ENABLE_SYNC must be true or false: ${ENABLE_SYNC}" >&2
  exit 2
fi
if [[ "${IMAGE_TOPIC}" != /* || "${IMAGE_TOPIC}" =~ [[:space:]] ]]; then
  echo "D435I_RAW_IMAGE_TOPIC must be an absolute ROS topic without whitespace: ${IMAGE_TOPIC}" >&2
  exit 2
fi
if [[ ! "${STOP_GRACE}" =~ ^[1-9][0-9]*$ ]]; then
  echo "D435I_STOP_GRACE_SECONDS must be a positive integer: ${STOP_GRACE}" >&2
  exit 2
fi

d435_present() {
  local serial_file observed
  local -a serial_files=()

  shopt -s nullglob
  serial_files=(/sys/bus/usb/devices/*/serial)
  shopt -u nullglob
  for serial_file in "${serial_files[@]}"; do
    observed=""
    IFS= read -r observed <"${serial_file}" 2>/dev/null || continue
    if [[ "${observed}" == "${EXPECTED_USB_SERIAL}" ]]; then
      return 0
    fi
  done
  return 1
}

# Identity is checked before sourcing ROS or spawning any process.  If the
# expected camera is unplugged, this path cannot accidentally start a driver.
if ! d435_present; then
  echo "D435 is not connected: expected USB descriptor serial ${EXPECTED_USB_SERIAL}." >&2
  echo "No ROS process was started." >&2
  exit 10
fi

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ROS 2 Jazzy setup is missing: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${WATCHDOG}" ]]; then
  echo "D435 image watchdog is missing: ${WATCHDOG}" >&2
  exit 1
fi
if [[ ! -x /usr/bin/python3 ]]; then
  echo "/usr/bin/python3 is required." >&2
  exit 1
fi
if ! /usr/bin/python3 "${WATCHDOG}" --self-test \
  --expected-serial "${EXPECTED_USB_SERIAL}" \
  --startup-timeout "${STARTUP_TIMEOUT}" \
  --loss-timeout "${LOSS_TIMEOUT}" \
  --device-loss-timeout "${DEVICE_LOSS_TIMEOUT}" >/dev/null; then
  echo "Invalid D435 watchdog configuration; refusing to start ROS." >&2
  exit 2
fi

stop_pid() {
  local pid="$1" signal="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

stop_launch_group() {
  local signal="$1"
  [[ -n "${launch_pid}" ]] || return 0

  # The session leader PID is also the ROS process-group ID.  Signal the group
  # even if its original leader has exited but descendants are still alive.
  kill "-${signal}" -- "-${launch_pid}" 2>/dev/null \
    || { ! kill -0 "${launch_pid}" 2>/dev/null \
      || kill "-${signal}" "${launch_pid}" 2>/dev/null; } \
    || true
}

launch_group_alive() {
  [[ -n "${launch_pid}" ]] || return 1
  # kill -0 reports zombies as alive.  Ignore Z members so we reach wait/reap
  # immediately instead of burning the whole grace period on dead children.
  ps -eo pgid=,stat= 2>/dev/null \
    | awk -v pgid="${launch_pid}" \
        '$1 == pgid && $2 !~ /^Z/ { alive = 1 } END { exit !alive }'
}

pid_running() {
  local pid="$1" state=""
  [[ -n "${pid}" ]] || return 1
  read -r state < <(ps -o stat= -p "${pid}" 2>/dev/null) || return 1
  [[ -n "${state}" && "${state}" != Z* ]]
}

cleanup() {
  local exit_status=$? deadline
  # Do not restore default signal actions: a second terminal signal must not
  # abort cleanup between TERM and wait/reap.
  trap '' INT TERM HUP
  trap - EXIT

  # Background commands may inherit ignored SIGINT from Bash.  Cleanup always
  # uses TERM, so Ctrl-C/SIGTERM of this foreground supervisor remains reliable.
  stop_pid "${watchdog_pid}" TERM
  stop_launch_group TERM

  deadline=$((SECONDS + STOP_GRACE))
  while (( SECONDS < deadline )); do
    if ! launch_group_alive \
      && { [[ -z "${watchdog_pid}" ]] || ! pid_running "${watchdog_pid}"; }; then
      break
    fi
    sleep 0.1
  done

  stop_pid "${watchdog_pid}" KILL
  stop_launch_group KILL
  [[ -z "${watchdog_pid}" ]] || wait "${watchdog_pid}" 2>/dev/null || true
  [[ -z "${launch_pid}" ]] || wait "${launch_pid}" 2>/dev/null || true
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

# Keep Conda Python and shared libraries out of ROS 2 Jazzy processes.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CONDA_EXE _CONDA_ROOT
unset PYTHONHOME PYTHONPATH GSETTINGS_SCHEMA_DIR GSETTINGS_SCHEMA_DIR_CONDA_BACKUP

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI is unavailable after sourcing ${ROS_SETUP}." >&2
  exit 1
fi
if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  echo "ROS package realsense2_camera is missing." >&2
  exit 1
fi

# A pre-existing publisher could keep the watchdog healthy after this launch
# dies.  Query the graph before spawning either child and fail closed whenever
# graph discovery or publisher-count parsing is inconclusive.
graph_topics=""
if ! graph_topics="$(
  timeout 8s ros2 topic list --no-daemon --spin-time 2
)"; then
  echo "Cannot query the ROS graph; refusing to start D435." >&2
  exit 1
fi
if grep -Fxq -- "${IMAGE_TOPIC}" <<<"${graph_topics}"; then
  topic_info=""
  if ! topic_info="$(
    timeout 8s ros2 topic info --no-daemon --spin-time 2 -v "${IMAGE_TOPIC}"
  )"; then
    echo "Cannot inspect publishers on ${IMAGE_TOPIC}; refusing to start D435." >&2
    exit 1
  fi
  publisher_count=""
  if ! publisher_count="$(
    awk '/^Publisher count:[[:space:]]*[0-9]+[[:space:]]*$/ { print $3; found = 1 }
         END { if (!found) exit 1 }' <<<"${topic_info}"
  )" || [[ ! "${publisher_count}" =~ ^[0-9]+$ ]]; then
    echo "Cannot parse publisher count for ${IMAGE_TOPIC}; refusing to start D435." >&2
    exit 1
  fi
  if (( publisher_count > 0 )); then
    echo "${IMAGE_TOPIC} already has ${publisher_count} publisher(s); refusing to start a second D435 driver." >&2
    exit 11
  fi
fi

echo "Starting supervised D435 color publisher:" >&2
echo "  librealsense serial=${EXPECTED_SERIAL}, USB descriptor=${EXPECTED_USB_SERIAL}, profile=${COLOR_PROFILE}" >&2
echo "  depth=${ENABLE_DEPTH}, depth_profile=${DEPTH_PROFILE}, sync=${ENABLE_SYNC}" >&2
echo "  namespace=${CAMERA_NAMESPACE}, name=${CAMERA_NAME}" >&2
echo "  topic=${IMAGE_TOPIC}, startup=${STARTUP_TIMEOUT}s, loss=${LOSS_TIMEOUT}s" >&2
echo "  USB serial absence=${DEVICE_LOSS_TIMEOUT}s" >&2

launch_command=(
  ros2 launch realsense2_camera rs_launch.py
  "camera_namespace:=${CAMERA_NAMESPACE}"
  "camera_name:=${CAMERA_NAME}"
  "serial_no:='${EXPECTED_SERIAL}'"
  enable_color:=true
  "rgb_camera.color_profile:=${COLOR_PROFILE}"
  "enable_depth:=${ENABLE_DEPTH}"
  "depth_module.depth_profile:=${DEPTH_PROFILE}"
  "enable_sync:=${ENABLE_SYNC}"
  enable_infra:=false
  enable_infra1:=false
  enable_infra2:=false
  enable_gyro:=false
  enable_accel:=false
  enable_motion:=false
)

# setsid makes launch_pid both session leader and PGID, isolating this exact ROS
# tree from the caller's terminal and every unrelated ROS node.
setsid "${launch_command[@]}" &
launch_pid=$!

/usr/bin/python3 "${WATCHDOG}" \
  --topic "${IMAGE_TOPIC}" \
  --expected-serial "${EXPECTED_USB_SERIAL}" \
  --startup-timeout "${STARTUP_TIMEOUT}" \
  --loss-timeout "${LOSS_TIMEOUT}" \
  --device-loss-timeout "${DEVICE_LOSS_TIMEOUT}" &
watchdog_pid=$!

set +e
finished_pid=""
wait -n -p finished_pid "${launch_pid}" "${watchdog_pid}"
finished_status=$?
set -e

if [[ "${finished_pid}" == "${watchdog_pid}" ]]; then
  if (( finished_status == 20 )); then
    echo "D435 never produced a valid color frame; stopping its ROS launch group." >&2
  elif (( finished_status == 21 )); then
    echo "D435 color stream was lost; stopping its ROS launch group." >&2
  elif (( finished_status == 22 )); then
    echo "D435 USB device disappeared; stopping its ROS launch group." >&2
  else
    echo "D435 image watchdog exited (${finished_status}); stopping its ROS launch group." >&2
  fi
  exit "${finished_status}"
fi

if (( finished_status == 0 )); then
  echo "D435 ROS launch ended unexpectedly; stopping its watchdog." >&2
  exit 23
fi
echo "D435 ROS launch exited (${finished_status}); stopping its watchdog." >&2
exit "${finished_status}"
