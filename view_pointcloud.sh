#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly VIEWER_DIR="${PROJECT_DIR}/viewer"
readonly ROS_VENV="${PROJECT_DIR}/.venv_ros"
readonly USER_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"
readonly ORIGINAL_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"

mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
driver_pid=""
server_pid=""
aux_server_pid=""

usage() {
  cat <<'EOF'
Usage:
  ./view_pointcloud.sh fused [--extrinsic PATH] [--timesync PATH] [--no-deskew]
  ./view_pointcloud.sh dual [--extrinsic PATH]   # backward-compatible alias
  ./view_pointcloud.sh mid360 [server options...]
  ./view_pointcloud.sh ros2 [topic|auto] [server options...]
  ./view_pointcloud.sh d435i [server options...]
  ./view_pointcloud.sh synthetic [server options...]
  ./view_pointcloud.sh synthetic-points [server options...]

Examples:
  ./view_pointcloud.sh fused
  ./view_pointcloud.sh fused --extrinsic results/mid360s_d435i_extrinsic.local.json
  ./view_pointcloud.sh mid360
  ./view_pointcloud.sh ros2 /camera/depth/color/points --max-points 150000

Set POINTCLOUD_NO_BROWSER=1 to suppress automatic browser opening.
EOF
}

select_python() {
  local import_check="$1"
  shift
  local candidate
  for candidate in "$@"; do
    if [[ -n "${candidate}" && -x "${candidate}" ]] &&
       "${candidate}" -c "${import_check}" >/dev/null 2>&1; then
      SELECTED_PYTHON="${candidate}"
      return 0
    fi
  done
  return 1
}

stop_process() {
  local pid="$1" signal_name="$2" max_checks="${3:-30}" state
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return
  fi
  kill "-${signal_name}" "${pid}" 2>/dev/null || true
  for ((check_i=0; check_i<max_checks; check_i++)); do
    state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d ' ')" || state=""
    if [[ -z "${state}" || "${state}" == Z* ]]; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local server_signal="${1:-TERM}"
  if [[ -n "${aux_server_pid}" ]]; then
    stop_process "${aux_server_pid}" "${server_signal}"
    aux_server_pid=""
  fi
  if [[ -n "${server_pid}" ]]; then
    stop_process "${server_pid}" "${server_signal}"
    server_pid=""
  fi
  if [[ -n "${driver_pid}" ]]; then
    # A non-interactive Bash background job may inherit SIGINT=ignored.  TERM
    # reliably enters start_mid360s.sh's cleanup trap.  Give that wrapper more
    # than its own 8 s TERM-to-KILL grace period before forcing the wrapper.
    stop_process "${driver_pid}" TERM 120
    driver_pid=""
  fi
}

handle_signal() {
  local signal_name="$1" exit_code="$2"
  trap - INT TERM
  cleanup "${signal_name}"
  exit "${exit_code}"
}

trap 'cleanup TERM' EXIT
trap 'handle_signal INT 130' INT
trap 'handle_signal TERM 143' TERM

prepare_ros_python() {
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
  unset CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CONDA_EXE _CONDA_ROOT
  unset GSETTINGS_SCHEMA_DIR GSETTINGS_SCHEMA_DIR_CONDA_BACKUP
  unset PYTHONHOME PYTHONPATH

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy not found at /opt/ros/jazzy." >&2
    exit 2
  fi
  # ROS-generated setup files read optional variables without ${var:-}; suspend
  # nounset only while sourcing them.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u

  local livox_ws="${LIVOX_WS:-${USER_DIR}/livox_mid360s_ws}"
  if [[ -f "${livox_ws}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${livox_ws}/install/setup.bash"
    set -u
  fi

  if [[ ! -x "${ROS_VENV}/bin/python" ]]; then
    echo "[setup] Creating ROS-compatible Python 3.12 environment..."
    /usr/bin/python3 -m venv --system-site-packages "${ROS_VENV}"
  fi
  if ! "${ROS_VENV}/bin/python" -c \
      'import websockets; assert int(websockets.__version__.split(".")[0]) >= 12' \
      >/dev/null 2>&1; then
    "${ROS_VENV}/bin/python" -m pip install --disable-pip-version-check \
      --upgrade 'websockets>=12'
  fi
  "${ROS_VENV}/bin/python" -c \
    'import rclpy, sensor_msgs, numpy, websockets' >/dev/null
  ROS_PYTHON="${ROS_VENV}/bin/python"
}

select_d435_python() {
  local active_python active_python3
  active_python="$(command -v python 2>/dev/null || true)"
  active_python3="$(command -v python3 2>/dev/null || true)"
  if ! select_python 'import numpy, cv2, pyrealsense2, websockets; assert int(websockets.__version__.split(".")[0]) >= 12' \
      "${D435I_PYTHON:-}" "${POINTCLOUD_PYTHON:-}" \
      "${active_python}" \
      "${USER_DIR}/miniconda3/envs/d435i-calib/bin/python" \
      "${USER_DIR}/anaconda3/envs/d435i-calib/bin/python" \
      "${active_python3}"; then
    echo "No Python environment with numpy/cv2/pyrealsense2/websockets was found." >&2
    echo "Activate d435i-calib or set D435I_PYTHON=/path/to/python." >&2
    exit 2
  fi
  D435_PYTHON="${SELECTED_PYTHON}"
}

ensure_mid360_driver() {
  local livox_dir topic_list="" node_list="" graph_query_ok=0
  livox_dir="${LIVOX_WS:-${USER_DIR}/livox_mid360s_ws}"
  if topic_list="$(timeout 2s ros2 topic list 2>/dev/null)"; then
    graph_query_ok=1
  fi
  if node_list="$(timeout 2s ros2 node list 2>/dev/null)"; then
    graph_query_ok=1
  fi
  if grep -Fxq '/livox/lidar' <<<"${topic_list}" ||
     grep -Fxq '/livox_lidar_publisher' <<<"${node_list}" ||
     pgrep -f '[l]ivox_ros_driver2_node' >/dev/null; then
    echo "[MID-360S] Reusing the existing /livox/lidar publisher."
  elif ((graph_query_ok == 0)); then
    echo "ROS graph queries failed; refusing to guess and start a second MID-360S driver." >&2
    echo "Check 'ros2 topic list' and retry." >&2
    exit 2
  else
    if [[ ! -x "${livox_dir}/start_mid360s.sh" ]]; then
      echo "MID-360S launcher not found: ${livox_dir}/start_mid360s.sh" >&2
      exit 2
    fi
    echo "[MID-360S] Starting driver..."
    "${livox_dir}/start_mid360s.sh" &
    driver_pid="$!"
  fi
}

open_browser_later() {
  if [[ "${POINTCLOUD_NO_BROWSER:-0}" == "1" ]] || ! command -v xdg-open >/dev/null; then
    return
  fi
  (
    for _ in {1..40}; do
      if curl -fsS "http://localhost:${HTTP_PORT}/" >/dev/null 2>&1; then
        xdg-open "${BROWSER_URL}" >/dev/null 2>&1 || true
        exit 0
      fi
      sleep 0.25
    done
  ) &
}

case "${mode}" in
  fused|dual)
    dual_extrinsic_args=()
    dual_lidar_args=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --extrinsic)
          if [[ $# -lt 2 || -z "$2" ]]; then
            echo "--extrinsic requires an explicit JSON path." >&2
            exit 2
          fi
          if [[ ! -f "$2" ]]; then
            echo "Local extrinsic does not exist: $2" >&2
            exit 2
          fi
          extrinsic_path="$(cd -- "$(dirname -- "$2")" && pwd)/$(basename -- "$2")"
          dual_extrinsic_args=(--extrinsic "${extrinsic_path}")
          shift 2
          ;;
        --extrinsic=*)
          extrinsic_value="${1#--extrinsic=}"
          if [[ -z "${extrinsic_value}" || ! -f "${extrinsic_value}" ]]; then
            echo "Local extrinsic does not exist: ${extrinsic_value}" >&2
            exit 2
          fi
          extrinsic_path="$(cd -- "$(dirname -- "${extrinsic_value}")" && pwd)/$(basename -- "${extrinsic_value}")"
          dual_extrinsic_args=(--extrinsic "${extrinsic_path}")
          shift
          ;;
        --extrinsic-draft)
          if [[ $# -lt 2 || -z "$2" ]]; then
            echo "--extrinsic-draft requires an explicit JSON path." >&2
            exit 2
          fi
          if [[ ! -f "$2" ]]; then
            echo "Extrinsic draft does not exist: $2" >&2
            exit 2
          fi
          draft_path="$(cd -- "$(dirname -- "$2")" && pwd)/$(basename -- "$2")"
          dual_extrinsic_args=(--extrinsic-draft "${draft_path}")
          shift 2
          ;;
        --extrinsic-draft=*)
          draft_value="${1#--extrinsic-draft=}"
          if [[ -z "${draft_value}" || ! -f "${draft_value}" ]]; then
            echo "Extrinsic draft does not exist: ${draft_value}" >&2
            exit 2
          fi
          draft_path="$(cd -- "$(dirname -- "${draft_value}")" && pwd)/$(basename -- "${draft_value}")"
          dual_extrinsic_args=(--extrinsic-draft "${draft_path}")
          shift
          ;;
        --timesync)
          if [[ $# -lt 2 || -z "$2" ]]; then
            echo "--timesync requires an explicit JSON path." >&2
            exit 2
          fi
          if [[ ! -f "$2" ]]; then
            echo "Timesync result does not exist: $2" >&2
            exit 2
          fi
          timesync_path="$(cd -- "$(dirname -- "$2")" && pwd)/$(basename -- "$2")"
          dual_lidar_args+=(--timesync "${timesync_path}")
          shift 2
          ;;
        --timesync=*)
          timesync_value="${1#--timesync=}"
          if [[ -z "${timesync_value}" || ! -f "${timesync_value}" ]]; then
            echo "Timesync result does not exist: ${timesync_value}" >&2
            exit 2
          fi
          timesync_path="$(cd -- "$(dirname -- "${timesync_value}")" && pwd)/$(basename -- "${timesync_value}")"
          dual_lidar_args+=(--timesync "${timesync_path}")
          shift
          ;;
        --no-deskew)
          dual_lidar_args+=(--no-deskew)
          shift
          ;;
        *)
          echo "Unknown fused-view option: $1" >&2
          echo "Use RGBD_WS_PORT/LIDAR_WS_PORT/DUAL_HTTP_PORT for port overrides." >&2
          exit 2
          ;;
      esac
    done
    # The fused view never falls back to two unrelated coordinate windows.
    # Prefer this rig's explicit LOCAL result when the caller did not provide
    # another transform; a canonical validated result remains the fallback.
    if ((${#dual_extrinsic_args[@]} == 0)); then
      default_local_extrinsic="${PROJECT_DIR}/results/mid360s_d435i_extrinsic.local.json"
      canonical_extrinsic="${PROJECT_DIR}/results/mid360s_d435i_extrinsic.json"
      if [[ -f "${default_local_extrinsic}" ]]; then
        dual_extrinsic_args=(--extrinsic "${default_local_extrinsic}")
      elif [[ ! -f "${canonical_extrinsic}" ]]; then
        echo "Fused view needs a LiDAR-camera extrinsic." >&2
        echo "Pass --extrinsic PATH or create results/mid360s_d435i_extrinsic.local.json." >&2
        exit 2
      fi
    fi
    # Pick the camera interpreter before sourcing ROS.  The two runtimes remain
    # separate processes because pyrealsense2/Conda and ROS Jazzy/rclpy target
    # different Python ABIs on this workstation.
    select_d435_python
    prepare_ros_python
    LIDAR_PYTHON="${ROS_PYTHON}"
    ensure_mid360_driver
    dual_mode=1
    ;;
  mid360)
    prepare_ros_python
    ensure_mid360_driver
    server_args=(--source ros2 --topic /livox/lidar "$@")
    ;;
  ros2)
    prepare_ros_python
    topic="auto"
    if [[ $# -gt 0 && "${1}" != --* ]]; then
      topic="$1"; shift
    fi
    server_args=(--source ros2 --topic "${topic}" "$@")
    ;;
  d435i)
    select_d435_python
    ROS_PYTHON="${D435_PYTHON}"
    server_args=(--source d435i "$@")
    ;;
  synthetic|synthetic-points)
    active_python="$(command -v python 2>/dev/null || true)"
    active_python3="$(command -v python3 2>/dev/null || true)"
    if ! select_python 'import numpy, cv2, websockets; assert int(websockets.__version__.split(".")[0]) >= 12' \
        "${POINTCLOUD_PYTHON:-}" "${active_python}" \
        "${USER_DIR}/miniconda3/envs/d435i-calib/bin/python" \
        "${USER_DIR}/anaconda3/envs/d435i-calib/bin/python" \
        "${active_python3}"; then
      echo "No Python environment with numpy/cv2/websockets was found." >&2
      echo "Activate d435i-calib or set POINTCLOUD_PYTHON=/path/to/python." >&2
      exit 2
    fi
    ROS_PYTHON="${SELECTED_PYTHON}"
    server_args=(--source "${mode}" "$@")
    ;;
  help|-h|--help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown mode: ${mode}" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ "${dual_mode:-0}" == "1" ]]; then
  HTTP_PORT="${DUAL_HTTP_PORT:-8080}"
  RGBD_PORT="${RGBD_WS_PORT:-9002}"
  LIDAR_PORT="${LIDAR_WS_PORT:-9003}"
  for port_value in "${HTTP_PORT}" "${RGBD_PORT}" "${LIDAR_PORT}"; do
    if [[ ! "${port_value}" =~ ^[0-9]+$ ]] || ((port_value < 1 || port_value > 65535)); then
      echo "Invalid fused-view port: ${port_value}" >&2
      exit 2
    fi
  done
  if [[ "${HTTP_PORT}" == "${RGBD_PORT}" || "${HTTP_PORT}" == "${LIDAR_PORT}" ||
        "${RGBD_PORT}" == "${LIDAR_PORT}" ]]; then
    echo "Fused-view HTTP/RGB-D/LiDAR ports must be distinct." >&2
    exit 2
  fi
  # Open the overlay renderer itself as the top-level page.  There is no outer
  # workbench and no pair of embedded viewers: one WebGL canvas consumes both
  # streams in camera_color_optical_frame.  The version tag also prevents an
  # old dual-layout tab from being reused by the browser.
  BROWSER_URL="http://localhost:${HTTP_PORT}/?overlay=1&role=overlay&rgbd_ws=${RGBD_PORT}&lidar_ws=${LIDAR_PORT}&ui=fused-v4"
  open_browser_later
  cd "${VIEWER_DIR}"
  echo "[RGB-D] Python: ${D435_PYTHON}  ws=${RGBD_PORT}"
  env -u PYTHONHOME -u PYTHONPATH LD_LIBRARY_PATH="${ORIGINAL_LD_LIBRARY_PATH}" \
    "${D435_PYTHON}" server.py --source d435i \
      --http "${HTTP_PORT}" --ws "${RGBD_PORT}" \
      --align --overlay-role rgbd "${dual_extrinsic_args[@]}" \
      --max-fps "${RGBD_MAX_FPS:-15}" &
  server_pid="$!"
  echo "[LiDAR] Python: ${LIDAR_PYTHON}  ws=${LIDAR_PORT}"
  "${LIDAR_PYTHON}" server.py --source ros2 --topic /livox/lidar \
    --no-http --ws "${LIDAR_PORT}" --max-fps "${LIDAR_MAX_FPS:-10}" \
    --max-points "${LIDAR_MAX_POINTS:-250000}" --overlay-role lidar \
    "${dual_extrinsic_args[@]}" "${dual_lidar_args[@]}" &
  aux_server_pid="$!"
  set +e
  # The workbench is only healthy while both backends are alive. Exit on the
  # first backend failure; when this script owns the Livox driver, its watchdog
  # is part of the same health boundary.  The EXIT trap stops every survivor.
  if [[ -n "${driver_pid}" ]]; then
    wait -n "${server_pid}" "${aux_server_pid}" "${driver_pid}"
  else
    wait -n "${server_pid}" "${aux_server_pid}"
  fi
  server_status="$?"
  set -e
  exit "${server_status}"
fi

HTTP_PORT=8080
WS_PORT=9002
for ((arg_i=0; arg_i<${#server_args[@]}; arg_i++)); do
  case "${server_args[arg_i]}" in
    --http)
      if ((arg_i+1<${#server_args[@]})); then HTTP_PORT="${server_args[arg_i+1]}"; fi
      ;;
    --http=*) HTTP_PORT="${server_args[arg_i]#--http=}" ;;
    --ws)
      if ((arg_i+1<${#server_args[@]})); then WS_PORT="${server_args[arg_i+1]}"; fi
      ;;
    --ws=*) WS_PORT="${server_args[arg_i]#--ws=}" ;;
  esac
done

BROWSER_URL="http://localhost:${HTTP_PORT}/?ws=${WS_PORT}"
open_browser_later
cd "${VIEWER_DIR}"
echo "[viewer] Python: ${ROS_PYTHON}"
"${ROS_PYTHON}" server.py "${server_args[@]}" &
server_pid="$!"
set +e
if [[ -n "${driver_pid}" ]]; then
  # In MID-360S mode, a driver/watchdog exit tears down the viewer immediately;
  # a viewer exit likewise tears down the driver through the EXIT trap.
  wait -n "${server_pid}" "${driver_pid}"
else
  wait "${server_pid}"
fi
server_status="$?"
set -e
# Keep server_pid until EXIT cleanup.  If the owned MID-360S watchdog was the
# process that finished first, the server is still alive and must be stopped;
# if the server itself finished, stop_process simply reaps/no-ops safely.
exit "${server_status}"
