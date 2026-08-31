#!/usr/bin/env bash
# Launch the MID-360S health acquisition tool in a clean ROS 2 Jazzy runtime.
set -euo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly TOOL="${PROJECT_DIR}/tools/mid360_health.py"
readonly ROS_VENV="${PROJECT_DIR}/.venv_ros"
readonly USER_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"

strip_conda_paths() {
  local source_value="$1" entry result=""
  local -a entries=()
  IFS=':' read -r -a entries <<<"${source_value}"
  for entry in "${entries[@]}"; do
    [[ -z "${entry}" ]] && continue
    case "${entry}" in
      */miniconda*|*/anaconda*|*/conda/envs/*) continue ;;
    esac
    result="${result:+${result}:}${entry}"
  done
  printf '%s' "${result}"
}

# Do not let an active Conda Python or its shared libraries leak into Jazzy.
PATH="$(strip_conda_paths "${PATH:-}")"
export PATH
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="$(strip_conda_paths "${LD_LIBRARY_PATH}")"
  export LD_LIBRARY_PATH
fi
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CONDA_EXE _CONDA_ROOT
unset GSETTINGS_SCHEMA_DIR GSETTINGS_SCHEMA_DIR_CONDA_BACKUP
unset PYTHONHOME PYTHONPATH

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

readonly LIVOX_WS="${LIVOX_WS:-${USER_DIR}/livox_mid360s_ws}"
if [[ ! -f "${LIVOX_WS}/install/setup.bash" ]]; then
  echo "Livox overlay not found: ${LIVOX_WS}/install/setup.bash" >&2
  echo "Set LIVOX_WS to the built livox_ros_driver2 workspace." >&2
  exit 2
fi
set +u
# shellcheck disable=SC1090
source "${LIVOX_WS}/install/setup.bash"
set -u

if [[ ! -x "${ROS_VENV}/bin/python" ]]; then
  echo "[setup] Creating ${ROS_VENV} with ROS system packages..." >&2
  /usr/bin/python3 -m venv --system-site-packages "${ROS_VENV}"
fi

if ! "${ROS_VENV}/bin/python" -c \
  'import rclpy, sensor_msgs, std_msgs, livox_ros_driver2' >/dev/null 2>&1; then
  echo "${ROS_VENV}/bin/python cannot import Jazzy/Livox modules after sourcing overlays." >&2
  echo "Recreate it with: /usr/bin/python3 -m venv --system-site-packages ${ROS_VENV}" >&2
  exit 2
fi

exec "${ROS_VENV}/bin/python" "${TOOL}" "$@"
