#!/bin/bash
# Kalibr 容器封装。
#   - 把 d435i_calib/ 挂进容器,bag 和结果都在宿主机可见
#   - 转发 X11,让 Kalibr 的报告图能显示
#   - 用宿主机 UID 运行,结果文件不会变成 root 属主
#
# 用法: ./kalibr.sh rosrun kalibr kalibr_calibrate_cameras --bag ... 
#       ./kalibr.sh                      # 不带参数进交互 shell
set -e
HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker >/dev/null 2>&1 || echo "[warn] xhost 失败,报告窗口可能弹不出来"

ARGS=("$@")
if [ ${#ARGS[@]} -eq 0 ]; then ARGS=(bash); fi

TTYFLAG=(-i); [ -t 0 ] && TTYFLAG=(-i -t)

docker run "${TTYFLAG[@]}" --rm \
  --entrypoint bash \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOST_DIR":/data \
  -w /data \
  --user "$(id -u):$(id -g)" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  kalibr:noetic \
  -c "source /catkin_ws/devel/setup.bash && $(printf '%q ' "${ARGS[@]}")"
