#!/bin/bash
# Kalibr cam-IMU 外参标定封装。
#
# 关键:--cam 传的是阶段1 已标定好的 camchain,Kalibr 会把相机内参当已知量,
# 只解 T_cam_imu 和 time_shift。少解 8 个参数,收敛稳得多,也避免内参被
# 运动模糊的帧带偏。
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:?用法: ./calibrate_imu_cam.sh <bag> [camchain] [imu.yaml]}"
CAM="${2:-$D/data/cam_rgb-camchain.yaml}"
IMU="${3:-$D/results/allan_imu.yaml}"
FROM="${4:-}"          # 可选:裁掉首尾,避开 B-spline 端点越界
TO="${5:-}"

BASE=$(basename "$BAG" .bag); DIR=$(dirname "$BAG")
rm -f "$DIR/$BASE-camchain-imucam.yaml" "$DIR/$BASE-imu.yaml" \
      "$DIR/$BASE-report-imucam.pdf" "$DIR/$BASE-results-imucam.txt"

# camchain 的 rostopic 必须与 bag 里的对上
echo "camchain: $CAM"; grep -E "rostopic|resolution|intrinsics" "$CAM" | sed 's/^/  /'
[ -n "$FROM" ] && echo "时间裁剪: ${FROM}s ~ ${TO}s  (避开 B-spline 端点越界)"
echo "imu:      $IMU"; grep -E "noise_density|random_walk|update_rate" "$IMU" | sed 's/^/  /'
echo

"$D/kalibr.sh" rosrun kalibr kalibr_calibrate_imu_camera \
  --bag "/data/${BAG#$D/}" \
  --cam "/data/${CAM#$D/}" \
  --imu "/data/${IMU#$D/}" \
  --target /data/aprilgrid_6x6_35.2mm.yaml \
  ${FROM:+--bag-from-to $FROM $TO} \
  --dont-show-report 2>&1 | sed 's/\x1b\[[0-9;]*m//g' \
  | grep -vE "Subpix refinement|partially in image|^\s*$|rospack|dbind-WARNING|secure directory"

echo
echo "=== 结果 ==="
[ -f "$DIR/$BASE-camchain-imucam.yaml" ] && cat "$DIR/$BASE-camchain-imucam.yaml"
[ -f "$DIR/$BASE-report-imucam.pdf" ] && echo && echo "报告: $DIR/$BASE-report-imucam.pdf"
