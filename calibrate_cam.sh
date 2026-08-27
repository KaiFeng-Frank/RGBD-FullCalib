#!/bin/bash
# Kalibr 相机内参 / 多相机外参标定封装。
#
#   单目: ./calibrate_cam.sh data/cam_rgb.bag /cam2/image_raw 914.2
#   双目: ./calibrate_cam.sh data/cam_ir.bag  "/cam0/image_raw /cam1/image_raw" 423.5
#
# 焦距初值必传的理由:Kalibr 的焦距自动初始化对姿态分布敏感,失败时它会走
# KALIBR_MANUAL_FOCAL_LENGTH_INIT 分支索要手工值;非交互下读到空值会把焦距
# 设成 2.6e-315,然后"成功"输出一整套 NaN。
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:?用法: ./calibrate_cam.sh <bag> [topics] [焦距初值] [模型]}"
TOPICS="${2:-/cam2/image_raw}"
FOCAL="${3:-914.2}"
MODEL="${4:-pinhole-radtan}"

N=$(echo $TOPICS | wc -w)
MODELS=""; for i in $(seq $N); do MODELS="$MODELS $MODEL"; done

BASE=$(basename "$BAG" .bag); DIR=$(dirname "$BAG")
rm -f "$DIR/$BASE-camchain.yaml" "$DIR/$BASE-results-cam.txt" "$DIR/$BASE-report-cam.pdf"

echo "bag=$BAG"
echo "topics=$TOPICS  ($N 个相机)  model=$MODEL  焦距初值=$FOCAL"
# 每个相机可能各被问一次焦距,多喂几遍无害
FEED=""; for i in $(seq $((N*3))); do FEED="$FEED$FOCAL\n"; done

printf "$FEED" | "$D/kalibr.sh" rosrun kalibr kalibr_calibrate_cameras \
  --bag "/data/${BAG#$D/}" \
  --topics $TOPICS \
  --models $MODELS \
  --target /data/aprilgrid_6x6_35.2mm.yaml \
  --dont-show-report 2>&1 | sed 's/\x1b\[[0-9;]*m//g' \
  | grep -vE "Subpix refinement|partially in image|Progress |rospack|dbind-WARNING|secure directory|rosdep|^\s*$"

echo
echo "=== 结果 ==="
[ -f "$DIR/$BASE-camchain.yaml" ] && cat "$DIR/$BASE-camchain.yaml"
[ -f "$DIR/$BASE-report-cam.pdf" ] && echo && echo "报告: $DIR/$BASE-report-cam.pdf"
