#!/bin/bash
# D435i 标定 —— 硬件侧一次性配置。需要 sudo。
set -e
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/4] 清掉签名验证失败的 librealsense apt 源"
rm -f /etc/apt/sources.list.d/librealsense.list /etc/apt/keyrings/librealsense.pgp
echo "      已删。(我们改走 pip 装 pyrealsense2,不需要这个源)"

echo "[2/4] 安装 RealSense udev 规则"
install -m 644 "$D/99-realsense-libusb.rules" /etc/udev/rules.d/99-realsense-libusb.rules
udevadm control --reload-rules && udevadm trigger
echo "      已装。普通用户即可打开相机,标定脚本不需要 sudo"

echo "[3/4] 验证 apt 不再报错"
apt update -qq 2>&1 | grep -iE "错误|error|W:" && echo "      ⚠ 仍有告警(见上)" || echo "      apt 干净"

echo "[4/4] 检查 D435i"
if lsusb | grep -q 8086; then
  lsusb | grep 8086
  echo "      >>> D435i 已连接"
else
  echo "      >>> 没看到 Intel 设备"
  echo "          相机没插,或者插在 USB Hub 后面没协商上"
  echo "          你机器上串了 3 个 Genesys Hub —— 请直插主板 USB3 口"
fi
