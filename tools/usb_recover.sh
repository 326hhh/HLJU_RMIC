#!/bin/sh
# 27RM USB 摄像头一键恢复脚本
#
# 用途：相机“能打开但不出帧/程序卡死”时执行。做三件事：
#   1) 重新枚举该 USB 设备（deauthorize/authorize，等效软插拔）
#   2) 关闭相机 / 所在 hub / 根 hub 的 USB 自动挂起
#   3) 用 v4l2-ctl 验证能取到帧
#
# 用法（在 Jetson 上）：
#   ./tools/usb_recover.sh            # 默认设备路径 1-2.3，摄像头 /dev/video0
#   ./tools/usb_recover.sh 1-2.4      # 相机换了 USB 口时指定新路径
#
# 注意：如果本脚本救不回来（取流仍然失败），说明需要【物理拔插】摄像头
# 或换 USB 口（别挂在外接 hub 上，直接插 Jetson 的 USB 口最稳）。

set -u
CAM="${1:-1-2.3}"
DEV="/dev/video0"
SYSDIR="/sys/bus/usb/devices"

if [ ! -e "$SYSDIR/$CAM" ]; then
  echo "找不到 USB 设备路径 $SYSDIR/$CAM"
  echo "先看看当前拓扑：ls /sys/bus/usb/devices/ ；lsusb"
  exit 1
fi

echo "[1/4] 重新枚举 $CAM（软插拔）"
sudo sh -c "echo 0 > $SYSDIR/$CAM/authorized" 2>/dev/null || true
sleep 1.5
sudo sh -c "echo 1 > $SYSDIR/$CAM/authorized" 2>/dev/null || true
sleep 4

echo "[2/4] 关闭 USB 自动挂起（全局 + 相机 / hub / 根 hub）"
sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend" 2>/dev/null || true
for d in 1-2 "$CAM" usb1; do
  [ -e "$SYSDIR/$d/power/control" ] &&
    sudo sh -c "echo on > $SYSDIR/$d/power/control" 2>/dev/null || true
done

echo "[3/4] 等设备节点就绪"
for _ in 1 2 3 4 5 6 7 8 9 10; do
  [ -e "$DEV" ] && break
  sleep 0.5
done
if [ ! -e "$DEV" ]; then
  echo "❌ $DEV 没出现：设备没枚举成功，请物理拔插摄像头"
  exit 1
fi

echo "[4/4] 取流验证（用本项目 bench，MJPG 优先、自动降级 YUYV）"
BIN=./build/27RM
if [ -x "$BIN" ]; then
  OUT=$(timeout 40 "$BIN" --bench --frames 150 2>&1 | tr '\r' '\n' | grep -E '采集|降级|收不到' | tail -3)
  echo "$OUT"
  if echo "$OUT" | grep -q "采集"; then
    echo "✅ 相机可用"
  elif echo "$OUT" | grep -q "降级"; then
    echo "⚠️ MJPG 拿不到帧但已自动降级 YUYV 跑通：比赛前请物理拔插恢复 MJPG"
  else
    echo "❌ 取不到帧：请【物理拔插】摄像头（断电 10 秒再插回）"
    exit 2
  fi
else
  echo "⚠️ 没有 $BIN，跳过验证（先 cmake --build build -j4 构建）"
fi
