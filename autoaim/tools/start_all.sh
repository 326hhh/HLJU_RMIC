#!/bin/sh
# 27RM 一站式启动：云台控制守护进程（~/gimbal）+ 自瞄/图传（27RM）
#
# 三件套：
#   控制   = gimbal 守护进程(Jetson) ←UDP 40001→ gimbal_ctl(PC)
#   图传   = 27RM 内嵌 MJPEG :8080 → gimbal_ctl 窗口
#   自瞄   = 27RM 检测/位姿/反小陀螺
#
# 用法（在 Jetson 上，工程根目录）：
#   ./tools/start_all.sh                     # 默认：gimbal --device /dev/ttyUSB0 + 27RM(图传8080)
#   ./tools/start_all.sh --chassis can0      # 混合后端（云台串口 + 底盘 CAN）
#   ./tools/start_all.sh --no-gimbal         # 只起 27RM（控制端先不接硬件）
#   ./tools/start_all.sh --27rm-daemon       # 27RM 交给 tools/run_27rm.sh 守护（崩了自动拉起）
#   ./tools/start_all.sh --27rm-args "--no-gui"   # 给 27RM 追加参数
#   ./tools/start_all.sh --stop              # 停止全部
#
# PC 控制端：
#   ./build/gimbal_ctl --host 192.168.2.2    # 控制 + 视频（视频来自 27RM 的 8080）
#
# 注意：
#   - 相机同一时刻只能被 27RM 占用，别再跑 ~/gimbal 里的 mjpeg_server；
#   - 端口 8080 归 27RM；要换端口用 --27rm-args "--stream-port 8090"；
#   - 从 ssh 启动时没有 DISPLAY，27RM 会自动加 --no-gui。

set -u

GIMBAL_DIR=${GIMBAL_DIR:-$HOME/gimbal}
PROJECT_DIR=$(cd "$(dirname "$0")/.." && pwd)
GIMBAL_ARGS="--device /dev/ttyUSB0"
R27_ARGS="--stream-port 8080 --stream-fps 30 --stream-quality 60 --stream-scale 0.75"
START_GIMBAL=1
START_27RM=1
RM27_DAEMON=0
MODE=start

usage() {
  sed -n '5,20p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --chassis)   GIMBAL_ARGS="--device $2 --chassis $2"; shift 2 ;;
    --device)    GIMBAL_ARGS="--device $2"; shift 2 ;;
    --gimbal-args) GIMBAL_ARGS="$2"; shift 2 ;;
    --no-gimbal) START_GIMBAL=0; shift ;;
    --no-27rm)   START_27RM=0; shift ;;
    --27rm-daemon) RM27_DAEMON=1; shift ;;
    --27rm-args) R27_ARGS="$R27_ARGS $2"; shift 2 ;;
    --stop)      MODE=stop; shift ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "未知参数: $1"; usage; exit 2 ;;
  esac
done

stop_all() {
  if pkill -f "run_27rm.sh" 2>/dev/null; then echo "已停止 27RM 守护"; fi
  if pkill -x 27RM 2>/dev/null; then echo "已停止 27RM"; fi
  if pkill -f "build/gimbal" 2>/dev/null; then echo "已停止 gimbal 守护进程"; fi
  sleep 0.5
}

if [ "$MODE" = "stop" ]; then
  stop_all
  exit 0
fi

# 启动前只清理「本次要启动的」进程（--no-gimbal 时不动已运行的守护进程）
if [ "$START_GIMBAL" = 1 ] && pgrep -x gimbal >/dev/null 2>&1; then
  pkill -x gimbal 2>/dev/null && echo "（先停掉旧的 gimbal 守护进程）"
  sleep 0.5
fi
if [ "$START_27RM" = 1 ] && pgrep -x 27RM >/dev/null 2>&1; then
  pkill -x 27RM 2>/dev/null && echo "（先停掉旧的 27RM）"
  sleep 0.5
fi

# 没有图形环境时，27RM 强制无窗口
if [ -z "${DISPLAY:-}" ]; then
  R27_ARGS="$R27_ARGS --no-gui"
fi

if [ "$START_GIMBAL" = 1 ]; then
  if [ -x "$GIMBAL_DIR/build/gimbal" ]; then
    ( cd "$GIMBAL_DIR" && nohup ./build/gimbal $GIMBAL_ARGS \
        > /tmp/gimbal.log 2>&1 < /dev/null & )
    sleep 1.5
    if pgrep -f "build/gimbal" >/dev/null 2>&1; then
      echo "[OK] gimbal 守护进程已启动（$GIMBAL_ARGS），日志 /tmp/gimbal.log"
    else
      echo "[WARN] gimbal 没起来，看 /tmp/gimbal.log："
      tail -5 /tmp/gimbal.log 2>/dev/null
    fi
  else
    echo "[WARN] 找不到 $GIMBAL_DIR/build/gimbal"
    echo "       （没接硬件可加 --no-gimbal；或 export GIMBAL_DIR=<路径>）"
  fi
fi

if [ "$START_27RM" = 1 ]; then
  if [ "$RM27_DAEMON" = 1 ]; then
    # 守护模式：交给 tools/run_27rm.sh（崩了自动重启，日志 ~/27rm_loop.log）
    ( cd "$PROJECT_DIR" && RM27_ARGS="$R27_ARGS" nohup setsid sh ./tools/run_27rm.sh \
        > /dev/null 2>&1 < /dev/null & )
    echo "（27RM 由守护脚本管理，停止用 ./tools/start_all.sh --stop）"
  else
    ( cd "$PROJECT_DIR" && nohup ./build/27RM $R27_ARGS \
        > /tmp/27rm.log 2>&1 < /dev/null & )
  fi
  sleep 2
  if pgrep -x 27RM >/dev/null 2>&1; then
    echo "[OK] 27RM 已启动（自瞄 + 图传 :8080），日志 /tmp/27rm.log"
    echo "     图传地址: http://<jetson-ip>:8080/stream"
  else
    echo "[WARN] 27RM 没起来，看 /tmp/27rm.log："
    tail -5 /tmp/27rm.log 2>/dev/null
  fi
fi

echo
echo "== 当前状态 =="
pgrep -af "build/gimbal" | head -2
pgrep -x 27RM | head -2
echo
echo "PC 控制端: ./build/gimbal_ctl --host <jetson-ip>   （视频自动来自 27RM 的 8080）"
echo "停止全部: ./tools/start_all.sh --stop"
