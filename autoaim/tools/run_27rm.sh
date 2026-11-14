#!/bin/sh
# ===========================================================================
#  run_27rm.sh —— 27RM 自瞄/图传守护启动器（崩了自动拉起）
#
#  为什么需要：27RM 一旦退出，**图传（:8080）和自瞄（UDP 40002）会一起消失**，
#  比赛现场遇到就废了。2026-11-08 实测它自己挂过两次（日志停在检测中途、
#  dmesg 里有带 Comm: 27RM 的内核告警），所以加这一层。
#
#  启动：
#      cd ~/stmproject/27RM_AUTO_AIM
#      nohup setsid sh ./tools/run_27rm.sh >/dev/null 2>&1 &
#  停止（两步都要，顺序别反）：
#      pkill -f run_27rm.sh          # 先杀守护，否则它会立刻把 27RM 拉起来
#      pkill -x 27RM
#  看状态 / 看日志：
#      pgrep -a -x 27RM
#      tail -f ~/27rm_loop.log       # 守护日志：每次启动/退出/重启原因
#      tail -f /tmp/27rm.log         # 27RM 自己的输出（检测/采集/自瞄发送端）
#
#  参数用环境变量覆盖（一般不用动）：
#      RM27_ARGS    传给 27RM 的参数
#      RM27_LOG     守护日志（默认 ~/27rm_loop.log）
#      RM27_STDOUT  27RM 的 stdout（默认 /tmp/27rm.log）
#      RM27_DELAY   正常退出后等多久重启（默认 2 秒）
# ===========================================================================
set -u

PROJ=$(cd "$(dirname "$0")/.." && pwd)
BIN="$PROJ/build/27RM"
LOG=${RM27_LOG:-$HOME/27rm_loop.log}
OUT=${RM27_STDOUT:-/tmp/27rm.log}
DELAY=${RM27_DELAY:-2}
# 默认参数就是比赛用的那套：
#   --stream-bind wlan0  图传只走 WiFi（有线访问不到 8080）
#   --aim-udp auto       自瞄广播按 wlan0 的掩码自动算（换路由器不用改）
#   后面再跟一个有线的，方便插网线调试
ARGS=${RM27_ARGS:-"--stream-port 8080 --stream-bind wlan0 --stream-fps 30 --stream-quality 60 --stream-scale 0.75 --aim-udp auto,192.168.2.255:40002 --no-gui"}

logline() { echo "=== $(date '+%F %T') $* ===" >> "$LOG"; }

if [ ! -x "$BIN" ]; then
  logline "找不到可执行文件 $BIN，退出"
  exit 1
fi

# ---- 防止起两份守护（否则两份抢相机/抢端口）----
# 用 pid 文件而不是 pgrep -f：后者会把这整条 ssh 命令（命令行里含脚本名）也匹配成守护。
PIDFILE=${RM27_PIDFILE:-/tmp/run_27rm.pid}
if [ -f "$PIDFILE" ]; then
  oldpid=$(cat "$PIDFILE" 2>/dev/null)
  if [ -n "${oldpid:-}" ] && kill -0 "$oldpid" 2>/dev/null; then
    logline "已有守护在跑（pid $oldpid），本次退出"
    exit 0
  fi
fi
echo $$ > "$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT INT TERM

# ---- 起之前清掉旧实例，避免端口/相机被占 ----
if pkill -x 27RM 2>/dev/null; then
  sleep 1
fi

logline "守护启动  args: $ARGS"
fail=0
while true; do
  # 相机不在就别硬起（起了也是 "打不开 /dev/video0"）：等一会儿，最多 30 秒
  if [ ! -e /dev/video0 ]; then
    logline "/dev/video0 不存在，等待相机枚举（可跑 tools/usb_recover.sh）"
    sleep 3
    [ ! -e /dev/video0 ] && continue
    logline "/dev/video0 出现了，继续"
  fi

  logline "启动 27RM"
  start=$(date +%s)
  # 用 stdbuf 让日志实时落盘（崩溃时最后几行才有价值）
  # shellcheck disable=SC2086
  ( cd "$PROJ" && stdbuf -oL -eL ./build/27RM $ARGS >> "$OUT" 2>&1 )
  code=$?
  ran=$(( $(date +%s) - start ))
  logline "27RM 退出 code=$code 运行 ${ran}s"

  if [ "$ran" -lt 5 ]; then
    # 秒退：多半是相机/端口问题，退避久一点，别把机器拖成死循环
    fail=$((fail + 1))
    logline "秒退第 $fail 次（相机掉了？端口被占？），等 $((DELAY * 3)) 秒再试"
    if [ "$fail" -ge 10 ]; then
      logline "连续 10 次秒退，守护放弃（人工介入：查 /dev/video0 与 :8080 占用）"
      exit 2
    fi
    sleep $((DELAY * 3))
  else
    fail=0
    sleep "$DELAY"
  fi
done
