#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法: capture_pose_bt.sh [采样秒] [日志目录]

说明:
  - 触发阈值固定为 1000MB，满足需求后自动捕获线程栈。
  - 第一个可选参数为采样间隔（秒），默认 1 秒。
  - 第二个可选参数为日志目录，默认 /home/jetson/ws_livox/src/LIO/监控。

脚本逻辑:
  1. 每隔 N 秒读取 PoseEstimation 进程的 RSS。
  2. 当 RSS 超过阈值时，自动附加 gdb，执行 thread apply all bt。
  3. 堆栈输出保存到日志目录，文件名包含时间戳。
  4. 捕获完成后脚本退出，不会杀死进程。

注意:
  - 需要先启动 PoseEstimation（例如 roslaunch），脚本才会找到 PID。
  - gdb 附加时会短暂停顿，结束后自动 detach。
  - 如果指定的日志目录不存在，会自动创建。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

THRESHOLD_MB=1000
INTERVAL=${1:-1}
LOG_DIR=${2:-/home/jetson/ws_livox/src/LIO/监控}
SESSION_STAMP=$(date '+%Y%m%d_%H%M%S')
RSS_LOG_PATH="${LOG_DIR}/pose_rss_${SESSION_STAMP}.csv"

mkdir -p "${LOG_DIR}"

PID=$(pgrep -n PoseEstimation || true)
if [[ -z "${PID}" ]]; then
  echo "未找到正在运行的 PoseEstimation 进程，请先启动节点再执行脚本。"
  exit 1
fi

echo "监控 PoseEstimation (PID=${PID})，阈值=${THRESHOLD_MB}MB，采样间隔=${INTERVAL}s，日志目录=${LOG_DIR}"
echo "timestamp,rss_mb" > "${RSS_LOG_PATH}"
echo "RSS 采样文件: ${RSS_LOG_PATH}"

while kill -0 "${PID}" 2>/dev/null; do
  rss_kb=$(ps -o rss= -p "${PID}" | tr -d '[:space:]')
  [[ -z "${rss_kb}" ]] && break
  rss_mb=$((rss_kb / 1024))
  timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
  printf "[%s] RSS=%dMB\r" "${timestamp}" "${rss_mb}"
  printf "%s,%d\n" "${timestamp}" "${rss_mb}" >> "${RSS_LOG_PATH}"

  if (( rss_mb >= THRESHOLD_MB )); then
    stamp=$(date '+%Y%m%d_%H%M%S')
    log_path="${LOG_DIR}/pose_bt_${stamp}.log"
    echo -e "\n达到阈值，使用 gdb 捕获线程栈 -> ${log_path}"
    if gdb -batch -p "${PID}" \
        -ex "set pagination off" \
        -ex "thread apply all bt" \
        -ex "detach" \
        -ex "quit" &> "${log_path}"; then
      {
        echo ""
        echo "==== RSS 采样信息 ===="
        echo "采样文件: ${RSS_LOG_PATH}"
        echo "最近 20 条样本:"
        tail -n 20 "${RSS_LOG_PATH}"
      } >> "${log_path}"
      echo "栈信息已保存到 ${log_path}"
      echo "RSS 采样记录保存在 ${RSS_LOG_PATH}"
      exit 0
    else
      echo "gdb 捕获失败，请确认 gdb 可用并重新运行脚本。"
      exit 2
    fi
  fi

  sleep "${INTERVAL}"
done

echo "PoseEstimation 进程已结束或 PID 不再有效，停止监控。"
exit 0

