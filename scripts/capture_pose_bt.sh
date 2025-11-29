#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法: capture_pose_bt.sh [采样秒] [日志目录]

说明:
  - 持续监控 PoseEstimation 进程的内存使用 (RSS/VSS)
  - 所有采样数据保存到 CSV 文件

参数:
  - 第一个可选参数: 采样间隔（秒），默认 1 秒
  - 第二个可选参数: 日志目录，默认 ~/ws_livox/src/LIO/logs

输出:
  - pose_rss_<timestamp>.csv: RSS/VSS 时序数据

注意:
  - 需要先启动 PoseEstimation（例如 roslaunch），脚本才会找到 PID
  - Ctrl+C 可安全退出，数据会保留
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

INTERVAL=${1:-1}
LOG_DIR=${2:-/home/orangepi/ws_livox/src/LIO/logs}
SESSION_STAMP=$(date '+%Y%m%d_%H%M%S')
RSS_LOG_PATH="${LOG_DIR}/pose_rss_${SESSION_STAMP}.csv"

mkdir -p "${LOG_DIR}"

# 等待 PoseEstimation 启动
echo "等待 PoseEstimation 进程启动..."
for i in {1..30}; do
  PID=$(pgrep -n PoseEstimation || true)
  if [[ -n "${PID}" ]]; then
    break
  fi
  sleep 1
done

if [[ -z "${PID:-}" ]]; then
  echo "超时: 未找到 PoseEstimation 进程，请先启动节点。"
  exit 1
fi

echo "========================================"
echo " PoseEstimation 内存监控"
echo "========================================"
echo " PID: ${PID}"
echo " 采样间隔: ${INTERVAL} s"
echo " 日志目录: ${LOG_DIR}"
echo " RSS 日志: ${RSS_LOG_PATH}"
echo "========================================"
echo ""

# CSV 头
echo "timestamp,rss_mb,vss_mb" > "${RSS_LOG_PATH}"

# 捕获 Ctrl+C，优雅退出
cleanup() {
  echo -e "\n\n监控结束。"
  echo "RSS 峰值: ${MAX_RSS} MB"
  echo "数据已保存到: ${RSS_LOG_PATH}"
  exit 0
}
trap cleanup INT TERM

MAX_RSS=0

while kill -0 "${PID}" 2>/dev/null; do
  # 获取 RSS 和 VSS (单位 KB)
  mem_info=$(ps -o rss=,vsz= -p "${PID}" 2>/dev/null | tr -s ' ')
  [[ -z "${mem_info}" ]] && break

  rss_kb=$(echo "${mem_info}" | awk '{print $1}')
  vss_kb=$(echo "${mem_info}" | awk '{print $2}')

  rss_mb=$((rss_kb / 1024))
  vss_mb=$((vss_kb / 1024))
  timestamp="$(date '+%Y-%m-%d %H:%M:%S')"

  # 更新最大值
  (( rss_mb > MAX_RSS )) && MAX_RSS=${rss_mb}

  # 实时显示
  printf "\r[%s] RSS: %4d MB | VSS: %4d MB | Peak: %4d MB" "${timestamp}" "${rss_mb}" "${vss_mb}" "${MAX_RSS}"

  # 写入 CSV
  printf "%s,%d,%d\n" "${timestamp}" "${rss_mb}" "${vss_mb}" >> "${RSS_LOG_PATH}"

  sleep "${INTERVAL}"
done

echo -e "\n\nPoseEstimation 进程已结束。"
echo "RSS 峰值: ${MAX_RSS} MB"
echo "数据已保存到: ${RSS_LOG_PATH}"
exit 0
