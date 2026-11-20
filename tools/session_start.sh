#!/bin/bash
# 第二十九节 29.1 第 1 条：真机会话开始时拉起 tcpdump 环形抓包（要 sudo，会问一次密码）。
#
# 用法：  tools/session_start.sh [网口] [ECB_IP]
#   不给网口就按到 ECB 的路由自动找；抓包文件在 runs/_pcap_ring/，
#   pid 写在 runs/_pcap_ring/tcpdump.pid，执行器连上时会把它写进会话头。
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RING="${ROOT}/runs/_pcap_ring"
ECB_IP="${2:-192.168.1.30}"
IFACE="${1:-}"
mkdir -p "${RING}"

if [ -z "${IFACE}" ]; then
  IFACE=$(route -n get "${ECB_IP}" 2>/dev/null | awk '/interface:/{print $2}')
fi
if [ -z "${IFACE}" ]; then
  echo "✗ 找不到去 ${ECB_IP} 的网口；请手动给：tools/session_start.sh en9" >&2
  exit 2
fi

PIDFILE="${RING}/tcpdump.pid"
if [ -f "${PIDFILE}" ]; then
  OLD=$(cat "${PIDFILE}" 2>/dev/null || true)
  if [ -n "${OLD}" ] && kill -0 "${OLD}" 2>/dev/null; then
    echo "已经有 tcpdump 在跑（PID ${OLD}，网口见 ${RING}/tcpdump.log）；先 session_stop.sh"
    exit 0
  fi
fi

echo "网口 ${IFACE} → ECB ${ECB_IP}；抓包目录 ${RING}（每个文件 100 MB，留 20 个）"
echo "下面要输一次 sudo 密码"
sudo tcpdump -i "${IFACE}" -n -s 0 \
  -C 100 -W 20 \
  -w "${RING}/ecb_%Y%m%d_%H%M%S.pcap" \
  "udp and host ${ECB_IP}" \
  >"${RING}/tcpdump.log" 2>&1 &
PID=$!
echo "${PID}" > "${PIDFILE}"
sleep 1
if kill -0 "${PID}" 2>/dev/null; then
  echo "✓ tcpdump 已起：PID ${PID}（网口 ${IFACE}）"
  echo "  抓包文件：${RING}/ecb_*.pcap；日志：${RING}/tcpdump.log"
else
  echo "✗ tcpdump 起来就退出了，看 ${RING}/tcpdump.log" >&2
  exit 1
fi
