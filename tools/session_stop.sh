#!/bin/bash
# 第二十九节 29.1 第 1 条：会话结束时停掉 tcpdump（要 sudo）。
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RING="${ROOT}/runs/_pcap_ring"
PIDFILE="${RING}/tcpdump.pid"
if [ ! -f "${PIDFILE}" ]; then
  echo "没有在跑的抓包（没有 ${PIDFILE}）"
  exit 0
fi
PID=$(cat "${PIDFILE}" 2>/dev/null || true)
if [ -z "${PID}" ]; then
  echo "pidfile 是空的"; rm -f "${PIDFILE}"; exit 0
fi
echo "停 tcpdump PID ${PID}（要 sudo，可能要输密码）"
sudo kill "${PID}" 2>/dev/null || true
for _ in $(seq 1 20); do
  kill -0 "${PID}" 2>/dev/null || break
  sleep 0.1
done
if kill -0 "${PID}" 2>/dev/null; then
  sudo kill -9 "${PID}" 2>/dev/null || true
fi
rm -f "${PIDFILE}"
echo "✓ 已停；抓包文件留在 ${RING}"
