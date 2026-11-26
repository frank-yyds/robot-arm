#!/bin/bash
# 第二十九节 29.1 第 1 条：真机会话开始时拉起抓包。
#
# 【2026-11-26 修订】默认不再用 tcpdump：Mac 上开 BPF 要 sudo，agent 拿不到密码，
# 13:00 以后就算过一整场、23:32 之后环写满也没人发现。现在默认用执行器内的抓包
# 转写（arm/pcapio.py 的 PacketTee）：每次 /api/connect 自动起，文件
# runs/_pcap_ring/ecb_tee_<时间>.pcap，不用 sudo、不受环容量限制、写的是发送前
# 的字节（pcap_check 照原样能读）。
#
# 用法：  tools/session_start.sh            # 检查执行器转写有没有在跑（默认）
#         tools/session_start.sh --tcpdump  # 仍然要 tcpdump 的独立见证（要 sudo）
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RING="${ROOT}/runs/_pcap_ring"
mkdir -p "${RING}"

if [ "${1:-}" != "--tcpdump" ]; then
  LATEST=$(ls -t "${RING}"/ecb_tee_*.pcap 2>/dev/null | head -1)
  if [ -n "${LATEST}" ]; then
    echo "✓ 执行器抓包转写在跑：${LATEST}"
    python3 - "${LATEST}" <<'PY'
import sys, os, time
p = sys.argv[1]
age = time.time() - os.path.getmtime(p)
print(f"  文件 {os.path.getsize(p)} 字节，{age:.0f} 秒前写过（会话中每 20 条写盘一次）")
PY
  else
    echo "✗ 还没看到执行器转写文件；连上执行器（/api/connect）就会自动起"
  fi
  echo "  （要 sudo 的 tcpdump 独立见证：tools/session_start.sh --tcpdump）"
  exit 0
fi

# ---------------- 以下是原来的 tcpdump 版本（要 sudo） ----------------
IFACE="${2:-}"
ECB_IP="${3:-192.168.1.30}"
if [ -z "${IFACE}" ]; then
  IFACE=$(route -n get "${ECB_IP}" 2>/dev/null | awk '/interface:/{print $2}')
fi
if [ -z "${IFACE}" ]; then
  echo "✗ 找不到去 ${ECB_IP} 的网口；请手动给：tools/session_start.sh --tcpdump en9" >&2
  exit 2
fi

PIDFILE="${RING}/tcpdump.pid"
if [ -f "${PIDFILE}" ]; then
  OLD=$(cat "${PIDFILE}" 2>/dev/null || true)
  if [ -n "${OLD}" ] && kill -0 "${OLD}" 2>/dev/null; then
    echo "有旧 tcpdump 在跑（PID ${OLD}），先停掉它"
    sudo -v
    sudo -n kill "${OLD}" 2>/dev/null || true
    sleep 1
  fi
  rm -f "${PIDFILE}"
fi
rm -f "${RING}"/ecb_%Y*.pcap* 2>/dev/null || true

echo "网口 ${IFACE} → ECB ${ECB_IP}；抓包目录 ${RING}（每个文件 200 MB，留 200 个，最多 40 GB）"
echo "下面要输一次 sudo 密码"
sudo -v || { echo "✗ sudo 没通过" >&2; exit 2; }
sudo -n tcpdump -i "${IFACE}" -n -s 0 -U \
  -C 200 -W 200 \
  -w "${RING}/ecb_ring.pcap" \
  "udp and host ${ECB_IP}" \
  >"${RING}/tcpdump.log" 2>&1 &
PID=$!
echo "${PID}" > "${PIDFILE}"
sleep 2
if kill -0 "${PID}" 2>/dev/null; then
  echo "✓ tcpdump 已起：PID ${PID}（网口 ${IFACE}）；文件 ${RING}/ecb_ring.pcap*（-U 立即落盘）"
else
  echo "✗ tcpdump 起来就退出了，看 ${RING}/tcpdump.log" >&2
  exit 1
fi
