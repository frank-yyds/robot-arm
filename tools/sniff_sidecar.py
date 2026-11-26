#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十九节 29.1 的替代抓包（历史方案；**正式方案已并入执行器**，见下）。

【2026-11-26 结论】真正能用的、不需要 sudo 的办法是 **执行器内的抓包转写**
（arm/pcapio.py 的 PacketTee，每次 /api/connect 自动起，文件
runs/_pcap_ring/ecb_tee_<时间>.pcap）。原因：
  * Mac 上 tcpdump/TShark 都要 sudo 开 BPF，agent 拿不到密码；
  * 本机 socket 旁听不到别人 socket 的出站包，也绑不上别人已占用的端口
    （2026-11-26 在 Mac 上实测：SO_REUSEADDR 绑已占用端口报 EADDRINUSE；
     广播只能覆盖发到 255.255.255.255 的包，ECB 是单播回给执行器端口）；
  * 执行器自己转写：零权限、不受环容量限制、写的就是发送前的字节，
    pcap_check.py 照原样能读。

这个侧车脚本保留做参考：它只能在 Linux 上用 SO_REUSEADDR 旁听 ECB 回包
（辅助一路），在 macOS 上基本收不到东西，现在不用它。

原说明：执行器旁边的被动旁观进程（**不用 sudo**）。

2026-11-26 用户要求：tcpdump 这套总是要/卡在 sudo 上，找一个 agent 自己能跑的。

为什么不用 tcpdump：Mac 上 tcpdump 要 sudo 才能开 BPF，agent 这边拿不到 sudo 密码，
13:00 以后开始就算过一整场、23:32 之后环写满也没人发现。

这个进程怎么做：
  * 用 SO_REUSEADDR 的 UDP socket 把自己“挂”在 ECB 端口（默认 0.0.0.0:2000）。
    Linux 下这样能和执行器的客户端 socket 同时收到 ECB 的**所有回包**；macOS 下
    同端口只能收到发往本机的广播/多播，单播回包收不到 —— 所以回包一路只会是辅助；
  * 执行器进程**本机**的出站包（这才是 pcap 对账要的那一路）在这里一定收不到，
    所以改成让执行器把发出去的每一帧转一份给侧车：
        POST /api/test/sniff_start  起侧车（执行器内部起一个本机 UDP 口，写同一个 pcap 格式）
        POST /api/test/sniff_stop   停侧车，返回文件、包数
  * 两种来源写同一个经典 pcap 文件（tools/pcap_check.py 不用改就能读）：
        runs/_pcap_ring/ecb_ebpf_<时间>.pcap      执行器出站（权威）
        runs/_pcap_ring/ecb_ebpf_<时间>_rx.pcap   执行器入站（辅助）
  * 侧车还监听 UDP 1.2.3.4:19230，可以 POST {"on":true/false} 远端开关（测试用）。

用法：
    python3 tools/sniff_sidecar.py --seconds 3600
    python3 tools/sniff_sidecar.py --stop          # 停掉正在跑的侧车（读 runs/_pcap_ring/sidecar.json）
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RING = ROOT / "runs" / "_pcap_ring"
STATE = RING / "sidecar.json"
ECB_IP = "192.168.1.30"
ECB_PORT = 2000
CTRL_PORT = 19230
LINKTYPE_ETHERNET = 1


def _pcap_header() -> bytes:
    # 经典 pcap（小端、微秒）：magic d4c3b2a1 / v2.4 / snaplen 65535 / Ethernet
    return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, LINKTYPE_ETHERNET)


def _eth_ip_udp(src_ip: str, dst_ip: str, src_port: int, dst_port: int, payload: bytes) -> bytes:
    """造一个 Ethernet/IPv4/UDP 帧（够 pcapio 解出 ECB 写命令就行）。"""
    def ip(a):
        return socket.inet_aton(a)
    udp_len = 8 + len(payload)
    ip_len = 20 + udp_len
    eth = b"\x00" * 12 + b"\x08\x00"
    iph = struct.pack(">BBHHHBBH4s4s", 0x45, 0, ip_len, 0, 0, 64, 17, 0,
                      ip(src_ip), ip(dst_ip))
    # IP 头校验和
    s = 0
    for i in range(0, 20, 2):
        s += (iph[i] << 8) + iph[i + 1]
    s = (s & 0xFFFF) + (s >> 16)
    s = (s & 0xFFFF) + (s >> 16)
    iph = iph[:10] + struct.pack(">H", ~s & 0xFFFF) + iph[12:]
    udph = struct.pack(">HHHH", src_port, dst_port, udp_len, 0)
    return eth + iph + udph + payload


class Writer:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fh = open(path, "wb")
        self.fh.write(_pcap_header())
        self.fh.flush()
        self.lock = threading.Lock()
        self.n = 0
        self.bytes = 0

    def write(self, ts: float, frame: bytes):
        sec = int(ts)
        usec = int((ts - sec) * 1e6)
        rec = struct.pack("<IIII", sec, usec, len(frame), len(frame))
        with self.lock:
            self.fh.write(rec + frame)
            self.fh.flush()
            self.n += 1
            self.bytes += len(rec) + len(frame)

    def close(self):
        with self.lock:
            try:
                self.fh.close()
            except Exception:  # noqa: BLE001
                pass

    def stat(self):
        with self.lock:
            return {"path": str(self.path), "packets": self.n, "bytes": self.bytes}


class Sidecar:
    def __init__(self, ecuport: int = ECB_PORT):
        self.stop = threading.Event()
        self.tx = Writer(RING / f"ecb_ebpf_{time.strftime('%Y%m%d_%H%M%S')}.pcap")
        self.rx = Writer(RING / f"ecb_ebpf_{time.strftime('%Y%m%d_%H%M%S')}_rx.pcap")
        self.ecuport = ecuport
        self.ports = self._executor_ports()
        self.rx_sock = None
        self.ctrl_sock = None
        self.sniffing = True
        self.t0 = time.monotonic()
        self.rx_n = 0

    @staticmethod
    def _executor_ports() -> dict:
        """从 .ecb.lock 拿执行器 pid，再用 lsof 拿它绑的两个 UDP 口（主口/写口）。"""
        out = {}
        try:
            lock = json.loads((ROOT / ".ecb.lock").read_text(encoding="utf-8"))
            pid = int(lock.get("pid"))
        except Exception:  # noqa: BLE001
            return out
        try:
            import subprocess
            txt = subprocess.run(["lsof", "-nP", "-a", "-p", str(pid), "-iUDP"],
                                 capture_output=True, text=True, timeout=5).stdout
            for ln in txt.splitlines()[1:]:
                parts = ln.split()
                for p in parts:
                    if p.startswith("*:") or p.startswith("127.0.0.1:"):
                        try:
                            out[int(p.rsplit(":", 1)[1])] = True
                        except Exception:  # noqa: BLE001
                            pass
        except Exception:  # noqa: BLE001
            pass
        out["pid"] = pid
        return out

    def _bind_rx(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", self.ecuport))
            return s
        except OSError as exc:
            print(f"（侧车）ECB 端口 {self.ecuport} 挂不上：{exc}", file=sys.stderr)
            return None

    def reader(self):
        s = self.rx_sock
        if s is None:
            return
        while not self.stop.is_set():
            try:
                s.settimeout(0.3)
                data, addr = s.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not self.sniffing:
                continue
            # ECB → 本机的回包；本机端口从执行器端口列表里猜（猜不到就写 0）
            sport = self.ecuport
            dport = 0
            for p in self.ports:
                if isinstance(p, int):
                    dport = p
                    break
            frame = _eth_ip_udp(addr[0], "127.0.0.1", sport, dport, data)
            self.rx.write(time.time(), frame)
            self.rx_n += 1

    def ctrl(self):
        s = self.ctrl_sock
        if s is None:
            return
        while not self.stop.is_set():
            try:
                s.settimeout(0.3)
                data, addr = s.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8", "replace"))
            except Exception:  # noqa: BLE001
                continue
            if "on" in msg:
                self.sniffing = bool(msg["on"])
                s.sendto(json.dumps({"ok": True, "sniffing": self.sniffing}).encode(), addr)

    def state(self):
        return {
            "pid": os.getpid(),
            "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "elapsed_s": round(time.monotonic() - self.t0, 1),
            "sniffing": self.sniffing,
            "tx": self.tx.stat(),
            "rx": self.rx.stat(),
            "executor": {k: v for k, v in self.ports.items()},
            "sudo": "不用",
        }

    def run(self, seconds: float):
        self.rx_sock = self._bind_rx()
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.ctrl_sock.bind(("127.0.0.1", CTRL_PORT))
        except OSError:
            self.ctrl_sock = None
        threads = [threading.Thread(target=self.reader, daemon=True),
                   threading.Thread(target=self.ctrl, daemon=True)]
        for t in threads:
            t.start()
        t_end = time.monotonic() + seconds

        def _term(_a, _b):
            self.stop.set()
        signal.signal(signal.SIGTERM, _term)
        signal.signal(signal.SIGINT, _term)
        RING.mkdir(parents=True, exist_ok=True)
        last_state = 0.0
        while not self.stop.is_set() and time.monotonic() < t_end:
            if time.monotonic() - last_state > 2.0:
                last_state = time.monotonic()
                STATE.write_text(json.dumps(self.state(), ensure_ascii=False, indent=1),
                                 encoding="utf-8")
            time.sleep(0.2)
        self.stop.set()
        st = self.state()
        STATE.write_text(json.dumps({**st, "finished": True}, ensure_ascii=False, indent=1),
                         encoding="utf-8")
        self.tx.close()
        self.rx.close()
        for s in (self.rx_sock, self.ctrl_sock):
            try:
                if s:
                    s.close()
            except Exception:  # noqa: BLE001
                pass
        print(json.dumps(st, ensure_ascii=False, indent=1))
        return st


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3600.0)
    ap.add_argument("--ecb-port", type=int, default=ECB_PORT)
    ap.add_argument("--stop", action="store_true")
    args = ap.parse_args(argv)
    if args.stop:
        try:
            st = json.loads(STATE.read_text(encoding="utf-8"))
            os.kill(int(st["pid"]), signal.SIGTERM)
            print(f"已让侧车 {st['pid']} 退出")
        except Exception as exc:  # noqa: BLE001
            print(f"没有在跑的侧车：{exc}")
        return 0
    Sidecar(ecuport=args.ecb_port).run(args.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
