#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.7 AP-5：ECB 回读链路诊断（只发读指令和握手，不写参数、不使能、不运动）。

前提：机械臂在搁架上、零扭矩使能后保持；执行器进程已停，避免和本工具抢总线。

用法（抓包里最好同时留一份证据，tcpdump 由用户用 sudo 另开）：
    python3 tools/link_quality_check.py --section all
    python3 tools/link_quality_check.py --section joints --n 2000
    python3 tools/link_quality_check.py --section ping
    python3 tools/link_quality_check.py --section intervals
    python3 tools/link_quality_check.py --section burst

结果写到 runs/ap5_link_quality_<stamp>.json/.txt。
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod  # noqa: E402
from arm.sca import R_CURRENT, R_POSITION, R_POWER, R_MODE, R_ERROR  # noqa: E402
from arm.transport import make_frame, parse_frames  # noqa: E402

HEAD = 0xEE
TAIL = 0xED


def pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return round(s[i], 3)


class RawClient:
    def __init__(self, ip: str, port: int = 2000, timeout: float = 0.25,
                 late_grace: float = 0.1):
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.late_grace = float(late_grace)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self.sent = 0
        self.crc_errors = 0

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def _drain(self):
        self.sock.setblocking(False)
        try:
            while True:
                try:
                    self.sock.recvfrom(8192)
                except (BlockingIOError, InterruptedError, OSError):
                    break
        finally:
            self.sock.setblocking(True)

    def _collect_until(self, end):
        frames = []
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            self.sock.settimeout(min(0.01, left))
            try:
                data, _ = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            got = parse_frames(data)
            self.crc_errors += sum(1 for f in got if f.cmd == 0x50)
            frames.extend(got)
        return frames

    def request(self, addr: int, cmd: int, data: bytes = b"", n: int | None = None,
                timeout: float | None = None, late: bool = True):
        """发一条，等第一条匹配回包；再等 late_grace 统计迟到/重复。"""
        timeout = self.timeout if timeout is None else timeout
        self._drain()
        t0 = time.monotonic()
        self.sock.sendto(make_frame(addr, cmd, data), (self.ip, self.port))
        self.sent += 1
        first = None
        latencies = []
        dups = 0
        end = t0 + timeout
        while True:
            for f in self._collect_until(min(end, time.monotonic() + 0.01)):
                if f.addr != addr or f.cmd != cmd:
                    continue
                lat = (time.monotonic() - t0) * 1000.0
                latencies.append(lat)
                if first is None:
                    first = f
            if first is not None or time.monotonic() >= end:
                break
        late_n = 0
        if late and first is not None:
            grace_end = t0 + timeout + self.late_grace
            while time.monotonic() < grace_end:
                for f in self._collect_until(min(grace_end, time.monotonic() + 0.005)):
                    if f.addr == addr and f.cmd == cmd:
                        dups += 1
                        latencies.append((time.monotonic() - t0) * 1000.0)
                if not dups:
                    break
        elif late:
            # 超时以后再看看有没有迟到的
            grace_end = t0 + timeout + self.late_grace
            while time.monotonic() < grace_end:
                for f in self._collect_until(min(grace_end, time.monotonic() + 0.005)):
                    if f.addr == addr and f.cmd == cmd:
                        late_n += 1
                        latencies.append((time.monotonic() - t0) * 1000.0)
                if late_n:
                    break
        return {
            "ok": first is not None,
            "first_latency_ms": None if first is None else round(latencies[0], 3),
            "late": late_n > 0,
            "dups": dups,
            "data": None if first is None else first.data,
        }


def joint_ids():
    cfg = cfgmod.load_joints()
    out = []
    for name in cfg.get("joint_order") or []:
        j = cfg["joints"][name]
        out.append((name, int(j["id"])))
    return out


def summarise(rows):
    lat = [r["first_latency_ms"] for r in rows if r["first_latency_ms"] is not None]
    return {
        "n": len(rows),
        "no_reply": sum(1 for r in rows if not r["ok"]),
        "late": sum(1 for r in rows if r["late"]),
        "dups": sum(r["dups"] for r in rows),
        "lat_ms": {
            "p50": pct(lat, 0.50), "p90": pct(lat, 0.90),
            "p99": pct(lat, 0.99), "p999": pct(lat, 0.999),
            "max": None if not lat else round(max(lat), 3),
            "median": None if not lat else round(statistics.median(lat), 3),
        },
    }


def section_joints(client, n_base, names_ids):
    out = {}
    for name, mid in names_ids:
        rows = []
        t0 = time.monotonic()
        for _ in range(n_base):
            rows.append(client.request(mid, R_POSITION, n=4))
        out[name] = summarise(rows)
        out[name]["elapsed_s"] = round(time.monotonic() - t0, 2)
    return out


def section_ping(ip, count=1000):
    cmd = ["ping", "-n", "-c", str(count), "-i", "0.02", "-W", "800", ip]
    t0 = time.monotonic()
    p = subprocess.run(cmd, capture_output=True, text=True)
    text = p.stdout + p.stderr
    loss = None
    for line in text.splitlines():
        if "packet loss" in line:
            try:
                loss = float(line.split("%")[0].split()[-1])
            except Exception:  # noqa: BLE001
                pass
        if "min/avg/max" in line or "round-trip" in line:
            pass
    return {"n": count, "loss_percent": loss, "elapsed_s": round(time.monotonic() - t0, 2),
            "raw_tail": text.strip().splitlines()[-3:]}


def section_ecb_only(client, n=1000):
    rows = [client.request(0x00, 0x44) for _ in range(n)]
    return summarise(rows)


def section_intervals(client, names_ids, n=3000):
    out = {}
    for interval_ms in (0, 2, 5, 10, 20):
        rows = []
        t0 = time.monotonic()
        i = 0
        while len(rows) < n:
            name, mid = names_ids[i % len(names_ids)]
            i += 1
            rows.append(client.request(mid, R_POSITION, n=4))
            if interval_ms:
                time.sleep(interval_ms / 1000.0)
        out[str(interval_ms)] = summarise(rows)
        out[str(interval_ms)]["elapsed_s"] = round(time.monotonic() - t0, 2)
    return out


def section_burst(client, names_ids, rounds=2000):
    # 串行：发一条、等回包、再下一条
    serial_rows = []
    t0 = time.monotonic()
    for _ in range(rounds):
        for name, mid in names_ids:
            serial_rows.append(client.request(mid, R_POSITION, n=4))
    serial_s = time.monotonic() - t0
    # 并发：六条一起发，再收六个
    parsed = 0
    no_reply = 0
    dup = 0
    rounds_t = []
    for _ in range(rounds):
        client._drain()
        t = time.monotonic()
        for name, mid in names_ids:
            client.sock.sendto(make_frame(mid, R_POSITION), (client.ip, client.port))
            client.sent += 1
        want = {mid for _, mid in names_ids}
        got = set()
        end = t + client.timeout
        while time.monotonic() < end and got != want:
            for f in client._collect_until(min(end, time.monotonic() + 0.01)):
                if f.cmd == R_POSITION and f.addr in want:
                    if f.addr in got:
                        dup += 1
                    got.add(f.addr)
        no_reply += len(want - got)
        rounds_t.append((time.monotonic() - t) * 1000.0)
        parsed += 1
    return {
        "serial": {**summarise(serial_rows), "elapsed_s": round(serial_s, 2),
                   "per_round_ms": round(serial_s / rounds * 1000.0, 3)},
        "burst": {"rounds": rounds, "no_reply": no_reply, "dups": dup,
                  "per_round_ms": {"p50": pct(rounds_t, 0.5), "p99": pct(rounds_t, 0.99),
                                   "max": round(max(rounds_t), 3)}},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--section", default="all",
                    choices=["all", "joints", "ping", "ecb", "intervals", "burst"])
    ap.add_argument("--n", type=int, default=2000)
    args = ap.parse_args()
    ecb = cfgmod.load_executor()
    ip = ecb.get("ecb_ip", "192.168.1.30")
    port = int(ecb.get("ecb_udp_port", 2000))
    client = RawClient(ip, port)
    names = joint_ids()
    result = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "ecb": f"{ip}:{port}",
              "timeout_ms": client.timeout * 1000, "late_grace_ms": client.late_grace * 1000,
              "transport_resync_ms": 30.0, "joints": [n for n, _ in names]}
    try:
        if args.section in ("all", "ping"):
            result["ping"] = section_ping(ip)
        if args.section in ("all", "ecb"):
            result["ecb_only"] = section_ecb_only(client)
        if args.section in ("all", "joints"):
            result["per_joint"] = section_joints(client, args.n, names)
        if args.section in ("all", "intervals"):
            result["intervals"] = section_intervals(client, names, max(300, args.n))
        if args.section in ("all", "burst"):
            result["burst"] = section_burst(client, names, max(200, args.n))
        result["crc_error_frames"] = client.crc_errors
        result["sent"] = client.sent
    finally:
        client.close()
    runs = Path("runs")
    runs.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    (runs / f"ap5_link_quality_{stamp}.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    lines = [json.dumps(result, ensure_ascii=False, indent=1)]
    (runs / f"ap5_link_quality_{stamp}.txt").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
