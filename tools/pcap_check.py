#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.6：抓包对账（独立见证），报两类问题：

  * "执行器记了但没发出去"：events.jsonl 里有这条写命令，抓包里没有；
  * "发出去了但执行器没记"：抓包里有这条写命令，events.jsonl 里没有；
  * 另外报"不是执行器发出的包"（抓包里源端口不是执行器的发送口）。

mock 下没有真抓包，用 mock 总线自己的写命令日志（`mock_writes.jsonl`）代替，
第三种问题变成"src != executor"的日志行（模拟另一个进程往 mock ECB 发包）。

用法：
    python3 tools/pcap_check.py runs/<会话> [--pcap-dir DIR] [--ecb 192.168.1.30]
                                  [--from 12 --to 20] [--tol-ms 300]

只读。退出码：0 = 两类问题都是 0；1 = 有对不上的。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

WRITE_CMDS = None    # 由 arm.pcapio 提供同一张表


def load_events(session: Path) -> list[dict]:
    out = []
    p = session / "events.jsonl"
    for ln in (p.read_text(encoding="utf-8").splitlines() if p.exists() else []):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def load_mock_writes(session: Path) -> list[dict]:
    out = []
    p = session / "mock_writes.jsonl"
    for ln in (p.read_text(encoding="utf-8").splitlines() if p.exists() else []):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--pcap-dir", default="", help="抓包目录（默认 会话/_pcap 或 runs/_pcap_ring）")
    ap.add_argument("--ecb", default="192.168.1.30")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--tol-ms", type=float, default=300.0)
    args = ap.parse_args(argv)
    d = Path(args.session)
    if not d.exists():
        print(f"没有这个会话目录：{d}", file=sys.stderr)
        return 2

    events = load_events(d)
    writes_ev = [e for e in events if e.get("kind") == "cmd"]
    # 会话开始的墙上时间：用来把 pcap 的 unix 时间换算成相对秒
    meta = {}
    try:
        meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        meta = {}
    start_ts = None
    try:
        import datetime as _dt
        start_ts = _dt.datetime.fromisoformat(
            str(meta.get("started"))).timestamp()
    except Exception:  # noqa: BLE001
        start_ts = None

    # mock 会话绝不能拿真抓包来对账（runs/_pcap_ring 里可能正有真机会话的包）
    is_mock = str(meta.get("connector") or "").lower().startswith("mock") or \
        bool(meta.get("mock"))
    pcap_files = []
    pcap_dir = None if is_mock else (Path(args.pcap_dir) if args.pcap_dir else None)
    if not is_mock and pcap_dir is None:
        for cand in (d / "_pcap", Path("runs/_pcap_ring")):
            if cand.exists():
                pcap_dir = cand
                break
    if pcap_dir is not None and pcap_dir.exists():
        pcap_files = sorted(pcap_dir.glob("*.pcap")) + \
            sorted(pcap_dir.glob("*.pcap*"))
        pcap_files = list(dict.fromkeys(pcap_files))

    source = "mock"
    packets = []
    if pcap_files:
        from arm import pcapio
        source = f"pcap {len(pcap_files)} 个文件"
        try:
            frames = pcapio.iter_write_frames(pcap_files, ecb_ip=args.ecb)
        except Exception as exc:  # noqa: BLE001
            print(f"读抓包失败：{exc}", file=sys.stderr)
            frames = []
        for ts, addr, cmd, data_hex, sport in frames:
            rel = (ts - start_ts) if start_ts else None
            packets.append({"rel": rel, "addr": addr, "cmd": cmd,
                            "data": data_hex, "sport": sport})
    else:
        for w in load_mock_writes(d):
            packets.append({"rel": None, "addr": None, "cmd": int(w.get("cmd", -1)),
                            "data": str(w.get("value")), "sport": -1,
                            "src": w.get("src", "executor"),
                            "mono": w.get("mono")})

    # ---- 1) events 里的写命令，抓包里有没有 ----
    missing_in_pcap = []
    if pcap_files:
        used = set()
        for e in writes_ev:
            cmd = int(e.get("cmd", -1))
            val = str(e.get("value"))
            lo = (e.get("mono") or 0.0) - args.tol_ms / 1000.0
            hi = (e.get("mono") or 0.0) + args.tol_ms / 1000.0
            hit = None
            for i, p in enumerate(packets):
                if i in used or p["cmd"] != cmd:
                    continue
                if p.get("rel") is not None and not (lo - 1.0 <= p["rel"] <= hi + 1.0):
                    continue
                hit = i
                break
            if hit is None:
                missing_in_pcap.append(e)
            else:
                used.add(hit)
    else:
        # mock：两边的时钟不同（events 是会话内相对秒，mock 日志是绝对 monotonic），
        # 按“顺序 + cmd + 值”对，不按时间
        used = set()
        j = 0
        for e in writes_ev:
            cmd = int(e.get("cmd", -1))
            val = str(e.get("value"))
            hit = None
            for i in range(j, len(packets)):
                if i in used or packets[i]["cmd"] != cmd:
                    continue
                if str(packets[i]["data"]) != val:
                    continue
                hit = i
                break
            if hit is None:
                # 再回头找一遍（允许跨过别的关节）
                for i, p in enumerate(packets):
                    if i in used or p["cmd"] != cmd or str(p["data"]) != val:
                        continue
                    hit = i
                    break
            if hit is None:
                missing_in_pcap.append(e)
            else:
                used.add(hit)
                j = hit + 1

    # ---- 2) 抓包里的写命令，events 里有没有 ----
    missing_in_events = [p for i, p in enumerate(packets) if i not in used]

    # ---- 3) 不是执行器发的 ----
    foreign = []
    if pcap_files:
        ports = {p["sport"] for p in packets}
        exec_port = min(ports) if ports else None
        foreign = [p for p in packets if p["sport"] != exec_port]
    else:
        foreign = [p for p in packets if p.get("src") != "executor"]

    print(f"# 会话 {d.name}；对账源：{source}")
    print(f"# events 写命令 {len(writes_ev)} 条；抓包写命令 {len(packets)} 条；"
          f"窗口 ±{args.tol_ms:g} ms")
    print(f"执行器记了但没发出去：{len(missing_in_pcap)}")
    for e in missing_in_pcap[:20]:
        print(f"  - {e.get('mono')}s {e.get('joint')} cmd=0x{int(e.get('cmd', 0)):02X} "
              f"值 {e.get('value')}：{str(e.get('text'))[:80]}")
    print(f"发出去了但执行器没记：{len(missing_in_events)}")
    for p in missing_in_events[:20]:
        print(f"  - {p.get('cmd')} addr={p.get('addr')} 数据 {p.get('data')}")
    print(f"不是执行器发的：{len(foreign)}")
    for p in foreign[:20]:
        print(f"  - 源 {'端口 ' + str(p.get('sport')) if p.get('sport') is not None else p.get('src')}"
              f" cmd=0x{p.get('cmd', 0):02X} 数据 {p.get('data')}")
    ok = not missing_in_pcap and not missing_in_events
    print("结果：", "对得上" if ok else "有对不上的")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
