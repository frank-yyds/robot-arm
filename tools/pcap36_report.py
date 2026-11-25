#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36.5：抓包按源端口再对一遍（只看抓包，不碰真机）。

1. 按本机源端口分组：每个端口第一个/最后一个包的时刻、对应会话、有没有对不上会话的端口；
2. 每个会话只拿它自己的端口对账：记了没发 / 发了没记各多少条；
3. 把按端口分组做进 pcap_check（本工具调用它，确认新报告格式）。
结果写 runs/<日期>_36/pcap_by_port.json。
"""
from __future__ import annotations

import contextlib
import io
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm import pcapio  # noqa: E402
from tools import pcap_check  # noqa: E402

RING = Path("runs/_pcap_ring")
ECB = "192.168.1.30"


def wall(s: str) -> datetime | None:
    try:
        return datetime.fromisoformat(s)
    except Exception:  # noqa: BLE001
        return None


def session_windows():
    out = []
    for d in sorted(Path("runs").glob("20261124_*_hw")):
        meta = {}
        try:
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            pass
        st = wall(str(meta.get("started"))) if meta.get("started") else None
        if st is None:
            continue
        last = None
        p = d / "events.jsonl"
        if p.exists():
            for ln in p.read_text(encoding="utf-8").splitlines():
                if not ln.strip():
                    continue
                try:
                    e = json.loads(ln)
                except Exception:  # noqa: BLE001
                    continue
                w = wall(str(e.get("wall") or ""))
                if w:
                    last = w
        out.append({"session": d.name, "start": st, "end": last, "dir": d})
    return out


def main():
    files = sorted(RING.glob("*.pcap*"))
    frames = list(pcapio.iter_write_frames(files, ecb_ip=ECB))
    ports = {}
    for ts, addr, cmd, data, sport in frames:
        row = ports.setdefault(sport, {"first": ts, "last": ts, "n": 0})
        row["n"] += 1
        row["first"] = min(row["first"], ts)
        row["last"] = max(row["last"], ts)
    sessions = session_windows()
    result = {"ring_files": [str(f) for f in files], "frames": len(frames),
              "ports": [], "sessions": [], "orphan_ports": []}
    for sport, row in sorted(ports.items(), key=lambda kv: kv[1]["first"]):
        f = datetime.fromtimestamp(row["first"])
        l = datetime.fromtimestamp(row["last"])
        matched = [s["session"] for s in sessions
                   if s["start"] and s["end"]
                   and l >= s["start"] and f <= s["end"]]
        result["ports"].append({
            "sport": sport, "n": row["n"],
            "first": f.strftime("%H:%M:%S.%f")[:-3],
            "last": l.strftime("%H:%M:%S.%f")[:-3],
            "sessions": matched})
        if not matched:
            result["orphan_ports"].append({"sport": sport, "n": row["n"],
                                           "first": f.strftime("%H:%M:%S"),
                                           "last": l.strftime("%H:%M:%S")})
    for s in sessions:
        buf = io.StringIO()
        started = s["start"].strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        try:
            with contextlib.redirect_stdout(buf):
                pcap_check.main([str(s["dir"]), "--pcap-dir", str(RING),
                                 "--ecb", ECB])
        except SystemExit:
            pass
        txt = buf.getvalue()
        def num(label):
            m = re.search(re.escape(label) + r"：(\d+)", txt)
            return int(m.group(1)) if m else None
        my_ports = [p["sport"] for p in result["ports"]
                    if s["session"] in p["sessions"]]
        result["sessions"].append({
            "session": s["session"],
            "started": started,
            "ports": my_ports,
            "recorded_not_sent": num("执行器记了但没发出去"),
            "sent_not_recorded": num("发出去了但执行器没记"),
            "foreign": num("不是执行器发的"),
            "output": txt.strip().splitlines()[0:3],
        })
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_36" / "pcap_by_port.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")
    print(f"抓包 {len(files)} 个文件、{len(frames)} 条写命令；端口 {len(ports)} 个")
    for row in result["ports"]:
        print(f"  端口 {row['sport']}：{row['n']} 条 {row['first']} → {row['last']} "
              f"会话 {row['sessions'] or '对不上'}")
    print("按会话对账：")
    for row in result["sessions"]:
        print(f"  {row['session']} {row['started']} 端口 {row['ports']} "
              f"记了没发 {row['recorded_not_sent']} 发了没记 {row['sent_not_recorded']} "
              f"非执行器 {row['foreign']}")
    print(f"对不上任何会话的端口：{result['orphan_ports']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
