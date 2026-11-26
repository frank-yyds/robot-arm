#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36.6 Q6 / Y14：今天（11-26）全部真机会话的抓包对账 + 按端口分组。

数据来源：执行器自己的转写（runs/_pcap_ring/ecb_tee_*.pcap，不用 sudo）。
结果写 runs/<日期>_36/pcap_1126.json。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm import pcapio  # noqa: E402


def main() -> int:
    tee = sorted(Path("runs/_pcap_ring").glob("ecb_tee_*.pcap"))
    out = {"files": [], "sessions": []}
    for f in tee:
        frames = pcapio.iter_write_frames([f], ecb_ip="192.168.1.30")
        ports = {}
        for ts, addr, cmd, data, sport in frames:
            row = ports.setdefault(sport, {"n": 0, "first": ts, "last": ts, "cmds": {}})
            row["n"] += 1
            row["first"] = min(row["first"], ts)
            row["last"] = max(row["last"], ts)
            row["cmds"][hex(cmd)] = row["cmds"].get(hex(cmd), 0) + 1
        out["files"].append({
            "file": f.name, "bytes": f.stat().st_size,
            "writes": len(frames),
            "ports": {str(k): {
                "n": v["n"],
                "first": time.strftime("%H:%M:%S", time.localtime(v["first"])),
                "last": time.strftime("%H:%M:%S", time.localtime(v["last"])),
                "cmds": v["cmds"]} for k, v in ports.items()},
        })
    # 每个会话跑一遍 pcap_check
    for d in sorted(Path("runs").glob("20261126_*_hw")):
        import subprocess
        r = subprocess.run([sys.executable, "tools/pcap_check.py", str(d)],
                           capture_output=True, text=True, timeout=900)
        lines = r.stdout.strip().splitlines()
        out["sessions"].append({"session": d.name, "rc": r.returncode,
                                "output": lines})
    p = Path("runs/20261126_36")
    p.mkdir(parents=True, exist_ok=True)
    (p / "pcap_1126.json").write_text(json.dumps(out, ensure_ascii=False, indent=1),
                                      encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
