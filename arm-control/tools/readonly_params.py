#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A8 / 21.5 只读工具：读六颗电机的环路参数（一个参数都不写）。

默认走执行器 HTTP 接口（A2 的规矩：执行器运行期间只有它和 ECB 通信）：
    python3 tools/readonly_params.py                  # 读参数
    python3 tools/readonly_params.py --timing         # 再量 read_all / set_targets_q 耗时
    python3 tools/readonly_params.py --direct --mock  # 不起服务，直接在进程里跑 mock 自检
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

BASE = "http://127.0.0.1:8770"


def http(method: str, path: str, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode("utf-8"))


def table(rows: dict):
    labels = []
    for name, row in rows.items():
        for k in row:
            if k not in labels:
                labels.append(k)
    print("关节 " + " | ".join(labels))
    for name, row in rows.items():
        cells = []
        for k in labels:
            item = row.get(k) or {}
            if item.get("raw_hex") is None:
                cells.append("—")
                continue
            raw = item.get("raw_hex")
            if "byte" in item:
                cells.append(f"{raw}→{item['byte']}")
            elif "iq24" in item:
                cells.append(f"{raw}→IQ24 {item['iq24']}")
            else:
                cells.append(raw)
        print(f"{name} " + " | ".join(cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--timing", action="store_true", help="量通信耗时（要二次确认）")
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()
    global BASE
    BASE = f"http://127.0.0.1:{args.port}"

    if args.direct:
        from arm.executor import Executor
        ex = Executor()
        ex.connect("mock" if args.mock else "hw")
        r = ex.readonly_params()
        table(r["rows"])
        if args.timing:
            t = ex.readonly_timing(args.n, True)
            print(json.dumps(t, ensure_ascii=False, indent=1))
        ex.disconnect()
        print("结果：过（只读工具跑通，没有写任何参数）")
        return 0

    try:
        cfg = http("GET", "/api/config")
    except urllib.error.URLError as exc:
        print(f"连不上执行器 {BASE}：{exc}")
        return 2
    cid = "readonly-tool"
    acq = http("POST", "/api/acquire",
               {"client_id": cid, "config_version": cfg.get("config_version")})
    token = acq.get("token")
    try:
        r = http("POST", "/api/readonly/params", {"token": token})
        table(r.get("rows") or {})
        if args.timing:
            t = http("POST", "/api/readonly/timing",
                     {"token": token, "n": args.n, "confirmed": True})
            print(json.dumps(t, ensure_ascii=False, indent=1))
    finally:
        try:
            http("POST", "/api/release", {"token": token})
        except Exception:  # noqa: BLE001
            pass
    if r.get("mock"):
        print("（mock：只验证工具走通，没有真机参数）")
    print("结果：过（只读工具跑通，没有写任何参数）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
