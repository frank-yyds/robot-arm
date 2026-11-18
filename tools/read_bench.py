#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.7 第 5 条：不同提交版本的 read_all 重试/耗时对比。

用法（臂在搁架上、执行器已停；脚本会零扭矩使能，不运动）：
    python3 tools/read_bench.py --root /tmp/arm_da92 --label da92bf1
"""
from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--n", type=int, default=100)
    args = ap.parse_args()
    sys.path.insert(0, args.root)
    from arm.executor import Executor  # noqa: E402

    ex = Executor()
    ex.connect("hw")
    time.sleep(0.3)
    try:
        ex.bus.enable_all(True)
    except Exception as exc:  # noqa: BLE001
        print("enable failed", exc)
        return 1
    time.sleep(0.5)
    # 单线程
    tb = ex.bus.link.timeouts
    t0 = time.monotonic()
    for _ in range(args.n):
        ex.bus.read_all(full=False)
    single = (time.monotonic() - t0) / args.n * 1000
    to_single = ex.bus.link.timeouts - tb
    # 带并发状态读（模拟执行器轮询 + 5 Hz 状态）
    stop = threading.Event()
    def status_loop():
        while not stop.is_set():
            try:
                ex.bus.read_status()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.2)
    th = threading.Thread(target=status_loop, daemon=True)
    th.start()
    tb = ex.bus.link.timeouts
    t0 = time.monotonic()
    for _ in range(args.n):
        ex.bus.read_all(full=False)
    conc = (time.monotonic() - t0) / args.n * 1000
    to_conc = ex.bus.link.timeouts - tb
    stop.set(); th.join(timeout=1)
    print(f"{args.label or args.root}: 单线程 {single:.1f} ms/次（超时 {to_single}）; "
          f"并发状态 {conc:.1f} ms/次（超时 {to_conc}）")
    ex.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
