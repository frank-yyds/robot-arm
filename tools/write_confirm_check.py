#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.7 第 7 条：mock 注入 10% 写目标丢包，确认每一颗都被发现并重发。

另量 1000 次写目标排队时间（mock）。
"""
from __future__ import annotations

import random
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def main():
    ex = Executor()
    ex.connect("mock")
    bus = ex.bus
    bus.drop_write_prob = 0.1
    rng = random.Random(20261118)
    d0, r0 = bus.write_dropped, bus.write_resent
    rounds = 200
    for _ in range(rounds):
        q0 = dict(bus.q)
        # 先把 mock 停稳，免得上一拍的残余速度让“丢包”看起来像在走
        bus.target = dict(q0)
        bus.v = {n: 0.0 for n in bus.order}
        tg = {n: q0[n] + rng.choice([-10.0, 10.0]) for n in bus.order}
        bus.set_targets_q(tg, confirm=True)
    dropped = bus.write_dropped - d0
    resent = bus.write_resent - r0
    # 队列时间
    q0 = dict(bus.q)
    times = []
    for _ in range(1000):
        t = time.monotonic()
        bus.set_targets_q({n: q0[n] for n in bus.order}, confirm=False)
        times.append((time.monotonic() - t) * 1000.0)
    times.sort()
    checks = [
        ("10% 写丢被注入", dropped > 0, f"丢 {dropped} 颗次"),
        ("全部发现并重发", dropped == resent, f"重发 {resent} 颗次"),
        ("没有'以为写了其实没写'", dropped == resent and resent > 0,
         f"{resent}/{dropped}"),
        ("1000 次写目标排队时间 ≤50 ms",
         times[-1] <= 50.0,
         f"中位 {times[500]:.3f} / 99 {times[990]:.3f} / 最大 {times[-1]:.3f} ms"),
    ]
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    ex.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
