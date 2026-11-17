#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A6 自检（mock）：报警锁存。

1. 报警位非 0：先清、再读，回 0 → 通过；
2. 清了还在：拒绝运动并点名。
（A7 的“第一次使能无应答单独重试”在真机 B1 上验；代码在 JointBus.enable_all。）

用法：
    cd arm-control && python3 tools/a6_latch_check.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def main():
    ex = Executor()
    ex.connect("mock")
    # 1) 锁存 0x0200，clear 后应回 0
    ex.bus.set_faults(error_bits={"J4": 0x0200})
    r1 = ex.check_alarm_latch()
    j4_after = ex.bus._errors.get("J4", 0)
    ok1 = r1["ok"] and j4_after == 0
    # 2) 清了还在 → 拒绝
    ex.bus.set_faults(error_bits={"J5": 0x0004})
    m5 = ex.bus.motors["J5"]
    orig_clear = m5.clear_error
    m5.clear_error = lambda: True          # 假装清了但报警还在
    try:
        r2 = ex.check_alarm_latch()
    finally:
        m5.clear_error = orig_clear
    ok2 = (not r2["ok"]) and any("J5" in x for x in r2["bad"])
    ex.bus.clear_faults()
    checks = [
        ("报警 0x0200 先清再读 → 通过", ok1,
         f"ok={r1['ok']} J4 清后=0x{j4_after:04X}"),
        ("清了还在 → 拒绝并点名 J5", ok2,
         f"ok={r2['ok']} bad={r2['bad']}"),
    ]
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    ok = all(c[1] for c in checks)
    print("结果：", "过" if ok else "没过")
    ex.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
