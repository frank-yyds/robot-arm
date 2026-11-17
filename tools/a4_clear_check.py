#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A4 自检（mock）：电源回包丢 50% 时清报警 200 次。

要求：一次都不能出现“切模式”或“失能”；读不到电源状态的那颗不动并报出来；
读不到时不能把 startup_done 置假（只有确认电源≠1/模式≠6 才置假）。

用法：
    cd arm-control && python3 tools/a4_clear_check.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def main():
    ex = Executor()
    ex.connect("mock")
    ex.bus.enabled = True
    ex.bus.startup_done = True
    ex.bus.set_faults(drop={"power": 0.5})
    mode0 = ex.bus.mode_writes
    dis0 = ex.bus.disable_calls
    bad = 0
    skipped = 0
    t0 = time.monotonic()
    for _ in range(200):
        try:
            r = ex.clear_errors()
        except Exception as exc:  # noqa: BLE001
            print("没过：clear_errors 抛异常", exc)
            return 1
        for row in r.get("rows", []):
            if row.get("error"):
                if "读不到电源状态" in str(row["error"]):
                    skipped += 1
                else:
                    bad += 1
            elif row.get("after") not in (None, "0x0000"):
                bad += 1
    dt = time.monotonic() - t0
    ex.bus.clear_faults()
    st = ex.state()
    checks = [
        ("200 次里没有切模式", ex.bus.mode_writes == mode0,
         f"mode_writes +{ex.bus.mode_writes - mode0}"),
        ("200 次里没有失能", ex.bus.disable_calls == dis0,
         f"disable_calls +{ex.bus.disable_calls - dis0}"),
        ("读不到电源的那颗跳过并报出来（不静默）", skipped > 0,
         f"跳过 {skipped} 颗次"),
        ("startup_done 没被置假", bool(st.get("startup_done")),
         str(st.get("startup_done"))),
        ("200 次里没有整体失败", bad == 0, f"失败 {bad} 次"),
    ]
    ok = all(c[1] for c in checks)
    print(f"A4 清报警自检（电源回包丢 50%，200 次，{dt:.2f} s）")
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    ex.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
