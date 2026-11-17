#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A3b 自检（mock）：注入“J3 不听目标，以 13°/s 往一个方向走”。

要求：1 s 内报保持失效；其余五颗保持；没有关节被自动失能；
网页状态里带 hold_failure（红色横幅用）；失效后拒绝新的运动命令。

用法：
    cd arm-control && python3 tools/hold_fail_check.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def wait_idle(ex, timeout=20.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        m = ex.motion.status
        if not m.get("busy") and m.get("phase") in ("done", "arrived", "idle"):
            return True
        time.sleep(0.02)
    return False


def main():
    ex = Executor()
    ex.connect("mock")
    ex.motion.speed_scale = 1.0
    # 先走到一个非零姿态，让六颗都有保持目标
    ex.move_joints([0.0, 10.0, 10.0, 0.0, 0.0, 0.0])
    if not wait_idle(ex):
        print("没过：mock 运动没有到位")
        return 1
    targets_before = dict(getattr(ex.bus, "_targets", {}))
    off_before = set(ex.bus.off)
    disables_before = ex.bus.disable_calls
    ex.bus.set_faults(drift={"J3": -13.0})
    t0 = time.monotonic()
    hf = None
    while time.monotonic() - t0 < 2.0:
        if ex.hold_failure:
            hf = dict(ex.hold_failure)
            break
        time.sleep(0.01)
    dt = time.monotonic() - t0
    ex.bus.clear_faults()
    st = ex.state()
    checks = []
    checks.append(("1 s 内报保持失效", bool(hf) and dt <= 1.0, f"{dt:.2f} s"))
    checks.append(("报的是 J3", bool(hf) and hf.get("joint") == "J3",
                   (hf or {}).get("joint")))
    checks.append(("其余五颗仍在保持（没被自动失能）",
                   set(ex.bus.off) == off_before
                   and ex.bus.disable_calls == disables_before
                   and all(targets_before.get(n) is not None
                           for n in ex.robot.joint_names if n != "J3"),
                   f"off={sorted(ex.bus.off)} disable_calls+{ex.bus.disable_calls - disables_before}"))
    checks.append(("state 里有 hold_failure", bool(st.get("hold_failure")),
                   "有" if st.get("hold_failure") else "无"))
    blocked = False
    try:
        ex._ready_for_motion()
    except Exception as exc:  # noqa: BLE001
        blocked = "保持失效" in str(exc)
    checks.append(("失效后拒绝新运动命令", blocked, "拒绝" if blocked else "仍然接受"))
    # 单颗失能要二次确认
    refused = False
    try:
        ex.hold_disable_joint("J3", confirmed=False)
    except Exception:  # noqa: BLE001
        refused = True
    checks.append(("单颗失能要二次确认", refused, "拒绝" if refused else "没拦"))
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    ex.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
