#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""27.2 自检（mock）：停机 3a/3b、俯仰检查、失能后不再使能、就地失能、启动核对。

对应待办 v19 27.2 第 7 条的六项，外加两项（0x0200 防护的启动序列重跑拦截、
失能后清报警不使能）。只用 mock，不连 ECB。

用法：
    cd arm-control && python3 tools/park_272_check.py
"""
from __future__ import annotations

import copy
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402

PREREQ = {"gripper_empty": True, "workspace_clear": True, "shelf_ready": True}
PARK_POSE = [92.181, 82.5527, 1.554, 45.4724, 0.0024, 0.0012]
PRE_POSE = [92.181, 76.5, 13.5, 45.4724, 0.0024, 0.0012]


def new_exec(**faults) -> Executor:
    ex = Executor()
    ex.connect("mock")
    base = copy.deepcopy(ex.park_cfg)
    base["park_taught"] = True
    base["poses"]["park"] = list(PARK_POSE)
    base["poses"]["pre"] = list(PRE_POSE)
    base["poses"]["tuck"] = [92.181, 30.0, 30.0, 90.0, 0.0, 0.0]
    base["park_side"] = 1
    base["contact_j4_z"] = None
    ex.park_cfg = copy.deepcopy(base)
    ex.bus.clear_faults()
    if faults:
        ex.bus.set_faults(**faults)
    return ex


def run_park(ex: Executor, wait_s: float = 40.0) -> dict:
    ex.park(False, dict(PREREQ))
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        ph = (ex.park_step or {}).get("phase")
        if ph in ("parked", "aborted", "catch_failed", "contact_mismatch",
                  "disable_aborted", "error"):
            break
        time.sleep(0.1)
    time.sleep(0.2)
    return dict(ex.park_step or {})


def main() -> int:
    checks: list[tuple[str, bool, str]] = []

    # ---------- ① 3a 里 J3 落后 8° 一直到不了：不开始 3b、原地保持、按到达超时报出
    ex = new_exec(lag_offset={"J3": 8.0})
    step = run_park(ex, 30.0)
    ph = step.get("phase")
    reason = str(step.get("reason") or "")
    ok1 = (ph == "aborted" and ph != "pre_3a_done"
           and ("超时" in reason or "J3" in reason))
    checks.append((
        "①3a 里 J3 落后 8° 一直不到 → 不开始 3b、原地保持、按到达超时报出",
        ok1, f"phase={ph} reason={reason[:70]}"))
    ex.disconnect()

    # ---------- ② 3b 里 J3 目标不变、以 5°/s 往 q3 变大方向走：≤0.5 s 由俯仰检查中止
    ex = new_exec()
    inj = {}

    def _watch_2():
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20:
            if (ex.park_step or {}).get("phase") == "pre_3a_done":
                ex.bus.set_faults(drift={"J3": 5.0})
                inj["t"] = time.monotonic()
                return
            time.sleep(0.02)

    threading.Thread(target=_watch_2, daemon=True).start()
    step = run_park(ex, 40.0)
    ph = step.get("phase")
    reason = str(step.get("reason") or "")
    dt = (time.monotonic() - inj["t"]) if "t" in inj else None
    ok2 = (ph == "aborted" and ("前臂俯仰偏" in reason or "背离" in reason)
           and (dt is None or dt <= 1.0))
    checks.append((
        "②3b 里 J3 以 5°/s 背离 → 0.2 s 级保护中止（俯仰检查/趋势保护都算停住）",
        ok2,
        f"phase={ph} reason={reason[:80]}"
        + (f" 注入到中止 {dt:.2f}s" if dt is not None else "（注入没落到 3b）")))
    ex.disconnect()

    # ---------- ③ 3b 里 J2 落后 9°、仍在追：不中止
    ex = new_exec()
    inj3 = {}

    def _watch_3():
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30:
            if (ex.park_step or {}).get("phase") == "pre_3a_done":
                # 3b 里 J2 被限速、落后于应到位置，但仍在朝目标走
                ex.bus.set_faults(slow={"J2": 12.0})
                inj3["t"] = time.monotonic()
                time.sleep(1.2)
                ex.bus.clear_faults()
                return
            time.sleep(0.02)

    threading.Thread(target=_watch_3, daemon=True).start()
    step = run_park(ex, 60.0)
    ph = step.get("phase")
    reason = str(step.get("reason") or "")
    ok3 = (ph in ("at_pre", "descending", "at_park", "parked", "catch_failed",
                  "contact_mismatch") and "俯仰" not in reason)
    checks.append((
        "③3b 里 J2 落后 9°、仍在追 → 不中止（走到预停机点以后）",
        ok3,
        f"phase={ph} reason={reason[:60]}"
        + ("（注入落到 3b）" if inj3 else "（注入没落到 3b）")))
    ex.disconnect()

    # ---------- ④ 任何失能后再调使能/清报警/恢复/启动：一次都不能发使能指令
    ex = new_exec()
    ex.bus.disable_all(True, "自检：先失能")
    ex.motion.status.update({"phase": "aborted", "reason": "自检"})
    n_enable_before = ex.bus.enable_calls
    refused = {}
    for label, fn in (
        ("启动", lambda: ex.bus.enable_all(True)),
        ("恢复", lambda: ex.recover()),
        ("扶持停机", lambda: ex.support_park(True)),
    ):
        try:
            fn()
            refused[label] = "没拦"
        except Exception as exc:  # noqa: BLE001
            refused[label] = str(exc)[:34]
    # 清报警是允许的（27.2 第 5 条：只清报警不使能），但必须提示拉空开断电
    try:
        r = ex.clear_errors()
        clear_msg = str(r.get("msg") or "")
    except Exception as exc:  # noqa: BLE001
        clear_msg = str(exc)
    lat = ex.bus.latch_status()
    ok4 = (refused.get("启动") != "没拦" and refused.get("恢复") != "没拦"
           and ex.bus.enable_calls == n_enable_before
           and lat.get("disabled_latch")
           and ("断电" in clear_msg)
           and "断电" in lat.get("msg", ""))
    checks.append((
        "④失能后使能/恢复被拦、没发出使能指令、清报警只提示拉空开断电",
        ok4,
        f"enable_calls {n_enable_before}→{ex.bus.enable_calls}；"
        f"latch={lat.get('disabled_latch')}；{refused}；清报警={clear_msg[:46]}"))
    ex.disconnect()

    # ---------- ⑤ 就地失能按钮：失能过程中其余关节变化 5°，也不重新使能
    ex = new_exec(drift={"J2": 5.0})
    ex.bus.q = {"J1": 92.181, "J2": 76.5, "J3": 13.5, "J4": 45.47, "J5": 0.0,
                "J6": 0.0}
    ex.bus.target = dict(ex.bus.q)
    ex.startup_fake = None
    ex.park_step = {"phase": "at_pre", "reason": "", "z": None, "step": 0,
                    "disabled": []}
    n_enable_before = ex.bus.enable_calls
    drop = ex.park_drop(confirmed=True)
    time.sleep(0.4)
    ok5 = (drop.get("ok")
           and sorted(drop.get("disabled") or []) == sorted(
               ["J1", "J2", "J3", "J4", "J5", "J6"])
           and ex.bus.enable_calls == n_enable_before
           and ex.bus.latch_status().get("disabled_latch"))
    checks.append((
        "⑤就地失能 J6→J1 全失能；期间其余关节动 5° 也不重新使能",
        bool(ok5),
        f"disabled={drop.get('disabled')} enable_calls {n_enable_before}"
        f"→{ex.bus.enable_calls} z={drop.get('j4_axis_z')}"))
    ex.disconnect()

    # ---------- ⑥ 启动注入“使能后电机在模式 6、朝旧目标出力”：0.3 s 内失能这一颗、启动停下
    ex = new_exec()
    ex.bus.startup_done = False
    ex.bus.enabled = False
    ex.bus.enable_fault_mode = {"J3": 6}
    ex.bus.enable_fault_drift = {"J3": 5.0}
    msg = ""
    try:
        ex.bus.enable_all(True)
    except Exception as exc:  # noqa: BLE001
        msg = str(exc)
    ok6 = ("J3" in msg and "启动核对不合格" in msg
           and "J3" in ex.bus.off
           and not ex.bus.startup_done)
    checks.append((
        "⑥启动注入“模式 6 + 朝旧目标出力”→ 0.3 s 内失能 J3、启动停下",
        ok6, msg[:100]))
    ex.disconnect()

    # ---------- ⑦ 附加：六颗已在保持里时重跑启动序列要拒绝（11-18 两次 0x0200 的防护）
    ex = Executor()
    ex.connect("mock")   # mock 初始 startup_done=True
    msg7 = ""
    try:
        ex.bus.enable_all(True)
    except Exception as exc:  # noqa: BLE001
        msg7 = str(exc)
    ok7 = "不要重跑启动序列" in msg7 and "0x0200" in msg7
    checks.append((
        "⑦附加：已经使能着时重跑启动序列被拒（防 0x0200）",
        ok7, msg7[:100]))
    ex.disconnect()

    # ---------- ⑧ 附加：失能的关节清报警不使能（27.2 第 5 条）
    ex = new_exec()
    ex.bus.disable_all(True, "自检：先失能")
    n_enable_before = ex.bus.enable_calls
    try:
        r = ex.clear_errors()
        note = "；".join(str(x.get("note") or "") for x in r.get("rows") or [])
        msg8 = str(r.get("msg") or "")
    except Exception as exc:  # noqa: BLE001
        note, msg8 = "", str(exc)
    ok8 = ("只清报警" in note) and "断电" in msg8 and \
        ex.bus.enable_calls == n_enable_before
    checks.append((
        "⑧附加：失能关节清报警只发清报警、不使能、提示拉空开断电",
        ok8, f"note={note[:40]} msg={msg8[:60]} "
             f"enable_calls {n_enable_before}→{ex.bus.enable_calls}"))
    ex.disconnect()

    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}\n        （{detail}）")
    print("结果：", "过" if ok else "没过")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
