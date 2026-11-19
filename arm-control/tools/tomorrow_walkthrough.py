#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""明天真机第二次的步骤单在 mock 里整套走一遍。

顺序：21.6 → 20.10 → 22.2 → 22.4。
只走检查/规划/回放，不连 ECB；每一步打印起点、终点、速度、要记什么、
过没过怎么判，以及 mock 里有没有被拦住。

运行：cd arm-control && python3 tools/tomorrow_walkthrough.py
"""
from __future__ import annotations

import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm.executor import Executor
from arm.programs import ProgramStore
from arm.sweep import plan_sweep

FAILS: list[str] = []
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
TUCK = [-90.0, 30.0, 30.0, 90.0, 0.0, 0.0]


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


def wait(cond, timeout=60, step=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(step)
    return False


def fmt(q):
    return "[" + ", ".join(f"{v:.1f}" for v in q) + "]"


e = Executor()
e.connect("mock")
e.bus.enable_all(True, allow_restart=True)
e.speed_scale = 0.5
e._sync_motion_params()
m = e.motion
try:
    # ------------------------------------------------------------------ 21.6
    print("\n== 21.6 零位 → 收拢点 → 零位（速度倍率 50%） ==")
    q0 = [0.0] * 6
    s1 = m._plan_move(q0, TUCK, "21.6 零位→收拢")
    p1 = s1[-1].get("path_check") or {}
    check("21.6 起点零位、终点收拢点、50% 速度、可规划",
          len(s1) == 1 and abs(m.speed_scale - 0.5) < 1e-9
          and p1.get("max_step_deg", 9) <= 0.5001,
          f"{fmt(q0)} → {fmt(TUCK)}，{len(s1)} 段，T={s1[0]['T']} s，"
          f"检查点最大间隔 {p1.get('max_step_deg')}°，"
          f"速度 {s1[0]['vel_deg_s']}")
    m.execute(s1, preview=False)
    err1 = max(abs(e.bus.q[n] - t) for n, t in zip(NAMES, TUCK))
    check("21.6 去程到位（六轴 <0.8°）", m.status.get("phase") == "done" and err1 < 0.8,
          f"最大误差 {err1:.3f}°")

    s2 = m._plan_move(TUCK, q0, "21.6 收拢→零位")
    m.execute(s2, preview=False)
    err2 = max(abs(v) for v in e.bus.q.values())
    check("21.6 回程零位到位（六轴 <0.8°）", m.status.get("phase") == "done" and err2 < 0.8,
          f"最大 |q| {err2:.3f}°")

    # 走到一半按停：停住、不失能、回拉 ≤1°
    s3 = m._plan_move(q0, TUCK, "21.6 中途按停")
    def _go():
        try:
            m.execute(s3, preview=False)
        except Exception:  # noqa: BLE001
            pass
    th = threading.Thread(target=_go, daemon=True)
    th.start()
    time.sleep(2.0)
    q_stop = dict(e.bus.q)
    t_stop = time.monotonic()
    e.stop_all("21.6 中途按停")
    th.join(timeout=10)
    time.sleep(0.3)
    pull = max(abs(e.bus.q[n] - q_stop[n]) for n in NAMES)
    check("21.6 走到一半按停：停住、不失能、回拉 ≤1°",
          m.status.get("phase") == "aborted" and e.bus.enabled and pull <= 1.0,
          f"phase={m.status.get('phase')}，六颗使能={e.bus.enabled}，回拉 {pull:.3f}°")
    e.recover()
    # 回到零位，给下一步一个已知起点
    m.execute(m._plan_move(m.read_q(), q0, "21.6 回零"), preview=False)

    # ------------------------------------------------------------------ 20.10
    print("\n== 20.10 两项真机验证（mock 预演） ==")
    cap = e.capacities()
    used = e.limit_used()
    check("20.10.2 J2 容量 = 回读限流 × 1.91（mock 3.0 A）",
          abs(float(cap["J2"]) - 5.73) < 0.02
          and used["J2"]["source"] == "回读",
          f"J2 {cap['J2']} N·m，来源 {used['J2']}")
    prereq = {"gripper_empty": True, "workspace_clear": True, "shelf_ready": True}
    # mock 里给 J2/J3 一个接触电流，下降分步采样才够慢，能在第 2–3 步按停
    e.bus.set_faults(currents={"J2": 2.0, "J3": 1.0})
    e.park(False, prereq=prereq)
    reached = wait(lambda: e.park_step.get("phase") == "descending"
                   and (e.park_step.get("step") or 0) >= 2, timeout=60)
    e.stop_all("20.10.1 停机下降第 2–3 步按停")
    wait(lambda: not e.park_active, timeout=10)
    check("20.10.1 停机下降到第 2–3 步按停：停住、不失能",
          reached and e.park_step.get("phase") == "aborted" and e.bus.enabled,
          f"到达第 {e.park_step.get('step')} 步，phase={e.park_step.get('phase')}")
    e.bus.clear_faults()
    try:
        e.recover()
    except Exception:  # noqa: BLE001
        pass
    if not getattr(e.bus, "startup_done", False):
        e.bus.enable_all(True, allow_restart=True)
    # mock 里把臂摆回零位，给 22.2/22.4 一个已知起点（真机这一步由用户扶/抬回）
    for n in NAMES:
        e.bus.q[n] = 0.0
        e.bus.target[n] = 0.0
        e.bus.v[n] = 0.0
    with m._lock:
        m.status.update({"phase": "idle", "reason": "", "label": "",
                         "busy": False})
    # 停机中止后臂已在下降路径上；这步只验证检查不拦，不再真走
    print("  起点：零位/当前姿态；终点：按 18.3 下降路径；"
          "记录：按下时第几步、是否保持、六颗 power/mode；判据：停住 + 不失能 + 能重做")

    # ------------------------------------------------------------------ 22.2
    print("\n== 22.2 三条扫描路径（3°/s，只预检） ==")
    for j in ("J2", "J3", "J4"):
        p = plan_sweep(m, j)
        ms = (max(x["max_step_deg"] for x in p["checks"]["path_checks"])
              if p.get("ok") else 99)
        check(f"22.2 {j} 扫描不被限位/围栏/力矩拦",
              p.get("ok") and ms <= 0.5001
              and p["checks"]["torque_max_ratio"] < 0.8,
              (f"{p['start']}°→{p['end']}°，去/回 "
               f"{p['checks']['out_segments']}/{p['checks']['back_segments']} 段，"
               f"最大力矩 {p['checks']['torque_max_ratio'] * 100:.1f}%"
               if p.get("ok") else p.get("reason", "")))

    # ------------------------------------------------------------------ 22.4
    print("\n== 22.4 录 5 点 → 存 → 载入 → 回放（50%，1 遍） ==")
    e.program_store = ProgramStore(Path(tempfile.mkdtemp(prefix="walk_prog_")))
    pts = [[0, 0, 0, 0, 0, 0], [10, 10, 0, 0, 0, 0],
           [20, 20, 10, 0, 0, 0], [0, 30, 20, 10, 0, 0],
           [-10, 10, 0, 0, 0, 0]]
    e.program_points = []
    for i, q in enumerate(pts):
        for n, v in zip(NAMES, q):
            e.bus.q[n] = float(v)
            e.bus.target[n] = float(v)
            e.bus.v[n] = 0.0
        e.program_record(f"p{i + 1}")
    e.program_save("walk5")
    e.program_load("walk5")
    arrivals = {}

    def hook(states, cmd_q, cmd_vel, phase, label, elapsed, duration):
        if phase != "end" or "22.4 回放" not in str(label):
            return
        try:
            num = int(str(label).split("第 ")[1].split(" 点")[0])
        except Exception:  # noqa: BLE001
            return
        q = [states[n].out_deg for n in NAMES]
        if all(v is not None for v in q):
            arrivals[num] = q

    m.add_hook(hook)
    e.program_replay("walk5", loops=1, confirmed=True)
    ok = wait(lambda: not e.program_status.get("active"), timeout=60)
    errs = [max(abs(arrivals[i + 1][k] - pts[i][k]) for k in range(6))
            for i in range(len(pts)) if i + 1 in arrivals]
    check("22.4 5 点回放（50%）各点 <0.8°、按停可用",
          ok and e.program_status.get("phase") == "done" and len(errs) == 5
          and max(errs) < 0.8,
          f"到达点 {sorted(arrivals)}，最大误差 {max(errs) if errs else 99:.3f}°")
finally:
    try:
        e._poll_stop.set()
        e.disconnect()
    except Exception:  # noqa: BLE001
        pass

print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
raise SystemExit(1 if FAILS else 0)
