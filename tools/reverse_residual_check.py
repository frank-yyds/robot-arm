#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""35.2 第 3 条：残留速度模型重放 + 第 1、2 条自检（mock，不碰真机）。

用法：python3 tools/reverse_residual_check.py
结果：结果写 runs/<日期>_35/residual_check.json，控制台打印每项过/不过。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.mock import MockJointBus  # noqa: E402


def settle(bus, timeout=8.0, dt=0.01):
    """跑到各关节速度≈0 且残留在空。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        bus._tick()
        if not bus.residual and all(abs(v) < 0.05 for v in bus.v.values()):
            # 再多跑两拍让位置收敛
            bus._tick(); bus._tick()
            return True
        time.sleep(dt)
    return False


def measure_reverse(bus, joint, window=1.2):
    """写目标之后固定 1.2 s 窗口里的最大反向；随后再 settle。"""
    q0 = bus.q[joint]
    sign = None
    rev = 0.0
    rev_at = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        bus._tick()
        t = time.monotonic() - t0
        tgt = bus._targets.get(joint)
        if sign is None and tgt is not None:
            sign = 1.0 if tgt >= q0 else -1.0
        if sign is not None:
            b = (bus.q[joint] - q0) * sign
            if b < rev:
                rev = b
                rev_at = t
        time.sleep(0.005)
    settle(bus, timeout=8.0)
    return rev, rev_at


def replay(name, seq, expect):
    """seq: [(vel, acc, target)]，target 是这一段的绝对目标。"""
    bus = MockJointBus()
    bus.enable_all(True, allow_restart=True)
    bus.residual_model = True
    # 真机起点：J3 在 30°
    bus.q["J3"] = 30.0
    bus.target["J3"] = 30.0
    bus._targets["J3"] = 30.0
    # 真机第一段之前：上一段是向上的移动（a/b 组之前刚转到起点），按 5°/s 算
    bus._prev_dir["J3"] = 1.0
    bus._plan_vel["J3"] = float(seq[0][0])
    out = []
    j = "J3"
    for vel, acc, target in seq:
        q0 = bus.q[j]
        disp = target - q0
        bus.set_profiles_q({j: vel}, {j: acc}, disp={j: disp})
        bus.set_targets_q({j: target})
        rev, rev_at = measure_reverse(bus, j)
        pred = None
        if bus._residual_log:
            pred = bus._residual_log[-1]["predicted_rev_deg"]
        out.append({"vel": vel, "acc": acc, "disp": round(disp, 2),
                    "predicted_rev_deg": pred, "measured_rev_deg": round(rev, 3),
                    "rev_at_s": None if rev_at is None else round(rev_at, 2)})
    ok = all(abs(abs(o["measured_rev_deg"]) - o["predicted_rev_deg"]) <=
             0.3 * o["predicted_rev_deg"]
             for o in out if o["predicted_rev_deg"])
    return {"name": name, "ok": ok, "expect": expect, "rows": out}


def check_hold_window():
    """第 1 条自检：3b 中止，J3 在窗口里、残留 2.4°/s → 位置变化 ≤0.1°。"""
    bus = MockJointBus()
    bus.enable_all(True, allow_restart=True)
    bus.residual_model = True
    j = "J3"
    bus.q[j] = 13.56
    bus.target[j] = 13.5
    bus._targets[j] = 13.5
    bus.set_profiles_q({j: 0.5}, {j: 0.5}, disp={j: 0.0})   # item2：不动的关节给下限
    bus.residual[j] = {"v": 2.4, "dir": -1.0, "a": float(bus.amax[j]),
                       "moved": 0.0, "rev": 2.4 * 2.4 / (2.0 * float(bus.amax[j]))}
    q_before = bus.q[j]
    r = bus.hold_all()
    settle(bus, timeout=3.0)
    drift = abs(bus.q[j] - q_before)
    return {"name": "第 1 条 窗口中不改写目标", "ok": drift <= 0.1,
            "drift_deg": round(drift, 4), "kept": r.get("kept")}


def check_accel_floor():
    """第 2 条自检：故意让第 1 条不生效，J3 残留 2.4°/s、加速度下限 40°/s² → 漂移 ≤0.2°。"""
    bus = MockJointBus()
    bus.enable_all(True, allow_restart=True)
    bus.residual_model = True
    j = "J3"
    bus.q[j] = 13.56
    bus.target[j] = 13.5
    bus._targets[j] = 13.5
    bus.set_profiles_q({j: 0.5}, {j: 0.5}, disp={j: 0.0})  # item2：a=40
    bus.residual[j] = {"v": 2.4, "dir": -1.0, "a": float(bus.amax[j]), "moved": 0.0,
                       "rev": 2.4 * 2.4 / (2.0 * 40.0)}
    q_before = bus.q[j]
    # 故意改写目标（绕过第 1 条）
    bus.target[j] = bus.q[j]
    bus._targets[j] = bus.q[j]
    settle(bus, timeout=3.0)
    drift = abs(bus.q[j] - q_before)
    return {"name": "第 2 条 加速度下限后漂移", "ok": drift <= 0.2,
            "drift_deg": round(drift, 4), "amax": round(float(bus.amax[j]), 1)}


def main():
    out = {"checks": {}}
    out["checks"]["replay_25"] = replay(
        "a/b 25% 半速重放", [(5.0, 10.0, 13.5), (5.0, 10.0, 30.0),
                              (5.0, 10.0, 13.5), (5.0, 10.0, 30.0)], [1.25, 1.25])
    out["checks"]["replay_50"] = replay(
        "a/b 50% 重放", [(10.0, 20.0, 13.5), (10.0, 20.0, 30.0),
                          (10.0, 20.0, 13.5), (10.0, 20.0, 30.0)], [2.5, 2.5])
    out["checks"]["replay_50_first"] = replay(
        "50% 第 1 段（上一段 25%）", [(5.0, 10.0, 30.0), (10.0, 20.0, 13.5)], [0.625])
    out["checks"]["hold_window"] = check_hold_window()
    out["checks"]["accel_floor"] = check_accel_floor()
    out["ok"] = all(v.get("ok") for v in out["checks"].values())
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_35" / "residual_check.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")
    for k, v in out["checks"].items():
        print(f"  {'过' if v.get('ok') else '没过'}：{v.get('name')}")
        for row in v.get("rows") or []:
            print(f"      v={row['vel']} a={row['acc']} disp={row['disp']} "
                  f"推算 {row['predicted_rev_deg']} 实测 {row['measured_rev_deg']} @{row['rev_at_s']}s")
        if "drift_deg" in v:
            print(f"      漂移 {v['drift_deg']}°")
    print("结果：", "过" if out["ok"] else "没过")
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
