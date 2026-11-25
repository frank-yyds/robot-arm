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


def replay(name, seq, expect, decel_negative=True, q0_deg=30.0):
    """seq: [(vel, acc, target)]，target 是这一段的绝对目标。

    36.1 第 4 条：decel_negative=True 时每段走完规划速度归零、下一段不带残留，
    推算反向应当没有；False 时走 35.2 的旧模型（正数减速度，每段带残留）。
    """
    bus = MockJointBus()
    bus.enable_all(True, allow_restart=True)
    bus.set_decel_negative(decel_negative)
    bus.residual_model = True
    # 真机起点：J3 在 30°
    bus.q["J3"] = float(q0_deg)
    bus.target["J3"] = float(q0_deg)
    bus._targets["J3"] = float(q0_deg)
    # 真机第一段之前：上一段是向上的移动（a/b 组之前刚转到起点），按 5°/s 算
    bus._prev_dir["J3"] = 1.0
    bus._plan_vel["J3"] = 5.0
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
    if decel_negative:
        # 全负数：模型不注入残留，每段（含第 1 段）实测反向都应当 <0.1°
        preds = [o["predicted_rev_deg"] for o in out if o["predicted_rev_deg"]]
        ok = (not preds) and all(abs(o["measured_rev_deg"]) < 0.1 for o in out)
    else:
        ok = all(abs(abs(o["measured_rev_deg"]) - o["predicted_rev_deg"]) <=
                 0.3 * o["predicted_rev_deg"]
                 for o in out if o["predicted_rev_deg"])
    return {"name": name, "ok": ok, "expect": expect,
            "decel_negative": bool(decel_negative), "rows": out}


def check_switch_carry():
    """36.1 第 4 条：正数→负数切换后的那一段仍带上一段残留，再下一段就没有。"""
    bus = MockJointBus()
    bus.enable_all(True, allow_restart=True)
    bus.residual_model = True
    j = "J3"
    bus.q[j] = 30.0
    bus.target[j] = 30.0
    bus._targets[j] = 30.0
    bus._prev_dir[j] = 1.0
    bus._plan_vel[j] = 5.0

    def seg(target, vel=5.0, acc=10.0):
        q0 = bus.q[j]
        bus.set_profiles_q({j: vel}, {j: acc}, disp={j: target - q0})
        n_log = len(bus._residual_log)
        bus.set_targets_q({j: target})
        rev, _ = measure_reverse(bus, j)
        pred = (bus._residual_log[-1]["predicted_rev_deg"]
                if len(bus._residual_log) > n_log else None)
        return round(rev, 3), pred

    bus.set_decel_negative(False)          # 先走正数减速度
    rev_pos, pred_pos = seg(13.5)          # 期望带残留 1.25°
    bus.set_decel_negative(True)           # 正→负：切换那一段放行残留一次
    rev_switch, pred_switch = seg(30.0)    # 期望仍 1.25°
    rev_next, pred_next = seg(13.5)        # 期望 0（不带残留）
    rows = [
        {"seg": "正数模型", "measured_rev_deg": rev_pos, "predicted_rev_deg": pred_pos},
        {"seg": "切到负数后的第一段", "measured_rev_deg": rev_switch,
         "predicted_rev_deg": pred_switch},
        {"seg": "负数模型的下一段", "measured_rev_deg": rev_next,
         "predicted_rev_deg": pred_next},
    ]
    ok = (abs(rev_pos) > 0.9 and abs(rev_switch) > 0.9
          and abs(rev_next) < 0.1 and pred_next is None)
    return {"name": "正→负切换只放行一次残留", "ok": ok, "rows": rows}


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
    seq25 = [(5.0, 10.0, 13.5), (5.0, 10.0, 30.0),
             (5.0, 10.0, 13.5), (5.0, 10.0, 30.0)]
    seq50 = [(10.0, 20.0, 13.5), (10.0, 20.0, 30.0),
             (10.0, 20.0, 13.5), (10.0, 20.0, 30.0)]
    # 36.1 第 4 条：减速度全负 → 每段推算反向都 <0.1°
    out["checks"]["replay_25_neg"] = replay(
        "a/b 25% 半速重放（减速度全负）", seq25, [0, 0, 0, 0])
    out["checks"]["replay_50_neg"] = replay(
        "a/b 50% 重放（减速度全负）", seq50, [0, 0, 0, 0])
    # 减速度全正 → 同 35.2 的旧模型
    out["checks"]["replay_25_pos"] = replay(
        "a/b 25% 半速重放（减速度全正，对照）", seq25, [1.25, 1.25],
        decel_negative=False)
    out["checks"]["replay_50_pos"] = replay(
        "a/b 50% 重放（减速度全正，对照）", seq50, [2.5, 2.5],
        decel_negative=False)
    out["checks"]["replay_50_first_pos"] = replay(
        "50% 第 1 段（上一段 25%，5°/s；减速度全正，对照）",
        [(5.0, 10.0, 30.0), (10.0, 20.0, 13.5)], [0.625],
        decel_negative=False, q0_deg=13.5)
    out["checks"]["switch_carry"] = check_switch_carry()
    out["checks"]["hold_window"] = check_hold_window()
    out["checks"]["accel_floor"] = check_accel_floor()
    out["ok"] = all(v.get("ok") for v in out["checks"].values())
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_36" / "residual_check_36.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")
    for k, v in out["checks"].items():
        print(f"  {'过' if v.get('ok') else '没过'}：{v.get('name')}")
        for row in v.get("rows") or []:
            if "seg" in row:
                print(f"      {row['seg']}：推算 {row['predicted_rev_deg']} "
                      f"实测 {row['measured_rev_deg']}°")
            else:
                print(f"      v={row['vel']} a={row['acc']} disp={row['disp']} "
                      f"推算 {row['predicted_rev_deg']} 实测 {row['measured_rev_deg']} "
                      f"@{row['rev_at_s']}s")
        if "drift_deg" in v:
            print(f"      漂移 {v['drift_deg']}°")
    print("结果：", "过" if out["ok"] else "没过")
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
