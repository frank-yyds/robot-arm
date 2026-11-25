#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36.1 第 3、4 条：启动梯形参数三项核对 + 写丢注入（mock 单测，不碰真机）。

用假的 JointBus 只跑 `_write_and_verify_profile`：
  - 正常：三项都对，一次通过；
  - "J1 减速度写丢一次"：第一次回读到旧值 → 重写一次 → 通过；
  - "连丢两次"：重写后还不对 → 拦下，点名 J1 减速度，并调用全失能。
结果写 runs/<日期>_36/startup_profile_check.json。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.joints import JointBus, StartupRefused, load_joint_configs  # noqa: E402
from arm.sca import DEG_PER_MOTOR_TURN  # noqa: E402

JOINT = "J1"


def _rpm(deg_s: float) -> float:
    return deg_s / DEG_PER_MOTOR_TURN * 60.0


class FakeMotor:
    def __init__(self, script):
        self.script = list(script)
        self.calls = 0
        self.writes = 0

    def profile(self):
        i = min(self.calls, len(self.script) - 1)
        self.calls += 1
        return tuple(float(x) for x in self.script[i])

    def set_profile_out(self, *a):
        self.writes += 1
        return True


class FakeBus:
    def __init__(self, script):
        self.cfg = load_joint_configs()
        self.motors = {JOINT: FakeMotor(script)}
        self.writes = 0
        self.disable_calls = 0

    def _write_profile(self, name, vel_deg_s):
        self.writes += 1
        c = self.cfg[name]
        acc = vel_deg_s / max(c.motion_accel_time_s, 0.05)
        self.motors[name].set_profile_out(vel_deg_s, acc, -acc)

    def _disable_all_quiet(self):
        self.disable_calls += 1


def run_case(name, script, expect_ok):
    bus = FakeBus(script)
    report = {"enabled": []}
    try:
        prof = JointBus._write_and_verify_profile(bus, JOINT, report)
        raised = None
    except StartupRefused as exc:
        prof = None
        raised = str(exc)
    rewrites = report.get("profile_rewrite") or []
    if expect_ok:
        ok = raised is None and bus.writes >= 1 and bus.disable_calls == 0
    else:
        ok = (raised is not None and JOINT in raised and "减速度" in raised
              and bus.disable_calls >= 1)
    return {"name": name, "ok": bool(ok), "expect_ok": bool(expect_ok),
            "raised": raised, "profile": prof, "rewrites": rewrites,
            "write_calls": bus.writes, "disable_calls": bus.disable_calls}


def main():
    cfg = load_joint_configs()[JOINT]
    want_vel = cfg.motion_speed_deg_s
    want_acc = want_vel / max(cfg.motion_accel_time_s, 0.05)
    good = (_rpm(want_vel), _rpm(want_acc), _rpm(want_acc))     # 0x1E 回读不带符号
    bad_dec = (_rpm(want_vel), _rpm(want_acc), _rpm(5.0))       # 旧值 5°/s²
    out = {"checks": {}, "want": {"vel_deg_s": want_vel,
                                  "acc_deg_s2": round(want_acc, 3),
                                  "dec_written_deg_s2": round(-want_acc, 3)},
           "cfg": {"motion_speed_deg_s": cfg.motion_speed_deg_s,
                   "motion_accel_time_s": cfg.motion_accel_time_s}}
    out["checks"]["normal"] = run_case("三项都对（减速度按绝对值核对）", [good], True)
    out["checks"]["lost_once"] = run_case(
        "J1 减速度写丢一次（回读旧值 5°/s²）", [bad_dec, good], True)
    out["checks"]["lost_twice"] = run_case(
        "J1 减速度连丢两次", [bad_dec, bad_dec], False)
    out["checks"]["lost_once_rewrote"] = {
        "name": "写丢一次后重写了一次、只重写一次", "ok": (
            out["checks"]["lost_once"]["ok"]
            and out["checks"]["lost_once"]["write_calls"] == 2
            and len([r for r in out["checks"]["lost_once"]["rewrites"]
                     if r.get("mismatch")]) == 1),
        "write_calls": out["checks"]["lost_once"]["write_calls"],
        "rewrites": out["checks"]["lost_once"]["rewrites"]}
    out["ok"] = all(v.get("ok") for v in out["checks"].values())
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_36" / "startup_profile_check.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")
    for k, v in out["checks"].items():
        print(f"  {'过' if v.get('ok') else '没过'}：{v.get('name')}")
        if v.get("raised"):
            print(f"      拦下：{v['raised']}")
        if v.get("rewrites"):
            print(f"      记录：{v['rewrites']}")
    print("结果：", "过" if out["ok"] else "没过")
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
