#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.1 AP-1：J3 速度环 I 对照试验（只写内存，不存储）。

流程：使能 → 抬到 J2=0/J3=0（保持 J1/J4/J5/J6）→ 改前静态测试（J3 0/30/60/90，
J2 0/30/60，各静置 5 s 采 20 组）→ 写 J3 速度环 I=0.01 并回读 → 改后同样的测试。
**不发 0x0D 存储**。结束不自动失能，由调用者收尾（断电前要失能）。

用法（用户在场、臂在搁架上、执行器已停）：
    python3 tools/ap1_j3_velocity_i.py
"""
from __future__ import annotations

import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def wait_motion(ex, timeout=60.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = ex.motion.status
        if not st.get("busy"):
            return st.get("phase"), st.get("reason", "")
        time.sleep(0.02)
    return "timeout", ""


def sample_pose(ex, settle=5.0, n=20):
    time.sleep(settle)
    rows = []
    for _ in range(n):
        cvp = ex.bus.read_cvp_all()
        lim = {k: v.get("limit_a") for k, v in cvp.items()}
        rows.append(cvp)
        time.sleep(0.02)
    out = {}
    for name in ex.bus.order:
        pos, cur, vel, rho = [], [], [], []
        for r in rows:
            d = r.get(name) or {}
            if not d.get("ok"):
                continue
            pos.append(float(d["pos_deg"]))
            if d.get("current_a") is not None:
                cur.append(float(d["current_a"]))
            if d.get("vel_deg_s") is not None:
                vel.append(float(d["vel_deg_s"]))
            if d.get("rho") is not None:
                rho.append(float(d["rho"]))
        out[name] = {
            "target_deg": round(float((ex.bus._targets or {}).get(name, 0.0)), 3),
            "pos_mean_deg": round(statistics.mean(pos), 3) if pos else None,
            "pos_dev_deg": round(abs(statistics.mean(pos) - float((ex.bus._targets or {}).get(name, 0.0))), 3) if pos else None,
            "current_mean_a": round(statistics.mean(cur), 3) if cur else None,
            "rho_mean": round(statistics.mean(rho), 2) if rho else None,
            "vel_mean_deg_s": round(statistics.mean(vel), 2) if vel else None,
        }
    return out


def run_static(ex, label):
    q = [float(v) for v in ex.motion.read_q()]
    base = list(q)
    base[1] = 0.0
    base[2] = 0.0
    r = {"label": label, "poses": {}}
    # 先把 J2/J3 拉到 0/0
    ex.move_joints(base)
    ph, why = wait_motion(ex)
    r["goto_base"] = {"phase": ph, "reason": why[:80]}
    if ph != "done":
        return r
    for a3 in (0.0, 30.0, 60.0, 90.0):
        tgt = list(base)
        tgt[2] = a3
        ex.move_joints(tgt)
        ph, why = wait_motion(ex)
        if ph != "done":
            r["poses"][f"J3={a3:g}"] = {"phase": ph, "reason": why[:100]}
            continue
        smp = sample_pose(ex)
        r["poses"][f"J3={a3:g}"] = {"J3": smp.get("J3"), "J2": smp.get("J2")}
    # 回到 J3=0，再测 J2
    tgt = list(base); tgt[2] = 0.0
    ex.move_joints(tgt); wait_motion(ex)
    for a2 in (0.0, 30.0, 60.0):
        tgt = list(base); tgt[1] = a2
        ex.move_joints(tgt)
        ph, why = wait_motion(ex)
        if ph != "done":
            r["poses"][f"J2={a2:g}"] = {"phase": ph, "reason": why[:100]}
            continue
        smp = sample_pose(ex)
        r["poses"][f"J2={a2:g}"] = {"J2": smp.get("J2"), "J3": smp.get("J3")}
    return r


def main():
    ex = Executor()
    ex.connect("hw")
    print("enable", ex.bus.enable_all(True, allow_restart=True).get("ok"))
    ex.motion.speed_scale = 0.5
    ex.park_block = None
    time.sleep(0.5)
    q0 = [round(float(v), 2) for v in ex.motion.read_q()]
    print("起始 q", q0)
    i_before = ex.bus.motors["J3"].read_pid_iq24(0x18)
    print("J3 速度环 I（改前回读）", i_before)
    before = run_static(ex, "改前 I=0.0001")
    # 写入 0.01（只内存）
    m3 = ex.bus.motors["J3"]
    okw = m3.write_pid_iq24(0x11, 0.01)
    time.sleep(0.2)
    i_after = m3.read_pid_iq24(0x18)
    print("写入", okw, "回读 J3 速度环 I", i_after)
    after = None
    if i_after is not None and abs(i_after - 0.01) < 0.0005:
        after = run_static(ex, "改后 I=0.01")
    else:
        print("回读不是 0.01，改在零位姿态保持时再写一次")
        try:
            m3.write_pid_iq24(0x11, 0.01); time.sleep(0.2)
            i_after = m3.read_pid_iq24(0x18)
            print("重写回读", i_after)
            if i_after is not None and abs(i_after - 0.01) < 0.0005:
                after = run_static(ex, "改后 I=0.01")
        except Exception as exc:  # noqa: BLE001
            print("重写失败", exc)
    out = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "q0": q0,
           "i_before": i_before, "i_after": i_after,
           "write_ok": okw, "before": before, "after": after}
    Path("runs").mkdir(exist_ok=True)
    p = Path("runs") / f"ap1_{time.strftime('%Y%m%d_%H%M%S')}.json"
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False, indent=1))
    ex.disconnect()


if __name__ == "__main__":
    main()
