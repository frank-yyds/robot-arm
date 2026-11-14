"""离线生成 50 个可达姿态，分别规划/走一遍回零和回搁架。

不连 ECB/网页，不写 config；不执行真实电机。
规划层检查：围栏、力矩、IK/抬高连续、目标步长（0x0200）。
再用虚拟驱动按段应用目标，检查终点和每段步长；不模拟动力学时间。
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.executor import Executor
from arm.fence import Fence
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion

OUT = Path("runs/_sim/reach50.csv")
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def make_motion(q0):
    bus = MockJointBus(initial_deg={n: q0[i] for i, n in enumerate(NAMES)})
    robot = RobotModel(cfgmod.URDF)
    fence = Fence(robot, workspace={"x": [-200, 600], "y": [-400, 400],
                                    "enabled": False})
    lim = ([bus.cfg[n].safe_limits_deg[0] for n in bus.order],
           [bus.cfg[n].safe_limits_deg[1] for n in bus.order])
    return Motion(bus, robot, fence, lim)


def max_step(segs):
    m = 0.0
    for s in segs:
        m = max(m, max(abs(a - b) for a, b in zip(s["q_target"], s["q_from"])))
    return m


def virtual_apply(segs, q_start):
    """按段把目标当作已到位；返回 (ok, 终点, 最大步长, 失败原因)。"""
    q = list(q_start)
    m = 0.0
    for s in segs:
        step = max(abs(a - b) for a, b in zip(s["q_target"], q))
        m = max(m, step)
        if step > 5.0 + 1e-9:
            return False, q, m, f"目标步长 {step:.2f}° >5°"
        lo = [-133, -88, -133, -133, -133, -163]
        hi = [133, 88, 133, 133, 133, 163]
        if any(not (lo[i] - 1e-6 <= s["q_target"][i] <= hi[i] + 1e-6)
               for i in range(6)):
            return False, q, m, "目标超安全限位"
        q = list(s["q_target"])
    return True, q, m, ""


def final_err(q, target):
    return max(abs(float(a) - float(b)) for a, b in zip(q, target))


def main():
    ex = Executor()
    park = ex.park_config()
    target_park = list(park["poses"]["park"])
    rng = np.random.default_rng(20261114)

    probe = make_motion([0.0] * 6)
    lo = np.asarray(probe.limits[0], float)
    hi = np.asarray(probe.limits[1], float)

    points = []
    tries = 0
    while len(points) < 50 and tries < 20000:
        tries += 1
        q = rng.uniform(lo + 5.0, hi - 5.0)
        if probe.start_problem(q):
            continue
        if probe.check_torque(q):
            continue
        points.append([round(float(v), 3) for v in q])
    print(f"生成 {len(points)} 点，尝试 {tries} 次", flush=True)
    if len(points) != 50:
        print("FAIL 生成不足 50 个")
        return 1

    OUT.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    home_ok = park_ok = 0
    home_fail = []
    park_fail = []
    for i, q0 in enumerate(points, 1):
        # 回零：规划 + 虚拟应用
        hm = make_motion(q0)
        hreason = ""
        hsegs = 0
        hstep = None
        hfin = None
        try:
            hs = hm.plan_home(q0, allow_direct_fallback=True)
            hsegs = len(hs)
            hstep = max_step(hs)
            hok_v, hq, hstep_v, hreason = virtual_apply(hs, q0)
            hfin = final_err(hq, [0.0] * 6)
            hok = hok_v and hfin <= 0.8
            if not hok and not hreason:
                hreason = f"终点误差 {hfin:.2f}°"
        except Exception as exc:  # noqa: BLE001
            hok = False
            hreason = str(exc)[:200]
        if hok:
            home_ok += 1
        else:
            home_fail.append((i, hreason))

        # 回搁架：规划 + 虚拟应用
        pm = make_motion(q0)
        preason = ""
        psegs = 0
        pstep = None
        pfin = None
        try:
            pp = pm.plan_park(q0, park, allow_direct_fallback=True)
            ps = pp["lift"] + pp["tuck"] + pp["pre"] + pp["park"]
            psegs = len(ps)
            pstep = max_step(ps)
            pok_v, pq, pstep_v, preason = virtual_apply(ps, q0)
            pfin = final_err(pq, target_park)
            pok = pok_v and pfin <= 0.8
            if not pok and not preason:
                preason = f"终点误差 {pfin:.2f}°"
        except Exception as exc:  # noqa: BLE001
            pok = False
            preason = str(exc)[:200]
        if pok:
            park_ok += 1
        else:
            park_fail.append((i, preason))

        rows.append({
            "i": i, **{n: q0[j] for j, n in enumerate(NAMES)},
            "home_ok": int(hok), "home_segs": hsegs,
            "home_max_step": None if hstep is None else round(hstep, 3),
            "home_final_err": None if hfin is None else round(hfin, 3),
            "home_fallback": int(any("兜底" in s["label"] for s in hs)) if 'hs' in locals() and hs else 0,
        "home_reason": hreason,
            "park_ok": int(pok), "park_segs": psegs,
            "park_max_step": None if pstep is None else round(pstep, 3),
            "park_final_err": None if pfin is None else round(pfin, 3),
            "park_fallback": int(bool(pp.get("direct_fallback"))) if 'pp' in locals() else 0,
        "park_reason": preason,
        })
        print(f"{i:02d} q={q0} home={hok} park={pok}", flush=True)

    with OUT.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"\n回零成功 {home_ok}/50；回搁架成功 {park_ok}/50")
    print("回零失败:", home_fail[:10])
    print("回搁架失败:", park_fail[:10])
    print("结果:", OUT)
    return 0 if home_ok == 50 and park_ok == 50 else 2


if __name__ == "__main__":
    raise SystemExit(main())
