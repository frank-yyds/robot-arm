"""离线仿真：21.4 后普通动作不再按 5° 拆段。

不连 ECB、不连网页、不控电机；只用 MockJointBus + 规划器。
验证：
1. 事故姿态下 plan_home 只产生“抬高 + 回零”段，没有“n/n”5° 拆段；
2. 单段目标可以大于 5°（21.3 已实测模式 6 下 8/12/20/40° 不报 0x0200）；
3. MockJointBus 能跟完计划回到零位。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.fence import Fence
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion

# 2026-11-14 22:53 J2 0x0200 时的实际姿态
Q0 = [84.6566, 46.9116, -24.7754, 92.0697, 60.0208, 17.8320]
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def make_motion(q0):
    bus = MockJointBus(initial_deg={n: q0[i] for i, n in enumerate(NAMES)})
    robot = RobotModel(cfgmod.URDF)
    fence = Fence(robot, workspace={"x": [-200, 600], "y": [-400, 400],
                                    "enabled": False})
    lim = ([bus.cfg[n].safe_limits_deg[0] for n in bus.order],
           [bus.cfg[n].safe_limits_deg[1] for n in bus.order])
    return Motion(bus, robot, fence, lim, poll_hz=50)


def main():
    m = make_motion(Q0)
    segs = m.plan_home(Q0)
    chunked = [s for s in segs if "/" in str(s.get("label", ""))]
    print("计划段数:", len(segs), [s["label"] for s in segs])
    if chunked:
        print(f"FAIL 仍有 {len(chunked)} 段 5° 拆段")
        return 1
    max_step = max(max(abs(a - b) for a, b in zip(s["q_target"], s["q_from"]))
                   for s in segs)
    print(f"单段最大目标步长 {max_step:.2f}°（21.4 允许 >5°）")

    # 让 MockJointBus 真正走完计划
    m2 = make_motion(Q0)
    segs2 = m2.plan_home(Q0)
    try:
        m2.execute(segs2, preview=False)
    except Exception as exc:  # noqa: BLE001
        print("FAIL Mock 执行失败:", exc)
        return 1
    q_end = [m2.bus.q[n] for n in NAMES]
    err = max(abs(v) for v in q_end)
    print(f"Mock 执行完成: 终点 {[round(v, 2) for v in q_end]}，最大误差 {err:.2f}°")
    if err > 0.8:
        print("FAIL Mock 没有走到零位")
        return 1
    print("PASS 离线仿真通过（21.4 不拆段）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
