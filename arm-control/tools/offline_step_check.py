"""离线仿真：驱动 0x0200“位置模式阶跃过大”。

不连 ECB、不连网页、不控电机；只用 MockJointBus + 规划器。
验证：
1. 事故姿态下，旧式“一步回零”目标步长会超限；
2. 新式 ≤5° 拆段每段都不超限；
3. MockJointBus 能跟完拆段回零。
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
STEP_LIMIT = 5.0


class StepDrive:
    def __init__(self, q):
        self.q = list(q)
        self.max_step = 0.0

    def command(self, q_target):
        step = max(abs(a - b) for a, b in zip(q_target, self.q))
        self.max_step = max(self.max_step, step)
        if step > STEP_LIMIT + 1e-9:
            return False, step
        self.q = list(q_target)
        return True, step


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
    print("计划段数:", len(segs))

    old = StepDrive(Q0)
    ok, step = old.command([0.0] * 6)
    print(f"旧式一步目标: 最大步长 {step:.2f}° -> {'接受' if ok else '0x0200'}")
    if ok or step <= STEP_LIMIT:
        print("FAIL 旧式一步居然没触发")
        return 1

    new = StepDrive(Q0)
    for s in segs:
        ok, step = new.command(s["q_target"])
        if not ok:
            print(f"FAIL {s['label']} 步长 {step:.2f}° 仍超 {STEP_LIMIT}°")
            return 1
    print(f"新式拆段: 最大步长 {new.max_step:.2f}°，无 0x0200")
    for i, n in enumerate(NAMES):
        old_step = abs(Q0[i])
        print(f"  {n}: 旧一步 {old_step:.2f}°；新拆段最大 "
              f"{max(abs(s['q_target'][i] - s['q_from'][i]) for s in segs):.2f}°")

    # 让 MockJointBus 真正走完拆段，验证不是只检查数字
    m2 = make_motion(Q0)
    segs2 = m2.plan_home(Q0)
    try:
        m2.execute(segs2, preview=False)
    except Exception as exc:  # noqa: BLE001
        print("FAIL Mock 执行拆段失败:", exc)
        return 1
    q_end = [m2.bus.q[n] for n in NAMES]
    err = max(abs(v) for v in q_end)
    print(f"Mock 执行完成: 终点 {[round(v, 2) for v in q_end]}，最大误差 {err:.2f}°")
    if err > 0.8:
        print("FAIL Mock 没有走到零位")
        return 1
    print("PASS 离线仿真通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
