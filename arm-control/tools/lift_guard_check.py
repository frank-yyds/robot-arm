"""抬高分支拦截自检：搁架事故姿态必须拒绝回零；零位正常回零不受影响。"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.fence import Fence
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import LiftBranchError, Motion


def make_motion(q0):
    names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    bus = MockJointBus(initial_deg={n: q0[i] for i, n in enumerate(names)})
    robot = RobotModel(cfgmod.URDF)
    fence = Fence(robot, workspace={"x": [-200, 600], "y": [-400, 400],
                                    "enabled": False})
    lim = ([bus.cfg[n].safe_limits_deg[0] for n in bus.order],
           [bus.cfg[n].safe_limits_deg[1] for n in bus.order])
    return Motion(bus, robot, fence, lim)


def main():
    shelf = [92.2, 82.0, 7.3, 57.9, 94.7, 0.1]
    try:
        make_motion(shelf).plan_home(shelf)
    except LiftBranchError as exc:
        print("PASS 搁架姿态拒绝生成抬高/回零计划:", exc)
    else:
        print("FAIL 搁架姿态居然生成了计划")
        return 1
    try:
        segs = make_motion([0] * 6).plan_home([0] * 6)
    except Exception as exc:  # noqa: BLE001
        print("FAIL 零位回零被误伤:", exc)
        return 1
    print("PASS 零位回零仍可生成", len(segs), "段")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
