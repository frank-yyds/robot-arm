"""抬高分支拦截自检：搁架姿态不得大跳/横收；零位正常回零不受影响。"""
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
        segs = make_motion(shelf).plan_home(shelf)
    except LiftBranchError as exc:
        # 找不到连续抬高：仍然拒绝，让用户先支撑/手动抬高
        print("PASS 搁架姿态拒绝生成抬高/回零计划:", exc)
    else:
        # 2026-11-15：保留工具轴后，搁架姿态可以只抬 0.9° 就离开；
        # 但必须是“原 XY、原工具轴、竖直抬到 150”，不能靠收/转腕硬解，
        # 也不能出现 >5° 的单段（旧事故是一次 150° 分支跳）。
        lift = (segs[0].get("lift") or {}) if segs else {}
        step = max((max(abs(a - b) for a, b in zip(s["q_target"], s["q_from"]))
                    for s in segs), default=0.0)
        vertical = (lift.get("scale") == 1.0
                    and lift.get("target_z_mm") == 150.0
                    and lift.get("jump_deg", 999.0) <= 5.0)
        if not vertical:
            print("FAIL 搁架姿态生成了非竖直/大跳计划:", lift)
            return 1
        # 21.4：普通动作不再按 5° 拆段；这里只要求没有“n/n”拆段标签，
        # 且抬高解本身是原 XY/原工具轴/竖直 ≤5°。
        chunked = [s for s in segs if "/" in str(s.get("label", ""))]
        if chunked:
            print(f"FAIL 搁架姿态仍有 {len(chunked)} 段 5° 拆段")
            return 1
        print(f"PASS 搁架姿态只做竖直抬高 {lift['jump_deg']:.2f}°，"
              f"共 {len(segs)} 段，单段最大 {step:.2f}°（21.4 不再拆段）")
    try:
        segs = make_motion([0] * 6).plan_home([0] * 6)
    except Exception as exc:  # noqa: BLE001
        print("FAIL 零位回零被误伤:", exc)
        return 1
    print("PASS 零位回零仍可生成", len(segs), "段")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
