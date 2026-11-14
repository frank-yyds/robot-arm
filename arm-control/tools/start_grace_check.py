"""启动容忍自检：1.9° 反向空程应通过；5° 反向应中止。"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.fence import Fence
from arm.joints import JointState, load_joint_configs
from arm.kinematics import RobotModel
from arm.motion import Motion, MotionError


class FakeBus:
    order = ["J1", "J2", "J3", "J4", "J5", "J6"]

    def __init__(self, reverse_deg: float):
        self.cfg = load_joint_configs()
        self.reverse_deg = abs(reverse_deg)
        self.t0 = None
        self.q = {n: 0.0 for n in self.order}

    def read_all(self, full: bool = False):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        if t < 0.65:
            q3 = -self.reverse_deg * (t / 0.65)
        else:
            q3 = min(5.0, -self.reverse_deg + 20.0 * (t - 0.65))
        self.q["J3"] = q3
        return {n: JointState(name=n, id=self.cfg[n].id, enabled=True,
                              out_deg=self.q[n], pos_turns=self.q[n] / 10,
                              target_deg=None, error=0, error_text="")
                for n in self.order}

    def verify_ready(self, names=None):
        return []

    def set_profiles_q(self, vel, acc=None):
        pass

    def set_targets_q(self, targets):
        pass

    def hold_all(self):
        return {"ok": True, "held": dict(self.q),
                "unconfirmed": [], "unreadable": []}

    def read_q(self):
        return [self.q[n] for n in self.order]


def make_motion(reverse_deg: float) -> Motion:
    bus = FakeBus(reverse_deg)
    robot = RobotModel(cfgmod.URDF)
    fence = Fence(robot, workspace={"x": [-200, 600], "y": [-400, 400],
                                    "enabled": False})
    lim = ([bus.cfg[n].safe_limits_deg[0] for n in bus.order],
           [bus.cfg[n].safe_limits_deg[1] for n in bus.order])
    return Motion(bus, robot, fence, lim, poll_hz=25)


def run(reverse_deg: float):
    m = make_motion(reverse_deg)
    seg = {
        "label": "start grace test", "q_from": [0.0] * 6,
        "q_target": [0, 0, 5, 0, 0, 0], "T": 1.0,
        "vel_deg_s": {n: 20.0 for n in m.order},
        "acc_deg_s2": {n: 40.0 for n in m.order},
        "descend": False, "fence": {"ok": True},
    }
    try:
        m.execute([seg], preview=False)
        return True, m.status.get("phase")
    except MotionError as exc:
        return False, str(exc)


def main():
    ok, detail = run(1.9)
    print(("PASS" if ok else "FAIL"), "J3 反向 1.9° 空程后正向 →", detail)
    bad_ok, bad_detail = run(5.0)
    print(("PASS" if not bad_ok else "FAIL"),
          "J3 反向 5° 应中止 →", bad_detail)
    return 0 if ok and not bad_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
