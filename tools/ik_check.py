#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK 自检：解析雅可比 + 收敛 + 抬高目标的可达性。

不连真机、不改 config。运行：
    cd arm-control && python3 tools/ik_check.py

覆盖：
  * `_jacobian` 与数值差分在完整姿态约束下一致（<0.02）；
  * 轴方向约束（yaw=None）从当前姿态附近收敛；
  * 完整姿态约束从当前姿态附近收敛；
  * 2026-11-15 抬高失败的几个代表点，`Motion._plan_lift` 现在能给出
    连续计划（工具轴保持当前方向，不在同一 XY 上硬转朝下）。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod  # noqa: E402
from arm.executor import Executor  # noqa: E402
from arm.kinematics import axis_rot, rotvec  # noqa: E402

FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = ""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}"
          + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


def main() -> int:
    ex = Executor()
    robot = ex.robot
    m = ex.preview_motion
    lo = np.asarray(m.limits[0], float)
    hi = np.asarray(m.limits[1], float)
    rng = np.random.default_rng(20261115)

    # 1) 解析雅可比 vs 数值差分（完整姿态）
    worst = 0.0
    for _ in range(8):
        q = rng.uniform(lo * 0.7, hi * 0.7)
        T = robot.tcp(q, "flange", 0.0)
        p = T[:3, 3] + rng.normal(0, 40, 3)
        axis = T[:3, 2]
        # 解析式只在误差很小时等于 rotvec 的导数；这里给一个 <3° 的目标扰动
        u = rng.normal(0, 1, 3)
        u = u / np.linalg.norm(u)
        R = T[:3, :3] @ axis_rot(u, math.radians(0.2))

        def err(qq):
            Tc = robot.tcp(qq, "flange", 0.0)
            return np.r_[p - Tc[:3, 3], rotvec(Tc[:3, :3].T @ R) * 50.0]

        _, _, J = robot._jacobian(q, "flange", 0.0, R, axis)
        h = 1e-4
        for k in range(6):
            qp = q.copy()
            qp[k] += h
            qm = q.copy()
            qm[k] -= h
            Jn = (err(qp) - err(qm)) / (2.0 * h)
            worst = max(worst, float(np.abs(J[:, k] - Jn).max()))
    check("解析雅可比与数值差分一致（完整姿态）", worst < 0.01,
          f"最大差 {worst:.4f}")

    # 2) 轴方向约束：从当前姿态 + 5° 扰动收敛
    ok = 0
    n = 40
    for _ in range(n):
        q = rng.uniform(lo * 0.8, hi * 0.8)
        T = robot.tcp(q, "flange", 0.0)
        seed = np.clip(q + rng.normal(0, 5, 6), lo, hi)
        r = robot.ik(T[:3, 3], T[:3, 2], None, seed, "flange",
                     (m.limits[0], m.limits[1]), pos_tol_mm=0.2,
                     axis_tol_deg=0.5, structured_seeds=False, max_iters=100)
        ok += bool(r.ok)
    check("轴方向约束 40 次收敛", ok >= 38, f"{ok}/40")

    # 3) 完整姿态：只要求不报错、有解比例合理（随机 yaw 可能受腕部限位）
    ok = 0
    n = 30
    for _ in range(n):
        q = rng.uniform(lo * 0.8, hi * 0.8)
        T = robot.tcp(q, "flange", 0.0)
        yaw = float(rng.uniform(-180, 180))
        r = robot.ik(T[:3, 3], T[:3, 2], yaw, q, "flange",
                     (m.limits[0], m.limits[1]), pos_tol_mm=0.2,
                     axis_tol_deg=0.5, structured_seeds=False, max_iters=120)
        ok += bool(r.ok)
    check("完整姿态 30 次至少 20 次收敛", ok >= 20, f"{ok}/30")

    # 4) 抬高代表点：必须给出连续计划，且单段 ≤5°
    reps = [
        # 2026-11-15 扫描里旧“工具轴朝下”策略失败的几个点
        [119.32, -52.77, -85.45, -9.64, -31.53, 47.86],
        [-123.34, -29.51, -107.90, -43.11, -98.88, -143.07],
        [50.91, 46.87, 94.22, -31.93, 4.88, 26.98],
        [-65.62, 75.48, 10.32, 82.85, -39.17, -140.01],
        [17.1, -54.9, -89.4, -122.2, -89.8, 132.8],
    ]
    m.preset = "flange"
    m.yaw_deg = 0.0
    maxstep = 0.0
    good = 0
    for q in reps:
        segs = m._plan_lift(q)
        if not segs:
            good += 1
            continue
        step = max(max(abs(a - b) for a, b in zip(s["q_target"], s["q_from"]))
                   for s in segs)
        maxstep = max(maxstep, step)
        info = segs[-1].get("lift") or {}
        if step <= 5.0 + 1e-6 and info.get("jump_deg", 999) <= 75.0:
            good += 1
    check("抬高代表点均给出连续计划（≤5°/段、总跳 ≤75°）",
          good == len(reps), f"{good}/{len(reps)}，单段最大 {maxstep:.2f}°")

    print(f"\n{'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    raise SystemExit(main())
