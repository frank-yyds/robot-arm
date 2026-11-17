#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""22.1 解析逆解自检：两点核对、随机 2000 回代/漏解、和数值解对比。

运行：cd arm-control && python3 tools/ik_analytic_check.py
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm.config import URDF, tcp_presets
from arm.joints import load_joint_configs
from arm.kinematics import RobotModel, orient_from_axis, rotvec

FAILS: list[str] = []


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


def diff_deg(a, b):
    return max(abs((x - y + 180.0) % 360.0 - 180.0) for x, y in zip(a, b))


r = RobotModel(URDF, tcp_presets())
jc = load_joint_configs()
names = list(jc)
lo = np.array([jc[n].safe_limits_deg[0] for n in names])
hi = np.array([jc[n].safe_limits_deg[1] for n in names])

# 1) 两个核对点：q5=0 是腕奇异，解析支应跳过；数值 IK 仍应到位
for q, want in (([0.0] * 6, [0.0, 144.0, 576.2]),
                ([0.0, 60.0, 30.0, 0.0, 0.0, 0.0], [428.44, 144.0, 222.25])):
    T = r.flange(q)
    pe = float(np.linalg.norm(T[:3, 3] - np.array(want)))
    check(f"FK 核对 {q}", pe < 0.01, f"{T[:3, 3].round(3)} 差 {pe:.4f} mm")
    sols = r.ik_analytic_flange(T[:3, 3], T[:3, :3], (lo, hi))
    check(f"腕奇异解析支跳过 {q}", len(sols) == 0, f"{len(sols)} 组")

# 2) 随机 2000 个姿态：解析解回代、原解在集里、漏解
rng = np.random.default_rng(221)
N = 2000
tested = 0
miss = 0
bad_fk = 0
bad_ori = 0
counts: list[int] = []
t0 = time.time()
while tested < N:
    q = rng.uniform(lo, hi)
    if abs(math.sin(math.radians(q[4]))) < 0.02:
        continue   # 腕奇异，按 22.1 跳过
    tested += 1
    T = r.flange(q)
    sols = r.ik_analytic_flange(T[:3, 3], T[:3, :3], (lo, hi))
    counts.append(len(sols))
    found = False
    for s in sols:
        T2 = r.flange(s["q_deg"])
        if float(np.linalg.norm(T2[:3, 3] - T[:3, 3])) > 0.01:
            bad_fk += 1
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(
            (np.trace(T2[:3, :3].T @ T[:3, :3]) - 1.0) / 2.0)))))
        if ang > 0.01:
            bad_ori += 1
        if diff_deg(s["q_deg"], q) < 1e-3:
            found = True
    if not found:
        miss += 1
elapsed = time.time() - t0
check("随机 2000 回代位置误差 <0.01 mm", bad_fk == 0, f"{bad_fk} 组失败")
check("随机 2000 回代姿态误差 <0.01°", bad_ori == 0, f"{bad_ori} 组失败")
check("随机 2000 漏解 0", miss == 0, f"漏 {miss}/{tested}")
check("每组解数 1–8（软限位内）", min(counts) >= 1 and max(counts) <= 8,
      f"min {min(counts)} max {max(counts)} avg {np.mean(counts):.2f}，{elapsed:.1f}s")

# 3) 和现有数值解对比：200 个目标，数值解必须出现在解析集里
rng = np.random.default_rng(222)
n_cmp = 0
miss_cmp = 0
while n_cmp < 200:
    q = rng.uniform(lo, hi)
    if abs(math.sin(math.radians(q[4]))) < 0.05:
        continue
    n_cmp += 1
    T = r.flange(q)
    axis = T[:3, :3][:, 2]
    # 与 ik 的约定对齐：yaw=0 时法兰 R = orient_from_axis(axis, 0)
    Rf = orient_from_axis(axis, 0.0)
    num = r.ik(T[:3, 3], axis, 0.0, seed_deg=q, preset="flange",
               limits=(lo, hi), structured_seeds=False, restarts=0,
               max_iters=80)
    # 数值 IK 的残差常有 0.05–0.2 mm；且可能贴限位截断。只比收敛到
    # 0.05 mm / 0.1° 以内、且不在限位边界上的解，关节容差 2°。
    if (not num.ok or num.pos_err_mm > 0.05 or num.axis_err_deg > 0.1):
        continue
    if (max(abs(num.q_deg - lo)) < 1e-3 or
            max(abs(num.q_deg - hi)) < 1e-3):
        continue
    sols = r.ik_analytic_flange(T[:3, 3], Rf, (lo, hi))
    if not any(diff_deg(s["q_deg"], num.q_deg) < 2.0 for s in sols):
        miss_cmp += 1
check("数值解在解析解集里（数值收敛的目标）", miss_cmp == 0,
      f"不在 {miss_cmp}/200；数值未收敛的不计入")

print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
raise SystemExit(1 if FAILS else 0)
