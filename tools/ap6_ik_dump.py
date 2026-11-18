#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.8 AP-6 第 3 条 c：Mac 侧对拍数据。

生成 1000 个随机位姿和 Mac 执行器 `ik_analytic_flange` 的全部解析解，存成文本，
传到笔记本给 `gluon_kinematics/test_crosscheck` 用同一套公式比对。

    python3 tools/ap6_ik_dump.py /tmp/ap6 [--n 1000]

限位用 ROS 侧的 URDF 值（±133° / J2 ±88° / J6 ±163°，和执行器软限位不同，
所以显式传给 ik_analytic_flange）。|sin q5| < 0.02 的位姿跳过：执行器解析解
没有腕奇异分支，插件有（26.8 第 2 条），单独在笔记本的 a 项里测。
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm.config import URDF  # noqa: E402
from arm.kinematics import RobotModel  # noqa: E402

# ROS 侧 gluon_description/urdf/gluon_2l6.urdf 的限位（度）
LIMITS = [(-133.0, 133.0), (-88.0, 88.0), (-133.0, 133.0),
          (-133.0, 133.0), (-133.0, 133.0), (-163.0, 163.0)]


def quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵 → 四元数 (x, y, z, w)，取 w ≥ 0 的那一支。"""
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=20261118)
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    r = RobotModel(URDF)
    lo = np.array([a for a, _ in LIMITS])
    hi = np.array([b for _, b in LIMITS])
    rng = random.Random(args.seed)

    poses: list[tuple] = []
    counts: list[int] = []
    skipped = 0
    while len(poses) < args.n:
        q = [rng.uniform(lo[i], hi[i]) for i in range(6)]
        if abs(math.sin(math.radians(q[4]))) < 0.02:
            skipped += 1
            continue
        T = r.flange(q)
        sols = r.ik_analytic_flange(T[:3, 3], T[:3, :3], (lo, hi))
        if not sols:
            # 名义几何下原解一定在集里；真出现说明限位/公式边界，记下来。
            skipped += 1
            continue
        px, py, pz = (float(v) for v in T[:3, 3] / 1000.0)
        qx, qy, qz, qw = quat_xyzw(T[:3, :3])
        poses.append((len(poses), px, py, pz, qx, qy, qz, qw,
                      [float(v) for v in q], sols))
        counts.append(len(sols))

    poses_path = args.out_dir / "poses.txt"
    sols_path = args.out_dir / "solutions.txt"
    with poses_path.open("w", encoding="utf-8") as f:
        f.write("# AP-6 crosscheck poses (m, quaternion xyzw, seed rad)\n")
        f.write("# id x y z qx qy qz qw s1 s2 s3 s4 s5 s6\n")
        for pid, px, py, pz, qx, qy, qz, qw, q, _ in poses:
            f.write(f"{pid} {px:.12f} {py:.12f} {pz:.12f} "
                    f"{qx:.12f} {qy:.12f} {qz:.12f} {qw:.12f} "
                    + " ".join(f"{math.radians(v):.12f}" for v in q) + "\n")
    with sols_path.open("w", encoding="utf-8") as f:
        f.write("# AP-6 crosscheck solutions from Mac arm.kinematics.ik_analytic_flange (deg)\n")
        f.write("# id count q1 q2 q3 q4 q5 q6 [x count ...]\n")
        for pid, *_, sols in poses:
            f.write(f"{pid} {len(sols)}")
            for s in sols:
                f.write(" " + " ".join(f"{v:.9f}" for v in s["q_deg"]))
            f.write("\n")

    print(f"写出 {len(poses)} 个位姿（跳过 {skipped} 个 |sin q5|<0.02 或无解）")
    print(f"解数：min {min(counts)} / max {max(counts)} / 平均 {sum(counts)/len(counts):.2f}")
    print(f"  {poses_path}")
    print(f"  {sols_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
