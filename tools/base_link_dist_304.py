#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.4 第 2 条：base_link（按实物改的地基几何）到其余 link 简化几何的最近距离。

不连 ECB、不需要 ROS。用的是执行器自己的模型：
  * 运动 link 的几何来自 arm/fence.py 的胶囊（Fence.bodies，16.4 第 5 条的简化几何），
    用 URDF 正解摆到 base 系（arm/kinematics.py RobotModel.fk_frames）；
  * base_link 的几何按 30.4 给的 URDF 那段：方块 120×120×48.5（z 0…48.5，
    碰撞按直角，不画 R16 圆角）+ 圆柱 Ø73×31（z 48.5…79.5）。
距离 = 几何表面之间的最短距离 − 运动件的胶囊半径（负值 = 重叠）。

方块与胶囊的距离先用 scipy 沿胶囊轴做有界最小化，再在最小值附近取网格复核；
圆柱与胶囊的距离在胶囊轴上解析取最小。所以结果是数值解，不是闭环精确公式。

用法：python3 tools/base_link_dist_304.py [--json 输出路径]
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod  # noqa: E402
from arm.fence import Fence  # noqa: E402
from arm.kinematics import RobotModel  # noqa: E402

try:
    from scipy.optimize import minimize_scalar
except Exception:  # pragma: no cover
    minimize_scalar = None

HALF = np.array([60.0, 60.0, 24.25])      # 方块半尺寸（mm）
CYL_LO = np.array([0.0, 0.0, 48.5])
CYL_HI = np.array([0.0, 0.0, 79.5])
CYL_R = 36.5
J1_LIST = (0.0, 35.0, -35.0, 40.0, -40.0, 45.0, -45.0, 60.0, -60.0)


def box_point_dist(p: np.ndarray) -> float:
    d = np.maximum(np.abs(p) - HALF, 0.0)
    return float(np.linalg.norm(d))


def cyl_point_dist(p: np.ndarray) -> float:
    z = min(max(p[2], CYL_LO[2]), CYL_HI[2])
    r = math.hypot(p[0], p[1])
    return math.hypot(r - CYL_R, p[2] - z)


def seg_t(p0, p1, t):
    return p0 + t * (p1 - p0)


def box_seg_dist(p0, p1, grid=2001):
    """胶囊轴线段到方块的最短距离；scipy 有界最小化 + 网格复核，取较小者。"""
    f = lambda t: box_point_dist(seg_t(p0, p1, t))
    best = min(f(t) for t in np.linspace(0.0, 1.0, grid))
    if minimize_scalar is not None:
        r = minimize_scalar(f, bounds=(0.0, 1.0), method="bounded",
                            options={"xatol": 1e-10})
        best = min(best, float(r.fun))
    return best


def cyl_seg_dist(p0, p1, grid=2001):
    f = lambda t: cyl_point_dist(seg_t(p0, p1, t))
    best = min(f(t) for t in np.linspace(0.0, 1.0, grid))
    if minimize_scalar is not None:
        r = minimize_scalar(f, bounds=(0.0, 1.0), method="bounded",
                            options={"xatol": 1e-10})
        best = min(best, float(r.fun))
    return best


def body_world(frames, body):
    T = frames[body.link]
    p0 = T[:3, :3] @ body.p0_local + T[:3, 3]
    p1 = T[:3, :3] @ body.p1_local + T[:3, 3]
    return p0.astype(float), p1.astype(float)


def distances_for(robot: RobotModel, fence: Fence, q_deg):
    frames = robot.fk_frames(q_deg)
    out = []
    for b in fence.bodies:
        p0, p1 = body_world(frames, b)
        db = box_seg_dist(p0, p1) - b.radius
        dc = cyl_seg_dist(p0, p1) - b.radius
        out.append({"body": b.name, "link": b.link,
                    "box_surface_mm": round(box_seg_dist(p0, p1), 3),
                    "box_dist_mm": round(db, 3),
                    "cyl_surface_mm": round(cyl_seg_dist(p0, p1), 3),
                    "cyl_dist_mm": round(dc, 3),
                    "dist_mm": round(min(db, dc), 3)})
    return out


def main(argv):
    robot = RobotModel(cfgmod.URDF)
    fence = Fence(robot)
    result = {"urdf": str(cfgmod.URDF),
              "base_geometry": {"box": {"size_mm": [120.0, 120.0, 48.5],
                                        "z_mm": [0.0, 48.5]},
                                "cylinder": {"radius_mm": CYL_R,
                                             "length_mm": 31.0,
                                             "z_mm": [48.5, 79.5]}},
              "method": "方块/圆柱表面到胶囊轴线最短距离 − 胶囊半径；"
                        "scipy bounded 最小化 + 2001 点网格复核（数值解）",
              "scipy": minimize_scalar is not None,
              "states": []}
    for j1 in J1_LIST:
        q = [j1, 0.0, 0.0, 0.0, 0.0, 0.0]
        rows = distances_for(robot, fence, q)
        link2 = min(r["dist_mm"] for r in rows if r["link"] == "link2")
        first = min(rows, key=lambda r: r["dist_mm"])
        result["states"].append({"j1_deg": j1, "others_deg": 0.0,
                                 "link2_min_mm": round(link2, 3),
                                 "min_dist_mm": first["dist_mm"],
                                 "min_body": first["body"],
                                 "min_body_link": first["link"],
                                 "rows": rows})
        print(f"J1={j1:+.0f}°  base_link↔link2 最近 {link2:8.3f} mm"
              f"   全部最近 {first['dist_mm']:8.3f} mm（{first['body']}/{first['link']}）")
    out = Path(argv[1]) if len(argv) > 1 else Path("runs/30_4_base_link_dist.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=1),
                   encoding="utf-8")
    print("数据写到", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
