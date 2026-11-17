#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A9 自检（mock）：18.3 停机侧的三条。

1. 教的点在 +y 侧时，收拢点、预备点也都在 +y 侧（q1 同一个值）；
2. 故意把 park_side 和教的 q1 设成相反，要拒绝停机；
3. 接触高度和记录值差 6 mm 要原地保持（判定 mismatch），4 mm 通过。

用法：
    cd arm-control && python3 tools/park_side_check.py
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def main():
    ex = Executor()
    ex.connect("mock")
    base = copy.deepcopy(ex.park_cfg)
    # 确保是“教过、+y 侧”
    base["park_taught"] = True
    base["poses"]["park"] = [92.181, 82.5527, 1.554, 45.4724, 0.0024, 0.0012]
    for key in ("tuck", "pre"):
        base["poses"][key][0] = 92.181
    base["park_side"] = 1
    base["contact_j4_z"] = None
    ex.park_cfg = copy.deepcopy(base)
    pk = ex.park_config()
    q1s = {k: pk["poses"][k][0] for k in ("tuck", "pre", "park")}
    ok1 = (pk["park_side"] == 1 and len(set(round(v, 2) for v in q1s.values())) == 1
           and all(v > 0 for v in q1s.values()) and pk["park_consistency"]["ok"])
    # 2) park_side 和教的 q1 相反 → 拒绝
    bad = copy.deepcopy(base)
    bad["park_side"] = -1
    ex.park_cfg = bad
    refused = False
    try:
        ex.park(False, {"gripper_empty": True, "workspace_clear": True, "shelf_ready": True})
    except Exception as exc:  # noqa: BLE001
        refused = "停机点配置对不上" in str(exc)
    # 3) 接触高度
    ex.park_cfg = copy.deepcopy(base)
    ex.park_cfg["contact_j4_z"] = 146.0
    v6, d6 = ex._contact_height_decision(140.0)
    v4, _ = ex._contact_height_decision(142.0)
    v0, _ = ex._contact_height_decision(146.0)
    ex.park_cfg["contact_j4_z"] = None
    vf, _ = ex._contact_height_decision(140.0)
    ok3 = (v6 == "mismatch" and v4 == "ok" and v0 == "ok" and vf == "first")
    checks = [
        ("教的点在 +y 侧，收拢/预备/停机 q1 一致且在 +y 侧", ok1,
         f"side={pk['park_side']} q1={q1s}"),
        ("park_side 和教的 q1 相反 → 拒绝停机", refused,
         "拒绝" if refused else "没拦"),
        ("接触高度差 6 mm 原地保持、4 mm 通过、第一次要记录", ok3,
         f"6mm={v6}, 4mm={v4}, 0mm={v0}, 未记录={vf}；6mm 说明={d6}"),
    ]
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    ex.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
