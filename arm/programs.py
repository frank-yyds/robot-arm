#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""22.4 关键点示教录放：程序 JSON 存取 + 载入校验 + 回放计划。

程序文件放 `arm-control/programs/<name>.json`：
  {
    "version": 1, "name": "...", "created": "...",
    "end_effector": "flange", "payload_kg": 0.0,
    "points": [{"name": "p1", "q_deg": [...], "speed_scale": 1.0,
                "dwell_s": 0.0, "gripper": null}, ...]
  }

录点只记当前读数，不运动；载入时逐点过限位/围栏/力矩预检，任何一点不过
整份拒绝并指出第几点。回放每一段仍是第二十一节的普通动作（模式 6 单目标），
所有保护照常。
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

DEFAULT_DIR = Path(__file__).resolve().parent.parent / "programs"
_NAME_RE = re.compile(r"^[\w\u4e00-\u9fff .\-]{1,48}$")


def _safe_name(name: str) -> str:
    n = str(name or "").strip()
    if not _NAME_RE.match(n) or n in (".", ".."):
        raise ValueError("程序名只能用中文/字母/数字/空格/._-，1–48 字")
    return n


class ProgramStore:
    def __init__(self, root: Path | None = None):
        self.root = Path(root) if root else DEFAULT_DIR
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, name: str) -> Path:
        return self.root / f"{_safe_name(name)}.json"

    def list(self) -> list[dict]:
        out = []
        for p in sorted(self.root.glob("*.json")):
            try:
                d = json.loads(p.read_text(encoding="utf-8"))
                out.append({"name": d.get("name") or p.stem,
                            "points": len(d.get("points") or []),
                            "created": d.get("created"),
                            "end_effector": d.get("end_effector"),
                            "payload_kg": d.get("payload_kg"),
                            "file": p.name})
            except Exception:  # noqa: BLE001
                out.append({"name": p.stem, "points": None, "file": p.name,
                            "error": "JSON 读不了"})
        return out

    def save(self, name: str, points: list[dict], end_effector: str,
             payload_kg: float, overwrite: bool = True) -> dict:
        n = _safe_name(name)
        p = self.path(n)
        if p.exists() and not overwrite:
            raise ValueError(f"程序 {n} 已存在")
        pts = []
        for i, raw in enumerate(points or []):
            q = [float(v) for v in raw.get("q_deg") or []]
            if len(q) != 6:
                raise ValueError(f"第 {i + 1} 点不是 6 个关节角")
            pts.append({
                "name": str(raw.get("name") or f"p{i + 1}"),
                "q_deg": [round(v, 5) for v in q],
                "speed_scale": float(raw.get("speed_scale", 1.0)),
                "dwell_s": max(0.0, float(raw.get("dwell_s", 0.0))),
                "gripper": raw.get("gripper"),
            })
        if not pts:
            raise ValueError("空程序不能保存")
        doc = {"version": 1, "name": n,
               "created": time.strftime("%Y-%m-%d %H:%M:%S"),
               "end_effector": str(end_effector), "payload_kg": float(payload_kg),
               "points": pts}
        p.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n",
                     encoding="utf-8")
        return {"ok": True, "name": n, "path": str(p), "points": len(pts)}

    def load(self, name: str) -> dict:
        p = self.path(name)
        if not p.is_file():
            raise ValueError(f"没有程序 {name}")
        d = json.loads(p.read_text(encoding="utf-8"))
        d.setdefault("points", [])
        d["file"] = str(p)
        return d


def validate_program(motion, program: dict) -> list[dict]:
    """逐点过限位/围栏/力矩；返回失败清单（index 从 1 开始）。"""
    fails: list[dict] = []
    for i, pt in enumerate(program.get("points") or []):
        q = [float(v) for v in pt.get("q_deg") or []]
        if len(q) != 6:
            fails.append({"index": i + 1, "reason": "不是 6 个关节角"})
            continue
        try:
            motion._check_q(q, False)
        except Exception as exc:  # noqa: BLE001
            fails.append({"index": i + 1, "reason": str(exc)})
    return fails


def plan_replay(motion, program: dict, q_now, global_scale: float = 0.5,
                only_index: int | None = None) -> list[dict]:
    """按点顺序规划；返回 [{point_index, point_name, segments, dwell_s}]。

    第一点从当前位置单独算一段（22.4 第 4 条）。only_index 给“单步”。
    """
    pts = program.get("points") or []
    if not pts:
        raise ValueError("程序没有点")
    steps = []
    cur = [float(v) for v in q_now]
    old_scale = float(getattr(motion, "speed_scale", 1.0))
    try:
        for i, pt in enumerate(pts):
            if only_index is not None and i != int(only_index):
                continue
            motion.speed_scale = max(0.05, min(1.0,
                float(global_scale) * max(0.05, float(pt.get("speed_scale", 1.0)))))
            q = [float(v) for v in pt["q_deg"]]
            segs = motion._plan_move(cur, q, f"22.4 回放 第 {i + 1} 点 "
                                             f"{pt.get('name') or ''}")
            steps.append({"point_index": i + 1,
                          "point_name": str(pt.get("name") or f"p{i + 1}"),
                          "segments": segs,
                          "dwell_s": max(0.0, float(pt.get("dwell_s", 0.0)))})
            cur = q
    finally:
        motion.speed_scale = old_scale
    return steps
