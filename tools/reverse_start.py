#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.2 / 30.3：从会话记录里算“每段运动、每颗关节的起步反向”。

用途（都只读记录，不碰 ECB）：
    python3 tools/reverse_start.py runs/<会话> [--json out.json] [--start-window 1.0]

输出每个运动段、每颗位移 >1° 的关节一行：
    段名 / 关节 / 位移 / 起步 1 s 内最大反向量（°）和时刻 / 最大反向速度（°/s） /
    有没有重发目标（时刻）/ 写进去的梯形速度·加速度·减速度（原始值+换算值）

定义（30.3 第 1 条）：
    反向 = (q − q起点) × sign(Δ) 的最小值（负值就是反向）；
    反向速度 = −(v × sign(Δ))，取 1 s 窗口里的最大值（正=反向）；
    段起点 = 该段第一条 command_move 的时刻；段终点 = 下一段起点 / abort / 到位事件。
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


def load_events(session: Path) -> list[dict]:
    p = session / "events.jsonl"
    out = []
    for ln in (p.read_text(encoding="utf-8").splitlines() if p.exists() else []):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def load_state(session: Path) -> list[dict]:
    p = session / "state.csv"
    if not p.exists():
        return []
    lines = p.read_text(encoding="utf-8").splitlines()
    if len(lines) < 2:
        return []
    header = lines[0].split(",")
    rows = []
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        r = dict(zip(header, parts))
        try:
            r["mono"] = float(r["mono"])
        except (TypeError, ValueError):
            continue
        for k in ("target_deg", "pos_deg", "vel_deg_s", "current_a"):
            v = r.get(k)
            try:
                r[k] = float(v) if v not in ("", None) else None
            except (TypeError, ValueError):
                r[k] = None
        rows.append(r)
    return rows


def parse_profile(events: list[dict], t0: float, t1: float,
                  joint: str) -> dict:
    """段起点前后写进去的梯形参数（0x1F / 0x20 / 0x21），原始值 + 换算值。"""
    out = {}
    names = {0x1F: "速度", 0x20: "加速度", 0x21: "减速度"}
    for e in events:
        if e.get("kind") != "cmd" or e.get("joint") != joint:
            continue
        m = float(e.get("mono") or -1e9)
        if not (t0 - 0.5 <= m <= t0 + 0.2):
            continue
        cmd = int(e.get("cmd", -1))
        if cmd not in names:
            continue
        text = str(e.get("text") or "")
        raw = ""
        mm = re.search(r"原始\s+([0-9a-fA-F]+)", text)
        if mm:
            raw = mm.group(1)
        val = ""
        mv = re.findall(r"=\s*([0-9.]+)\s*(RPM|°/s)", text)
        if mv:
            val = " / ".join(f"{a} {b}" for a, b in mv)
        out[names[cmd]] = {"raw": raw, "value": val, "text": text[:70]}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--json", default="")
    ap.add_argument("--start-window", type=float, default=1.0)
    ap.add_argument("--min-displacement", type=float, default=1.0)
    args = ap.parse_args(argv)
    d = Path(args.session)
    if not d.exists():
        print(f"没有这个会话目录：{d}", file=sys.stderr)
        return 2
    events = load_events(d)
    rows = load_state(d)

    # ---- 段：command_move 按 label+时刻分组；同时记下每颗写的目标 ----
    segs: list[dict] = []
    for e in events:
        if e.get("kind") != "command_move":
            continue
        label = str(e.get("text") or "")
        t = float(e.get("mono") or 0.0)
        if not segs or segs[-1]["label"] != label or abs(segs[-1]["t0"] - t) > 2.0:
            segs.append({"label": label, "t0": t, "t1": None, "targets": {}})
        segs[-1]["joint"] = e.get("joint")
        if e.get("target_deg") is not None:
            segs[-1]["targets"][str(e.get("joint"))] = float(e["target_deg"])
    # 段结束：下段起点 / abort / 该段到位
    stops = []
    for e in events:
        if e.get("kind") in ("abort", "execute_done"):
            stops.append(float(e.get("mono") or 0.0))
    for i, s in enumerate(segs):
        t1 = None
        for e in events:
            if e.get("kind") not in ("arrived_error", "abort", "execute_done"):
                continue
            m = float(e.get("mono") or 0.0)
            if m < s["t0"] - 1e-9:
                continue
            lab = str(e.get("text") or "")
            if e.get("kind") == "arrived_error" and not lab.startswith(s["label"]):
                continue
            t1 = m if t1 is None else min(t1, m)
        if i + 1 < len(segs) and segs[i + 1]["t0"] > s["t0"]:
            t1 = segs[i + 1]["t0"] if t1 is None else min(t1, segs[i + 1]["t0"])
        s["t1"] = t1 if t1 is not None else s["t0"] + 10.0

    # ---- 重发目标：write_resend 原文里点了哪些关节 ----
    resends: list[tuple[float, set[str]]] = []
    for e in events:
        if e.get("kind") != "write_resend":
            continue
        text = str(e.get("text") or "")
        joints = set(re.findall(r"(J[1-6])→", text))
        resends.append((float(e.get("mono") or 0.0), joints))

    out_rows = []
    for s in segs:
        t0, t1, label = s["t0"], s["t1"], s["label"]
        joints = list(dict.fromkeys(r["joint"] for r in rows
                                    if t0 - 0.2 <= r["mono"] <= t1 + 0.5))
        for jn in ("J1", "J2", "J3", "J4", "J5", "J6"):
            # 段起点位置：取段起点之前最后一条有位置的状态行（空闲行也算），
            # 不能用第一条运动行——那时臂已经动了一点（23:57 就是这样差 0.59°）
            pre = [r for r in rows if r["joint"] == jn
                   and r["pos_deg"] is not None and r["mono"] < t0]
            if not pre:
                pre = [r for r in rows if r["joint"] == jn
                       and r["pos_deg"] is not None and r["mono"] <= t0 + 0.05]
            if not pre:
                continue
            q0 = pre[-1]["pos_deg"]
            sel = [r for r in rows if r["joint"] == jn and r["vel_deg_s"] is not None
                   and t0 <= r["mono"] <= t1 + 0.6]
            if len(sel) < 2:
                continue
            sel.sort(key=lambda r: r["mono"])
            qend = sel[-1]["pos_deg"]
            if q0 is None or qend is None:
                continue
            plan = s["targets"].get(jn)
            # 位移按“计划方向”算：目标 − 起点；没有目标就退回实测首尾
            delta = (plan - q0) if plan is not None else (qend - q0)
            if abs(delta) < args.min_displacement:
                continue
            sign = 1.0 if delta >= 0 else -1.0
            win = [r for r in sel if r["mono"] <= t0 + args.start_window]
            if not win:
                continue
            back = [(r["pos_deg"] - q0) * sign for r in win]
            rev = min(0.0, min(back))          # 只把“反方向”记成负值
            idx = back.index(min(back))
            rev_at = win[idx]["mono"] - t0
            rev_v = max(0.0, max((-(r["vel_deg_s"] or 0.0) * sign) for r in win))
            rs = [(m, js) for m, js in resends
                  if t0 - 0.1 <= m <= t1 + 0.3 and jn in js]
            prof = parse_profile(events, t0, t1, jn)
            out_rows.append({
                "segment": label, "joint": jn,
                "t0": round(t0, 2), "t1": round(t1, 2),
                "displacement": round(delta, 2),
                "reverse_max": round(rev, 2),
                "reverse_at_s": round(rev_at, 2),
                "reverse_vel_max": round(rev_v, 2),
                "resend": bool(rs),
                "resend_at_s": [round(m - t0, 2) for m, _ in rs],
                "profile": prof,
            })

    # ---- 打印 ----
    print(f"# {d.name}")
    print(f"# 段 {len(segs)} 个；位移 >{args.min_displacement:g}° 的关节行 {len(out_rows)}")
    print(f"{'段':<22}{'关节':<4}{'位移':>8}{'反向':>8}{'时刻':>7}{'反向速度':>10}"
          f"{'重发':>5}  profile(原始/换算)")
    for r in out_rows:
        prof = r["profile"]
        ptxt = "；".join(
            f"{k} {v.get('raw') or '—'}→{v.get('value') or '—'}"
            for k, v in prof.items()) or "—"
        print(f"{r['segment'][:20]:<22}{r['joint']:<4}{r['displacement']:>8.2f}"
              f"{r['reverse_max']:>8.2f}{r['reverse_at_s']:>7.2f}"
              f"{r['reverse_vel_max']:>10.2f}{'有' if r['resend'] else '无':>5}  {ptxt}")
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"session": str(d), "segments": segs, "rows": out_rows},
            ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"# JSON 已写 {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
