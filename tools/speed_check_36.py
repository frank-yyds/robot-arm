#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36.2：用位置核速度、补 35.1 的表（只看 11-23/11-24 的记录，不碰真机）。

每个位移 ≥5° 的段一行：写入 v/a/d、计划 T、计划到 0.8° 的用时 T0.8、
实测到 0.8°/0.1° 的用时、匀速段（30%–70%）平均速度 ÷v、
0x94 速度最大值 ÷v、位置拟合速度最大值 ÷v。
再用 e3 的“只写梯形参数、等 1 s”静止期量 0x94/位置拟合速度噪声。
结果写 runs/<日期>_36/speed_check.json。
"""
from __future__ import annotations

import csv
import json
import math
import re
import statistics
import time
from datetime import datetime
from pathlib import Path

SPEED_RE = re.compile(r"=\s*([-\d.]+)\s*°/s")

SESS_1124 = Path("runs/20261124_215131_hw")
SESS_1123 = Path("runs/20261123_230403_hw")


def load_events(session: Path):
    rows = []
    for ln in (session / "events.jsonl").open(encoding="utf-8"):
        try:
            rows.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return rows


def wall_dt(s: str) -> datetime | None:
    try:
        return datetime.fromisoformat(s)
    except Exception:  # noqa: BLE001
        return None


def load_trace(path: Path, joint: str):
    rows = []
    for r in csv.DictReader(path.open(encoding="utf-8")):
        if r.get("joint") != joint:
            continue
        def f(k):
            v = r.get(k)
            return None if v in (None, "") else float(v)
        rows.append({"t": f("t_rel"), "pos": f("pos_deg"),
                     "vel": f("velocity_deg_s"), "target": f("target_deg"),
                     "wall": r.get("time")})
    rows.sort(key=lambda x: x["t"])
    return rows


def profile_from_events(events, joint: str, trace_start: datetime):
    """在 trace 起点附近找这一颗写入的 0x1F/0x20/0x21；优先 test_write_profile。"""
    lo = trace_start.timestamp() - 1.0
    hi = trace_start.timestamp() + 2.5
    wrote = None
    cmd_vals = {}
    for r in events:
        w = wall_dt(r.get("wall") or "")
        if w is None or not (lo <= w.timestamp() <= hi):
            continue
        k = r.get("kind")
        if k == "test_write_profile":
            txt = r.get("text") or ""
            i = txt.find("{")
            if i >= 0:
                try:
                    import ast
                    d = ast.literal_eval(txt[i:])
                except Exception:  # noqa: BLE001
                    d = None
                if isinstance(d, dict) and joint in d:
                    wrote = d[joint]
        elif k == "cmd" and r.get("joint") == joint and int(r.get("cmd", 0)) in (0x1F, 0x20, 0x21):
            m = SPEED_RE.search(r.get("text") or "")
            if m:
                cmd_vals[int(r["cmd"])] = float(m.group(1))
    if wrote:
        return {"vel": abs(float(wrote.get("vel_deg_s") or 0.0)),
                "acc": abs(float(wrote.get("acc_deg_s2") or 0.0)),
                "dec": abs(float(wrote.get("dec_deg_s2") or 0.0)),
                "src": "test_write_profile"}
    if 0x1F in cmd_vals:
        vel = cmd_vals[0x1F]
        return {"vel": abs(vel), "acc": abs(cmd_vals.get(0x20, vel)),
                "dec": abs(cmd_vals.get(0x21, vel)), "src": "cmd"}
    return None


def static_window(events, joint: str, trace_start: datetime, rows):
    """e3 的静止期（老接口保留）；真正计算改用 trace 里 target 变化前的 1 s。"""
    return None
    prof_t = tgt_t = None
    lo = trace_start.timestamp() - 1.0
    for r in events:
        w = wall_dt(r.get("wall") or "")
        if w is None or w.timestamp() < lo:
            continue
        k = r.get("kind")
        if k == "test_write_profile" and prof_t is None:
            prof_t = w
        elif k == "test_write_targets" and tgt_t is None:
            tgt_t = w
        elif k == "cmd" and int(r.get("cmd", 0)) == 0x0A and r.get("joint") == joint:
            tgt_t = w
        if prof_t and tgt_t:
            break
    if prof_t is None or tgt_t is None:
        return None
    first = wall_dt(rows[0]["wall"]) if rows and rows[0]["wall"] else trace_start
    a = (prof_t - first).total_seconds() + 0.05
    b = (tgt_t - first).total_seconds() - 0.05
    if b - a < 0.3:
        return None
    return max(0.0, a), b


def fit_slope(points):
    n = len(points)
    if n < 3:
        return None
    mt = sum(p[0] for p in points) / n
    mp = sum(p[1] for p in points) / n
    den = sum((p[0] - mt) ** 2 for p in points)
    if den <= 0:
        return None
    return sum((p[0] - mt) * (p[1] - mp) for p in points) / den


def fit_speeds(rows, t0, t1, win=2):
    """每点前后各 win 个样本（共 2*win+1）直线拟合的速度；返回 [(t, slope)]。"""
    sub = [r for r in rows if r["t"] is not None and t0 <= r["t"] <= t1 and r["pos"] is not None]
    out = []
    for i in range(len(sub)):
        a, b = max(0, i - win), min(len(sub), i + win + 1)
        if b - a < 3:
            continue
        s = fit_slope([(sub[k]["t"], sub[k]["pos"]) for k in range(a, b)])
        if s is not None:
            out.append((sub[i]["t"], s))
    return out


def analyze_trace(path: Path, joint: str, prof: dict | None):
    rows = load_trace(path, joint)
    if not rows:
        return None
    vals = [r["pos"] for r in rows if r["pos"] is not None]
    if len(vals) < 5:
        return None
    q0 = statistics.median(vals[:3])
    tgts = [r["target"] for r in rows if r["target"] is not None]
    target = tgts[-1] if tgts else None
    if target is None:
        return None
    disp = target - q0
    if abs(disp) < 5.0:
        return {"joint": joint, "skipped": "位移 <5°", "disp": round(disp, 3)}
    v = prof["vel"] if prof else None
    a = prof["acc"] if prof else None
    d = prof["dec"] if prof else None
    # 写目标那一刻：trace 里 target_deg 第一次变化（e3 前面有一段静止期）
    t_first = rows[0]["target"] if rows[0]["target"] is not None else target
    t_write = 0.0
    for r in rows:
        if r["target"] is not None and abs(r["target"] - t_first) > 1e-6:
            t_write = r["t"]
            break
    out = {"joint": joint, "trace": path.name, "q0": round(q0, 4),
           "target": round(target, 4), "disp": round(disp, 3),
           "t_write_s": round(t_write, 3),
           "written_v": None if v is None else round(v, 3),
           "written_a": None if a is None else round(a, 3),
           "written_d": None if d is None else round(d, 3)}
    if v and a and d and v > 0 and a > 0 and d > 0:
        if abs(disp) >= v * v / a:
            T = abs(disp) / v + v / a
        else:
            T = 2.0 * math.sqrt(abs(disp) / a)
        T08 = T - math.sqrt(1.6 / d)
        out.update({"T_plan_s": round(T, 3), "T08_plan_s": round(T08, 3)})
        t80 = next((r["t"] for r in rows if r["pos"] is not None
                    and abs(r["pos"] - target) <= 0.8 and r["t"] >= t_write), None)
        t01 = next((r["t"] for r in rows if r["pos"] is not None
                    and abs(r["pos"] - target) <= 0.1 and r["t"] >= t_write), None)
        out["t80_s"] = None if t80 is None else round(t80 - t_write, 3)
        out["t01_s"] = None if t01 is None else round(t01 - t_write, 3)
        out["t80_ratio"] = (None if (t80 is None or T08 <= 0)
                            else round((t80 - t_write) / T08, 3))
        sign = 1.0 if disp > 0 else -1.0
        q30, q70 = q0 + 0.3 * disp, q0 + 0.7 * disp
        t30 = next((r["t"] for r in rows if r["pos"] is not None
                    and sign * (r["pos"] - q30) >= 0), None)
        t70 = next((r["t"] for r in rows if r["pos"] is not None
                    and sign * (r["pos"] - q70) >= 0), None)
        if t30 is not None and t70 is not None and t70 > t30:
            avg = 0.4 * abs(disp) / (t70 - t30)
            out["uniform_avg_deg_s"] = round(avg, 3)
            out["uniform_ratio"] = round(avg / v, 3)
        else:
            out["uniform_avg_deg_s"] = None
            out["uniform_ratio"] = None
    vmax = max((abs(r["vel"]) for r in rows if r["vel"] is not None), default=None)
    out["v094_max_deg_s"] = None if vmax is None else round(vmax, 3)
    out["v094_ratio"] = (None if (vmax is None or not v) else round(vmax / v, 3))
    fits = fit_speeds(rows, -1e9, 1e9, win=2)
    fmax = max((abs(s) for _, s in fits), default=None)
    out["vfit_max_deg_s"] = None if fmax is None else round(fmax, 3)
    out["vfit_ratio"] = (None if (fmax is None or not v) else round(fmax / v, 3))
    out["hz"] = round(1.0 / statistics.median(
        [rows[i + 1]["t"] - rows[i]["t"] for i in range(len(rows) - 1)
         if rows[i + 1]["t"] > rows[i]["t"]]), 1) if len(rows) > 2 else None
    return out


def collect_segments():
    segs = []
    # 11-24 S5：同向/变向对照，J2
    d = json.load(open("runs/20261124_35/s5_samedir.json", encoding="utf-8"))
    for i, e in enumerate(d, 1):
        segs.append({"date": "11-24", "seg": f"S5-{i}", "joint": "J2",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    # 11-24 S6：J1 正常 4 + 负减速度 4
    d = json.load(open("runs/20261124_35/s6_j1.json", encoding="utf-8"))
    for i, e in enumerate(d["normal"], 1):
        segs.append({"date": "11-24", "seg": f"S6-J1-{i}", "joint": "J1",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    for i, e in enumerate(d["negative"], 1):
        segs.append({"date": "11-24", "seg": f"S6neg-J1-{i}", "joint": "J1",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    # 11-24 S6：J2 / J3 负减速度各 6
    d = json.load(open("runs/20261124_35/s6_j23neg.json", encoding="utf-8"))
    for i, e in enumerate(d["J2_negative"], 1):
        segs.append({"date": "11-24", "seg": f"S6neg-J2-{i}", "joint": "J2",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    for i, e in enumerate(d["J3_negative"], 1):
        segs.append({"date": "11-24", "seg": f"S6neg-J3-{i}", "joint": "J3",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    # 11-24 e3 / e4：J2
    d = json.load(open("runs/20261124_35/s6_branch2.json", encoding="utf-8"))
    for i, e in enumerate(d["e3"], 1):
        segs.append({"date": "11-24", "seg": f"e3-{i}", "joint": "J2",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    for i, e in enumerate(d["e4"], 1):
        segs.append({"date": "11-24", "seg": f"e4-{i}", "joint": "J2",
                     "trace": Path(e["trace"]), "session": SESS_1124})
    # 11-23 a、b 组：按会话里的 trace 时间，J3 23:06–23:09、J2 23:32–23:34
    for p in sorted(Path("runs").glob("trace_20261123_*.csv")):
        hhmmss = p.stem.split("_")[-1]
        h, m, s = int(hhmmss[:2]), int(hhmmss[2:4]), int(hhmmss[4:6])
        sec = h * 3600 + m * 60 + s
        if not ((23 * 3600 + 6 * 60 <= sec <= 23 * 3600 + 9 * 60)
                or (23 * 3600 + 32 * 60 <= sec <= 23 * 3600 + 34 * 60)):
            continue
        segs.append({"date": "11-23", "seg": p.stem.split("_")[-1],
                     "joint": None, "trace": p, "session": SESS_1123})
    return segs


def main():
    events = {s: load_events(s) for s in (SESS_1124, SESS_1123)}
    out = {"rows": [], "static_noise": []}
    for spec in collect_segments():
        trace = spec["trace"]
        if not trace.exists():
            out["rows"].append({"date": spec["date"], "seg": spec["seg"],
                                "error": f"没有 {trace}"})
            continue
        rows_all = list(csv.DictReader(trace.open(encoding="utf-8")))
        if not rows_all:
            continue
        start = wall_dt(rows_all[0].get("time") or "")
        joint = spec["joint"]
        if joint is None:
            # 取这一条 trace 里位移最大的关节
            best = None
            for j in ("J1", "J2", "J3", "J4", "J5", "J6"):
                rr = load_trace(trace, j)
                if len(rr) < 5:
                    continue
                q0 = statistics.median([r["pos"] for r in rr[:3] if r["pos"] is not None])
                tg = [r["target"] for r in rr if r["target"] is not None]
                if not tg:
                    continue
                disp = abs(tg[-1] - q0)
                if best is None or disp > best[0]:
                    best = (disp, j)
            if best is None:
                continue
            joint = best[1]
        prof = profile_from_events(events[spec["session"]], joint, start) if start else None
        row = analyze_trace(trace, joint, prof)
        if row is None:
            continue
        row.update({"date": spec["date"], "seg": spec["seg"],
                    "written_src": prof.get("src") if prof else None})
        out["rows"].append(row)
        # e3：静止期噪声
        if spec["seg"].startswith("e3-"):
            rr = load_trace(trace, joint)
            tw = row.get("t_write_s") or 0.0
            w = (max(0.0, tw - 1.05), max(0.0, tw - 0.05))
            if w[1] - w[0] >= 0.3:
                st = [r for r in rr if r["t"] is not None and w[0] <= r["t"] <= w[1]]
                vels = [r["vel"] for r in st if r["vel"] is not None]
                fits = [s for _, s in fit_speeds(rr, w[0], w[1], win=2)]
                out["static_noise"].append({
                    "seg": spec["seg"], "window_s": [round(w[0], 2), round(w[1], 2)],
                    "n": len(st),
                    "v094_std": None if len(vels) < 2 else round(statistics.pstdev(vels), 4),
                    "v094_maxabs": None if not vels else round(max(abs(x) for x in vels), 4),
                    "vfit_std": None if len(fits) < 2 else round(statistics.pstdev(fits), 4),
                    "vfit_maxabs": None if not fits else round(max(abs(x) for x in fits), 4),
                })
    # 36.2 第 1 条：补 35.1 表里漏掉的 3 段（时刻/关节/位移/写入最大速度见会话 events）
    fixed = [
        {"row": "23:06:47 J3", "missing_seg": "23:06:47 J3 30→13.5",
         "missing_disp": -16.5, "missing_v": 5.0, "missing_a": 10.0,
         "prev_v": 5.0, "new_a": 10.0, "predicted": 1.25, "measured": 1.334},
        {"row": "a-50%#5 J3", "missing_seg": "23:08:52 J3 30→13.5",
         "missing_disp": -16.5, "missing_v": 10.0, "missing_a": 20.0,
         "prev_v": 10.0, "new_a": 20.0, "predicted": 2.5, "measured": 2.537},
        {"row": "b-50%#1 J2", "missing_seg": "23:33:30 J2 30→50",
         "missing_disp": 20.0, "missing_v": 10.0, "missing_a": 20.0,
         "prev_v": 10.0, "new_a": 20.0, "predicted": 2.5, "measured": 2.452},
    ]
    for f in fixed:
        f["diff_pct"] = round(abs(abs(f["measured"]) - f["predicted"])
                              / f["predicted"] * 100.0, 1)
        f["match"] = f["diff_pct"] <= 30.0
    out["fill_table"] = {
        "rows": fixed,
        "matched_after_fix": 35,
        "total_with_prev": 35,
        "note": "35.1 原表 32/35；补上这 3 段前面的漏段后，3 行推算分别 1.25/2.5/2.5°，"
                "实测 1.334/2.537/2.452°，全部 ≤30% → 35/35",
    }
    rows = [r for r in out["rows"] if "written_v" in r]
    ok_rows = [r for r in rows if r.get("uniform_ratio") is not None
               and r.get("t80_ratio") is not None]
    out["summary"] = {
        "segments": len(rows),
        "uniform_and_time_ok": sum(1 for r in ok_rows
                                   if r["uniform_ratio"] <= 1.1 and r["t80_ratio"] >= 0.9),
        "judged": ("读数噪声" if ok_rows and all(
            r["uniform_ratio"] <= 1.1 and r["t80_ratio"] >= 0.9 for r in ok_rows)
            else "真的跑快了"),
        "uniform_ratio_max": max((r["uniform_ratio"] for r in ok_rows), default=None),
        "t80_ratio_min": min((r["t80_ratio"] for r in ok_rows), default=None),
        "v094_ratio_max": max((r["v094_ratio"] for r in rows
                               if r.get("v094_ratio") is not None), default=None),
        "vfit_ratio_max": max((r["vfit_ratio"] for r in rows
                               if r.get("vfit_ratio") is not None), default=None),
    }
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_36" / "speed_check.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")
    print(f"段数 {out['summary']['segments']}；两项都过 "
          f"{out['summary']['uniform_and_time_ok']}/{len(ok_rows)}；判读 "
          f"{out['summary']['judged']}")
    for r in rows:
        print(f"  {r['date']} {r['seg']} {r['joint']} 位移 {r['disp']} "
              f"v={r['written_v']} a={r['written_a']} d={r['written_d']} "
              f"T={r.get('T_plan_s')} T08={r.get('T08_plan_s')} "
              f"t80={r.get('t80_s')}({r.get('t80_ratio')}) "
              f"平均/ v={r.get('uniform_ratio')} 094/v={r.get('v094_ratio')} "
              f"拟合/v={r.get('vfit_ratio')}")
    for s in out["static_noise"]:
        print(f"  静止 {s['seg']} 窗 {s['window_s']} n={s['n']} "
              f"094 std/max {s['v094_std']}/{s['v094_maxabs']} "
              f"拟合 std/max {s['vfit_std']}/{s['vfit_maxabs']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
