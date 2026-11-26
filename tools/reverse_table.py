#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""35.1：起步反向对拍表（只读记录）。

输入：会话目录（events.jsonl / state.csv）+ trace_*.csv（0x94 高频记录，可选）。
输出：每段、每个位移 >1° 的关节一行，含
  段名/速度档、上一段方向和写入最大速度、这一段写入的最大速度/加速度/减速度（原始+换算）、
  0x1C/1D/1E 回读（没记录写 None）、推算反向、实测起步反向、实测最大反向速度、实测峰速、
  离目标 1°/0.5°/0.2° 时的速度。
用法：python3 tools/reverse_table.py --session runs/20261123_230403_hw \
        --traces 'runs/trace_20261123_*.csv' --out runs/20261123_35/s1_table.json
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
from pathlib import Path

import trace_match as tm

SPEED_RE = re.compile(r"=\s*([-\d.]+)\s*RPM\s*=\s*([-\d.]+)\s*°/s")
RAW_RE = re.compile(r"原始\s+([0-9a-fA-F]+)")


def load_events(session: Path):
    rows = []
    for ln in (session / "events.jsonl").open(encoding="utf-8"):
        try:
            rows.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return rows


def actions_from_events(rows):
    """把 events 切成动作：[execute_start .. execute_done / abort]。"""
    out = []
    cur = None
    for r in rows:
        k = r.get("kind")
        if k == "execute_start":
            if cur:
                out.append(cur)
            cur = {"start": r, "rows": []}
        elif cur is not None:
            cur["rows"].append(r)
            if k in ("execute_done",):
                cur["end"] = r
                out.append(cur)
                cur = None
            elif k == "abort":
                cur["abort"] = r
                out.append(cur)
                cur = None
    if cur:
        out.append(cur)
    return out


def cmd_value(text: str):
    m = SPEED_RE.search(text or "")
    raw = RAW_RE.search(text or "")
    return {
        "raw_hex": None if raw is None else raw.group(1),
        "value_deg_s": None if m is None else float(m.group(2)),
    }


def action_info(act):
    info = {"start_wall": act["start"].get("wall"), "mono": act["start"].get("mono"),
            "end_wall": (act.get("end") or act.get("abort") or {}).get("wall"),
            "aborted": bool(act.get("abort")),
            "profiles": {}, "targets": {}, "trend": {}, "arrive_err": {}}
    for r in act["rows"]:
        k = r.get("kind")
        j = r.get("joint")
        if k == "cmd" and int(r.get("cmd", 0)) in (0x1F, 0x20, 0x21):
            info["profiles"].setdefault(j, {})[int(r["cmd"])] = cmd_value(r.get("text"))
        elif k == "command_move":
            info["targets"][j] = r.get("target_deg")
        elif k == "trend_segment":
            info["trend"][j] = r.get("text")
        elif k == "arrived_error":
            info["arrive_err"][j] = r.get("target_deg")
    return info


def wall_dt(s: str | None):
    """'2026-11-23T23:05:20.609' -> datetime（trace_match 要 datetime 入参）。"""
    if not s:
        return None
    try:
        import datetime as _dt
        return _dt.datetime.fromisoformat(s)
    except ValueError:
        return None


def load_trace(path: Path):
    rows = []
    for r in csv.DictReader(path.open(encoding="utf-8")):
        def f(k):
            v = r.get(k)
            return None if v in (None, "") else float(v)
        rows.append({"t": f("t_rel"), "joint": r.get("joint"),
                     "pos": f("pos_deg"), "vel": f("velocity_deg_s"),
                     "target": f("target_deg"),
                     "wall": r.get("time")})
    return rows


def wall_secs(w: str):
    """'2026-11-23T23:05:20.609' -> 秒（当天）。"""
    if not w:
        return None
    return w[11:19]


def trace_secs(path: Path):
    with path.open(encoding="utf-8") as f:
        head = f.readline()
        first = f.readline()
    if not first:
        return None, None
    return first.split(",")[0][11:19], None


def analyze_joint(trace_rows, joint, act_start_sec, q0_default):
    # 37.2 起 trace_rows 已经是这一颗的行（含 joint 的行已由 TraceFile.samples 滤好）
    seq = [r for r in trace_rows if r["pos"] is not None]
    seq.sort(key=lambda r: r["t"])
    if len(seq) < 8:
        return None
    q0 = statistics.median([r["pos"] for r in seq[:5]])
    tg = [r["target"] for r in seq if r["target"] is not None]
    tgt = tg[-1] if tg else None
    if tgt is None:
        return None
    sign = 1.0 if tgt >= q0 else -1.0
    win = [r for r in seq if r["t"] <= 1.0]
    if not win:
        return None
    backs = [(r["pos"] - q0) * sign for r in win]
    rev = min(backs)
    rev_at = win[backs.index(rev)]["t"]
    rev_vel = max([0.0] + [-(r["vel"] or 0.0) * sign for r in win])
    peak = max([abs(r["vel"] or 0.0) for r in seq])
    remain = tgt - q0
    vel_at = {}
    for thr in (1.0, 0.5, 0.2):
        cand = [r for r in seq if remain != 0 and
                (abs(tgt - r["pos"]) <= thr and ((r["pos"] - q0) * sign) > 0)]
        vel_at[str(thr)] = None if not cand else round(cand[0]["vel"] or 0.0, 2)
    return {"q0": round(q0, 3), "target": round(tgt, 3), "disp": round(tgt - q0, 3),
            "rev_1s": round(rev, 3), "rev_at_s": round(rev_at, 3),
            "rev_vel_deg_s": round(rev_vel, 3), "peak_vel_deg_s": round(peak, 3),
            "vel_near_target": vel_at}


def seg_name(action_idx, a):
    # 用 trace 文件所在动作序号 + 是否中止；速度档从写入最大速度推
    return f"动作{action_idx + 1}" + ("（中止）" if a["aborted"] else "")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--traces", default="")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    sess = Path(a.session)
    rows = load_events(sess)
    actions = actions_from_events(rows)
    infos = [action_info(x) for x in actions]
    # 37.2 第 1 条：按动作的起止时刻找"覆盖它的"trace（不按最新文件拿）。
    # 覆盖判据和容差见 tools/trace_match.py。
    index = tm.build_index([a.traces]) if a.traces else []
    trace_by_action = {}
    for i, inf in enumerate(infos):
        t0, t1 = wall_dt(inf["start_wall"]), wall_dt(inf["end_wall"])
        if t0 is None or t1 is None:
            continue
        tr = tm.find_trace_covering(index, t0, t1, inf["targets"])
        if tr is not None:
            trace_by_action[i] = tr
    out = {"session": str(sess), "actions": []}
    prev = {}
    for i, inf in enumerate(infos):
        tr = trace_by_action.get(i)
        joints = sorted(set(inf["targets"].keys()) | set(inf["profiles"].keys()))
        line = {"seg": seg_name(i, inf), "start": inf["start_wall"],
                "aborted": inf["aborted"], "joints": {}}
        for j in joints:
            tgt = inf["targets"].get(j)
            prof = inf["profiles"].get(j) or {}
            vel = (prof.get(0x1F) or {}).get("value_deg_s")
            acc = (prof.get(0x20) or {}).get("value_deg_s")
            dec = (prof.get(0x21) or {}).get("value_deg_s")
            pv = prev.get(j) or {}
            row = {"written_vel_deg_s": vel, "written_acc_deg_s2": acc,
                   "written_dec_deg_s2": dec,
                   "written_raw": {k: (prof.get(k) or {}).get("raw_hex") for k in (0x1F, 0x20, 0x21)},
                   "prev_dir": pv.get("dir"), "prev_written_vel_deg_s": pv.get("vel"),
                   "readback_1c_1d_1e": None}
            an = None if tr is None else analyze_joint(tr.samples(j), j, inf["start_wall"], tgt)
            if an:
                row.update(an)
                dir_now = 1 if an["disp"] > 0 else -1
                if pv.get("dir") is not None and pv.get("vel") is not None and acc:
                    if dir_now == pv["dir"]:
                        row["predicted_rev_deg"] = 0.0
                    else:
                        row["predicted_rev_deg"] = round(pv["vel"] ** 2 / (2.0 * acc), 3)
                else:
                    row["predicted_rev_deg"] = None
                prev[j] = {"dir": dir_now, "vel": vel or 0.0}
            else:
                row["predicted_rev_deg"] = None
            line["joints"][j] = row
        out["actions"].append(line)

    # 统计：位移 >1° 的段、推算/实测相差 ≤30%
    tot = ok = 0
    bad = []
    for act in out["actions"]:
        for j, row in act["joints"].items():
            if row.get("disp") is None or abs(row["disp"]) <= 1.0:
                continue
            tot += 1
            p, m = row.get("predicted_rev_deg"), row.get("rev_1s")
            if p in (None, 0.0) or m is None:
                continue
            if p != 0 and abs(abs(m) - abs(p)) <= 0.3 * abs(p):
                ok += 1
            else:
                bad.append({"seg": act["seg"], "joint": j, "predicted": p,
                            "measured_abs": abs(m)})
    out["stats"] = {"segments_disp_gt_1": tot, "pred_vs_meas_within_30pct": ok,
                    "mismatch": bad}
    text = json.dumps(out, ensure_ascii=False, indent=1)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(text, encoding="utf-8")
        print(f"写到 {a.out}")
    print(json.dumps(out["stats"], ensure_ascii=False)[:800])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
