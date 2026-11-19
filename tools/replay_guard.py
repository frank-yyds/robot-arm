#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.6：用记录重放保护判定，回答"新的保护在那一刻会不会触发"。

对会话 state.csv 的每一拍，按 26.2 趋势保护的同一套量（d 落后、u 接近速度、
ρ 电流占比）重算，并给出"如果当时用的是新判法，会在哪一拍判成哪一种"。

用法：
    python3 tools/replay_guard.py runs/<会话> [--joints J2,J3] [--from 12 --to 20]
                                     [--lag-deg 20] [--away-deg-s -3]
                                     [--rho 0.5] [--hold-s 0.3]

判据默认值和 arm/motion.py 里 26.2 的一致；改了那边要同步这里。
只读。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def load_state(session: Path):
    p = session / "state.csv"
    if not p.exists():
        return []
    lines = p.read_text(encoding="utf-8").splitlines()
    if len(lines) < 2:
        return []
    header = lines[0].split(",")
    out = []
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        row = dict(zip(header, parts))
        try:
            row["mono"] = float(row["mono"])
        except (TypeError, ValueError):
            continue
        for k in ("target_deg", "pos_deg", "vel_deg_s", "current_a", "rho"):
            try:
                row[k] = float(row[k]) if row.get(k) not in ("", None) else None
            except (TypeError, ValueError):
                row[k] = None
        out.append(row)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--joints", default="")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--lag-deg", type=float, default=20.0,
                    help="落后兜底阈值（26.2 表里的 20°）")
    ap.add_argument("--away-deg-s", type=float, default=-3.0,
                    help="背离/被推走的接近速度阈值（°/s）")
    ap.add_argument("--rho", type=float, default=0.5,
                    help="ρ 阈值（电流占比，26.2 表里的值）")
    ap.add_argument("--hold-s", type=float, default=0.3,
                    help="要持续多久才判")
    args = ap.parse_args(argv)
    d = Path(args.session)
    rows = load_state(d)
    if not rows:
        print(f"会话里没有状态行：{d}", file=sys.stderr)
        return 2
    joints = [j.strip().upper() for j in args.joints.split(",") if j.strip()] \
        or sorted({r["joint"] for r in rows})
    print(f"# 会话 {d.name}；判据：落后 >{args.lag_deg:g}° / "
          f"u <{args.away_deg_s:g}°/s / ρ >{args.rho:g}；持续 {args.hold_s:g}s")
    verdicts = []
    for jn in joints:
        sel = [r for r in rows if r["joint"] == jn
               and (args.t_from is None or r["mono"] >= args.t_from)
               and (args.t_to is None or r["mono"] <= args.t_to)]
        if not sel:
            print(f"{jn}: 没有状态行")
            continue
        since = None
        kind = ""
        fired = None
        max_lag = 0.0
        min_u = None
        max_rho = 0.0
        for i, r in enumerate(sel):
            lag = (None if r["target_deg"] is None or r["pos_deg"] is None
                   else abs(r["pos_deg"] - r["target_deg"]))
            if lag is not None:
                max_lag = max(max_lag, lag)
            u = r["vel_deg_s"]
            if u is not None:
                min_u = u if min_u is None else min(min_u, u)
            rho = r["rho"] or 0.0
            max_rho = max(max_rho, rho)
            trig = ""
            if lag is not None and lag > args.lag_deg:
                trig = "落后兜底"
            elif u is not None and u < args.away_deg_s:
                trig = ("背离（ρ>阈值，有抵抗）" if rho > args.rho
                        else "背离（ρ 小，没抵抗）")
            elif rho > args.rho and u is not None and abs(u) < 0.5:
                trig = "卡住或力矩不够"
            if trig:
                if since is None or trig != kind:
                    since, kind = r["mono"], trig
                elif r["mono"] - since >= args.hold_s:
                    fired = (r["mono"], kind, lag, u, rho)
                    break
            else:
                since, kind = None, ""
        print(f"{jn}: 最大落后 {max_lag:.2f}°，最小 u "
              f"{'—' if min_u is None else f'{min_u:.2f}'}，最大 ρ {max_rho:.2f}；"
              + (f"重放判成「{fired[1]}」在 {fired[0]:.2f}s" if fired else "重放不触发"))
        verdicts.append((jn, fired, max_lag, min_u, max_rho))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
