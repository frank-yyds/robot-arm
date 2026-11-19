#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.6：把一段会话按时间打成紧凑文本表（事件 + 逐拍状态），给回报用。

用法：
    python3 tools/timeline.py runs/<会话> [--from 12.5] [--to 20] [--joints J2,J3]
                                       [--step 0.5] [--events-only]

时刻用会话内的 mono 秒（state.csv/events.jsonl 里都有）。只读，不改任何文件。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_events(session: Path) -> list[dict]:
    p = session / "events.jsonl"
    out = []
    for ln in p.read_text(encoding="utf-8").splitlines() if p.exists() else []:
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
    header = p.read_text(encoding="utf-8").splitlines()[0].split(",")
    rows = []
    for ln in p.read_text(encoding="utf-8").splitlines()[1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        rows.append(dict(zip(header, parts)))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--joints", default="")
    ap.add_argument("--step", type=float, default=0.2,
                    help="状态行按这个时间间隔抽稀（秒）")
    ap.add_argument("--events-only", action="store_true")
    args = ap.parse_args(argv)
    d = Path(args.session)
    if not d.exists():
        print(f"没有这个会话目录：{d}", file=sys.stderr)
        return 2
    t_from = args.t_from if args.t_from is not None else 0.0
    t_to = args.t_to
    joints = [j.strip().upper() for j in args.joints.split(",") if j.strip()]

    print(f"# 会话 {d.name}")
    meta = d / "meta.json"
    if meta.exists():
        try:
            m = json.loads(meta.read_text(encoding="utf-8"))
            print(f"# 连接={m.get('connector')} git={m.get('git', {}).get('commit')} "
                  f"配置={m.get('config_version')} 网页={m.get('web_version')} "
                  f"开始={m.get('started')}")
        except Exception:  # noqa: BLE001
            pass

    events = load_events(d)
    rows = load_state(d)
    items = []
    for e in events:
        items.append(("event", float(e.get("mono", 0)), e))
    if not args.events_only:
        last_t = {}
        for r in rows:
            try:
                t = float(r["mono"])
            except (TypeError, ValueError):
                continue
            if joints and r.get("joint") not in joints:
                continue
            key = r.get("joint")
            prev = last_t.get(key)
            if prev is not None and args.step > 0 and t - prev < args.step:
                continue
            last_t[key] = t
            items.append(("state", t, r))
    items.sort(key=lambda x: x[1])
    shown = 0
    for kind, t, obj in items:
        if t < t_from or (t_to is not None and t > t_to):
            continue
        shown += 1
        if kind == "event":
            jt = f" {obj['joint']}" if obj.get("joint") else ""
            print(f"{t:8.2f}s  {obj.get('kind',''):<14}{jt:<4} "
                  f"{str(obj.get('text') or '')[:120]}")
        else:
            print(f"{t:8.2f}s  {obj.get('joint',''):<3} "
                  f"目标 {obj.get('target_deg') or '—':>8} "
                  f"实际 {obj.get('pos_deg') or '—':>8} "
                  f"速度 {obj.get('vel_deg_s') or '—':>7} "
                  f"电流 {obj.get('current_a') or '—':>7} "
                  f"ρ {obj.get('rho') or '—':>5} "
                  f"模式 {obj.get('mode') or '—':>2} "
                  f"使能 {obj.get('enabled') or '—':>2} "
                  f"报警 {obj.get('error') or '—'}")
    print(f"# 共 {shown} 行（事件 {sum(1 for k,_,_ in items if k=='event')}，"
          f"状态 {sum(1 for k,_,_ in items if k=='state')}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
