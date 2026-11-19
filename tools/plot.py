#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.6：把一段会话里每颗的目标 / 实际 / 电流 / 模式画成图。

用法：
    MPLCONFIGDIR=/tmp/mpl python3 tools/plot.py runs/<会话> [--from 12] [--to 20]
                                        [--joints J2,J3] [--out /tmp/xx.png]

没装 matplotlib 时退化成一张文本表（照样能看），不会报错退出。
只读会话目录，不改任何记录。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl")


def load_state(session: Path) -> list[dict]:
    p = session / "state.csv"
    if not p.exists():
        return []
    lines = p.read_text(encoding="utf-8").splitlines()
    if not lines:
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
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--joints", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)
    d = Path(args.session)
    rows = [r for r in load_state(d)
            if (args.t_from is None or r["mono"] >= args.t_from)
            and (args.t_to is None or r["mono"] <= args.t_to)]
    joints = [j.strip().upper() for j in args.joints.split(",") if j.strip()] \
        or sorted({r["joint"] for r in rows})
    if not rows:
        print(f"会话里没有状态行：{d}", file=sys.stderr)
        return 2
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"没装 matplotlib（{exc}），出文本表：")
        print(f"{'t':>7} {'joint':<4} {'target':>9} {'pos':>9} {'vel':>8} "
              f"{'cur':>8} {'rho':>6} {'mode':>4} {'err':<6}")
        step = max(1, len(rows) // 200)
        for r in rows[::step]:
            print(f"{r['mono']:7.2f} {r['joint']:<4} "
                  f"{r['target_deg'] if r['target_deg'] is not None else '—':>9} "
                  f"{r['pos_deg'] if r['pos_deg'] is not None else '—':>9} "
                  f"{r['vel_deg_s'] if r['vel_deg_s'] is not None else '—':>8} "
                  f"{r['current_a'] if r['current_a'] is not None else '—':>8} "
                  f"{r['rho'] or '—':>6} {r['mode'] or '—':>4} {r['error'] or '—':<6}")
        return 0
    n = len(joints)
    fig, axes = plt.subplots(n, 1, figsize=(11, max(2.2 * n, 3.0)), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, jn in zip(axes, joints):
        sel = [r for r in rows if r["joint"] == jn]
        if not sel:
            ax.set_ylabel(jn)
            continue
        t = [r["mono"] for r in sel]
        for key, label, style in (("target_deg", "目标", "--"),
                                  ("pos_deg", "实际", "-")):
            y = [r[key] for r in sel]
            ax.plot(t, y, style, linewidth=1.0, label=label)
        ax.set_ylabel(f"{jn} (°)")
        ax.grid(alpha=0.25)
        ax2 = ax.twinx()
        cur = [r["current_a"] if r["current_a"] is not None else 0.0 for r in sel]
        ax2.plot(t, cur, color="#c44", linewidth=0.8, alpha=0.7, label="电流 A")
        ax2.set_ylabel("A", color="#c44")
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="upper right")
    axes[-1].set_xlabel("会话内秒 (mono)")
    fig.suptitle(f"{d.name} {'…'.join(joints)}", fontsize=10)
    fig.tight_layout()
    out = Path(args.out or (d / "plot.png"))
    fig.savefig(out, dpi=110)
    print(f"图已存：{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
