#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复核 roundtrip_sweep 的抬高失败：把 IK 搜得更狠，看是"没搜到"还是"真没有"。

对 `results.jsonl` 里每个 `home.phase == "抬高"` 失败的点，按
`Motion._plan_lift` 完全相同的目标、种子和判据重搜：
    目标 = [tcp.x, tcp.y, 150]，工具轴 (0,0,-1)，先锁 yaw=0，再自由偏航；
    种子 = 当前 q + J1/J3/J5 各 ±180° 镜像（同 motion.py）；
    判据 = 位置 ≤0.5 mm、轴 ≤0.5°、单个关节相对当前姿态跳 ≤30°。
区别只有一个：max_iters 80 → 300（每种子 time_budget 1.5 s）。

结果分三类：
    found     同一判据下找到连续解 → 当前 80 次迭代/种子不够（搜索问题）
    branch    有精确解（位置/轴达标），但最小跳 >30° → 必须换分支（策略问题）
    infeasible 一个达标解都没有 → z=150 工具轴朝下这条路不可达（几何问题）

只读扫描结果和 config，不连真机、不写 config。

注意（2026-11-15）：`Motion._plan_lift` 已改成保留当前工具轴、允许收/转腕；
本脚本仍按旧的“目标工具轴 (0,0,-1)”复核历史扫描，只用于解释 2026-11-14
及以前的失败。新结果直接看 `roundtrip_sweep.py`，不要再用本脚本判定
“infeasible”。
用法：
  cd arm-control
  python3 tools/lift_audit.py --dir runs/_sim/sweep_10000 --workers 9
  python3 tools/lift_audit.py --dir ... --resume
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from arm import config as cfgmod  # noqa: E402
from arm.executor import Executor  # noqa: E402

_CTX = None


def _init_worker(settings):
    global _CTX
    ex = Executor()
    m = ex.preview_motion
    if settings["tcp"] not in (ex.robot.tcp_presets or {}):
        raise RuntimeError(f"未知 TCP 预设 {settings['tcp']}")
    m.preset = settings["tcp"]
    m.yaw_deg = 0.0
    _CTX = SimpleNamespace(
        ex=ex, m=m, robot=ex.robot, tcp=settings["tcp"],
        iters=int(settings["iters"]),
        budget=float(settings["budget"]),
        limits=(list(m.limits[0]), list(m.limits[1])),
    )
    if cfgmod.config_version() != settings["config_version"]:
        raise RuntimeError("配置已变，不能和旧扫描结果混用")


def _seeds(q0, lo, hi):
    """和 Motion._plan_lift 的种子完全一致。"""
    out = [np.asarray(q0, float).copy()]
    for i, delta in ((0, 180.0), (0, -180.0), (2, 180.0),
                     (2, -180.0), (4, 180.0), (4, -180.0)):
        s = np.asarray(q0, float).copy()
        s[i] += delta
        out.append(np.clip(s, lo, hi))
    return out


def audit_point(ctx, rec):
    q0 = np.asarray(rec["q"], float)
    tcp = rec["tcp"]
    target = [float(tcp[0]), float(tcp[1]), 150.0]
    lo = np.asarray(ctx.limits[0], float)
    hi = np.asarray(ctx.limits[1], float)
    out = {"i": int(rec["i"]), "status": "", "jump": None, "joint": "",
           "free": None, "pos_err": None, "axis_err": None,
           "best_jump": None, "best_joint": "", "start_ok": bool(rec.get("start_ok")),
           "q": rec["q"], "tcp": tcp}
    best_ok = None          # (jump, joint, free, res)
    best_any = None         # (pos+axis 加权, res)
    for free in (False, True):
        for seed in _seeds(q0, lo, hi):
            try:
                res = ctx.robot.ik(
                    target, (0.0, 0.0, -1.0),
                    None if free else 0.0, seed, ctx.tcp,
                    (ctx.limits[0], ctx.limits[1]),
                    pos_tol_mm=0.5, axis_tol_deg=0.5,
                    structured_seeds=False, max_iters=ctx.iters,
                    time_budget_s=ctx.budget)
            except Exception:  # noqa: BLE001
                continue
            d = np.abs(np.asarray(res.q_deg, float) - q0)
            jump = float(d.max())
            j = int(d.argmax())
            score = res.pos_err_mm + res.axis_err_deg * 0.5
            if best_any is None or score < best_any[0]:
                best_any = (score, res, jump, j, free)
            if not res.ok:
                continue
            if best_ok is None or jump < best_ok[0]:
                best_ok = (jump, j, free, res)
            if jump <= 30.0 + 1e-9:
                out.update(status="found", jump=round(jump, 2),
                           joint=f"J{j + 1}", free=bool(free),
                           pos_err=round(res.pos_err_mm, 3),
                           axis_err=round(res.axis_err_deg, 3),
                           best_jump=round(jump, 2), best_joint=f"J{j + 1}")
                return out
    if best_ok is not None:
        jump, j, free, res = best_ok
        out.update(status="branch", jump=round(jump, 2), joint=f"J{j + 1}",
                   free=bool(free), pos_err=round(res.pos_err_mm, 3),
                   axis_err=round(res.axis_err_deg, 3),
                   best_jump=round(jump, 2), best_joint=f"J{j + 1}")
        return out
    if best_any is not None:
        _, res, jump, j, free = best_any
        out.update(status="infeasible", best_jump=round(float(jump), 2),
                   best_joint=f"J{j + 1}",
                   pos_err=round(res.pos_err_mm, 3),
                   axis_err=round(res.axis_err_deg, 3))
    else:
        out.update(status="infeasible")
    return out


def _worker_batch(items):
    out = []
    for rec in items:
        try:
            out.append(audit_point(_CTX, rec))
        except Exception as exc:  # noqa: BLE001
            out.append({"i": rec["i"], "status": "error", "reason": str(exc),
                        "q": rec.get("q"), "tcp": rec.get("tcp")})
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description="抬高失败复核（更强 IK 搜索）")
    p.add_argument("--dir", required=True, help="roundtrip_sweep 输出目录")
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    p.add_argument("--iters", type=int, default=300)
    p.add_argument("--budget", type=float, default=1.5,
                   help="每个种子的 IK 时间上限 s")
    p.add_argument("--limit", type=int, default=0, help="只复核前 N 个（调试）")
    p.add_argument("--out", default="", help="默认 <dir>/lift_audit.jsonl")
    p.add_argument("--resume", action="store_true", help="跳过已复核的点")
    args = p.parse_args(argv)

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    results = d / "results.jsonl"
    run = {}
    if (d / "run.json").exists():
        run = json.loads((d / "run.json").read_text(encoding="utf-8"))
    if not results.exists():
        print(f"找不到 {results}")
        return 2
    recs = [json.loads(line) for line in results.open(encoding="utf-8")
            if line.strip()]
    fail = [r for r in recs
            if r["home"]["phase"] == "抬高" and not r["home"]["ok"]]
    fail.sort(key=lambda r: r["i"])
    if args.limit:
        fail = fail[:args.limit]
    out_path = Path(args.out) if args.out else d / "lift_audit.jsonl"
    if not out_path.is_absolute():
        out_path = ROOT / out_path
    done = {}
    if args.resume and out_path.exists():
        for line in out_path.open(encoding="utf-8"):
            try:
                rec = json.loads(line)
                done[int(rec["i"])] = rec
            except Exception:  # noqa: BLE001
                pass
    todo = [r for r in fail if r["i"] not in done]
    print(f"抬高失败 {len(fail)} 点；已复核 {len(done)}；待复核 {len(todo)}；"
          f"进程 {args.workers}；iters={args.iters} budget={args.budget}s")
    settings = {
        "tcp": run.get("tcp", cfgmod.load_tcp().get("default", "flange")),
        "iters": args.iters, "budget": args.budget,
        "config_version": cfgmod.config_version(),
    }
    if run.get("config_version") not in (None, settings["config_version"]):
        print("配置和扫描时不一致，停止")
        return 2

    t0 = time.time()
    if todo:
        batch = 10
        tasks = [todo[k:k + batch] for k in range(0, len(todo), batch)]
        n_found = n_branch = 0
        with ProcessPoolExecutor(
                max_workers=args.workers, mp_context=mp.get_context("spawn"),
                initializer=_init_worker, initargs=(settings,)) as pool:
            futs = [pool.submit(_worker_batch, b) for b in tasks]
            try:
                for k, fut in enumerate(as_completed(futs), 1):
                    for rec in fut.result():
                        with out_path.open("a", encoding="utf-8") as f:
                            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                        done[rec["i"]] = rec
                        n_found += rec["status"] == "found"
                        n_branch += rec["status"] == "branch"
                    if k % 20 == 0 or k == len(futs):
                        el = time.time() - t0
                        print(f"  {len(done)}/{len(fail)}  found={n_found} "
                              f"branch={n_branch}  {el:.0f}s", flush=True)
            except KeyboardInterrupt:
                print("中断；已写的行保留，可加 --resume")
                raise
        # 追加写完后再排序一次，保证行序
        rows = [done[i] for i in sorted(done)]
        tmp = out_path.with_suffix(".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        os.replace(tmp, out_path)
    else:
        rows = [done[i] for i in sorted(done)]

    # ---------------- 汇总 ----------------
    cat = {}
    for r in rows:
        cat[r["status"]] = cat.get(r["status"], 0) + 1
    by_start = {}
    for r in rows:
        key = ("起点合法" if r.get("start_ok") else "起点不合法") + "/" + r["status"]
        by_start[key] = by_start.get(key, 0) + 1
    found = [r for r in rows if r["status"] == "found"]
    branch = [r for r in rows if r["status"] == "branch"]
    lines = []
    for r in rows:
        if r["status"] == "found":
            lines.append(f"#{r['i']:05d} found  跳 {r['jump']:.1f}°({r['joint']}, "
                         f"{'自由' if r['free'] else '锁 0°'})  "
                         f"残差 {r['pos_err']:.2f} mm/{r['axis_err']:.2f}°  "
                         f"q={r['q']}  tcp={r['tcp']}")
        elif r["status"] == "branch":
            lines.append(f"#{r['i']:05d} branch 最小跳 {r['jump']:.1f}°({r['joint']})"
                         f" 残差 {r['pos_err']:.2f} mm/{r['axis_err']:.2f}°  "
                         f"q={r['q']}  tcp={r['tcp']}")
        elif r["status"] == "infeasible":
            lines.append(f"#{r['i']:05d} infeasible 最好 {r['pos_err']} mm/"
                         f"{r['axis_err']}°  q={r['q']}  tcp={r['tcp']}")
        else:
            lines.append(f"#{r['i']:05d} {r['status']} {r.get('reason', '')}")
    (d / "lift_audit.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary = {
        "audited": len(rows), "status": cat, "by_start": by_start,
        "found_indices": [r["i"] for r in found],
        "iters": args.iters, "budget_s": args.budget,
        "elapsed_s": round(time.time() - t0, 1) if todo else None,
    }
    (d / "lift_audit_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("\n" + "=" * 70)
    print(f"复核 {len(rows)} 个抬高失败点：")
    for k, v in sorted(cat.items(), key=lambda kv: -kv[1]):
        print(f"  {k}: {v}")
    print("按起点是否合法：")
    for k, v in sorted(by_start.items(), key=lambda kv: -kv[1]):
        print(f"  {k}: {v}")
    if found:
        print(f"\n加强搜索后能连续回零的点（{len(found)} 个）：")
        print("  " + ", ".join(f"#{r['i']}" for r in found[:80])
              + (" …" if len(found) > 80 else ""))
    print(f"\n明细：{d / 'lift_audit.txt'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        raise SystemExit(1)
