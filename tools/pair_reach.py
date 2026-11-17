#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机点对互达验证：从一批姿态里随机抽两点，A→B 和 B→A 各规划一遍。

“到达”按当前 `move_joints` 的逻辑：`Motion._plan_move(qA, qB)`，也就是关节空间
直线插补、5° 拆段，起点过围栏，目标/中间 10 点过安全限位 + 围栏 + 18.2 力矩
预检（含远端先收重试）；规划完再虚拟走一遍（段连续、单段 ≤5°、终点到位）。
不放抬高、不放 `allow_direct_fallback`。

两个方向分别算，所以会出现“单向能到、反向不能”的不对称（起点围栏、力矩重试
顺序不同）。只报失败点对和原因。

输入是 roundtrip_sweep 的输出目录（读 points.csv；可选 results.jsonl 用于
统计端点合法性）。用法：
  cd arm-control
  python3 tools/pair_reach.py --dir runs/_sim/sweep_10000 --pairs 5000 --workers 9
  python3 tools/pair_reach.py --dir ... --resume
"""
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

from arm import config as cfgmod  # noqa: E402
from arm.executor import Executor  # noqa: E402
from arm.motion import MotionError, StartFenceError, TorqueError  # noqa: E402
from roundtrip_sweep import VerifyError, _fmt_q, apply_chain  # noqa: E402

NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
_CTX = None


def _init_worker(settings):
    global _CTX
    ex = Executor()
    m = ex.preview_motion
    if settings["tcp"] not in (ex.robot.tcp_presets or {}):
        raise RuntimeError(f"未知 TCP 预设 {settings['tcp']}")
    if cfgmod.config_version() != settings["config_version"]:
        raise RuntimeError("配置已变，不能和旧点集混用")
    m.preset = settings["tcp"]
    m.yaw_deg = 0.0
    q = []
    with open(settings["points_csv"], newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            q.append([float(row[n]) for n in NAMES])
    _CTX = SimpleNamespace(
        ex=ex, m=m, tcp=settings["tcp"], q=q,
        limits=([float(v) for v in m.limits[0]],
                [float(v) for v in m.limits[1]]),
    )


def _dest_problem(ctx, qb):
    """目标姿态不能作为终点时的原因（限位先查，再围栏，再力矩）。"""
    lo, hi = ctx.limits
    for i, n in enumerate(NAMES):
        if not (lo[i] - 1e-9 <= qb[i] <= hi[i] + 1e-9):
            return "dest_limit", f"{n} {qb[i]:.2f}° 超安全限位"
    problem = ctx.m.start_problem(qb)
    if problem:
        return "dest_fence", problem
    bad = ctx.m.check_torque(qb)
    if bad:
        b = bad[0]
        return "dest_torque", (f"{b['joint']} {b['tau']:+.2f} N·m > 0.8×"
                               f"{b['capacity']:.2f}（{b['ratio'] * 100:.0f}%）")
    return None


def _leg(ctx, qa, qb, label):
    """一个方向：规划 + 虚拟走位。返回 dict，不抛。"""
    out = {"ok": False, "phase": "", "kind": "", "reason": "", "etype": "",
           "segs": 0, "max_step": None}
    try:
        segs = ctx.m._plan_move(qa, qb, label)
    except Exception as exc:  # noqa: BLE001
        out.update(reason=str(exc)[:400], etype=type(exc).__name__)
        if isinstance(exc, StartFenceError):
            out.update(phase="start", kind="start_fence")
        elif isinstance(exc, TorqueError):
            d = _dest_problem(ctx, qb)
            if d and d[0] == "dest_torque":
                out.update(phase="dest", kind="dest_torque")
            else:
                out.update(phase="path", kind="path_torque")
        elif isinstance(exc, MotionError):
            msg = str(exc)
            d = _dest_problem(ctx, qb)
            if "超出安全限位" in msg:
                out.update(phase="dest", kind="dest_limit")
            elif d:
                out.update(phase="dest", kind=d[0])
            elif "力矩预检" in msg:
                out.update(phase="path", kind="path_torque")
            elif "围栏" in msg:
                out.update(phase="path", kind="path_fence")
            else:
                out.update(phase="path", kind="path_other")
        elif isinstance(exc, VerifyError):
            out.update(phase="verify", kind="verify")
        else:
            out.update(phase="other", kind="other")
        return out
    try:
        qend, step = apply_chain(segs, qa, ctx.limits)
        err = max(abs(a - b) for a, b in zip(qend, qb))
        if err > 0.05:
            raise VerifyError(f"终点误差 {err:.3f}°")
    except Exception as exc:  # noqa: BLE001
        out.update(phase="verify", kind="verify", reason=str(exc)[:400],
                   etype=type(exc).__name__, segs=len(segs))
        return out
    out.update(ok=True, segs=len(segs), max_step=round(step, 3))
    return out


def eval_pair(ctx, pair):
    i, j = int(pair[0]), int(pair[1])
    qa, qb = ctx.q[i], ctx.q[j]
    ab = _leg(ctx, qa, qb, f"{i}->{j}")
    ba = _leg(ctx, qb, qa, f"{j}->{i}")
    rec = {"i": i, "j": j, "ab": ab, "ba": ba}
    rec["pair"] = ("both_ok" if ab["ok"] and ba["ok"] else
                   "one_way" if ab["ok"] or ba["ok"] else "both_fail")
    return rec


def _worker_batch(pairs):
    out = []
    for p in pairs:
        try:
            out.append(eval_pair(_CTX, p))
        except Exception as exc:  # noqa: BLE001
            out.append({"i": int(p[0]), "j": int(p[1]), "pair": "error",
                        "ab": {"ok": False, "phase": "other",
                               "reason": str(exc), "etype": type(exc).__name__},
                        "ba": {"ok": False, "phase": "other",
                               "reason": str(exc), "etype": type(exc).__name__}})
    return out


def make_pairs(n_points, n_pairs, seed):
    """不放回抽点对（同一对不重复，i≠j）。"""
    rng = np.random.default_rng(seed)
    seen = set()
    out = []
    while len(out) < n_pairs:
        k = max(64, (n_pairs - len(out)) * 2)
        a = rng.integers(0, n_points, size=k)
        b = rng.integers(0, n_points, size=k)
        for i, j in zip(a.tolist(), b.tolist()):
            if i == j:
                continue
            key = (i, j) if i < j else (j, i)
            if key in seen:
                continue
            seen.add(key)
            out.append((int(i), int(j)))
            if len(out) >= n_pairs:
                break
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description="随机姿态点对互达验证（只报失败）")
    p.add_argument("--dir", required=True, help="roundtrip_sweep 输出目录")
    p.add_argument("--pairs", type=int, default=5000)
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    p.add_argument("--seed", type=int, default=20261115)
    p.add_argument("--batch", type=int, default=10)
    p.add_argument("--print-failures", type=int, default=200)
    p.add_argument("--resume", action="store_true")
    args = p.parse_args(argv)

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    points_csv = d / "points.csv"
    if not points_csv.exists():
        print(f"找不到 {points_csv}")
        return 2
    run = {}
    if (d / "run.json").exists():
        run = json.loads((d / "run.json").read_text(encoding="utf-8"))
    q = []
    with points_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            q.append([float(row[n]) for n in NAMES])
    n_points = len(q)
    pairs_csv = d / "pairs.csv"
    if args.resume and pairs_csv.exists():
        pairs = []
        with pairs_csv.open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                pairs.append((int(row["i"]), int(row["j"])))
        print(f"续跑：用已有 {len(pairs)} 对（忽略 --pairs）")
    else:
        if not 1 <= args.pairs <= 1000000:
            print("--pairs 需要在 1..1000000")
            return 2
        pairs = make_pairs(n_points, args.pairs, args.seed)
        with pairs_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["i", "j"])
            w.writerows(pairs)
        print(f"{n_points} 个点，抽 {len(pairs)} 对 → {pairs_csv}")

    out_path = d / "pair_results.jsonl"
    done = {}
    if args.resume and out_path.exists():
        for line in out_path.open(encoding="utf-8"):
            try:
                rec = json.loads(line)
                done[(rec["i"], rec["j"])] = rec
            except Exception:  # noqa: BLE001
                pass
    todo = [pr for pr in pairs if pr not in done]
    settings = {
        "tcp": run.get("tcp", cfgmod.load_tcp().get("default", "flange")),
        "config_version": cfgmod.config_version(),
        "points_csv": str(points_csv),
    }
    if run.get("config_version") not in (None, settings["config_version"]):
        print("配置和点集不匹配，停止")
        return 2
    print(f"已完成 {len(done)} 对，待算 {len(todo)} 对，进程 {args.workers}")

    t0 = time.time()
    if todo:
        batch = max(1, args.batch)
        tasks = [todo[k:k + batch] for k in range(0, len(todo), batch)]
        n_ok = 0
        with ProcessPoolExecutor(
                max_workers=args.workers, mp_context=mp.get_context("spawn"),
                initializer=_init_worker, initargs=(settings,)) as pool:
            # 把 q 表塞进每个 worker 的 ctx（init 里没有传，第一次用时补）
            futs = []
            for b in tasks:
                futs.append(pool.submit(_worker_batch, b))
            try:
                for k, fut in enumerate(as_completed(futs), 1):
                    for rec in fut.result():
                        with out_path.open("a", encoding="utf-8") as f:
                            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                        done[(rec["i"], rec["j"])] = rec
                        n_ok += rec["pair"] == "both_ok"
                    if k % 20 == 0 or k == len(futs):
                        el = time.time() - t0
                        print(f"  {len(done)}/{len(pairs)} 对  互达 {n_ok}  "
                              f"{el:.0f}s", flush=True)
            except KeyboardInterrupt:
                print("中断；已写的对保留，可加 --resume")
                raise
        rows = [done[pr] for pr in pairs if pr in done]
        tmp = out_path.with_suffix(".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            for rec in rows:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        os.replace(tmp, out_path)
    else:
        rows = [done[pr] for pr in pairs if pr in done]
    missing = [pr for pr in pairs if pr not in done]

    # ---------------- 汇总 ----------------
    both_ok = [r for r in rows if r["pair"] == "both_ok"]
    one_way = [r for r in rows if r["pair"] == "one_way"]
    both_fail = [r for r in rows if r["pair"] == "both_fail"]
    errs = [r for r in rows if r["pair"] == "error"]
    # 按两端姿态本身是否合法（起点围栏）拆开：否则“一端本来就在围栏外”
    # 会把“两点之间走不通”的比例冲高。
    both_invalid = [r for r in rows
                    if r["ab"].get("phase") == "start"
                    and r["ba"].get("phase") == "start"]
    one_invalid = [r for r in rows
                   if (r["ab"].get("phase") == "start")
                   ^ (r["ba"].get("phase") == "start")]
    both_valid = [r for r in rows
                  if r["ab"].get("phase") != "start"
                  and r["ba"].get("phase") != "start"]
    both_valid_ok = [r for r in both_valid if r["ab"]["ok"] and r["ba"]["ok"]]
    both_valid_path_fail = [r for r in both_valid
                            if not (r["ab"]["ok"] and r["ba"]["ok"])]
    legs = [(r, dr) for r in rows for dr in ("ab", "ba")]
    leg_fail = [(r, dr) for r, dr in legs if not r[dr]["ok"]]
    by_kind = {}
    for r, dr in leg_fail:
        k = r[dr].get("kind", "other")
        by_kind[k] = by_kind.get(k, 0) + 1
    asym = []
    for r in one_way:
        ok_dir = "A→B" if r["ab"]["ok"] else "B→A"
        bad_dir = "ba" if r["ab"]["ok"] else "ab"
        asym.append((r, ok_dir, bad_dir))

    # 失败点对（一行一对）
    lines = []
    for r in rows:
        if r["pair"] == "both_ok":
            continue
        parts = [f"#{r['i']:05d} <-> #{r['j']:05d}  {r['pair']}"]
        for dr, name in (("ab", "A→B"), ("ba", "B→A")):
            info = r[dr]
            parts.append(f"    {name}: " + (
                "OK" if info["ok"] else
                f"[{info.get('phase', '')}] {info.get('reason', '')}"))
        lines.append("\n".join(parts))
    (d / "pair_failures_compact.txt").write_text(
        "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    with (d / "pair_failures.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["i", "j", "pair", "ab_ok", "ab_phase", "ab_reason",
                    "ba_ok", "ba_phase", "ba_reason"])
        for r in rows:
            if r["pair"] == "both_ok":
                continue
            w.writerow([r["i"], r["j"], r["pair"],
                        int(r["ab"]["ok"]), r["ab"].get("phase", ""),
                        r["ab"].get("reason", ""),
                        int(r["ba"]["ok"]), r["ba"].get("phase", ""),
                        r["ba"].get("reason", "")])
    summary = {
        "points": n_points, "pairs": len(rows), "missing": len(missing),
        "both_ok": len(both_ok), "one_way": len(one_way),
        "both_fail": len(both_fail), "errors": len(errs),
        "endpoint_crosstab": {
            "both_valid": {"pairs": len(both_valid),
                           "mutual": len(both_valid_ok),
                           "path_fail": len(both_valid_path_fail)},
            "one_invalid": {"pairs": len(one_invalid), "mutual": 0},
            "both_invalid": {"pairs": len(both_invalid), "mutual": 0},
        },
        "both_valid_path_fail": [
            {"i": r["i"], "j": r["j"],
             "ab": r["ab"].get("reason"), "ba": r["ba"].get("reason")}
            for r in both_valid_path_fail],
        "legs": len(legs), "leg_fail": len(leg_fail),
        "fail_kind": by_kind,
        "elapsed_s": round(time.time() - t0, 1),
        "seed": args.seed, "tcp": settings["tcp"],
        "asymmetric": [{"i": r["i"], "j": r["j"],
                        "ok_dir": ok_dir,
                        "bad_phase": r[bad_dir].get("phase"),
                        "bad_reason": r[bad_dir].get("reason")}
                       for r, ok_dir, bad_dir in asym],
    }
    (d / "pair_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    pct = lambda a, b: (100.0 * a / b) if b else 0.0
    print("\n" + "=" * 78)
    print(f"随机点对 {len(rows)} 对（{n_points} 个点，seed={args.seed}，"
          f"TCP={settings['tcp']}）：互达 {len(both_ok)} "
          f"({pct(len(both_ok), len(rows)):.1f}%)，"
          f"单向 {len(one_way)}，两向都不行 {len(both_fail)}")
    print(f"单向腿 {len(legs)} 条，失败 {len(leg_fail)} 条")
    if both_valid:
        print(f"两端都合法 {len(both_valid)} 对：互达 {len(both_valid_ok)} "
              f"({pct(len(both_valid_ok), len(both_valid)):.2f}%)，"
              f"路径撞围栏 {len(both_valid_path_fail)}")
    print(f"一端不合法 {len(one_invalid)} 对、两端都不合法 {len(both_invalid)} 对："
          "都不可能互达（一头在围栏外）")
    print("按失败类型：")
    for k, v in sorted(by_kind.items(), key=lambda kv: -kv[1]):
        print(f"  {k}: {v}")
    print(f"\n失败点对（共 {len(rows) - len(both_ok)} 对，"
          f"显示前 {args.print_failures} 对）：")
    for s in lines[:args.print_failures]:
        print(s)
    if len(lines) > args.print_failures:
        print(f"... 其余 {len(lines) - args.print_failures} 对见 "
              f"{d / 'pair_failures_compact.txt'}")
    print(f"\n文件：{d / 'pair_failures_compact.txt'}\n"
          f"      {d / 'pair_failures.csv'}\n"
          f"      {d / 'pair_summary.json'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        raise SystemExit(1)
