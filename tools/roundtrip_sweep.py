#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""大样本离线扫描：随机可达姿态 → 回零 / 回搁架，全检查，多进程。

和 `reach50_check.py` 同一条思路，但按真机实际流程、不放宽、不兜底：

  * 点 = 六个关节在安全限位内随机取的一个姿态（保证物理上存在），
    同时算出该姿态的 TCP 坐标；不预先过滤围栏/力矩，那些本身也是结果。
  * 回零 = 和 `Motion.plan_home(allow_direct_fallback=False)` 完全一致：
    `_plan_lift`（抬高 IK + 分支/连续性检查）→ 一步同步插补回 q=0。
  * 回搁架 = 和 `Executor.park()` 一致：`_plan_lift` → 收拢 → 预停机点 →
    按 `_park_q_at_z()` 从 z=168 每 `descent_step_mm` 走一步到
    `descent_min_j4_z`（真机分步下降的规划部分；不模拟电流/接住判定）。
  * 每段都过现有的 `_plan_move`：安全限位、围栏、18.2 力矩预检（含
    远端先收重试）、5° 拆段（0x0200）。不打开 `allow_direct_fallback`。
  * 规划完再"虚拟走位"一遍：段与段必须连续、单段目标 ≤5°、终点必须到
    零位 / 降到最低点；只算目标，不模拟动力学时间。

只输出失败项（哪一点、哪条腿、哪个阶段、原报错、q 和 TCP），成功项只计数。
跑完写：
  points.csv      生成的点 + TCP
  results.jsonl   每点完整结果（断点续跑用，可追加）
  failures.csv    失败明细（Excel 可直接开）
  failures.txt    失败明细（人看）
  summary.json    计数 / 原因分类 / 运行参数
  run.json        参数、config_version、容量

用法（cd arm-control）：
  python3 tools/roundtrip_sweep.py --points 10000 --workers 9
  python3 tools/roundtrip_sweep.py --selftest          # 只校验复制出来的流程
  python3 tools/roundtrip_sweep.py --resume runs/_sim/sweep_20261115_003000

注意：离线只读 config，不连 ECB、不动电机、不写 park_state。
"""
from __future__ import annotations

import argparse
import csv
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
from arm.kinematics import RobotModel  # noqa: E402
from arm.motion import MotionError  # noqa: E402

NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
STEP_LIMIT = 5.0          # _plan_move 的 0x0200 拆段上限
CONT_TOL = 0.01           # 相邻段 q_target/q_from 允许的舍入差（deg）
FINAL_TOL = 0.05          # 终点到位容差（deg）
REASON_MAX = 400

_CTX: SimpleNamespace | None = None


class VerifyError(RuntimeError):
    """虚拟走位发现规划结果自相矛盾。"""


# --------------------------------------------------------------------------- #
# 公共小工具
# --------------------------------------------------------------------------- #
def _r(v, n=4):
    return round(float(v), n)


def _fmt_q(q):
    return "[" + ",".join(f"{float(v):.2f}" for v in q) + "]"


def _fmt_tcp(t):
    return "(" + ",".join(f"{float(v):.1f}" for v in t) + ")"


def category_of(reason: str) -> str:
    """把报错归到一类，方便统计。原报错仍逐条保留。"""
    if "抬高无连续解" in reason:
        return "抬高无连续解"
    if "起点在围栏外" in reason:
        return "起点在围栏外"
    if "力矩预检不通过" in reason:
        return "力矩预检不通过"
    if "超出安全限位" in reason:
        return "超出安全限位"
    if "围栏不通过" in reason:
        return "围栏不通过"
    if "IK 无解" in reason:
        return "IK 无解"
    if "虚拟走位" in reason or "不连续" in reason or "步长" in reason:
        return "规划结果自检"
    return "其他"


def fence_detail(reason: str) -> str:
    """围栏报错归到具体规则（不带具体毫米数，便于统计）。"""
    text = reason.replace("围栏不通过：", "")
    if "运动件离地" in text:
        return "运动件离地 <25 mm"
    if "TCP 离地" in text:
        return "TCP 离地 <20 mm"
    if "水平距离" in text:
        return "到 J1 轴水平距离 >490 mm"
    if "底座区" in text:
        return "进入底座区"
    if "工作区" in text:
        return "超出工作区矩形"
    return ""


def lift_kind(reason: str) -> str:
    """抬高失败再细分：IK 没收敛 / 解到另一个分支。"""
    if "抬高无连续解" not in reason:
        return ""
    if "跳" in reason and ">" in reason:
        return "解到别的分支（单关节跳 >30°）"
    if "残余" in reason:
        return "抬高 IK 没收敛（位置/轴残差超限）"
    return "抬高其他"


def apply_chain(segs, q_start, limits):
    """按段把目标当已到位，检查连续性 / 5° 步长 / 安全限位；返回 (终点, 最大步长)。"""
    q = [float(v) for v in q_start]
    max_step = 0.0
    for k, s in enumerate(segs):
        qf = [float(v) for v in s["q_from"]]
        qt = [float(v) for v in s["q_target"]]
        label = s.get("label", "")
        gap = max(abs(a - b) for a, b in zip(qf, q))
        if gap > CONT_TOL:
            raise VerifyError(f"第 {k + 1} 段起点不连续 {gap:.3f}°（{label}）")
        step = max(abs(a - b) for a, b in zip(qt, qf))
        if step > STEP_LIMIT + 1e-6:
            raise VerifyError(f"第 {k + 1} 段目标步长 {step:.2f}° >"
                              f"{STEP_LIMIT:g}°（{label}）")
        for j, v in enumerate(qt):
            if not (limits[0][j] - 1e-6 <= v <= limits[1][j] + 1e-6):
                raise VerifyError(f"{NAMES[j]} 目标 {v:.2f}° 超安全限位（{label}）")
        q = qt
        max_step = max(max_step, step)
    return q, max_step


def _mark(res: dict, phase: str, exc: BaseException) -> dict:
    msg = str(exc).strip() or exc.__class__.__name__
    if len(msg) > REASON_MAX:
        msg = msg[:REASON_MAX] + "…"
    res.update(ok=False, phase=phase, reason=msg,
               etype=exc.__class__.__name__)
    return res


def _new_res() -> dict:
    return {"ok": False, "phase": "", "reason": "", "etype": "",
            "segs": 0, "lift_segs": 0, "max_step": None,
            "descent_steps": 0, "fail_z": None}


# --------------------------------------------------------------------------- #
# 两条腿（与 Motion.plan_home / Executor.park 的实际流程逐段一致）
# --------------------------------------------------------------------------- #
def eval_home(ctx, q0):
    """抬高 → 回零位。不放兜底；返回结果 dict（不抛）。"""
    m, lim = ctx.m, ctx.limits
    res = _new_res()
    try:
        lift = m._plan_lift(q0)
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "抬高", exc)
    res["lift_segs"] = len(lift)
    cur = list(lift[-1]["q_target"]) if lift else list(q0)
    try:
        rest = m._plan_move(cur, [0.0] * 6, "回零位")
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "回零位", exc)
    segs = lift + rest
    try:
        qend, step = apply_chain(segs, q0, lim)
        worst = max(abs(v) for v in qend)
        if worst > FINAL_TOL:
            raise VerifyError(f"终点离零位 {worst:.3f}°")
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "虚拟走位", exc)
    res.update(ok=True, segs=len(segs), max_step=_r(step, 3))
    return res


def eval_park(ctx, q0):
    """抬高 → 收拢 → 预停机 → 1 mm/步下降到最低点。返回结果 dict（不抛）。"""
    m, ex, lim = ctx.m, ctx.ex, ctx.limits
    pk = ctx.park
    res = _new_res()
    poses = pk["poses"]
    m.park_side = int(pk.get("park_side", -1))

    try:
        lift = m._plan_lift(q0)
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "抬高", exc)
    res["lift_segs"] = len(lift)
    cur = list(lift[-1]["q_target"]) if lift else list(q0)

    try:
        tuck = m._plan_move(cur, poses["tuck"], "收拢+折腕+转 J1")
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "收拢", exc)
    try:
        pre = m._plan_move(poses["tuck"], poses["pre"], "J2/J3 到预停机点")
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "预停机", exc)

    head = lift + tuck + pre
    try:
        _, step = apply_chain(head, q0, lim)
    except Exception as exc:  # noqa: BLE001
        return _mark(res, "虚拟走位", exc)
    cur = list(head[-1]["q_target"])
    total = len(head)
    q1 = float(poses["pre"][0])

    if ctx.descent:
        z = 168.0
        steps = 0
        while z > ctx.z_min + 1e-6:
            nz = max(ctx.z_min, round(z - ctx.step, 3))
            steps += 1
            try:
                segs = m._plan_move(cur, ex._park_q_at_z(nz, q1),
                                    f"分步下降 z={nz:.1f}")
            except Exception as exc:  # noqa: BLE001
                res["descent_steps"] = steps
                res["fail_z"] = nz
                return _mark(res, f"分步下降 z={nz:.1f}", exc)
            try:
                qq, s2 = apply_chain(segs, cur, lim)
            except Exception as exc:  # noqa: BLE001
                res["descent_steps"] = steps
                res["fail_z"] = nz
                return _mark(res, f"分步下降 z={nz:.1f}", exc)
            cur = qq
            step = max(step, s2)
            total += len(segs)
            z = nz
        res["descent_steps"] = steps
        target = ex._park_q_at_z(ctx.z_min, q1)
    else:
        # 快速模式：只查最低点那一步，不分步（可能比真机宽松）
        try:
            segs = m._plan_move(cur, ex._park_q_at_z(ctx.z_min, q1),
                                f"下降到 z={ctx.z_min:.1f}")
        except Exception as exc:  # noqa: BLE001
            res["fail_z"] = ctx.z_min
            return _mark(res, f"下降到 z={ctx.z_min:.1f}", exc)
        try:
            qq, s2 = apply_chain(segs, cur, lim)
        except Exception as exc:  # noqa: BLE001
            res["fail_z"] = ctx.z_min
            return _mark(res, f"下降到 z={ctx.z_min:.1f}", exc)
        cur = qq
        step = max(step, s2)
        total += len(segs)
        target = ex._park_q_at_z(ctx.z_min, q1)

    err = max(abs(a - b) for a, b in zip(cur, target))
    if err > FINAL_TOL:
        res["fail_z"] = ctx.z_min
        return _mark(res, "虚拟走位",
                     VerifyError(f"下降终点误差 {err:.3f}°"))
    res.update(ok=True, segs=total, max_step=_r(step, 3))
    return res


def eval_point(ctx, idx, q0):
    """一个姿态：起点诊断 + 回零 + 回搁架。不抛。"""
    ex, m = ctx.ex, ctx.m
    rec = {"i": int(idx), "q": [_r(v) for v in q0]}
    try:
        tcp = ex.robot.tcp(q0, ctx.tcp, 0.0)[:3, 3]
        rec["tcp"] = [_r(v, 3) for v in tcp]
    except Exception as exc:  # noqa: BLE001
        rec["tcp"] = None
        rec["tcp_error"] = str(exc)
    try:
        problem = m.start_problem(q0)
        torque = m.check_torque(q0)
        rec["start_ok"] = problem is None and not torque
        rec["start_reason"] = (problem or ("" if not torque else
                               f"力矩 {torque[0]['joint']} {torque[0]['tau']:+.2f} "
                               f"> 0.8×{torque[0]['capacity']:.2f} N·m"))
    except Exception as exc:  # noqa: BLE001
        rec["start_ok"] = False
        rec["start_reason"] = f"起点检查异常：{exc}"
    rec["home"] = eval_home(ctx, q0)
    rec["park"] = eval_park(ctx, q0)
    return rec


# --------------------------------------------------------------------------- #
# 子进程
# --------------------------------------------------------------------------- #
def _init_worker(settings):
    global _CTX
    version = cfgmod.config_version()
    if settings.get("config_version") and version != settings["config_version"]:
        raise RuntimeError(
            f"配置在扫描期间变了（{settings['config_version']} → {version}）；"
            "结果不可比，请重新开始")
    ex = Executor()
    m = ex.preview_motion
    if settings["tcp"] not in (ex.robot.tcp_presets or {}):
        raise RuntimeError(f"未知 TCP 预设 {settings['tcp']}")
    m.preset = settings["tcp"]
    m.yaw_deg = 0.0
    pk = ex.park_config()
    _CTX = SimpleNamespace(
        ex=ex, m=m, park=pk, tcp=settings["tcp"],
        limits=([float(v) for v in m.limits[0]],
                [float(v) for v in m.limits[1]]),
        z_min=float(settings["z_min"]), step=float(settings["step"]),
        descent=bool(settings["descent"]),
    )
    return True


def _worker_batch(items):
    out = []
    for idx, q in items:
        try:
            out.append(eval_point(_CTX, idx, q))
        except Exception as exc:  # noqa: BLE001
            out.append({
                "i": int(idx), "q": list(q), "tcp": None, "fatal": True,
                "start_ok": False, "start_reason": f"扫描异常：{exc}",
                "home": {**_new_res(), "phase": "扫描", "reason": str(exc),
                         "etype": type(exc).__name__},
                "park": {**_new_res(), "phase": "扫描", "reason": str(exc),
                         "etype": type(exc).__name__},
            })
    return out


# --------------------------------------------------------------------------- #
# 主进程
# --------------------------------------------------------------------------- #
def build_settings(args, ex, config_version):
    return {
        "tcp": args.tcp,
        "z_min": float(ex.park_cfg.get("descent_min_j4_z") or 130.5),
        "step": float(ex.park_cfg.get("descent_step_mm") or 1.0),
        "descent": not args.no_descent,
        "config_version": config_version,
    }


def generate_points(n, seed, margin, limits):
    rng = np.random.default_rng(seed)
    lo = np.asarray(limits[0], float) + margin
    hi = np.asarray(limits[1], float) - margin
    return rng.uniform(lo, hi, size=(n, 6)).round(4)


def write_points(path, points, robot, preset):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["i", *NAMES, "tcp_x", "tcp_y", "tcp_z"])
        for i, q in enumerate(points):
            tcp = robot.tcp(q.tolist(), preset, 0.0)[:3, 3]
            w.writerow([i, *[f"{v:.4f}" for v in q],
                        *[f"{v:.3f}" for v in tcp]])


def read_points(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append((int(r["i"]), [float(r[n]) for n in NAMES]))
    rows.sort(key=lambda x: x[0])
    return rows


def read_results(path):
    done = {}
    if not path.exists():
        return done
    bad = 0
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
                done[int(rec["i"])] = rec
            except Exception:  # noqa: BLE001
                bad += 1
    if bad:
        print(f"  （results.jsonl 有 {bad} 行不完整，已忽略）")
    return done


def batches(items, size):
    for k in range(0, len(items), size):
        yield items[k:k + size]


def run_sweep(args):
    # 续跑时不要先建一个默认时间戳目录
    if args.resume:
        args.out = args.resume
    out_dir = Path(args.out)
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    points_csv = out_dir / "points.csv"
    results_jsonl = out_dir / "results.jsonl"
    failures_csv = out_dir / "failures.csv"
    failures_txt = out_dir / "failures.txt"
    by_point_txt = out_dir / "failures_by_point.txt"
    by_point_csv = out_dir / "failures_by_point.csv"
    compact_txt = out_dir / "failures_compact.txt"
    summary_json = out_dir / "summary.json"
    run_json = out_dir / "run.json"

    ex = Executor()
    robot = ex.robot
    limits = ([float(v) for v in ex.limits[0]],
              [float(v) for v in ex.limits[1]])
    if args.tcp not in (robot.tcp_presets or {}):
        print(f"未知 TCP 预设 {args.tcp}；可选 {sorted(robot.tcp_presets)}")
        return 2
    version = cfgmod.config_version()

    # 参数 / 点数
    if args.resume:
        if not points_csv.exists():
            print(f"续跑目录里没有 points.csv：{out_dir}")
            return 2
        pts = read_points(points_csv)
        if args.points and len(pts) != args.points:
            print(f"续跑用已有的 {len(pts)} 个点（忽略 --points {args.points}）")
        print(f"续跑：{out_dir}，{len(pts)} 个点")
    else:
        if not 1 <= args.points <= 100000:
            print("--points 需要在 1..100000")
            return 2
        qs = generate_points(args.points, args.seed, args.margin, limits)
        if points_csv.exists() and not args.force:
            print(f"输出目录已有 points.csv：{out_dir}（换 --out 或加 --force）")
            return 2
        write_points(points_csv, qs, robot, args.tcp)
        pts = [(i, qs[i].tolist()) for i in range(len(qs))]
        print(f"生成 {len(pts)} 个点 → {points_csv}")

    t_start = time.time()
    settings = build_settings(args, ex, version)
    capacities = ex.capacities()
    print(f"TCP={settings['tcp']}  末端={ex.end_effector}  "
          f"J2 容量={capacities.get('J2')} N·m  "
          f"下降={settings['z_min']:.1f}..168 mm / {settings['step']:g} mm 步"
          f"{'' if settings['descent'] else '（快速模式：不分步）'}")

    # 续跑前先确认参数没变，别把两次不同的扫描混在一起
    if run_json.exists():
        try:
            prev = json.loads(run_json.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            prev = {}
        if prev:
            if prev.get("config_version") not in (None, version):
                print(f"配置已变：{prev.get('config_version')} → {version}；"
                      "结果不可比，请换新目录重跑")
                return 2
            if prev.get("tcp") not in (None, settings["tcp"]):
                print(f"续跑 TCP 不一致：{prev.get('tcp')} → {settings['tcp']}")
                return 2
            if (prev.get("descent_full") is not None
                    and bool(prev["descent_full"]) != bool(settings["descent"])):
                print("续跑下降模式不一致（--no-descent）；请按原参数续跑")
                return 2

    done = read_results(results_jsonl)
    todo = [(i, q) for i, q in pts if i not in done]
    print(f"已完成 {len(done)}，待算 {len(todo)}，进程 {args.workers}")

    if not run_json.exists():
        run_json.write_text(json.dumps({
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "points": len(pts), "seed": args.seed, "margin_deg": args.margin,
            "tcp": args.tcp, "end_effector": ex.end_effector,
            "payload_kg": ex.payload_kg, "j2_bracket": ex.j2_bracket,
            "limit_a": ex.preview_motion.limit_a,
            "capacities_nm": capacities,
            "park_side": ex.park_config().get("park_side"),
            "park_poses": ex.park_config().get("poses"),
            "descent_min_j4_z": settings["z_min"],
            "descent_step_mm": settings["step"],
            "descent_full": settings["descent"],
            "config_version": version,
            "config_versions": cfgmod.config_versions(),
            "urdf": str(cfgmod.URDF),
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "host_cpu": os.cpu_count(),
        }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if todo:
        t0 = time.time()
        total = len(todo)
        finished = 0
        n_home_fail = n_park_fail = 0
        batch_size = max(1, args.batch)
        tasks = list(batches(todo, batch_size))
        print(f"开始：{len(tasks)} 批 × ≤{batch_size} 点", flush=True)
        with ProcessPoolExecutor(
                max_workers=args.workers,
                mp_context=mp.get_context("spawn"),
                initializer=_init_worker,
                initargs=(settings,)) as pool:
            futures = [pool.submit(_worker_batch, b) for b in tasks]
            try:
                for fut in as_completed(futures):
                    recs = fut.result()
                    with results_jsonl.open("a", encoding="utf-8") as f:
                        for rec in recs:
                            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                            done[rec["i"]] = rec
                            n_home_fail += not rec["home"].get("ok")
                            n_park_fail += not rec["park"].get("ok")
                        f.flush()
                        os.fsync(f.fileno())
                    finished += len(recs)
                    el = time.time() - t0
                    eta = el / max(finished, 1) * (total - finished)
                    if finished == total or finished % max(batch_size, 50) < batch_size:
                        print(f"  {finished}/{total}  "
                              f"home 失败 {n_home_fail}  park 失败 {n_park_fail}  "
                              f"{el:.0f}s  ETA {eta:.0f}s", flush=True)
            except KeyboardInterrupt:
                print("\n中断：已算的批次都写进 results.jsonl，"
                      f"可用 --resume {out_dir} 接着跑")
                for f_ in futures:
                    f_.cancel()
                raise

    # ---------------- 汇总 ----------------
    recs = [done[i] for i, _ in pts if i in done]
    recs.sort(key=lambda r: r["i"])
    missing = [i for i, _ in pts if i not in done]
    if missing:
        print(f"警告：还有 {len(missing)} 点没算（{missing[:5]}…）")

    home_fail = [r for r in recs if not r["home"].get("ok")]
    park_fail = [r for r in recs if not r["park"].get("ok")]
    both = [r for r in recs if not r["home"].get("ok") and not r["park"].get("ok")]
    either = [r for r in recs if not r["home"].get("ok") or not r["park"].get("ok")]
    start_bad = [r for r in recs if not r.get("start_ok")]

    cat = {}
    for leg in ("home", "park"):
        for r in (home_fail if leg == "home" else park_fail):
            c = category_of(r[leg].get("reason", ""))
            key = f"{leg}/{c}"
            cat[key] = cat.get(key, 0) + 1
    fence_detail_counts = {}
    for leg, lst in (("home", home_fail), ("park", park_fail)):
        for r in lst:
            d = fence_detail(r[leg].get("reason", ""))
            if d:
                fence_detail_counts[d] = fence_detail_counts.get(d, 0) + 1
    lift_counts = {}
    for leg, lst in (("home", home_fail), ("park", park_fail)):
        for r in lst:
            d = lift_kind(r[leg].get("reason", ""))
            if d:
                lift_counts[d] = lift_counts.get(d, 0) + 1

    # failures.csv / failures.txt：一行一个失败腿（同一点可能两行）
    rows = []
    for r in recs:
        for leg in ("home", "park"):
            info = r[leg]
            if info.get("ok"):
                continue
            rows.append({
                "i": r["i"], "leg": leg, "phase": info.get("phase", ""),
                "category": category_of(info.get("reason", "")),
                "etype": info.get("etype", ""),
                "reason": info.get("reason", ""),
                "fail_z": info.get("fail_z", ""),
                "q": _fmt_q(r["q"]), "tcp": _fmt_tcp(r["tcp"]) if r.get("tcp") else "",
                "start_ok": int(bool(r.get("start_ok"))),
                "start_reason": r.get("start_reason", ""),
            })
    with failures_csv.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else
                           ["i", "leg", "phase", "category", "etype", "reason",
                            "fail_z", "q", "tcp", "start_ok", "start_reason"])
        w.writeheader()
        w.writerows(rows)

    lines = []
    for r in recs:
        for leg, label in (("home", "回零"), ("park", "搁架")):
            info = r[leg]
            if info.get("ok"):
                continue
            z = (f"（失败高度 z={info['fail_z']:.1f}）"
                 if info.get("fail_z") is not None else "")
            start_txt = ("OK" if r.get("start_ok")
                         else "不合法(" + str(r.get("start_reason", "")) + ")")
            lines.append(
                f"#{r['i']:05d} {label} {info.get('phase', '')}{z}  "
                f"{info.get('reason', '')}\n"
                f"        q={_fmt_q(r['q'])}  tcp={_fmt_tcp(r['tcp']) if r.get('tcp') else '?'}"
                f"  起点{start_txt}")
    failures_txt.write_text("\n".join(lines) + ("\n" if lines else ""),
                            encoding="utf-8")

    # failures_by_point：一行一个"点"（同一点两腿失败只占一行）
    by_point_rows = []
    by_point_lines = []
    for r in recs:
        legs = [leg for leg in ("home", "park") if not r[leg].get("ok")]
        if not legs:
            continue
        legs_cn = "+".join("回零" if l == "home" else "搁架" for l in legs)
        start_txt = ("OK" if r.get("start_ok")
                     else "不合法(" + str(r.get("start_reason", "")) + ")")
        by_point_lines.append(
            f"#{r['i']:05d}  {legs_cn}  q={_fmt_q(r['q'])}  "
            f"tcp={_fmt_tcp(r['tcp']) if r.get('tcp') else '?'}  起点{start_txt}")
        merged = {}
        for leg in legs:
            info = r[leg]
            key = (info.get("phase", ""), info.get("reason", ""))
            merged.setdefault(key, []).append(leg)
        for (phase, reason), ls in merged.items():
            tag = "+".join("回零" if l == "home" else "搁架" for l in ls)
            by_point_lines.append(f"        {tag}/{phase}: {reason}")
        by_point_rows.append({
            "i": r["i"], "legs": legs_cn,
            "home_phase": r["home"].get("phase", ""),
            "home_reason": r["home"].get("reason", ""),
            "park_phase": r["park"].get("phase", ""),
            "park_reason": r["park"].get("reason", ""),
            "q": _fmt_q(r["q"]),
            "tcp": _fmt_tcp(r["tcp"]) if r.get("tcp") else "",
            "start_ok": int(bool(r.get("start_ok"))),
            "start_reason": r.get("start_reason", ""),
        })
    by_point_txt.write_text("\n".join(by_point_lines) + ("\n" if by_point_lines else ""),
                            encoding="utf-8")
    with by_point_csv.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(by_point_rows[0].keys())
                           if by_point_rows else
                           ["i", "legs", "home_phase", "home_reason", "park_phase",
                            "park_reason", "q", "tcp", "start_ok", "start_reason"])
        w.writeheader()
        w.writerows(by_point_rows)

    # failures_compact：一行一个失败点，只留编号/阶段/原报错
    compact = []
    for r in recs:
        if r["home"].get("ok") and r["park"].get("ok"):
            continue
        info = r["home"] if not r["home"].get("ok") else r["park"]
        compact.append(f"#{r['i']:05d}\t{info.get('phase', '')}\t"
                       f"{info.get('reason', '')}")
    compact_txt.write_text("\n".join(compact) + ("\n" if compact else ""),
                           encoding="utf-8")

    summary = {
        "points": len(recs),
        "expected_points": len(pts),
        "missing": missing,
        "home_ok": len(recs) - len(home_fail),
        "home_fail": len(home_fail),
        "park_ok": len(recs) - len(park_fail),
        "park_fail": len(park_fail),
        "both_fail": len(both),
        "either_fail": len(either),
        "start_invalid": len(start_bad),
        "start_invalid_home_fail": sum(1 for r in start_bad
                                       if not r["home"].get("ok")),
        "categories": dict(sorted(cat.items(), key=lambda kv: -kv[1])),
        "fence_details": dict(sorted(fence_detail_counts.items(),
                                     key=lambda kv: -kv[1])),
        "lift_kinds": dict(sorted(lift_counts.items(), key=lambda kv: -kv[1])),
        "elapsed_s": round(time.time() - t_start, 1),
        "out_dir": str(out_dir),
    }
    if run_json.exists():
        try:
            prev = json.loads(run_json.read_text(encoding="utf-8"))
            prev["elapsed_s"] = max(float(prev.get("elapsed_s") or 0.0),
                                    summary["elapsed_s"])
            prev["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            run_json.write_text(json.dumps(prev, ensure_ascii=False, indent=2) + "\n",
                                encoding="utf-8")
            summary["elapsed_s"] = prev["elapsed_s"]
        except Exception:  # noqa: BLE001
            pass
    summary_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")

    # ---------------- 只打印失败 ----------------
    pct = lambda a, b: (100.0 * a / b) if b else 0.0
    print("\n" + "=" * 78)
    print(f"扫描 {len(recs)} 点  TCP={settings['tcp']}  "
          f"config={version}")
    print(f"回零失败 {len(home_fail)} ({pct(len(home_fail), len(recs)):.1f}%)  "
          f"搁架失败 {len(park_fail)} ({pct(len(park_fail), len(recs)):.1f}%)  "
          f"两腿都过 {len(recs) - len(either)}  "
          f"两腿都失败 {len(both)}")
    print(f"起点围栏/力矩不合法 {len(start_bad)}（其中回零失败 "
          f"{sum(1 for r in start_bad if not r['home'].get('ok'))}）")
    print("按原因（按腿计，同一点可能两条腿各一条）：")
    for k, v in summary["categories"].items():
        print(f"  {k}: {v}")
    if summary["fence_details"]:
        print("围栏细分（按腿计）：")
        for k, v in summary["fence_details"].items():
            print(f"  {k}: {v}")
    if summary["lift_kinds"]:
        print("抬高失败细分（按腿计）：")
        for k, v in summary["lift_kinds"].items():
            print(f"  {k}: {v}")
    shown = min(args.print_failures, len(lines))
    head = "全部" if shown == len(lines) else f"前 {shown} 条"
    print(f"\n失败明细（共 {len(rows)} 条，下面显示{head}）：")
    for line in lines[:args.print_failures]:
        print(line)
    if len(lines) > args.print_failures:
        print(f"... 其余 {len(lines) - args.print_failures} 条见 {failures_txt}")
    print(f"\n文件（按腿）：{failures_csv}\n            {failures_txt}")
    print(f"文件（按点）：{by_point_csv}\n            {by_point_txt}")
    print(f"文件（一行一点）：{compact_txt}")
    print(f"汇总：{summary_json}")
    return 0


# --------------------------------------------------------------------------- #
# 自检：复制出来的两条腿必须和公开方法给出同样的分段
# --------------------------------------------------------------------------- #
def selftest():
    ex = Executor()
    m = ex.preview_motion
    m.preset = ex.tcp_preset
    m.yaw_deg = 0.0
    pk = ex.park_config()
    ctx = SimpleNamespace(ex=ex, m=m, park=pk, tcp=m.preset,
                          limits=([float(v) for v in m.limits[0]],
                                  [float(v) for v in m.limits[1]]),
                          z_min=float(ex.park_cfg.get("descent_min_j4_z") or 130.5),
                          step=float(ex.park_cfg.get("descent_step_mm") or 1.0),
                          descent=True)
    poses = [
        [0.0] * 6,
        [30.0, -20.0, 40.0, 90.0, 0.0, 0.0],
        [92.1962, 82.0133, 7.276, 57.851, 94.6588, 0.097],
        [115.968, -32.323, -124.471, 119.58, -60.746, 127.34],
    ]
    fails = []
    for q in poses:
        try:
            pub = m.plan_home(q)
            pub_t = [s["q_target"] for s in pub]
        except Exception as exc:  # noqa: BLE001
            pub_t = ("EXC", str(exc))
        mine = eval_home(ctx, q)
        if isinstance(pub_t, tuple):
            if mine["ok"]:
                fails.append(f"回零 {q}: 公开方法抛 {pub_t[1]}，复制流程却成功")
            continue
        # 重新生成一遍复制流程，比 q_target
        try:
            lift = m._plan_lift(q)
            cur = list(lift[-1]["q_target"]) if lift else list(q)
            rest = m._plan_move(cur, [0.0] * 6, "回零位")
            mine_t = [s["q_target"] for s in lift + rest]
        except Exception as exc:  # noqa: BLE001
            fails.append(f"回零 {q}: 复制流程抛 {exc}")
            continue
        if len(mine_t) != len(pub_t) or any(
                max(abs(a - b) for a, b in zip(x, y)) > 1e-3
                for x, y in zip(mine_t, pub_t)):
            fails.append(f"回零 {q}: 分段和 plan_home 不一致")
        try:
            pubp = m.plan_park(q, pk, include_down=False)
            pubp_t = [s["q_target"] for s in pubp["lift"] + pubp["tuck"] + pubp["pre"]]
        except Exception as exc:  # noqa: BLE001
            pubp_t = ("EXC", str(exc))
        pr = eval_park(ctx, q)
        if isinstance(pubp_t, tuple):
            if pr["ok"]:
                fails.append(f"搁架 {q}: 公开方法抛 {pubp_t[1]}，复制流程却成功")
            continue
        try:
            lift = m._plan_lift(q)
            cur = list(lift[-1]["q_target"]) if lift else list(q)
            tuck = m._plan_move(cur, pk["poses"]["tuck"], "收拢+折腕+转 J1")
            pre = m._plan_move(pk["poses"]["tuck"], pk["poses"]["pre"],
                               "J2/J3 到预停机点")
            minep_t = [s["q_target"] for s in lift + tuck + pre]
        except Exception as exc:  # noqa: BLE001
            fails.append(f"搁架 {q}: 复制流程抛 {exc}")
            continue
        if len(minep_t) != len(pubp_t) or any(
                max(abs(a - b) for a, b in zip(x, y)) > 1e-3
                for x, y in zip(minep_t, pubp_t)):
            fails.append(f"搁架 {q}: 分段和 plan_park 不一致")
        print(f"  {_fmt_q(q)}  home={mine['ok']}  park={pr['ok']}"
              f"  park阶段={pr['phase'] or '-'}")
    if fails:
        for f in fails:
            print("FAIL", f)
        return 1
    print("PASS 复制流程与 plan_home/plan_park(include_down=False) 完全一致")
    return 0


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="随机姿态 → 回零/回搁架 离线大样本扫描（只报失败）")
    p.add_argument("--points", type=int, default=8000,
                   help="生成点数（5000..10000 为常用范围；默认 8000）")
    p.add_argument("--workers", type=int,
                   default=max(1, (os.cpu_count() or 2) - 1),
                   help="进程数（默认 CPU-1）")
    p.add_argument("--seed", type=int, default=20261115)
    p.add_argument("--margin", type=float, default=5.0,
                   help="采样离安全限位的余量 deg（默认 5）")
    p.add_argument("--tcp", default=cfgmod.load_tcp().get("default", "flange"),
                   help="TCP 预设（默认读 config/tcp.json 的 default）")
    p.add_argument("--out", default="runs/_sim/sweep_" +
                   time.strftime("%Y%m%d_%H%M%S"))
    p.add_argument("--resume", default="", help="续跑已有目录")
    p.add_argument("--force", action="store_true", help="允许覆盖同目录 points.csv")
    p.add_argument("--batch", type=int, default=20)
    p.add_argument("--no-descent", action="store_true",
                   help="不做 168→最低点的 1 mm 分步，只查最低点一步（快但偏松）")
    p.add_argument("--print-failures", type=int, default=200,
                   help="控制台最多打印多少条失败（0=全部）")
    p.add_argument("--selftest", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.selftest:
        return selftest()
    if args.workers < 1:
        print("--workers 至少 1")
        return 2
    if args.print_failures <= 0:
        args.print_failures = 10 ** 9
    try:
        return run_sweep(args)
    except KeyboardInterrupt:
        return 130
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
