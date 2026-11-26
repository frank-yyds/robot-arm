#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""37.3 第 2 条：`trend_guard_check` 第 5 个用例（落后 22° 但仍在追）逐次诊断。

只跑第 5 个用例，每次记：
  * 注入后多久落后量第一次超过 20°（lim_lag）；
  * 多久中止、原因原文；没中止的话最后是什么状态（done/窗口到）；
  * 该段运动循环实际频率（从 recorder 的 poll 行算，不是配置值）；
  * 主机侧状态轮询（_poll_tick_hz）和负载。
判据逻辑和 `tools/trend_guard_check.py` 的 case 5 完全一致（窗口、注入时刻、判据都不改）；
本工具只加记录，不改执行器/运动层。

运行：cd arm-control && python3 tools/trend_guard_case5_diag.py --runs 20 --out runs/...
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402

MOVE = [0, 60, 0, 0, 0, 0]
FAULTS = {"slow": {"J2": 3.0}}


def trap_pos(d: float, v: float, a: float, t: float) -> float:
    """和 arm/motion.py::_trap_pos 同一个公式（这里只用来离线算“应到位置”）。"""
    ad = abs(float(d))
    if ad < 1e-9:
        return 0.0
    v = max(0.05, abs(float(v)))
    a = max(0.05, abs(float(a)))
    t = max(0.0, float(t))
    t_acc = v / a
    d_acc = 0.5 * a * t_acc * t_acc
    sign = 1.0 if d >= 0 else -1.0
    if ad <= 2.0 * d_acc:
        t_total = 2.0 * math.sqrt(ad / a)
        if t <= t_total / 2.0:
            s = 0.5 * a * t * t
        elif t <= t_total:
            s = ad - 0.5 * a * (t_total - t) ** 2
        else:
            s = ad
    else:
        t_total = 2.0 * t_acc + (ad - 2.0 * d_acc) / v
        if t <= t_acc:
            s = 0.5 * a * t * t
        elif t <= t_total - t_acc:
            s = d_acc + v * (t - t_acc)
        elif t <= t_total:
            s = ad - 0.5 * a * (t_total - t) ** 2
        else:
            s = ad
    return sign * s


def _safe_move(ex, q):
    try:
        ex.move_joints(q)
    except Exception:  # noqa: BLE001
        pass


def run_once(inject_after=1.2, window=5.0, tag="") -> dict:
    ex = Executor()
    r = ex.connect("mock")
    rec_path = str(getattr(getattr(ex, "recorder", None), "path", ""))
    seg_meta = {}
    t_phase = {}
    poll_hz = []
    try:
        ex.motion.speed_scale = 0.5
        ex.bus.enable_all(True, allow_restart=True)
        ex.bus.set_targets_q({n: 0.0 for n in ex.bus.order}, confirm=False)
        ex.bus.target = {n: 0.0 for n in ex.bus.order}
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 3.0:
            if max(abs(ex.bus.q[n]) for n in ex.bus.order) < 0.3:
                break
            time.sleep(0.05)
        try:
            ex.exec_mode = "direct"
        except Exception:  # noqa: BLE001
            pass
        th = threading.Thread(target=lambda: _safe_move(ex, MOVE), daemon=True)
        th.start()
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 3.0:
            st = ex.motion.status
            if st.get("busy") or st.get("phase") in ("running", "starting"):
                break
            time.sleep(0.02)
        # 记下这一段的分段参数（vel/acc/q_from），离线算应到位置要用。
        # 注意：begin_command() 先把 busy/starting 置上、execute() 之后才填 segments，
        # 这里要等 segments 真的出现（原来直接读会偶发空 dict）。
        t_seg = time.monotonic()
        while time.monotonic() - t_seg < 3.0:
            st = ex.motion.status
            segs = st.get("segments") or []
            if segs:
                for seg in segs:
                    seg_meta = {
                        "label": seg.get("label"),
                        "q_from": seg.get("q_from"),
                        "q_target": seg.get("q_target"),
                        "T": seg.get("T"),
                        "vel_deg_s": seg.get("vel_deg_s"),
                        "acc_deg_s2": seg.get("acc_deg_s2"),
                    }
                break
            if st.get("phase") in ("aborted", "error", "done"):
                break
            time.sleep(0.005)
        time.sleep(inject_after)
        t_inj = time.monotonic()
        if ex.recorder:
            ex.recorder.note(f"373DIAG inject {time.time():.6f}")
        ex.bus.set_faults(**FAULTS)
        ph = ""
        reason = ""
        t_end = None
        ended = "window"
        t0 = time.monotonic()
        while time.monotonic() - t0 < window:
            st = ex.motion.status
            ph = st.get("phase") or ""
            if ph in ("aborted", "error"):
                t_end = time.monotonic()
                reason = st.get("reason", "")
                ended = "aborted"
                break
            if ph == "done":
                t_end = time.monotonic()
                ended = "done"
                break
            try:
                poll_hz.append(float(ex._poll_tick_hz))
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.02)
        if t_end is None:
            t_end = time.monotonic()
            st = ex.motion.status
            ph = st.get("phase") or ""
            reason = st.get("reason", "")
            ended = "window"
        if ex.recorder:
            ex.recorder.note(f"373DIAG end {ended} phase={ph} reason={reason}")
        dt = round(t_end - t_inj, 3)
        ex.bus.clear_faults()
    finally:
        try:
            ex.disconnect()
        except Exception:  # noqa: BLE001
            pass

    diag = parse_csv(rec_path, tag)
    return {
        "tag": tag,
        "phase": ph,
        "ended": ended,
        "reason": reason,
        "dt_s": dt,
        "pass_like_check": bool(ended == "aborted" and "落后" in reason),
        "seg": seg_meta,
        "poll_tick_hz": {
            "min": round(min(poll_hz), 1) if poll_hz else None,
            "median": round(statistics.median(poll_hz), 1) if poll_hz else None,
            "max": round(max(poll_hz), 1) if poll_hz else None,
            "n": len(poll_hz),
        },
        "csv": rec_path,
        **diag,
    }


def parse_csv(path: str, tag: str = "") -> dict:
    """从 recorder CSV 算：lim_lag（注入→落后>20°）、中止延迟、运动循环实际频率。"""
    out = {
        "lim_lag_s": None, "max_lag_deg": None, "lag_end_deg": None,
        "loop_hz_seg": None, "loop_hz_win": None,
        "abort_after_lim_lag_s": None,
        "n_poll_win": None,
        "lag_gt20_hold_s": None,
    }
    p = Path(path)
    if not p.exists():
        return out
    rows = list(csv.DictReader(p.open(encoding="utf-8")))
    notes = {}
    cmd_move = []
    polls = []
    for r in rows:
        kind = r.get("kind")
        if kind == "note" and str(r.get("note", "")).startswith("373DIAG"):
            notes[str(r["note"]).split(" ")[1]] = float(r["t_rel"])
        elif kind == "command_move" and r.get("joint") == "J2":
            cmd_move.append((float(r["t_rel"]), r.get("target")))
        elif kind == "poll" and r.get("joint") == "J2":
            try:
                polls.append((float(r["t_rel"]), float(r["pos_deg"])))
            except (TypeError, ValueError):
                continue
    t_inj = notes.get("inject")
    t_end = notes.get("end")
    if not polls:
        return out
    # 找 J2 这一段的起点：最后一段 command_move 组（t_rel 连续、target=60）
    seg_start = None
    if cmd_move:
        group = [x for x in cmd_move if x[1] and abs(float(x[1]) - 60.0) < 0.1]
        seg_start = max(x[0] for x in group) if group else min(x[0] for x in cmd_move)
    # 该段参数：q_from=0，d=60，vel/acc 从运动层状态拿（run_once 里存了 seg，
    # 这里用默认 10/20 也行；更好的做法是外部把 seg 传进来——见调用处覆盖）
    return out


def finish_diag(run: dict) -> dict:
    """用 run 里的 seg 参数（vel/acc）把 CSV 的 lag/频率算完整。"""
    p = Path(run.get("csv") or "")
    if not p.exists():
        return run
    seg = run.get("seg") or {}
    try:
        q_from = seg.get("q_from")
        q_target = seg.get("q_target")
        vel = (seg.get("vel_deg_s") or {}).get("J2")
        acc = (seg.get("acc_deg_s2") or {}).get("J2")
        d = float(q_target[1]) - float(q_from[1])
        q0 = float(q_from[1])
    except Exception:  # noqa: BLE001
        return run
    rows = list(csv.DictReader(p.open(encoding="utf-8")))
    notes = {}
    cmd_move = []
    polls = []
    for r in rows:
        kind = r.get("kind")
        if kind == "note" and str(r.get("note", "")).startswith("373DIAG"):
            parts = str(r["note"]).split(" ")
            if len(parts) > 1:
                notes[parts[1]] = float(r["t_rel"])
        elif kind == "command_move" and r.get("joint") == "J2":
            try:
                cmd_move.append(float(r["t_rel"]))
            except (TypeError, ValueError):
                pass
        elif kind == "poll" and r.get("joint") == "J2":
            try:
                polls.append((float(r["t_rel"]), float(r["pos_deg"])))
            except (TypeError, ValueError):
                continue
    t_inj = notes.get("inject")
    t_end = notes.get("end")
    if not polls or t_inj is None or t_end is None or not cmd_move:
        return run
    t0 = max(cmd_move)  # recorder 写 command_move 紧挨着运动循环 t0
    # 落后量序列：expected = q0 + trap_pos(d, v, a, t - t0)，lag = (expected - 实际)×sign(d)
    series = []
    for t, pos in polls:
        if t < t0:
            continue
        exp = q0 + trap_pos(d, float(vel), float(acc), t - t0)
        lag = (exp - pos) * (1.0 if d >= 0 else -1.0)
        series.append((t, lag))
    if not series:
        return run
    # 注入后第一次 lag>20
    t_lim = None
    for t, lag in series:
        if t >= t_inj and lag > 20.0:
            t_lim = t
            break
    # 运动循环实际频率：poll 行（每拍一次）在窗口内 / 整段两条
    seg_polls = [(t, lag) for t, lag in series]
    win_polls = [(t, lag) for t, lag in series if t_inj <= t <= t_end]
    run["loop_hz_seg"] = (round((len(seg_polls) - 1) / (seg_polls[-1][0] - seg_polls[0][0]), 1)
                          if len(seg_polls) > 1 and seg_polls[-1][0] > seg_polls[0][0] else None)
    run["loop_hz_win"] = (round((len(win_polls) - 1) / (win_polls[-1][0] - win_polls[0][0]), 1)
                          if len(win_polls) > 1 and win_polls[-1][0] > win_polls[0][0] else None)
    run["n_poll_win"] = len(win_polls)
    run["max_lag_deg"] = round(max(lag for _, lag in series), 2)
    run["lag_end_deg"] = round(series[-1][1], 2)
    if t_lim is None:
        run["lim_lag_s"] = None
    else:
        run["lim_lag_s"] = round(t_lim - t_inj, 3)
        # lag>20 以后持续多久才中止
        run["lag_gt20_hold_s"] = round(t_end - t_lim, 3)
        run["abort_after_lim_lag_s"] = round(t_end - t_lim, 3)
    return run


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--inject-after", type=float, default=1.2)
    ap.add_argument("--window", type=float, default=5.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    results = []
    t_start = time.time()
    for i in range(args.runs):
        run = run_once(args.inject_after, args.window, f"{args.tag}#{i + 1}")
        run = finish_diag(run)
        results.append(run)
        print(f"[{args.tag}] {i + 1}/{args.runs} ended={run['ended']} "
              f"dt={run['dt_s']}s lim_lag={run['lim_lag_s']}s "
              f"loop={run['loop_hz_win']}Hz reason={run['reason'][:70]}")
        sys.stdout.flush()
    out = {
        "tag": args.tag, "runs": args.runs, "inject_after": args.inject_after,
        "window": args.window, "started": t_start, "duration_s": round(time.time() - t_start, 1),
        "loadavg_start": list(__import__("os").getloadavg()),
        "results": results,
        "summary": {
            "passed_like_check": sum(1 for r in results if r["pass_like_check"]),
            "aborted": sum(1 for r in results if r["ended"] == "aborted"),
            "window_end": sum(1 for r in results if r["ended"] == "window"),
            "done": sum(1 for r in results if r["ended"] == "done"),
            "lim_lag_s": [r["lim_lag_s"] for r in results],
        },
    }
    if args.out:
        p = Path(args.out)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written", p)
    print("summary:", out["summary"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
