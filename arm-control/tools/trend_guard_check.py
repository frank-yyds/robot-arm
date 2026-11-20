#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.2 第 8 条：运动趋势保护 mock 自检（五种注入各 20 次）+ 11-17 停机 CSV 离线重判。"""
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def wait_done(ex, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ph = ex.motion.status.get("phase")
        if ph in ("done", "aborted", "error"):
            return ph, ex.motion.status.get("reason", "")
        time.sleep(0.02)
    return "timeout", ""


def run_case(move, faults, inject_after=1.2, trial_time=6.0):
    ex = Executor()
    ex.connect("mock")
    ex.motion.speed_scale = 0.5
    ex.bus.enable_all(True, allow_restart=True)
    ex.bus.set_targets_q({n: 0.0 for n in ex.bus.order}, confirm=False)
    ex.bus.target = {n: 0.0 for n in ex.bus.order}
    # 等 mock 真的停在 0（慢机器上规划到执行之间臂还在动，会被"姿态和预演不一致"拦掉）
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 3.0:
        if max(abs(ex.bus.q[n]) for n in ex.bus.order) < 0.3:
            break
        time.sleep(0.05)
    try:
        ex.exec_mode = "direct"
    except Exception:  # noqa: BLE001
        pass
    th = threading.Thread(target=lambda: _safe_move(ex, move), daemon=True)
    th.start()
    # 慢机器上线程起来≠运动真的开始：等运动层进入 running/starting 再计时，
    # 否则注入可能落在起步 1 s 的容忍窗口里，判成"启动反向"而不是"背离"
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 3.0:
        st = ex.motion.status
        if st.get("busy") or st.get("phase") in ("running", "starting"):
            break
        time.sleep(0.02)
    time.sleep(inject_after)
    t_inj = time.monotonic()
    if faults:
        ex.bus.set_faults(**faults)
    t_abort = None
    reason = ""
    t0 = time.monotonic()
    while time.monotonic() - t0 < trial_time:
        if ex.motion.status.get("phase") in ("aborted", "error"):
            t_abort = time.monotonic()
            reason = ex.motion.status.get("reason", "")
            break
        if ex.motion.status.get("phase") == "done":
            break
        time.sleep(0.02)
    ph = ex.motion.status.get("phase")
    ex.bus.clear_faults()
    ex.disconnect()
    dt = None if t_abort is None else round(t_abort - t_inj, 2)
    return {"phase": ph, "reason": reason, "dt_s": dt}


def _safe_move(ex, q):
    try:
        ex.move_joints(q)
    except Exception:  # noqa: BLE001
        pass


def main():
    results = {}
    cases = [
        ("1 落后 8° 但一直在追", [0, 60, 0, 0, 0, 0],
         {"slow": {"J2": 8.0}}, "no_abort"),
        ("2 J3 背离 13°/s 电流 0", [0, 0, 60, 0, 0, 0],
         {"drift": {"J3": -13.0}, "currents": {"J3": 0.0}}, "away0"),
        ("3 J2 被推 5°/s ρ=1", [0, 60, 0, 0, 0, 0],
         {"drift": {"J2": -5.0}, "currents": {"J2": 4.0}}, "away1"),
        ("4 J2 堵住不动 ρ=1", [0, 60, 0, 0, 0, 0],
         {"stuck": ["J2"], "currents": {"J2": 4.0}}, "stuck"),
        ("5 落后 22° 但仍在追", [0, 60, 0, 0, 0, 0],
         {"slow": {"J2": 3.0}}, "fallback"),
    ]
    ok_all = True
    for name, move, faults, kind in cases:
        ok = 0
        details = []
        for _ in range(20):
            r = run_case(move, faults, trial_time=(5.0 if kind == "fallback" else 3.0))
            good = False
            if kind == "no_abort":
                good = not ("背离" in r["reason"] or "卡住" in r["reason"]
                            or "不动" in r["reason"] or "落后" in r["reason"])
            elif kind == "away0":
                good = r["phase"] == "aborted" and "背离" in r["reason"] and "没抵抗" in r["reason"] and (r["dt_s"] or 9) <= 0.6
            elif kind == "away1":
                good = r["phase"] == "aborted" and "背离" in r["reason"] and "被推走" in r["reason"]
            elif kind == "stuck":
                good = r["phase"] == "aborted" and "卡住或力矩不够" in r["reason"] and (r["dt_s"] or 9) <= 0.8
            elif kind == "fallback":
                good = r["phase"] == "aborted" and "落后" in r["reason"]
            ok += 1 if good else 0
            if not good and len(details) < 2:
                details.append({**r, "raw": r["reason"][:90]})
        results[name] = {"pass": ok, "fail": 20 - ok, "samples": details}
        ok_all &= ok == 20
        print(f"  {'过' if ok == 20 else '没过'}：{name}  {ok}/20")
        for d in details:
            print("      ", d)
    # 11-17 停机 CSV 离线重判
    replay = replay_incidents()
    for k, v in replay.items():
        print(f"  离线重判 {k}: {v}")
    print("结果：", "过" if ok_all else "没过")
    Path("runs").mkdir(exist_ok=True)
    (Path("runs") / f"trend_guard_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(
        json.dumps({"results": results, "replay": replay}, ensure_ascii=False, indent=1),
        encoding="utf-8")
    return 0 if ok_all else 1


def replay_incidents():
    """按新判法离线重放 11-17 的两次异常（无电流，ρ 记“无”）。"""
    import csv
    path = "runs/incident_20261117_J3/20261117_225853_run.csv"
    out = {}
    if not Path(path).exists():
        return out
    rows = list(csv.DictReader(open(path)))
    windows = [
        ("11-17 回零抬高段（J3）", "J3", "23:07:33.5", "23:07:38"),
        ("11-17 停机 50%（J2）", "J2", "23:07:40", "23:09:30"),
    ]
    for label, joint, t0, t1 in windows:
        polls = [r for r in rows if r.get("kind") == "poll" and r.get("joint") == joint
                 and t0 <= r["time"][11:] <= t1]
        samples = []
        for k in range(1, len(polls)):
            try:
                p0, p1 = float(polls[k-1]["pos_deg"]), float(polls[k]["pos_deg"])
                tgt = float(polls[k]["target_deg"]) if polls[k]["target_deg"] else None
                a, b = float(polls[k-1]["t_rel"]), float(polls[k]["t_rel"])
            except Exception:  # noqa: BLE001
                continue
            if tgt is None or b <= a:
                continue
            d = tgt - p1
            u = (1 if d >= 0 else -1) * (p1 - p0) / (b - a)
            samples.append({"d": d, "u": u})
        if not samples:
            continue
        away = sum(1 for x in samples if x["u"] <= -1.0)
        stuck = sum(1 for x in samples if abs(x["u"]) < 0.5 and abs(x["d"]) > 0.8)
        chase = sum(1 for x in samples if x["u"] >= 0.5)
        if away > len(samples) * 0.2:
            cat = "背离（ρ 无，不能分没抵抗/被推走）"
        elif stuck > len(samples) * 0.2:
            cat = "不动（ρ 无）"
        else:
            cat = "慢但在追（新判法不中止）"
        out[label] = {
            "category": cat,
            "samples": len(samples),
            "away": away, "stuck": stuck, "chasing": chase,
            "min_u": round(min(x["u"] for x in samples), 2),
            "max_lag_deg": round(max(abs(x["d"]) for x in samples), 2),
        }
    return out


if __name__ == "__main__":
    raise SystemExit(main())
