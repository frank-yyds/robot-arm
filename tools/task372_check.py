#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""37.2 只看记录：c 组第一次跑 6 段、27.3 d 俯仰偏差、t_lag / t_c。

只读 `runs/` 里的记录（两个会话的 events.jsonl + trace_*.csv），不碰真机、
不连 ECB、不改执行器。结果写 runs/<日期>_37/task372_check.json。

口径（写在回报里，事实与推测分开）：

1) c 组第一次跑的 6 段、重跑 6 段（36.3 第 3 条）：
   - 每段按 execute_start..execute_done 找覆盖它的 trace（tools/trace_match.py，
     不按"最新文件"拿）；起步反向 = 写目标后 1 s 内 (q − q起点)·sign(Δ) 的最小值
     （负=反向），q起点取写目标前最后 3 个位置的中位数；
   - 匀速段比 = 位移 30%–70% 的平均速度 ÷ 本段写入的 0x1F（°/s）；
   - 到位误差 = trace 末点 − 目标（trace 常常比 execute_done 早一点结束，
     所以另列 events 里 execute_done 前的 arrived_error 原文值）。

2) 27.3 d 的前臂俯仰偏差（27.2 第 2 条的定义）：
   每个样本 dev = (q2实测 + q3实测) − (q2应到 + q3应到)，带符号，报每段最大值。
   应到位置按 `arm/motion.py::_trap_pos` 的梯形规划公式复算（同一公式，本文件
   独立实现，避免 import 正在改的执行器代码）：
       q_from + _trap_pos(Δ, v, a, elapsed)
   q_from 取写目标前最后 3 个位置中位数；Δ/目标取 command_move 事件；v/a 取本段
   写入的 0x1F/0x20。elapsed 的时间基准：trace 的 `time` 列是整秒、t_rel 高精度；
   trace 的 t_rel 是在 0x94 读取之前打的（arm/trace.py），位置读数约晚
   L_POS_READ_S；执行器的 t0 在最后一次 command_move 记录之后。所以
   elapsed = (t_rel − t_switch) + L_POS_READ_S − (t0 − 写目标)。
   L_POS_READ_S 用 d3 的 trend_trace（执行器自己记的"落后"）标定：约 0.02 s。
   另按 L=0 / 0.04 给一组上下界（±0.1–0.3°，按该段速度）。

3) t_lag：每段、每个位移 >1° 的关节，从写目标（trace 里目标第一次变成新值）到
   位置朝目标变化 >0.02° 的时间。t_lag = t_cross − t_switch（trace 采样时刻口径，
   trace 的 t_rel 打在读之前、位置读数约晚 0.02 s，这点不加上去，偏小不到 0.02 s）。
   报全部段的最大值、中位数、段数；t_c = t_lag_max + √(0.2/a)，a 用该档该关节
   实际写入的 0x20（按该档所有段的 0x20 中位数取；写的是负减速度时取绝对值）。

用法：
    python3 tools/task372_check.py            # 全做，写到 runs/20261126_37/
    python3 tools/task372_check.py --out runs/20261126_37/task372_check.json
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import re
import statistics
import time
from pathlib import Path

import trace_match as tm

OLD_SESS = Path("runs/20261125_231905_hw")
NEW_SESS = Path("runs/20261126_091146_hw")
TRACE_GLOBS = ["runs/trace_20261125_2*.csv", "runs/trace_20261126_0*.csv"]

SPEED_RE = re.compile(r"=\s*([-\d.]+)\s*°/s")
LAG_RE = re.compile(r"最大落后\s*([-\d.]+)°")

L_POS_READ_S = 0.02      # （c 组/t_lag 用）trace 位置读数比 t_rel 晚约 0.02 s
L_DEFAULT_S = 0.05       # 俯仰偏差标定缺失时的 trace→t0 偏移估计（d1/d2/d3 实测 0.01–0.10）
T_CM_TO_T0_S = 0.005     # 最后一条 command_move 记完到 t0 的估计
TARGET_DEG = 0.02        # t_lag 的位移门槛（°）
MIN_DISP_T_LAG = 1.0     # t_lag 只算位移 >1° 的关节段


# ---------------------------------------------------------------- 基础读取
def load_events(session: Path) -> list[dict]:
    out = []
    p = session / "events.jsonl"
    for ln in p.read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def wall(s: str | None) -> dt.datetime | None:
    if not s:
        return None
    try:
        return dt.datetime.fromisoformat(s)
    except ValueError:
        return None


def actions_from_events(rows: list[dict]) -> list[dict]:
    """[execute_start .. execute_done / abort] 切成动作。"""
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
            if k == "execute_done":
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


def action_info(act: dict) -> dict:
    """把一段的 target / 写入 v,a,d / 0x0A 写目标时刻 / trend 落后取出来。"""
    info = {
        "start_wall": act["start"].get("wall"),
        "end_wall": (act.get("end") or act.get("abort") or {}).get("wall"),
        "aborted": bool(act.get("abort")),
        "targets": {}, "cm_wall": {}, "w0a": {}, "prof": {},
        "trend_max_lag": {}, "trend_text": {}, "trend_trace_rows": [],
    }
    for r in act["rows"]:
        k = r.get("kind")
        j = r.get("joint")
        if k == "command_move":
            if j not in info["targets"]:
                info["targets"][j] = r.get("target_deg")
                info["cm_wall"][j] = r.get("wall")
        elif k == "cmd":
            c = int(r.get("cmd") or -1)
            if c in (0x1F, 0x20, 0x21):
                m = SPEED_RE.search(r.get("text") or "")
                if m:
                    info["prof"].setdefault(j, {})[c] = float(m.group(1))
            elif c == 0x0A:
                info["w0a"].setdefault(j, []).append(r.get("wall"))
        elif k == "trend_segment":
            m = LAG_RE.search(r.get("text") or "")
            if m:
                info["trend_max_lag"][j] = float(m.group(1))
            info["trend_text"][j] = r.get("text")
        elif k == "trend_trace":
            info["trend_trace_rows"].append(r)
    return info


def trace_sample_series(tr: "tm.TraceFile", joint: str) -> list[dict]:
    return tr.samples(joint)


def switch_t(tr: "tm.TraceFile", joint: str, target: float,
             samples: list[dict]) -> float | None:
    """目标第一次变成 target 的 t_rel；目标本来就是这个值（不动关节）返回 None。"""
    changed = False
    for r in samples:
        if r["target"] is not None and abs(r["target"] - float(target)) > 1e-6:
            changed = True
            break
    if not changed:
        return None
    return tr.switch_t(joint, target)


def q_before(samples: list[dict], t: float, n: int = 3) -> float | None:
    vals = [r["pos"] for r in samples
            if r["pos"] is not None and r["t"] < t - 0.005][-n:]
    if not vals:
        vals = [r["pos"] for r in samples[:n] if r["pos"] is not None]
    return statistics.median(vals) if vals else None


def trap_pos(d: float, v: float, a: float, t: float) -> float:
    """和 arm/motion.py::_trap_pos 同一公式的独立实现（输入：位移、速度、加速度、秒）。"""
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


def speed_level(prof: dict, targets: dict, q0: dict) -> int | None:
    """从本段写入的 0x1F 推速度档：取动着的关节里最大的写入速度 ÷ 满速 20°/s。"""
    best = 0.0
    for j, d in prof.items():
        v = d.get(0x1F)
        if not v:
            continue
        disp = None
        if j in targets:
            try:
                disp = abs(float(targets[j]) - float(q0.get(j)))
            except (TypeError, ValueError):
                disp = None
        if disp is not None and disp <= 0.8:
            continue
        best = max(best, abs(v))
    if best <= 0:
        return None
    return int(round(best / 20.0 * 100.0 / 5.0) * 5)


# ---------------------------------------------------------------- c 组
def analyze_move(tr: "tm.TraceFile", info: dict, joint: str) -> dict | None:
    """一段一个关节：起步反向、匀速段比、到位误差。"""
    samples = trace_sample_series(tr, joint)
    if not samples:
        return None
    target = info["targets"].get(joint)
    if target is None:
        return None
    t_sw = switch_t(tr, joint, float(target), samples)
    if t_sw is None:
        return None
    q0 = q_before(samples, t_sw)
    if q0 is None:
        return None
    disp = float(target) - q0
    if abs(disp) < 0.5:
        return None
    prof = info["prof"].get(joint) or {}
    v = prof.get(0x1F)
    a = prof.get(0x20)
    sign = 1.0 if disp > 0 else -1.0
    win = [r for r in samples
           if r["pos"] is not None and t_sw <= r["t"] <= t_sw + 1.0]
    backs = [(r["pos"] - q0) * sign for r in win]
    rev = min(backs) if backs else None
    q30, q70 = q0 + 0.3 * disp, q0 + 0.7 * disp
    t30 = next((r["t"] for r in samples
                if r["t"] >= t_sw and r["pos"] is not None
                and sign * (r["pos"] - q30) >= 0), None)
    t70 = next((r["t"] for r in samples
                if r["t"] >= t_sw and r["pos"] is not None
                and sign * (r["pos"] - q70) >= 0), None)
    uniform_ratio = None
    uniform_avg = None
    if t30 is not None and t70 is not None and t70 > t30 and v:
        uniform_avg = 0.4 * abs(disp) / (t70 - t30)
        uniform_ratio = uniform_avg / abs(v)
    last = [r["pos"] for r in samples if r["pos"] is not None][-1]
    out = {
        "joint": joint, "trace": tr.path.name,
        "t_switch": round(t_sw, 3), "q0": round(q0, 4),
        "target": round(float(target), 4), "disp": round(disp, 3),
        "written_v": None if v is None else round(abs(v), 3),
        "written_a": None if a is None else round(abs(a), 3),
        "rev_1s": None if rev is None else round(rev, 3),
        "uniform_avg_deg_s": None if uniform_avg is None else round(uniform_avg, 3),
        "uniform_ratio": None if uniform_ratio is None else round(uniform_ratio, 3),
        "trace_last_err": round(last - float(target), 3),
        "arrived_error_events": None,
        "hz": tr.hz(),
    }
    return out


def c_group(trace_index) -> dict:
    rows = load_events(NEW_SESS)
    acts = actions_from_events(rows)
    out = {"first_run": [], "rerun": [], "unmatched": []}
    for tag, subset in (("first_run", acts[0:6]), ("rerun", acts[6:12])):
        for i, act in enumerate(subset, 1):
            info = action_info(act)
            t0, t1 = wall(info["start_wall"]), wall(info["end_wall"])
            tr = tm.find_trace_covering(trace_index, t0, t1, info["targets"])
            seg = {
                "seg": f"c{tag[:1]}#{i}",
                "start": info["start_wall"], "end": info["end_wall"],
                "aborted": info["aborted"],
                "targets": info["targets"],
                "level_pct": None,
                "trace": None, "trace_start_delta_s": None, "trace_end_delta_s": None,
                "joints": [],
            }
            if tr is None:
                out["unmatched"].append(seg["seg"])
                out[tag].append(seg)
                continue
            m = tm.describe_match(tr, t0, t1)
            seg.update({"trace": m["trace"],
                        "trace_start_delta_s": m["start_delta_s"],
                        "trace_end_delta_s": m["end_delta_s"]})
            # 位移 >1° 的关节逐行
            q0map = {}
            for j, target in info["targets"].items():
                samples = trace_sample_series(tr, j)
                if not samples:
                    continue
                t_sw = switch_t(tr, j, float(target), samples)
                q0 = q_before(samples, t_sw) if t_sw is not None else None
                if q0 is not None:
                    q0map[j] = q0
                row = analyze_move(tr, info, j)
                if row is None:
                    continue
                # execute_done 前的 arrived_error 原文值（trace 末点之外的另一口径）
                for r in act["rows"]:
                    if r.get("kind") == "arrived_error" and r.get("joint") == j:
                        row["arrived_error_events"] = r.get("target_deg")
                seg["joints"].append(row)
            seg["level_pct"] = speed_level(info["prof"], info["targets"], q0map)
            out[tag].append(seg)
    return out


# ---------------------------------------------------------------- 27.3 d 俯仰偏差
def pitch_segments(trace_index) -> dict:
    """d1、d2（旧会话 08:41、08:50 两轮）和 d3（新会话 09:28 那段）。"""
    out = {"rounds": [], "uncovered": []}
    rounds = [
        ("d1", OLD_SESS, "2026-11-26T08:41:19", "2026-11-26T08:48:04"),
        ("d2", OLD_SESS, "2026-11-26T08:50:16", "2026-11-26T08:57:01"),
        ("d3", NEW_SESS, "2026-11-26T09:28:00", "2026-11-26T09:28:37"),
    ]
    for name, session, lo, hi in rounds:
        acts = actions_from_events(load_events(session))
        sel = [a for a in acts if lo <= (a["start"].get("wall") or "") <= hi]
        rnd = {"round": name, "session": str(session), "segments": []}
        for idx, act in enumerate(sel, 1):
            info = action_info(act)
            t0, t1 = wall(info["start_wall"]), wall(info["end_wall"])
            tr = tm.find_trace_covering(trace_index, t0, t1, info["targets"])
            seg = {"seg": f"{name}-{idx}", "start": info["start_wall"],
                   "end": info["end_wall"], "aborted": info["aborted"],
                   "targets": {j: info["targets"][j] for j in ("J2", "J3")
                               if j in info["targets"]},
                   "trace": None, "covered": False,
                   "dev_max": None, "dev_max_at_s": None,
                   "dev_max_bound": {}, "joints": {}}
            if tr is None:
                out["uncovered"].append(seg["seg"])
                rnd["segments"].append(seg)
                continue
            seg.update({"trace": tr.path.name, "covered": True})
            analysis = pitch_one_segment(tr, info)
            if analysis:
                seg.update(analysis)
            rnd["segments"].append(seg)
        out["rounds"].append(rnd)
    return out


def _interp_pos(samples: list[dict], t: float) -> float | None:
    """按 t_rel 线性插值位置；t 超出范围返回 None。"""
    prev = None
    for r in samples:
        if r["pos"] is None:
            continue
        if prev is not None and prev["t"] <= t <= r["t"]:
            dt_ = r["t"] - prev["t"]
            if dt_ <= 0:
                return r["pos"]
            return prev["pos"] + (r["pos"] - prev["pos"]) * (t - prev["t"]) / dt_
        prev = r
    return None


def _trap_series(samples: list[dict], plan: dict, sw: float, k: float) -> list[tuple]:
    """给一个关节、一个时间偏移 k，返回 [(t_rel, actual, expected)]。"""
    out = []
    for r in samples:
        if r["pos"] is None:
            continue
        exp = plan["q0"] + trap_pos(plan["d"], plan["v"], plan["a"],
                                    (r["t"] - sw) + k)
        out.append((r["t"], r["pos"], exp))
    return out


def _recon_max_lag(series: list[tuple], sign: float, moving: bool,
                   q0: float, window=None) -> float:
    """按 26.2 的"落后"口径复算一颗的最大值（沿运动方向，正=实际落后）。"""
    vals = []
    for t, pos, exp in series:
        if window and not (window[0] <= t <= window[1]):
            continue
        if moving:
            vals.append((exp - pos) * sign)
        else:
            vals.append(abs(pos - q0))
    return max(vals) if vals else 0.0


def pitch_one_segment(tr: "tm.TraceFile", info: dict) -> dict | None:
    """一段的逐样本 dev；时间偏移用执行器自己的记录标定。

    dev(t) = (q2实测+q3实测) − (q2应到+q3应到)。应到 = q0 + _trap_pos(Δ,v,a,elapsed)，
    elapsed = (t_rel − 本段写目标后第一次采样的 t_rel) + k，k 是"trace 时间 → 执行器
    t0 时间"的偏移。k 的标定优先级：
      1. 中止段且有 trend_trace（执行器逐样本记的"落后"）：按这些点拟合 k；
      2. 有 trend_segment 记录的运动关节（|Δ|>1°）最大落后：解出让复算值等于记录值的 k；
      3. 都没有：k = L_DEFAULT − (t0 − 写目标)，L_DEFAULT=0.05。
    这样"应到"和执行器自己的参考对齐，不再靠猜 trace 读延迟。
    """
    tj = info["targets"].get("J2")
    tk = info["targets"].get("J3")
    if tj is None or tk is None:
        return None
    sj = trace_sample_series(tr, "J2")
    sk = trace_sample_series(tr, "J3")
    if not sj or not sk:
        return None
    sw_j = switch_t(tr, "J2", float(tj), sj)
    sw_k = switch_t(tr, "J3", float(tk), sk)
    switches = {"J2": sw_j, "J3": sw_k}
    moving_joints = [j for j, t in switches.items() if t is not None]
    if not moving_joints:
        return None
    t_sw = min(switches[j] for j in moving_joints)

    plans = {}
    for j in ("J2", "J3"):
        samples = sj if j == "J2" else sk
        target = float(info["targets"][j])
        sw = switches[j]
        q0 = q_before(samples, t_sw if sw is None else sw)
        if q0 is None:
            return None
        prof = info["prof"].get(j) or {}
        if sw is None:
            plans[j] = {"q0": q0, "d": 0.0, "v": 0.0, "a": 0.0,
                        "cm": info["cm_wall"].get(j),
                        "w0a": (info["w0a"].get(j) or [None])[0], "switch": None}
        else:
            plans[j] = {"q0": q0, "d": target - q0,
                        "v": abs(prof.get(0x1F) or 0.0),
                        "a": abs(prof.get(0x20) or 0.0),
                        "cm": info["cm_wall"].get(j),
                        "w0a": (info["w0a"].get(j) or [None])[0], "switch": sw}

    # 本段窗口：写目标后 → 到位/中止前（中止段按 abort 时刻截断）
    window_end = None
    for r in sj:
        if r["target"] is not None and abs(r["target"] - float(tj)) > 1e-6 and r["t"] > t_sw + 0.05:
            window_end = r["t"]
            break
    if info["aborted"]:
        end_wall = wall(info["end_wall"])
        w0a = plans["J2"]["w0a"] or plans["J3"]["w0a"]
        if end_wall is not None and w0a is not None:
            t_abort = t_sw + (end_wall - wall(w0a)).total_seconds()
            window_end = t_abort if window_end is None else min(window_end, t_abort)
    window = (t_sw - 0.005, window_end if window_end is not None else 1e9)

    def series_for(k: float, j: str):
        return _trap_series(sj if j == "J2" else sk, plans[j],
                            switches[j] if switches[j] is not None else t_sw, k)

    # ---- 标定 k ----
    k_used = None
    k_src = None
    residual = None
    # 1) 中止段：trend_trace 逐样本"落后"
    lag_trace = []
    for r in info.get("trend_trace_rows", []):
        if r.get("joint") != "J2":
            continue
        m = re.match(r"t=([\d.]+)s 落后 (-?[\d.]+)°", r.get("text") or "")
        if not m:
            continue
        lag_trace.append((float(m.group(1)), float(m.group(2))))
    if lag_trace and switches["J2"] is not None:
        def err(k):
            plan = plans["J2"]
            s = 0.0
            for tt, lag_rec in lag_trace:
                pos = _interp_pos(sj, switches["J2"] + tt - k)
                if pos is None:
                    return 1e9
                exp = plan["q0"] + trap_pos(plan["d"], plan["v"], plan["a"], tt)
                s += (exp - pos - lag_rec) ** 2
            return s
        best = None
        for i in range(-200, 201):
            k = i / 1000.0
            e = err(k)
            if best is None or e < best[1]:
                best = (k, e)
        k_used, k_src = best[0], "trend_trace"
        residual = None
        if lag_trace:
            residual = round(max(abs(
                (plans["J2"]["q0"] + trap_pos(plans["J2"]["d"], plans["J2"]["v"],
                                              plans["J2"]["a"], tt))
                - (_interp_pos(sj, switches["J2"] + tt - k_used) or 0.0)
                - rec) for tt, rec in lag_trace), 3)
    # 2) trend_segment 的最大落后（运动关节）
    if k_used is None:
        cand = []
        for j in ("J2", "J3"):
            p = plans[j]
            sw = switches[j]
            rec = info["trend_max_lag"].get(j)
            if sw is None or rec is None or abs(p["d"]) <= 1.0:
                continue
            sign = 1.0 if p["d"] >= 0 else -1.0
            f = lambda k, j=j, sign=sign: _recon_max_lag(
                series_for(k, j), sign, True, p["q0"], window)
            lo, hi = -0.30, 0.30
            f_lo, f_hi = f(lo), f(hi)
            if f_lo > rec or f_hi < rec:
                cand.append((abs(f_lo - rec), abs(f_hi - rec), None))
                continue
            for _ in range(48):
                mid = (lo + hi) / 2.0
                if f(mid) < rec:
                    lo = mid
                else:
                    hi = mid
            cand.append((0.0, 0.0, (lo + hi) / 2.0))
        if cand:
            ok = [c for c in cand if c[2] is not None]
            if ok:
                k_used = statistics.mean(c[2] for c in ok)
                k_src = "trend_segment"
    # 3) 缺省
    if k_used is None:
        if plans["J2"]["cm"] is not None and plans["J2"]["w0a"] is not None:
            dt_cm = (wall(plans["J2"]["cm"]) + dt.timedelta(seconds=T_CM_TO_T0_S)
                     - wall(plans["J2"]["w0a"])).total_seconds()
        else:
            dt_cm = 0.09
        k_used = L_DEFAULT_S - dt_cm
        k_src = "default"

    def dev_series(k: float):
        common = t_sw
        s2 = _trap_series(sj, plans["J2"], switches["J2"] if switches["J2"] is not None else common, k)
        s3 = _trap_series(sk, plans["J3"], switches["J3"] if switches["J3"] is not None else common, k)
        d2 = {round(t, 4): pos - exp for t, pos, exp in s2}
        d3 = [(t, pos - exp) for t, pos, exp in s3]
        if not d2 or not d3:
            return []
        import bisect
        t3 = [x[0] for x in d3]
        v3 = [x[1] for x in d3]
        out = []
        for t in sorted(d2):
            if t < t3[0] - 0.05 or t > t3[-1] + 0.05:
                continue
            i = bisect.bisect_left(t3, t)
            if i <= 0:
                q = v3[0]
            elif i >= len(t3):
                q = v3[-1]
            else:
                a0, b0 = t3[i - 1], v3[i - 1]
                a1, b1 = t3[i], v3[i]
                q = b0 if a1 <= a0 else b0 + (b1 - b0) * (t - a0) / (a1 - a0)
            if window[0] <= t <= window[1]:
                out.append((t, d2[t] + q))
        return out

    base = dev_series(k_used)
    if not base:
        return None
    dev_max_t, dev_max = max(base, key=lambda x: x[1])
    # k 上下 0.03 s 的敏感度（trace 时间对时的不确定度）
    bounds = {}
    for tag, dk in (("k-0.03", -0.03), ("k+0.03", 0.03)):
        s2 = dev_series(k_used + dk)
        if s2:
            bounds[tag] = round(max(x[1] for x in s2), 3)

    # 对拍：同一 k 下复算的每颗最大落后 vs trend_segment 原文。
    # 运动关节的 k 是拿它的记录值标定的（不是独立验证）；不动的关节 k 不影响
    # 它的"落后"（|q−q起点| 的最大值），那个才算独立对拍。
    recon_lag = {}
    for j in ("J2", "J3"):
        p = plans[j]
        sign = 1.0 if p["d"] >= 0 else -1.0
        recon_lag[j] = round(_recon_max_lag(
            series_for(k_used, j), sign, abs(p["d"]) > 0.1, p["q0"], window), 3)
    return {
        "dev_max": round(dev_max, 3),
        "dev_max_at_s": round(dev_max_t, 3),
        "dev_max_bound": bounds,
        "k_used_s": round(k_used, 4),
        "k_src": k_src,
        "k_fit_max_residual": residual if k_src == "trend_trace" else None,
        "recon_max_lag": recon_lag,
        "recorded_max_lag": {j: info["trend_max_lag"].get(j)
                             for j in ("J2", "J3")
                             if j in info["trend_max_lag"]},
        "plans": {j: {"q0": round(plans[j]["q0"], 3),
                      "target": round(info["targets"][j], 3),
                      "d": round(plans[j]["d"], 3),
                      "written_v": round(plans[j]["v"], 3),
                      "written_a": round(plans[j]["a"], 3)}
                  for j in plans},
    }


# ---------------------------------------------------------------- t_lag / t_c
def t_lag_all(trace_index) -> dict:
    """全部有 trace 的段、每个位移 >1° 的关节量 t_lag。"""
    rows = []
    covered = unmatched = 0
    for session in (OLD_SESS, NEW_SESS):
        acts = actions_from_events(load_events(session))
        for act in acts:
            info = action_info(act)
            t0, t1 = wall(info["start_wall"]), wall(info["end_wall"])
            if t0 is None or t1 is None:
                continue
            tr = tm.find_trace_covering(trace_index, t0, t1, info["targets"])
            if tr is None:
                unmatched += 1
                continue
            covered += 1
            q0map = {}
            for j, target in info["targets"].items():
                samples = trace_sample_series(tr, j)
                if not samples:
                    continue
                t_sw = switch_t(tr, j, float(target), samples)
                if t_sw is None:
                    continue
                q0 = q_before(samples, t_sw)
                if q0 is None:
                    continue
                q0map[j] = q0
                disp = float(target) - q0
                if abs(disp) <= MIN_DISP_T_LAG:
                    continue
                prof = info["prof"].get(j) or {}
                v = abs(prof.get(0x1F) or 0.0)
                a = abs(prof.get(0x20) or 0.0)
                if v <= 0 or a <= 0:
                    continue
                sign = 1.0 if disp > 0 else -1.0
                t_cross = next((r["t"] for r in samples
                                if r["t"] > t_sw and r["pos"] is not None
                                and sign * (r["pos"] - q0) > TARGET_DEG), None)
                if t_cross is None:
                    continue
                rows.append({
                    "session": session.name,
                    "seg_start": info["start_wall"],
                    "seg_label": (act["start"].get("text") or ""),
                    "aborted": info["aborted"],
                    "trace": tr.path.name,
                    "joint": j,
                    "disp": round(disp, 3),
                    "written_v": round(v, 3),
                    "written_a": round(a, 3),
                    "level_pct": None,
                    "t_switch": round(t_sw, 4),
                    "t_cross": round(t_cross, 4),
                    "t_lag_s": round(t_cross - t_sw, 4),
                })
            lvl = speed_level(info["prof"], info["targets"], q0map)
            for r in rows:
                if r["seg_start"] == info["start_wall"] and r["level_pct"] is None:
                    r["level_pct"] = lvl
    lags = [r["t_lag_s"] for r in rows]
    summary = {
        "segments_with_trace": covered,
        "segments_without_trace": unmatched,
        "joint_segments": len(rows),
        "t_lag_max_s": None if not lags else round(max(lags), 4),
        "t_lag_median_s": None if not lags else round(statistics.median(lags), 4),
        "t_lag_min_s": None if not lags else round(min(lags), 4),
    }
    # 每档每关节：a 取该档该关节写入 0x20 的中位数；t_c = t_lag_max + sqrt(0.2/a)
    by = {}
    for r in rows:
        key = (r["level_pct"], r["joint"])
        by.setdefault(key, []).append(r)
    t_c = []
    t_lag_max = summary["t_lag_max_s"]
    for (lvl, j), rs in sorted(by.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
        accs = [x["written_a"] for x in rs if x["written_a"]]
        if not accs:
            continue
        a_med = statistics.median(accs)
        if t_lag_max is None or a_med <= 0:
            continue
        t_c.append({
            "level_pct": lvl, "joint": j,
            "n_segments": len(rs),
            "a_used_deg_s2": round(a_med, 3),
            "t_lag_max_s": t_lag_max,
            "t_c_s": round(t_lag_max + math.sqrt(0.2 / a_med), 4),
            "written_a_values": sorted({round(x, 3) for x in accs}),
        })
    # 25%/50% 两档补齐 6 颗：没有实测加速度的关节用名义 a = 2 × 20°/s × 档位
    # （motion.py 的 accel = v / motion_accel_time_s，六颗 motion_accel_time_s 都是 0.5），
    # 标 nominal=true。
    have = {(r["level_pct"], r["joint"]) for r in t_c}
    acc_map = {(r["level_pct"], r["joint"]): r["a_used_deg_s2"] for r in t_c}
    joint_order = ["J1", "J2", "J3", "J4", "J5", "J6"]
    for lvl in sorted({r["level_pct"] for r in rows if r["level_pct"] in (25, 50)}):
        for j in joint_order:
            if (lvl, j) in have:
                continue
            a_nom = 2.0 * 20.0 * lvl / 100.0
            if t_lag_max is None:
                continue
            t_c.append({
                "level_pct": lvl, "joint": j,
                "n_segments": 0,
                "a_used_deg_s2": round(a_nom, 3),
                "t_lag_max_s": t_lag_max,
                "t_c_s": round(t_lag_max + math.sqrt(0.2 / a_nom), 4),
                "written_a_values": [],
                "nominal": True,
            })
    t_c.sort(key=lambda r: (r["level_pct"], joint_order.index(r["joint"])))
    return {"rows": rows, "summary": summary, "t_c": t_c}


# ---------------------------------------------------------------- main
def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)
    trace_index = tm.build_index(TRACE_GLOBS)
    out = {
        "generated": dt.datetime.now().isoformat(timespec="seconds"),
        "sources": {
            "old_session": str(OLD_SESS),
            "new_session": str(NEW_SESS),
            "traces": TRACE_GLOBS,
            "n_traces": len(trace_index),
        },
        "convention": {
            "L_POS_READ_S": L_POS_READ_S,
            "T_CM_TO_T0_S": T_CM_TO_T0_S,
            "TARGET_DEG": TARGET_DEG,
            "MIN_DISP_T_LAG": MIN_DISP_T_LAG,
        },
        "c_group": c_group(trace_index),
        "pitch": pitch_segments(trace_index),
        "t_lag": t_lag_all(trace_index),
    }
    # 把 t_lag 的 rows 精简到输出里（全部段都进 JSON）
    p = Path(args.out) if args.out else Path(
        "runs") / f"{time.strftime('%Y%m%d')}_37" / "task372_check.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"结果写 {p}")

    print("\n== c 组第一次跑 6 段 ==")
    for seg in out["c_group"]["first_run"]:
        for row in seg["joints"]:
            print(f"  {seg['seg']} {seg['start'][:19]} {row['joint']} "
                  f"disp {row['disp']:+.2f} 起步反向 {row['rev_1s']:+.3f} "
                  f"匀速/v {row['uniform_ratio']} (v={row['written_v']}) "
                  f"末点误差 {row['trace_last_err']:+.3f} trace {row['trace']}")
    print("\n== c 组重跑 6 段 ==")
    for seg in out["c_group"]["rerun"]:
        for row in seg["joints"]:
            print(f"  {seg['seg']} {seg['start'][:19]} {row['joint']} "
                  f"disp {row['disp']:+.2f} 起步反向 {row['rev_1s']:+.3f} "
                  f"匀速/v {row['uniform_ratio']} (v={row['written_v']}) "
                  f"末点误差 {row['trace_last_err']:+.3f} trace {row['trace']}")
    print("\n== 27.3 d 俯仰偏差 ==")
    for rnd in out["pitch"]["rounds"]:
        for seg in rnd["segments"]:
            print(f"  {seg['seg']} {seg['start'][:19]} covered={seg['covered']} "
                  f"dev_max {seg.get('dev_max')} 上下界 {seg.get('dev_max_bound')} "
                  f"对拍落后 {seg.get('recon_max_lag')} / 记录 {seg.get('recorded_max_lag')} "
                  f"trace {seg.get('trace')}")
    print("\n== t_lag ==")
    s = out["t_lag"]["summary"]
    print(f"  段数 {s['joint_segments']}（有 trace 的动作 {s['segments_with_trace']}，"
          f"无 trace {s['segments_without_trace']}）；"
          f"max {s['t_lag_max_s']} s，中位 {s['t_lag_median_s']} s，"
          f"min {s['t_lag_min_s']} s")
    for r in out["t_lag"]["rows"]:
        if r["t_lag_s"] >= 0.05:
            print(f"  长 {r['t_lag_s']}s {r['seg_start'][:19]} {r['joint']} "
                  f"档 {r['level_pct']}% a={r['written_a']} trace {r['trace']}")
    print("\n== t_c ==")
    for r in out["t_lag"]["t_c"]:
        print(f"  {r['level_pct']}% {r['joint']} a={r['a_used_deg_s2']} "
              f"n={r['n_segments']} t_c={r['t_c_s']} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
