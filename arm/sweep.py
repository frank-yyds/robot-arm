#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""22.2 往返扫描标定：力矩系数 Kt、摩擦电流、电流偏置、零偏。

做法（Mac_agent待办.md §22.2）：
  * 其余关节摆到固定姿态保持；被测关节以 3°/s 匀速走一个来回；
  * 去程电流 = 重力部分 + 摩擦，回程 = 重力部分 − 摩擦；
  * I_实测 = k × τ_模型(q + δ) + f × 方向 + c，δ 在 ±5° 里每 0.5° 试一格，
    每格对 (k, f, c) 做线性最小二乘，取残差最小的一格；
  * Kt_eff = 1/k，结果写进 config/kt.json（新增 i_fric / i_bias /
    zero_offset_deg / rms 字段）。

本文件不碰 ECB：采集通过 Motion 的普通单段运动完成，真机/ mock 共用。
"""
from __future__ import annotations

import csv
import math
import threading
import time
from pathlib import Path

import numpy as np

from . import gravity

# 每个关节的扫描定义；hold 是其余关节的姿态（被测关节位单独填）。
# J5 空法兰重力力矩接近 0，等夹爪装回再做（22.2 表）。
SWEEP_JOBS: dict[str, dict] = {
    "J2": {"joint": "J2", "index": 1, "hold": [0, 0, 0, 0, 0, 0],
           "preferred_start": -30.0, "fallback_start": 0.0, "end": 80.0,
           "nominal_kt": 1.91, "r2_min": 0.98, "defer": False,
           "note": "围栏允许就从 −30° 起，否则 0°→+80°"},
    "J3": {"joint": "J3", "index": 2, "hold": [0, 0, 0, 0, 0, 0],
           "preferred_start": -90.0, "fallback_start": -90.0, "end": 90.0,
           "nominal_kt": 1.35, "r2_min": 0.98, "defer": False,
           "note": "−90°→+90° 往返"},
    "J4": {"joint": "J4", "index": 3, "hold": [0, 0, 0, 0, 0, 0],
           "preferred_start": -90.0, "fallback_start": -90.0, "end": 90.0,
           "nominal_kt": None, "r2_min": 0.90, "defer": False,
           "note": "−90°→+90° 往返；空法兰力矩约 0.22 N·m"},
    "J5": {"joint": "J5", "index": 4, "hold": [0, 0, 90, 0, 0, 0],
           "preferred_start": -90.0, "fallback_start": -90.0, "end": 90.0,
           "nominal_kt": None, "r2_min": 0.90, "defer": True,
           "note": "空法兰接近 0，等夹爪装回再做"},
}

DELTA_GRID_DEG = [round(-5.0 + 0.5 * i, 1) for i in range(21)]
EDGE_DROP_DEG = 5.0


def _with_joint(hold, idx: int, value: float) -> list[float]:
    q = [float(v) for v in hold]
    q[idx] = float(value)
    return q


def _torque_max_ratio(motion, q0, q1, step=0.5) -> dict:
    """沿路径每 0.5° 算一次 18.2 力矩占容量比例，返回最大值和位置。"""
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.max(np.abs(q1 - q0)))
    n = max(1, int(math.ceil(d / step)))
    cap = gravity.capacities(
        {k: v for k, v in (motion.limit_a or {}).items() if v is not None},
        getattr(motion, "kt", {}) or {},
        getattr(motion, "j2_bracket", "PLA"))
    out = {"max_ratio": 0.0, "joint": "", "tau": 0.0, "capacity": None,
           "q": None, "points": n + 1}
    for k in range(n + 1):
        q = q0 + (q1 - q0) * (k / n)
        tau = gravity.torques(motion.robot, q, motion.end_effector,
                              float(getattr(motion, "payload_kg", 0.0) or 0.0))
        for i in (1, 2, 3, 4):
            name = f"J{i + 1}"
            c = cap.get(name)
            if c is None or not math.isfinite(float(c)) or float(c) <= 0:
                continue
            r = abs(float(tau[i])) / float(c)
            if r > out["max_ratio"]:
                out = {"max_ratio": round(r, 4), "joint": name,
                       "tau": round(float(tau[i]), 3),
                       "capacity": round(float(c), 3),
                       "q": [round(float(v), 2) for v in q],
                       "points": n + 1}
    return out


def plan_sweep(motion, joint: str, start: float | None = None,
               end: float | None = None) -> dict:
    """只规划、不运动：返回往返两程的段和所有检查结果。

    优先用 preferred_start（J2 = −30°）；被围栏/力矩拦下时自动退回
    fallback_start，并把原因写进 warnings（明天真机按这个结果走）。
    """
    job = SWEEP_JOBS[str(joint).upper()]
    if job["defer"]:
        return {"ok": False, "joint": job["joint"], "deferred": True,
                "reason": job["note"], "warnings": [], "out": [], "back": []}
    idx = job["index"]
    candidates = []
    if start is not None:
        candidates.append(float(start))
    else:
        candidates.extend([job["preferred_start"], job["fallback_start"]])
    end_v = float(job["end"] if end is None else end)
    warnings: list[str] = []
    last_exc: Exception | None = None
    for s in candidates:
        q_start = _with_joint(job["hold"], idx, s)
        q_end = _with_joint(job["hold"], idx, end_v)
        try:
            out = motion._plan_move(q_start, q_end,
                                    f"22.2 {job['joint']} 去程 {s:g}→{end_v:g}")
            back = motion._plan_move(q_end, q_start,
                                     f"22.2 {job['joint']} 回程 {end_v:g}→{s:g}")
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if s != job["fallback_start"]:
                warnings.append(
                    f"{job['joint']} 起点 {s:g}° 被拦：{exc}；改用 "
                    f"{job['fallback_start']:g}°")
                continue
            return {"ok": False, "joint": job["joint"], "deferred": False,
                    "reason": str(exc), "warnings": warnings,
                    "start": s, "end": end_v, "out": [], "back": [],
                    "checks": {}}
        # 路径检查点密度（两程各自）
        pc = []
        for seg in out + back:
            meta = seg.get("path_check") or {}
            if meta:
                pc.append({"label": seg["label"],
                           "n_points": meta.get("n_points"),
                           "max_step_deg": meta.get("max_step_deg")})
        ratio = _torque_max_ratio(motion, q_start, q_end)
        ratio_back = _torque_max_ratio(motion, q_end, q_start)
        checks = {"out_segments": len(out), "back_segments": len(back),
                  "path_checks": pc,
                  "torque_max_ratio": max(ratio["max_ratio"],
                                          ratio_back["max_ratio"]),
                  "torque_worst": (ratio if ratio["max_ratio"] >=
                                   ratio_back["max_ratio"] else ratio_back),
                  "start": q_start, "end": q_end}
        if s != job["preferred_start"] and start is None:
            warnings.append(
                f"{job['joint']} 按 {s:g}° 起（首选 "
                f"{job['preferred_start']:g}° 被拦）")
        return {"ok": True, "joint": job["joint"], "deferred": False,
                "reason": "", "warnings": warnings, "start": s, "end": end_v,
                "out": out, "back": back, "checks": checks,
                "last_exc": None if last_exc is None else str(last_exc)}
    return {"ok": False, "joint": job["joint"], "deferred": False,
            "reason": str(last_exc), "warnings": warnings,
            "out": [], "back": []}


def fit_sweep(samples: list[dict], joint: str, robot,
              end_effector: str = "flange", payload_kg: float = 0.0,
              nominal_kt: float | None = None) -> dict:
    """I = k·τ(q+δ) + f·dir + c；δ 网格搜索，线性最小二乘拟合其余三项。"""
    job = SWEEP_JOBS[str(joint).upper()]
    idx = job["index"]
    rows: list[dict] = []
    for r in samples:
        try:
            q = [float(x) for x in r["q"]]
            i = float(r["current"])
            d = 1.0 if int(r["dir"]) >= 0 else -1.0
        except Exception:  # noqa: BLE001
            continue
        if not all(math.isfinite(x) for x in q + [i]):
            continue
        rows.append({"q": q, "current": i, "dir": d})
    if len(rows) < 20:
        return {"ok": False, "joint": job["joint"], "n": len(rows),
                "reason": "样本不足（<20）"}
    qs = np.array([r["q"][idx] for r in rows], float)
    q_lo = float(np.percentile(qs, 1.0))
    q_hi = float(np.percentile(qs, 99.0))
    keep = [r for r in rows
            if q_lo + EDGE_DROP_DEG <= r["q"][idx] <= q_hi - EDGE_DROP_DEG]
    if len(keep) < 10:
        keep = rows
    cur = np.array([r["current"] for r in keep], float)
    dirs = np.array([r["dir"] for r in keep], float)
    best = None
    for delta in DELTA_GRID_DEG:
        taus = np.empty(len(keep), float)
        for j, r in enumerate(keep):
            q2 = list(r["q"])
            q2[idx] += delta
            taus[j] = float(gravity.torques(robot, q2, end_effector,
                                            float(payload_kg or 0.0))[idx])
        A = np.column_stack([taus, dirs, np.ones(len(keep))])
        try:
            with np.errstate(all="ignore"):
                (k, f, c), *_ = np.linalg.lstsq(A, cur, rcond=None)
                pred = A @ np.array([k, f, c])
        except Exception:  # noqa: BLE001
            continue
        if not (math.isfinite(k) and math.isfinite(f) and math.isfinite(c)):
            continue
        rms = float(np.sqrt(np.mean((cur - pred) ** 2)))
        if not math.isfinite(rms):
            continue
        if best is None or rms < best["rms"]:
            best = {"delta": delta, "k": float(k), "f": float(f),
                    "c": float(c), "rms": rms, "pred": pred}
    if best is None:
        return {"ok": False, "joint": job["joint"], "n": len(keep),
                "reason": "拟合失败"}
    ss_res = float(np.sum((cur - best["pred"]) ** 2))
    ss_tot = float(np.sum((cur - float(np.mean(cur))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    k = best["k"]
    kt_eff = (1.0 / k) if abs(k) > 1e-12 else None
    nom = nominal_kt if nominal_kt is not None else job.get("nominal_kt")
    kt_err_pct = (None if (kt_eff is None or not nom)
                  else 100.0 * (kt_eff - nom) / nom)
    checks = {
        "kt_in_15pct": (None if kt_err_pct is None else abs(kt_err_pct) <= 15.0),
        "zero_offset_lt_1": abs(best["delta"]) < 1.0,
        "bias_lt_friction": abs(best["c"]) < abs(best["f"]),
        "r2_ok": r2 >= float(job["r2_min"]),
    }
    return {
        "ok": (kt_eff is not None
               and all(v is not False for v in checks.values())),
        "joint": job["joint"], "n": len(keep), "n_all": len(samples),
        "kt_eff": None if kt_eff is None else round(float(kt_eff), 6),
        "k": round(float(k), 6), "i_fric": round(float(best["f"]), 6),
        "i_bias": round(float(best["c"]), 6),
        "zero_offset_deg": round(float(best["delta"]), 3),
        "rms": round(float(best["rms"]), 5), "r2": round(float(r2), 5),
        "kt_err_pct": None if kt_err_pct is None else round(float(kt_err_pct), 2),
        "nominal_kt": nom, "checks": checks,
        "q_range": [round(q_lo, 2), round(q_hi, 2)],
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


class SweepRunner:
    """在后台线程里跑一条扫描；状态给执行器/网页读。"""

    def __init__(self, executor, joint: str, speed_deg_s: float = 3.0,
                 save: bool = True):
        self.executor = executor
        self.joint = str(joint).upper()
        self.speed = max(0.5, float(speed_deg_s))
        self.save = bool(save)
        self.samples: list[dict] = []
        self.path = ""
        self._sampling = False
        self._dir = 1
        self._label = ""
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.status = {
            "active": False, "joint": self.joint, "phase": "idle",
            "reason": "", "samples": 0, "start": None, "end": None,
            "fit": None, "path": "", "started": None, "finished": None,
            "warnings": [], "checks": None,
        }

    # ---------------- 钩子 ----------------
    def feed(self, states, cmd_q, cmd_vel, phase, label, elapsed, duration):
        if not self._sampling or phase != "run" or states is None:
            return
        st = states.get(self.joint)
        if st is None or st.current_a is None or st.out_deg is None:
            return
        q = []
        for n in getattr(self.executor, "robot").joint_names:
            s = states.get(n)
            if s is None or s.out_deg is None:
                return
            q.append(float(s.out_deg))
        rec = {
            "t": time.time(), "q": [round(v, 5) for v in q],
            "current": round(float(st.current_a), 6),
            "current_raw": st.current_raw,
            "full_scale_a": st.current_full_scale_a,
            "dir": self._dir, "phase": self._label,
        }
        with self._lock:
            self.samples.append(rec)
        self.status["samples"] = len(self.samples)

    # ---------------- 采集 ----------------
    @staticmethod
    def _cap(segs, speed):
        for seg in segs:
            d = np.asarray(seg["q_target"], float) - np.asarray(seg["q_from"], float)
            vel = {}
            acc = {}
            for i, n in enumerate(seg["vel_deg_s"]):
                v = min(abs(float(seg["vel_deg_s"][n])), speed)
                vel[n] = max(0.2, v)
                acc[n] = max(0.2, min(abs(float((seg.get("acc_deg_s2") or {}).get(n, v / 0.5))),
                                      v / 0.5))
            seg["vel_deg_s"] = vel
            seg["acc_deg_s2"] = acc
            seg["T"] = round(max(0.5, max(
                abs(float(d[i])) / max(float(vel[n]), 0.05)
                for i, n in enumerate(seg["vel_deg_s"]))), 3)
        return segs

    def _run_leg(self, motion, seg, direction: int, label: str):
        self._dir = direction
        self._label = label
        self._sampling = True
        try:
            seg["read_full"] = True
            motion.execute([seg], preview=False, clear_stop=False)
        finally:
            self._sampling = False
            seg.pop("read_full", None)

    def _write_csv(self):
        if not self.samples:
            return
        runs = Path(getattr(self.executor, "runs_dir",
                            Path(__file__).resolve().parent.parent / "runs"))
        runs.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        p = runs / f"sweep_{self.joint}_{stamp}.csv"
        with p.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["time", "dir", "phase", "q1", "q2", "q3", "q4", "q5",
                        "q6", "current_a", "current_raw", "full_scale_a"])
            for r in self.samples:
                w.writerow([time.strftime("%Y-%m-%dT%H:%M:%S",
                                          time.localtime(r["t"])),
                            r["dir"], r["phase"], *r["q"],
                            r["current"],
                            "" if r["current_raw"] is None else r["current_raw"],
                            "" if r["full_scale_a"] is None else r["full_scale_a"]])
        self.path = str(p)
        self.status["path"] = str(p)

    def _run(self):
        ex = self.executor
        m = ex.motion
        st = self.status
        try:
            plan = plan_sweep(m, self.joint)
            st["warnings"] = list(plan.get("warnings") or [])
            st["checks"] = plan.get("checks")
            if not plan.get("ok"):
                st.update({"phase": "aborted", "reason": plan.get("reason") or "",
                           "active": False, "finished": time.time()})
                return
            st.update({"start": plan["start"], "end": plan["end"],
                       "phase": "move_start"})
            idx = SWEEP_JOBS[self.joint]["index"]
            q_start = plan["checks"]["start"]
            cur = m.read_q()
            pre = m._plan_move(cur, q_start, f"22.2 {self.joint} 到起点")
            m.execute(pre, preview=False, clear_stop=False)
            time.sleep(0.5)
            st["phase"] = "out"
            out = self._cap(plan["out"], self.speed)
            for seg in out:
                seg["label"] = f"22.2 {self.joint} 去程 {plan['start']:g}→{plan['end']:g}"
                self._run_leg(m, seg, +1, "out")
            time.sleep(1.0)
            st["phase"] = "back"
            back = self._cap(plan["back"], self.speed)
            for seg in back:
                seg["label"] = f"22.2 {self.joint} 回程 {plan['end']:g}→{plan['start']:g}"
                self._run_leg(m, seg, -1, "back")
            self._write_csv()
            st["phase"] = "fit"
            fit = fit_sweep(self.samples, self.joint, ex.robot,
                            ex.end_effector, float(ex.payload_kg or 0.0),
                            SWEEP_JOBS[self.joint].get("nominal_kt"))
            st["fit"] = fit
            if self.save and fit.get("ok"):
                data = dict(getattr(ex, "kt", {}) or {})
                rec = {k: v for k, v in fit.items() if k not in ("checks",)}
                data[self.joint] = rec
                data["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
                gravity.save_kt(data)
                ex.kt = gravity.load_kt()
                ex._sync_motion_params()
            st.update({"phase": "done", "active": False, "finished": time.time(),
                       "reason": "" if fit.get("ok") else "拟合判据未全部通过"})
        except Exception as exc:  # noqa: BLE001
            st.update({"phase": "aborted", "reason": str(exc), "active": False,
                       "finished": time.time()})

    def _run_guarded(self):
        m = getattr(self.executor, "motion", None)
        if m is not None:
            m.add_hook(self.feed)
        try:
            self._run()
        finally:
            if m is not None:
                m.remove_hook(self.feed)

    def start(self) -> threading.Thread:
        self.status.update({"active": True, "phase": "planning",
                            "started": time.time(), "finished": None,
                            "reason": "", "samples": 0, "fit": None})
        self._thread = threading.Thread(target=self._run_guarded, daemon=True,
                                        name=f"sweep-{self.joint}")
        self._thread.start()
        return self._thread

    def stop(self):
        try:
            self.executor.motion.stop(f"22.2 {self.joint} 扫描按停")
        except Exception:  # noqa: BLE001
            pass
