#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""22.3 碰撞/卡住检测（第一阶段：只记录，不动作）。

每次运动对 J2–J5 算电流残差：
    I_预测 = τ_重力(q) / Kt_eff + f · tanh(v / 1°/s) + c
    残差   = I_实测 − I_预测
阈值 = max(4 × 22.2 残差均方根, 下限)，下限初值 J2 0.5 A、J3 0.4 A、
J4/J5 0.5 A（标定后再定）。残差连续 3 次超阈值 → 记一次“标记”，
但第一阶段**不中止**，只写 runs/collision_*.csv，供一周统计。

性能要求（22.3）：残差计算不能影响运动循环、不能拖慢回读节拍。
所以 Motion 钩子只把这一拍的实际位置/指令速度丢进一个长度 4 的无锁队列，
真正的重力/残差计算和写盘都在后台线程；钩子入口整段 try/except。
"""
from __future__ import annotations

import csv
import math
import queue
import threading
import time
from pathlib import Path

from . import gravity

NOMINAL_KT = {"J2": 1.91, "J3": 1.35, "J4": 0.22, "J5": 0.22}
FLOOR_A = {"J2": 0.5, "J3": 0.4, "J4": 0.5, "J5": 0.5}
CHECKED = ("J2", "J3", "J4", "J5")


class CollisionMonitor:
    def __init__(self, executor, log_dir: Path | None = None,
                 enabled: bool = True):
        self.executor = executor
        self.enabled = bool(enabled)
        self.log_path = Path(log_dir or executor.runs_dir) / (
            "collision_" + time.strftime("%Y%m%d") + ".csv")
        self._queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True,
                                        name="collision-monitor")
        self._lock = threading.Lock()
        self._cur: dict | None = None
        self.flag_events: list[dict] = []
        self.summaries: list[dict] = []
        self.status = {"enabled": self.enabled, "phase": "idle", "label": "",
                       "samples": 0, "flags": 0, "last_summary": None,
                       "log": str(self.log_path), "error": ""}
        self._cache_current: dict[str, float | None] = {n: None for n in CHECKED}
        self._last_current_poll = 0.0

    # ---------------- Motion 钩子（必须极轻） ----------------
    def feed(self, states, cmd_q, cmd_vel, phase, label, elapsed, duration):
        if not self.enabled:
            return
        try:
            if phase == "end":
                self._queue.put_nowait(("end", str(label or "")))
                return
            if phase != "run" or cmd_vel is None:
                return
            if elapsed < 0.3:
                return
            if duration and elapsed > max(0.3, duration - 0.3):
                return
            if states is None:
                return
            # 重力模型要 6 个关节角；残差只对 J2–J5 算。
            q = []
            for n in self.executor.robot.joint_names:
                st = states.get(n)
                if st is None or st.out_deg is None:
                    return
                q.append(float(st.out_deg))
            cur = {}
            for n in CHECKED:
                st = states.get(n)
                cur[n] = None if st is None else st.current_a
            snap = {"q": q, "currents": cur,
                    "cmd_vel": [float(cmd_vel[i]) for i in range(4)],
                    "label": str(label or ""), "t": time.monotonic()}
            try:
                self._queue.put_nowait(("run", snap))
            except queue.Full:
                pass
        except Exception:  # noqa: BLE001
            # 22.3 硬要求：记录异常绝不能进运动循环
            pass

    # ---------------- 后台线程 ----------------
    def _params(self, name: str) -> dict:
        rec = (getattr(self.executor, "kt", {}) or {}).get(name) or {}
        kt = rec.get("kt_eff") or NOMINAL_KT.get(name)
        f = abs(float(rec.get("i_fric") or 0.0))
        c = float(rec.get("i_bias") or 0.0)
        rms = abs(float(rec.get("rms") or 0.0))
        thr = max(4.0 * rms, FLOOR_A.get(name, 0.5))
        return {"kt": kt, "f": f, "c": c, "rms": rms, "threshold": thr}

    def _refresh_currents(self):
        now = time.monotonic()
        if now - self._last_current_poll < 0.2:
            return
        self._last_current_poll = now
        bus = getattr(self.executor, "bus", None)
        if bus is None:
            return
        # 真机 read_all(full=False) 不带电流：按颗读 0x04（每 0.2 s 一次）
        motors = getattr(bus, "motors", None)
        if motors:
            for n in CHECKED:
                try:
                    self._cache_current[n] = motors[n].current_a(timeout=0.08,
                                                                 retries=0)
                except Exception:  # noqa: BLE001
                    self._cache_current[n] = None

    def _start_run(self, label: str):
        self._cur = {"label": label, "started": time.monotonic(),
                     "rows": {n: {"max": 0.0, "exceed": 0, "consec": 0,
                                  "flagged": False, "n": 0, "thr": 0.0}
                              for n in CHECKED}}

    def _finish_run(self):
        cur = self._cur
        self._cur = None
        if not cur:
            return
        rows = []
        for n in CHECKED:
            r = cur["rows"][n]
            if r["n"] <= 0:
                continue
            row = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "label": cur["label"], "joint": n,
                   "max_residual_a": round(r["max"], 4),
                   "threshold_a": round(r["thr"], 4),
                   "exceed_samples": int(r["exceed"]),
                   "flagged": int(bool(r["flagged"])),
                   "samples": int(r["n"])}
            rows.append(row)
            self.summaries.append(row)
        # 6 小时长跑不能无限存摘要/事件；只留最近一段给网页看
        if len(self.summaries) > 2000:
            self.summaries = self.summaries[-2000:]
        if len(self.flag_events) > 500:
            self.flag_events = self.flag_events[-500:]
        if rows:
            self._write_rows(rows)
            self.status["last_summary"] = rows
        self.status["phase"] = "idle"
        self.status["label"] = ""

    def _write_rows(self, rows: list[dict]):
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            new = not self.log_path.exists()
            with self.log_path.open("a", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=[
                    "time", "label", "joint", "max_residual_a", "threshold_a",
                    "exceed_samples", "flagged", "samples"])
                if new:
                    w.writeheader()
                for r in rows:
                    w.writerow(r)
        except Exception as exc:  # noqa: BLE001
            self.status["error"] = str(exc)

    def _one(self, snap: dict):
        if self._cur is None or self._cur["label"] != snap["label"]:
            self._start_run(snap["label"])
        if any(v is None for v in snap["currents"].values()):
            self._refresh_currents()
            snap = dict(snap)
            snap["currents"] = {n: (snap["currents"].get(n)
                                    if snap["currents"].get(n) is not None
                                    else self._cache_current.get(n))
                                for n in CHECKED}
        if any(v is None for v in snap["currents"].values()):
            return
        try:
            tau = gravity.torques(
                self.executor.robot, snap["q"],
                getattr(self.executor, "end_effector", "flange"),
                float(getattr(self.executor, "payload_kg", 0.0) or 0.0))
        except Exception as exc:  # noqa: BLE001
            self.status["error"] = str(exc)
            return
        self.status["samples"] = int(self.status["samples"]) + 1
        for k, name in enumerate(CHECKED):
            p = self._params(name)
            v = float(snap["cmd_vel"][k])
            pred = float(tau[k + 1]) / max(float(p["kt"]), 1e-6) \
                + p["f"] * math.tanh(v / 1.0) + p["c"]
            res = float(snap["currents"][name]) - pred
            thr = p["threshold"] + (p["f"] if abs(v) < 0.5 else 0.0)
            r = self._cur["rows"][name]
            r["n"] += 1
            r["thr"] = thr
            r["max"] = max(r["max"], abs(res))
            if abs(res) > thr:
                r["exceed"] += 1
                r["consec"] += 1
                if r["consec"] >= 3 and not r["flagged"]:
                    r["flagged"] = True
                    ev = {"t": time.time(), "label": snap["label"],
                          "joint": name, "residual_a": round(res, 4),
                          "threshold_a": round(thr, 4)}
                    self.flag_events.append(ev)
                    self.status["flags"] = int(self.status["flags"]) + 1
            else:
                r["consec"] = 0

    def _worker(self):
        while not self._stop.is_set():
            try:
                kind, payload = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if kind == "end":
                    self._finish_run()
                else:
                    self._one(payload)
            except Exception as exc:  # noqa: BLE001
                self.status["error"] = str(exc)

    def start(self):
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._worker, daemon=True,
                                            name="collision-monitor")
            self._thread.start()

    def close(self):
        self._stop.set()
        try:
            self._queue.put_nowait(("end", ""))
        except Exception:  # noqa: BLE001
            pass
        self._thread.join(timeout=1.0)
