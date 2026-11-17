#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""今晚第 9 项：mock 长跑（随机目标 + 随机按停 + 故障注入）。

只连 mock，不碰 ECB / 192.168.1.30。默认跑 6 小时：
    cd arm-control && python3 tools/mock_soak.py --hours 6
快速验证：
    python3 tools/mock_soak.py --minutes 3

早上看 `runs/soak_<stamp>.json`（每 10 个动作刷新一次），字段：
    actions / max_stop_latency_ms / false_not_enabled /
    rss_start_mb / rss_end_mb / unhandled_exceptions /
    all_six_disabled_on_one_fault / abort_reasons
"""
from __future__ import annotations

import argparse
import json
import os
import random
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.executor import Executor
from arm.motion import MotionError
from arm.sca import R_ERROR, R_MODE, R_POSITION, R_POWER

NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
LIMITS = {"J1": 40.0, "J2": 45.0, "J3": 60.0, "J4": 60.0, "J5": 10.0, "J6": 10.0}


def rss_mb() -> float:
    try:
        out = subprocess.check_output(["ps", "-o", "rss=", "-p", str(os.getpid())],
                                      text=True, timeout=2)
        return round(float(out.strip()) / 1024.0, 1)
    except Exception:  # noqa: BLE001
        try:
            import resource
            v = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
            # macOS 的 ru_maxrss 是字节，Linux 是 KB
            if sys.platform == "darwin":
                v /= 1024.0 * 1024.0
            else:
                v /= 1024.0
            return round(v, 1)
        except Exception:  # noqa: BLE001
            return -1.0


class Soak:
    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.seed)
        self.stop = threading.Event()
        self.actions = 0
        self.completed = 0
        self.aborted = 0
        self.skipped = 0
        self.stop_cases = 0
        self.max_stop_latency = 0.0
        self.false_not_enabled = 0
        self.unhandled = 0
        self.all_six_disabled = 0
        self.abort_reasons: dict[str, int] = {}
        self.stop_t0 = None
        self._write_events = 0
        self.rss_start = rss_mb()
        self.rss_end = self.rss_start
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out_path = None
        self._timers: list[threading.Timer] = []
        self.e = Executor()
        self.e.connect("mock")
        self.e.bus.enable_all(True)
        self.e.speed_scale = 0.5
        self.e._sync_motion_params()
        self.m = self.e.motion
        self._install_bus_hooks()
        # 丢包 30%：位置/电源/模式/错误四条读回都按 30% 丢
        self.base_drop = {"position": 0.30, "power": 0.30, "mode": 0.30,
                          "error": 0.30}
        self.e.bus.set_faults(drop=self.base_drop)

    def _install_bus_hooks(self):
        b = self.e.bus
        orig_set, orig_hold = b.set_targets_q, b.hold_all

        def set_t(targets):
            if self.stop_t0 is not None:
                self.max_stop_latency = max(self.max_stop_latency,
                                            time.monotonic() - self.stop_t0)
            self._write_events += 1
            return orig_set(targets)

        def hold():
            if self.stop_t0 is not None:
                self.max_stop_latency = max(self.max_stop_latency,
                                            time.monotonic() - self.stop_t0)
            self._write_events += 1
            return orig_hold()

        b.set_targets_q, b.hold_all = set_t, hold

    def _summary(self) -> dict:
        return {
            "ok": True, "stamp": self.stamp,
            "duration_s": round(time.time() - self.t0, 1) if hasattr(self, "t0") else 0,
            "actions": self.actions, "completed": self.completed,
            "aborted": self.aborted, "skipped": self.skipped,
            "stop_cases": self.stop_cases,
            "max_stop_latency_ms": round(self.max_stop_latency * 1000.0, 1),
            "stop_latency_ok": self.max_stop_latency <= 0.100,
            "false_not_enabled": self.false_not_enabled,
            "unhandled_exceptions": self.unhandled,
            "all_six_disabled_on_one_fault": self.all_six_disabled,
            "rss_start_mb": self.rss_start, "rss_end_mb": self.rss_end,
            "abort_reasons": dict(sorted(self.abort_reasons.items(),
                                         key=lambda kv: -kv[1])[:20]),
            "faults": {"drop": self.base_drop,
                       "silent": "偶尔一颗 0.4–1.2 s",
                       "alarm": "偶尔一颗 0x0004 0.4–0.8 s",
                       "stop": "随机 0–1.5 s 按停"},
        }

    def _write_summary(self):
        if not self.out_path:
            self.out_path = (self.e.runs_dir /
                             f"soak_{self.stamp}.json")
        try:
            self.out_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.out_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(self._summary(), ensure_ascii=False,
                                      indent=2) + "\n", encoding="utf-8")
            tmp.replace(self.out_path)
        except Exception:  # noqa: BLE001
            pass

    def _clear_timers(self):
        for t in self._timers:
            t.cancel()
        self._timers = []

    def _random_target(self):
        q = [0.0] * 6
        kind = self.rng.random()
        if kind < 0.15:
            q = [-90.0, 30.0, 30.0, 90.0, 0.0, 0.0]
        elif kind < 0.30:
            q = [self.rng.uniform(-30, 30), self.rng.uniform(-20, 20),
                 self.rng.uniform(-20, 20), self.rng.uniform(-30, 30), 0, 0]
        else:
            for i, n in enumerate(NAMES):
                if n in ("J5", "J6"):
                    continue
                # 小步移动，动作短、次数多
                q[i] = self.rng.uniform(-LIMITS[n] * 0.5, LIMITS[n] * 0.5)
        return q

    def _one(self):
        e, m = self.e, self.m
        # 清掉上一轮的中止保持；只有 aborted 才需要
        if m.status.get("phase") in ("aborted", "error"):
            try:
                e.recover()
            except Exception:  # noqa: BLE001
                pass
        if not getattr(e.bus, "startup_done", False):
            try:
                e.bus.enable_all(True)
            except Exception:  # noqa: BLE001
                self.skipped += 1
                return
        self.e.bus.set_faults(drop=self.base_drop)
        q0 = [0.0] * 6
        for n in NAMES:
            q0[NAMES.index(n)] = float(e.bus.q.get(n, 0.0))
        target = self._random_target()
        try:
            segs = m._plan_move(q0, target, "soak 随机目标")
        except MotionError:
            self.skipped += 1
            return
        self.actions += 1
        try:
            m.begin_command()
        except Exception:  # noqa: BLE001
            self.skipped += 1
            return
        # 偶尔一颗不应答 / 偶尔报警
        fault_silent = None
        fault_alarm = None
        if self.rng.random() < 0.18:
            fault_silent = self.rng.choice(NAMES)
            e.bus.silent = {fault_silent}
            self._timers.append(threading.Timer(
                self.rng.uniform(0.4, 1.2), lambda: setattr(e.bus, "silent", set())))
            self._timers[-1].start()
        if self.rng.random() < 0.18:
            fault_alarm = self.rng.choice(NAMES)
            e.bus.error_bits = {fault_alarm: 0x0004}
            self._timers.append(threading.Timer(
                self.rng.uniform(0.4, 0.8),
                lambda: setattr(e.bus, "error_bits", {})))
            self._timers[-1].start()
        # 随机时刻按停
        stop_case = self.rng.random() < 0.30

        def do_stop():
            # 延迟从真正按下那一刻算，不是从排定时算
            self.stop_t0 = time.monotonic()
            e.stop_all("soak 随机按停")

        if stop_case:
            self.stop_cases += 1
            self._timers.append(threading.Timer(
                self.rng.uniform(0.03, 1.5), do_stop))
            self._timers[-1].start()
        try:
            m.execute(segs, preview=False, clear_stop=False)
            self.completed += 1
        except MotionError as exc:
            self.aborted += 1
            reason = str(exc)
            key = reason.split("；")[0][:60]
            self.abort_reasons[key] = self.abort_reasons.get(key, 0) + 1
            if "没有真正使能" in reason:
                # 电源/模式回读失败但总线实际仍是使能的，就是误判；
                # 我们故意点了一颗 silent 的不算误判。
                if e.bus.enabled and not fault_silent:
                    self.false_not_enabled += 1
            if not e.bus.enabled:
                # 只点了一颗的故障，却六颗全失能
                named = [n for n in NAMES if n in reason]
                if len(e.bus.silent) <= 1 and len(named) <= 2:
                    self.all_six_disabled += 1
        except Exception:  # noqa: BLE001
            self.unhandled += 1
            traceback.print_exc()
        finally:
            self._clear_timers()
            e.bus.silent = set()
            e.bus.error_bits = {}
            # 只统计到中止时 _abort 写的那一次保持目标；不要再补写，
            # 否则会把自检自己 80 ms 后的写目标算成“停的延迟”。
            self.stop_t0 = None
            if self.actions % 10 == 0:
                self.rss_end = rss_mb()
                self._write_summary()

    def run(self):
        self.t0 = time.time()
        deadline = self.t0 + float(self.args.hours) * 3600.0 \
            + float(self.args.minutes) * 60.0
        self._write_summary()
        print(f"mock soak start: {self.stamp} -> {self.out_path}")
        while not self.stop.is_set() and time.time() < deadline:
            if self.args.max_actions and self.actions >= self.args.max_actions:
                break
            try:
                self._one()
            except Exception:  # noqa: BLE001
                self.unhandled += 1
                traceback.print_exc()
                time.sleep(0.2)
            time.sleep(0.03)
        self.rss_end = rss_mb()
        self._write_summary()
        print(json.dumps(self._summary(), ensure_ascii=False, indent=2))
        try:
            self.e._poll_stop.set()
            self.e.disconnect()
        except Exception:  # noqa: BLE001
            pass


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=0.0)
    ap.add_argument("--minutes", type=float, default=0.0,
                    help="不写 --hours 时默认 360 分钟")
    ap.add_argument("--max-actions", type=int, default=0)
    ap.add_argument("--seed", type=int, default=20261116)
    a = ap.parse_args(argv)
    if a.hours <= 0 and a.minutes <= 0:
        a.minutes = 360.0
    Soak(a).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
