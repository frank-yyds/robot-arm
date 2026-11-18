#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十一节 21.1 mock 回归自检（不连真机、不改 config）。

以后每次改运动循环都跑这个脚本：
    * `_run_segment`、`_run_chunks` 目标流、到位等待、停机流程四处，
      执行过程中随机时刻按停（`stop_all`，即 `/api/stop` 的同一入口），
      从发停到最后一次写运动目标 ≤100 ms；
    * ≥30° 长动作走到一半按停：不再有新目标、六颗保持使能、目标=实际位置；
    * 运动中停发心跳：0.5 s 后按中止原地保持。

运行：cd arm-control && python3 tools/task21_selfcheck.py
"""
from __future__ import annotations

import random
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.config import URDF, tcp_presets
from arm.executor import Executor
from arm.fence import Fence
from arm.joints import load_joint_configs
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion, MotionError

# 自检的停机完成会写 park_state.json；mock 的结果不能留到真机
cfgmod.save_park_state = lambda _data: None

STOP_LATENCY_S = 0.100
FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = ""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


class TimedBus(MockJointBus):
    """记录每一次目标写入（含 hold_all 写目标=当前位置）和时刻。"""

    def __init__(self, initial_deg=None):
        super().__init__(initial_deg)
        self.log: list[tuple[float, str, dict | None]] = []

    def set_targets_q(self, targets):
        self.log.append((time.monotonic(), "target", dict(targets)))
        return super().set_targets_q(targets)

    def hold_all(self):
        self.log.append((time.monotonic(), "hold", None))
        return super().hold_all()


class SlowArrivalBus(TimedBus):
    """profile 速度压到 1°/s：目标流走完（T=0.5 s）后还要等真正到位。"""

    def set_profiles_q(self, vel, acc=None):
        super().set_profiles_q(vel, acc)
        self.vmax["J2"] = 1.0
        self.amax["J2"] = 2.0


def mk_motion(bus):
    jc = load_joint_configs()
    lim = ([jc[n].safe_limits_deg[0] for n in bus.order],
           [jc[n].safe_limits_deg[1] for n in bus.order])
    robot = RobotModel(URDF, tcp_presets())
    m = Motion(bus, robot, Fence(robot), lim)
    m.preset = "flange"
    return m


def run_thread(fn):
    box: list[BaseException] = []

    def wrapped():
        try:
            fn()
        except BaseException as exc:  # noqa: BLE001
            box.append(exc)

    t = threading.Thread(target=wrapped, daemon=True)
    t.start()
    return t, box


def stop_latency(bus, t_stop):
    after = [x for x in bus.log if x[0] >= t_stop - 1e-9]
    if not after:
        return 0.0, []
    return max(x[0] for x in after) - t_stop, after


def all_held(bus):
    return all(abs(bus.target[n] - bus.q[n]) < 1e-6 for n in bus.order)


def motion_stop_case(bus, m, segs, kind: str, delay: float):
    """起一个线程跑 segs，delay 秒后按停，返回 (延迟 ms, 是否中止, 停前是否在动)。"""
    t_stop = None

    def worker():
        nonlocal t_stop
        if kind == "chunks":
            m._run_chunks(segs)
        else:
            m._run_segment(segs[0])

    th, box = run_thread(worker)
    time.sleep(delay)
    moving = m.status.get("phase") in ("moving", "running")
    t_stop = time.monotonic()
    m.stop("回归按停")
    th.join(timeout=10)
    lat, after = stop_latency(bus, t_stop)
    return lat, m.status.get("phase") == "aborted", moving, after


random.seed(21)

# ---------------- 21.1 `_run_chunks` 目标流中途按停（≥30°） ----------------
for trial in range(3):
    bus = TimedBus()
    m = mk_motion(bus)
    segs = m._plan_move([0] * 6, [0, 60, 0, 0, 0, 0], "长动作")
    delay = random.uniform(0.4, 1.4)
    lat, aborted, moving, after = motion_stop_case(bus, m, segs, "chunks", delay)
    targets_after = [x for x in after if x[1] == "target"]
    check(f"21.1 目标流按停 #{trial + 1} 停前在动", moving,
          f"delay {delay:.2f}s")
    check(f"21.1 目标流按停 #{trial + 1} 原地保持", aborted and bus.enabled
          and all_held(bus), m.status.get("reason", "")[:60])
    check(f"21.1 目标流按停 #{trial + 1} 写目标延迟 ≤100ms", lat <= STOP_LATENCY_S,
          f"{lat * 1000:.1f} ms")
    check(f"21.1 目标流按停 #{trial + 1} 不再发新目标", len(targets_after) <= 1,
          f"停后 target 写入 {len(targets_after)} 次")

# ---------------- 21.1 `_run_segment` 中途按停 ----------------
for trial in range(3):
    bus = TimedBus()
    m = mk_motion(bus)
    segs = m._plan_move([0] * 6, [0, 5, 0, 0, 0, 0], "单段")
    delay = random.uniform(0.05, 0.18)
    lat, aborted, moving, after = motion_stop_case(bus, m, segs, "segment", delay)
    check(f"21.1 单段按停 #{trial + 1} 原地保持", aborted and bus.enabled
          and all_held(bus), m.status.get("reason", "")[:60])
    check(f"21.1 单段按停 #{trial + 1} 写目标延迟 ≤100ms", lat <= STOP_LATENCY_S,
          f"{lat * 1000:.1f} ms")

# ---------------- 21.1 到位等待阶段按停 ----------------
for trial in range(3):
    bus = SlowArrivalBus()
    m = mk_motion(bus)
    segs = m._plan_move([0] * 6, [0, 3, 0, 0, 0, 0], "等到位")
    th, box = run_thread(lambda: m._run_chunks(segs))
    time.sleep(random.uniform(0.62, 0.85))
    waiting = abs(bus.q["J2"]) < 2.9 and m.status.get("phase") == "moving"
    t_stop = time.monotonic()
    m.stop("到位等待按停")
    th.join(timeout=10)
    lat, after = stop_latency(bus, t_stop)
    check(f"21.1 到位等待按停 #{trial + 1} 确实还在等到位", waiting,
          f"J2 {bus.q['J2']:.2f}°")
    check(f"21.1 到位等待按停 #{trial + 1} 原地保持", m.status.get("phase") == "aborted"
          and bus.enabled and all_held(bus), m.status.get("reason", "")[:60])
    check(f"21.1 到位等待按停 #{trial + 1} 写目标延迟 ≤100ms", lat <= STOP_LATENCY_S,
          f"{lat * 1000:.1f} ms")


# ---------------- 21.1 停机流程中途按停 ----------------
def mk_exec() -> Executor:
    e = Executor()
    e.connect("mock")
    return e


def patch_bus_log(e: Executor):
    log: list[tuple[float, str, dict | None]] = []
    b = e.bus
    orig_set, orig_hold = b.set_targets_q, b.hold_all

    def set_t(targets):
        log.append((time.monotonic(), "target", dict(targets)))
        return orig_set(targets)

    def hold():
        log.append((time.monotonic(), "hold", None))
        return orig_hold()

    b.set_targets_q, b.hold_all = set_t, hold
    return log


def park_prereq():
    return {"gripper_empty": True, "workspace_clear": True, "shelf_ready": True}


def close_exec(e: Executor):
    e._poll_stop.set()
    e._watch_stop.set()
    try:
        e.disconnect()
    except Exception:  # noqa: BLE001
        pass


# (a) 停机刚开始、还在往预停机点走的阶段按停
e = mk_exec()
log = patch_bus_log(e)
e.park(False, prereq=park_prereq())
time.sleep(random.uniform(0.4, 1.2))
t_stop = time.monotonic()
e.stop_all("停机流程按停")
t0 = time.time()
while e.park_active and time.time() - t0 < 30:
    time.sleep(0.02)
after = [x for x in log if x[0] >= t_stop - 1e-9]
lat = (max(x[0] for x in after) - t_stop) if after else 0.0
check("21.1 停机流程按停（运动段）停住不失能",
      e.park_step.get("phase") == "aborted" and e.bus.enabled,
      e.msg[:70])
check("21.1 停机流程按停（运动段）写目标延迟 ≤100ms", lat <= STOP_LATENCY_S,
      f"{lat * 1000:.1f} ms")
close_exec(e)

# (b) 停机分步下降第 2 步之后按停
e = mk_exec()
log = patch_bus_log(e)
e.bus.set_faults(currents={"J2": 2.0, "J3": 1.0})
e.park(False, prereq=park_prereq())
t0 = time.time()
while time.time() - t0 < 60:
    if (e.park_step.get("phase") == "descending"
            and (e.park_step.get("step") or 0) >= 2):
        break
    time.sleep(0.02)
reached = e.park_step.get("phase") == "descending"
t_stop = time.monotonic()
e.stop_all("停机下降按停")
t0 = time.time()
while e.park_active and time.time() - t0 < 15:
    time.sleep(0.02)
after = [x for x in log if x[0] >= t_stop - 1e-9]
lat = (max(x[0] for x in after) - t_stop) if after else 0.0
check("21.1 停机下降按停 停在第 2 步后不再下降", reached and
      e.park_step.get("phase") == "aborted" and e.bus.enabled, e.msg[:70])
check("21.1 停机下降按停 写目标延迟 ≤100ms", lat <= STOP_LATENCY_S,
      f"{lat * 1000:.1f} ms")
close_exec(e)

# ---------------- 21.1 运动中停发心跳：0.5 s 后原地保持 ----------------
e = mk_exec()
e.start_watchdog()
e.acquire("hb", e.config_version)
m = e.motion
segs = m._plan_move([0] * 6, [0, 60, 0, 0, 0, 0], "心跳长动作")
m.begin_command()
th, box = run_thread(lambda: m.execute(segs, preview=False, clear_stop=False))
time.sleep(0.3)
t0 = time.time()
while time.time() - t0 < 3 and m.status.get("phase") != "aborted":
    time.sleep(0.02)
elapsed = time.time() - t0
check("21.1 停发心跳 0.5 s 后中止保持",
      m.status.get("phase") == "aborted" and e.bus.enabled
      and "心跳" in (m.status.get("reason") or ""),
      f"{elapsed:.2f}s，{m.status.get('reason', '')[:60]}")
th.join(timeout=2)
e._watch_stop.set()
close_exec(e)

# ---------------- 21.4 跟踪保护：d≈0 的关节被扰动也要抓 ----------------
class J3Drift(MockJointBus):
    def __init__(self):
        super().__init__()
        self.t0 = None

    def read_all(self, full=False):
        d = super().read_all(full)
        if self.t0 is None:
            self.t0 = time.monotonic()
        if time.monotonic() - self.t0 > 0.5:
            d["J3"].out_deg = -20.0  # 目标≈0，实际被带走；26.2 兜底阈值 15°
        return d


bus = J3Drift()
m = mk_motion(bus)
segs = m._plan_move([0] * 6, [0, 20, 0, 0, 0, 0], "J3 扰动")
t0 = time.time()
try:
    m.execute(segs, preview=False)
    check("21.4 d≈0 关节被带走应中止", False)
except MotionError as exc:
    check("21.4 d≈0 关节被带走应中止",
          "J3" in str(exc) and "应到位置" in str(exc) and time.time() - t0 < 2.0,
          str(exc)[:90])

# ---------------- 21.3 试验工具：单目标 + 10°/s ----------------
bus = TimedBus()
m = mk_motion(bus)
segs = m._plan_move([0] * 6, [0, 40, 0, 0, 0, 0], "单目标")
check("21.4 普通规划不再按 5° 拆段", len(segs) == 1, f"{len(segs)} 段")
single = m._plan_move([0] * 6, [0, 40, 0, 0, 0, 0], "单目标",
                      max_step_deg=180.0)
m.cap_speeds(single, 10.0)
check("21.4 single_target 仍是 1 段", len(single) == 1, f"{len(single)} 段")
check("21.3 速度压到 10°/s", max(single[0]["vel_deg_s"].values()) <= 10.0
      and abs(single[0]["T"] - 4.0) < 0.2,
      f"T={single[0]['T']}，vel={single[0]['vel_deg_s']}")

print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
raise SystemExit(1 if FAILS else 0)
