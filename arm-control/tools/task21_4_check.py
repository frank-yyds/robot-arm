#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""21.4 第 1–7 条 + 今晚补充里的三个必查项的 mock 自检。

运行：cd arm-control && python3 tools/task21_4_check.py

覆盖：
  1. 普通动作不再按 5° 拆段（1 段、模式 6、只写一次终点）
  2. 六轴同时到达：速度按位移比例、加速时间相同
  3. 170° 动作的路径检查点间隔 ≤0.5°
  4. 全局速度倍率默认 50%
  5. 跟踪保护：位移很小的关节被同步缩到很低速度不误判；一颗卡住 0.3 s 后中止
  6. 按停回拉（mock 量）
  7. 进度条按实际位置
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm.executor import Executor
from arm.fence import Fence
from arm.joints import load_joint_configs
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion, MotionError

FAILS: list[str] = []
ROBOT = RobotModel(cfgmod.URDF, cfgmod.tcp_presets())
JC = load_joint_configs()
LIM = ([JC[n].safe_limits_deg[0] for n in JC],
       [JC[n].safe_limits_deg[1] for n in JC])
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


class LogBus(MockJointBus):
    def __init__(self):
        super().__init__()
        self.log = []

    def read_all(self, full=False):
        d = super().read_all(full)
        self.log.append((time.monotonic(),
                         {n: d[n].out_deg for n in NAMES}))
        return d


class StuckBus(LogBus):
    def __init__(self, joint="J4", after=0.5):
        super().__init__()
        self.joint = joint
        self.after = after
        self.freeze_at = None
        self.frozen = None

    def read_all(self, full=False):
        d = super().read_all(full)
        now = time.monotonic()
        if self.freeze_at is not None and now >= self.freeze_at:
            if self.frozen is None:
                self.frozen = self.q[self.joint]
            self.q[self.joint] = self.frozen
            self.v[self.joint] = 0.0
            d[self.joint].out_deg = self.frozen
        return d


def mk_motion(bus, scale=0.5):
    m = Motion(bus, ROBOT, Fence(ROBOT), LIM)
    m.preset = "flange"
    m.speed_scale = scale
    return m


def run_thread(fn):
    box = []

    def w():
        try:
            fn()
        except BaseException as exc:  # noqa: BLE001
            box.append(exc)

    t = threading.Thread(target=w, daemon=True)
    t.start()
    return t, box


# --------------------------------------------------------------------------- #
# 1. 一段、模式 6、单目标
# --------------------------------------------------------------------------- #
bus = LogBus()
m = mk_motion(bus)
segs = m._plan_move([0] * 6, [-90, 30, 30, 90, 0, 0], "21.6 收拢")
check("21.4.1 普通动作只规划 1 段", len(segs) == 1, f"{len(segs)} 段，T={segs[0]['T']} s")
check("21.4.1 不再有 5° 拆段标签", all("1/" not in s["label"] for s in segs),
      str([s["label"] for s in segs]))
# 执行时只写一次终点；mock 目标写入次数
before = bus.writes
m.execute(segs, preview=False)
check("21.4.1 执行只写一次终点目标、全程模式 6",
      m.status.get("phase") == "done" and bus.writes - before <= 2
      and getattr(bus, "force_mode", {}).get("J1", 6) == 6,
      f"writes +{bus.writes - before}，phase={m.status.get('phase')}")

# --------------------------------------------------------------------------- #
# 2. 六轴同步：速度按位移、加速时间相同
# --------------------------------------------------------------------------- #
bus = LogBus()
m = mk_motion(bus)
segs = m._plan_move([0] * 6, [-90, 40, 20, 60, 1, 0.5], "同步")
seg = segs[-1]
d = np.abs(np.asarray(seg["q_target"]) - np.asarray(seg["q_from"]))
pairs = [(i, NAMES[i]) for i in range(6) if d[i] > 0.1]
v = [seg["vel_deg_s"][n] for _, n in pairs]
a = [seg["acc_deg_s2"][n] for _, n in pairs]
ratios = [v[k] / a[k] for k in range(len(v))]
vper = [v[k] / d[i] for k, (i, _) in enumerate(pairs)]
# 位移小到规划速度 <0.25°/s 的轴会被抬到 0.25°/s（避免 0 速度），
# 比例检查只对有实际位移且未被抬升的轴做。
scaled = [(pairs[k][1], vper[k]) for k in range(len(vper)) if v[k] > 0.2501]
check("21.4.2 各轴加速时间相同",
      max(ratios) - min(ratios) < 0.02,
      "v/a = " + ", ".join(f"{r:.3f}" for r in ratios) + " s")
check("21.4.2 速度按位移比例缩放（>0.25°/s 的轴）",
      len(scaled) >= 2 and
      (max(x[1] for x in scaled) - min(x[1] for x in scaled))
      / max(x[1] for x in scaled) < 0.02,
      "v/|d| = " + ", ".join(f"{n}:{r:.3f}" for n, r in scaled)
      + f"；低速抬升轴：{[(pairs[k][1], v[k]) for k in range(len(v)) if v[k] <= 0.2501]}")

# --------------------------------------------------------------------------- #
# 3. 170° 路径检查点 ≤0.5°
# --------------------------------------------------------------------------- #
bus = LogBus()
m = mk_motion(bus)
# 从 −85° 到 +85° 才是 170°
segs = m._plan_move([-85, 0, 0, 0, 0, 0], [85, 0, 0, 0, 0, 0], "170° 检查")
pc = segs[-1].get("path_check") or {}
check("21.4.3 170° 动作路径检查点间隔 ≤0.5°",
      pc.get("max_step_deg", 999) <= 0.5001 and pc.get("n_points", 0) >= 341,
      f"{pc.get('n_points')} 点，最大间隔 {pc.get('max_step_deg')}°")

# --------------------------------------------------------------------------- #
# 4. 速度倍率默认 50%
# --------------------------------------------------------------------------- #
e = Executor()
check("21.4.4 全局速度倍率默认 50%", abs(float(e.speed_scale) - 0.5) < 1e-9,
      f"{e.speed_scale}")

# --------------------------------------------------------------------------- #
# 5a. 位移很小的关节被同步缩到很低速度：跟踪保护不误判
# --------------------------------------------------------------------------- #
bus = LogBus()
m = mk_motion(bus, 0.5)
segs = m._plan_move([0] * 6, [20, 0, 0, 0, 0, 0.2], "小位移低速")
seg = segs[0]
v6 = seg["vel_deg_s"]["J6"]
t0 = time.time()
try:
    m.execute(segs, preview=False)
    ok = True
    err = str(m.status.get("reason") or "")
except MotionError as exc:
    ok = False
    err = str(exc)
arr = abs(bus.q["J6"] - 0.2)
check("21.4.5 位移最小关节低速（≤0.25°/s）不被误判",
      ok and arr < 0.8 and m.status.get("phase") == "done",
      f"J6 规划速度 {v6}°/s，到位误差 {arr:.3f}°，用时 {time.time() - t0:.1f}s，{err[:60]}")

# --------------------------------------------------------------------------- #
# 5b. 一颗卡住：超过阈值后 0.3 s 中止
# --------------------------------------------------------------------------- #
bus = StuckBus("J4", after=0.5)
m = mk_motion(bus, 0.5)
segs = m._plan_move([0] * 6, [0, 0, 0, 10, 0, 0], "J4 卡住")
m.cap_speeds(segs, 5.0)
seg = segs[0]
bus.freeze_at = time.monotonic() + 0.5
t0 = time.monotonic()
box = []
th, _ = None, None


def worker():
    try:
        m.execute(segs, preview=False)
        box.append(None)
    except BaseException as exc:  # noqa: BLE001
        box.append(exc)


th = threading.Thread(target=worker, daemon=True)
th.start()
th.join(timeout=15)
abort_t = time.monotonic()
reason = str(box[0]) if box and box[0] else ""
# 后处理：按同一梯形算应到位置，找第一个 lag>lim 的时刻
d = float(np.asarray(seg["q_target"])[3] - np.asarray(seg["q_from"])[3])
v = float(seg["vel_deg_s"]["J4"])
a = float(seg["acc_deg_s2"]["J4"])
lim = max(3.0, v * 0.5)
first = None
for t, qd in bus.log:
    if t < t0 + 0.3:      # 新判法不再整体跳 1 s，只看起步空程
        continue
    expected = np.asarray(seg["q_from"])[3] + m._trap_pos(d, v, a, t - t0)
    lag = expected - qd["J4"]
    if lag > lim:
        first = t
        break
dt_abort = (abort_t - first) if first else 999
# 26.2 新判法：不动（|u| < u_min 持续 0.5 s）算“卡住/不动”，不再是旧的
# “落后 3° 持续 0.3 s”。量从注入卡住（freeze_at）到中止的时间。
dt_from_freeze = (abort_t - bus.freeze_at) if bus.freeze_at else 999
# 新判法不要求“落后 >3°”，所以 first（旧阈值）不再作为判据；只看
# “注入卡住 → 判不动/卡住 → 中止”的时间。
check("21.4.5 一颗卡住：按 26.2 判“不动/卡住”，注入后 ≤0.8 s 中止",
      0.4 <= dt_from_freeze <= 0.8
      and "J4" in reason and ("卡住" in reason or "不动" in reason),
      f"注入卡住→中止 {dt_from_freeze * 1000:.0f} ms；{reason[:80]}")

# --------------------------------------------------------------------------- #
# 6. 按停回拉 + 7. 进度按实际
# --------------------------------------------------------------------------- #
bus = LogBus()
m = mk_motion(bus, 0.5)
segs = m._plan_move([0] * 6, [90, 0, 0, 0, 0, 0], "按停回拉")
th, box = run_thread(lambda: m.execute(segs, preview=False))
time.sleep(2.0)
p_before = float(m.status.get("progress") or 0.0)
q_stop = bus.q["J1"]
t_stop = time.monotonic()
m.stop("21.4 按停回拉")
th.join(timeout=5)
time.sleep(0.3)
excursion = max([abs(qd["J1"] - q_stop) for t, qd in bus.log
                  if t >= t_stop and qd["J1"] is not None] or [0.0])
check("21.4.6 按停回拉（mock）≤1°",
      excursion <= 1.0 and bus.enabled and m.status.get("phase") == "aborted",
      f"回拉 {excursion:.3f}°，phase={m.status.get('phase')}，六颗仍使能={bus.enabled}")
check("21.4.7 进度条按实际位置（2 s 时约 20%）",
      0.10 <= p_before <= 0.45,
      f"2 s 时 progress={p_before:.2f}（J1 {q_stop:.1f}°/90°）")

print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
raise SystemExit(1 if FAILS else 0)
