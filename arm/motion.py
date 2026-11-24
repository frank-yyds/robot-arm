#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动层：同步点到点、路径、预演、中止、记录。

要求（16.4）：
  * 同步点到点：T = max |Δ|/v，≥0.5 s；各关节按 |Δ|/T 配梯形速度；
  * 路径：相邻点插 10 个中间点，每点 FK + 围栏，任何一点不过就整条拒绝；
  * 预演只算不发；
  * 中止 = 保持（目标改成当前位置），不是失能；
  * 全部失能只在“已回零并扶住”或“通信完全断了”时做；
  * 18.2：每条路径的起点、终点、10 个插补点都过重力力矩预检；
    被拒绝时自动重试一次（先把 q4–q6 走到目标，再走 q1–q3）；
  * 18.3：停机路径（抬高 → 收拢 → 预停机 → P）用同一套检查。
"""
from __future__ import annotations

import math
import re
import threading
import time

import numpy as np

from . import gravity
from .fence import Fence
from .joints import JointState
from .kinematics import RobotModel
from .recorder import Recorder
from .sca import MODE_POSITION, MODE_PROFILE_POSITION


class MotionError(RuntimeError):
    pass


# 启动容忍：每段开始，若某颗起始误差 >5°，允许 1 s 内最多 3° 的反向空程；
# 一旦朝目标动，仍用原来的 >5° 且 0.3 s 不降 1° 判据。超过 3° 立即中止。
START_GRACE_S = 1.0
# 27.2 第 2 条：前臂俯仰检查（实测 q2+q3 比应到大 2° 以上，持续 0.2 s 就中止）
PITCH_TOL_DEG = 2.0
PITCH_HOLD_S = 0.2
# 2026-11-15：J2 在停机起步实测反向 3.2°（重力/背隙），3° 太紧；
# 放宽到 5°，仍远小于“换分支/走反”的量级。
START_REVERSE_DEG = 5.0

# 驱动 0x0200“位置模式阶跃过大”保护：一次下发的目标不能离当前太远。
# 5° 与已实测可用的 J3/J2 ±5° 点动一致；大动作自动拆成多段。
MAX_TARGET_STEP_DEG = 5.0

# 抬高（回零/停机共用）：目标 TCP z=150。
# 随机姿态的工具轴不一定朝下；强行把工具轴转成 (0,0,-1) 会让腕部大角度重排，
# 甚至在同一 XY 上无解（2026-11-15 的 10000 点扫描：1850 个失败里 1162 个是
# “残余”，450/734 个合法起点的工具轴是水平的）。所以抬高保留当前工具轴。
# 同一 XY 抬到 150 超出可达范围时，先向底座方向收一点（最多 10%），
# 再允许工具轴朝 -z 倾斜；最后才降低抬高高度。
LIFT_Z_MM = 150.0
LIFT_GOOD_JUMP_DEG = 45.0
LIFT_MAX_JUMP_DEG = 75.0
LIFT_SCALES = (1.0, 0.99, 0.98, 0.96, 0.94, 0.92, 0.90, 0.88, 0.85, 0.80)
LIFT_TILTS = (15.0, 30.0, 45.0, 60.0, 75.0, 90.0)
LIFT_LOWER_Z_MM = (148.0, 145.0, 140.0, 135.0, 130.0)


class TorqueError(MotionError):
    """18.2 重力力矩预检不通过（可以尝试“远端先收”重试）。"""


class StartFenceError(MotionError):
    """16.4 第 11 条：起点在围栏外；只允许零扭矩拖动或先抬高再回零。"""


def _axis_toward(axis, target, deg: float) -> np.ndarray:
    """把单位轴 axis 朝 target 转 deg（球面插值），用于抬高的工具轴候选。"""
    a = np.asarray(axis, dtype=float)
    b = np.asarray(target, dtype=float)
    a = a / max(float(np.linalg.norm(a)), 1e-12)
    b = b / max(float(np.linalg.norm(b)), 1e-12)
    cos = max(-1.0, min(1.0, float(np.dot(a, b))))
    ang = math.acos(cos)
    if ang < 1e-9:
        return a.copy()
    if ang > math.pi - 1e-9:
        return b.copy()
    t = min(1.0, math.radians(float(deg)) / ang)
    s = math.sin(ang)
    return (math.sin((1.0 - t) * ang) * a
            + math.sin(t * ang) * b) / s


class LiftBranchError(MotionError):
    """抬高 IK 解到了另一个分支；拒绝执行，避免整臂甩过去。"""


class Motion:
    def __init__(self, bus, robot: RobotModel, fence: Fence,
                 limits: tuple[list[float], list[float]],
                 recorder: Recorder | None = None,
                 poll_hz: float = 25.0):
        self.bus = bus
        self.robot = robot
        self.fence = fence
        self.limits = (np.asarray(limits[0], float), np.asarray(limits[1], float))
        self.recorder = recorder
        self.poll_hz = max(10.0, float(poll_hz))
        # 21.4：全局速度倍率；第一次上真机默认 50%，网页可调
        self.speed_scale = 1.0
        # 22.2 / 22.3：回读后钩子（扫描采样、碰撞残差记录）。钩子异常一律吞掉，
        # 不允许影响运动循环；真正的重活由钩子自己丢到后台线程。
        self.hooks: list = []
        self.brake_comp_enabled = False
        self.order = list(bus.order)
        self.preset = "probe"
        self.yaw_deg = 0.0
        # 18.2：末端配置 / 载荷 / 回读限流 / Kt 标定结果，由执行器刷新
        self.end_effector = "flange"
        self.payload_kg = 0.0
        self.limit_a: dict[str, float] = {}
        self.kt: dict = {}
        self.j2_bracket = "PLA"          # P08-9：PLA 8 N·m / PETG100 12 N·m
        self._stop = threading.Event()
        self._stop_reason = "用户按停"
        self._lock = threading.RLock()
        self.status = {
            "busy": False, "phase": "idle", "reason": "", "label": "",
            "progress": 0.0, "current_q": None, "target_q": None,
            "segments": [], "started": None, "finished": None,
        }

    # ---------------- 基础 ----------------
    def read_q(self) -> list[float]:
        states = self.bus.read_all(full=False)
        q = []
        for name in self.order:
            v = states[name].out_deg
            if v is None:
                raise MotionError(f"{name} 无位置读数（失能或掉线）")
            q.append(float(v))
        return q

    def read_states(self):
        return self.bus.read_all(full=False)

    def add_hook(self, fn):
        """注册回读钩子：fn(states, cmd_q, cmd_vel, phase, label, elapsed, duration)。

        phase = "run" / "end"。钩子必须自己保证不阻塞（22.3 用后台线程）。
        """
        if fn not in self.hooks:
            self.hooks.append(fn)

    def remove_hook(self, fn):
        try:
            self.hooks.remove(fn)
        except ValueError:
            pass

    def _call_hooks(self, states, cmd_q, cmd_vel, phase: str, label: str = "",
                    elapsed: float = 0.0, duration: float = 0.0):
        for fn in list(self.hooks):
            try:
                fn(states, cmd_q, cmd_vel, phase, label, elapsed, duration)
            except Exception:  # noqa: BLE001
                # 22.3 的硬要求：任何残差/记录异常都不能影响运动循环
                pass

    def q_dict(self, q) -> dict[str, float]:
        return {n: float(v) for n, v in zip(self.order, q)}

    def _near_limit(self, q) -> list[dict]:
        out = []
        for i, n in enumerate(self.order):
            m = min(q[i] - self.limits[0][i], self.limits[1][i] - q[i])
            if m < 10.0:
                out.append({"joint": n, "margin_deg": round(float(m), 2)})
        return out

    def check_torque(self, q) -> list[dict]:
        """18.2 预检：单个姿态，返回超限项。"""
        return gravity.check_q(self.robot, q, self.end_effector,
                               self.payload_kg, self.limit_a, self.kt,
                               j2_bracket=self.j2_bracket)

    def capacities(self) -> dict:
        return gravity.capacities(self.limit_a, self.kt, self.j2_bracket)

    def start_problem(self, q) -> str | None:
        """16.4 第 11 条：把当前实际姿态也过一次围栏。"""
        fr = self.fence.check(q, self.preset, self.yaw_deg, False, self.limits)
        return None if fr.ok else "；".join(fr.reasons)

    def _check_q(self, q, descend: bool = False,
                 relax_below_z: float | None = None) -> dict:
        for i, n in enumerate(self.order):
            if not (self.limits[0][i] - 1e-9 <= q[i] <= self.limits[1][i] + 1e-9):
                raise MotionError(
                    f"{n} 目标 {q[i]:.2f}° 超出安全限位 "
                    f"{self.limits[0][i]:.0f}..{self.limits[1][i]:.0f}°")
        fr = self.fence.check(q, self.preset, self.yaw_deg, descend, self.limits,
                              relax_below_z)
        if not fr.ok:
            raise MotionError("围栏不通过：" + "；".join(fr.reasons))
        bad = self.check_torque(q)
        if bad:
            b = bad[0]
            raise TorqueError(
                f"力矩预检不通过：{b['joint']} {b['tau']:+.2f} N·m "
                f"> 0.8×{b['capacity']:.2f}（{b['ratio'] * 100:.0f}%）"
                f"，姿态 {tuple(round(float(v), 1) for v in q)}"
                + ("；末端 " + self.end_effector
                   + (f" + {self.payload_kg:g} kg" if self.payload_kg else "")))
        return fr.as_dict()

    def _interp(self, q0, q1, n=10):
        return [np.asarray(q0, float) * (1 - t) + np.asarray(q1, float) * t
                for t in np.linspace(0, 1, n + 1)]

    def _plan_joint(self, q_from, q_target, label: str, descend: bool = False,
                    relax_below_z: float | None = None) -> dict:
        q0 = np.asarray(q_from, float)
        q1 = np.asarray(q_target, float)
        # 目标先查
        fr = self._check_q(q1, descend, relax_below_z)
        # 21.4：取消 5° 拆段后，围栏/限位/力矩的中间点密度不能降：
        # 按位移最大的关节 ≤0.5° 插点（原来固定 10 点，大动作只到几度一个点）。
        d = q1 - q0
        maxd = float(np.max(np.abs(d))) if np.max(np.abs(d)) > 0 else 0.0
        n_int = max(1, int(math.ceil(maxd / 0.5)))
        pts = self._interp(q0, q1, n_int)
        for p in pts[1:]:
            self._check_q(p, descend, relax_below_z)
        steps = [float(np.max(np.abs(pts[k + 1] - pts[k])))
                 for k in range(len(pts) - 1)]
        max_step = max(steps) if steps else 0.0
        scale = max(0.05, min(1.0, float(getattr(self, "speed_scale", 1.0))))
        vmax = np.array([self.bus.cfg[n].motion_speed_deg_s for n in self.order]) * scale
        T = max(0.5, float(np.max(np.abs(d) / vmax))) if np.max(np.abs(d)) > 0 else 0.5
        vel = {}
        acc = {}
        for i, n in enumerate(self.order):
            v = abs(float(d[i])) / T
            v = max(0.25, min(vmax[i], v))
            vel[n] = v
            acc[n] = v / max(self.bus.cfg[n].motion_accel_time_s, 0.05)
        return {
            "label": label,
            "q_from": [round(float(v), 4) for v in q0],
            "q_target": [round(float(v), 4) for v in q1],
            "descend": bool(descend),
            "T": round(T, 3),
            "vel_deg_s": {k: round(v, 3) for k, v in vel.items()},
            "acc_deg_s2": {k: round(v, 3) for k, v in acc.items()},
            "fence": fr,
            "near_limit": self._near_limit(q1),
            # 21.4.3：围栏/限位/力矩中间点密度的可核对记录（按位移最大关节
            # ≤0.5° 插点，170° 的动作应有 ≥340 个间隔）。
            "path_check": {
                "n_points": len(pts),
                "max_step_deg": round(float(max_step), 4),
                "target_max_err_deg": round(float(maxd), 4),
            },
        }

    # ---------------- 规划 ----------------
    def _plan_move(self, q0, q1, label: str, descend: bool = False,
                   allow_outside_start: bool = False,
                   relax_below_z: float | None = None,
                   max_step_deg: float | None = None) -> list[dict]:
        """18.2 第 2 条：被力矩预检拒绝的路径自动重试一次（远端先收）。

        中间点 = q1 的 q4–q6 + q0 的 q1–q3：先把腕走到目标，再走大臂。
        16.4 第 11 条：起点也要过围栏；只有“抬高”段 allow_outside_start=True。
        """
        if not allow_outside_start:
            problem = self.start_problem(q0)
            if problem:
                raise StartFenceError(
                    "起点在围栏外（" + problem + "）：只允许零扭矩拖动，"
                    "或先“竖直抬高”再回零；普通目标一律拒绝")
        # 21.4 主方案：不再按 5° 拆段。21.3 已实测模式 6 下单目标 8/12/20/40°
        # 都不报 0x0200；普通动作一段、一次写终点，由驱动自己规划 S 曲线。
        # max_step_deg 参数保留给旧的试验工具，不再默认拆段。
        if max_step_deg is not None and float(max_step_deg) > 0:
            q0a = np.asarray(q0, float)
            q1a = np.asarray(q1, float)
            maxd = float(np.max(np.abs(q1a - q0a)))
            step = float(max_step_deg)
            if maxd > step + 1e-9:
                n = int(math.ceil(maxd / step))
                out: list[dict] = []
                prev = q0a.copy()
                for k in range(1, n + 1):
                    qmid = q0a + (q1a - q0a) * (k / n)
                    out.extend(self._plan_move(
                        prev, qmid, f"{label} {k}/{n}", descend,
                        allow_outside_start=(allow_outside_start or k > 1),
                        relax_below_z=relax_below_z, max_step_deg=step))
                    prev = qmid
                return out
        try:
            return [self._plan_joint(q0, q1, label, descend, relax_below_z)]
        except TorqueError as exc:
            # P08-10：先“远端先收”，还不过再反过来“伸臂后伸腕”
            last = exc
            for first in ("wrist", "arm"):
                try:
                    return self._split_move(q0, q1, label, descend,
                                            relax_below_z, first, str(last))
                except TorqueError as e2:
                    last = e2
            raise last

    def cap_speeds(self, segments: list[dict], speed_deg_s: float) -> list[dict]:
        """把已规划段的速度上限压到 speed_deg_s（21.3 真机试验用 10°/s）。

        只改 vel/acc/T，不改目标、围栏和力矩检查；网页正常点动不调用。
        """
        cap = abs(float(speed_deg_s))
        if cap <= 0:
            return segments
        for seg in segments:
            d = np.asarray(seg["q_target"], float) - np.asarray(seg["q_from"], float)
            vel = dict(seg.get("vel_deg_s") or {})
            acc = dict(seg.get("acc_deg_s2") or {})
            for i, n in enumerate(self.order):
                v = min(float(vel.get(n, cap)), cap)
                vel[n] = v
                acc[n] = min(float(acc.get(n, v / 0.5)),
                             v / max(self.bus.cfg[n].motion_accel_time_s, 0.05))
            seg["vel_deg_s"] = {k: round(float(v), 3) for k, v in vel.items()}
            seg["acc_deg_s2"] = {k: round(float(v), 3) for k, v in acc.items()}
            seg["T"] = round(max(0.5, max(
                abs(float(d[i])) / max(float(vel[n]), 0.1)
                for i, n in enumerate(self.order))), 3)
        return segments

    def _split_move(self, q0, q1, label, descend, relax_below_z,
                    first: str, note: str) -> list[dict]:
        q1 = list(q1)
        if first == "wrist":
            mid = list(q0)
            mid[3:] = q1[3:]
            tags = ("（远端先收 q4–q6）", "（再走 q1–q3）")
        else:
            mid = list(q1)
            mid[3:] = list(q0)[3:]
            tags = ("（先走 q1–q3）", "（再走 q4–q6）")
        if max(abs(mid[i] - q0[i]) for i in range(6)) < 0.5:
            raise TorqueError(note)
        segs = [self._plan_joint(q0, mid, label + tags[0], descend, relax_below_z),
                self._plan_joint(mid, q1, label + tags[1], descend, relax_below_z)]
        for s in segs:
            s["torque_retry"] = True
            s["torque_retry_order"] = first
            s["torque_note"] = note
        return segs

    def plan_joint_move(self, q_target, q_from=None, descend: bool = False,
                        label: str = "move_joints") -> list[dict]:
        q0 = self.read_q() if q_from is None else list(q_from)
        return self._plan_move(q0, q_target, label, descend)

    def plan_tcp_move(self, target_mm, tool_axis=(0, 0, -1), yaw_deg=None,
                      q_from=None, preset: str | None = None,
                      descend: bool = False, label: str = "move_tcp",
                      free_yaw: bool = False, fast: bool = False,
                      allow_outside_start: bool = False,
                      relax_below_z: float | None = None,
                      pos_tol_mm: float = 0.2) -> list[dict]:
        old_preset, old_yaw = self.preset, self.yaw_deg
        if preset:
            self.preset = preset
        if yaw_deg is not None:
            self.yaw_deg = yaw_deg
        try:
            q0 = self.read_q() if q_from is None else list(q_from)
            use_yaw = None if free_yaw else self.yaw_deg
            # 22.1：解析解当种子，按“相对当前姿态变化最大者最小”排序；
            # 逐个过限位/围栏/力矩，第一个通过的就用它规划。
            sols = self.robot.ik_all(
                target_mm, tool_axis, use_yaw, seed_deg=q0, preset=self.preset,
                limits=(self.limits[0], self.limits[1]),
                pos_tol_mm=pos_tol_mm,
                time_budget_s=(1.5 if fast else 4.0))
            failures: list[str] = []
            for sol in sols:
                try:
                    segs = self._plan_move(q0, sol["q_deg"], label, descend,
                                           allow_outside_start, relax_below_z)
                except MotionError as exc:
                    failures.append(str(exc))
                    continue
                segs[-1]["ik"] = {
                    **sol, "branch": sol.get("branch"),
                    "selected_by": "解析解：变化最大关节最小",
                }
                return segs
            if not sols:
                kw = {"max_iters": 60, "time_budget_s": 0.6} if fast else {}
                res = self.robot.ik(target_mm, tool_axis, use_yaw, q0,
                                    self.preset,
                                    (self.limits[0], self.limits[1]),
                                    pos_tol_mm=pos_tol_mm, **kw)
                if not res.ok:
                    tx, ty, tz = (round(float(v), 1) for v in target_mm)
                    raise MotionError(
                        f"IK 无解（目标 {tx},{ty},{tz}；偏航{'自由' if free_yaw else '锁 0°'}）："
                        f"残差 {res.pos_err_mm:.2f} mm / 轴 {res.axis_err_deg:.2f}°；"
                        "往可达区拖，或勾“偏航自由”再试")
                segs = self._plan_move(q0, res.q_deg, label, descend,
                                       allow_outside_start, relax_below_z)
                segs[-1]["ik"] = res.as_dict()
                return segs
            tx, ty, tz = (round(float(v), 1) for v in target_mm)
            raise MotionError(
                f"IK 解析解 {len(sols)} 组都过不了围栏/限位/力矩"
                f"（目标 {tx},{ty},{tz}）：" + "；".join(failures[:3]))
        finally:
            self.preset, self.yaw_deg = old_preset, old_yaw

    def plan_click(self, x: float, y: float, z_target: float = 20.0,
                   yaw_deg: float = 0.0, q_from=None,
                   fast: bool = False) -> list[dict]:
        """16.5 的三段路径：抬高 → 平移到目标上方 z=100 → 下探到 z=20。"""
        q0 = self.read_q() if q_from is None else list(q_from)
        old_yaw = self.yaw_deg
        self.yaw_deg = yaw_deg
        try:
            tcp0 = self.robot.tcp(q0, self.preset, yaw_deg)[:3, 3]
            segs: list[dict] = []
            cur = q0
            if tcp0[2] < 100.0 - 0.5:
                lift = self.plan_tcp_move(
                    [tcp0[0], tcp0[1], 100.0], (0, 0, -1), yaw_deg,
                    cur, label="抬到 z=100", fast=fast)
                segs.extend(lift)
                cur = lift[-1]["q_target"]
            mid = self.plan_tcp_move(
                [x, y, 100.0], (0, 0, -1), yaw_deg, cur,
                label="平移到目标上方", fast=fast)
            segs.extend(mid)
            fin = self.plan_tcp_move(
                [x, y, z_target], (0, 0, -1), yaw_deg, mid[-1]["q_target"],
                descend=True, label="下探", fast=fast)
            segs.extend(fin)
            return segs
        finally:
            self.yaw_deg = old_yaw

    def plan_home(self, q_from=None, allow_direct_fallback: bool = False) -> list[dict]:
        """16.5 第 4 条：抬高 → 一步同步插补回 q=0。

        2026-11-09（P05/18.2）去掉“先收 J5 到 0”和“收臂到 (0,30,30)”两段：
        J5 转到 0 对 J4 没有卸载作用（J4 的 0.57 N·m 不变），反而给 J5 加载。
        大关节的过载由 18.2 预检 + 远端先收重试处理。
        """
        q0 = self.read_q() if q_from is None else list(q_from)
        try:
            segs: list[dict] = self._plan_lift(q0)
        except LiftBranchError:
            if not allow_direct_fallback:
                raise
            # 离线可达性/人工换姿态用：不走竖直抬高，直接关节空间回零。
            # 真机默认不打开；它会少掉“先离开搁架”的保护。
            segs = self._plan_move(
                q0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "回零位（无抬高兜底）")
            return segs
        cur = segs[-1]["q_target"] if segs else list(q0)
        segs.extend(self._plan_move(cur, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "回零位"))
        return segs

    def plan_through(self, waypoints, descend_flags=None, labels=None) -> list[dict]:
        """路径：相邻点之间再插 10 个中间点，全查 FK + 围栏 + 力矩，任一点不过整条拒绝。"""
        q0 = self.read_q()
        pts = [list(q0)] + [list(w) for w in waypoints]
        flags = descend_flags or [False] * (len(pts) - 1)
        segs: list[dict] = []
        for i in range(1, len(pts)):
            segs.extend(self._plan_move(
                pts[i - 1], pts[i],
                (labels[i - 1] if labels else f"path {i}"),
                bool(flags[i - 1])))
        return segs

    # ---------------- 18.3 停机 ----------------
    def default_park(self) -> dict:
        side = getattr(self, "park_side", -1)
        q1 = 90.0 * side          # park_side=-1 → 臂沿 −y，J1 = −90°；+1 → +90°
        return {
            "park_side": side,
            "poses": {
                "tuck": [q1, 30.0, 30.0, 90.0, 0.0, 0.0],
                "pre": [q1, 76.5, 13.5, 90.0, 0.0, 0.0],
                "park": [q1, 85.0, 5.0, 90.0, 0.0, 0.0],
            },
        }

    def _plan_lift(self, q0) -> list[dict]:
        """抬高到 TCP z ≈ 150（回零和停机共用）。

        2026-11-15：保留当前工具轴，不再强行把工具轴转成 (0,0,-1)。
        随机姿态的工具轴可能水平/朝上；强行朝下会让腕部大幅重排，
        在同一 XY 上经常无解（10000 点扫描里 1162 个“残余”失败）。
        目标按以下顺序枚举，取代价最小的连续解：
          1. 同一 XY、当前工具轴，竖直抬到 150；
          2. 向底座方向收 1%..10%（同一 XY 超出可达范围时）；
          3. 工具轴朝 -z 倾斜 15°..90°；
          4. 最后降低抬高高度 148..130 mm。
        单关节跳 >75° 视为换分支，拒绝；45° 以内的正常抬升直接采用。
        """
        t0 = self.robot.tcp(q0, self.preset, self.yaw_deg)
        p0 = t0[:3, 3]
        if p0[2] >= LIFT_Z_MM - 0.5:
            return []
        axis0 = t0[:3, 2].copy()
        q0a = np.asarray(q0, float)
        lo = np.asarray(self.limits[0], float)
        hi = np.asarray(self.limits[1], float)
        limits = (self.limits[0], self.limits[1])

        def tilted(deg):
            return _axis_toward(axis0, (0.0, 0.0, -1.0), deg)

        candidates: list[tuple[float, float, np.ndarray, str]] = []
        for s in LIFT_SCALES:
            candidates.append((s, LIFT_Z_MM, axis0,
                               "收 " + f"{(1 - s) * 100:.0f}%" if s < 1 else "保持姿态"))
        for deg in LIFT_TILTS:
            candidates.append((1.0, LIFT_Z_MM, tilted(deg),
                               f"转腕 {deg:.0f}°"))
        for deg in (60.0, 90.0):
            candidates.append((0.96, LIFT_Z_MM, tilted(deg),
                               f"收 4% + 转腕 {deg:.0f}°"))
        for z in LIFT_LOWER_Z_MM:
            candidates.append((1.0, z, axis0, f"抬到 z={z:.0f}"))

        def attempt(scale, z, axis, seeds):
            """返回 (最小跳的解, 最小跳, 最近一次不达标的原因)。"""
            best = None
            worst = None
            for seed in seeds:
                try:
                    res = self.robot.ik(
                        [scale * float(p0[0]), scale * float(p0[1]), z],
                        axis, None, seed, self.preset, limits,
                        pos_tol_mm=0.5, axis_tol_deg=0.5,
                        structured_seeds=False, max_iters=100,
                        time_budget_s=0.10)
                except Exception as exc:  # noqa: BLE001
                    worst = str(exc)
                    continue
                if not res.ok:
                    worst = (f"残余 {res.pos_err_mm:.2f} mm / "
                             f"轴 {res.axis_err_deg:.2f}°")
                    continue
                jump = float(np.max(np.abs(
                    np.asarray(res.q_deg, float) - q0a)))
                if jump > LIFT_MAX_JUMP_DEG + 1e-9:
                    idx = int(np.argmax(np.abs(
                        np.asarray(res.q_deg, float) - q0a)))
                    worst = f"{self.order[idx]} 跳 {jump:.1f}°"
                    continue
                if best is None or jump < best[0]:
                    best = (jump, res)
            return best, worst

        best = None     # (cost, jump, scale, z, axis, label, res)
        worst_reason = ""
        for scale, z, axis, label in candidates:
            found, worst = attempt(scale, z, axis, [q0a])
            if worst:
                worst_reason = worst
            if found is None:
                continue
            jump, res = found
            axis_chg = math.degrees(math.acos(max(
                -1.0, min(1.0, float(np.dot(axis0, axis))))))
            cost = (jump + 0.6 * axis_chg
                    + 25.0 * (1.0 - scale)
                    + 8.0 * max(0.0, LIFT_Z_MM - z))
            if best is None or cost < best[0]:
                best = (cost, jump, scale, z, axis, label, res)
            # 同一 XY、原工具轴、跳 ≤45°：正常情况，不必再找
            if (scale >= 1.0 - 1e-9 and z >= LIFT_Z_MM - 1e-9
                    and axis_chg < 1e-6
                    and jump <= LIFT_GOOD_JUMP_DEG):
                best = (cost, jump, scale, z, axis, label, res)
                break

        # 第一轮都没找到时，再试 J1/J3/J5 翻 180° 的镜像分支
        if best is None:
            seeds = []
            for i, delta in ((0, 180.0), (0, -180.0), (2, 180.0),
                             (2, -180.0), (4, 180.0), (4, -180.0)):
                sig = q0a.copy()
                sig[i] += delta
                seeds.append(np.clip(sig, lo, hi))
            for scale, z, axis, label in candidates[:9]:
                found, worst = attempt(scale, z, axis, seeds)
                if worst:
                    worst_reason = worst
                if found is None:
                    continue
                jump, res = found
                axis_chg = math.degrees(math.acos(max(
                    -1.0, min(1.0, float(np.dot(axis0, axis))))))
                cost = (jump + 0.6 * axis_chg
                        + 25.0 * (1.0 - scale)
                        + 8.0 * max(0.0, LIFT_Z_MM - z))
                best = (cost, jump, scale, z, axis, label, res)
                break

        if best is None:
            raise LiftBranchError(
                f"抬高无连续解：{worst_reason or '无可用分支'}；"
                "请先支撑/手动抬高或分段点动")
        _, jump, scale, z, axis, label, res = best
        lift = self._plan_move(
            q0, res.q_deg, f"抬高（{label}）",
            allow_outside_start=True, relax_below_z=100.0)
        if lift:
            lift[-1]["lift"] = {
                "scale": round(float(scale), 4),
                "target_z_mm": round(float(z), 2),
                "axis": [round(float(v), 4) for v in axis],
                "jump_deg": round(float(jump), 3),
                "pos_err_mm": round(float(res.pos_err_mm), 4),
                "axis_err_deg": round(float(res.axis_err_deg), 4),
            }
        return lift

    def plan_park(self, q_from=None, park: dict | None = None,
                  include_down: bool = True,
                  allow_direct_fallback: bool = False) -> dict:
        """18.3 停机：抬高 → 收拢+折腕+转 J1 → 预停机点 → P。

        返回四段列表；执行器在 pre / park 到达后各停 1 s 读 I₀、I₁。
        """
        q0 = self.read_q() if q_from is None else list(q_from)
        pk = park or self.default_park()
        self.park_side = int(pk.get("park_side", getattr(self, "park_side", -1)))
        poses = dict(pk.get("poses") or {})
        q4, q5, q6 = self._park_wrist(pk, poses)
        pre_pose = list(poses["pre"])
        pre_pose[3] = q4
        pre_pose[4] = q5
        pre_pose[5] = q6
        park_pose = list(poses["park"])
        park_pose[3] = q4
        park_pose[4] = q5
        park_pose[5] = q6
        poses["pre"] = pre_pose
        poses["park"] = park_pose
        try:
            lift = self._plan_lift(q0)
        except LiftBranchError:
            if not allow_direct_fallback:
                raise
            # 离线/人工 staging：不抬高，直接关节空间到收拢位
            tuck = self._plan_move(q0, poses["tuck"], "收拢（无抬高兜底）")
            pre3a, pre3b = self._plan_pre_split(poses["tuck"], poses["pre"])
            pre = pre3a + pre3b
            down = (self._plan_move(poses["pre"], poses["park"], "下降到 P")
                    if include_down else [])
            return {"lift": [], "tuck": tuck, "pre": pre,
                    "pre3a": pre3a, "pre3b": pre3b, "park": down,
                    "poses": poses, "direct_fallback": True}
        cur = lift[-1]["q_target"] if lift else list(q0)
        tuck = self._plan_move(cur, poses["tuck"], "收拢+折腕+转 J1")
        pre3a, pre3b = self._plan_pre_split(poses["tuck"], poses["pre"])
        pre = pre3a + pre3b
        down = (self._plan_move(poses["pre"], poses["park"], "下降到 P")
                if include_down else [])
        return {"lift": lift, "tuck": tuck, "pre": pre,
                "pre3a": pre3a, "pre3b": pre3b, "park": down,
                "poses": poses}

    def _plan_pre_split(self, tuck_pose, pre_pose) -> tuple[list[dict], list[dict]]:
        """27.2 第 1 条：第 3 步拆成 3a（J2 不动，只动 J3 + 腕）和 3b（只动 J2）。

        3a 走完 J3 到位以后才开始 3b；3b 起加前臂俯仰检查（seg["pitch_check"]）。
        """
        mid = list(tuck_pose)
        mid[2] = float(pre_pose[2])
        mid[3], mid[4], mid[5] = (float(pre_pose[3]), float(pre_pose[4]),
                                  float(pre_pose[5]))
        a = self._plan_move(list(tuck_pose), mid, "3a J3 到预停机点")
        b = self._plan_move(mid, list(pre_pose), "3b J2 到预停机点")
        for seg in b:
            seg["pitch_check"] = True
        return a, b

    @staticmethod
    def _park_wrist(pk: dict, poses: dict) -> tuple[float, float, float]:
        """停机下降用的 J4/J5/J6：默认用教的 park 点，可被 descent_q4/j5/j6_deg 覆盖。"""
        p = list(poses.get("park") or [0.0] * 6)
        q4 = pk.get("descent_q4_deg")
        q5 = pk.get("descent_j5_deg")
        q6 = pk.get("descent_j6_deg")
        return (float(p[3]) if q4 is None else float(q4),
                float(p[4]) if q5 is None else float(q5),
                float(p[5]) if q6 is None else float(q6))

    def plan_unpark(self, q_from=None, park: dict | None = None) -> list[dict]:
        """18.3 从停机启动：J2/J3 同步抬回预停机点（离开搁架）。"""
        q0 = self.read_q() if q_from is None else list(q_from)
        pk = park or self.default_park()
        self.park_side = int(pk.get("park_side", getattr(self, "park_side", -1)))
        poses = dict(pk.get("poses") or {})
        q4, q5, q6 = self._park_wrist(pk, poses)
        pre = list(poses["pre"])
        pre[3] = q4
        pre[4] = q5
        pre[5] = q6
        # 27.2 第 1 条反序：先 J2 单独到预停机点的 q2（远端离开 V 槽），再 J3 单独到 q3
        mid = list(q0)
        mid[1] = float(pre[1])
        up_j2 = self._plan_move(list(q0), mid, "抬离搁架：先 J2 到预停机点")
        up_j3 = self._plan_move(mid, pre, "抬离搁架：再 J3 到预停机点")
        return up_j2 + up_j3

    # ---------------- 执行 ----------------
    def begin_command(self):
        """20.3：接受新命令时清停止标志并立即置 busy。

        这样从 /api/execute 返回“已启动”那一刻起，按“停”一定有效，
        不会掉进“线程还没跑到 execute”的空档。
        """
        with self._lock:
            self._stop.clear()
            self._stop_reason = "用户按停"
            self.status.update({
                "busy": True, "phase": "starting", "reason": "",
                "started": time.time(), "finished": None, "progress": 0.0,
            })

    def stop(self, reason: str = "用户按停"):
        # 20.3/20.7：停止标志无条件置位；不忙时也不能把 aborted/error 洗成 idle
        self._stop.set()
        self._stop_reason = reason
        with self._lock:
            if self.status.get("busy"):
                self.status["phase"] = "stopping"
                self.status["reason"] = reason
            elif self.status.get("phase") not in ("aborted", "error"):
                self.status["phase"] = "idle"
                self.status["reason"] = ""

    def _trend_log(self, why: str):
        """26.2 第 6 条：每段每个关节一行；中止时把前 2 s 的 d/u/ρ 存下来。"""
        t = getattr(self, "_trend_ctx", None)
        if not t:
            return
        self._trend_ctx = None
        try:
            if self.recorder:
                for n in self.order:
                    self.recorder.command(
                        "trend_segment",
                        f"{t['label']}（{why}）最大落后 {t['max_lag'][n]:.2f}°、"
                        f"最小 u {t['min_u'][n] if t['min_u'][n] is not None else '—'}、"
                        f"最大 ρ {t['max_rho'][n]:.2f}、触发 {t['trig'][n]}",
                        joint=n)
                if why not in ("到位",) and t["samples"]:
                    for smp in list(t["samples"])[-40:]:
                        for n, vals in smp["joints"].items():
                            self.recorder.command(
                                "trend_trace",
                                f"t={smp['t']}s 落后 {vals[0]}° u {vals[1]}°/s "
                                f"ρ {'—' if vals[2] is None else vals[2]}",
                                joint=n)
        except Exception:  # noqa: BLE001
            pass

    def _abort(self, reason: str):
        """中止 = 原地保持；只有六颗全部读不到（通信完全断）才全部失能。"""
        self._trend_log("中止")
        # 22.3：先给记录钩子一个结束信号（不允许它影响中止流程）
        try:
            self._call_hooks(None, None, None, "end",
                             str(self.status.get("label") or ""))
        except Exception:  # noqa: BLE001
            pass
        res = None
        try:
            res = self.bus.hold_all()
        except Exception as exc:  # noqa: BLE001
            reason += f"（保持时通信失败：{exc}）"
        unreadable = list((res or {}).get("unreadable") or [])
        unconfirmed = list((res or {}).get("unconfirmed") or [])
        if len(unreadable) >= len(self.order):
            reason += "；六颗都读不到位置（通信完全断），按 16.4 全部失能"
            try:
                self.bus.disable_all(True, "通信中断")
            except Exception:  # noqa: BLE001
                pass
        elif unconfirmed:
            reason += "；未确认保持：" + "，".join(
                f"{x.get('name')}({x.get('reason')})" for x in unconfirmed)
        with self._lock:
            self.status["phase"] = "aborted"
            self.status["reason"] = reason
        if self.recorder:
            self.recorder.command("abort", reason)
        raise MotionError(reason)

    @staticmethod
    def _trap_pos(d: float, v: float, a: float, t: float) -> float:
        """同一组梯形速度/加速度下，t 时刻应走的位移（带符号，deg）。"""
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

    @staticmethod
    def _trap_vel(d: float, v: float, a: float, t: float) -> float:
        """同一组梯形参数下，t 时刻的指令速度（带符号，deg/s）。"""
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
                s = a * t
            elif t <= t_total:
                s = a * (t_total - t)
            else:
                s = 0.0
        else:
            t_total = 2.0 * t_acc + (ad - 2.0 * d_acc) / v
            if t <= t_acc:
                s = a * t
            elif t <= t_total - t_acc:
                s = v
            elif t <= t_total:
                s = a * (t_total - t)
            else:
                s = 0.0
        return sign * s

    def _run_segment(self, seg: dict):
        q_target = np.asarray(seg["q_target"], float)
        q_from = np.asarray(seg["q_from"], float)
        d_all = q_target - q_from
        v_all = np.array([float((seg.get("vel_deg_s") or {}).get(n, 10.0))
                          for n in self.order])
        a_all = np.array([float((seg.get("acc_deg_s2") or {}).get(n, 20.0))
                          for n in self.order])
        bad = self.bus.verify_ready()
        if bad:
            self._abort("这些关节没有真正使能/不在位置模式，拒绝发目标："
                        + "，".join(f"{n}({m})" for n, m in bad))
        self.bus.set_profiles_q(seg["vel_deg_s"], seg["acc_deg_s2"],
                                disp={n: float(d_all[i]) for i, n in enumerate(self.order)})
        with self._lock:
            self.status.update({"phase": "moving", "label": seg["label"],
                                "target_q": seg["q_target"], "progress": 0.0})
        # 21.1：和 _run_chunks 同一口径，按停后不再补发新目标（段入口/
        # verify_ready 读参数期间按的停也不会漏过去）。
        if self._stop.is_set():
            self._abort(self._stop_reason)
        self.bus.set_targets_q(self.q_dict(q_target))
        if self.recorder:
            for n, q in zip(self.order, q_target):
                self.recorder.command("command_move", seg["label"], n, float(q))
        t0 = time.monotonic()
        # 计划 T 只按匀速估，实际还有加速段/摩擦/轮询；给足余量
        timeout = max(5.0, seg["T"] * 1.5 + 3.0)
        last_ok = {n: t0 for n in self.order}
        consec = 0
        lag_ref_t = {n: None for n in self.order}
        err0 = {n: None for n in self.order}
        # 26.2：运动趋势保护（取代原“落后应到位置”判法）
        from collections import deque
        trend = {
            "t0": t0, "label": seg["label"],
            "max_lag": {n: 0.0 for n in self.order},
            "min_u": {n: None for n in self.order},
            "max_rho": {n: 0.0 for n in self.order},
            "trig": {n: "无" for n in self.order},
            "away_since": {}, "stuck_since": {},
            "tight": bool(seg.get("descend") or seg.get("tight")),
            "samples": deque(maxlen=80),
        }
        self._trend_ctx = trend
        self._target_q = {n: float(q_target[i]) for i, n in enumerate(self.order)}
        # 27.2 第 2 条：前臂俯仰检查状态
        pitch_since = None
        pitch_dev_max = 0.0
        i_j2 = self.order.index("J2") if "J2" in self.order else 1
        i_j3 = self.order.index("J3") if "J3" in self.order else 2
        err_cache = {n: None for n in self.order}
        cmd_v = None
        tick = 0
        while True:
            if self._stop.is_set():
                self._abort(self._stop_reason)
            tick += 1
            # 22.2 扫描段要带电流/原始计数：read_full=True 时按全量回读。
            if seg.get("read_full"):
                states = self.bus.read_all(full=True)
            elif not hasattr(self.bus, "read_cvp_all"):
                states = self.bus.read_all(full=bool(seg.get("read_full")))
            else:
                # 26.2：运动/保持期间用 0x94 一条指令取位置/速度/电流（快）
                cvp = self.bus.read_cvp_all(timeout=0.03, retries=1)
                if tick % 5 == 1:
                    err_cache = self.bus.read_errors()
                states = {}
                for n in self.order:
                    c = self.bus.cfg[n]
                    d = cvp.get(n) or {}
                    ok = bool(d.get("ok"))
                    states[n] = JointState(
                        name=n, id=c.id, enabled=True if ok else None,
                        out_deg=(None if not ok else d.get("pos_deg")),
                        current_a=(None if not ok else d.get("current_a")),
                        velocity_deg_s=(None if not ok else d.get("vel_deg_s")),
                        error=err_cache.get(n), target_deg=self._target_q.get(n),
                        current_limit_a=self.bus.current_limit_readback.get(n),
                    )
            # 26.2 第 1 条：没有 0x94 速度时（扫描段 read_full），用最近
            # 位置差分估速度，别把“没速度”当成“不动”。
            now = time.monotonic()
            if "prev_q" not in locals():
                prev_q = {}
                prev_t = None
            if prev_t is not None and now > prev_t:
                dt_est = now - prev_t
                for n in self.order:
                    st = states.get(n)
                    if st is None or st.out_deg is None:
                        continue
                    if st.velocity_deg_s is None and n in prev_q:
                        st.velocity_deg_s = (float(st.out_deg) - prev_q[n]) / dt_est
            prev_q = {n: float(states[n].out_deg) for n in self.order
                      if states.get(n) is not None and states[n].out_deg is not None}
            prev_t = now
            if self.recorder:
                self.recorder.poll(states)
            q = []
            errs: dict[str, float | None] = {}
            for i, n in enumerate(self.order):
                st = states[n]
                if st.out_deg is None:
                    q.append(None)
                    errs[n] = None
                    if now - last_ok[n] > 0.5:
                        self._abort(f"{n} 0.5 s 没应答")
                    continue
                last_ok[n] = now
                q.append(st.out_deg)
                errs[n] = abs(st.out_deg - q_target[i])
                if err0[n] is None:
                    err0[n] = errs[n]
                lo_s, hi_s = self.bus.cfg[n].safe_limits_deg
                if st.out_deg < lo_s - 1.0 or st.out_deg > hi_s + 1.0:
                    self._abort(
                        f"{n} 实际位置 {st.out_deg:.1f}° 超出安全限位 "
                        f"{lo_s:.0f}..{hi_s:.0f}°（撞限位）")
                if st.error:
                    self._abort(f"{n} 报警 0x{st.error:04X} {st.error_text}")
            if None not in q:
                # 22.2/22.3：把这一拍的指令位置/速度和实际回读交给钩子。
                elapsed = now - t0
                cmd_q = [float(q_from[i]) + self._trap_pos(
                    float(d_all[i]), v_all[i], a_all[i], elapsed)
                    for i in range(6)]
                cmd_v = [self._trap_vel(
                    float(d_all[i]), v_all[i], a_all[i], elapsed)
                    for i in range(6)]
                self._call_hooks(states, cmd_q, cmd_v, "run", seg["label"],
                                 elapsed, float(seg.get("T") or 0.0))
                # 27.2 第 2 条：前臂俯仰检查（只查“比计划低”这一方向）
                if seg.get("pitch_check"):
                    exp_sum = float(cmd_q[i_j2]) + float(cmd_q[i_j3])
                    act_sum = float(q[i_j2]) + float(q[i_j3])
                    dev = act_sum - exp_sum
                    if dev > pitch_dev_max:
                        pitch_dev_max = dev
                        with self._lock:
                            self.status["pitch_dev_max"] = round(dev, 2)
                    if dev > PITCH_TOL_DEG:
                        if pitch_since is None:
                            pitch_since = now
                        elif now - pitch_since >= PITCH_HOLD_S:
                            self._abort(
                                f"前臂俯仰偏 {dev:.1f}°"
                                f"（J2 落后 {float(q[i_j2]) - float(cmd_q[i_j2]):.1f}°、"
                                f"J3 落后 {float(q[i_j3]) - float(cmd_q[i_j3]):.1f}°）")
                    else:
                        pitch_since = None
                maxerr = max(errs.values())
                # 0.8° 以内算到位；J3 这种有摩擦/背隙的关节常停在 0.3~0.5°
                if maxerr < 0.8:
                    consec += 1
                else:
                    consec = 0
                if consec >= 3:
                    with self._lock:
                        self.status["phase"] = "arrived"
                        self.status["progress"] = 1.0
                        self.status["current_q"] = [round(v, 4) for v in q]
                    if self.recorder:
                        for i, n in enumerate(self.order):
                            err = float(q[i] - q_target[i])
                            self.recorder.command(
                                "arrived_error",
                                f"{seg['label']} 到位误差 {err:+.3f}°",
                                n, float(q_target[i]), pos_deg=float(q[i]),
                                error=states[n].error)
                    self._call_hooks(states, q_target.tolist(), [0.0] * 6, "end",
                                     seg["label"], elapsed,
                                     float(seg.get("T") or 0.0))
                    self._trend_log("到位")
                    return
            # 26.2：运动趋势判法
            if None not in q and cmd_v is not None:
                # 起步 1 s / 5° 反向空程照旧不算
                in_start = now - t0 <= START_GRACE_S
                sample = {"t": round(now - t0, 3), "joints": {}}
                for i, n in enumerate(self.order):
                    st = states[n]
                    if st.out_deg is None:
                        continue
                    vp = abs(float(v_all[i]))          # 本轴规划峰值速度
                    should = abs(float(cmd_v[i])) > 2.0
                    if not should:
                        continue
                    d_end = float(q_target[i]) - float(st.out_deg)
                    sdir = 1.0 if d_end >= 0 else -1.0
                    u = sdir * float(st.velocity_deg_s or 0.0)
                    lim = (self.bus.current_limit_readback.get(n)
                           or (self.bus.cfg[n].required_current_limit_a))
                    rho = (None if (st.current_a is None or not lim)
                           else abs(float(st.current_a)) / float(lim))
                    u_min = max(0.5, 0.1 * vp)
                    # 落后量（相对主机算的应到位置，沿运动方向）
                    d_move = float(d_all[i])
                    if abs(d_move) > 0.1:
                        expected = float(q_from[i]) + self._trap_pos(
                            d_move, v_all[i], a_all[i], now - t0)
                        lag = (expected - float(q[i])) * (1.0 if d_move > 0 else -1.0)
                    else:
                        lag = abs(float(q[i]) - float(q_from[i]))
                    trend["max_lag"][n] = max(trend["max_lag"][n], lag)
                    if trend["min_u"][n] is None or u < trend["min_u"][n]:
                        trend["min_u"][n] = u
                    if rho is not None:
                        trend["max_rho"][n] = max(trend["max_rho"][n], rho)
                    sample["joints"][n] = [round(lag, 2), round(u, 2),
                                           None if rho is None else round(rho, 2)]
                    # 26.2：起步 1 s 内、5° 以内的反向空程不算；反向超过 5°
                    # 立即中止；正向、不动照常判。
                    back = ((float(q[i]) - float(q_from[i]))
                            * (1.0 if d_all[i] >= 0 else -1.0))
                    if in_start and back < 0:
                        if back < -START_REVERSE_DEG:
                            trend["trig"][n] = "启动反向"
                            self._trend_log("启动反向")
                            self._abort(
                                f"{n} 启动反向 {-back:.1f}° >"
                                f"{START_REVERSE_DEG:g}°（起始误差 {err0[n]:.1f}°）")
                        trend["away_since"].pop(n, None)
                        trend["stuck_since"].pop(n, None)
                        continue
                    if u >= u_min:
                        # 慢但在追：不中止
                        trend["away_since"].pop(n, None)
                        trend["stuck_since"].pop(n, None)
                        continue
                    # 背离：u <= -1 持续 0.3 s
                    if u <= -1.0:
                        t_away = trend["away_since"].get(n)
                        if t_away is None:
                            trend["away_since"][n] = now
                        elif now - t_away >= 0.3:
                            kind = "没抵抗" if (rho is None or rho < 0.3) else "被推走"
                            trend["trig"][n] = "背离、没抵抗" if kind == "没抵抗" else "背离、被推走"
                            self._trend_log("背离")
                            self._abort(
                                f"{n} 背离目标（{kind}），电流占比 "
                                f"{'无' if rho is None else f'{rho*100:.0f}%'}"
                                f"；目标 {float(q_target[i]):.1f}°，实际 {float(q[i]):.1f}°，u={u:.1f}°/s")
                        continue
                    else:
                        trend["away_since"].pop(n, None)
                    # 不动：|u| < u_min 且 |d| > 0.8
                    if abs(u) < u_min and abs(d_end) > 0.8:
                        t_st = trend["stuck_since"].get(n)
                        if t_st is None:
                            trend["stuck_since"][n] = now
                        elif now - t_st >= 0.5:
                            if rho is not None and rho >= 0.9:
                                trend["trig"][n] = "卡住或力矩不够"
                                self._trend_log("卡住")
                                self._abort(
                                    f"{n} 卡住或力矩不够，电流占比 {rho*100:.0f}%；"
                                    f"目标 {float(q_target[i]):.1f}°，实际 {float(q[i]):.1f}°")
                            else:
                                trend["trig"][n] = "不动"
                                self._trend_log("不动")
                                self._abort(
                                    f"{n} 不动，电流占比 "
                                    f"{'无' if rho is None else f'{rho*100:.0f}%'}；"
                                    f"目标 {float(q_target[i]):.1f}°，实际 {float(q[i]):.1f}°")
                        continue
                    else:
                        trend["stuck_since"].pop(n, None)
                # 26.2 第 4 条：不规划在动的关节也要兜底（被带走）
                for i, n in enumerate(self.order):
                    st = states[n]
                    if st.out_deg is None:
                        continue
                    d_move = float(d_all[i])
                    if abs(d_move) > 0.1:
                        expected = float(q_from[i]) + self._trap_pos(
                            d_move, v_all[i], a_all[i], now - t0)
                        lag = (expected - float(q[i])) * (1.0 if d_move > 0 else -1.0)
                    else:
                        expected = float(q_from[i])
                        lag = abs(float(q[i]) - expected)
                    trend["max_lag"][n] = max(trend["max_lag"][n], lag)
                    lim_lag = (max(3.0, abs(float(v_all[i])) * 0.5)
                               if trend["tight"]
                               else max(15.0, abs(float(v_all[i])) * 2.0))
                    if lag > lim_lag:
                        if lag_ref_t[n] is None:
                            lag_ref_t[n] = now
                        elif now - lag_ref_t[n] > 0.3:
                            trend["trig"][n] = "落后兜底"
                            self._trend_log("落后")
                            self._abort(
                                f"{n} 落后 {lag:.1f}° > {lim_lag:.1f}° 且持续 0.3 s"
                                f"（应到位置 {expected:.2f}°，实际 {q[i]:.2f}°）")
                    else:
                        lag_ref_t[n] = None
                trend["samples"].append(sample)
            if now - t0 > timeout:
                self._abort(f"到达超时（{timeout:.1f} s）")
            if None not in q:
                with self._lock:
                    self.status["current_q"] = [round(v, 4) for v in q]
                    # 21.4.7：进度条按实际位置（位移最大的关节）算，不按时间
                    prog = 0.0
                    for i in range(6):
                        den = abs(float(d_all[i]))
                        if den > 0.05:
                            prog = max(prog, min(1.0, abs(
                                float(q[i]) - float(q_from[i])) / den))
                    self.status["progress"] = round(min(0.99, prog), 3)
            time.sleep(max(0.0, 1.0 / self.poll_hz - (time.monotonic() - now)))

    @staticmethod
    def _group_segments(segments: list[dict]) -> list[list[dict]]:
        """把同一条长动作拆出来的 “xxx 1/19” 段归成一组，组内连续走。"""
        groups: list[tuple[str, list[dict]]] = []
        for seg in segments:
            key = re.sub(r"\s+\d+/\d+$", "", str(seg.get("label", "")))
            if groups and groups[-1][0] == key:
                groups[-1][1].append(seg)
            else:
                groups.append((key, [seg]))
        return [g for _, g in groups]

    def _run_chunks(self, segs: list[dict]):
        """连续执行一组 5° 拆段：沿同一条直线按时间流式下发小目标。

        2026-11-15：旧做法每段都等 <0.8° 三拍、再 sleep 0.2 s，长动作会
        走一步停一下。这里把整组当成一条直线，每 0.1 s 把目标推进 0.5–1°
        （远小于驱动的 5° 阶跃上限），驱动看到的是连续移动的目标；
        仍逐轮检查停止/报警/限位，误差 >8° 且 0.4 s 内降不到 1° → 中止。
        """
        label = re.sub(r"\s+\d+/\d+$", "", str(segs[0].get("label", "")))
        bad = self.bus.verify_ready()
        if bad:
            self._abort("这些关节没有真正使能/不在位置模式，拒绝发目标："
                        + "，".join(f"{n}({m})" for n, m in bad))
        q0 = np.asarray(segs[0]["q_from"], float)
        qf = np.asarray(segs[-1]["q_target"], float)
        delta = qf - q0
        T = 0.5
        for i, n in enumerate(self.order):
            v = max(0.25, float(segs[0]["vel_deg_s"].get(n, 10.0)))
            T = max(T, abs(float(delta[i])) / v)
        vel = {}
        acc = {}
        for i, n in enumerate(self.order):
            # 余弦插补的峰值速度是平均值的 π/2≈1.57 倍；驱动 profile 的
            # 最大速度要按峰值给，否则目标会一直跑在实际前面。
            v = max(0.25, 1.6 * abs(float(delta[i])) / T)
            v = min(float(self.bus.cfg[n].motion_speed_deg_s), v)
            vel[n] = v
            acc[n] = v / max(self.bus.cfg[n].motion_accel_time_s, 0.05)
        stream_pos = False
        if hasattr(self.bus, "set_stream_mode"):
            stream_pos = bool(self.bus.set_stream_mode(MODE_POSITION))
        if not stream_pos:
            self.bus.set_profiles_q(vel, acc,
                                    disp={n: float(delta[i]) for i, n in enumerate(self.order)})
        with self._lock:
            self.status.update({"phase": "moving", "label": label,
                                "target_q": [round(float(v), 4) for v in qf],
                                "progress": 0.0})
        if self.recorder:
            for n, q in zip(self.order, qf):
                self.recorder.command("command_move", label, n, float(q))

        # 官方 gluon SDK 的 MoveJointPath 是 4 ms 插补一次（Mode_Pos 3）；
        # Python + ECB 做不到 250 Hz，这里按“墙上时间”推进目标（不是按步数），
        # 每次最多 20 ms 发一次，目标步长 ≤ 约 0.2–0.3°；状态回读 20 Hz。
        POLL_DT = 0.05
        TARGET_DT = 0.02
        t_start = time.monotonic()
        timeout = max(10.0, T * 0.8 + 8.0)   # 只用于“目标流结束后等真正到位”
        last_ok = {n: t_start for n in self.order}
        err_ref = {n: None for n in self.order}
        err_ref_t = {n: t_start for n in self.order}
        last_poll = 0.0
        last_send = 0.0
        stall_lim = [12.0 if abs(float(delta[i])) > 10.0
                     else 8.0 if abs(float(delta[i])) > 2.0
                     else 5.0
                     for i in range(6)]

        def read_once(cmd, cmd_vel=None):
            states = self.bus.read_all(full=bool(segs[0].get("read_full")))
            if self.recorder:
                self.recorder.poll(states)
            now = time.monotonic()
            q = []
            errs: dict[str, float | None] = {}
            for i, n in enumerate(self.order):
                st = states[n]
                if st.out_deg is None:
                    q.append(None)
                    errs[n] = None
                    if now - last_ok[n] > 0.5:
                        self._abort(f"{n} 0.5 s 没应答")
                    continue
                last_ok[n] = now
                q.append(st.out_deg)
                errs[n] = abs(st.out_deg - float(cmd[i]))
                lo_s, hi_s = self.bus.cfg[n].safe_limits_deg
                if st.out_deg < lo_s - 1.0 or st.out_deg > hi_s + 1.0:
                    self._abort(
                        f"{n} 实际位置 {st.out_deg:.1f}° 超出安全限位 "
                        f"{lo_s:.0f}..{hi_s:.0f}°（撞限位）")
                if st.error:
                    self._abort(f"{n} 报警 0x{st.error:04X} {st.error_text}")
            maxerr = None if None in errs.values() else max(errs.values())
            if None not in q:
                self._call_hooks(states, [float(v) for v in cmd],
                                 (cmd_vel or [0.0] * 6), "run", label,
                                 now - t_start, T)
            return q, maxerr, states

        while True:
            # 21.1：目标流每一拍发目标之前先查停止标志。这一段以前没有检查
            # （578fc36 把 for 改成 while True 时丢了），长动作十几秒内按“停”、
            # 心跳断都不生效。置位后不再发新目标，按 _abort 原地保持。
            if self._stop.is_set():
                self._abort(self._stop_reason)
            now = time.monotonic()
            u = min(1.0, (now - t_start) / T)
            sfrac = 0.5 - 0.5 * math.cos(math.pi * u)
            cmd = q0 + delta * sfrac
            self.bus.set_targets_q(self.q_dict(cmd))
            last_send = time.monotonic()
            if now - last_poll >= POLL_DT:
                last_poll = now
                # 余弦插补的指令速度（仅给 22.3 残差记录用）
                cv = (delta * (0.5 * math.pi / max(T, 1e-6))
                      * math.sin(math.pi * u)).tolist()
                q, maxerr, _ = read_once(cmd, cv)
                now = time.monotonic()
                if maxerr is not None:
                    with self._lock:
                        self.status["current_q"] = [round(v, 4) for v in q]
                    # 起步 1 s 内抓反向空程（>5° 才停）
                    if now - t_start <= START_GRACE_S:
                        for i, n in enumerate(self.order):
                            if q[i] is None or abs(float(delta[i])) < 1.0:
                                continue
                            sign = 1.0 if float(delta[i]) > 0 else -1.0
                            back = (float(q[i]) - float(q0[i])) * sign
                            if back < -START_REVERSE_DEG:
                                self._abort(
                                    f"{n} 启动反向 {(-back):.1f}° >"
                                    f"{START_REVERSE_DEG:g}°")
                    if maxerr > 5.0:
                        for i, n in enumerate(self.order):
                            e = (abs(float(q[i]) - float(cmd[i]))
                                 if q[i] is not None else None)
                            lim = stall_lim[i]
                            if e is None or e <= lim:
                                err_ref[n] = None
                                err_ref_t[n] = now
                                continue
                            ref = err_ref[n]
                            if ref is None or e <= ref - 1.0:
                                err_ref[n] = e
                                err_ref_t[n] = now
                            elif now - err_ref_t.get(n, t_start) > 0.4:
                                self._abort(
                                    f"{n} 实际位置和目标差 {e:.1f}° >{lim:g}°，"
                                    "且 0.4 s 内下降不到 1°")
                    else:
                        for n in self.order:
                            err_ref[n] = None
                            err_ref_t[n] = now
            if u >= 1.0:
                break
            sleep = TARGET_DT - (time.monotonic() - last_send)
            if sleep > 0:
                time.sleep(sleep)
            with self._lock:
                self.status["progress"] = round(min(0.95, u), 3)

        # 最后精确下发终点并等真正到位（21.1：目标流末尾也要先看停止标志，
        # 免得刚好在 u 到 1 的这一刻按停又被补发一次终点目标）
        if self._stop.is_set():
            self._abort(self._stop_reason)
        self.bus.set_targets_q(self.q_dict(qf))
        consec = 0
        while True:
            if self._stop.is_set():
                self._abort(self._stop_reason)
            q, maxerr, states = read_once(qf)
            now = time.monotonic()
            if maxerr is not None:
                if maxerr < 0.8:
                    consec += 1
                else:
                    consec = 0
                if consec >= 3:
                    with self._lock:
                        self.status["phase"] = "arrived"
                        self.status["progress"] = 1.0
                        self.status["current_q"] = [round(v, 4) for v in q]
                    if self.recorder:
                        for i, n in enumerate(self.order):
                            self.recorder.command(
                                "arrived_error", f"{label} 到位误差 "
                                f"{float(q[i]) - float(qf[i]):+.3f}°",
                                n, float(qf[i]), pos_deg=float(q[i]),
                                error=states[n].error)
                    self._call_hooks(states, [float(v) for v in qf],
                                     [0.0] * 6, "end", label, now - t_start, T)
                    if stream_pos:
                        # 回到 S 曲线位置模式，后续单段/保持仍按原逻辑
                        self.bus.set_stream_mode(MODE_PROFILE_POSITION)
                        self.bus.set_profiles_q(
                            vel, acc,
                            disp={n: float(delta[i]) for i, n in enumerate(self.order)})
                    return
            if now - t_start > timeout:
                self._abort(f"到达超时（{timeout:.1f} s）")
            time.sleep(max(0.0, 1.0 / self.poll_hz
                           - (time.monotonic() - now)))

    def execute(self, segments: list[dict], preview: bool = False,
                pause_between: float = 0.0, clear_stop: bool = True) -> dict:
        """执行（或预演）一组已规划好的段。预演绝不写电机。

        clear_stop=False 用于“命令已在接受时 begin_command() 过”，
        避免把刚按下的停又清掉。
        """
        if clear_stop:
            self._stop.clear()
            self._stop_reason = "用户按停"
        elif self._stop.is_set():
            self._abort(self._stop_reason or "启动前已按停")
        for seg in segments:
            self._check_q(seg["q_target"], seg.get("descend", False))
        with self._lock:
            self.status.update({
                "busy": True, "phase": "preview" if preview else "running",
                "reason": "", "segments": segments, "started": time.time(),
                "finished": None, "progress": 0.0,
            })
        if self.recorder:
            self.recorder.command("plan" if preview else "execute_start",
                                  f"{len(segments)} 段" + ("（预演，不发写命令）" if preview else ""))
        if preview:
            with self._lock:
                self.status.update({"busy": False, "phase": "preview_done"})
            return {"preview": True, "segments": segments}
        try:
            groups = self._group_segments(segments)
            for i, group in enumerate(groups):
                with self._lock:
                    self.status["index"] = i
                if len(group) > 1:
                    self._run_chunks(group)
                else:
                    self._run_segment(group[0])
                if i < len(groups) - 1 and pause_between > 0:
                    time.sleep(pause_between)
            with self._lock:
                self.status.update({"busy": False, "phase": "done",
                                    "finished": time.time(), "progress": 1.0})
            if self.recorder:
                self.recorder.command("execute_done", "全部段到达")
            return {"ok": True, "segments": segments}
        except MotionError as exc:
            with self._lock:
                self.status.update({"busy": False, "phase": "aborted",
                                    "reason": str(exc), "finished": time.time()})
            raise
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.status.update({"busy": False, "phase": "error",
                                    "reason": str(exc), "finished": time.time()})
            try:
                res = self.bus.hold_all()
                un = (res or {}).get("unconfirmed") or []
                if un:
                    with self._lock:
                        self.status["reason"] += "；未确认保持：" + "，".join(
                            f"{x.get('name')}({x.get('reason')})" for x in un)
            except Exception:  # noqa: BLE001
                pass
            raise
