#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十二节 22.2 / 22.3 / 22.4 的 mock 自检（不连 ECB、不写真 config）。

运行：cd arm-control && python3 tools/task22_offline.py [--quick]

--quick：22.2 三条扫描用 30°/s（默认 3°/s，完整采集一遍约 5–6 分钟）。
真机相关项（22.4 用户录 5 点、22.2 真机扫描）不在这里做。
"""
from __future__ import annotations

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm import config as cfgmod
from arm import gravity
from arm.collision import CollisionMonitor
from arm.executor import Executor
from arm.fence import Fence
from arm.joints import load_joint_configs
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion, MotionError
from arm.programs import ProgramStore
from arm.sweep import SWEEP_JOBS, SweepRunner, fit_sweep, plan_sweep

FAILS: list[str] = []
ROBOT = RobotModel(cfgmod.URDF, cfgmod.tcp_presets())
JC = load_joint_configs()
LIM = ([JC[n].safe_limits_deg[0] for n in JC],
       [JC[n].safe_limits_deg[1] for n in JC])
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def check(name: str, cond: bool, detail: str = ""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


class PhysicsBus(MockJointBus):
    """按已知 Kt/摩擦/电流偏置/零偏生成 0x04 电流。"""

    def __init__(self, params: dict, offsets: dict | None = None):
        super().__init__()
        self.params = params          # {joint: (kt, f, c)}
        self.offsets = offsets or {}  # {joint: zero_offset_deg}
        self.inject: list[dict] = []
        self.read_times: list[float] = []
        self.current_fn = self._current

    def _current(self, name, q, v):
        p = self.params.get(name)
        if not p:
            return 0.0
        kt, f, c = p
        idx = SWEEP_JOBS[name]["index"]
        qq = [float(q[n]) for n in NAMES]
        qq[idx] += float(self.offsets.get(name, 0.0))
        tau = float(gravity.torques(ROBOT, qq, "flange", 0.0)[idx])
        cur = tau / kt + f * math.tanh(float(v.get(name, 0.0)) / 1.0) + c
        now = time.time()
        for inj in self.inject:
            if inj["joint"] == name and now >= inj["after"]:
                cur += float(inj["amp"])
        return cur

    def read_all(self, full: bool = False):
        self.read_times.append(time.monotonic())
        return super().read_all(full)


def build_exec(bus) -> Executor:
    e = Executor()
    e.bus = bus
    e.mode = "mock"
    e.connected = True
    e.ip = "mock"
    e.msg = "自检 mock"
    e.runs_dir = Path(tempfile.mkdtemp(prefix="arm22_runs_"))
    e.speed_scale = 0.5
    e.motion = Motion(bus, ROBOT, Fence(ROBOT), LIM, recorder=None)
    e.motion.preset = "flange"
    e.motion.yaw_deg = 0.0
    e.collision = CollisionMonitor(e, log_dir=e.runs_dir)
    e.collision.start()
    e.motion.add_hook(e.collision.feed)
    e.program_store = ProgramStore(Path(tempfile.mkdtemp(prefix="arm22_prog_")))
    e._sync_motion_params()
    return e


def close_exec(e: Executor):
    try:
        if e.collision:
            e.collision.close()
    except Exception:  # noqa: BLE001
        pass
    try:
        e.motion.stop("自检结束")
    except Exception:  # noqa: BLE001
        pass


def wait(cond, timeout=120.0, step=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(step)
    return False


# --------------------------------------------------------------------------- #
# 22.2 拟合函数：已知 Kt/摩擦/零偏，纯合成数据
# --------------------------------------------------------------------------- #
def test_fit_synthetic():
    cases = {"J2": (1.80, 0.18, 0.03, 0.5),
             "J3": (1.28, 0.12, -0.02, -0.5),
             "J4": (0.205, 0.05, 0.01, 0.0)}
    for joint, (kt, f, c, d0) in cases.items():
        idx = SWEEP_JOBS[joint]["index"]
        samples = []
        for direction in (1, -1):
            for ang in [SWEEP_JOBS[joint]["preferred_start"] +
                        i * (SWEEP_JOBS[joint]["end"] -
                             SWEEP_JOBS[joint]["preferred_start"]) / 300
                        for i in range(301)]:
                q = [0.0] * 6
                q[idx] = float(ang) + d0
                tau = float(gravity.torques(ROBOT, q, "flange", 0.0)[idx])
                samples.append({"q": [float(ang)] * 6, "dir": direction,
                                "current": tau / kt + f * direction + c})
                samples[-1]["q"] = [float(v) for v in [0, 0, 0, 0, 0, 0]]
                samples[-1]["q"][idx] = float(ang)
        r = fit_sweep(samples, joint, ROBOT)
        e_kt = abs(r["kt_eff"] - kt) / kt * 100
        e_f = abs(r["i_fric"] - f)
        e_c = abs(r["i_bias"] - c)
        ok = (e_kt < 3.0 and e_f < 0.03 * max(abs(f), 0.02)
              and e_c < 0.03 * max(abs(c), 0.02)
              and abs(r["zero_offset_deg"] - d0) <= 0.5)
        check(f"22.2 拟合 {joint} 误差 <3%", ok,
              f"Kt {r['kt_eff']:.4f}/{kt}（{e_kt:.2f}%），f {r['i_fric']:.4f}/{f}，"
              f"c {r['i_bias']:.4f}/{c}，δ {r['zero_offset_deg']}°/{d0}°，R² {r['r2']}")


# --------------------------------------------------------------------------- #
# 22.2 扫描采集完整跑一遍（mock 物理电流）
# --------------------------------------------------------------------------- #
def test_scan(joint: str, speed: float):
    params = {"J2": (1.80, 0.18, 0.03), "J3": (1.28, 0.12, -0.02),
              "J4": (0.205, 0.05, 0.01)}
    offs = {"J2": 0.5, "J3": -0.5, "J4": 0.0}
    bus = PhysicsBus(params, offs)
    e = build_exec(bus)
    try:
        e.bus.enable_all(True, allow_restart=True)
        e.kt = {}
        runner = SweepRunner(e, joint, speed_deg_s=speed, save=False)
        e.sweep_runner = runner
        e.motion.begin_command()
        runner.start()
        ok = wait(lambda: not runner.status.get("active"), timeout=400)
        fit = runner.status.get("fit") or {}
        kt, f, c = params[joint]
        e_kt = (abs(fit.get("kt_eff") - kt) / kt * 100
                if fit.get("kt_eff") is not None else 999)
        e_f = abs((fit.get("i_fric") or 0.0) - f)
        e_c = abs((fit.get("i_bias") or 0.0) - c)
        good = (ok and fit.get("ok") and e_kt < 3.0
                and e_f < 0.03 * max(f, 0.02) and e_c < 0.03 * max(abs(c), 0.02))
        check(f"22.2 {joint} 扫描采集+拟合", good,
              f"{fit.get('n')} 样本，{speed:g}°/s，Kt {fit.get('kt_eff')}/{kt}"
              f"（{e_kt:.2f}%），f {fit.get('i_fric')}/{f}，c {fit.get('i_bias')}/{c}，"
              f"δ {fit.get('zero_offset_deg')}°，R² {fit.get('r2')}，"
              f"路径 {runner.path or '（未写盘）'}")
        return runner
    finally:
        close_exec(e)


# --------------------------------------------------------------------------- #
# 22.2 三条真机扫描路径的限位/围栏/力矩预检
# --------------------------------------------------------------------------- #
def test_scan_paths():
    e = build_exec(MockJointBus())
    try:
        e.bus.enable_all(True, allow_restart=True)
        for joint in ("J2", "J3", "J4"):
            p = plan_sweep(e.motion, joint)
            if not p.get("ok"):
                check(f"22.2 {joint} 扫描路径预检", False, p.get("reason", ""))
                continue
            max_step = max(x["max_step_deg"] for x in p["checks"]["path_checks"])
            ratio = p["checks"]["torque_max_ratio"]
            check(f"22.2 {joint} 扫描路径预检", max_step <= 0.5001 and ratio < 0.8,
                  f"{p['start']}°→{p['end']}°，去 {p['checks']['out_segments']} 段 / "
                  f"回 {p['checks']['back_segments']} 段，路径点间隔 ≤{max_step}°，"
                  f"最大力矩 {ratio * 100:.1f}% 容量，warnings={p['warnings']}")
    finally:
        close_exec(e)


# --------------------------------------------------------------------------- #
# 22.4 录 5 点 → 存 → 载入 → 回放
# --------------------------------------------------------------------------- #
def program_points():
    return [[0, 0, 0, 0, 0, 0],
            [10, 10, 0, 0, 0, 0],
            [20, 20, 10, 0, 0, 0],
            [0, 30, 20, 10, 0, 0],
            [-10, 10, 0, 0, 0, 0]]


def record_points(e, pts, name="selftest5"):
    for i, q in enumerate(pts):
        for n, v in zip(NAMES, q):
            e.bus.q[n] = float(v)
            e.bus.target[n] = float(v)
            e.bus.v[n] = 0.0
        e.program_record(f"p{i + 1}")
    r = e.program_save(name)
    return r


def test_program_roundtrip():
    e = build_exec(MockJointBus())
    arrivals: dict[int, list[float]] = {}

    def hook(states, cmd_q, cmd_vel, phase, label, elapsed, duration):
        if phase != "end" or "22.4 回放" not in str(label):
            return
        try:
            num = int(str(label).split("第 ")[1].split(" 点")[0])
        except Exception:  # noqa: BLE001
            return
        if states:
            q = [states[n].out_deg for n in NAMES]
            if all(v is not None for v in q):
                arrivals[num] = [float(v) for v in q]

    e.motion.add_hook(hook)
    try:
        e.bus.enable_all(True, allow_restart=True)
        pts = program_points()
        record_points(e, pts)
        e.program_load("selftest5")
        e.program_replay("selftest5", loops=1, confirmed=True)
        ok = wait(lambda: not e.program_status.get("active"), timeout=60)
        errs = []
        for i, q in enumerate(pts):
            got = arrivals.get(i + 1)
            if got is None:
                errs.append(f"第{i + 1}点没记录到")
                continue
            err = max(abs(got[k] - q[k]) for k in range(6))
            if err >= 0.8:
                errs.append(f"第{i + 1}点差 {err:.2f}°")
        check("22.4 录 5 点→存→载入→回放 各点 <0.8°",
              ok and not errs and e.program_status.get("phase") == "done",
              f"到达 {sorted(arrivals)}；{('；'.join(errs) if errs else '误差均 <0.8°')}")

        # 一点改到围栏外 → 载入被拒并指出第 3 点
        p = e.program_store.path("selftest5")
        import json
        d = json.loads(p.read_text(encoding="utf-8"))
        d["points"][2]["q_deg"][0] = 200.0
        p.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
        try:
            e.program_load("selftest5")
            check("22.4 围栏外点载入被拒并指出第 3 点", False, "居然载入成功")
        except MotionError as exc:
            check("22.4 围栏外点载入被拒并指出第 3 点",
                  "第 3 点" in str(exc), str(exc)[:100])
        # 恢复
        d["points"][2]["q_deg"][0] = 20.0
        p.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
        e.program_load("selftest5")

        # 回放中按停 → 停住、不失能
        e.program_replay("selftest5", loops=1, confirmed=True)
        wait(lambda: e.program_status.get("point", 0) >= 1, timeout=10)
        time.sleep(0.4)
        e.stop_all("22.4 回放按停自检")
        ok2 = wait(lambda: not e.program_status.get("active"), timeout=10)
        qs = e.bus.q
        held = all(abs(e.bus.target[n] - qs[n]) < 1e-6 for n in NAMES)
        check("22.4 回放中按停：停住、不失能",
              ok2 and e.program_status.get("phase") == "aborted"
              and e.bus.enabled and held,
              f"{e.program_status.get('phase')} / {e.program_status.get('reason', '')[:60]}")
    finally:
        close_exec(e)


# --------------------------------------------------------------------------- #
# 22.3 残差记录：叠 1 A 0.2 s 内标记；不加时 200 次运动 0 标记
# --------------------------------------------------------------------------- #
def test_collision_flag():
    params = {"J2": (1.91, 0.10, 0.0), "J3": (1.35, 0.08, 0.0),
              "J4": (0.22, 0.03, 0.0), "J5": (0.22, 0.03, 0.0)}
    bus = PhysicsBus(params)
    e = build_exec(bus)
    try:
        e.bus.enable_all(True, allow_restart=True)
        # 监视器用同一组标定值预测（摩擦/偏置已知），阈值下限 0.5 A
        e.kt = {n: {"kt_eff": p[0], "i_fric": p[1], "i_bias": p[2], "rms": 0.0}
                for n, p in params.items()}
        t_inj = time.time() + 0.6
        bus.inject.append({"joint": "J2", "after": t_inj, "amp": 1.0})
        segs = e.motion._plan_move([0] * 6, [0, 20, 0, 0, 0, 0], "22.3 注入")
        e.motion.begin_command()
        e.motion.execute(segs, preview=False, clear_stop=False)
        evs = [x for x in e.collision.flag_events if x["joint"] == "J2"]
        lat = (min(x["t"] for x in evs) - t_inj) if evs else 999
        check("22.3 叠 1 A 后 0.2 s 内被标记",
              0 <= lat <= 0.25 and e.motion.status.get("phase") in ("arrived", "done"),
              f"延迟 {lat * 1000:.0f} ms；事件 {evs[:2]}；运动仍到位（第一阶段不中止）")
    finally:
        close_exec(e)


def test_collision_no_false():
    params = {"J2": (1.91, 0.10, 0.0), "J3": (1.35, 0.08, 0.0),
              "J4": (0.22, 0.03, 0.0), "J5": (0.22, 0.03, 0.0)}
    bus = PhysicsBus(params)
    e = build_exec(bus)
    try:
        e.bus.enable_all(True, allow_restart=True)
        e.kt = {n: {"kt_eff": p[0], "i_fric": p[1], "i_bias": p[2], "rms": 0.0}
                for n, p in params.items()}
        n0 = len(e.collision.flag_events)
        ok = 0
        for i in range(200):
            q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            q[3] = 2.0 if i % 2 == 0 else -2.0
            e.bus.q = {n: 0.0 for n in NAMES}
            e.bus.target = {n: 0.0 for n in NAMES}
            e.bus.v = {n: 0.0 for n in NAMES}
            segs = e.motion._plan_move([0] * 6, q, f"22.3 无注入 {i}")
            e.motion.begin_command()
            try:
                e.motion.execute(segs, preview=False, clear_stop=False)
                ok += 1
            except MotionError:
                pass
        n1 = len(e.collision.flag_events)
        check("22.3 不加时连续 200 次运动标记 0 次",
              n1 - n0 == 0 and ok >= 198,
              f"标记 {n1 - n0} 次 / 完成 {ok}/200")
    finally:
        close_exec(e)


def test_collision_exception_and_speed():
    """残差计算抛异常不影响运动；开/关监视器回读节拍差 <10%。"""
    params = {"J2": (1.91, 0.10, 0.0), "J3": (1.35, 0.08, 0.0),
              "J4": (0.22, 0.03, 0.0), "J5": (0.22, 0.03, 0.0)}
    bus = PhysicsBus(params)
    e = build_exec(bus)
    try:
        e.bus.enable_all(True, allow_restart=True)
        e.kt = {n: {"kt_eff": p[0], "i_fric": p[1], "i_bias": p[2], "rms": 0.0}
                for n, p in params.items()}

        def boom(_n):
            raise RuntimeError("故意让残差计算抛异常")
        e.collision._params = boom
        segs = e.motion._plan_move([0] * 6, [0, 10, 0, 0, 0, 0], "22.3 异常")
        e.motion.begin_command()
        e.motion.execute(segs, preview=False, clear_stop=False)
        bad = e.motion.status.get("phase") not in ("arrived", "done")
        check("22.3 残差计算抛异常不影响运动", not bad,
              f"phase={e.motion.status.get('phase')}，monitor error={e.collision.status.get('error', '')[:40]}")
        # 对比回读间隔：关 vs 开
        def measure(enabled):
            b = PhysicsBus(params)
            x = build_exec(b)
            try:
                x.bus.enable_all(True, allow_restart=True)
                x.collision.enabled = enabled
                b.read_times.clear()
                segs = x.motion._plan_move([0] * 6, [0, 20, 0, 0, 0, 0],
                                           f"22.3 节拍 {enabled}")
                x.motion.begin_command()
                x.motion.execute(segs, preview=False, clear_stop=False)
                ts = b.read_times[:]
                dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
                dts = [d for d in dts if d > 0]
                dts.sort()
                return dts[len(dts) // 2] if dts else 0.0
            finally:
                close_exec(x)
        off = measure(False)
        on = measure(True)
        diff = abs(on - off) / max(off, 1e-6) * 100
        check("22.3 开监视器不拖慢回读节拍（中位间隔差 <10%）", diff < 10.0,
              f"关 {off * 1000:.1f} ms / 开 {on * 1000:.1f} ms（差 {diff:.1f}%）")
    finally:
        close_exec(e)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="22.2 扫描用 30°/s（默认 3°/s 完整采集）")
    a = ap.parse_args(argv)
    print("== 22.2 拟合函数（合成已知参数） ==")
    test_fit_synthetic()
    print("\n== 22.2 三条真机扫描路径预检 ==")
    test_scan_paths()
    print("\n== 22.2 扫描采集完整跑一遍（mock） ==")
    speed = 30.0 if a.quick else 3.0
    for j in ("J2", "J3", "J4"):
        test_scan(j, speed)
    print("\n== 22.4 关键点录放 ==")
    test_program_roundtrip()
    print("\n== 22.3 电流残差（第一阶段只记录） ==")
    test_collision_flag()
    test_collision_no_false()
    test_collision_exception_and_speed()
    print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())
