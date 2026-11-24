#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无硬件模拟：和 JointBus 同接口，按梯形速度把关节推向目标。

用于网页/运动层离线测试（--mock），不连接 ECB。

20.0：MockJointBus 带故障注入（仅 mock）：
    silent        某一颗完全不应答
    drop_prob     指定指令回包按概率丢（R_POSITION/R_POWER/R_MODE/...）
    delay_rounds  指定指令的回包延迟一轮，下一轮恢复
    error_bits    某一颗报警位非零
    limits        限流回读值（A）
    force_mode    模式回读值（默认 6）
"""
from __future__ import annotations

import math
import random
import threading
import time

from .joints import JointState, load_joint_configs
from .sca import (MODE_PROFILE_POSITION, R_CURRENT_LIMIT, R_ERROR, R_MODE,
                  R_POSITION, R_POWER)


class VirtualJointBus:
    """离线预演用的假总线：六轴固定在 q=0，只读不算写。

    电机没连/没使能时，网页显示零位姿态、IK/围栏照常预演；执行会被执行器拒绝。
    """

    is_mock = False
    is_virtual = True

    def __init__(self, cfg=None):
        self.cfg = cfg or load_joint_configs()
        self.order = list(self.cfg)
        self.startup_done = False
        self.q = {n: 0.0 for n in self.order}

    def read_all(self, full: bool = False):
        out = {}
        for n in self.order:
            c = self.cfg[n]
            out[n] = JointState(name=n, id=c.id, enabled=False,
                                pos_turns=0.0, out_deg=0.0, target_deg=0.0,
                                error=0, error_text="",
                                current_a=None, voltage_v=None, temp_c=None,
                                mode=None, current_limit_a=None,
                                speed_limit_rpm=None,
                                pos_limit_lo_r=c.eeprom_limits_deg[0] / 10.0,
                                pos_limit_hi_r=c.eeprom_limits_deg[1] / 10.0)
        return out

    def set_targets_q(self, targets):
        return {n: True for n in targets}

    def set_profiles_q(self, vel, acc=None):
        return None

    def set_stream_mode(self, mode: int) -> bool:
        self.force_mode = {n: mode for n in self.order}
        return True

    def hold_all(self):
        return {"ok": True, "held": dict(self.q),
                "unconfirmed": [], "unreadable": []}

    def stats(self):
        return {"virtual": True}


class MockJointBus:
    is_mock = True

    def __init__(self, initial_deg=None):
        self.cfg = load_joint_configs()
        self.order = list(self.cfg)
        # 35.2 第 3 条：规划速度残留模型（默认关，只给自检用）
        self.residual_model = False
        self.residual = {}
        self._plan_vel = {}
        self._plan_acc = {}
        self._prev_dir = {}
        self._residual_log = []
        self.q = {n: float((initial_deg or {}).get(n, 0.0)) for n in self.order}
        self.target = dict(self.q)
        self._targets = dict(self.q)
        self.v = {n: 0.0 for n in self.order}
        self.vmax = {n: self.cfg[n].motion_speed_deg_s for n in self.order}
        self.amax = {n: self.cfg[n].motion_speed_deg_s / self.cfg[n].motion_accel_time_s
                     for n in self.order}
        self.enabled = True
        self.connected = True
        self.startup_done = True
        self.discovered = {self.cfg[n].id: int(self.cfg[n].sn4, 16) for n in self.order}
        self.startup_report = {"ok": True, "mock": True}
        self._lock = threading.RLock()
        self._t = time.monotonic()
        self._errors = {n: 0 for n in self.order}
        self.writes = 0
        self.off = set()          # 18.3：被单独失能的关节
        # 20.0 故障注入（仅 mock）
        self.silent: set[str] = set()
        self.drop_prob: dict[int, float] = {}
        self.delay_rounds: dict[tuple[str, int], int] = {}
        self.error_bits: dict[str, int] = {}
        self.force_mode: dict[str, int] = {}
        self.current_override: dict[str, float] = {}
        # 22.2 自检：按已知 Kt/摩擦/零偏生成电流。fn(name, q_dict, v_dict) -> A
        self.current_fn = None
        self.current_limit_readback: dict[str, float | None] = {
            n: 3.0 for n in self.order}
        # 30.1：0x53 满量程（mock 固定 33 A，和真机口径一致：0x94 电流按满量程换）
        self.current_full_scale_readback: dict[str, float | None] = {
            n: 33.0 for n in self.order}
        self.mock_fault_log: list[str] = []
        # A4/A6：给按颗操作的代码（clear_errors / check_alarm_latch）一个电机代理
        self.motors = {n: MockMotor(self, n) for n in self.order}
        self.mode_writes = 0
        self.mode_set_log: list[tuple[str, int]] = []
        self.disable_calls = 0
        self.enable_calls = 0
        self.drift: dict[str, float] = {}
        self.stuck: set[str] = set()
        self.lag_offset: dict[str, float] = {}
        self.slow: dict[str, float] = {}
        # 26.7 7：写目标丢包注入 + 统计
        self.drop_write_prob = 0.0
        self.write_dropped = 0
        self.write_resent = 0
        # 30.5 第 3 条：只在“使能过、之后又失能”时才置锁存
        self._ever_enabled = True if self.enabled else False
        # 27.2 第 3 条：失能后这一次上电不再使能（mock 不写真实配置文件）
        self.disabled_latch = False
        self.disabled_latch_at = None
        self.disabled_latch_reason = ""
        # 27.2 第 6 条自检注入：使能后这颗“带着旧目标/旧模式使劲”
        self.enable_fault_mode: dict[str, int] = {}
        self.enable_fault_drift: dict[str, float] = {}
        self.enable_fault_current: dict[str, float] = {}
        self.enable_call_log: list[str] = []
        self.enable_refused_log: list[str] = []
        # 29.6：mock 下没有真抓包，用它自己的“实际发出去的写命令”日志代替
        self.write_log_path = None
        self.write_log_count = 0
        self.event_hook = None

    def _emit_event(self, joint: str, cmd: int, value, src: str = "executor"):
        """29.1：执行器“记了”这条写命令（会话事件）。"""
        hook = getattr(self, "event_hook", None)
        if hook is not None:
            try:
                hook(joint, cmd, value)
            except Exception:  # noqa: BLE001
                pass

    def _emit_write(self, joint: str, cmd: int, value, src: str = "executor"):
        """mock 电机真正收下这条写命令：记“实际发出去”的日志 + 会话事件。"""
        self.log_write(joint, cmd, value, src=src)
        self._emit_event(joint, cmd, value, src=src)

    def log_write(self, joint: str, cmd: int, value, src: str = "executor"):
        """29.6：记一条“实际发出去的写命令”，供 pcap_check 对账。"""
        self.write_log_count += 1
        if not self.write_log_path:
            return
        try:
            import json as _json
            rec = {"mono": round(time.monotonic(), 4), "wall": time.strftime(
                "%Y-%m-%dT%H:%M:%S"), "joint": joint,
                "cmd": int(cmd), "value": value, "src": src}
            with open(self.write_log_path, "a", encoding="utf-8") as fh:
                fh.write(_json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:  # noqa: BLE001
            pass

    def latch_disabled(self, reason: str = ""):
        if not self._ever_enabled:
            return False
        if not self.disabled_latch:
            self.disabled_latch = True
            self.disabled_latch_at = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.disabled_latch_reason = str(reason or "")
        return self.disabled_latch

    def latch_status(self):
        return {"disabled_latch": bool(self.disabled_latch),
                "at": self.disabled_latch_at,
                "reason": self.disabled_latch_reason,
                "msg": ("已经失能过：这一次上电里不再使能任何关节，请拉空开断电后重上电"
                        if self.disabled_latch else "")}

    def clear_latch_on_power_cycle(self):
        self.disabled_latch = False
        self.disabled_latch_at = None
        self.disabled_latch_reason = ""
        self._ever_enabled = False

    # ---------------- 生命周期 ----------------
    def preflight(self):
        self.connected = True
        self.discovered = {self.cfg[n].id: int(self.cfg[n].sn4, 16) for n in self.order}
        return {"ok": True, "mock": True, "handshake": True,
                "scan": self.discovered, "mismatch": []}

    def enable_all(self, confirmed_supported: bool = False,
                   power_cycle_confirmed: bool = False,
                   allow_restart: bool = False):
        from .joints import StartupRefused, startup_enable_issues
        self.enable_call_log.append("enable_all")
        if not confirmed_supported:
            raise StartupRefused("没有确认臂已支撑，拒绝使能")
        if self.startup_done and self.enabled and not allow_restart:
            raise StartupRefused(
                "六颗已经在使能保持里（startup_done=True），不要重跑启动序列："
                "启动序列会把电机里存着的旧目标当阶跃，可能报 0x0200。"
                "要重新启动，请先正常停机、拉空开断电、再上电")
        if self.disabled_latch:
            if not power_cycle_confirmed:
                self.enable_refused_log.append("latch")
                raise StartupRefused(
                    "已经失能过（" + (self.disabled_latch_reason or "原因没记")
                    + "）：27.2 第 3 条规定这一次上电里不再使能任何关节。"
                    "请拉空开断电、重新上电，再勾选“已经断电重上电”后启动")
            self.clear_latch_on_power_cycle()
        # 27.2 第 6 条：逐颗模拟“使能 0.3 s 后读模式/位置/电流/ρ”的核对
        for n in reversed(self.order):
            mode = int(self.enable_fault_mode.get(n, 1))
            cur = float(self.enable_fault_current.get(n, 0.0))
            drift = float(self.enable_fault_drift.get(n, 0.0))
            pos = self.q[n] / 10.0
            pos_before = pos - (drift * 0.3 / 10.0)
            issues = startup_enable_issues(
                self.cfg[n], n, mode=mode, cur=cur, pos=pos,
                pos_before=pos_before,
                limit_a=self.current_limit_readback.get(n))
            if issues:
                self.enabled = True
                self.off.add(n)
                self.startup_done = False
                self.enabled = False
                self.latch_disabled(f"{n} 启动核对不合格")
                raise StartupRefused(
                    f"{n} 启动核对不合格，已把这颗失能并停下："
                    + "；".join(issues))
        self.enabled = True
        self.startup_done = True
        self._ever_enabled = True
        for n in self.order:
            self.current_limit_readback.setdefault(n, 3.0)
        enabled = [{"name": n, "out_deg": round(self.q[n], 3),
                    "pos_turns": round(self.q[n] / 10.0, 5),
                    "current_limit_a": self.current_limit_readback.get(n),
                    "speed_limit_rpm": 360.0,
                    "pos_limits_deg": list(self.cfg[n].eeprom_limits_deg),
                    "voltage_v": 24.0, "profile_deg_s": 20.0} for n in reversed(self.order)]
        return {"ok": True, "mock": True, "enabled": enabled,
                "zero_torque": [{"name": n, "mode": 1, "current_a": 0.0}
                                for n in reversed(self.order)],
                "chain": {"max_v": 24.0, "voltages": {n: 24.0 for n in self.order}},
                "warnings": []}

    def disable_all(self, confirmed: bool = False, reason: str = ""):
        if not confirmed:
            from .joints import StartupRefused
            raise StartupRefused("全部失能要二次确认")
        self.enabled = False
        self.startup_done = False
        self.latch_disabled(reason or "全部失能")
        return True

    def try_takeover(self):
        """30.6 第 4 条（mock）：六颗使能着、模式 6、报警 0 → 接管，只写目标=当前位。"""
        import time as _time
        if self.disabled_latch:
            return {"took_over": False, "enabled_count": 0, "blocked": "latch",
                    "msg": "已经失能过：请拉空开断电后重上电"}
        enabled = [n for n in self.order if n not in self.off and self.enabled]
        if not enabled:
            return {"took_over": False, "enabled_count": 0,
                    "msg": "六颗都是失能状态：等正常启动序列"}
        bad = []
        for n in self.order:
            mode = self.force_mode.get(n, MODE_PROFILE_POSITION)
            err = self.error_bits.get(n, self._errors.get(n, 0))
            if mode != MODE_PROFILE_POSITION:
                bad.append(f"{n} 模式={mode}")
            if err:
                bad.append(f"{n} 报警 0x{int(err):04X}")
            if n in self.silent:
                bad.append(f"{n} 读不到")
        if bad:
            return {"took_over": False, "enabled_count": len(enabled), "bad": bad,
                    "msg": ("臂还使能着，但不能接管：" + "，".join(bad)
                            + "。先扶住臂（或确认臂在搁架上），再拉空开断电")}
        before = {n: self.q[n] for n in self.order}
        # 只发 0x0A：目标=当前位置（mock 的 set_target_turns 记 0x0A，不碰模式/使能）
        for n in self.order:
            self.motors[n].set_target_turns(self.cfg[n].turns(before[n]))
        _time.sleep(0.05)
        moved = {n: round(abs(self.q[n] - before[n]), 3) for n in self.order
                 if abs(self.q[n] - before[n]) > 0.2}
        if moved:
            return {"took_over": False, "enabled_count": len(enabled),
                    "moved": moved,
                    "msg": f"接管时这些关节位置变了 {moved}°：请先扶住臂再拉空开"}
        self.startup_done = True
        self._ever_enabled = True
        positions = {n: round(before[n], 3) for n in self.order}
        for n in self.order:
            self._targets[n] = before[n]
        self.startup_report = {"ok": True, "took_over": True,
                               "positions": positions,
                               "msg": "已接管（本来就在使能保持里）：没有重跑启动序列"}
        return {"took_over": True, "enabled_count": len(enabled),
                "positions": positions,
                "msg": "已接管（本来就在使能保持里）：没有重跑启动序列，原地保持"}

    def close(self):
        pass

    # ---------------- 仿真 ----------------
    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self._t))
        self._t = now
        if not self.enabled:
            return
        for n in self.order:
            if n in self.stuck:
                self.v[n] = 0.0
                continue
            if n in self.drift:
                # A3b 自检：这颗不听目标，按给定 °/s 往一个方向走
                self.q[n] += float(self.drift[n]) * dt
                self.v[n] = float(self.drift[n])
                continue
            if n in getattr(self, "residual", {}):
                res = self.residual[n]
                v0 = float(res["v"])
                a = float(res["a"])
                if v0 <= a * dt:
                    move_res = v0 * v0 / (2.0 * a) if a > 0 else 0.0
                    res["v"] = 0.0
                else:
                    move_res = v0 * dt - 0.5 * a * dt * dt
                    res["v"] = v0 - a * dt
                self.q[n] += float(res["dir"]) * move_res   # 残留速度沿上一段方向继续走
                res["moved"] += move_res
                self.v[n] = 0.0
                if res["v"] <= 0.0:
                    del self.residual[n]
                continue
            err = self.target[n] - self.q[n]
            dist = abs(err)
            # 允许的最高速度同时受减速距离限制
            v_allow = math.sqrt(max(0.0, 2.0 * self.amax[n] * dist))
            cap = min(self.vmax[n], self.slow.get(n, self.vmax[n]))
            desired = min(cap, v_allow) * (1.0 if err >= 0 else -1.0)
            dv = desired - self.v[n]
            step = self.amax[n] * dt
            self.v[n] += max(-step, min(step, dv))
            move = self.v[n] * dt
            if abs(move) >= dist:
                self.q[n] = self.target[n]
                self.v[n] = 0.0
            else:
                self.q[n] += move
        # 26.2 自检：让这颗固定落后目标 offset 度，但仍在“追”
        for n, off in self.lag_offset.items():
            if n in self.stuck or n in self.drift:
                continue
            err = self.target[n] - self.q[n]
            sgn = 1.0 if err >= 0 else -1.0
            self.q[n] = self.target[n] - sgn * abs(float(off))
            self.v[n] = sgn * max(1.0, abs(self.v[n]))

    # ---------------- 20.0 故障注入 ----------------
    def _lost(self, name: str, cmd: int) -> bool:
        """按概率丢包；delay_rounds 让这一轮丢、下一轮恢复。"""
        if name in self.silent:
            return True
        key = (name, cmd)
        if self.delay_rounds.get(key, 0) > 0:
            self.delay_rounds[key] -= 1
            self._fault_log(f"delay {name} cmd=0x{cmd:02X}")
            return True
        p = float(self.drop_prob.get(cmd, 0.0))
        if p > 0 and random.random() < p:
            self._fault_log(f"drop {name} cmd=0x{cmd:02X}")
            return True
        return False

    def _fault_log(self, msg: str):
        # 长跑时每次丢包都记一条会无限涨内存；只留最近 2000 条
        self.mock_fault_log.append(msg)
        if len(self.mock_fault_log) > 2000:
            del self.mock_fault_log[:-2000]

    def set_faults(self, silent=None, drop=None, delay=None, error_bits=None,
                   limits=None, mode=None, currents=None, drift=None,
                   stuck=None, lag_offset=None, slow=None):
        """drop: {"position"/"power"/"mode"/"error"/"current": 概率}；
        delay: {"J4": 1} 或 1（所有关节位置延迟轮数）；
        error_bits: {关节: 位}；limits: {关节: A}；mode: {关节: 模式}。"""
        if silent is not None:
            self.silent = {str(x).upper() for x in silent}
        if drop is not None:
            names = {"position": R_POSITION, "power": R_POWER,
                     "mode": R_MODE, "error": R_ERROR, "current": R_CURRENT_LIMIT}
            self.drop_prob = {names[k]: float(v) for k, v in drop.items() if k in names}
        if delay is not None:
            if isinstance(delay, dict):
                self.delay_rounds = {(str(k).upper(), R_POSITION): int(v)
                                     for k, v in delay.items()}
            else:
                self.delay_rounds = {(n, R_POSITION): int(delay) for n in self.order}
        if error_bits is not None:
            self.error_bits = {str(k).upper(): int(v) for k, v in error_bits.items()}
        if limits is not None:
            for k, v in limits.items():
                self.current_limit_readback[str(k).upper()] = None if v is None else float(v)
        if mode is not None:
            self.force_mode = {str(k).upper(): int(v) for k, v in mode.items()}
        if currents is not None:
            self.current_override = {str(k).upper(): float(v) for k, v in currents.items()}
        if drift is not None:
            self.drift = {str(k).upper(): float(v) for k, v in drift.items()}
        if stuck is not None:
            self.stuck = {str(x).upper() for x in stuck}
        if lag_offset is not None:
            self.lag_offset = {str(k).upper(): float(v) for k, v in lag_offset.items()}
        if slow is not None:
            self.slow = {str(k).upper(): float(v) for k, v in slow.items()}

    def clear_faults(self):
        self.silent = set()
        self.drop_prob = {}
        self.delay_rounds = {}
        self.error_bits = {}
        self.force_mode = {}
        self.current_override = {}
        self.drift = {}
        self.stuck = set()
        self.lag_offset = {}
        self.slow = {}
        self.mock_fault_log = []

    def mock_faults(self):
        return {"silent": sorted(self.silent),
                "drop_prob": {f"0x{k:02X}": v for k, v in self.drop_prob.items()},
                "delay_rounds": {f"{k[0]}:0x{k[1]:02X}": v
                                 for k, v in self.delay_rounds.items()},
                "error_bits": dict(self.error_bits),
                "force_mode": dict(self.force_mode),
                "limits": dict(self.current_limit_readback),
                "drift": dict(self.drift),
                "stuck": sorted(self.stuck),
                "lag_offset": dict(self.lag_offset),
                "slow": dict(self.slow)}

    # ---------------- 读 ----------------
    def read_all(self, full: bool = False):
        with self._lock:
            self._tick()
            out = {}
            for n in self.order:
                c = self.cfg[n]
                err = self.error_bits.get(n, self._errors.get(n, 0))
                pos_lost = self._lost(n, R_POSITION)
                power_lost = self._lost(n, R_POWER)
                mode_lost = self._lost(n, R_MODE)
                error_lost = self._lost(n, R_ERROR)
                enabled = (self.enabled and n not in self.off)
                cur_a = self.current_override.get(n)
                if cur_a is None and self.current_fn is not None:
                    try:
                        cur_a = float(self.current_fn(n, dict(self.q), dict(self.v)))
                    except Exception:  # noqa: BLE001
                        cur_a = 0.0
                if cur_a is None:
                    cur_a = 0.0
                full_scale = 33.0
                out[n] = JointState(
                    name=n, id=c.id,
                    enabled=None if power_lost else enabled,
                    pos_turns=None if pos_lost else c.turns(self.q[n]),
                    out_deg=None if pos_lost else self.q[n],
                    target_deg=self.target[n],
                    error=None if error_lost else err,
                    error_text="" if error_lost else ("报警" if err else "正常"),
                    current_a=(None if full and self._lost(n, R_CURRENT_LIMIT)
                               else cur_a),
                    current_raw=int(round(cur_a / full_scale * (1 << 24))),
                    current_full_scale_a=full_scale,
                    voltage_v=24.0, temp_c=30.0,
                    mode=None if mode_lost else self.force_mode.get(
                        n, MODE_PROFILE_POSITION),
                    current_limit_a=self.current_limit_readback.get(n),
                    speed_limit_rpm=360.0,
                    pos_limit_lo_r=c.eeprom_limits_deg[0] / 10.0,
                    pos_limit_hi_r=c.eeprom_limits_deg[1] / 10.0)
            return out

    # ---------------- 写 ----------------
    def read_cvp_all(self, timeout=None, retries=None):
        with self._lock:
            states = self.read_all(full=True)
            out = {}
            for n in self.order:
                st = states[n]
                lim = self.current_limit_readback.get(n) or self.cfg[n].required_current_limit_a
                out[n] = {"ok": st.out_deg is not None,
                          "pos_deg": st.out_deg,
                          "vel_deg_s": self.v[n],
                          "current_a": st.current_a,
                          "limit_a": lim,
                          "rho": None if (not lim or st.current_a is None)
                          else abs(st.current_a) / lim}
            return out

    def read_errors(self, timeout=None, retries=None):
        with self._lock:
            return {n: self.error_bits.get(n, self._errors.get(n, 0))
                    for n in self.order}

    def set_targets_q(self, targets, confirm: bool = True):
        dropped = []
        with self._lock:
            self.writes += 1
            self._tick()
            for n, q in targets.items():
                turns = round(float(q) / 10.0, 6)
                # 执行器的记录（发之前就记：丢了以后要能对出“记了没发”）
                self._emit_event(n, 0x0A, turns)
                if self.drop_write_prob and random.random() < self.drop_write_prob:
                    dropped.append(n)
                    self.write_dropped += 1
                    continue
                self.log_write(n, 0x0A, turns)
                lo, hi = self.cfg[n].soft_limits_deg
                new_t = max(lo, min(hi, float(q)))
                old_t = self.target[n]
                dir_new = 1.0 if new_t >= old_t else -1.0
                if (getattr(self, "residual_model", False) and abs(new_t - old_t) > 1e-6
                        and self._prev_dir.get(n) is not None
                        and dir_new != self._prev_dir[n]):
                    v_prev = float(self._plan_vel.get(n) or 0.0)
                    a_new = float(self._plan_acc.get(n) or 40.0)
                    if v_prev > 0.2:
                        rev = v_prev * v_prev / (2.0 * a_new)
                        self.residual[n] = {"v": v_prev, "dir": self._prev_dir[n],
                                            "a": a_new, "moved": 0.0, "rev": rev}
                        self._residual_log.append(
                            {"joint": n, "predicted_rev_deg": round(rev, 3),
                             "v_prev": round(v_prev, 3), "a_new": round(a_new, 3)})
                self.target[n] = new_t
                self._targets[n] = self.target[n]
                self._prev_dir[n] = dir_new
        if confirm and dropped:
            # 行为确认：丢了的这颗速度/位置不会朝目标走，重发一次
            time.sleep(0.06)
            resent = []
            with self._lock:
                for n in dropped:
                    q = float(targets[n])
                    d = q - self.q[n]
                    u = (1.0 if d > 0 else -1.0) * self.v[n]
                    if abs(d) > 0.8 and u < 0.5:
                        self.target[n] = q
                        self._targets[n] = q
                        resent.append(n)
                        self.write_resent += 1
            self.last_resend = resent
        else:
            self.last_resend = []
        return {n: True for n in targets}

    def set_profiles_q(self, vel, acc=None, disp=None):
        self.writes += 1
        acc = acc or {}
        for n in self.order:
            if n in vel:
                self.vmax[n] = max(0.2, abs(float(vel[n])))
                a = abs(float(acc.get(n, vel[n] / max(self.cfg[n].motion_accel_time_s, 0.05))))
                if disp is not None and abs(float(disp.get(n, 0.0))) <= 0.8:
                    floor = self.cfg[n].motion_speed_deg_s / max(
                        self.cfg[n].motion_accel_time_s, 0.05)
                    a = max(a, floor)
                self.amax[n] = max(0.2, a)
                self._plan_vel[n] = abs(float(vel[n]))
                self._plan_acc[n] = a

    def hold_all(self):
        """20.1：和真机 JointBus 同返回结构；silent 关节算“读不到”。"""
        with self._lock:
            self.writes += 1
            self._tick()
            held = {}
            kept = {}
            unconfirmed = []
            unreadable = []
            for n in self.order:
                if n in self.silent:
                    unreadable.append(n)
                    unconfirmed.append({"name": n, "reason": "读不到位置"})
                    continue
                # 35.2 第 1 条：到位窗口内的关节不改写目标、不清速度
                tgt_now = self._targets.get(n)
                if tgt_now is not None and abs(self.q[n] - float(tgt_now)) <= 0.8:
                    kept[n] = float(tgt_now)
                    held[n] = self.q[n]
                    continue
                self.target[n] = self.q[n]
                self.v[n] = 0.0
                self._targets[n] = self.q[n]
                held[n] = self.q[n]
                if not self.enabled:
                    unconfirmed.append({"name": n, "reason": "读到了但 power=0"})
            for name, why in self.verify_ready():
                if not any(x["name"] == name for x in unconfirmed):
                    unconfirmed.append({"name": name, "reason": why})
            return {"ok": not unconfirmed, "held": held, "kept": kept,
                    "unconfirmed": unconfirmed, "unreadable": unreadable}

    # ---------------- 18.3 单颗失能 / 重新使能 ----------------
    def read_positions_turns(self):
        with self._lock:
            self._tick()
            out = {}
            for n in self.order:
                if n in self.silent:
                    continue
                out[n] = self.cfg[n].turns(self.q[n])
            return out

    def disable_one(self, name):
        with self._lock:
            self.off.add(name)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            self.disable_calls += 1
            self.latch_disabled(f"{name} 失能")
            return True

    def enable_one_hold(self, name, pos_turns):
        with self._lock:
            if name in self.silent:
                return False
            self.off.discard(name)
            self.q[name] = self.cfg[name].out_deg(pos_turns)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            return True

    def current_limit_a(self, name):
        return self.current_limit_readback.get(name)

    def current_full_scale_a(self, name):
        return 33.0

    def current_raw_counts(self, name):
        return (0, 0)

    def verify_ready(self, names=None):
        names = list(names or self.order)
        if not self.enabled:
            return [(n, "读到了但 power=0") for n in names]

        def read12(n, cmd):
            # 20.2：和真机 JointBus 一样，单颗最多 12 次短重试
            for _ in range(12):
                if not self._lost(n, cmd):
                    return 1 if cmd == R_POWER else self.force_mode.get(
                        n, MODE_PROFILE_POSITION)
            return None

        bad = []
        for n in names:
            if n in self.silent:
                bad.append((n, "读不到电源(0x2B)和模式(0x55)"))
                continue
            p = read12(n, R_POWER)
            m = read12(n, R_MODE)
            if p is None and m is None:
                bad.append((n, "读不到电源(0x2B)和模式(0x55)"))
            elif p is None:
                bad.append((n, "读不到电源(0x2B)"))
            elif m is None:
                bad.append((n, "读不到模式(0x55)"))
            elif p != 1 or m != MODE_PROFILE_POSITION:
                bad.append((n, f"读到了但 power={p},mode={m}"))
        return bad

    def stats(self):
        return {"tx": 0, "rx": 0, "mock": True, "faults": self.mock_faults()}


class MockMotor:
    """MockJointBus 的单颗代理：只给 clear_errors / check_alarm_latch 用。"""

    def __init__(self, bus, name):
        self.bus = bus
        self.name = name
        self.mid = bus.cfg[name].id

    def power(self, timeout=None, retries=None):
        b = self.bus
        if b._lost(self.name, R_POWER):
            return None
        return 1 if (b.enabled and self.name not in b.off) else 0

    def error(self, timeout=None, retries=None):
        if self.bus._lost(self.name, R_ERROR):
            return None
        return self.bus.error_bits.get(self.name, self.bus._errors.get(self.name, 0))

    def clear_error(self):
        self.bus._errors[self.name] = 0
        self.bus.error_bits.pop(self.name, None)
        return True

    def mode(self, timeout=None, retries=None):
        if self.bus._lost(self.name, R_MODE):
            return None
        return self.bus.force_mode.get(self.name, MODE_PROFILE_POSITION)

    def set_mode(self, mode):
        self.bus._emit_write(self.name, 0x07, int(mode))
        self.bus.mode_writes += 1
        self.bus.mode_set_log.append((self.name, int(mode)))
        return True

    def set_current_a(self, amp):
        self.bus._emit_write(self.name, 0x08, float(amp))
        return True

    def enable(self):
        self.bus.enable_calls += 1
        self.bus._emit_write(self.name, 0x2A, 1)
        return True

    def disable(self):
        self.bus.disable_calls += 1
        self.bus.off.add(self.name)
        self.bus._emit_write(self.name, 0x2A, 0)
        return True

    def pos_turns(self, timeout=None, retries=None):
        if self.bus._lost(self.name, R_POSITION):
            return None
        return self.bus.cfg[self.name].turns(self.bus.q[self.name])

    def set_target_turns(self, turns):
        self.bus._emit_write(self.name, 0x0A, float(turns))
        return True

    def set_profile_out(self, *a, **kw):
        return True
