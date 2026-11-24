#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节层：配置文件、ID↔SN 核对、使能、参数读写、单位换算。

对外角度一律是“URDF 正方向”的输出端角度：
    out_deg = sign × 电机位置(电机侧圈数) × 10°/圈 + offset
    target_turns = sign × (q_target − offset) / 10
"""
from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field

from . import config as cfgmod
from .sca import (DEG_PER_MOTOR_TURN, GEAR, IQ8, IQ10, IQ24,
                  MODE_CURRENT, MODE_POSITION, MODE_PROFILE_POSITION,
                  R_CURRENT, R_CURRENT_LIMIT, R_CURRENT_MAX, R_ERROR,
                  R_MODE, R_POSITION, R_POS_LIMIT_H, R_POS_LIMIT_L, R_POWER,
                  R_TEMP_MOTOR, R_VEL_LIMIT, R_VOLTAGE,
                  ScaMotor, VELOCITY_FULL_RPM, describe_error)
from .transport import EcbTransport


def startup_enable_issues(cfg, name: str, mode=None, cur=None, pos=None,
                         pos_before=None, limit_a=None) -> list[str]:
    """27.2 第 6 条：启动时每颗使能、进入零扭矩以后 0.3 s 内的核对项。

    - 模式不是电流模式（1）→ 失败
    - 位置比使能前变了 >0.5° → 失败（电机像带着旧目标/旧模式使劲）
    - 零扭矩时电流 >0.25 A → 失败
    - ρ = |I|/限流 >0.1 → 失败
    """
    bad: list[str] = []
    if mode is not None and mode != MODE_CURRENT:
        bad.append(f"模式={mode}，不是电流模式（零扭矩）")
    if cur is not None and abs(cur) > 0.25:
        bad.append(f"零扭矩时电流 {cur:.2f} A > 0.25 A")
    if pos is not None and pos_before is not None:
        try:
            moved = abs(cfg.out_deg(pos) - cfg.out_deg(pos_before))
        except Exception:  # noqa: BLE001
            moved = None
        if moved is not None and moved > 0.5:
            bad.append(f"使能后 0.3 s 内位置变了 {moved:.2f}° > 0.5°"
                       "（电机像是带着旧目标/旧模式使劲）")
    if cur is not None and limit_a:
        rho = abs(cur) / float(limit_a)
        if rho > 0.1:
            bad.append(f"零扭矩时 ρ={rho:.2f} > 0.1"
                       f"（电流 {cur:.2f} A / 限流 {float(limit_a):.2f} A）")
    return bad


@dataclass
class JointConfig:
    name: str
    id: int
    sn4: str
    model: str
    sign: int
    offset_deg: float
    eeprom_limits_deg: tuple[float, float]
    soft_limits_deg: tuple[float, float]
    safe_limits_deg: tuple[float, float]
    eeprom_speed_limit_out_deg_s: float
    motion_speed_deg_s: float
    motion_accel_time_s: float
    required_current_limit_a: float | None
    current_check_max_a: float | None
    j2_bracket: str = "PLA"
    zero_date: str = ""
    sign_checked: bool = False
    note: str = ""

    @property
    def gear(self) -> float:
        return GEAR

    def out_deg(self, turns: float) -> float:
        return self.sign * turns * DEG_PER_MOTOR_TURN + self.offset_deg

    def turns(self, out_deg: float) -> float:
        return self.sign * (out_deg - self.offset_deg) / DEG_PER_MOTOR_TURN


@dataclass
class JointState:
    name: str
    id: int
    enabled: bool | None = None
    out_deg: float | None = None
    pos_turns: float | None = None
    target_deg: float | None = None
    error: int | None = None
    error_text: str = ""
    current_a: float | None = None
    velocity_deg_s: float | None = None
    current_raw: int | None = None
    current_full_scale_a: float | None = None
    voltage_v: float | None = None
    temp_c: float | None = None
    mode: int | None = None
    current_limit_a: float | None = None
    speed_limit_rpm: float | None = None
    pos_limit_lo_r: float | None = None
    pos_limit_hi_r: float | None = None
    # 24 节 A3：读数时间戳 / 合理性
    pos_read_at: float | None = None      # unix 时间，读到这一刻
    pos_age_s: float | None = None        # 相对 state() 计算的时长由执行器填
    pos_invalid: bool = False             # 合理性检查没过：不许拿来当保持目标/规划起点
    pos_raw_deg: float | None = None      # 未通过时保留原始换算值，仅供显示/日志
    raw_pos_hex: str = ""                 # 原始回包十六进制
    mode_read_at: float | None = None
    mode_age_s: float | None = None

    def as_dict(self) -> dict:
        def r(v, n=3):
            return None if v is None else round(float(v), n)
        return {
            "name": self.name, "id": self.id, "enabled": self.enabled,
            "out_deg": r(self.out_deg), "pos_turns": r(self.pos_turns, 5),
            "target_deg": r(self.target_deg), "error": self.error,
            "error_text": self.error_text, "current_a": r(self.current_a),
            "current_raw": self.current_raw,
            "current_full_scale_a": r(self.current_full_scale_a, 2),
            "voltage_v": r(self.voltage_v, 2), "temp_c": r(self.temp_c, 1),
            "mode": self.mode, "current_limit_a": r(self.current_limit_a, 2),
            "speed_limit_rpm": r(self.speed_limit_rpm, 1),
            "pos_limit_deg": (None if self.pos_limit_hi_r is None else
                              [r(self.pos_limit_lo_r * DEG_PER_MOTOR_TURN, 1),
                               r(self.pos_limit_hi_r * DEG_PER_MOTOR_TURN, 1)]),
            "pos_read_at": self.pos_read_at,
            "pos_age_s": (None if self.pos_age_s is None
                          else round(float(self.pos_age_s), 3)),
            "pos_invalid": bool(self.pos_invalid),
            "pos_raw_deg": r(self.pos_raw_deg),
            "raw_pos_hex": self.raw_pos_hex,
            "mode_read_at": self.mode_read_at,
            "mode_age_s": (None if self.mode_age_s is None
                           else round(float(self.mode_age_s), 3)),
        }



def load_joint_configs(path=None) -> dict[str, JointConfig]:
    raw = cfgmod.load_joints()
    out: dict[str, JointConfig] = {}
    for name in raw.get("joint_order", ["J1", "J2", "J3", "J4", "J5", "J6"]):
        j = raw["joints"][name]
        out[name] = JointConfig(
            name=name, id=int(j["id"]), sn4=str(j["sn4"]).upper(),
            model=j.get("model", "?"), sign=int(j.get("sign", 1)),
            offset_deg=float(j.get("offset_deg", 0.0)),
            eeprom_limits_deg=tuple(j["eeprom_limits_deg"]),
            soft_limits_deg=tuple(j["soft_limits_deg"]),
            safe_limits_deg=tuple(j.get("safe_limits_deg", j["soft_limits_deg"])),
            eeprom_speed_limit_out_deg_s=float(j["eeprom_speed_limit_out_deg_s"]),
            motion_speed_deg_s=float(j["motion_speed_deg_s"]),
            motion_accel_time_s=float(j["motion_accel_time_s"]),
            required_current_limit_a=j.get("required_current_limit_a"),
            current_check_max_a=j.get("current_check_max_a"),
            j2_bracket=str(j.get("j2_bracket", "PLA")),
            zero_date=j.get("zero_date", ""),
            sign_checked=bool(j.get("sign_checked", False)),
            note=j.get("note", ""),
        )
    return out


class StartupRefused(RuntimeError):
    pass


class JointBus:
    """真机（ECB）关节总线。"""

    is_mock = False

    def __init__(self, ip: str = "192.168.1.30", port: int = 2000,
                 joint_configs: dict[str, JointConfig] | None = None,
                 debug: bool = False):
        self.cfg = joint_configs or load_joint_configs()
        self.order = list(self.cfg)
        raw_cfg = cfgmod.load_joints()
        self.expected_count = int(raw_cfg.get("expected_count", len(self.order)))
        self.gripper_installed = bool(raw_cfg.get("gripper_installed", False))
        self.link = EcbTransport(ip, port, debug=debug)
        self.motors = {n: ScaMotor(self.link, self.cfg[n].id) for n in self.order}
        self.discovered: dict[int, int] = {}
        self.connected = False
        # 30.5 第 3 条 / 27.2 第 3 条：锁存只在“这次上电里至少有一颗被使能过、
        # 之后又失能了”时才置位。启动在任何一颗使能之前就停下不算。
        self._ever_enabled = False
        # 27.2 第 3 条：失能以后同一次上电里不再使能（唯一例外 18.3 第 7 条）。
        # 锁存写进 config/disabled_latch.json，只有用户确认“已断电重上电”才清。
        self.disabled_latch = False
        self.disabled_latch_at = None
        self.disabled_latch_reason = ""
        try:
            _lat = cfgmod.load_disabled_latch() or {}
        except Exception:  # noqa: BLE001
            _lat = {}
        if _lat.get("disabled_latch"):
            self.disabled_latch = True
            self.disabled_latch_at = _lat.get("at")
            self.disabled_latch_reason = _lat.get("reason") or ""
        self.startup_done = False
        self.startup_report: dict = {}
        self._lock = threading.RLock()
        self._enabled_cache: dict[str, bool | None] = {n: None for n in self.order}
        self._targets: dict[str, float] = {}
        self._last_state: dict[str, JointState] = {}
        self._error_tick = 0
        # 24 节 A3：上一次可信读数（name -> (out_deg, monotonic)）与异常回调
        self._trust: dict[str, tuple[float, float]] = {}
        self._invalid_streak: dict[str, int] = {}
        self._invalid_candidate: dict[str, float] = {}
        self._anomaly_at: dict[str, float] = {}
        self.last_resend: list[str] = []
        self.write_resend_hook = None
        self.anomaly_hook = None      # fn(name, raw_hex, raw_deg, prev_deg, dt)
        self.plausible_rate = 70.0    # °/s
        self.plausible_margin = 2.0   # °
        self.last_read_hz = 0.0
        # 20.5：启动/写限位/校准时真正回读的限流（A），不拿配置值冒充
        self.current_limit_readback: dict[str, float | None] = {}
        # 30.1：0x53 电流满量程（A）。0x94 的电流原始值是 IQ14 乘满量程，
        # 不是乘限流；这里缓存一份，避免每拍都去读 0x53。
        self.current_full_scale_readback: dict[str, float | None] = {}
        # 30.3 e4：测试里关掉“写目标后 0.06 s 重发”，只给测试工具用，不影响正常流程
        self.skip_resend = False

    def reload_config(self, cfg: dict | None = None):
        """重新读 config/joints.json（sign/offset/限位），不重建电机连接。"""
        self.cfg = cfg or load_joint_configs()
        self.order = list(self.cfg)
        self._targets = {}

    # ---------------- 连接 / 启动 ----------------
    def preflight(self) -> dict:
        """握手 + 枚举 + ID↔SN 核对；不使能、不动。"""
        report = {"handshake": False, "scan": {}, "mismatch": [], "ok": False}
        if not self.link.hello(timeout=0.9):
            report["error"] = f"ECB {self.link.ip}:{self.link.port} 没有握手应答"
            self.startup_report = report
            return report
        report["handshake"] = True
        disc = self.link.scan(2.0)
        report["scan"] = {int(k): v for k, v in disc.items()}
        self.discovered = disc
        problems = []
        if len(disc) != self.expected_count:
            problems.append(f"枚举到 {len(disc)} 颗，配置要求 {self.expected_count} 颗"
                            + ("（带夹爪）" if self.gripper_installed else "（不带夹爪）"))
        for name in self.order:
            want = self.cfg[name]
            sn = disc.get(want.id)
            if sn is None:
                problems.append(f"{name}: ID {want.id} 没找到")
                continue
            sn4 = f"{sn:08X}"[-4:]
            if sn4 != want.sn4:
                problems.append(f"{name}: ID {want.id} SN {sn4} ≠ 配置 {want.sn4}")
        report["mismatch"] = problems
        report["ok"] = not problems
        report["ports"] = self.link.stats
        self.connected = report["ok"]
        self.startup_report = report
        return report

    def enable_all(self, confirmed_supported: bool = False,
                   power_cycle_confirmed: bool = False,
                   allow_restart: bool = False) -> dict:
        """16.4 第 1 条启动序列（v2）：

        握手 → 枚举/计数/ID↔SN → 用户确认支撑 → J6→J1 一次一颗
        **零扭矩使能（电流模式，给 0 A）** → 回读限流/速度限制/位置限位/母线电压
        → 成链电压检查 → 全部切 S 曲线保持（目标=当前位置）→ 写梯形参数并回读。

        启动流程**不改写**电机参数；要改走 16.3。
        """
        if not confirmed_supported:
            raise StartupRefused("没有确认臂已支撑，拒绝使能")
        # 27.2 第 3 条：失能以后同一次上电里不再使能
        if self.disabled_latch:
            if not power_cycle_confirmed:
                raise StartupRefused(
                    "已经失能过（" + (self.disabled_latch_reason or "原因没记")
                    + "）：27.2 第 3 条规定这一次上电里不再使能任何关节。"
                    "请拉空开断电、重新上电，再勾选“已经断电重上电”后启动")
            self.clear_latch_on_power_cycle()
        # 27.2 第 6 条的配套：六颗已经在保持里时，不许再跑一遍启动序列
        # （11-18 两次 0x0200 就是这么来的：旧目标当阶跃，见 incident_20261118_AP1.md）
        if self.startup_done and not allow_restart:
            raise StartupRefused(
                "六颗已经在使能保持里（startup_done=True），不要重跑启动序列："
                "启动序列会把电机里存着的旧目标当阶跃，可能报 0x0200。"
                "要重新启动，请先正常停机、拉空开断电、再上电")
        if not self.connected:
            r = self.preflight()
            if not r["ok"]:
                raise StartupRefused("预检未通过：" + "；".join(r.get("mismatch", [])))

        t_start = time.monotonic()
        report = {"enabled": [], "voltages": {}, "zero_torque": [],
                  "warnings": [], "chain": {}}
        positions: dict[str, float] = {}
        hold_ref: dict[str, float] = {}

        # ---- 1) J6→J1：零扭矩使能 + 读核 ----
        for name in reversed(self.order):
            m = self.motors[name]
            c = self.cfg[name]
            # 尽量在使能前就把模式设成电流、电流 0（写命令无回包，失败也继续）
            try:
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.04)
            # 27.2 第 6 条：记下使能前的位置，使能后 0.3 s 内比对
            pos_before = None
            try:
                pos_before = m.pos_turns(timeout=0.08, retries=1)
            except Exception:  # noqa: BLE001
                pos_before = None
            if not self._enable_with_retry(m, name, report):
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 使能两次都无应答；已全部失能")
            self._ever_enabled = True
            # 使能后立刻零扭矩，避免“使能那一下出力”
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
            # 27.2 第 6 条：0.3 s 内读一次模式/位置/电流（原来只等 0.12 s）
            time.sleep(0.3)
            m.set_current_a(0.0)
            mode = m.mode()
            cur = m.current_a()
            pos = m.pos_turns()
            if pos is None:
                m.disable()
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 零扭矩使能后读不到位置；已全部失能")
            tel = m.telemetry(full=True)
            # A6：启动时逐颗核报警码；不是 0 先清一次再读，还不 0 就拒绝
            if tel.error:
                report.setdefault("alarm_clear", []).append(name)
                try:
                    m.clear_error()
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.12)
                tel.error = m.error(timeout=0.1, retries=2)
            issues = self._validate_enabled(name, tel)
            out_now = c.out_deg(pos)
            lo_s, hi_s = c.safe_limits_deg
            if out_now < lo_s - 0.5 or out_now > hi_s + 0.5:
                issues.append(
                    f"当前位置 {out_now:.1f}° 超出安全限位 {lo_s:.0f}..{hi_s:.0f}°；"
                    "先断电手动移回安全范围内再使能")
            # 27.2 第 6 条：使能后 0.3 s 内核对模式 / 位置 / 电流 / ρ
            issues.extend(startup_enable_issues(
                c, name, mode=mode, cur=cur, pos=pos, pos_before=pos_before,
                limit_a=tel.current_limit_a))
            if issues:
                m.disable()
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 启动核对不合格，已把这颗失能并停下："
                    + "；".join(issues))
            positions[name] = pos
            v = m.voltage_v()
            report["voltages"][name] = None if v is None else round(v, 2)
            report["zero_torque"].append({
                "name": name, "mode": mode,
                "current_a": None if cur is None else round(cur, 3),
            })
            report["enabled"].append({
                "name": name, "pos_turns": round(pos, 5),
                "out_deg": round(c.out_deg(pos), 3),
                "current_limit_a": None if tel.current_limit_a is None else round(tel.current_limit_a, 2),
                "speed_limit_rpm": None if tel.speed_limit_rpm is None else round(tel.speed_limit_rpm, 1),
                "pos_limits_deg": None if tel.pos_limit_hi_r is None else [
                    round(tel.pos_limit_lo_r * DEG_PER_MOTOR_TURN, 1),
                    round(tel.pos_limit_hi_r * DEG_PER_MOTOR_TURN, 1)],
                "voltage_v": None if v is None else round(v, 2),
            })
            self.current_limit_readback[name] = tel.current_limit_a
            if c.required_current_limit_a is not None and tel.current_limit_a is not None:
                if abs(tel.current_limit_a - c.required_current_limit_a) > 0.15:
                    report["warnings"].append(
                        f"{name} 限流 {tel.current_limit_a:.2f} A ≠ 要求 "
                        f"{c.required_current_limit_a:.2f} A（启动不改写，记录）")

        # ---- 2) 成链检查：母线电压 ----
        vs = {n: v for n, v in report["voltages"].items() if v is not None}
        if len(vs) < len(self.order):
            self._disable_all_quiet()
            raise StartupRefused("成链检查：有电机读不到母线电压；已全部失能")
        # B4AE（J7）读数比实际低约 1.5 V；现在带夹爪时把 J7 加回去
        for n, v in list(vs.items()):
            if n.upper() == "J7":
                vs[n] = v + 1.5
        vmax = max(vs.values())
        low = {n: v for n, v in vs.items() if vmax - v > 1.0}
        report["chain"] = {"max_v": round(vmax, 2),
                           "voltages": {n: round(v, 2) for n, v in vs.items()}}
        if low:
            self._disable_all_quiet()
            raise StartupRefused(
                "成链检查失败：母线电压比最高低 1.0 V 以上："
                + "；".join(f"{n} {v:.2f} V" for n, v in low.items())
                + f"（最高 {vmax:.2f} V）；已全部失能")

        # 35.2 第 4 条：第一次写梯形参数之前，只读六颗 0x1C / 0x1D / 0x1E
        # （原始值记进会话记录；35.3 S4 要报“启动时读到的是正还是负”）
        report["profile_before"] = {}
        try:
            pre = {}
            for i in range(0, len(self.order), 2):
                chunk = self.order[i:i + 2]
                items = [(self.motors[n].mid, c, 4) for n in chunk
                         for c in (0x1C, 0x1D, 0x1E)]
                got = self.link.read_many(items, timeout=0.06, retries=2)
                for n in chunk:
                    row = {}
                    for c in (0x1C, 0x1D, 0x1E):
                        d = got.get((self.motors[n].mid, c))
                        if d and len(d) >= 4:
                            raw = struct.unpack(">i", d)[0]
                            row[hex(c)] = {"raw_hex": d.hex(),
                                           "raw_iq8": raw,
                                           "value_deg_s": round(raw / 256.0 * 10.0 / 60.0, 3)}
                        else:
                            row[hex(c)] = None
                    pre[n] = row
            report["profile_before"] = pre
        except Exception as exc:  # noqa: BLE001
            report["profile_before_error"] = str(exc)
        if self.recorder and report["profile_before"]:
            for n, row in report["profile_before"].items():
                try:
                    self.recorder.command(
                        "profile_readback_before",
                        " ".join(f"{k}={None if v is None else v['raw_hex']}"
                                 for k, v in row.items()), joint=n)
                except Exception:  # noqa: BLE001
                    pass

        # ---- 3) 切 S 曲线保持，写梯形参数并回读 ----
        for name in self.order:
            m = self.motors[name]
            c = self.cfg[name]
            pos = m.pos_turns()
            if pos is None:
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 进入保持前读不到位置；已全部失能")
            hold_ref[name] = c.out_deg(pos)
            m.set_target_turns(pos)       # 目标=当前位置，不跳变
            time.sleep(0.02)
            m.set_target_turns(pos)       # 再发一次，降低丢包导致切模式阶跃
            time.sleep(0.02)
            m.set_mode(MODE_PROFILE_POSITION)
            time.sleep(0.08)
            mode_now = m.mode()
            if mode_now != MODE_PROFILE_POSITION:
                m.set_mode(MODE_PROFILE_POSITION)
                time.sleep(0.12)
                mode_now = m.mode()
            if mode_now != MODE_PROFILE_POSITION:
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 没有进入 S 曲线位置模式（当前模式 {mode_now}）；已全部失能")
            self._write_profile(name, c.motion_speed_deg_s)
            time.sleep(0.05)
            prof = m.profile()
            if prof is None:
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 梯形参数读不回来；已全部失能")
            vel_deg_s = prof[0] * DEG_PER_MOTOR_TURN / 60.0
            acc_deg_s2 = prof[1] * DEG_PER_MOTOR_TURN / 60.0
            dec_deg_s2 = prof[2] * DEG_PER_MOTOR_TURN / 60.0
            want_acc = c.motion_speed_deg_s / max(c.motion_accel_time_s, 0.05)
            if abs(vel_deg_s - c.motion_speed_deg_s) > 1.0:
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 梯形最大速度回读 {vel_deg_s:.1f}°/s ≠ "
                    f"{c.motion_speed_deg_s:.1f}°/s；已全部失能")
            if abs(acc_deg_s2 - want_acc) > max(2.0, want_acc * 0.1):
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 梯形加速度回读 {acc_deg_s2:.1f}°/s² ≠ "
                    f"{want_acc:.1f}°/s²；已全部失能")
            self._targets[name] = c.out_deg(pos)
            for item in report["enabled"]:
                if item["name"] == name:
                    item["profile_deg_s"] = round(vel_deg_s, 2)
                    item["profile_acc_deg_s2"] = round(acc_deg_s2, 2)
                    item["profile_dec_deg_s2"] = round(dec_deg_s2, 2)
                    item["mode"] = m.mode()

        # ---- 4) 保持目标确认：如果目标写丢了，不能让它停在旧目标 ----
        pos2 = self.link.read_many(
            [(self.motors[n].mid, R_POSITION, 4) for n in self.order],
            timeout=0.07, retries=3)
        moved = []
        for name in self.order:
            d = pos2.get((self.motors[name].mid, R_POSITION))
            if d is None:
                moved.append(f"{name} 读不到保持位置")
                continue
            p2 = struct.unpack(">i", d)[0] / IQ24
            dq = abs(self.cfg[name].out_deg(p2) - hold_ref.get(name, self.cfg[name].out_deg(p2)))
            if dq > 2.0:
                moved.append(f"{name} 保持位置偏了 {dq:.2f}°")
        if moved:
            self._disable_all_quiet()
            raise StartupRefused(
                "S 曲线保持目标没有确认（这是启动自检，不是电机报警，清报警不能消），"
                "已全部失能：" + "；".join(moved)
                + "。请确认臂已支撑、J2/J3 没被重力带走，再重试")
        self.startup_done = True
        report["ok"] = True
        report["elapsed_s"] = round(time.monotonic() - t_start, 2)
        report["msg"] = "零扭矩使能/读核/成链检查通过；已切 S 曲线保持"
        self.startup_report = report
        return report

    def try_takeover(self) -> dict:
        """30.6 第 4 条：重连时六颗还使能着 → “接管”，不跑启动序列。

        条件：六颗都读得到、电源=1、模式=6、报警=0、位置过 A3 合理性检查。
        满足时**只发 0x0A**（目标=当前位置），不发 0x07 / 0x08 / 0x2A，然后进入保持。
        有任何一条不满足：一条写命令都不发，返回能不能接管的说明。
        """
        import time as _time
        if self.disabled_latch:
            return {"took_over": False, "enabled_count": 0, "blocked": "latch",
                    "msg": "已经失能过（同一次上电不再使能/接管）：请拉空开断电后重上电"}
        try:
            st = self.read_all(full=True)
        except Exception as exc:  # noqa: BLE001
            return {"took_over": False, "enabled_count": 0,
                    "msg": f"接管检查读不到状态：{exc}"}
        enabled = [n for n in self.order if getattr(st.get(n), "enabled", None) is True]
        if not enabled:
            return {"took_over": False, "enabled_count": 0,
                    "msg": "六颗都是失能状态：等正常启动序列"}
        bad: list[str] = []
        for n in self.order:
            s = st.get(n)
            if s is None or s.enabled is not True:
                bad.append(f"{n} 电源≠1")
                continue
            if s.mode != MODE_PROFILE_POSITION:
                bad.append(f"{n} 模式={s.mode}")
            if s.error:
                bad.append(f"{n} 报警 0x{int(s.error):04X}")
            if s.out_deg is None:
                bad.append(f"{n} 位置读不到或没通过合理性检查")
        if bad:
            return {"took_over": False, "enabled_count": len(enabled),
                    "bad": bad,
                    "msg": ("臂还使能着，但不能接管：" + "，".join(bad)
                            + "。先扶住臂（或确认臂在搁架上），再拉空开断电")}
        # 只发 0x0A：逐颗写目标=当前位置（模式 6 下写当前位置不会动）
        before: dict[str, float] = {}
        for n in self.order:
            pos = self.motors[n].pos_turns(timeout=0.1, retries=2)
            if pos is None:
                return {"took_over": False, "enabled_count": len(enabled),
                        "msg": f"{n} 接管前读不到位置：一条写命令都没发，先扶住臂再拉空开"}
            before[n] = pos
            self.motors[n].set_target_turns(pos)
        _time.sleep(0.2)
        moved: dict[str, float] = {}
        for n in self.order:
            p2 = self.motors[n].pos_turns(timeout=0.1, retries=2)
            if p2 is None:
                moved[n] = 99.0
                continue
            dq = abs(self.cfg[n].out_deg(p2) - self.cfg[n].out_deg(before[n]))
            if dq > 0.2:
                moved[n] = round(dq, 2)
        if moved:
            return {"took_over": False, "enabled_count": len(enabled),
                    "moved": moved,
                    "msg": (f"接管时这些关节位置变了 {moved}°（不是原地保持）："
                            "请先扶住臂再拉空开；没有继续发写命令")}
        self.startup_done = True
        self._ever_enabled = True
        positions = {}
        for n in self.order:
            c = self.cfg[n]
            deg = c.out_deg(before[n])
            positions[n] = round(deg, 3)
            self._targets[n] = deg
            self._enabled_cache[n] = True
            self._trust[n] = (deg, _time.monotonic())
        self.startup_report = {"ok": True, "took_over": True,
                               "positions": positions,
                               "msg": "已接管（本来就在使能保持里）：没有重跑启动序列"}
        return {"took_over": True, "enabled_count": len(enabled),
                "positions": positions,
                "msg": "已接管（本来就在使能保持里）：没有重跑启动序列，原地保持"}

    @staticmethod
    def _enable_with_retry(m, name: str, report: dict) -> bool:
        """A7：第一次使能无应答，单独再试这一颗；第二次还不行才失败。"""
        if m.enable():
            return True
        report.setdefault("enable_retry", []).append(name)
        time.sleep(0.2)
        return bool(m.enable())

    def _validate_enabled(self, name: str, tel) -> list[str]:
        c = self.cfg[name]
        bad = []
        lo, hi = c.eeprom_limits_deg
        if tel.pos_limit_lo_r is None or tel.pos_limit_hi_r is None:
            bad.append("读不到位置限位")
        else:
            lo_d = tel.pos_limit_lo_r * DEG_PER_MOTOR_TURN
            hi_d = tel.pos_limit_hi_r * DEG_PER_MOTOR_TURN
            if lo_d < lo - 0.5 or hi_d > hi + 0.5:
                bad.append(f"位置限位 {lo_d:.1f}..{hi_d:.1f}° 比表 {lo}..{hi}° 宽")
        if tel.speed_limit_rpm is not None:
            v = tel.speed_limit_rpm * DEG_PER_MOTOR_TURN / 60.0
            if v > 70.0:
                bad.append(f"速度限制 {v:.1f}°/s > 70°/s")
        else:
            bad.append("读不到速度限制值")
        # P08-2/P08-6：J2 阈值跟 #0/#16 支架材料走（PLA 4.5 A / PETG100 6.5 A）
        limit = c.current_check_max_a
        if name == "J2":
            limit = 6.5 if c.j2_bracket.upper() == "PETG100" else 4.5
        if limit is not None:
            if tel.current_limit_a is None:
                bad.append("读不到限流值")
            elif tel.current_limit_a > limit:
                bad.append(f"限流 {tel.current_limit_a:.2f} A > {limit:.1f} A（支架 {c.j2_bracket}）")
        if tel.error:
            bad.append(f"报警 0x{tel.error:04X} {describe_error(tel.error)}")
        return bad

    def _write_profile(self, name: str, vel_deg_s: float):
        c = self.cfg[name]
        acc = vel_deg_s / max(c.motion_accel_time_s, 0.05)
        self.motors[name].set_profile_out(vel_deg_s, acc, acc)

    def _disable_all_quiet(self):
        for m in self.motors.values():
            try:
                m.disable()
            except Exception:  # noqa: BLE001
                pass
        self.latch_disabled("启动自检失败/全部失能")

    def disable_all(self, confirmed: bool = False, reason: str = "") -> bool:
        if not confirmed:
            raise StartupRefused("全部失能要二次确认（臂在零位且已扶住）")
        ok = True
        for name in reversed(self.order):
            ok &= self.motors[name].disable()
            self._enabled_cache[name] = False
        self.startup_done = False
        self.latch_disabled(reason or "全部失能")
        return ok

    def latch_disabled(self, reason: str = ""):
        """27.2 第 3 条：记下“这一次上电里已经失能过”，之后不许再使能。"""
        # 30.5 第 3 条：还没使能过就停下的（握手/枚举/ID↔SN/读不到），不置锁存
        if not self._ever_enabled:
            return False
        if not self.disabled_latch:
            self.disabled_latch = True
            self.disabled_latch_at = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.disabled_latch_reason = str(reason or "")
            if not self.is_mock:
                try:
                    cfgmod.save_disabled_latch(
                        {"disabled_latch": True, "at": self.disabled_latch_at,
                         "reason": self.disabled_latch_reason})
                except Exception:  # noqa: BLE001
                    pass
        return self.disabled_latch

    def latch_status(self) -> dict:
        return {"disabled_latch": bool(self.disabled_latch),
                "at": self.disabled_latch_at,
                "reason": self.disabled_latch_reason,
                "msg": ("已经失能过：这一次上电里不再使能任何关节，请拉空开断电后重上电"
                        if self.disabled_latch else "")}

    def clear_latch_on_power_cycle(self):
        """用户确认“已经拉空开断电、重新上电”以后才清掉锁存（27.2 第 3 条）。"""
        self.disabled_latch = False
        self.disabled_latch_at = None
        self.disabled_latch_reason = ""
        self._ever_enabled = False
        if not self.is_mock:
            try:
                cfgmod.save_disabled_latch({})
            except Exception:  # noqa: BLE001
                pass


    # ---------------- 读 ----------------
    def _plausible(self, name: str, val: float, now: float) -> tuple[bool, float]:
        """A3：和上一次可信读数比，超过 70°/s×间隔 + 2° 就不可信。"""
        prev = self._trust.get(name)
        if prev is None:
            return True, float("inf")
        dt = max(1e-3, now - prev[1])
        if dt > 5.0:
            # 长时间没有可信读数（失能/断电/拖动过）：不拿旧值卡新值
            return True, float("inf")
        allowed = self.plausible_rate * dt + self.plausible_margin
        return abs(val - prev[0]) <= allowed, allowed

    def _record_anomaly(self, name: str, raw_hex: str, raw_deg: float,
                        prev_deg: float | None, dt: float):
        if self.anomaly_hook is not None:
            try:
                self.anomaly_hook(name, raw_hex, raw_deg, prev_deg, dt)
            except Exception:  # noqa: BLE001
                pass

    def read_all(self, full: bool = False, fast: bool = True) -> dict[str, JointState]:
        out: dict[str, JointState] = {}
        to = 0.07 if fast else 0.15
        rt = 2 if fast else 3
        names = list(self.order)
        with self._lock:
            ids = [(self.motors[n].mid, n) for n in names]
            def batch(cmd, n, size=2):
                got = {}
                for i in range(0, len(ids), size):
                    chunk = ids[i:i + size]
                    got.update(self.link.read_many(
                        [(mid, cmd, n) for mid, _ in chunk], timeout=to, retries=rt))
                return got
            # 位置 0x06 六颗并发很稳；其它指令 6 颗一起发会丢，按 2 颗一组
            pos = batch(R_POSITION, 4, size=6)
            err = batch(R_ERROR, 2, size=2)
            full_maps: dict[int, dict] = {}
            if full:
                # 状态页只需要电压/温度/模式/电流；限流/速度/限位在启动报告里已有
                specs = [(R_VOLTAGE, 2), (R_TEMP_MOTOR, 2), (R_MODE, 1),
                         (R_CURRENT_MAX, 2), (R_CURRENT, 4), (R_CURRENT_LIMIT, 4)]
                for cmd, n in specs:
                    full_maps[cmd] = batch(cmd, n, size=2)
            now = time.monotonic()
            for mid, name in ids:
                m = self.motors[name]
                c = self.cfg[name]
                st = JointState(name=name, id=c.id)
                d = pos.get((mid, R_POSITION))
                raw_hex = "" if d is None else d.hex()
                candidate = None
                if d is not None:
                    p = struct.unpack(">i", d)[0] / IQ24
                    st.enabled = True
                    st.pos_turns = p
                    candidate = c.out_deg(p)
                else:
                    p = m.pos_turns(timeout=to, retries=rt)
                    if p is None:
                        pw = m.power(timeout=to, retries=rt)
                        st.enabled = None if pw is None else (pw == 1)
                        self._enabled_cache[name] = st.enabled
                    else:
                        st.enabled = True
                        st.pos_turns = p
                        candidate = c.out_deg(p)
                        self._enabled_cache[name] = True
                if candidate is not None:
                    ok, allowed = self._plausible(name, candidate, now)
                    if ok and self._trust.get(name) is None:
                        # 第一次读数：再单独读一次，两次一致才建立可信基准
                        pchk = m.pos_turns(timeout=max(0.09, to), retries=3)
                        if pchk is not None and abs(c.out_deg(pchk) - candidate) > 0.5:
                            ok = False
                    if not ok:
                        # A3 第 1 条：立刻单独再读一次；还不合理就判读数异常
                        p2 = m.pos_turns(timeout=max(0.09, to), retries=3)
                        if p2 is not None:
                            cand2 = c.out_deg(p2)
                            ok2, _ = self._plausible(name, cand2, time.monotonic())
                            if ok2:
                                ok, candidate = True, cand2
                                st.pos_turns, st.enabled = p2, True
                                raw_hex = ""
                        if not ok:
                            prev = self._trust.get(name)
                            # 连续多次“新值彼此一致、只是离旧可信值很远”：
                            # 说明臂确实被移到了新位置（例如断电后手推过），
                            # 重新锚定并大声记一笔；拖拽/乱跳时不会一致。
                            last = self._invalid_candidate.get(name)
                            if last is not None and abs(candidate - last) <= 0.5:
                                self._invalid_streak[name] = \
                                    self._invalid_streak.get(name, 0) + 1
                            else:
                                self._invalid_streak[name] = 1
                            self._invalid_candidate[name] = candidate
                            if self._invalid_streak[name] >= 5:
                                self._trust[name] = (candidate, time.monotonic())
                                self._invalid_streak[name] = 0
                                st.out_deg = candidate
                                self._record_anomaly(
                                    name, raw_hex, candidate,
                                    None if prev is None else prev[0], float("inf"))
                            else:
                                st.pos_invalid = True
                                st.pos_raw_deg = candidate
                                st.raw_pos_hex = raw_hex
                                st.out_deg = None
                                tprev = self._anomaly_at.get(name, 0.0)
                                if time.monotonic() - tprev > 1.0:
                                    self._anomaly_at[name] = time.monotonic()
                                    self._record_anomaly(
                                        name, raw_hex, candidate,
                                        None if prev is None else prev[0],
                                        float("inf") if prev is None else now - prev[1])
                        else:
                            self._trust[name] = (candidate, time.monotonic())
                            st.out_deg = candidate
                    else:
                        self._trust[name] = (candidate, now)
                        self._invalid_streak[name] = 0
                        st.out_deg = candidate
                st.pos_read_at = time.time()
                st.target_deg = self._targets.get(name)
                st.current_limit_a = self.current_limit_readback.get(name)


                de = err.get((mid, R_ERROR))
                if de is not None:
                    st.error = struct.unpack(">H", de)[0]
                elif st.enabled:
                    st.error = m.error(timeout=to, retries=rt)
                st.error_text = describe_error(st.error or 0)

                if full and st.enabled:
                    def u(cmd, fmt):
                        dd = full_maps.get(cmd, {}).get((mid, cmd))
                        return struct.unpack(fmt, dd)[0] if dd is not None else None
                    cm = u(R_CURRENT_MAX, ">h")
                    if cm is not None:
                        cm = cm / IQ8
                        self.current_full_scale_readback[name] = cm
                    v = u(R_VOLTAGE, ">H")
                    st.voltage_v = None if v is None else v / IQ10
                    t = u(R_TEMP_MOTOR, ">h")
                    st.temp_c = None if t is None else t / 256.0
                    md = u(R_MODE, ">B")
                    st.mode = md
                    cur = u(R_CURRENT, ">i")
                    st.current_raw = cur
                    st.current_full_scale_a = cm
                    st.current_a = None if cur is None or cm is None else cur / IQ24 * cm
                    cl = u(R_CURRENT_LIMIT, ">i")
                    if cl is not None and cm is not None:
                        st.current_limit_a = cl / IQ24 * cm
                        self.current_limit_readback[name] = st.current_limit_a
                    else:
                        st.current_limit_a = self.current_limit_readback.get(name)
                out[name] = st
                self._last_state[name] = st
        return out

    def read_cvp_all(self, timeout: float = 0.05, retries: int = 2,
                     names=None) -> dict[str, dict]:
        """26.2：六颗一轮用 0x94 同时取位置/速度/电流（输出端角度）。

        names 给了就只读这几颗（27.3 高频记录时减少一轮的指令数、把节拍提上去）。
        """
        order = [str(x).upper() for x in names] if names else list(self.order)
        with self._lock:
            got = self.link.read_many(
                [(self.motors[n].mid, 0x94, 7) for n in order],
                timeout=timeout, retries=retries)
            out: dict[str, dict] = {}
            for name in order:
                d = got.get((self.motors[name].mid, 0x94))
                if not d or len(d) < 7:
                    out[name] = {"ok": False}
                    continue
                raw = int.from_bytes(d[0:3], "big")
                if raw & 0x800000:
                    raw -= 1 << 24
                pos_turns = raw / 65536.0
                vel_raw = struct.unpack(">h", d[3:5])[0]
                cur_raw = struct.unpack(">h", d[5:7])[0]
                c = self.cfg[name]
                vel_rpm = vel_raw / 16384.0 * VELOCITY_FULL_RPM
                # 30.1：0x94 电流是 IQ14 × 满量程（0x53），不是 × 限流（0x59）
                fs = self.current_full_scale_readback.get(name)
                if fs is None:
                    try:
                        fs = self.motors[name].current_max_a()
                    except Exception:  # noqa: BLE001
                        fs = None
                    if fs is not None:
                        self.current_full_scale_readback[name] = fs
                lim = self.current_limit_readback.get(name)
                cur_a = None if fs is None else cur_raw / 16384.0 * float(fs)
                out[name] = {
                    "ok": True, "raw": d.hex(),
                    "pos_deg": c.out_deg(pos_turns),
                    "vel_deg_s": c.sign * vel_rpm * DEG_PER_MOTOR_TURN / 60.0,
                    "current_a": cur_a,
                    "limit_a": lim,
                    "rho": (None if cur_a is None or not lim else abs(cur_a) / lim),
                }
        return out

    def read_errors(self, timeout: float = 0.04, retries: int = 1) -> dict[str, int | None]:
        """26.2：运动循环里 5 Hz 读报警码（按 2 颗一批）。"""
        with self._lock:
            got = {}
            order = list(self.order)
            for i in range(0, len(order), 2):
                chunk = order[i:i + 2]
                got.update(self.link.read_many(
                    [(self.motors[n].mid, R_ERROR, 2) for n in chunk],
                    timeout=timeout, retries=retries))
            out = {}
            for n in order:
                d = got.get((self.motors[n].mid, R_ERROR))
                out[n] = None if d is None else struct.unpack(">H", d[:2])[0]
        return out

    def read_many_raw(self, items, timeout: float = 0.05, retries: int = 2):
        """A2：给 trace 用；和 read_all 走同一把总线锁，不另开收发路径。"""
        with self._lock:
            return self.link.read_many(items, timeout=timeout, retries=retries)

    def read_status(self, timeout: float = 0.08) -> dict[str, dict]:
        """A5：电源 0x2B、模式 0x55、报警 0xFF。六颗一起发，丢的按颗单独补。"""
        with self._lock:
            # 18 条一起发会超 ECB 的缓冲、造成自己丢包重试；按指令拆成 6 条一批
            got = {}
            order = list(self.order)
            for cmd, n in ((R_POWER, 1), (R_MODE, 1), (R_ERROR, 2)):
                for i in range(0, len(order), 2):
                    chunk = order[i:i + 2]
                    got.update(self.link.read_many(
                        [(self.motors[nm].mid, cmd, n) for nm in chunk],
                        timeout=timeout, retries=2))
            out: dict[str, dict] = {}
            for n in self.order:
                mid = self.motors[n].mid
                row = {}
                for key, cmd in (("power", R_POWER), ("mode", R_MODE),
                                 ("error", R_ERROR)):
                    d = got.get((mid, cmd))
                    if d is None:
                        try:
                            if cmd == R_POWER:
                                row[key] = self.motors[n].power(timeout=0.06, retries=2)
                            elif cmd == R_MODE:
                                row[key] = self.motors[n].mode(timeout=0.06, retries=2)
                            else:
                                row[key] = self.motors[n].error(timeout=0.06, retries=2)
                        except Exception:  # noqa: BLE001
                            row[key] = None
                    elif cmd == R_ERROR:
                        row[key] = struct.unpack(">H", d[:2])[0]
                    else:
                        row[key] = d[0]
                out[n] = row
        return out

    # ---------------- 写 ----------------
    def set_targets_q(self, targets: dict[str, float], confirm: bool = True) -> dict[str, bool]:
        """写目标；26.7 7b 用优先发送口，26.7 7 用行为确认送到。"""
        res = {}
        with self._lock:
            for name, q in targets.items():
                c = self.cfg[name]
                res[name] = self.motors[name].set_target_turns(c.turns(q), priority=True)
                self._targets[name] = q
        if confirm and not self.skip_resend:
            self.confirm_targets(targets)
        return res

    def confirm_targets(self, targets: dict[str, float], wait: float = 0.06) -> list[str]:
        """26.7 7：写出目标以后看行为（速度朝目标起来/已在到位窗口）。

        只对 |d| > 2° 的关节判；没起来就重发一次并记进 last_resend。
        """
        time.sleep(max(0.02, wait))
        resend: list[str] = []
        try:
            cvp = self.read_cvp_all(timeout=0.03, retries=1)
        except Exception:  # noqa: BLE001
            return resend
        for name, q in targets.items():
            st = cvp.get(name) or {}
            if not st.get("ok"):
                continue
            d = float(q) - float(st.get("pos_deg") or 0.0)
            if abs(d) <= 0.8:
                continue
            u = (1.0 if d > 0 else -1.0) * float(st.get("vel_deg_s") or 0.0)
            if u >= 0.5:
                continue
            try:
                self.motors[name].set_target_turns(self.cfg[name].turns(q), priority=True)
                self._targets[name] = q
                resend.append(name)
            except Exception:  # noqa: BLE001
                pass
        self.last_resend = resend
        if resend and self.write_resend_hook is not None:
            try:
                self.write_resend_hook(resend, targets)
            except Exception:  # noqa: BLE001
                pass
        return resend

    def set_profiles_q(self, vel: dict[str, float], acc: dict[str, float] | None = None,
                       disp: dict[str, float] | None = None):
        """写梯形参数。

        35.2 第 2 条：本段不动的关节（|位移| ≤0.8°）加速度/减速度不小于这颗正常
        全速动作时的值（motion_speed ÷ motion_accel_time），避免“残留速度 + 很小加速度”
        把不动的关节带着漂出去。动着的关节照旧。
        """
        acc = acc or {}
        for name in self.order:
            if name in vel:
                a = acc.get(name, vel[name] / max(self.cfg[name].motion_accel_time_s, 0.05))
                if disp is not None and abs(float(disp.get(name, 0.0))) <= 0.8:
                    floor = self.cfg[name].motion_speed_deg_s / max(
                        self.cfg[name].motion_accel_time_s, 0.05)
                    a = max(float(a), floor)
                self.motors[name].set_profile_out(vel[name], a, a)

    def verify_ready(self, names=None) -> list[tuple[str, str]]:
        """检查每颗：电源必须为 1，模式必须为 S 曲线位置 6。

        20.2：批量读丢了的，按颗单独再问一次（单条、带重试）；仍读不到
        才算坏，并把“读不到”和“读到了但值不对”分开报。
        """
        names = list(names or self.order)
        ids = [(self.motors[n].mid, n) for n in names]
        power, mode = {}, {}
        for i in range(0, len(ids), 2):
            chunk = ids[i:i + 2]
            try:
                power.update(self.link.read_many(
                    [(mid, R_POWER, 1) for mid, _ in chunk], timeout=0.06, retries=2))
                mode.update(self.link.read_many(
                    [(mid, R_MODE, 1) for mid, _ in chunk], timeout=0.06, retries=2))
            except Exception:  # noqa: BLE001
                # 通信异常按“读不到”处理，交给单颗重试/上层中止
                pass

        def single(mid: int, cmd: int) -> int | None:
            # 20.2：单条再问，最多 12 次短超时。30% 丢包时 0.3^13 ≈ 1.6e-7，
            # 200 次起步 × 12 个字段的理论误判上限约 4e-4 次。
            for _ in range(12):
                try:
                    d = self.link.read(mid, cmd, 1, timeout=0.06, retries=1)
                except Exception:  # noqa: BLE001
                    d = None
                if d:
                    return d[0]
            return None

        bad: list[tuple[str, str]] = []
        for mid, name in ids:
            p = (power.get((mid, R_POWER)) or b"")[:1]
            m = (mode.get((mid, R_MODE)) or b"")[:1]
            p = p[0] if p else None
            m = m[0] if m else None
            if p is None:
                p = single(mid, R_POWER)
            if m is None:
                m = single(mid, R_MODE)
            if p is None and m is None:
                bad.append((name, "读不到电源(0x2B)和模式(0x55)"))
            elif p is None:
                bad.append((name, "读不到电源(0x2B)"))
            elif m is None:
                bad.append((name, "读不到模式(0x55)"))
            elif p != 1 and m not in (MODE_PROFILE_POSITION, MODE_POSITION):
                bad.append((name, f"读到了但 power={p},mode={m}"))
            elif p != 1:
                bad.append((name, f"读到了但 power={p}"))
            elif m not in (MODE_PROFILE_POSITION, MODE_POSITION):
                bad.append((name, f"读到了但 mode={m}"))
        return bad

    # ---------------- 18.3 停机：单颗失能 / 重新使能到保持 ----------------
    def read_positions_turns(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for n in self.order:
            p = self.motors[n].pos_turns()
            if p is not None:
                out[n] = p
        return out

    def disable_one(self, name: str) -> bool:
        ok = bool(self.motors[name].disable())
        self._enabled_cache[name] = False
        self.latch_disabled(f"{name} 失能")
        return ok

    def enable_one_hold(self, name: str, pos_turns: float) -> bool:
        """把一颗重新使能到“目标 = 该位置”的 S 曲线保持（停机流程报警时用）。"""
        m = self.motors[name]
        c = self.cfg[name]
        try:
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
        except Exception:  # noqa: BLE001
            pass
        if not m.enable():
            return False
        self._ever_enabled = True
        time.sleep(0.1)
        # 20.9.1：先写目标=当前位置，再切位置模式，再写一次目标
        m.set_target_turns(pos_turns)
        self._write_profile(name, c.motion_speed_deg_s)
        m.set_mode(MODE_PROFILE_POSITION)
        m.set_target_turns(pos_turns)
        self._targets[name] = c.out_deg(pos_turns)
        return True

    def current_limit_a(self, name: str) -> float | None:
        try:
            return self.motors[name].current_limit_a()
        except Exception:  # noqa: BLE001
            return None

    def current_raw_counts(self, name: str):
        """18.4：返回 (0x04 原始计数, 0x59 原始计数)。"""
        try:
            return self.motors[name].current_raw_counts()
        except Exception:  # noqa: BLE001
            return (None, None)

    def current_full_scale_a(self, name: str) -> float | None:
        """0x53 原始满量程（A）。18.4 要求记这个，不是换算后的限流。"""
        try:
            return self.motors[name].current_max_a()
        except Exception:  # noqa: BLE001
            return None

    def set_stream_mode(self, mode: int) -> bool:
        """切位置模式（3）或 S 曲线位置模式（6），用于流式目标。

        官方 SDK 的 MoveJointPath 用 Mode_Pos 3 + 外部插补；这里在长动作
        开始前切 3，结束/中止时 hold_all 会切回 6。
        先把目标写成当前位置，再切模式，避免切换瞬间阶跃。
        """
        try:
            with self._lock:
                for name in self.order:
                    m = self.motors[name]
                    pos = m.pos_turns(timeout=0.15, retries=2)
                    if pos is None:
                        return False
                    m.set_target_turns(pos)
                    m.set_mode(mode)
                    m.set_target_turns(pos)
                time.sleep(0.2)
                for name in self.order:
                    now = self.motors[name].mode()
                    if now != mode:
                        return False
            return True
        except Exception:  # noqa: BLE001
            return False

    def hold_all(self) -> dict:
        """逐颗尽力把目标改写成当前位置，并切回 S 曲线位置模式。

        20.1：不因个别关节读不到/没确认就整体抛异常。返回：
            {"ok": bool, "held": {name: deg},
             "unconfirmed": [{"name": n, "reason": r}],
             "unreadable": [name, ...]}
        """
        held: dict[str, float] = {}
        kept: dict[str, float] = {}
        unconfirmed: list[dict] = []
        unreadable: list[str] = []
        seen: set[str] = set()

        def add(name: str, reason: str):
            if name not in seen:
                seen.add(name)
                unconfirmed.append({"name": name, "reason": reason})

        with self._lock:
            for name in self.order:
                m = self.motors[name]
                try:
                    pos = m.pos_turns(timeout=0.15, retries=2)
                except Exception:  # noqa: BLE001
                    pos = None
                if pos is None:
                    unreadable.append(name)
                    add(name, "读不到位置")
                    continue
                # A3 第 2 条：保持目标只用刚读到、并且通过合理性检查的值
                cand = self.cfg[name].out_deg(pos)
                ok, _ = self._plausible(name, cand, time.monotonic())
                if not ok:
                    pos2 = m.pos_turns(timeout=0.12, retries=3)
                    if pos2 is not None:
                        cand2 = self.cfg[name].out_deg(pos2)
                        ok2, _ = self._plausible(name, cand2, time.monotonic())
                        if ok2:
                            ok, pos, cand = True, pos2, cand2
                if not ok:
                    unreadable.append(name)
                    add(name, "读数异常（不合理跳变），不当保持目标")
                    self._record_anomaly(name, "", cand,
                                         (self._trust.get(name) or (None,))[0],
                                         float("inf"))
                    continue
                self._trust[name] = (cand, time.monotonic())
                # 35.2 第 1 条：到位窗口里（|实际 − 当前目标| ≤0.8°）的关节
                # 不改写目标、不重切模式，保留原来的目标
                tgt_now = self._targets.get(name)
                if tgt_now is not None and abs(cand - float(tgt_now)) <= 0.8:
                    kept[name] = float(tgt_now)
                    continue
                try:
                    m.set_target_turns(pos, priority=True)
                    m.set_mode(MODE_PROFILE_POSITION)
                    m.set_target_turns(pos, priority=True)
                except Exception as exc:  # noqa: BLE001
                    add(name, f"写目标/切模式失败：{exc}")
                    continue
                held[name] = cand
                self._targets[name] = held[name]

        if held:
            for n, why in self.verify_ready(list(held)):
                add(n, why)
        return {"ok": not unconfirmed, "held": held, "kept": kept,
                "unconfirmed": unconfirmed, "unreadable": unreadable}

    def stats(self) -> dict:
        return self.link.stats

    def close(self):
        self.link.close()
