#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段一 A 执行器（独立后台程序）。

职责：传输层 + 关节层 + 运动层 + 限位/围栏检查；通过网络接口收命令、推状态。
网页只是它的一个客户端，不直接碰 ECB。

运行：
    python3 -m arm.executor --mock          # 模拟臂
    python3 -m arm.executor                 # 真机，网页里再点连接/使能
    python3 -m arm.executor --host 0.0.0.0 --port 8770   # 给局域网客户端

接口（HTTP/JSON）：
    GET  /api/config                 配置版本、接口信息
    GET  /api/state                  当前状态（轮询兜底）
    GET  /api/events?client_id=...   状态推送（SSE，默认 20 Hz）
    POST /api/acquire                拿控制权 {client_id, config_version}
    POST /api/heartbeat              心跳 {token}
    POST /api/release                释放控制权 {token}
    POST /api/connect                连接 ECB / 模拟
    POST /api/enable                 启动序列（J6→J1 逐个使能）
    POST /api/click                  虚拟点击（按模式预演或直接执行）
    POST /api/plan_click             只规划
    POST /api/execute                执行待执行计划
    POST /api/stop                   中止（原地保持）
    POST /api/jog                    点动
    POST /api/home                   回零位
    POST /api/disable_all            全部失能（二次确认）
    POST /api/move_joints            关节角数组
    POST /api/move_tcp               TCP 位姿
    POST /api/move_through           路径（每段可带下探标记）
    POST /api/tcp /api/yaw /api/exec_mode
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import mimetypes
import secrets
import struct
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from . import config as cfgmod
from . import gravity
from . import programs as progmod
from . import sweep as sweepmod
from .calib import CalibSession
from .collision import CollisionMonitor
from .fence import Fence
from .joints import JointBus, StartupRefused, load_joint_configs
from .kinematics import RobotModel
from .mock import MockJointBus, VirtualJointBus
from .motion import Motion, MotionError
from .sca import MODE_CURRENT, MODE_PROFILE_POSITION, describe_error
from .recorder import Recorder
from .session import Session
from .trace import TraceSampler

MIME = {".js": "text/javascript", ".mjs": "text/javascript",
        ".css": "text/css", ".html": "text/html; charset=utf-8",
        ".json": "application/json", ".svg": "image/svg+xml",
        ".png": "image/png", ".ico": "image/x-icon"}


class AuthError(RuntimeError):
    pass


class Executor:
    def __init__(self, cfg: dict | None = None):
        self.cfg = dict(cfg or cfgmod.load_executor())
        self.bind_host = self.cfg.get("bind_host", "127.0.0.1")
        self.bind_port = int(self.cfg.get("bind_port", 8770))
        self.ecb_ip = self.cfg.get("ecb_ip", "192.168.1.30")
        self.ecb_port = int(self.cfg.get("ecb_udp_port", 2000))
        self.web_dir = self._path(self.cfg.get("web_dir", "web"))
        self.runs_dir = self._path(self.cfg.get("runs_dir", "runs"))
        self.models_dir = self._path(self.cfg.get("models_dir", "models"))
        self.serve_web = bool(self.cfg.get("serve_web", True))
        self.push_hz = max(10.0, float(self.cfg.get("state_push_hz", 20)))
        # 37.1 第 6 条：空闲自动停机秒数（真 ECB 不许 >600、不许关；mock 可更短）
        self.idle_park_s = float(self.cfg.get("idle_park_s", 600.0))
        self.trace_hz = max(5.0, min(200.0, float(self.cfg.get("trace_hz", 100))))
        self.trace_seconds = max(1.0, min(60.0, float(self.cfg.get("trace_seconds", 10))))
        self.hb_timeout = float(self.cfg.get("heartbeat_timeout_s", 0.5))
        self.control_stale = float(self.cfg.get("control_stale_s", 3.0))
        self.exec_mode = self.cfg.get("default_exec_mode", "preview")
        # 21.4.4：全局速度倍率；第一次上真机默认 50%，网页可调
        self.speed_scale = max(0.05, min(1.0, float(
            self.cfg.get("motion_speed_scale", 0.5))))
        self.config_version = cfgmod.config_version()
        self.config_info = cfgmod.config_versions()
        self.web_version = self._read_web_version()

        self.robot = RobotModel(cfgmod.URDF, cfgmod.tcp_presets())
        self.joint_cfg = load_joint_configs()
        self.limits = (
            [self.joint_cfg[n].safe_limits_deg[0] for n in self.robot.joint_names],
            [self.joint_cfg[n].safe_limits_deg[1] for n in self.robot.joint_names],
        )
        self.fence = Fence(self.robot, workspace={
            "x": [-200.0, 600.0], "y": [-400.0, 400.0], "enabled": False,
        })

        self.bus = None
        self.motion: Motion | None = None
        # 离线/未使能时的零位虚拟模型（只读预演）
        self.preview_bus = VirtualJointBus(self.joint_cfg)
        self.preview_motion = Motion(self.preview_bus, self.robot, self.fence,
                                     self.limits, recorder=None)
        self.recorder: Recorder | None = None
        self.session: Session | None = None
        self.takeover_report: dict | None = None
        self._prev_errors: dict[str, int] = {}
        self._capture = None
        self.trace: TraceSampler | None = None
        self.mode = None
        self.ip = ""
        self.msg = "未连接"
        self.connected = False
        self.tcp_preset = cfgmod.load_tcp().get("default", "probe")
        self.yaw_deg = 0.0
        self.preview = None
        self.pending = None
        self._home_preview = None
        self.cache = {}
        self._lock = threading.RLock()
        self._poll_stop = threading.Event()
        self._poll_thread = None
        self._exec_thread = None
        self._exec_lock = threading.Lock()
        self._last_full = 0.0
        self._last_states = {}

        # 20.3：停机线程自己的停止标志；park_active 覆盖整个停机流程
        self._park_stop = threading.Event()
        self.park_active = False

        # 20.6：配置热重载（方案 B：连接期间配置变了就拒绝运动/使能，直到重连）
        self.config_stale = False
        self.config_stale_msg = ""

        # 控制权 / 心跳
        self.owner: str | None = None
        self.token: str | None = None
        self.owner_since = 0.0
        self.last_hb = 0.0
        self.clients: dict[str, float] = {}
        self._watch_stop = threading.Event()
        self._watch_thread = None
        self._calib_active = False
        self.calib = CalibSession(self)
        self.drag_active = False
        self.measurements: list[dict] = []
        self._measure_path = self.runs_dir / "measure.csv"

        # 22.2 往返扫描标定 / 22.4 关键点录放 / 22.3 碰撞残差记录
        self.sweep_runner = None
        self.program_store = progmod.ProgramStore(
            self._path(self.cfg.get("programs_dir", "programs")))
        # state 是 20 Hz 推送，不能每拍扫 programs/*.json
        try:
            self._program_names_cache = self.program_store.list()
        except Exception:  # noqa: BLE001
            self._program_names_cache = []
        self.program_points: list[dict] = []
        self.program_loaded: dict | None = None
        self.program_status = {"active": False, "phase": "idle", "name": "",
                               "loops": 0, "loop": 0, "point": 0,
                               "reason": "", "started": None, "finished": None}
        self.collision: CollisionMonitor | None = None
        self._measure_header = False
        self._web_state_lock = threading.Lock()

        # 18.1/18.2/18.3：搁架、末端配置、力矩预检、停机状态
        self.park_cfg = cfgmod.load_park()
        self.park_state = cfgmod.load_park_state()
        self.end_effector = "flange"
        self.payload_kg = 0.0
        self.kt = gravity.load_kt()
        self.kt_rows: list[dict] = []
        self.hold = {"since": None, "ratio": 0.0, "warn": ""}
        self._park_thread = None
        self.park_step = {"phase": "idle", "reason": "", "z": None, "step": 0,
                          "I0": None, "I2": None, "I1": None, "contact_z": None}
        self.park_block: dict | None = None
        # 37.1 第 6 条：空闲自动停机（用户 2026-11-26 定）
        # 空闲计时从最近一次“会改变臂状态的操作”起算；心跳/state/SSE/trace/只读不算
        self._last_op_at = time.monotonic()
        self._idle_triggered = False
        self._idle_running = False
        self._idle_last_trigger_at = 0.0
        self._idle_blocked_reason = ""
        self._on_shelf = False
        self.j2_bracket = str(getattr(self.joint_cfg.get("J2"), "j2_bracket", "PLA"))
        # 24 节 A3b：保持失效检测 / A5：状态/命令记录
        self.hold_guard = {"active": False, "hist": {}, "resent": {},
                           "resend_dist": {}, "since": None}
        self.hold_failure: dict | None = None
        self._poll_tick_hz = 0.0
        self._last_status_at = 0.0
        self._status_period_s = 0.2          # 5 Hz
        self._last_write_event = 0.0
        self._sync_motion_params()

    # ---------------- 路径/配置 ----------------
    def _read_web_version(self) -> str:
        try:
            return str(json.loads((self.web_dir / "version.json").read_text(encoding="utf-8"))
                       .get("version", ""))
        except Exception:  # noqa: BLE001
            return ""

    def _path(self, p: str) -> Path:
        q = Path(p)
        return q if q.is_absolute() else (cfgmod.ROOT / q)

    def refresh_config_version(self, force: bool = False):
        new_version = cfgmod.config_version()
        self.web_version = self._read_web_version()
        if new_version == self.config_version and not force:
            return
        self.config_version = new_version
        self.config_info = cfgmod.config_versions()
        # 20.6 方案 B：连接期间配置变了，不偷偷换一半派生量；
        # 拒绝使能/运动，直到重新 connect。断开时 refresh 才全量重载。
        if self.bus is not None and not force:
            self.config_stale = True
            self.config_stale_msg = (
                "配置已变（joints/tcp/URDF），请重新连接后再使能/运动")
            return
        self.config_stale = False
        self.config_stale_msg = ""
        self.joint_cfg = load_joint_configs()
        self.limits = (
            [self.joint_cfg[n].safe_limits_deg[0] for n in self.robot.joint_names],
            [self.joint_cfg[n].safe_limits_deg[1] for n in self.robot.joint_names],
        )
        # URDF 也可能变了：重连时整条链重建，Fence/预演一起换
        ws = getattr(self.fence, "workspace", None)
        self.robot = RobotModel(cfgmod.URDF, cfgmod.tcp_presets())
        self.fence = Fence(self.robot, workspace=ws)
        self.preview_bus = VirtualJointBus(self.joint_cfg)
        self.preview_motion.bus = self.preview_bus
        self.preview_motion.robot = self.robot
        self.preview_motion.fence = self.fence
        self.preview_motion.limits = (list(self.limits[0]), list(self.limits[1]))
        self.j2_bracket = str(getattr(self.joint_cfg.get("J2"), "j2_bracket", "PLA"))
        self.cache = {}
        self.preview = None
        self.pending = None
        self._home_preview = None
        self._sync_motion_params()

    def use_actual(self) -> bool:
        return bool(self.connected and self.motion is not None
                    and getattr(self.bus, "startup_done", False))

    def planning_motion(self):
        return self.motion if self.use_actual() else self.preview_motion

    def start_poll(self):
        if self._poll_thread and self._poll_thread.is_alive():
            return
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def config_payload(self) -> dict:
        return {
            "config_version": self.config_version,
            "web_version": self._read_web_version(),
            **self.config_info,
            "exec_mode": self.exec_mode,
            "default_exec_mode": self.cfg.get("default_exec_mode", "preview"),
            "speed_scale": round(float(self.speed_scale), 3),
            "state_push_hz": self.push_hz,
            "trace_hz": self.trace_hz,
            "trace_seconds": self.trace_seconds,
            "heartbeat_hz": float(self.cfg.get("heartbeat_hz", 5)),
            "heartbeat_timeout_s": self.hb_timeout,
            "control_stale_s": self.control_stale,
            "joint_names": self.robot.joint_names,
            "expected_count": int(cfgmod.load_joints().get("expected_count", 6)),
            "gripper_installed": bool(cfgmod.load_joints().get("gripper_installed", False)),
            "limits": {"lo": self.limits[0], "hi": self.limits[1]},
            "tcp_presets": self.robot.tcp_presets,
            "end_effector": self.end_effector,
            "end_effectors": list(gravity.END_EFFECTORS),
            "payload_kg": self.payload_kg,
            "park": self.park_config(),
            "ecb": {"ip": self.ecb_ip, "port": self.ecb_port},
            "bind": {"host": self.bind_host, "port": self.bind_port},
        }

    # ---------------- 控制权 ----------------
    def touch(self, client_id: str | None):
        if not client_id:
            return
        # 20.4：touch 只说明“这个客户端还连着”，不刷新控制权心跳
        self.clients[client_id] = time.monotonic()

    def acquire(self, client_id: str, version: str, token: str | None = None) -> dict:
        if not client_id:
            raise AuthError("缺 client_id")
        if version != self.config_version:
            raise AuthError(f"配置版本不一致：客户端 {version}，执行器 {self.config_version}；"
                            "拒绝一切运动命令，请刷新/重新拉配置")
        now = time.monotonic()
        if self.owner and self.owner != client_id:
            age = now - self.last_hb
            # 0 = 不自动释放：只要 owner 还在，别的 client 不能抢
            if self.control_stale <= 0 or age < self.control_stale:
                raise AuthError(f"控制权在 {self.owner} 手里（{age:.1f}s 前还有心跳）")
        if self.owner == client_id and self.token:
            # 20.9.5：同一个 client_id 重复 acquire 必须带原 token；
            # 光知道 client_id（/api/state 里公开）不能拿走控制权。
            if token != self.token:
                raise AuthError("client_id 已被占用；重复 acquire 必须带原 token")
            self.last_hb = now
            self.clients[client_id] = now
            return {"token": self.token, "client_id": client_id,
                    "config_version": self.config_version, "exec_mode": self.exec_mode}
        self.owner = client_id
        self.token = secrets.token_hex(12)
        self.owner_since = now
        self.last_hb = now
        self.clients[client_id] = now
        self.msg = f"{client_id} 取得控制权"
        return {"token": self.token, "client_id": client_id,
                "config_version": self.config_version, "exec_mode": self.exec_mode}

    def require_token(self, body: dict) -> str:
        token = str(body.get("token") or "")
        if not self.owner or not self.token or token != self.token:
            raise AuthError("没有控制权（另一个客户端在控制，或控制权已超时）")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return token

    def require_control(self, body: dict) -> str:
        token = str(body.get("token") or "")
        if not self.owner or not self.token or token != self.token:
            raise AuthError("没有控制权（另一个客户端在控制，或控制权已超时）")
        version = body.get("config_version")
        if version is not None and version != self.config_version:
            raise AuthError("配置版本不一致，拒绝运动命令")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return token

    def heartbeat(self, token: str) -> dict:
        if not self.token or token != self.token:
            raise AuthError("token 无效或控制权已释放")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return {"ok": True, "owner": self.owner}

    def is_busy(self) -> bool:
        """20.3：停机线程活着的整段时间都算忙，普通执行线程也算。"""
        if self.park_active:
            return True
        if self._park_thread and self._park_thread.is_alive():
            return True
        if self._exec_thread and self._exec_thread.is_alive():
            return True
        if self.sweep_runner and self.sweep_runner.status.get("active"):
            return True
        if self.program_status.get("active"):
            return True
        return bool(self.motion and self.motion.status.get("busy"))

    def stop_all(self, reason: str = "用户按停", quiet: bool = False,
                 source: str | None = None) -> dict:
        """20.3：统一的停；停机线程、校准、拖动、普通运动都收到。

        37.1 第 3 条：按停记来源。source=web/agent/None → "按停（网页）/
        "按停（agent）/按停（来源没写）"，只替换默认的"用户按停"；心跳超时、
        关网页那些有自己的原因文字，不变。
        """
        src = str(source or "").lower()
        if src == "web":
            label = "按停（网页）"
        elif src == "agent":
            label = "按停（agent）"
        else:
            label = "按停（来源没写）"
        if reason == "用户按停":
            reason = label
        self._mark_operation("stop")
        self._park_stop.set()
        if self.calib.active:
            try:
                self.calib.abort(reason=reason)
            except Exception:  # noqa: BLE001
                pass
        elif self.drag_active:
            try:
                self.drag_stop()
            except Exception:  # noqa: BLE001
                pass
        if self.motion:
            self.motion.stop(reason)
        if not quiet:
            text = reason
            # 37.1 第 4 条：空闲（没在运动）按停时，当场 hold_all 并把 kept 记进事件
            try:
                busy = bool(self.motion and self.motion.status.get("busy"))
            except Exception:  # noqa: BLE001
                busy = False
            if self.bus is not None and not busy:
                try:
                    res = self.bus.hold_all()
                    kept = dict((res or {}).get("kept") or {})
                    if kept:
                        text += "；kept=" + "，".join(
                            f"{n}={float(v):.3f}°" for n, v in kept.items())
                except Exception:  # noqa: BLE001
                    pass
            self.msg = f"已按停：{text}"
            # 29.2：按停要自动存快照；37.1 第 3 条记来源
            if self.session:
                try:
                    self.session.event("user_stop", text)
                    self.session.incident(reason, detail={"原因": text})
                except Exception:  # noqa: BLE001
                    pass
        return {"ok": True}

    def release(self, token: str) -> dict:
        if self.token and token == self.token:
            if self.is_busy():
                self.stop_all("控制权释放", quiet=True)
            self.owner = None
            self.token = None
            self.msg = "控制权已释放"
        return {"ok": True}

    def _watchdog(self):
        while not self._watch_stop.is_set():
            try:
                now = time.monotonic()
                hb = self.last_hb if self.owner else 0.0
                if self.calib.active:
                    # 校准会话：没有控制方、或（自动释放模式）心跳断了 >0.5 s，
                    # 就结束并失能。control_stale=0 时不因心跳断结束会话，
                    # 否则浏览器后台/长确认框会把正在校准的会话踢掉。
                    # 正在执行长命令（设零/点动等）时不打断，操作自己会检查。
                    if (not self.calib.busy) and (
                            (not self.owner) or
                            (self.control_stale > 0 and now - hb > self.hb_timeout)):
                        age = (now - hb) if self.owner else 999.0
                        reason = (f"控制方心跳断了 {age:.2f}s"
                                  f"（>{self.hb_timeout}s），校准会话已结束并失能")
                        self.calib.abort(reason=reason)
                        self.msg = reason
                elif self.drag_active and (
                        (not self.owner) or now - hb > self.hb_timeout):
                    self.drag_stop()
                    self.msg = "拖动时控制方心跳断了，已切回位置保持"
                elif self.owner:
                    busy = self.is_busy()
                    if busy and now - hb > self.hb_timeout:
                        reason = (f"控制方心跳断了 {now-hb:.2f}s"
                                  f"（>{self.hb_timeout}s），已按中止处理：原地保持")
                        self.msg = reason
                        self.stop_all(reason, quiet=True)
                    elif (not busy and self.control_stale > 0
                          and now - hb > self.control_stale):
                        self.owner = None
                        self.token = None
                        self.msg = "控制方心跳超时，控制权已释放"
            except Exception:  # noqa: BLE001
                traceback.print_exc()
            time.sleep(0.05)

    # ---------------- 连接 ----------------
    def connect(self, mode: str, ip: str = "", port: int = 0,
                debug: bool = False) -> dict:
        if self.calib.active:
            raise MotionError("校准会话进行中，先结束校准")
        self.disconnect()
        self.refresh_config_version(force=True)
        self.config_stale = False
        self.config_stale_msg = ""
        ip = ip or self.ecb_ip
        port = int(port or self.ecb_port)
        if mode == "mock":
            self.bus = MockJointBus()
            self.mode = "mock"
            self.ip = "mock"
            rep = self.bus.preflight()
            self.msg = "模拟模式（6 轴）"
            self.connected = True
        else:
            # 37.1 第 6 条自检 ⑦：真 ECB 配置里 idle_park_s 必须 0 < s ≤ 600
            if not (0 < float(self.idle_park_s) <= 600.0):
                raise MotionError(
                    "config/executor.json 的 idle_park_s 必须是 0 < 秒数 ≤ 600"
                    f"（现在是 {self.idle_park_s:g}）：连真 ECB 时不允许关掉或调大，"
                    "要改这个数先问用户（37.1 第 6 条）")
            self.bus = JointBus(ip, port, self.joint_cfg, debug=debug)
            self.mode = "hw"
            self.ip = f"{ip}:{port}"
            rep = self.bus.preflight()
            self.connected = bool(rep.get("ok"))
            if self.connected:
                # 30.6 第 4 条：重连时六颗还使能着 → 接管，不跑启动序列
                self.takeover_report = None
                try:
                    tk = self.bus.try_takeover()
                except Exception as exc:  # noqa: BLE001
                    tk = {"took_over": False, "enabled_count": 0,
                          "msg": f"接管检查失败：{exc}"}
                self.takeover_report = tk
                if tk.get("took_over"):
                    self.msg = str(tk.get("msg") or "已接管，原地保持")
                elif tk.get("enabled_count"):
                    self.msg = str(tk.get("msg") or "臂还使能着，但不能接管")
                else:
                    self.msg = "ECB 已连接，ID↔SN 一致；六颗失能，等待使能"
            else:
                self.msg = "预检未通过：" + "；".join(rep.get("mismatch", []))
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        # 第二十九节 29.1：每次连上开一个会话目录 runs/<日期时间>/
        try:
            self.session = Session(self.runs_dir, tag=self.mode)
        except Exception as exc:  # noqa: BLE001
            self.session = None
            self.msg += f"（会话目录创建失败：{exc}）"
        self.recorder = Recorder(self.runs_dir / f"{stamp}_run.csv",
                                 note=f"connect {self.mode} {self.ip}",
                                 session=self.session)
        self.motion = Motion(self.bus, self.robot, self.fence, self.limits,
                             recorder=self.recorder)
        # A2/A3/A5：读异常回调、写命令记录钩子（真机总线才有）
        if isinstance(self.bus, JointBus):
            self.bus.anomaly_hook = self._on_read_anomaly
            self.bus.write_resend_hook = self._on_write_resend
            self.bus.link.event_hook = self._on_bus_event
            # 29.1 修订（2026-11-26）：本机抓包不依赖 sudo。执行器把发出去的每一帧
            # 转写一份经典 pcap（第二次"独立见证"：走的是发送前的字节，不是 recorder
            # 的行），和 tools/pcap_check.py 读的格式一样。
            try:
                from .pcapio import PacketTee
                tee_dir = self.runs_dir / "_pcap_ring"
                self._packet_tee = PacketTee(tee_dir, tag=self.mode)
                self._packet_tee.start()
                self.bus.link.tee_hook = self._packet_tee.feed
                self.msg += f"；抓包转写：{self._packet_tee.path}"
            except Exception as exc:  # noqa: BLE001
                self._packet_tee = None
                self.msg += f"（抓包转写起不来：{exc}）"
        else:
            self._packet_tee = None
        if self.session and self.mode == "mock" and hasattr(self.bus, "write_log_path"):
            self.bus.write_log_path = str(self.session.dir / "mock_writes.jsonl")

            def _mock_write_event(joint, cmd, value):
                try:
                    self.session.event("cmd", f"0x{int(cmd):02X} 值 {value}",
                                       joint=joint, cmd=int(cmd), value=str(value))
                except Exception:  # noqa: BLE001
                    pass

            self.bus.event_hook = _mock_write_event
        if self.session:
            try:
                self.session.header(
                    connector=f"{self.mode} {self.ip}",
                    config_version=self.config_version,
                    web_version=self._read_web_version(),
                    joints_cfg=self.joint_cfg,
                    bus_stats=self._param_stats(),
                    iface=str(self.cfg.get("ecb_iface") or ""))
                if self.mode == "hw":
                    # 29.1：抓包由 tools/session_start.sh 拉起（要 sudo）；
                    # 这里只接管已经在跑的那个 pid，并写进会话头。
                    from .capture import Capture
                    cap = Capture(self.runs_dir / "_pcap_ring",
                                  iface=str(self.cfg.get("ecb_iface") or ""),
                                  ecb_ip=self.ecb_ip)
                    st_cap = cap.status()
                    if st_cap.get("active"):
                        self._capture = cap
                        self.session.attach_capture(st_cap.get("pid"))
                        self.session.event("capture", "抓包已接管：pid "
                                           f"{st_cap.get('pid')}，"
                                           f"{st_cap.get('files_count')} 个文件")
                    elif bool(self.cfg.get("auto_start_capture", False)):
                        self.session.attach_capture(cap.start().get("pid"))
            except Exception as exc:  # noqa: BLE001
                self.msg += f"（会话头/抓包启动失败：{exc}）"
        if self.session and self.takeover_report is not None:
            try:
                tk = self.takeover_report
                self.session.event(
                    "takeover",
                    ("接管成功：" + str(tk.get("positions") or "")) if tk.get("took_over")
                    else ("未接管：" + str(tk.get("msg") or "")),
                    took_over=bool(tk.get("took_over")))
            except Exception:  # noqa: BLE001
                pass
        if self.session:
            try:
                threading.Thread(target=self.session.enforce_quota,
                                 daemon=True).start()
            except Exception:  # noqa: BLE001
                pass
        self.hold_failure = None
        self.hold_guard = {"active": False, "hist": {}, "resent": {},
                           "resend_dist": {}, "since": None}
        # 22.3 第一阶段：只记录，不参与中止；钩子只入队，计算在后台线程
        self.collision = CollisionMonitor(self)
        self.collision.start()
        self.motion.add_hook(self.collision.feed)
        self.motion.preset = self.tcp_preset
        self.motion.yaw_deg = self.yaw_deg
        self.preview_motion.preset = self.tcp_preset
        self.preview_motion.yaw_deg = self.yaw_deg
        self._sync_motion_params()
        self.start_poll()
        return {"ok": self.connected, "mode": self.mode, "report": rep,
                "msg": self.msg, "config_version": self.config_version}

    def start_trace(self, hz: float = 100.0, seconds: float = 10.0,
                    joints=None, status_hz: float = 5.0, light: bool = False,
                    mode: str = "") -> dict:
        if self.trace and self.trace.status().get("active"):
            raise MotionError("高频采样已经在跑")
        if not self.connected:
            raise MotionError("高频采样要连上（真机或 mock）")
        if not isinstance(self.bus, JointBus) and not getattr(self.bus, "is_mock", False):
            raise MotionError("高频采样需要 JointBus（真机或 mock）")
        self.trace = TraceSampler(self, hz=hz, seconds=seconds, joints=joints,
                                  mode=mode,
                                  status_hz=status_hz, light=light)
        self.trace.start()
        self.msg = f"高频采样开始：{hz:g} Hz / {seconds:g} s"
        return self.trace.status()

    def stop_trace(self) -> dict:
        if not self.trace:
            self._poll_hold = False
            return {"ok": True, "active": False}
        tr = self.trace
        r = tr.stop()
        self._poll_hold = False
        self.msg = "高频采样已停止"
        return {**r, **tr.status()}

    def disconnect(self):
        self._park_stop.set()
        # 37.3 第 2 条查到：_poll_loop 在 disconnect 后一直跑（20 Hz 预演算 FK/围栏），
        # 自检一次进程里反复 connect/disconnect 会越积越多、拖垮慢机器。这里停掉并 join。
        try:
            self._poll_stop.set()
            t = getattr(self, "_poll_thread", None)
            if t is not None and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.0)
        except Exception:  # noqa: BLE001
            pass
        if self.sweep_runner and self.sweep_runner.status.get("active"):
            try:
                self.sweep_runner.stop()
            except Exception:  # noqa: BLE001
                pass
        if self.collision:
            try:
                self.collision.close()
            except Exception:  # noqa: BLE001
                pass
            self.collision = None
        if self.trace:
            try:
                self.trace.stop()
            except Exception:  # noqa: BLE001
                pass
        if self.motion:
            self.motion.stop("断开连接")
        if self._park_thread and self._park_thread.is_alive():
            self._park_thread.join(timeout=2.0)
        if self._exec_thread and self._exec_thread.is_alive() and self.motion:
            self.motion.stop("断开连接")
            self._exec_thread.join(timeout=1.0)
        if self.bus:
            try:
                if getattr(self.bus, "startup_done", False):
                    self.bus.hold_all()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.bus.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._capture is not None:
                self._capture.stop()
                self._capture = None
        except Exception:  # noqa: BLE001
            pass
        # 29.1 修订：断开时把抓包转写收尾，并把统计写进会话（Y14 要报这个）
        tee = getattr(self, "_packet_tee", None)
        if tee is not None:
            try:
                stat = tee.stat()
                tee.close()
                if self.session:
                    self.session.event("capture", "抓包转写收尾：" + str(stat))
            except Exception:  # noqa: BLE001
                pass
            self._packet_tee = None
        if self.recorder:
            self.recorder.close()
        self.session = None
        self.bus = None
        self.motion = None
        self.connected = False
        self.mode = None
        self.config_stale = False
        self.config_stale_msg = ""
        self.cache = {}
        self.preview = None
        self.pending = None
        self._home_preview = None
        self.msg = "未连接"

    # ---------------- 状态轮询 ----------------
    def _poll_loop(self):
        last_cfg_check = 0.0
        last_tick = 0.0
        period = 1.0 / self.push_hz
        while not self._poll_stop.is_set():
            t0 = time.monotonic()
            try:
                now = t0
                if last_tick > 0:
                    inst = 1.0 / max(1e-3, now - last_tick)
                    self._poll_tick_hz = (inst if self._poll_tick_hz <= 0
                                          else 0.8 * self._poll_tick_hz + 0.2 * inst)
                last_tick = now
                if now - last_cfg_check > 1.0:
                    last_cfg_check = now
                    if cfgmod.config_version() != self.config_version:
                        self.refresh_config_version()
                        self.msg = (self.config_stale_msg
                                    or "配置已更新（sign/零位/限位），已热重载")
                self._refresh()
            except Exception as exc:  # noqa: BLE001
                self.msg = f"轮询异常：{exc}"
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _on_trace_latest(self, tr, states):
        """37.1 第 1 条：trace 每轮把读数交给执行器（poll_hold 期间用它跑保护）。"""
        if self.trace is tr:
            self._trace_latest_at = time.time()

    def _on_trace_finished(self, tr):
        """37.1 第 1 条：trace 正常结束/出错/被停 → poll_hold 自动关。"""
        if self.trace is not tr:
            return
        was = bool(getattr(self, "_poll_hold", False))
        self._poll_hold = False
        if was:
            self.msg = (self.msg + "；poll_hold 已自动关闭").strip("；")

    def _refresh(self):
        """37.1 第 1 条改：poll_hold 只停总线读取，保护/网页照常跑。

        - poll_hold 且 trace 在跑、采六颗：用 trace 最近一轮读数，不读总线；
        - 否则正常读总线；poll_hold 没有可用的 trace 时自动关掉。
        """
        tr = self.trace
        hold = bool(getattr(self, "_poll_hold", False))
        use_trace = (hold and tr is not None
                     and tr.status().get("active")
                     and len(tr.latest_states) == len(self.robot.joint_names))
        if use_trace:
            states = {n: tr.latest_states.get(n) for n in self.robot.joint_names}
            q = [None if states[n] is None else states[n].out_deg
                 for n in self.robot.joint_names]
        else:
            if hold:
                self._poll_hold = False
            if self.use_actual():
                m = self.motion
                busy = m.status.get("busy")
                if busy:
                    q = m.status.get("current_q")
                    states = self._last_states
                else:
                    full = time.monotonic() - self._last_full > 5.0
                    states = self.bus.read_all(full=full)
                    if full:
                        self._last_full = time.monotonic()
                    self._last_states = states
                    q = [states[n].out_deg for n in self.robot.joint_names]
                    if any(v is None for v in q):
                        q = None if getattr(self.bus, "startup_done", False) else None
                if q is None:
                    if getattr(self.bus, "startup_done", False):
                        if self._last_states:
                            states = self._last_states
                        bad = [n for n in self.robot.joint_names
                               if states.get(n) is not None
                               and getattr(states[n], "pos_invalid", False)]
                        if bad:
                            self.msg = "读数异常（不合理跳变）：" + "、".join(bad) \
                                       + "；已停止把它当保持目标/规划起点"
                        q = None
                    else:
                        states = self.preview_bus.read_all()
                        q = [0.0] * 6
            else:
                states = self.preview_bus.read_all()
                q = [0.0] * 6
        now = time.monotonic()
        pos_age = {}
        for n in self.robot.joint_names:
            st = states.get(n)
            ts = getattr(st, "pos_read_at", None) if st is not None else None
            age = None if ts is None else max(0.0, time.time() - ts)
            if st is not None:
                st.pos_age_s = age
            pos_age[n] = age
        with self._lock:
            self.cache = {"states": states, "q": q, "pos_age": pos_age}
        self._sync_motion_params()
        self._hold_tick(q)
        self._hold_guard_tick(states, q)
        self._status_tick(states)
        if q is not None:
            try:
                self.cache["scene"] = self.fence.scene(q, self.tcp_preset, self.yaw_deg)
                self.cache["tcp"] = self.robot.tcp(q, self.tcp_preset, self.yaw_deg)[:3, 3].tolist()
                self.cache["flange"] = self.robot.flange(q)[:3, 3].tolist()
                fr = self.fence.check(q, self.tcp_preset, self.yaw_deg, False, self.limits)
                self.cache["fence"] = fr.as_dict()
                self.cache["near_limit"] = fr.near_limit
            except Exception as exc:  # noqa: BLE001
                self.msg = f"场景计算异常：{exc}"
        try:
            self._idle_tick()
        except Exception:  # noqa: BLE001
            pass

    # ---------------- 24 节 A2/A3/A3b/A5 辅助 ----------------
    # 写命令（需要记录的那些）；读命令不记
    WRITE_CMDS = {
        0x07: "模式", 0x08: "电流", 0x09: "速度", 0x0A: "目标位置",
        0x0E: "电流环P", 0x0F: "电流环I", 0x10: "速度环P", 0x11: "速度环I",
        0x12: "位置环P", 0x13: "位置环I", 0x14: "位置环D",
        0x1F: "梯形最大速度", 0x20: "梯形加速度", 0x21: "梯形减速度",
        0x25: "速度梯形最大速度", 0x26: "速度梯形加速度", 0x27: "速度梯形减速度",
        0x2A: "使能/失能", 0x2E: "电流环输出下限", 0x2F: "电流环输出上限",
        0x30: "速度环输出下限", 0x31: "速度环输出上限",
        0x32: "位置环输出下限", 0x33: "位置环输出上限",
        0x0D: "存储参数", 0x70: "电流环滤波器开关", 0x74: "速度环滤波器开关",
        0x78: "位置环滤波器开关", 0x72: "电流环滤波带宽",
        0x76: "速度环滤波带宽", 0x7A: "位置环滤波带宽",
        0x58: "电流限流", 0x5A: "速度限制", 0x7E: "堵转能量",
        0x83: "位置上限", 0x84: "位置下限", 0x87: "Homing 值",
        0x88: "清 Homing", 0x89: "位置偏置", 0x8C: "位置限位开关",
        0x90: "归零电流下限", 0x91: "归零电流上限", 0xFE: "清报警",
    }

    def _decode_write(self, cmd: int, data: bytes, joint: str) -> str:
        """把写命令的 data 解成便于回报的值；解不出就报原始十六进制。"""
        hexs = data.hex() or "-"
        if not data:
            return "（无数据）"
        try:
            if cmd in (0x07, 0x2A, 0x70, 0x74, 0x78, 0x8C) and len(data) == 1:
                return f"{data[0]}"
            if cmd == 0x0A and len(data) == 4:
                turns = struct.unpack(">i", data)[0] / (2.0 ** 24)
                cfg = self.joint_cfg.get(joint)
                deg = None if cfg is None else cfg.out_deg(turns)
                return f"原始 {hexs} = {turns:.6f} 圈 = {deg:.3f}°" if deg is not None \
                    else f"原始 {hexs} = {turns:.6f} 圈"
            if cmd in (0x1F, 0x20, 0x21, 0x25, 0x26, 0x27) and len(data) == 4:
                from .sca import PROFILE_SCALE
                rpm = struct.unpack(">i", data)[0] / (2.0 ** 24) * PROFILE_SCALE
                return f"原始 {hexs} = {rpm:.3f} RPM = {rpm * 10.0 / 60.0:.3f}°/s"
            if cmd in (0x83, 0x84) and len(data) == 4:
                turns = struct.unpack(">i", data)[0] / (2.0 ** 24)
                return f"原始 {hexs} = {turns:.6f} 圈 = {turns * 10.0:.2f}°"
            if cmd == 0xFE:
                return f"0x{data.hex()}"
            if len(data) == 4:
                raw = struct.unpack(">i", data)[0]
                return f"原始 {hexs} = int32 {raw} = IQ24 {raw / (2.0 ** 24):.6f}"
        except Exception:  # noqa: BLE001
            pass
        return f"原始 {hexs}"

    def _packet_tee_stat(self) -> dict:
        """抓包转写的现状（网页/回报里看）；没开就返回空。"""
        tee = getattr(self, "_packet_tee", None)
        if tee is None:
            return {}
        try:
            return tee.stat()
        except Exception:  # noqa: BLE001
            return {}

    def _on_bus_event(self, addr: int, cmd: int, data: bytes, ok):
        """A5：EcbTransport 的写命令钩子。只记写命令，读不回灌记录。"""
        if cmd not in self.WRITE_CMDS:
            return
        if self.recorder is None:
            return
        joint = "?"
        try:
            for n, m in self.bus.motors.items():
                if m.mid == addr:
                    joint = n
                    break
        except Exception:  # noqa: BLE001
            pass
        if joint == "?":
            joint = f"ID{addr}"
        try:
            self.recorder.write_cmd(joint, cmd, self._decode_write(cmd, data, joint),
                                    ok, self.WRITE_CMDS.get(cmd, ""))
        except Exception:  # noqa: BLE001
            pass

    def _on_read_anomaly(self, name, raw_hex, raw_deg, prev_deg, dt):
        """A3：把原始回包十六进制记进日志，并在网页提示。"""
        txt = (f"{name} 读数异常：原始 {raw_hex or '（单条读回，无原始帧）'} "
               f"换算 {raw_deg:.3f}°")
        if prev_deg is not None:
            txt += f"，上一次可信 {prev_deg:.3f}°"
        if dt is not None and math.isfinite(dt):
            txt += f"，间隔 {dt:.2f} s"
        self.msg = txt
        if self.recorder:
            try:
                self.recorder.command("read_anomaly", txt, joint=name,
                                      target_deg=raw_deg)
            except Exception:  # noqa: BLE001
                pass

    def _on_write_resend(self, names, targets):
        txt = ("写目标后 0.06 s 没见速度朝目标起来，已重发一次："
               + "、".join(f"{n}→{float(targets.get(n, 0.0)):.2f}°" for n in names))
        self.msg = txt
        if self.recorder:
            try:
                self.recorder.command("write_resend", txt)
            except Exception:  # noqa: BLE001
                pass

    def _status_tick(self, states):
        """A5：电源/模式/报警 5 Hz 落盘；回读节拍掉到 15 Hz 以下时降到 2 Hz。

        第二十九节：不运动时（包括 mock）也要把逐拍状态写进会话（29.1 第 2 条）。
        """
        if self.recorder is None:
            return
        if self.session is not None and not self._session_busy():
            now_s = time.monotonic()
            if now_s - getattr(self, "_last_session_state_at", 0.0) >= 0.1:
                self._last_session_state_at = now_s
                try:
                    snap = dict(self._last_states or {})
                    for n, st in list(snap.items()):
                        tgt = self._targets.get(n) if hasattr(self, "_targets") else None
                        if tgt is not None:
                            st.target_deg = tgt
                    self.session.state(snap)
                except Exception:  # noqa: BLE001
                    pass
        if not isinstance(self.bus, JointBus):
            return
        if not getattr(self.bus, "startup_done", False):
            return
        now = time.monotonic()
        period = self._status_period_s
        if getattr(self, "_poll_tick_hz", 0.0) and self._poll_tick_hz < 15.0:
            period = 0.5
        if now - self._last_status_at < period:
            return
        self._last_status_at = now
        try:
            rows = self.bus.read_status()
        except Exception as exc:  # noqa: BLE001
            self.msg = f"状态回读（0x2B/0x55/0xFF）失败：{exc}"
            return
        for n, row in rows.items():
            st = (states or {}).get(n) or self._last_states.get(n)
            if st is None:
                continue
            if row.get("mode") is not None:
                st.mode = row["mode"]
                st.mode_read_at = time.time()
            if row.get("error") is not None:
                prev_err = self._prev_errors.get(n)
                st.error = row["error"]
                self._prev_errors[n] = row["error"]
                # 29.2 第 1 条：报警码变化要记事件；非 0 时存快照
                if self.session is not None and row["error"] != prev_err:
                    try:
                        self.session.event(
                            "alarm_change",
                            f"报警 0x{int(row['error']):04X}（上一拍 "
                            f"{'—' if prev_err is None else f'0x{int(prev_err):04X}'}）",
                            joint=n)
                        if int(row["error"]):
                            self.session.incident(
                                "报警码非 0", joint=n,
                                detail={"报警": f"0x{int(row['error']):04X}"})
                    except Exception:  # noqa: BLE001
                        pass
            if row.get("power") is not None:
                st.enabled = (row["power"] == 1)
        try:
            self.recorder.status(rows)
        except Exception:  # noqa: BLE001
            pass


    def _hold_guard_reset(self):
        self.hold_guard = {"active": False, "hist": {}, "resent": {},
                           "resend_dist": {}, "since": None}

    def _hold_guard_tick(self, states, q):
        """A3b：保持失效检测。

        保持状态（中止以后、到位以后、停机等待时）里，若可信读数离保持目标
        超过 3° 并且在 0.3 s 内继续远离：先重发一次“模式 6 + 保持目标”；
        0.5 s 后还在远离 → 判保持失效（红色横幅、停收新命令、不自动失能）。
        """
        if self.hold_failure:
            return
        if self.bus is None or self.motion is None:
            return
        busy = bool(self.motion.status.get("busy")) or self.park_active
        if busy or self.drag_active or self.calib.active or self.park_block:
            self._hold_guard_reset()
            return
        if not getattr(self.bus, "startup_done", False):
            self._hold_guard_reset()
            return
        targets = dict(getattr(self.bus, "_targets", {}) or {})
        if not targets:
            self._hold_guard_reset()
            return
        g = self.hold_guard
        now = time.monotonic()
        if not g["active"]:
            g["active"] = True
            g["since"] = now
        for name in self.robot.joint_names:
            tgt = targets.get(name)
            st = states.get(name) if states else None
            if tgt is None or st is None:
                continue
            val = st.out_deg
            if val is None or getattr(st, "pos_invalid", False):
                continue
            dist = abs(float(val) - float(tgt))
            hist = g["hist"].setdefault(name, [])
            hist.append((now, dist))
            while hist and now - hist[0][0] > 1.5:
                hist.pop(0)
            if dist <= 3.0:
                g["resent"].pop(name, None)
                g["resend_dist"].pop(name, None)
                continue
            old = None
            for t, d in hist:
                if now - t >= 0.3:
                    old = d
                else:
                    break
            away = old is not None and dist > old + 0.3
            if not away:
                continue
            resent = g["resent"].get(name)
            if resent is None:
                self._hold_resend(name, tgt)
                g["resent"][name] = now
                g["resend_dist"][name] = dist
                if self.recorder:
                    try:
                        self.recorder.command(
                            "hold_resend",
                            f"{name} 保持偏 {dist:.2f}° 且仍在远离，已重发模式 6 + 保持目标 "
                            f"{tgt:.3f}°", joint=name, target_deg=tgt)
                    except Exception:  # noqa: BLE001
                        pass
            elif now - resent >= 0.5 and dist > g["resend_dist"].get(name, dist) + 0.3:
                d = float(val) - float(tgt)
                direction = "正方向" if d > 0 else "负方向"
                self.hold_failure = {
                    "active": True, "joint": name, "direction": direction,
                    "offset_deg": round(d, 2), "dist_deg": round(dist, 2),
                    "target_deg": round(float(tgt), 3),
                    "actual_deg": round(float(val), 3),
                    "since": time.time(), "disabled": [],
                    "msg": (f"保持失效：{name} 往{direction}偏了 {dist:.1f}°（目标 "
                            f"{float(tgt):.1f}°，实际 {float(val):.1f}°）。"
                            "扶住这一段，再决定是否单颗失能或拉空开。"),
                }
                self.msg = self.hold_failure["msg"]
                if self.recorder:
                    try:
                        self.recorder.command("hold_failure",
                                              self.hold_failure["msg"], joint=name,
                                              target_deg=float(tgt),
                                              pos_deg=float(val))
                    except Exception:  # noqa: BLE001
                        pass
                return

    def _hold_resend(self, name: str, target_deg: float):
        """重发“模式 6 + 保持目标”（A3b 第 1 条）。"""
        try:
            m = self.bus.motors[name]
            cfg = self.bus.cfg[name]
            m.set_target_turns(cfg.turns(target_deg))
            m.set_mode(MODE_PROFILE_POSITION)
            m.set_target_turns(cfg.turns(target_deg))
            self.bus._targets[name] = target_deg
        except Exception as exc:  # noqa: BLE001
            self.msg = f"{name} 保持重发失败：{exc}"

    def hold_disable_joint(self, joint: str, confirmed: bool = False) -> dict:
        """A3b 第 3 条：只在保持失效状态下出现，单颗失能要二次确认。"""
        if not self.hold_failure:
            raise MotionError("当前没有保持失效")
        if not confirmed:
            raise MotionError("单颗失能要二次确认：提示臂会掉，先扶住这一段")
        j = str(joint or "").upper()
        if j != self.hold_failure.get("joint"):
            raise MotionError(f"只有报保持失效的 {self.hold_failure.get('joint')} 可以单颗失能")
        if self.motion and self.motion.status.get("busy"):
            raise MotionError("还在运动，先按停")
        ok = self.bus.disable_one(j)
        self.hold_failure["disabled"].append(j)
        self.hold_failure["msg"] = (f"{j} 已单颗失能；其余关节保持。"
                                    "现在只能扶持停机或拉直流空开。")
        self.msg = self.hold_failure["msg"]
        if self.recorder:
            try:
                self.recorder.command("disable_joint", f"保持失效：单颗失能 {j}",
                                      joint=j, power=0)
            except Exception:  # noqa: BLE001
                pass
        self.hold_guard = {"active": False, "hist": {}, "resent": {},
                           "resend_dist": {}, "since": None}
        return {"ok": bool(ok), "msg": self.msg}

    # ---------------- 状态 ----------------
    def save_web_state(self, snap: dict) -> dict:
        """页面运行时快照：只保留最新一份，覆盖写 runs/web_state.json。"""
        if not isinstance(snap, dict):
            raise MotionError("page_state 必须是 JSON object")
        snap = dict(snap)
        snap["server_received_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        raw = json.dumps(snap, ensure_ascii=False, allow_nan=False)
        if len(raw.encode("utf-8")) > 2_000_000:
            raise MotionError("page_state 超过 2 MB，拒绝")
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        path = self.runs_dir / "web_state.json"
        tmp = path.with_suffix(".json.tmp")
        with self._web_state_lock:
            tmp.write_text(raw + "\n", encoding="utf-8")
            tmp.replace(path)
        return {"ok": True, "path": str(path), "bytes": len(raw.encode("utf-8"))}

    def load_web_state(self) -> dict:
        path = self.runs_dir / "web_state.json"
        if not path.is_file():
            return {"ok": False, "error": "还没有 web_state.json；先让网页刷新并保持打开"}
        try:
            return {"ok": True, **json.loads(path.read_text(encoding="utf-8"))}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    # ---------------- 37.1 第 6 条：空闲自动停机 ----------------
    def _mark_operation(self, what: str = ""):
        """记一次“会改变臂状态的操作”：空闲计时从头算。"""
        self._last_op_at = time.monotonic()
        self._idle_triggered = False
        self._idle_blocked_reason = ""

    def idle_continue(self) -> dict:
        """网页“继续保持”按钮：把空闲计时清零。"""
        self._mark_operation("idle_continue")
        return {"ok": True, "msg": "空闲计时已从头算",
                **self._idle_state()}

    def _idle_state(self) -> dict:
        limit = float(getattr(self, "idle_park_s", 600.0))
        elapsed = time.monotonic() - getattr(self, "_last_op_at", time.monotonic())
        try:
            st = self.cache.get("states") or {}
            six_on = bool(getattr(self.bus, "startup_done", False)) and all(
                st.get(n) is not None and st[n].enabled is True
                for n in self.robot.joint_names)
        except Exception:  # noqa: BLE001
            six_on = False
        try:
            busy = bool(self.motion and self.motion.status.get("busy")) \
                or bool(self.park_active) or bool(getattr(self, "drag_active", False)) \
                or bool(getattr(self.calib, "active", False))
        except Exception:  # noqa: BLE001
            busy = False
        enabled = bool(self.connected and self.bus is not None and six_on and not busy)
        remain = max(0.0, limit - elapsed)
        return {"enabled": enabled, "remaining_s": round(remain, 1),
                "limit_s": round(limit, 1), "park_s": round(remain, 1),
                "blocked": self._idle_blocked_reason,
                "triggered": bool(self._idle_triggered)}

    def _idle_abnormal_reason(self) -> str | None:
        """空闲到点时如果状态异常，返回原因；正常返回 None（不停机）。"""
        if self.hold_failure:
            return "保持失效"
        if self.park_block:
            return "启动比对还没解除"
        try:
            lat = self.check_alarm_latch()
            if not lat.get("ok"):
                return "报警码非 0：" + "，".join(lat.get("bad") or [])
        except Exception as exc:  # noqa: BLE001
            return f"读报警失败：{exc}"
        try:
            st = self.cache.get("states") or {}
            bad = [n for n in self.robot.joint_names
                   if st.get(n) is not None and getattr(st[n], "pos_invalid", False)]
            if bad:
                return "读数异常：" + "、".join(bad)
        except Exception:  # noqa: BLE001
            pass
        if self.end_effector != "flange":
            return f"末端是 {self.end_effector}，停机只对空法兰开放"
        try:
            cons = self.park_config().get("park_consistency")
            if cons and not cons.get("ok"):
                return "停机点配置对不上：" + "；".join(cons.get("problems") or [])
        except Exception as exc:  # noqa: BLE001
            return f"停机点检查失败：{exc}"
        return None

    def _idle_tick(self):
        """在 _refresh 里每拍检查一次；到点自动停机/失能（用户 2026-11-26 定）。"""
        if self._idle_triggered or self._idle_running or self._idle_blocked_reason:
            return
        if not self.connected or self.bus is None:
            return
        if not getattr(self.bus, "startup_done", False):
            return
        try:
            st = self.cache.get("states") or {}
            if not all(st.get(n) is not None and st[n].enabled is True
                       for n in self.robot.joint_names):
                return
        except Exception:  # noqa: BLE001
            return
        if (self.motion and self.motion.status.get("busy")) or self.park_active \
                or getattr(self, "drag_active", False) or getattr(self.calib, "active", False):
            return
        limit = float(getattr(self, "idle_park_s", 600.0) or 600.0)
        if time.monotonic() - getattr(self, "_last_op_at", time.monotonic()) < limit:
            return
        why = self._idle_abnormal_reason()
        if why:
            self._idle_blocked_reason = why
            text = f"空闲超时，状态异常（{why}），没有自动停机"
            self.msg = text
            if self.recorder:
                try:
                    self.recorder.command("idle_blocked", text)
                except Exception:  # noqa: BLE001
                    pass
            return
        self._idle_triggered = True
        self._idle_last_trigger_at = time.monotonic()
        on_shelf = bool(self._on_shelf)
        if on_shelf:
            text = "空闲 10 分钟，自动失能（臂在搁架上）"
            self.msg = text
            if self.recorder:
                try:
                    self.recorder.command("idle_auto_disable", text)
                except Exception:  # noqa: BLE001
                    pass
            try:
                self.bus.disable_all(True, "空闲 10 分钟，自动失能")
            except Exception as exc:  # noqa: BLE001
                self.msg = f"{text}：失败 {exc}"
            return
        # 臂在空中：走正常停机流程，速度 25%
        text = "空闲 10 分钟，自动停机"
        self.msg = text
        if self.recorder:
            try:
                self.recorder.command("idle_auto_park", text)
            except Exception:  # noqa: BLE001
                pass

        def worker():
            self._idle_running = True
            try:
                if self.motion and self.motion.status.get("phase") in (
                        "aborted", "error", "stopping"):
                    self.recover()   # 规则同网页：按钮要先恢复就先恢复
                prev = self.speed_scale
                self.speed_scale = min(0.25, prev)
                try:
                    if self.motion:
                        self.motion.speed_scale = self.speed_scale
                    self.park(False, prereq={"gripper_empty": True,
                                             "workspace_clear": True,
                                             "shelf_ready": True})
                finally:
                    self.speed_scale = prev
                    if self.motion:
                        self.motion.speed_scale = prev
            except Exception as exc:  # noqa: BLE001
                self.msg = f"空闲自动停机失败：{exc}"
                if self.recorder:
                    try:
                        self.recorder.command(
                            "idle_blocked",
                            f"空闲自动停机失败：{exc}；按停机流程规则原地保持")
                    except Exception:  # noqa: BLE001
                        pass
            finally:
                self._idle_running = False

        threading.Thread(target=worker, daemon=True).start()

    def state(self) -> dict:
        with self._lock:
            cache = dict(self.cache)
            states = cache.get("states") or {}
            q = cache.get("q")
            m_state = self.motion if self.use_actual() else self.preview_motion
            ms = dict(m_state.status) if m_state else {}
            preview = self.preview
            pending = bool(self.pending)
        ms.pop("segments", None)
        joints = []
        now_wall = time.time()
        for n in self.robot.joint_names:
            st = states.get(n)
            if st is not None:
                st.mode_age_s = (None if getattr(st, "mode_read_at", None) is None
                                 else max(0.0, now_wall - st.mode_read_at))
            joints.append(st.as_dict() if st is not None else
                          {"name": n, "id": self.joint_cfg[n].id,
                           "enabled": None, "out_deg": None})
        # 六关节安全限位检查：实际位置只有在 startup_done 后才是真机角；
        # 未使能时固件不回位置，页面只显示“待启动检查”，不拿预演 q=0 冒充。
        startup = bool(getattr(self.bus, "startup_done", False))
        q_actual = cache.get("q")
        if not self.connected:
            safe_check = {"ok": None, "state": "offline",
                          "text": "未连接：连接后实际角度要等零扭矩启动读取"}
        elif not startup or q_actual is None:
            safe_check = {"ok": None, "state": "unknown",
                          "text": "未使能：实际角度由启动序列逐颗零扭矩读取；"
                                  "超安全限位会拒绝使能"}
        else:
            lo, hi = self.limits
            rows = []
            bad = []
            for i, n in enumerate(self.robot.joint_names):
                qv = float(q_actual[i])
                a, b = float(lo[i]), float(hi[i])
                ok = a - 1e-6 <= qv <= b + 1e-6
                rows.append({"joint": n, "q": round(qv, 2),
                             "lo": round(a, 1), "hi": round(b, 1), "ok": ok})
                if not ok:
                    bad.append(f"{n} {qv:+.1f}°（{a:.0f}..{b:.0f}）")
            safe_check = {
                "ok": not bad, "state": "ok" if not bad else "bad",
                "rows": rows,
                "text": ("六关节安全限位正常" if not bad else
                         "超安全限位：" + "，".join(bad)
                         + "；禁止使能/运动，先断电手动移回"),
            }
        now = time.monotonic()
        # 29.1 第 3 条：把当前流程/步号同步给会话（事故快照里要写）
        if self.session is not None:
            try:
                self.session.set_ctx(
                    phase=str((self.motion.status or {}).get("phase") or ""),
                    label=str((self.motion.status or {}).get("label") or ""),
                    park_step=str((self.park_step or {}).get("phase") or ""))
            except Exception:  # noqa: BLE001
                pass
        return {
            "ok": True, "ts": time.time(),
            "connected": self.connected, "mode": self.mode,
            "mock": self.mode == "mock", "ip": self.ip, "msg": self.msg,
            "startup_done": bool(getattr(self.bus, "startup_done", False)),
            "q": q, "tcp": cache.get("tcp"), "flange": cache.get("flange"),
            "scene": cache.get("scene"), "fence": cache.get("fence"),
            "near_limit": cache.get("near_limit", []), "joints": joints,
            "safe_check": safe_check,
            "motion": ms, "tcp_preset": self.tcp_preset, "yaw": self.yaw_deg,
            "limits": {"lo": self.limits[0], "hi": self.limits[1]},
            "preview": preview, "has_plan": pending,
            "recorder": str(self.recorder.path) if self.recorder else None,
            "stats": (self.bus.stats() if self.bus else None),
            "trace": (self.trace.status() if self.trace else
                      {"active": False, "hz": 0, "seconds": 0, "samples": 0,
                       "actual_hz": 0, "started": None, "finished": None,
                       "path": None, "error": ""}),
            "exec_mode": self.exec_mode,
            "config_version": self.config_version,
            "calib": self.calib.snapshot(),
            "drag": {"active": bool(self.drag_active)},
            "measurements": self.measurements[-20:],
            "startup_report": (getattr(self.bus, "startup_report", None)
                               if self.bus else None),
            "end_effector": self.end_effector,
            "payload_kg": self.payload_kg,
            "capacities": self.capacities(),
            "hold_failure": (dict(self.hold_failure) if self.hold_failure else None),
            "ecb_lock": (self._ecb_lock_status()),
            "takeover": (dict(self.takeover_report) if self.takeover_report else None),
            "disabled_latch": (self.bus.latch_status()
                               if self.bus and hasattr(self.bus, "latch_status")
                               else {"disabled_latch": False, "msg": ""}),
            "hold_guard": {"active": bool(self.hold_guard.get("active")),
                           "since_s": (None if self.hold_guard.get("since") is None
                                       else round(time.monotonic() - self.hold_guard["since"], 1)),
                           "resent": sorted(self.hold_guard.get("resent", {}).keys())},
            "read_tick_hz": round(float(getattr(self, "_poll_tick_hz", 0.0) or 0.0), 1),
            "hold": {"ratio": self.hold.get("ratio", 0.0),
                     "since_s": (None if self.hold.get("since") is None else
                                 round(time.monotonic() - self.hold["since"], 1)),
                     "warn": self.hold.get("warn", ""),
                     "threshold": gravity.HOLD_WARN_RATIO},
            "idle": self._idle_state(),
            "flow": {"phase": (self.motion.status.get("phase")
                               if self.motion else ""),
                     "label": (self.motion.status.get("label")
                               if self.motion else ""),
                     "park_phase": str((self.park_step or {}).get("phase") or "")},
            "park": {**self.park_config(),
                     "step": dict(self.park_step),
                     "check": self._park_check(q),
                     "block": self.park_block},
            "kt": {"rows": self.kt_rows, "kt": self.kt},
            "sweep": (dict(getattr(self.sweep_runner, "status", {}))
                      if self.sweep_runner else
                      {"active": False, "phase": "idle", "joint": None,
                       "samples": 0, "fit": None}),
            "programs": {"names": list(self._program_names_cache),
                         "recorded": self.program_points,
                         "loaded": (None if not self.program_loaded else
                                    {"name": self.program_loaded.get("name"),
                                     "points": len(self.program_loaded.get("points") or []),
                                     "end_effector": self.program_loaded.get("end_effector"),
                                     "payload_kg": self.program_loaded.get("payload_kg")}),
                         "replay": dict(self.program_status)},
            "collision": (dict(self.collision.status) if self.collision else
                          {"enabled": False, "phase": "idle", "flags": 0}),
            "control": {
                "owner": self.owner,
                "age_s": None if not self.owner else round(now - self.last_hb, 2),
                "stale_s": self.control_stale,
                "timeout_s": self.hb_timeout,
            },
            "config_stale": bool(self.config_stale),
            "config_stale_msg": self.config_stale_msg,
            "limits_used": self.limit_used(),
        }

    def calibrated(self) -> tuple[bool, list[str]]:
        if self.mode == "mock":
            return True, []
        cfg = load_joint_configs()
        missing = [n for n, c in cfg.items() if not (c.zero_date and c.sign_checked)]
        return (not missing), missing

    # ---------------- 规划 ----------------
    def _ready_for_motion(self, allow_hold: bool = False):
        if self.hold_failure:
            raise MotionError(
                "保持失效中，已停止接受新的运动命令：" + self.hold_failure.get("msg", "")
                + "（其余关节照常保持；先决定单颗失能或拉空开/扶持停机）")
        if self.park_active or (self._park_thread and self._park_thread.is_alive()):
            raise MotionError("停机流程正在跑：先按停，等它停在保持状态")
        if self.config_stale:
            raise MotionError(self.config_stale_msg or "配置已变，请重新连接")
        if self.drag_active:
            raise MotionError("零扭矩拖动中，先“锁定保持”再执行")
        if self.sweep_runner and self.sweep_runner.status.get("active"):
            raise MotionError("22.2 扫描标定进行中，先按停")
        if self.program_status.get("active"):
            raise MotionError("22.4 程序回放进行中，先按停")
        if self.calib.active:
            raise MotionError("校准会话进行中，先结束校准")
        if not self.motion or not self.connected:
            raise MotionError("还没连接")
        if not getattr(self.bus, "startup_done", False):
            if hasattr(self.bus, "latch_status") and self.bus.latch_status().get("disabled_latch"):
                raise MotionError(
                    "已经失能过：按 27.2 第 3 条这一次上电里不再使能。"
                    "请拉空开断电后重上电")
            raise MotionError("还没完成启动/使能，不能执行")
        if self.park_block:
            raise MotionError(
                (str(self.park_block.get("reason")) + "：")
                if self.park_block.get("reason") else
                f"启停比对不过（和 parked_q 差 {self.park_block.get('max_diff_deg')}°）："
                "看过实物、确认没问题后点“已确认”才解除")
        phase = (self.motion.status or {}).get("phase")
        if phase in ("aborted", "error") and not allow_hold:
            raise MotionError("臂在中止保持状态：先点“恢复”，或走“停机 / 扶持停机”")
        ok, missing = self.calibrated()
        if not ok:
            raise MotionError("还没完成设零/方向核对：缺 " + ", ".join(missing))

    def _reject_if_parking(self):
        if self.park_active or (self._park_thread and self._park_thread.is_alive()):
            raise MotionError("停机流程正在跑：先按停，等它停在保持状态")

    # ---------------- 18.1 / 18.2 搁架、末端配置、力矩 ----------------
    def _save_park_state_safe(self, state: dict) -> None:
        """保存停车状态；mock 不写真实 config（防止自检污染真机配置）。"""
        if getattr(self.bus, "is_mock", False):
            return
        try:
            cfgmod.save_park_state(state)
        except Exception:  # noqa: BLE001
            pass

    def park_config(self) -> dict:
        """A9/18.3：收拢点、预停机点的 q1 都取教的停机位置；park_side 由它推出来。

        三者对不上（含配置里另存的 park_side 和 q1 符号相反）就写进
        ``park_consistency``，由 park() 拒绝停机。
        """
        pk = json.loads(json.dumps(self.park_cfg))   # deep copy
        taught = bool(pk.get("park_taught"))
        raw_poses = pk.get("poses") or {}
        problems: list[str] = []
        if taught:
            qp = list(raw_poses.get("park") or [0] * 6)
            side = -1 if float(qp[0]) < 0 else 1
            q1 = float(qp[0])
            stored = pk.get("park_side")
            if stored is not None and int(stored) != side:
                problems.append(
                    f"配置里另存的 park_side={int(stored)} 和教的停机位置 q1={q1:.2f}°"
                    f"（{'+y' if side > 0 else '−y'} 侧）相反")
            # 收拢点、预停机点的 q1 必须和教的 q1 对得上（容差 2°）
            for key in ("tuck", "pre"):
                q = list(raw_poses.get(key) or [0] * 6)
                if abs(float(q[0]) - q1) > 2.0:
                    problems.append(
                        f"{key} 的 q1={float(q[0]):.2f}° 和教的停机位置 q1={q1:.2f}° 差"
                        f"{abs(float(q[0]) - q1):.1f}°")
        else:
            side = int(pk.get("park_side", -1))
            q1 = 90.0 * side
        pk["park_side"] = side
        sh = pk.get("shelf") or {}
        if side > 0 and sh.get("center"):
            sh["center"] = [-float(sh["center"][0]), -float(sh["center"][1])]
            sh["rotate_deg"] = 90.0
        pk["shelf"] = sh
        poses = {}
        for key in ("tuck", "pre", "park"):
            q = list(raw_poses.get(key) or [0] * 6)
            if taught:
                q[0] = q1
            else:
                q[0] = q1
            poses[key] = q
        pk["poses"] = poses
        pk["park_consistency"] = {"ok": not problems, "problems": problems}
        pk["contact_j4_z"] = pk.get("contact_j4_z")
        pk["contact_j4_z_at"] = pk.get("contact_j4_z_at")
        z_min = float(pk.get("descent_min_j4_z") or 130.5)
        step = float(pk.get("descent_step_mm") or 1.0)
        pk["descent"] = {
            "start_z": 168.0, "min_z": z_min, "step_mm": step,
            "note": ("教的点：J1 决定左右侧、J4–J6 决定腕姿；下降高度按固定公式从 z=168 到 "
                     f"{z_min:.1f}，每 {step:g} mm 一步"),
        }
        return pk

    def _session_busy(self) -> bool:
        """运动中由运动层写逐拍行；空闲态才由状态线程补写，避免混进缓存旧值。"""
        try:
            if self.motion and self.motion.status.get("busy"):
                return True
            if self.park_active or self.drag_active or self.calib.active:
                return True
        except Exception:  # noqa: BLE001
            pass
        return False

    # ---------------- 27.3/30.3 测试专用（不影响正常流程） ----------------
    def test_write_profile(self, q, negative_decel: bool | None = None) -> dict:
        """只写梯形参数（0x1F/0x20/0x21），不写目标、不切模式、不使能。

        36.1 第 1 条：正常流程一律写负减速度。不传 negative_decel 时写负数；
        35.3 第 5 条留着的对照选项：显式传 False 才写正数（测试用，正常流程不用）。
        只写内存（不发 0x0D 存 EEPROM）。
        35.2 第 2 条：本段不动的关节（|位移| ≤0.8°）加速度/减速度给下限。
        """
        self._reject_if_parking()
        self._mark_operation("test_write_profile")
        if self.motion is None or not self._bus_startup_done():
            raise MotionError("还没完成启动/使能，不能做这个测试")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动")
        segs = self.motion.plan_joint_move(q, label="test_write_profile")
        seg = segs[-1]
        q_now = self.motion.read_q()
        wrote = {}
        for n in self.bus.order:
            c = self.bus.cfg[n]
            disp = float((seg.get("q_target") or [0] * 6)[self.robot.joint_names.index(n)]
                         - q_now[self.robot.joint_names.index(n)])
            vel = float((seg.get("vel_deg_s") or {}).get(n, c.motion_speed_deg_s))
            acc = float((seg.get("acc_deg_s2") or {}).get(n,
                        vel / max(c.motion_accel_time_s, 0.05)))
            if abs(disp) <= 0.8:
                floor = c.motion_speed_deg_s / max(c.motion_accel_time_s, 0.05)
                acc = max(acc, floor)
            dec = acc if negative_decel is False else -acc
            try:
                self.bus.motors[n].set_profile_out(vel, acc, dec)
                wrote[n] = {"vel_deg_s": round(vel, 3), "acc_deg_s2": round(acc, 3),
                            "dec_deg_s2": round(dec, 3), "disp_deg": round(disp, 3)}
            except Exception as exc:  # noqa: BLE001
                wrote[n] = {"error": str(exc)}
        time.sleep(0.05)
        if self.recorder:
            try:
                self.recorder.command(
                    "test_write_profile",
                    f"negative_decel={negative_decel is not False} " + str(wrote))
            except Exception:  # noqa: BLE001
                pass
        return {"ok": True, "wrote": wrote,
                "negative_decel": negative_decel is not False,
                "msg": "只写了梯形参数，没有写目标"}

    def test_read_params(self, joints=None, cmds=None) -> dict:
        """35.3：只读若干原始参数（0x1B/0x7B/0x77/0x1C/0x1D/0x1E/0x53/0x0D/0x06/0x04/0x05）。

        返回每颗每条的原始 hex；0x1C/1D/1E 另给 °/s 换算，0x53 给 A。
        """
        if self.bus is None:
            raise MotionError("还没连接")
        js = [str(x).upper() for x in (joints or self.bus.order)]
        cs = [int(x, 0) if isinstance(x, str) else int(x)
              for x in (cmds or (0x1B, 0x7B, 0x77, 0x1C, 0x1D, 0x1E, 0x53))]
        # 只读白名单：0x0D 是“存储参数”写命令，绝不能当只读发（2026-11-24 误发过）
        read_only = {0x04, 0x05, 0x06, 0x1B, 0x1C, 0x1D, 0x1E, 0x2B, 0x53,
                     0x55, 0x77, 0x7B, 0xFF}
        bad = [hex(c) for c in cs if c not in read_only]
        if bad:
            raise MotionError(f"这些不是只读命令，拒绝发送：{', '.join(bad)}")
        out = {}
        for n in js:
            m = self.bus.motors[n]
            c = self.bus.cfg[n]
            out[n] = {}
            for cmd in cs:
                # 逐条读（一次读太多条在真总线上会整批超时）；读不到就 null
                nbytes = {0x53: 2, 0x2B: 1, 0x55: 1, 0xFF: 2}.get(cmd, 4)
                d = None
                for _ in range(3):
                    try:
                        d = self.bus.link.read(m.mid, cmd, nbytes, timeout=0.12, retries=1)
                    except Exception:  # noqa: BLE001
                        d = None
                    if d and len(d) >= nbytes:
                        break
                row = {"raw_hex": None if d is None else d.hex()}
                if d and len(d) >= 2:
                    if cmd in (0x1C, 0x1D, 0x1E) and len(d) >= 4:
                        raw = struct.unpack(">i", d)[0]
                        rpm = raw / float(1 << 24) * 960.0
                        row.update({"raw_iq24": raw, "rpm": round(rpm, 3),
                                    "deg_s": round(rpm * 10.0 / 60.0, 3)})
                    elif cmd == 0x53 and len(d) >= 2:
                        raw = struct.unpack(">h", d)[0]
                        row.update({"raw_iq8": raw, "a": round(raw / 256.0, 3)})
                    elif cmd == 0x06 and len(d) >= 4:
                        raw = struct.unpack(">i", d)[0]
                        row["pos_deg"] = round(c.out_deg(raw / float(1 << 24)), 3)
                out[n][hex(cmd)] = row
        return {"ok": True, "rows": out}

    def test_write_targets(self, q) -> dict:
        """30.3 e3 第二步 / e4：只写目标位置（0x0A），不重发、不切模式。"""
        self._reject_if_parking()
        if self.motion is None or not self._bus_startup_done():
            raise MotionError("还没完成启动/使能，不能做这个测试")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动")
        q = [float(v) for v in q]
        if len(q) != 6:
            raise MotionError("需要 6 个关节角")
        self.pending = None
        self.preview = None
        self.bus.set_targets_q({n: q[i] for i, n in enumerate(self.robot.joint_names)},
                               confirm=False)
        if self.recorder:
            try:
                self.recorder.command("test_write_targets",
                                      " ".join(f"{n}{v:+.2f}" for n, v in
                                               zip(self.robot.joint_names, q)))
            except Exception:  # noqa: BLE001
                pass
        return {"ok": True, "q": [round(v, 3) for v in q],
                "msg": "只写了目标，没有重发"}

    def test_skip_resend(self, on: bool = True) -> dict:
        """30.3 e4：关掉“写目标后 0.06 s 重发”（只对本次会话有效）。"""
        self.bus.skip_resend = bool(on)
        return {"ok": True, "skip_resend": bool(on),
                "msg": f"skip_resend = {bool(on)}"}

    def test_cvp_crosscheck(self, n: int = 50, joints=("J2", "J3")) -> dict:
        """30.1 第 3 条：0x94 电流和单独读 0x04 电流对照（只读，不发写命令）。

        报：每颗对照的差（占 0x53 满量程的 %）、比值中位数、六颗 0x53 满量程（原始 IQ8 和 A）。
        """
        if self.bus is None:
            raise MotionError("还没连接")
        bus = self.bus
        fs = {}
        for nm in bus.order:
            m = bus.motors[nm]
            raw = None
            val = None
            try:
                d = bus.link.read_many([(m.mid, 0x53, 2)], timeout=0.1, retries=2)
                dd = d.get((m.mid, 0x53))
                raw = None if dd is None else int.from_bytes(dd, "big", signed=True)
            except Exception:  # noqa: BLE001
                raw = None
            try:
                val = m.current_max_a()
            except Exception:  # noqa: BLE001
                val = None
            fs[nm] = {"raw_iq8": raw, "full_scale_a": val}
        out = {"full_scales": fs, "compare": {}}
        for nm in joints:
            pairs = []
            t0 = time.monotonic()
            while len(pairs) < int(n) and time.monotonic() - t0 < 30.0:
                try:
                    cvp = bus.read_cvp_all(timeout=0.03, retries=1).get(nm) or {}
                    st = bus.read_all(full=True).get(nm)
                except Exception:  # noqa: BLE001
                    continue
                a94 = cvp.get("current_a")
                a04 = None if st is None else st.current_a
                if a94 is not None and a04 is not None:
                    pairs.append((a04, a94))
            if not pairs:
                out["compare"][nm] = None
                continue
            scale = fs.get(nm, {}).get("full_scale_a") or 1.0
            diffs = [abs(b - a) for a, b in pairs]
            ratios = [b / a for a, b in pairs if abs(a) > 0.2]
            diffs.sort()
            out["compare"][nm] = {
                "n": len(pairs),
                "max_diff_pct_fs": round(max(diffs) / scale * 100.0, 3),
                "median_ratio": (None if not ratios else
                                 round(sorted(ratios)[len(ratios) // 2], 4)),
            }
        # 判据：差 ≤ 满量程 2%，比值中位 0.9–1.1
        ok = True
        for nm, r in out["compare"].items():
            if r is None:
                ok = False
                continue
            if r["max_diff_pct_fs"] > 2.0:
                ok = False
            if r["median_ratio"] is not None and not (0.9 <= r["median_ratio"] <= 1.1):
                ok = False
        out["ok"] = ok
        if self.recorder:
            try:
                self.recorder.command("test_cvp_crosscheck", json.dumps(out, ensure_ascii=False)[:300])
            except Exception:  # noqa: BLE001
                pass
        return out

    def test_poll_hold(self, on: bool = True) -> dict:
        """37.1 第 1 条：只暂停空闲轮询的总线读取，保护/网页状态照常用 trace 读数跑。

        只有 trace 正在跑、且采六颗全部时才允许打开；trace 结束会自动关。
        """
        on = bool(on)
        if on:
            tr = self.trace
            if tr is None or not tr.status().get("active"):
                raise MotionError("poll_hold 只能在 trace 正在跑时打开（37.1 第 1 条）")
            js = tr.joints
            if js is not None and len(js) != len(self.robot.joint_names):
                raise MotionError("poll_hold 只能在 trace 采六颗全部时打开；"
                                  f"现在只采 {len(js)} 颗")
            if getattr(tr, "mode", "") not in ("cvp",):
                raise MotionError("poll_hold 要配 trace 的 cvp 模式（0x94 一轮六颗）")
        self._poll_hold = on
        return {"ok": True, "poll_hold": on}

    def test_read_profiles(self) -> dict:
        """27.3：回读六颗的梯形参数（0x1C/0x1D/0x1E）。"""
        out = {}
        for n in self.bus.order:
            try:
                pr = self.bus.motors[n].profile()
            except Exception:  # noqa: BLE001
                pr = None
            out[n] = None if pr is None else {
                "vel_rpm": round(float(pr[0]), 3),
                "acc_rpm_s": round(float(pr[1]), 3),
                "dec_rpm_s": round(float(pr[2]), 3),
                "vel_deg_s": round(float(pr[0]) * 10.0 / 60.0, 3),
                "acc_deg_s2": round(float(pr[1]) * 10.0 / 60.0, 3),
                "dec_deg_s2": round(float(pr[2]) * 10.0 / 60.0, 3),
            }
        return {"ok": True, "profiles": out}

    def _bus_startup_done(self) -> bool:
        return bool(getattr(self.bus, "startup_done", False))

    def _ecb_lock_status(self) -> dict:
        """30.6：ECB 独占锁现在被谁占着（给网页/回报看）。"""
        try:
            from . import ecblock
            return ecblock.status()
        except Exception as exc:  # noqa: BLE001
            return {"held": None, "error": str(exc)}

    def _param_stats(self) -> dict:
        """29.4：会话头里要的六颗可读参数（读不到就留空，不猜）。"""
        stats: dict = {"limits": {}, "speed_limits": {}, "pid": {}}
        bus = self.bus
        if bus is None or not hasattr(bus, "motors"):
            return stats
        for name, m in getattr(bus, "motors", {}).items():
            try:
                stats["limits"][name] = bus.current_limit_readback.get(name)
            except Exception:  # noqa: BLE001
                pass
            try:
                sp = m.speed_limit_rpm()
                stats["speed_limits"][name] = (None if sp is None
                                               else round(float(sp) * 10.0 / 60.0, 2))
            except Exception:  # noqa: BLE001
                pass
        return stats

    def mark(self, text: str = "", kind: str = "其他") -> dict:
        """29.3 用户标记：记一条事件（带当前流程/步号）并触发事故快照。"""
        phase = ""
        step = ""
        try:
            phase = str((self.motion.status or {}).get("phase") or "")
        except Exception:  # noqa: BLE001
            pass
        try:
            step = str((self.park_step or {}).get("phase") or "")
        except Exception:  # noqa: BLE001
            pass
        flow = "park" if step and step != "idle" else (
            "motion" if phase and phase not in ("idle",) else "")
        text = (f"{kind}" + (f"：{text}" if text else ""))
        if self.session:
            self.session.event("user_mark", text, phase=phase, step=step,
                               flow=flow)
            self.session.incident("用户标记", phase=phase, step=step,
                                  detail={"标记": text})
        if self.recorder:
            try:
                self.recorder.command("user_mark", text)
            except Exception:  # noqa: BLE001
                pass
        self.msg = f"已记标记：{text}（{phase or '—'}.{step or '—'}）"
        return {"ok": True, "mark": text, "phase": phase, "step": step,
                "flow": flow, "session": (self.session.id if self.session else None),
                "msg": self.msg}

    def current_limits(self) -> dict:
        """20.5：优先用总线真正回读的限流；J2 读不到时按最保守 3.0 A。

        不再从状态缓存里拿（read_all 的 0x59 未读时会拿配置值冒充）。
        """
        readbacks = getattr(self.bus, "current_limit_readback", {}) if self.bus else {}
        out = {}
        for n in self.robot.joint_names:
            v = readbacks.get(n)
            if v is None:
                if n == "J2":
                    v = 3.0          # 最保守；网页会标“未回读”
                else:
                    v = getattr(self.joint_cfg.get(n),
                                "required_current_limit_a", None)
            out[n] = None if v is None else float(v)
        return out

    def limit_used(self) -> dict:
        """给网页看：每个关节用的是回读值、配置值还是固定容量。"""
        readbacks = getattr(self.bus, "current_limit_readback", {}) if self.bus else {}
        out = {}
        for n in self.robot.joint_names:
            v = readbacks.get(n)
            if v is not None:
                out[n] = {"a": round(float(v), 3), "source": "回读"}
            elif n == "J2":
                out[n] = {"a": 3.0, "source": "保守(未回读)"}
            elif n in ("J4", "J5"):
                out[n] = {"a": None, "source": "固定容量2.30"}
            else:
                out[n] = {"a": None, "source": "固定"}
        return out

    def _sync_motion_params(self):
        limits = self.current_limits()
        for m in (self.motion, self.preview_motion):
            if m is None:
                continue
            m.end_effector = self.end_effector
            m.payload_kg = float(self.payload_kg)
            m.limit_a = {k: v for k, v in limits.items() if v is not None}
            m.kt = self.kt
            m.j2_bracket = self.j2_bracket
            m.speed_scale = float(self.speed_scale)

    def capacities(self) -> dict:
        cap = gravity.capacities(
            {k: v for k, v in self.current_limits().items() if v is not None},
            self.kt, self.j2_bracket)
        return {k: (None if not math.isfinite(v) else round(float(v), 3))
                for k, v in cap.items()}

    def set_end_effector(self, end_effector: str, payload_kg=None) -> dict:
        self._reject_if_parking()
        ee = str(end_effector or "")
        if ee not in gravity.END_EFFECTORS:
            raise MotionError("末端只能是 " + " / ".join(gravity.END_EFFECTORS))
        if payload_kg is not None:
            self.payload_kg = max(0.0, float(payload_kg))
        self.end_effector = ee
        # 20.9.4：旧计划的中间点按旧末端查过，换末端后必须清掉
        self.pending = None
        self.preview = None
        self._home_preview = None
        self._sync_motion_params()
        warn = ""
        if ee == "gripper":
            j2a = self.current_limits().get("J2")
            if j2a is None or j2a <= 4.5:
                warn = (f"带爪但 J2 回读限流 {j2a if j2a is not None else '未知'} A ≤ 4.5 A；"
                        "按 16.3 第 2 条改 6 A（用户在场）。容量仍按回读值算，"
                        "带爪、前臂水平的姿态会被 18.2 预检拒绝。")
                self.msg = warn
        return {"ok": True, "end_effector": ee, "payload_kg": self.payload_kg,
                "capacities": self.capacities(), "warn": warn}

    def _hold_tick(self, q):
        """18.2 第 4 条：保持/停留时 τ/容量 >70% 持续 30 s → 提醒。"""
        if not q or not self.use_actual() or not getattr(self.bus, "startup_done", False):
            self.hold = {"since": None, "ratio": 0.0, "warn": ""}
            return
        if (self.motion and self.motion.status.get("busy")) or self.drag_active \
                or self.calib.active:
            self.hold["since"] = None
            return
        try:
            ratios = gravity.hold_ratios(self.robot, q, self.end_effector, self.payload_kg,
                                         {k: v for k, v in self.current_limits().items() if v},
                                         self.kt, self.j2_bracket)
        except Exception:  # noqa: BLE001
            return
        mx = max([v for v in ratios.values() if v is not None] or [0.0])
        now = time.monotonic()
        self.hold["ratio"] = round(float(mx), 3)
        if mx > gravity.HOLD_WARN_RATIO:
            if self.hold.get("since") is None:
                self.hold["since"] = now
            if now - self.hold["since"] > gravity.HOLD_WARN_S and not self.hold.get("warn"):
                self.hold["warn"] = (f"保持力矩高（{mx * 100:.0f}% 容量，超过 70% 已 "
                                     f"{now - self.hold['since']:.0f} s），请恢复或停机")
                self.msg = self.hold["warn"]
        else:
            self.hold["since"] = None
            self.hold["warn"] = ""

    def _park_check(self, q) -> dict:
        parked = (self.park_state or {}).get("parked_q")
        if not parked:
            return {"has_parked": False, "ok": None}
        if (self.park_state or {}).get("valid") is False:
            return {"has_parked": True, "ok": False,
                    "reason": "上次不是正常停机（扶持停机，或 18.3 没走完），"
                              "不比对数值；请看实物后点“已确认”解除",
                    "parked_q": parked,
                    "ts": (self.park_state or {}).get("ts", "")}
        if not q:
            return {"has_parked": True, "ok": None}
        diffs = {n: round(abs(float(a) - float(b)), 2)
                 for n, a, b in zip(self.robot.joint_names, q, parked)}
        d = max(diffs.values())
        return {"has_parked": True, "ok": d <= float(
                    (self.park_cfg.get("restart_check_deg") or 2.0)),
                "max_diff_deg": round(d, 2), "diffs": diffs, "parked_q": parked,
                "ts": (self.park_state or {}).get("ts", "")}

    def ik_all(self, target, axis=(0.0, 0.0, -1.0), yaw=None,
               preset=None, free_yaw: bool = False) -> dict:
        """22.1：列出解析逆解（法兰位姿精修后），并过滤围栏/力矩。

        返回项带 branch（肩 A/B、肘 ±、腕 ±）、q、残差、相对当前角变化。
        """
        m = self.planning_motion()
        q0 = m.read_q()
        preset = preset or self.tcp_preset
        use_yaw = None if free_yaw else (self.yaw_deg if yaw is None else yaw)
        sols = self.robot.ik_all(
            target, axis, use_yaw, seed_deg=q0, preset=preset,
            limits=self.limits, time_budget_s=4.0)
        out = []
        for sol in sols:
            q = sol["q_deg"]
            reason = ""
            try:
                fr = self.fence.check(q, preset, 0.0 if free_yaw else float(use_yaw or 0.0),
                                      False, self.limits)
                if not fr.ok:
                    reason = "围栏：" + "；".join(fr.reasons)
            except Exception as exc:  # noqa: BLE001
                reason = f"围栏检查失败：{exc}"
            try:
                bad = gravity.check_q(
                    self.robot, q, self.end_effector, self.payload_kg,
                    {k: v for k, v in self.current_limits().items() if v},
                    self.kt, j2_bracket=self.j2_bracket)
                if bad and not reason:
                    b0 = bad[0]
                    reason = (f"力矩：{b0['joint']} {b0['tau']:+.2f} N·m "
                              f"> 0.8×{b0['capacity']:.2f}")
            except Exception as exc:  # noqa: BLE001
                reason = reason or f"力矩检查失败：{exc}"
            if reason:
                continue
            out.append({**sol, "ok": True})
        return {"ok": True, "solutions": out, "selected": (out[0] if out else None)}

    def plan_click(self, x: float, y: float, z: float = 20.0,
                   yaw: float | None = None) -> dict:
        self._reject_if_parking()
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，点击已忽略")
        if yaw is not None:
            self.yaw_deg = float(yaw)
            if self.motion:
                self.motion.yaw_deg = self.yaw_deg
            self.preview_motion.yaw_deg = self.yaw_deg
        segs = m.plan_click(float(x), float(y), float(z), self.yaw_deg, fast=True)
        self.pending = segs
        last = segs[-1]
        self.preview = {
            "segments": brief_segments(segs),
            "scene": self.fence.scene(last["q_target"], self.tcp_preset, self.yaw_deg),
            "q": last["q_target"],
            "tcp": self.robot.tcp(last["q_target"], self.tcp_preset, self.yaw_deg)[:3, 3].tolist(),
            "fence": last["fence"],
            "ik": last.get("ik"),
            "target": [float(x), float(y), float(z)],
            "yaw": self.yaw_deg,
            "near_limit": last.get("near_limit", []),
            "offline": not self.use_actual(),
        }
        return {"ok": True, "preview": self.preview,
                "segments": brief_segments(segs), "started": False}

    def click(self, x: float, y: float, z: float = 20.0,
              yaw: float | None = None) -> dict:
        self._reject_if_parking()
        r = self.plan_click(x, y, z, yaw)
        if self.exec_mode == "direct":
            self._ready_for_motion()
            r["started"] = True
            self._start_exec(self.pending, preview=False, what="虚拟点击",
                             motion=self.motion)
            r["msg"] = "直接执行"
        else:
            r["msg"] = ("预演（离线，零位模型）：确认后点执行" if not self.use_actual()
                        else "预演：确认后点执行")
        return r

    def execute(self, preview: bool | None = None, use_pending: bool = True) -> dict:
        self._reject_if_parking()
        # 默认按预演处理，避免漏传 preview 时直接动电机
        preview = True if preview is None else bool(preview)
        segs = self.pending if use_pending and self.pending else None
        if not segs:
            raise MotionError("没有待执行的计划")
        m = self.planning_motion()
        if not preview:
            self._ready_for_motion()
            m = self.motion
        for seg in segs:
            m._check_q(seg["q_target"], seg.get("descend", False))
        return self._start_exec(segs, preview, "执行", motion=m)

    def _start_exec(self, segments, preview: bool, what: str,
                    motion: Motion | None = None, already_begun: bool = False):
        m = motion or self.motion
        if m is None:
            raise MotionError("还没连接，不能执行")
        if not preview:
            pr = (self.robot.tcp_presets or {}).get(self.tcp_preset) or {}
            if pr.get("measured") is False:
                raise MotionError(
                    f"TCP 预设 {self.tcp_preset} 还没实测（measured=false）："
                    "只允许预演；先用卡尺量 L、写入 config/tcp.json 并置 measured=true")

        def worker():
            try:
                if not preview:
                    qnow = m.read_q()
                    q0 = segments[0].get("q_from") if segments else None
                    if q0 is not None and max(abs(a - b) for a, b in zip(qnow, q0)) > 3.0:
                        raise MotionError("当前姿态和预演时不一致，请重新预演后再执行")
                # 20.3：停止标志已在接受命令时 begin_command() 清过，
                # 这里不能再清，否则 10 ms 内按的停会丢。
                m.execute(segments, preview=preview, clear_stop=False)
            except MotionError as exc:
                with m._lock:
                    if m.status.get("busy"):
                        m.status.update({"busy": False, "phase": "aborted",
                                         "reason": str(exc)})
                self.msg = f"{what}中止：{exc}"
            except Exception as exc:  # noqa: BLE001
                with m._lock:
                    if m.status.get("busy"):
                        m.status.update({"busy": False, "phase": "error",
                                         "reason": str(exc)})
                self.msg = f"{what}失败：{exc}"
                traceback.print_exc()

        with self._exec_lock:
            if self.park_active or (self._park_thread and self._park_thread.is_alive()):
                raise MotionError("停机流程正在跑：先按停，等它结束")
            if self._exec_thread and self._exec_thread.is_alive():
                raise MotionError("已经在执行")
            if not already_begun:
                if m.status.get("busy"):
                    raise MotionError("正在运动")
                m.begin_command()      # 接受命令即 busy，按停立刻有效
            self._exec_thread = threading.Thread(target=worker, daemon=True)
            self._exec_thread.start()
        return {"ok": True, "started": True, "preview": preview}

    def jog(self, joint: str, delta: float, preview: bool = False,
            speed_deg_s: float | None = None,
            single_target: bool = False) -> dict:
        self._reject_if_parking()
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        if not preview:
            self._mark_operation("jog")
            self._on_shelf = False
        joint = joint.upper()
        if joint not in self.robot.joint_names:
            raise MotionError(f"未知关节 {joint}")
        q = m.read_q()
        i = self.robot.joint_names.index(joint)
        q2 = list(q)
        q2[i] += float(delta)
        # 20.3：命令一接受就置 busy，规划本身可能 0.5–3 s；这期间心跳断
        # 也必须能被 watchdog 按停，不能等规划完才开始判忙。
        if not preview:
            m.begin_command()
        try:
            segs = m._plan_move(
                q, q2, f"点动 {joint}{delta:+.0f}°", False,
                max_step_deg=(180.0 if single_target else None))
            if speed_deg_s is not None:
                m.cap_speeds(segs, speed_deg_s)
        except Exception as exc:
            with m._lock:
                m.status.update({"busy": False, "phase": "error",
                                 "reason": str(exc)})
            raise
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, f"点动 {joint}", motion=self.motion,
                                already_begun=True)

    def home(self, preview: bool = False) -> dict:
        self._reject_if_parking()
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        q_home = None
        if not preview:
            hp = self._home_preview
            qnow = m.read_q()
            if (not hp or time.time() - hp["t"] > 60.0
                    or max(abs(a - b) for a, b in zip(hp["q"], qnow)) > 0.02):
                raise MotionError("回零位必须先对当前姿态预演；请先点一次“回零位（预演）”，检查轨迹/围栏后再执行")
            self._home_preview = None
        else:
            q_home = list(m.read_q())
        segs = m.plan_home()
        if preview and q_home is not None:
            self._home_preview = {"q": q_home, "t": time.time()}
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            # 回零可能是“抬高 → 回零位”两段；把完整分段一并给网页显示
            return {"ok": True, "preview": self.preview,
                    "segments": brief_segments(segs), "started": False}
        return self._start_exec(segs, False, "回零", motion=self.motion)

    def move_joints(self, q, preview: bool = False, descend: bool = False) -> dict:
        self._reject_if_parking()
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        if len(q) != 6:
            raise MotionError("需要 6 个关节角")
        if not preview:
            self._mark_operation("move_joints")
            self._on_shelf = False
        segs = m.plan_joint_move(q, descend=descend, label="move_joints")
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_joints", motion=self.motion)

    def move_tcp(self, target, axis=(0, 0, -1), yaw=None, preview: bool = False,
                 descend: bool = False, preset: str | None = None,
                 free_yaw: bool = False) -> dict:
        self._reject_if_parking()
        if not preview:
            self._mark_operation("move_tcp")
            self._on_shelf = False
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        if yaw is not None:
            self.yaw_deg = float(yaw)
            if self.motion:
                self.motion.yaw_deg = self.yaw_deg
            self.preview_motion.yaw_deg = self.yaw_deg
        segs = m.plan_tcp_move(target, axis, self.yaw_deg,
                               preset=preset, descend=descend,
                               label="move_tcp", free_yaw=free_yaw,
                               fast=preview)
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["target"] = [round(float(v), 2) for v in target]
        self.preview["source"] = "tcp"
        self.preview["free_yaw"] = bool(free_yaw)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_tcp", motion=self.motion)

    def move_through(self, waypoints, descend_flags=None, labels=None,
                     preview: bool = False) -> dict:
        self._reject_if_parking()
        if not preview:
            self._mark_operation("move_through")
            self._on_shelf = False
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        segs = m.plan_through(waypoints, descend_flags, labels)
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_through", motion=self.motion)

    # ---------------- 18.3 停机 / 从停机启动 ----------------
    def _sample_currents(self, names, n=None, stop_event=None) -> dict:
        n = int(n or self.park_cfg.get("sample_n") or 20)
        acc = {k: [] for k in names}
        for _ in range(n):
            if stop_event is not None and stop_event.is_set():
                break
            st = self.bus.read_all(full=True)
            for k in names:
                v = getattr(st.get(k), "current_a", None)
                if v is not None:
                    acc[k].append(float(v))
            time.sleep(0.02)
        return {k: (round(sum(v) / len(v), 4) if v else None) for k, v in acc.items()}

    def _park_q_at_z(self, z: float, q1: float,
                     q4: float | None = None,
                     q5: float | None = None,
                     q6: float | None = None) -> list[float]:
        """实际分步下降用的公式（20.8：预演和实际共用）。

        q2/q3 保证 J4 轴按 z 下降；腕部 q4/q5/q6 默认用教的
        ``poses.park[3:6]``，复现教停机姿态时的斜腕/线缆方向；
        也可用 ``descent_q4_deg`` / ``descent_j5_deg`` / ``descent_j6_deg``
        单独覆盖。
        """
        c = max(-1.0, min(1.0, (z - 120.5) / 203.5))
        q2 = math.degrees(math.acos(c))
        pk = self.park_cfg
        park_pose = (pk.get("poses") or {}).get("park", [0.0] * 6)
        if q4 is None:
            q4 = pk.get("descent_q4_deg")
            if q4 is None:
                q4 = float(park_pose[3])
        if q5 is None:
            q5 = pk.get("descent_j5_deg")
            if q5 is None:
                q5 = float(park_pose[4])
        if q6 is None:
            q6 = pk.get("descent_j6_deg")
            if q6 is None:
                q6 = float(park_pose[5])
        return [q1, q2, 90.0 - q2, float(q4), float(q5), float(q6)]

    def park(self, preview: bool = False, prereq: dict | None = None) -> dict:
        self._reject_if_parking()
        if not preview:
            self._mark_operation("park")
        cons = self.park_config().get("park_consistency") or {"ok": True, "problems": []}
        if not cons.get("ok"):
            raise MotionError("停机点配置对不上，拒绝停机：" + "；".join(cons["problems"]))
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，先按停")
        if self.end_effector != "flange":
            raise MotionError(
                "停机只对空法兰开放（18.1a：带爪失能后静力余量只有 1.1 倍；"
                "腕托做好之前，带爪请用“扶持停机”）")
        if not preview:
            self._ready_for_motion(allow_hold=True)   # 接住失败后允许重做停机
            m = self.motion
            st = self.bus.read_all(full=True)
            alarms = [f"{n}(0x{s.error:04X})" for n, s in st.items() if s.error]
            if alarms:
                raise MotionError("有报警位非零：" + "，".join(alarms)
                                  + "；先走清报警流程再停机")
        missing = [label for key, label in (
            ("gripper_empty", "夹爪里没有东西"),
            ("workspace_clear", "工作区已清空"),
            ("shelf_ready", "搁架已放好")) if not (prereq or {}).get(key)]
        if missing:
            raise MotionError("停机前提没勾选：" + "、".join(missing))
        pk = self.park_config()
        q0 = m.read_q()
        # 20.8：名义 P 只用于旧画面；实际下降走固定公式，先不规划名义 P
        # （旧教的 P 可能离地 19.8 mm，围栏会拒绝，不能因此挡住预演）
        plan = m.plan_park(q0, pk, include_down=False)
        segs = plan["lift"] + plan["tuck"] + plan["pre"]
        self.pending = segs
        # 20.8：预演画“实际会走”的下降终点，而不是名义 P。
        # 实际下降用 _park_q_at_z() 从 z=168 起每 1 mm 一步，最低到 130.5。
        z_min = float(self.park_cfg.get("descent_min_j4_z") or 130.5)
        q1 = float((pk.get("poses") or {}).get("pre", [90.0])[0])
        q_end = self._park_q_at_z(z_min, q1)
        fr = self.fence.check(q_end, self.tcp_preset, self.yaw_deg, False, self.limits)
        pseudo = {"label": "分步下降到最低点（实际）", "q_target": q_end,
                  "T": 30.0, "fence": fr.as_dict(), "near_limit": fr.near_limit}
        self.preview = preview_payload(pseudo, self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        note = ("教的点：J1 决定左右侧、J4–J6 决定腕姿；下降高度按固定公式从 z=168 到 "
                f"{z_min:.1f}，每 {float(self.park_cfg.get('descent_step_mm') or 1.0):g} mm 一步")
        # 20.8：给网页画一条实际会走的下降路径（TCP 点，z 每 4 mm 一个）
        path = []
        zz = 168.0
        while zz > z_min + 1e-6:
            qq = self._park_q_at_z(zz, q1)
            path.append([round(float(v), 2) for v in
                         self.robot.tcp(qq, self.tcp_preset, self.yaw_deg)[:3, 3]])
            zz -= 4.0
        path.append([round(float(v), 2) for v in
                     self.robot.tcp(q_end, self.tcp_preset, self.yaw_deg)[:3, 3]])
        self.preview["park"] = {"poses": pk["poses"], "shelf": pk["shelf"],
                                "note": note, "descent_path": path}
        if preview:
            return {"ok": True, "preview": self.preview,
                    "segments": brief_segments(segs + plan["park"]), "started": False,
                    "descent": {"start_z": 168.0, "min_z": z_min,
                                "q_min": [round(float(v), 3) for v in q_end],
                                "note": note},
                    "phases": {"lift": len(plan["lift"]), "tuck": len(plan["tuck"]),
                               "pre": len(plan["pre"]), "descent": note}}
        self._start_park(plan)
        return {"ok": True, "started": True, "msg": "停机流程已启动（到预停机点后分步下降）"}

    def teach_park(self) -> dict:
        """把当前实际六关节位置记录为搁架停机位置（只记录，不运动）。"""
        self._reject_if_parking()
        if self.end_effector != "flange":
            raise MotionError("当前不是空法兰：停机只对 flange 开放，先切回 flange 再教")
        if not self.connected or not getattr(self.bus, "startup_done", False):
            raise MotionError("还没使能：实际角度要先完成零扭矩启动才能读，不能记录")
        if self.motion is None:
            raise MotionError("还没连接")
        if self.motion.status.get("busy") or self.drag_active:
            raise MotionError("正在运动/零扭矩拖动：先停住/退出拖动，等保持稳定再记录")
        if self._park_thread and self._park_thread.is_alive():
            raise MotionError("停机流程正在跑，不能改停机位置")
        q = list(self.motion.read_q())
        if len(q) != 6 or any(v is None for v in q):
            raise MotionError("读不到完整六关节位置，不能记录")
        lo, hi = self.limits
        bad = [f"{n} {q[i]:+.1f}°（{lo[i]:.0f}..{hi[i]:.0f}）"
               for i, n in enumerate(self.robot.joint_names)
               if not (lo[i] - 1e-6 <= q[i] <= hi[i] + 1e-6)]
        if bad:
            raise MotionError("当前位置超安全限位，不能记录：" + "，".join(bad))
        pk = json.loads(json.dumps(self.park_cfg))
        poses = pk.get("poses") or {}
        poses["park"] = [round(float(v), 4) for v in q]
        pk["poses"] = poses
        pk["park_taught"] = True
        pk["park_taught_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        pk["park_taught_source"] = "web: current actual q"
        pk["park_side"] = -1 if float(q[0]) < 0 else 1
        if not pk.get("version"):
            pk["version"] = "2026-11-12.1"
        if not getattr(self.bus, "is_mock", False):
            cfgmod.save_park(pk)
        self.park_cfg = pk
        self.msg = ("停机位置已记录（教的点：J1 决定左右侧、J4–J6 决定腕姿；下降高度按公式）："
                    + " ".join(f"{n}{v:+.1f}" for n, v in zip(self.robot.joint_names, q)))
        return {"ok": True, "q": [round(float(v), 4) for v in q],
                "park_side": int(pk["park_side"]),
                "taught_at": pk["park_taught_at"], "msg": self.msg}

    def park_ack(self) -> dict:
        """18.3 从停机启动比对不过：用户看过实物后点“已确认”，解除硬拦。"""
        if not self.park_block:
            return {"ok": True, "msg": "当前没有启停比对拦截"}
        self.park_block = None
        self.msg = "已确认启停比对差异，解除运动拦截"
        return {"ok": True, "msg": self.msg}

    def park_drop(self, confirmed: bool = False) -> dict:
        """27.2 第 4 条：停机中途停在搁架上方时，就地失能、让臂落到搁架上。

        只在这种情况下开放：停机序列在第 3–6 步里停住/中止、q1 和教的 q1 差 ≤2°、
        按实测关节角算出的 J4 轴高 ≤175 mm。按 J6→J1 每颗隔 0.5 s 失能；
        这时不做 18.3 第 7 条“其余关节变化 >1° 就重新使能”（本来就要让它滑进 V 槽）。
        """
        if not confirmed:
            raise MotionError(
                "要让臂就地失能落到搁架上，要二次确认（确认以后臂会自己滑下来）")
        self._reject_if_parking()
        if self.motion is None:
            raise MotionError("还没连接")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError(
                "六颗现在不在使能保持里：不能做“就地失能”（这一条只给停机中途、还使能着的时候用）")
        phase = str((self.park_step or {}).get("phase") or "")
        allowed = ("moving_pre", "pre_3a_done", "at_pre", "descending",
                   "contact_mismatch", "catch_failed", "at_park", "aborted")
        if phase not in allowed:
            raise MotionError(
                f"现在不在停机流程第 3–6 步里（phase={phase or '空'}），不开放就地失能")
        pk = self.park_config()
        q1p = float((pk.get("poses") or {}).get("pre", [90.0])[0])
        q = list(self.motion.read_q())
        if len(q) != 6 or any(v is None for v in q):
            raise MotionError("读不到完整六关节位置，不能做就地失能")
        dq1 = abs(float(q[0]) - q1p)
        if dq1 > 2.0:
            raise MotionError(
                f"q1={q[0]:.2f}° 和教的 q1={q1p:.2f}° 差 {dq1:.1f}° > 2°："
                "不在搁架正上方，就地失能会摔倒，拒绝")
        z = 120.5 + 203.5 * math.cos(math.radians(float(q[1])))
        if z > 175.0:
            raise MotionError(
                f"按实测角算出的 J4 轴高 {z:.1f} mm > 175 mm：太高，落到搁架上会撞，拒绝")
        gap = float(self.park_cfg.get("disable_gap_s") or 0.5)
        disabled: list[str] = []
        order = list(self.park_cfg.get("disable_order") or
                     ["J6", "J5", "J4", "J3", "J2", "J1"])
        for name in order:
            try:
                self.bus.disable_one(name)
            except Exception as exc:  # noqa: BLE001
                raise MotionError(f"失能 {name} 失败：{exc}") from exc
            disabled.append(name)
            time.sleep(gap)
        self.bus.startup_done = False
        try:
            self.park_state = dict(self.park_state or {})
            self.park_state["valid"] = False
            self._save_park_state_safe(self.park_state)
        except Exception:  # noqa: BLE001
            pass
        self.park_step.update({"phase": "dropped_onto_shelf",
                              "disabled": list(disabled), "z": round(z, 1),
                              "q": [round(float(v), 2) for v in q]})
        self.msg = ("已按 J6→J1 就地失能（q1 差 "
                    f"{dq1:.1f}°、J4 轴高 {z:.1f} mm），臂自己落到搁架上。"
                    "这次上电不再使能：请拉空开断电。")
        if self.recorder:
            try:
                self.recorder.command(
                    "park_drop",
                    f"就地失能落到搁架上：q1 差 {dq1:.2f}°、J4 轴高 {z:.1f} mm、"
                    f"失能顺序 {','.join(disabled)}")
            except Exception:  # noqa: BLE001
                pass
        return {"ok": True, "disabled": disabled, "q1_diff_deg": round(dq1, 2),
                "j4_axis_z": round(z, 1), "msg": self.msg}

    def _contact_height_decision(self, z: float) -> tuple[str, str]:
        """A9 第 5 条：接住高度和第一次成功停机记录比。

        返回 ("first", "") 第一次（要记录）/ ("ok", "") 在 5 mm 内 /
        ("mismatch", 说明) 超 5 mm（要原地保持）。
        """
        rec = self.park_cfg.get("contact_j4_z")
        if rec is None:
            return "first", ""
        diff = abs(float(z) - float(rec))
        if diff > 5.0:
            return "mismatch", (f"接触高度和记录差 {diff:.1f} mm"
                                f"（现在 z={float(z):.1f}，记录 {float(rec):.1f}）")
        return "ok", ""

    def _start_park(self, plan: dict):
        if self.park_active or (self._park_thread and self._park_thread.is_alive()):
            raise MotionError("停机流程已经在跑")
        m = self.motion
        if m is None:
            raise MotionError("还没连接")
        with self._exec_lock:
            if self._exec_thread and self._exec_thread.is_alive():
                raise MotionError("已经在执行别的运动")
            if m.status.get("busy"):
                raise MotionError("正在运动")
            self._park_stop.clear()
            self.park_active = True
            # 26.5 第 1 条：停机没走完就断电，parked_q 无效
            try:
                self.park_state = dict(self.park_state or {})
                self.park_state["valid"] = False
                self._save_park_state_safe(self.park_state)
            except Exception:  # noqa: BLE001
                pass
            self.park_step = {"phase": "moving_pre", "reason": "", "z": None, "step": 0,
                              "I0": None, "I2": None, "I1": None, "contact_z": None,
                              "disabled": []}
            m.begin_command()      # 整个停机流程一开始就可以被“停”打断

        def worker():
            gap = float(self.park_cfg.get("disable_gap_s") or 0.5)
            settle = float(self.park_cfg.get("settle_s") or 1.0)
            tol = float(self.park_cfg.get("disable_check_deg") or 1.0)
            ratio = float(self.park_cfg.get("catch_ratio") or 0.5)
            step_mm = float(self.park_cfg.get("descent_step_mm") or 1.0)
            z_min = float(self.park_cfg.get("descent_min_j4_z") or 130.5)
            q1 = list(plan["poses"]["pre"])[0]
            pre_q = list(plan["poses"]["pre"])
            disabled: list[str] = []

            class _ParkStop(Exception):
                pass

            def stopped() -> bool:
                return self._park_stop.is_set()

            def best_hold():
                try:
                    return self.bus.hold_all()
                except Exception:  # noqa: BLE001
                    return None

            def check_stop():
                if stopped():
                    raise _ParkStop()

            def hold_after_stop(why: str):
                res = None
                try:
                    res = self.bus.hold_all()
                except Exception:  # noqa: BLE001
                    pass
                un = (res or {}).get("unconfirmed") or []
                self.park_step.update({
                    "phase": "aborted", "reason": why,
                    "disabled": list(disabled),
                })
                tail = ""
                if disabled:
                    tail = "；已失能：" + "、".join(disabled)
                if un:
                    tail += "；保持未确认：" + "，".join(
                        f"{x.get('name')}({x.get('reason')})" for x in un)
                with m._lock:
                    m.status.update({"busy": False, "phase": "aborted",
                                     "reason": why})
                self.msg = f"停机已停在原地（{why}）{tail}"
                # 已经失能过任何一颗，启动态就不再成立
                if disabled:
                    try:
                        self.bus.startup_done = False
                    except Exception:  # noqa: BLE001
                        pass

            try:
                # 1–3：抬高、收拢、到预停机点，读 I₀
                check_stop()
                pre3a = plan.get("pre3a")
                pre3b = plan.get("pre3b")
                if pre3a is None or pre3b is None:      # 兼容旧 plan 结构
                    m.execute(plan["lift"] + plan["tuck"] + plan["pre"],
                              preview=False, clear_stop=False)
                else:
                    # 27.2 第 1 条：3a（只动 J3+腕）走完、J3 到位，才走 3b（只动 J2）
                    m.execute(plan["lift"] + plan["tuck"] + pre3a,
                              preview=False, clear_stop=False)
                    check_stop()
                    self.park_step.update({"phase": "pre_3a_done"})
                    m.execute(pre3b, preview=False, clear_stop=False)
                check_stop()
                # A9/18.3 第 3 步：到预停机点以后静置 1 s 再采 I₀
                time.sleep(settle)
                check_stop()
                i0 = self._sample_currents(["J2", "J3"], stop_event=self._park_stop)
                check_stop()
                self.park_step.update({"phase": "at_pre", "I0": i0})
                sgn = 1.0 if (i0.get("J2") or 0.0) >= 0 else -1.0
                # 4：1 mm/步分步下降找接触（q2+q3=90，前臂保持水平）
                z = 168.0
                cur_q = m.read_q()
                lowest = None
                contact = False
                while z > z_min + 1e-6:
                    check_stop()
                    nz = max(z_min, round(z - step_mm, 3))
                    segs = m._plan_move(cur_q, self._park_q_at_z(nz, q1),
                                        f"分步下降 z={nz:.1f}")
                    for seg in segs:
                        seg["pitch_check"] = True
                    m.execute(segs, preview=False, clear_stop=False)
                    check_stop()
                    time.sleep(0.15)
                    cur_q = m.read_q()
                    z = nz
                    i2 = self._sample_currents(["J2"], stop_event=self._park_stop).get("J2")
                    check_stop()
                    lowest = i2
                    self.park_step.update({"phase": "descending", "z": round(z, 2),
                                           "step": int(round((168.0 - z) / step_mm)),
                                           "I2": i2})
                    if i2 is not None and sgn * i2 <= ratio * abs(i0.get("J2") or 0.0):
                        contact = True
                        self.park_step["contact_z"] = round(z, 2)
                        break
                # A9 第 5 条：接触高度和第一次成功停机记下的值比，差 >5 mm 原地保持
                if contact:
                    verdict, detail = self._contact_height_decision(z)
                    if verdict == "mismatch":
                        res = best_hold()
                        un = (res or {}).get("unconfirmed") or []
                        self.park_step.update({"phase": "contact_mismatch",
                                               "z": round(z, 2),
                                               "contact_z": round(z, 2)})
                        self.msg = (detail + "（搁架挪了，或者接触判错）。"
                                    "已原地保持，没有往下失能。")
                        if un:
                            self.msg += "；保持未确认：" + "，".join(
                                f"{x.get('name')}({x.get('reason')})" for x in un)
                        return
                    if verdict == "first":
                        try:
                            if getattr(self.bus, "is_mock", False):
                                raise RuntimeError("mock：不写真实 config/park.json")
                            pk2 = json.loads(json.dumps(self.park_cfg))
                            pk2["contact_j4_z"] = round(float(z), 2)
                            pk2["contact_j4_z_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
                            cfgmod.save_park(pk2)
                            self.park_cfg = pk2
                        except Exception as exc:  # noqa: BLE001
                            self.msg = f"接触高度记录失败（不影响停机）：{exc}"
                        if self.recorder:
                            try:
                                self.recorder.command(
                                    "contact_record",
                                    f"第一次成功停机：接住 J4 轴高度 z={z:.2f} mm 已记进 park.json")
                            except Exception:  # noqa: BLE001
                                pass
                if not contact:
                    check_stop()
                    back = m._plan_move(cur_q, pre_q, "回预停机点")
                    m.execute(back, preview=False, clear_stop=False)
                    self.park_step.update({"phase": "catch_failed",
                                           "lowest_I2": lowest, "z": round(z, 2)})
                    self.msg = (f"搁架没接住：I₀ {i0.get('J2')} A，降到 z={z:.1f} 时 J2 电流 "
                                f"{lowest} A（要 ≤{ratio:g}·I₀）。已回预停机点保持，没有失能。"
                                "检查搁架位置/垫片后可以直接再按“停机”重做，或点“恢复”。")
                    return
                # 再多走一步让搁架接实，然后读 I₁
                check_stop()
                if z > z_min + 1e-6:
                    nz = max(z_min, round(z - step_mm, 3))
                    segs = m._plan_move(cur_q, self._park_q_at_z(nz, q1),
                                        f"再走一步 z={nz:.1f}")
                    for seg in segs:
                        seg["pitch_check"] = True
                    m.execute(segs, preview=False, clear_stop=False)
                    z = nz
                    cur_q = m.read_q()
                check_stop()
                time.sleep(settle)
                i1 = self._sample_currents(["J2", "J3"], stop_event=self._park_stop)
                check_stop()
                self.park_step.update({"phase": "at_park", "I1": i1, "z": round(z, 2)})
                # 7：写 parked_q（取失能前最后一次读到的值）
                q_park = m.read_q()
                self.park_state = {"parked_q": [round(float(v), 4) for v in q_park],
                                   "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                                   "valid": True,
                                   "I0": i0, "I1": i1, "contact_z": round(z, 2),
                                   "end_effector": self.end_effector}
                self._save_park_state_safe(self.park_state)
                # 6：按序失能，每颗之后检查其余还使能的关节
                order = list(self.park_cfg.get("disable_order") or
                             ["J6", "J5", "J4", "J3", "J2", "J1"])
                for name in order:
                    check_stop()
                    raw_before = self.bus.read_positions_turns()
                    if name not in raw_before:
                        res = self.bus.hold_all()
                        self.park_step.update({"phase": "disable_aborted",
                                               "reason": f"{name} 失能前读不到位置",
                                               "disabled": list(disabled)})
                        self.msg = (f"停机在失能 {name} 前读不到它的位置，"
                                    "已停在保持、没有继续失能")
                        if disabled:
                            self.bus.startup_done = False
                        return
                    self.bus.disable_one(name)
                    disabled.append(name)
                    time.sleep(gap)
                    check_stop()
                    raw_after = self.bus.read_positions_turns()
                    moved = {}
                    for k, v in raw_before.items():
                        if k == name or k not in raw_after:
                            continue
                        deg = abs(self.joint_cfg[k].out_deg(raw_after[k])
                                  - self.joint_cfg[k].out_deg(v))
                        if deg > tol:
                            moved[k] = round(deg, 2)
                    if moved:
                        self.bus.enable_one_hold(name, raw_before.get(name, 0.0))
                        self.park_step.update({"phase": "disable_aborted",
                                               "moved": moved,
                                               "disabled": [x for x in disabled if x != name]})
                        self.msg = (f"失能 {name} 时这些关节动了 {moved}°，"
                                    f"已把 {name} 重新使能到保持并停止停机流程；"
                                    "请检查支撑后再处理。")
                        self.bus.startup_done = False
                        return
                    self.park_step["phase"] = f"disabled_{name}"
                    self.park_step["disabled"] = list(disabled)
                self.park_step.update({"phase": "parked", "disabled": list(disabled)})
                self.bus.startup_done = False
                self.msg = "已停机（搁架接住，六颗按 J6→J1 失能），可以关电源"
            except _ParkStop:
                hold_after_stop("用户按停")
            except MotionError as exc:
                if self._park_stop.is_set():
                    hold_after_stop("用户按停")
                    return
                with m._lock:
                    if m.status.get("busy"):
                        m.status.update({"busy": False, "phase": "aborted",
                                         "reason": str(exc)})
                res = best_hold()
                un = (res or {}).get("unconfirmed") or []
                self.park_step.update({"phase": "aborted", "reason": str(exc),
                                       "disabled": list(disabled)})
                if disabled:
                    self.bus.startup_done = False
                self.msg = f"停机中止：{exc}（臂保持使能，没有失能）"
                if un:
                    self.msg += "；保持未确认：" + "，".join(
                        f"{x.get('name')}({x.get('reason')})" for x in un)
            except Exception as exc:  # noqa: BLE001
                traceback.print_exc()
                res = best_hold()
                un = (res or {}).get("unconfirmed") or []
                with m._lock:
                    if m.status.get("busy"):
                        m.status.update({"busy": False, "phase": "error",
                                         "reason": str(exc)})
                self.park_step.update({"phase": "error", "reason": str(exc),
                                       "disabled": list(disabled)})
                if disabled:
                    self.bus.startup_done = False
                self.msg = f"停机失败：{exc}"
                if un:
                    self.msg += "；保持未确认：" + "，".join(
                        f"{x.get('name')}({x.get('reason')})" for x in un)
            finally:
                self.park_active = False

        thread = threading.Thread(target=worker, daemon=True)
        self._park_thread = thread
        thread.start()

    def unpark(self, preview: bool = False, confirmed: bool = False) -> dict:
        """从停机启动：J2/J3 同步抬回预停机点（离开搁架）。"""
        if not preview:
            self._mark_operation("unpark")
            self._on_shelf = False
        self._reject_if_parking()
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，先按停")
        if not preview:
            self._ready_for_motion()
            m = self.motion
        q0 = m.read_q()
        chk = self._park_check(q0)
        if chk.get("has_parked") and chk.get("ok") is False and not confirmed:
            raise MotionError(
                f"当前姿态和 parked_q 差 {chk.get('max_diff_deg')}°（>2°），"
                "停机后可能有人动过臂；请先确认再点“确认抬回”")
        segs = m.plan_unpark(q0, self.park_config())
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        self._start_exec(segs, False, "抬回预停机点", motion=self.motion)
        return {"ok": True, "started": True,
                "msg": "抬回预停机点；离开搁架后可以把搁架拿走"}

    def set_speed_scale(self, scale: float) -> dict:
        """21.4.4：全局速度倍率（0.05…1.0）；只影响之后的规划。"""
        v = max(0.05, min(1.0, float(scale)))
        self.speed_scale = v
        self._sync_motion_params()
        self.pending = None
        self.preview = None
        self._home_preview = None
        self.msg = f"全局速度倍率 {v * 100:.0f}%（之后规划生效）"
        return {"ok": True, "speed_scale": v, "msg": self.msg}

    def check_alarm_latch(self, names=None) -> dict:
        """A6：逐颗读报警码；不是 0 先清、再读；还不是 0 就拒绝运动并点名。

        每次真机开始、每次“恢复”之前都要走一遍。
        """
        if not self.bus:
            raise MotionError("还没连接")
        names = list(names or self.bus.order)
        rows: list[dict] = []
        bad: list[str] = []
        for name in names:
            m = self.bus.motors.get(name)
            if m is None:
                bad.append(f"{name}(未知关节)")
                continue
            e = None
            try:
                e = m.error(timeout=0.1, retries=2)
            except Exception:  # noqa: BLE001
                e = None
            if e is None:
                rows.append({"name": name, "before": None, "after": None,
                             "cleared": False, "error": "读不到报警码"})
                bad.append(f"{name}(读不到报警码)")
                continue
            before = e
            cleared = False
            if e != 0:
                try:
                    cleared = bool(m.clear_error())
                except Exception:  # noqa: BLE001
                    cleared = False
                time.sleep(0.12)
                try:
                    e = m.error(timeout=0.1, retries=2)
                except Exception:  # noqa: BLE001
                    e = None
            rows.append({"name": name, "before": f"0x{before:04X}",
                         "after": None if e is None else f"0x{e:04X}",
                         "cleared": cleared, "error": ""})
            if e not in (0, None):
                bad.append(f"{name}(0x{e:04X})")
            elif e is None:
                bad.append(f"{name}(清后读不到)")
        if self.recorder:
            try:
                self.recorder.command(
                    "alarm_latch", "报警锁存检查：" + "；".join(
                        f"{r['name']} {r['before']}→{r['after']}" for r in rows)
                    + ("；不合格 " + "、".join(bad) if bad else "；全部为 0"))
            except Exception:  # noqa: BLE001
                pass
        return {"ok": not bad, "bad": bad, "rows": rows}

    def recover(self) -> dict:
        """16.4 第 7 条：中止后保持状态的出口“恢复”。"""
        self._reject_if_parking()
        self._mark_operation("recover")
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("还在运动/预演，不能恢复")
        if self.motion.status.get("phase") not in ("aborted", "error", "stopping"):
            raise MotionError("当前不在中止保持状态")
        # 27.2 第 3 条：已经失能过就不再使能/恢复
        if hasattr(self.bus, "latch_status") and self.bus.latch_status().get("disabled_latch"):
            raise MotionError(
                "已经失能过：按 27.2 第 3 条这一次上电里不再使能、也不恢复。"
                "请拉空开断电后重上电")
        lat = self.check_alarm_latch()
        if not lat["ok"]:
            raise MotionError("恢复前报警码不是 0，拒绝运动：" + "，".join(lat["bad"]))
        res = self.bus.hold_all()
        un = (res or {}).get("unconfirmed") or []
        if un:
            raise MotionError("恢复前保持未确认：" + "，".join(
                f"{x.get('name')}({x.get('reason')})" for x in un))
        with self.motion._lock:
            self.motion.status.update({"phase": "idle", "reason": "", "label": ""})
        self.hold["since"] = None
        self.hold["warn"] = ""
        self.hold_failure = None
        self._hold_guard_reset()
        self.pending = None
        self.preview = None
        self.msg = "已恢复：原地保持；可以重新预演或执行"
        return {"ok": True, "msg": self.msg}

    def support_park(self, confirmed: bool = False) -> dict:
        """扶持停机（备用）：回零位 + 用户扶住 + 二次确认 → 六颗一起失能。"""
        self._reject_if_parking()
        if not confirmed:
            raise MotionError("扶持停机要二次确认（提示：臂在零位且已扶住）")
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动；扶持停机要求臂已在零位并扶住")
        q = self.motion.read_q()
        if max(abs(float(v)) for v in q) > 2.0:
            raise MotionError("臂不在零位；先回零位并用手扶住前臂，再扶持停机")
        ok = self.bus.disable_all(True, "扶持停机（用户扶住）")
        self.park_state = dict(self.park_state or {})
        self.park_state["valid"] = False
        self.park_state["ts"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.park_state["reason"] = "扶持停机"
        self._save_park_state_safe(self.park_state)
        self.park_step = {"phase": "support_parked", "reason": "", "I0": None, "I1": None}
        self.pending = None
        self.preview = None
        self.msg = "扶持停机：已六颗一起失能"
        return {"ok": bool(ok)}

    # ---------------- 22.2 往返扫描标定 ----------------
    def sweep_plan(self, joint: str) -> dict:
        """只规划不运动：明天真机先用这个确认三条扫描路径都不会被拦。"""
        if not self.motion:
            raise MotionError("还没连接")
        return sweepmod.plan_sweep(self.motion, str(joint).upper())

    def sweep_status(self) -> dict:
        return {"ok": True, **(dict(self.sweep_runner.status)
                               if self.sweep_runner else
                               {"active": False, "phase": "idle"})}

    def start_sweep(self, joint: str, speed_deg_s: float = 3.0,
                    save: bool = True) -> dict:
        self._reject_if_parking()
        j = str(joint).upper()
        if j not in sweepmod.SWEEP_JOBS:
            raise MotionError("扫描只支持 J2 / J3 / J4（J5 等夹爪装回）")
        if sweepmod.SWEEP_JOBS[j].get("defer"):
            raise MotionError(f"{j}：{sweepmod.SWEEP_JOBS[j]['note']}")
        self._ready_for_motion()
        if self.sweep_runner and self.sweep_runner.status.get("active"):
            raise MotionError("22.2 扫描已经在跑")
        with self._exec_lock:
            if self._exec_thread and self._exec_thread.is_alive():
                raise MotionError("另一个执行线程还在跑，先按停")
            runner = sweepmod.SweepRunner(self, j, speed_deg_s, save)
            self.sweep_runner = runner
            self.motion.begin_command()
            th = runner.start()
            self._exec_thread = th
        self.msg = f"22.2 {j} 扫描开始：{speed_deg_s:g}°/s"
        return {"ok": True, "sweep": self.sweep_status()}

    def stop_sweep(self) -> dict:
        if not self.sweep_runner:
            return {"ok": True, "sweep": self.sweep_status()}
        self.sweep_runner.stop()
        self.msg = "22.2 扫描按停"
        return {"ok": True, "sweep": self.sweep_status()}

    def sweep_save_fit(self) -> dict:
        """把当前 runner 的拟合结果写进 config/kt.json（判据不过也允许存，标 ok=false）。"""
        if not self.sweep_runner or not self.sweep_runner.status.get("fit"):
            raise MotionError("还没有扫描拟合结果")
        fit = dict(self.sweep_runner.status["fit"])
        j = fit.get("joint") or self.sweep_runner.joint
        data = dict(self.kt or {})
        data[j] = {k: v for k, v in fit.items() if k != "checks"}
        data["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        gravity.save_kt(data)
        self.kt = gravity.load_kt()
        self._sync_motion_params()
        return {"ok": bool(fit.get("ok")), "kt": self.kt.get(j),
                "fit": fit, "capacities": self.capacities()}

    # ---------------- 22.4 关键点示教录放 ----------------
    def program_record(self, name: str = "", dwell_s: float = 0.0) -> dict:
        if not self.motion or not self.connected:
            raise MotionError("还没连接")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没使能，不能读关节角")
        if self.park_active or self.program_status.get("active"):
            raise MotionError("停机/回放进行中，不能记点")
        q = self.motion.read_q()
        i = len(self.program_points)
        pt = {"name": str(name or f"p{i + 1}"), "q_deg": [round(float(v), 5) for v in q],
              "speed_scale": 1.0, "dwell_s": max(0.0, float(dwell_s)),
              "gripper": None}
        self.program_points.append(pt)
        self.msg = f"22.4 已记第 {i + 1} 点 {pt['name']}"
        return {"ok": True, "point": pt, "recorded": self.program_points}

    def program_clear(self) -> dict:
        self.program_points = []
        return {"ok": True, "recorded": []}

    def _refresh_program_names(self):
        try:
            self._program_names_cache = self.program_store.list()
        except Exception:  # noqa: BLE001
            self._program_names_cache = []
        return self._program_names_cache

    def program_list(self) -> dict:
        return {"ok": True, "names": self._refresh_program_names(),
                "recorded": self.program_points}

    def program_save(self, name: str, overwrite: bool = True) -> dict:
        if not self.program_points:
            raise MotionError("还没有记点")
        r = self.program_store.save(name, self.program_points,
                                    self.end_effector, float(self.payload_kg or 0.0),
                                    overwrite=overwrite)
        self.msg = f"22.4 程序已存：{r['name']}（{r['points']} 点）"
        return {**r, "names": self._refresh_program_names()}

    def program_load(self, name: str) -> dict:
        d = self.program_store.load(name)
        fails = progmod.validate_program(self.motion, d) if self.motion else []
        if fails:
            self.program_loaded = None
            detail = "；".join(f"第 {f['index']} 点：{f['reason']}" for f in fails[:6])
            raise MotionError(f"载入被拒：{detail}")
        self.program_loaded = d
        self.msg = f"22.4 已载入 {d.get('name')}（{len(d.get('points') or [])} 点）"
        warn = ""
        if (d.get("end_effector") != self.end_effector
                or abs(float(d.get("payload_kg") or 0.0)
                       - float(self.payload_kg or 0.0)) > 1e-9):
            warn = (f"程序和当前末端/载荷不同：录制时 {d.get('end_effector')} "
                    f"+{d.get('payload_kg')} kg，现在 {self.end_effector} "
                    f"+{self.payload_kg} kg；回放要确认")
        return {"ok": True, "program": {
            "name": d.get("name"), "points": len(d.get("points") or []),
            "end_effector": d.get("end_effector"),
            "payload_kg": d.get("payload_kg")},
            "warn": warn, "names": self._refresh_program_names()}

    def program_replay(self, name: str = "", loops: int = 1,
                       preview: bool = False, confirmed: bool = False,
                       only_index: int | None = None) -> dict:
        self._reject_if_parking()
        self._ready_for_motion(allow_hold=True)
        if self.program_status.get("active"):
            raise MotionError("22.4 回放已经在跑")
        prog = self.program_loaded
        if not name and prog:
            name = prog.get("name")
        if not name:
            raise MotionError("缺程序名")
        if not prog or prog.get("name") != name:
            prog = self.program_store.load(name)
            fails = progmod.validate_program(self.motion, prog)
            if fails:
                detail = "；".join(f"第 {f['index']} 点：{f['reason']}" for f in fails[:6])
                raise MotionError(f"载入被拒：{detail}")
            self.program_loaded = prog
        mismatch = (prog.get("end_effector") != self.end_effector
                    or abs(float(prog.get("payload_kg") or 0.0)
                           - float(self.payload_kg or 0.0)) > 1e-9)
        if mismatch and not confirmed:
            raise MotionError("程序和当前末端/载荷不同；确认后加 confirmed=true 再回放")
        q_now = self.motion.read_q()
        steps = progmod.plan_replay(self.motion, prog, q_now,
                                    float(self.speed_scale),
                                    None if only_index is None else int(only_index) - 1)
        if not steps:
            raise MotionError("程序没有可回放的点")
        if preview:
            return {"ok": True, "preview": True, "name": name, "loops": loops,
                    "steps": [{"point": s["point_index"], "name": s["point_name"],
                               "segments": len(s["segments"])} for s in steps]}
        with self._exec_lock:
            if self._exec_thread and self._exec_thread.is_alive():
                raise MotionError("另一个执行线程还在跑，先按停")
            self.program_status.update({
                "active": True, "phase": "running", "name": name,
                "loops": max(1, int(loops)), "loop": 0, "point": 0,
                "reason": "", "started": time.time(), "finished": None})
            self.motion.begin_command()
            th = threading.Thread(target=self._run_program,
                                  args=(steps, max(1, int(loops)), name),
                                  daemon=True, name="program-replay")
            self._exec_thread = th
            th.start()
        self.msg = f"22.4 回放开始：{name}（{len(steps)} 点 × {max(1, int(loops))} 遍）"
        return {"ok": True, "replay": dict(self.program_status)}

    def _run_program(self, steps, loops, name):
        m = self.motion
        st = self.program_status
        try:
            for loop in range(loops):
                st["loop"] = loop + 1
                for step in steps:
                    st["point"] = step["point_index"]
                    for seg in step["segments"]:
                        if m._stop.is_set():
                            m._abort("22.4 回放按停")
                        m.execute([seg], preview=False, clear_stop=False)
                    dwell = float(step.get("dwell_s") or 0.0)
                    if dwell > 0:
                        end = time.monotonic() + dwell
                        while time.monotonic() < end:
                            if m._stop.is_set():
                                m._abort("22.4 回放按停")
                            time.sleep(0.05)
            st.update({"active": False, "phase": "done", "finished": time.time(),
                       "reason": ""})
            self.msg = f"22.4 回放完成：{name}"
        except Exception as exc:  # noqa: BLE001
            st.update({"active": False, "phase": "aborted",
                       "reason": str(exc), "finished": time.time()})
            self.msg = f"22.4 回放中止：{exc}"

    # ---------------- 18.4 Kt 标定 ----------------
    def kt_points(self) -> dict:
        return {"ok": True, "points": gravity.KT_POINTS,
                "j5_points": gravity.KT_J5_POINTS,
                "kt": self.kt, "rows": self.kt_rows,
                "note": "J5 一组要等夹爪装回；挂重只在保持状态下加减，人不要站在 +x"}

    def kt_goto(self, q, preview: bool = False) -> dict:
        self._reject_if_parking()
        return self.move_joints(q, preview=preview, descend=False)

    def kt_sample(self, joints, load_kg: float = 0.0, label: str = "") -> dict:
        self._reject_if_parking()
        if not self.use_actual():
            raise MotionError("Kt 标定要连真机并已使能")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没使能")
        names = [str(n).upper() for n in (joints or [])]
        if not names:
            raise MotionError("缺 joints")
        # 18.4：每个点到位后零速保持 2 s 再取样
        time.sleep(2.0)
        q = self.motion.read_q()
        tgt = (self.motion.status or {}).get("target_q")
        tau = gravity.torques(self.robot, q, "flange", float(load_kg))
        cur = self._sample_currents(names)
        fs, lim, raw = {}, {}, {}
        for n in names:
            fs[n] = self.bus.current_full_scale_a(n)      # 0x53 原始满量程
            lim[n] = self.bus.current_limit_a(n)          # 0x59 × 0x53，A
            raw[n] = self.bus.current_raw_counts(n)       # (0x04, 0x59) 原始计数
        rows = []
        for n in names:
            i = cur.get(n)
            if i is None:
                raise MotionError(f"{n} 读不到电流")
            idx = int(n[1]) - 1
            err = None
            if tgt and len(tgt) == 6:
                err = round(abs(float(tgt[idx]) - float(q[idx])), 3)
            row = {"joint": n, "current": round(float(i), 5),
                   "tau": round(float(tau[idx]), 5),
                   "load_kg": float(load_kg), "label": label,
                   "q": [round(float(v), 3) for v in q],
                   "err_deg": err,
                   "full_scale_a": fs.get(n),
                   "limit_a": lim.get(n),
                   "raw_current": (raw.get(n) or (None, None))[0],
                   "raw_limit": (raw.get(n) or (None, None))[1]}
            self.kt_rows.append(row)
            rows.append(row)
        return {"ok": True, "rows": rows, "all_rows": self.kt_rows}

    def kt_fit(self, save: bool = True) -> dict:
        fit = gravity.fit_kt(self.kt_rows) if self.kt_rows else {}
        for name, rec in fit.items():
            rec["kt_date"] = time.strftime("%Y-%m-%d")
        if save and fit:
            data = dict(self.kt or {})
            data.update(fit)
            data["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
            gravity.save_kt(data)
            self.kt = gravity.load_kt()
            self._sync_motion_params()
            self.refresh_config_version()
        yes = [n for n, r in fit.items()
               if r.get("r2") is not None and r["r2"] >= 0.95]
        return {"ok": True, "fit": fit, "saved": bool(save and fit),
                "r2_ok": yes, "capacities": self.capacities()}

    def kt_reset(self) -> dict:
        self.kt_rows = []
        return {"ok": True, "rows": []}

    # ---------------- 维护：只写限位/限流/限速，不动零位 ----------------
    def write_limits_all(self) -> dict:
        self._reject_if_parking()
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.config_stale:
            raise MotionError(self.config_stale_msg or "配置已变，请重新连接")
        if self.calib.active or self.motion.status.get("busy"):
            raise MotionError("校准/运动进行中，先停")
        if self.drag_active:
            raise MotionError("零扭矩拖动中，先锁定保持")
        self.pending = None
        self.preview = None
        rows = []
        for name in self.bus.order:
            m = self.bus.motors[name]
            c = self.bus.cfg[name]
            was_on = m.power() == 1
            if not was_on:
                state_changed = True
                try:
                    m.set_mode(MODE_CURRENT)
                    m.set_current_a(0.0)
                except Exception:  # noqa: BLE001
                    pass
                if not m.enable():
                    raise MotionError(f"{name} 使能无应答")
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
                time.sleep(0.15)
            lo, hi = c.eeprom_limits_deg
            lo_t = lo / 10.0
            hi_t = hi / 10.0
            m.set_pos_limits_turns(lo_t, hi_t)
            if c.required_current_limit_a is not None:
                m.set_current_limit_a(c.required_current_limit_a)
            m.set_speed_limit_out_deg_s(c.eeprom_speed_limit_out_deg_s)
            m.save()
            time.sleep(0.3)
            lim = m.pos_limits_turns()
            spd = m.speed_limit_rpm()
            cur = m.current_limit_a()
            if cur is not None and hasattr(self.bus, "current_limit_readback"):
                self.bus.current_limit_readback[name] = float(cur)
            ok = (lim is not None and abs(lim[0] - lo_t) < 0.01
                  and abs(lim[1] - hi_t) < 0.01
                  and spd is not None and abs(spd - 360.0) < 1.0)
            if c.required_current_limit_a is not None:
                ok = ok and cur is not None and abs(cur - c.required_current_limit_a) < 0.05
            rows.append({"name": name, "ok": bool(ok),
                         "limits_deg": [lo, hi],
                         "limits_readback": None if lim is None else
                         [round(lim[0] * 10.0, 1), round(lim[1] * 10.0, 1)],
                         "current_limit_a": None if cur is None else round(cur, 2),
                         "speed_limit_rpm": None if spd is None else round(spd, 1)})
            if was_on:
                pos = m.pos_turns()
                if pos is not None:
                    m.set_target_turns(pos)
                    m.set_mode(MODE_PROFILE_POSITION)
                    self.bus._write_profile(name, c.motion_speed_deg_s)
                    m.set_target_turns(pos)
            else:
                m.disable()
        bad = [r["name"] for r in rows if not r["ok"]]
        self.msg = ("限位/限速已按配置写入并回读" if not bad
                    else "写限位回读不符：" + ", ".join(bad))
        return {"ok": not bad, "rows": rows, "msg": self.msg}

    # ---------------- 清除报警锁存（不发运动/使能） ----------------
    def clear_errors(self, joint: str | None = None) -> dict:
        """A4：已经使能的关节只清报警，不切模式、不失能。

        读不到电源状态就重试；还读不到就不动这颗并报出来。
        只有确认了“电源 ≠ 1”或“模式 ≠ 6”才把 startup_done 置假。
        """
        self._reject_if_parking()
        if not self.bus:
            raise MotionError("还没连接")
        if self.calib.active or self.drag_active or self.motion.status.get("busy"):
            raise MotionError("校准/拖动/运动进行中，先停")
        names = [joint.upper()] if joint else list(self.bus.order)
        state_changed = False
        mode_bad: list[str] = []
        rows = []
        for name in names:
            if name not in self.bus.motors:
                raise MotionError(f"未知关节 {name}")
            m = self.bus.motors[name]
            row = {"name": name, "cleared": False, "before": None,
                   "before_text": None, "after": None, "after_text": None,
                   "pos_deg": None, "out_of_limit": None, "error": "",
                   "was_enabled": None, "mode_after": None}
            # 读电源状态：最多三次；读不到就完全不动这颗
            pw = None
            for _ in range(3):
                try:
                    pw = m.power(timeout=0.08, retries=1)
                except Exception:  # noqa: BLE001
                    pw = None
                if pw is not None:
                    break
                time.sleep(0.05)
            if pw is None:
                row["error"] = "读不到电源状态（0x2B），未对这颗粒动任何东西"
                rows.append(row)
                continue
            row["was_enabled"] = (pw == 1)
            before = None
            try:
                before = m.error(timeout=0.08, retries=1)
            except Exception:  # noqa: BLE001
                before = None
            row["before"] = None if before is None else f"0x{before:04X}"
            row["before_text"] = None if before is None else describe_error(before)
            # 27.2 第 5 条：失能的关节只发清报警指令，不使能（旧 A4 会先零扭矩使能再失能）
            if pw != 1:
                state_changed = True
                row["note"] = "这颗是失能状态：只清报警，不使能"
                try:
                    before = m.error(timeout=0.08, retries=1)
                except Exception:  # noqa: BLE001
                    before = None
                row["before"] = None if before is None else f"0x{before:04X}"
                row["before_text"] = None if before is None else describe_error(before)
            try:
                pp = m.pos_turns(timeout=0.1, retries=2)
                if pp is not None:
                    row["pos_deg"] = round(float(self.bus.cfg[name].out_deg(pp)), 2)
            except Exception:  # noqa: BLE001
                pass
            cleared = False
            try:
                cleared = bool(m.clear_error())
            except Exception:  # noqa: BLE001
                cleared = False
            time.sleep(0.12)
            after = None
            try:
                after = m.error(timeout=0.08, retries=1)
            except Exception:  # noqa: BLE001
                after = None
            row["cleared"] = cleared
            row["after"] = None if after is None else f"0x{after:04X}"
            row["after_text"] = None if after is None else describe_error(after)
            # 使能着的关节：A4 要求只清报警，不切模式、不失能；只核对模式
            if pw == 1:
                try:
                    row["mode_after"] = m.mode(timeout=0.08, retries=1)
                except Exception:  # noqa: BLE001
                    row["mode_after"] = None
                if row["mode_after"] not in (MODE_PROFILE_POSITION, None):
                    mode_bad.append(f"{name}(mode={row['mode_after']})")
            lo_s, hi_s = self.bus.cfg[name].safe_limits_deg
            if row["pos_deg"] is not None:
                row["out_of_limit"] = bool(row["pos_deg"] < lo_s - 0.5
                                           or row["pos_deg"] > hi_s + 0.5)
            rows.append(row)
        bad = [r["name"] for r in rows if r.get("error")
               or r["after"] not in (None, "0x0000")]
        out_bad = [r["name"] for r in rows if r.get("out_of_limit")]
        self.msg = ("报警锁存已清除；请先手查机械，再重新使能" if not bad
                    else "清除后仍有报警：" + ", ".join(bad))
        if out_bad:
            self.msg += "；但这些关节当前位置超出安全限位，先断电手动移回：" + ", ".join(out_bad)
        if state_changed:
            # 27.2 第 3/5 条：失能的关节只清报警、不使能；这一次上电不许再使能
            self.bus.startup_done = False
            self.pending = None
            self.preview = None
            lat = ({}
                   if not hasattr(self.bus, "latch_status") else self.bus.latch_status())
            if lat.get("disabled_latch"):
                self.msg += "；有电机是失能状态，本次上电已经失能过，不再使能：请拉空开断电后重上电"
            else:
                self.msg += "；有电机确认是失能状态，清报警没有使能它们（要走启动序列请确认臂已支撑）"
        elif mode_bad:
            self.bus.startup_done = False
            self.pending = None
            self.preview = None
            self.msg += "；确认有电机的模式不是 S 曲线位置模式：" + "，".join(mode_bad) \
                        + "（已置为未使能）"
        return {"ok": not bad and not out_bad, "rows": rows, "msg": self.msg}

    # ---------------- 26.1 AP-1：只写内存的 PID 调试 ----------------
    def write_pid_param(self, joint: str, cmd: int, value: float,
                        confirmed: bool = False) -> dict:
        """AP-1 用：写 PID 类参数（只进内存，不发 0x0D）。"""
        if not confirmed:
            raise MotionError("写电机 PID 参数要二次确认（只进内存，不存储）")
        if not self.bus or not isinstance(self.bus, JointBus):
            raise MotionError("只对真机开放")
        j = str(joint or "").upper()
        if j not in self.bus.motors:
            raise MotionError(f"未知关节 {j}")
        m = self.bus.motors[j]
        ok = m.write_pid_iq24(int(cmd), float(value))
        time.sleep(0.2)
        back = m.read_pid_iq24(int(cmd))
        if self.recorder:
            try:
                self.recorder.command(
                    "pid_write", f"{j} 0x{int(cmd):02X} 写 {float(value):g} 回读 {back}",
                    joint=j)
            except Exception:  # noqa: BLE001
                pass
        return {"ok": bool(ok and back is not None and abs(back - float(value)) < 5e-4),
                "joint": j, "cmd": int(cmd), "value": float(value), "readback": back}

    def read_pid_param(self, joint: str, cmd: int) -> dict:
        if not self.bus or not isinstance(self.bus, JointBus):
            raise MotionError("只对真机开放")
        j = str(joint or "J3").upper()
        m = self.bus.motors.get(j)
        if m is None:
            raise MotionError(f"未知关节 {j}")
        v = m.read_pid_iq24(int(cmd))
        return {"ok": v is not None, "joint": j, "cmd": int(cmd), "value": v}

    # ---------------- A8 / 21.5 只读环路参数 ----------------
    READONLY_PARAMS = [
        (0x19, "位置环P", 4), (0x1A, "位置环I", 4), (0x1B, "位置环D", 4),
        (0x17, "速度环P", 4), (0x18, "速度环I", 4),
        (0x79, "位置环滤波器开关", 1), (0x7B, "位置环滤波带宽", 4),
        (0x75, "速度环滤波器开关", 1), (0x77, "速度环滤波带宽", 4),
        (0x38, "位置环输出下限", 4), (0x39, "位置环输出上限", 4),
        (0x1C, "梯形最大速度", 4), (0x1D, "梯形加速度", 4), (0x1E, "梯形减速度", 4),
        (0x7F, "堵转能量", 4), (0x53, "电流满量程", 2), (0x59, "电流限流", 4),
        (0x5B, "速度限流", 4),
    ]

    def readonly_params(self) -> dict:
        """A8：只读环路参数，一个参数都不写。工具走执行器接口。"""
        if not self.bus:
            raise MotionError("还没连接")
        if not isinstance(self.bus, JointBus):
            # mock：给一组可核对的假值，让自检能在 mock 里跑通
            rows = {}
            for name in self.bus.order:
                rows[name] = {label: {"raw_hex": "（mock）", "int32": None,
                                      "iq24": None, "float": None}
                              for _cmd, label, _n in self.READONLY_PARAMS}
            return {"ok": True, "mock": True, "rows": rows,
                    "msg": "mock：只读工具走通（没有真机参数）"}
        if self.motion and self.motion.status.get("busy"):
            raise MotionError("正在运动，先按停再读参数（只读，但会占总线）")
        rows: dict[str, dict] = {}
        for name in self.bus.order:
            mid = self.bus.motors[name].mid
            row = {}
            for cmd, label, n in self.READONLY_PARAMS:
                d = None
                for nn in (n, 4, 2, 1):
                    if nn < n:
                        break
                    try:
                        d = self.bus.link.read(mid, cmd, nn, timeout=0.08, retries=2)
                    except Exception:  # noqa: BLE001
                        d = None
                    if d is not None:
                        break
                if d is None:
                    row[label] = {"raw_hex": None, "note": "无应答"}
                    continue
                item = {"raw_hex": d.hex()}
                if len(d) == 1:
                    item["byte"] = d[0]
                elif len(d) == 2:
                    item["int16"] = struct.unpack(">h", d)[0]
                elif len(d) >= 4:
                    raw = struct.unpack(">i", d[:4])[0]
                    item["int32"] = raw
                    item["iq24"] = round(raw / (2.0 ** 24), 6)
                    try:
                        item["float"] = round(struct.unpack(">f", d[:4])[0], 6)
                    except Exception:  # noqa: BLE001
                        pass
                row[label] = item
            rows[name] = row
        if self.recorder:
            try:
                self.recorder.command("readonly_params", "A8 只读环路参数（未写任何参数）")
            except Exception:  # noqa: BLE001
                pass
        return {"ok": True, "mock": False, "rows": rows}

    def readonly_timing(self, n: int = 500, confirmed: bool = False) -> dict:
        """A8 第 3 条：量 read_all(full=False) 和 set_targets_q 的耗时。"""
        if not confirmed:
            raise MotionError("量通信耗时会在保持状态下重写当前目标，要二次确认")
        if not self.bus:
            raise MotionError("还没连接")
        if self.motion and self.motion.status.get("busy"):
            raise MotionError("正在运动，先按停")
        n = max(20, min(int(n or 500), 2000))
        if isinstance(self.bus, MockJointBus):
            return {"ok": True, "mock": True,
                    "read_all": {"n": n, "median_ms": 0.05, "p99_ms": 0.1,
                                 "max_ms": 0.2, "retries": 0},
                    "set_targets_q": {"n": n, "median_ms": 0.05, "p99_ms": 0.1,
                                      "max_ms": 0.2}}
        def stats(vals):
            vals = sorted(vals)
            def p(q):
                i = min(len(vals) - 1, max(0, int(round(q * (len(vals) - 1)))))
                return round(vals[i] * 1000.0, 3)
            return {"n": len(vals), "median_ms": p(0.5), "p99_ms": p(0.99),
                    "max_ms": p(1.0)}
        t_before = self.bus.link.timeouts
        ra = []
        for _ in range(n):
            t0 = time.monotonic()
            self.bus.read_all(full=False)
            ra.append(time.monotonic() - t0)
        ra_retry = self.bus.link.timeouts - t_before
        st = []
        q = self.motion.read_q() if self.motion else [0.0] * len(self.bus.order)
        tg = {name: float(q[i]) for i, name in enumerate(self.bus.order)}
        for _ in range(n):
            t0 = time.monotonic()
            self.bus.set_targets_q(tg)
            st.append(time.monotonic() - t0)
        return {"ok": True, "mock": False, "read_all": {**stats(ra), "retries": ra_retry},
                "set_targets_q": stats(st)}

    # ---------------- 零扭矩拖动 / 手动测量 ----------------
    def drag_start(self, confirmed: bool = False) -> dict:
        self._reject_if_parking()
        if not confirmed:
            raise MotionError("零扭矩拖动要确认臂已扶住/支撑")
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.mode == "mock":
            raise MotionError("模拟模式不能零扭矩拖动")
        if self.calib.active:
            raise MotionError("校准会话进行中")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没完成启动/使能，不能拖动")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动，先停")
        if self.drag_active:
            return {"ok": True, "drag_active": True}
        self.pending = None
        self.preview = None
        for name in self.bus.order:
            m = self.bus.motors[name]
            try:
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.1)
        bad = []
        for name in self.bus.order:
            m = self.bus.motors[name]
            md = m.mode()
            cur = m.current_a()
            if md is not None and md != MODE_CURRENT:
                bad.append(f"{name} 模式 {md}")
            if cur is not None and abs(cur) > 0.3:
                bad.append(f"{name} 电流 {cur:.2f} A")
        if bad:
            self.drag_stop()
            raise MotionError("切零扭矩失败：" + "；".join(bad))
        self.drag_active = True
        self.msg = "零扭矩拖动中（所有关节 0 A，臂会掉，扶住）"
        return {"ok": True, "drag_active": True, "msg": self.msg}

    def drag_stop(self) -> dict:
        if not self.bus:
            return {"ok": True, "drag_active": False}
        for name in self.bus.order:
            m = self.bus.motors[name]
            c = self.bus.cfg[name]
            try:
                pos = m.pos_turns()
                if pos is None:
                    continue
                m.set_target_turns(pos)
                m.set_mode(MODE_PROFILE_POSITION)
                self.bus._write_profile(name, c.motion_speed_deg_s)
                m.set_target_turns(pos)
            except Exception:  # noqa: BLE001
                pass
        self.drag_active = False
        self.pending = None
        self.preview = None
        bad = self.bus.verify_ready()
        if bad:
            # 再补发一次目标/模式；仍未恢复就要求重新使能
            for n, _why in bad:
                try:
                    mm = self.bus.motors[n]
                    cc = self.bus.cfg[n]
                    pp = mm.pos_turns()
                    if pp is not None:
                        mm.set_target_turns(pp)
                        mm.set_mode(MODE_PROFILE_POSITION)
                        self.bus._write_profile(n, cc.motion_speed_deg_s)
                        mm.set_target_turns(pp)
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(0.1)
            bad = self.bus.verify_ready()
        if bad:
            self.bus.startup_done = False
            self.msg = ("已退出拖动，但这些关节没有恢复 S 曲线位置模式："
                        + "，".join(f"{n}({m})" for n, m in bad)
                        + "；已置为未使能，请点“重新使能（恢复 S 曲线）”")
            return {"ok": True, "drag_active": False, "warning": self.msg,
                    "bad_modes": [{"joint": n, "reason": m} for n, m in bad]}
        self.msg = "已切回位置保持"
        return {"ok": True, "drag_active": False, "msg": self.msg}

    def _write_measure_csv(self, rec: dict):
        import csv
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        new = not self._measure_path.exists()
        with self._measure_path.open("a", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["time", "preset", "x", "y", "z", "actual_x", "actual_y",
                            "actual_z", "err_mm", "q1..q6", "note"])
            w.writerow([rec["time"], rec["preset"],
                        f"{rec['tcp'][0]:.3f}", f"{rec['tcp'][1]:.3f}", f"{rec['tcp'][2]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][0]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][1]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][2]:.3f}",
                        "" if rec["err_mm"] is None else f"{rec['err_mm']:.3f}",
                        " ".join(f"{v:.3f}" for v in rec["q"]), rec.get("note", "")])

    def record_measurement(self, actual=None, note: str = "") -> dict:
        if not self.motion or not self.connected:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动，等停稳再记录")
        q = self.motion.read_q()
        tcp = self.robot.tcp(q, self.tcp_preset, self.yaw_deg)[:3, 3]
        actual_v = None
        if actual is not None:
            try:
                actual_v = [float(actual[0]), float(actual[1]), float(actual[2])]
            except Exception as exc:  # noqa: BLE001
                raise MotionError(f"实际坐标不合法：{exc}") from exc
        err = None
        if actual_v is not None:
            err = float(sum((float(tcp[i]) - actual_v[i]) ** 2 for i in range(3)) ** 0.5)
        rec = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "preset": self.tcp_preset,
            "q": [round(float(v), 4) for v in q],
            "tcp": [round(float(v), 3) for v in tcp],
            "actual": None if actual_v is None else [round(v, 3) for v in actual_v],
            "err_mm": None if err is None else round(err, 3),
            "note": note,
            "drag": bool(self.drag_active),
        }
        self.measurements.append(rec)
        if len(self.measurements) > 200:
            self.measurements = self.measurements[-200:]
        self._write_measure_csv(rec)
        self.msg = f"记录点 #{len(self.measurements)}：{rec['tcp']} mm"
        return {"ok": True, "record": rec, "measurements": self.measurements[-20:]}

    def clear_measurements(self) -> dict:
        self.measurements = []
        self.msg = "测量记录已清空（CSV 保留）"
        return {"ok": True, "measurements": []}

    def reach_tcp(self, target, axis=(0, 0, -1), free_yaw: bool = True) -> dict:
        """拖动时的快速可达性检查：0.25 s 预算，不做路径规划、不发命令。"""
        if not target or len(target) != 3:
            raise MotionError("target 需要 3 个数")
        t = [float(target[0]), float(target[1]), float(target[2])]
        rz = t[2]
        r = (t[0] ** 2 + t[1] ** 2) ** 0.5
        if rz < 20.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "z < 20 mm（围栏/桌面）",
                    "pos_err_mm": None, "axis_err_deg": None}
        if rz < 200.0 and r < 90.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "底座禁区 r<90 且 z<200",
                    "pos_err_mm": None, "axis_err_deg": None}
        if r > 480.0 or rz > 620.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "超出粗略可达范围（r>480 或 z>620）",
                    "pos_err_mm": None, "axis_err_deg": None}
        q0 = self.planning_motion().read_q()
        try:
            res = self.robot.ik(t, axis, None if free_yaw else self.yaw_deg, q0,
                                self.tcp_preset, (self.limits[0], self.limits[1]),
                                max_iters=45, time_budget_s=0.25)
            return {"ok": True, "reachable": bool(res.ok), "target": t,
                    "pos_err_mm": round(float(res.pos_err_mm), 2),
                    "axis_err_deg": round(float(res.axis_err_deg), 2),
                    "q": [round(float(v), 2) for v in res.q_deg]}
        except Exception as exc:  # noqa: BLE001
            return {"ok": True, "reachable": False, "target": t, "reason": str(exc),
                    "pos_err_mm": None, "axis_err_deg": None}

    def reach(self, step: float = 20.0) -> dict:
        pts = []
        for x in range(-200, 601, max(5, int(step))):
            for y in range(-400, 401, max(5, int(step))):
                try:
                    res = self.robot.ik([x, y, 20.0], (0, 0, -1), 0.0,
                                        [0, 0, 0, 0, 0, 0], "probe", self.limits,
                                        structured_seeds=False, max_iters=60)
                    ok = res.ok and self.fence.check(res.q_deg, "probe", 0.0,
                                                     False, self.limits).ok
                except Exception:  # noqa: BLE001
                    ok = False
                pts.append([x, y, 1 if ok else 0])
        return {"ok": True, "step": step, "z": 20.0, "points": pts}

    # ---------------- 主循环 ----------------
    def start_watchdog(self):
        self._watch_stop.clear()
        self._watch_thread = threading.Thread(target=self._watchdog, daemon=True)
        self._watch_thread.start()


def brief_segments(segs):
    return [{k: s.get(k) for k in ("label", "q_from", "q_target", "descend", "T",
                                   "fence", "near_limit", "ik")}
            for s in segs]


def preview_payload(seg, robot, fence, preset, yaw):
    q = seg["q_target"]
    return {
        "segments": brief_segments([seg]),
        "scene": fence.scene(q, preset, yaw),
        "q": q,
        "tcp": robot.tcp(q, preset, yaw)[:3, 3].tolist(),
        "fence": seg["fence"],
        "ik": seg.get("ik"),
        "target": None,
        "yaw": yaw,
        "near_limit": seg.get("near_limit", []),
    }


EXEC: Executor | None = None


class Handler(BaseHTTPRequestHandler):
    server_version = "ArmExecutor/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    # ---------------- 响应 ----------------
    def _cors(self):
        # 20.9.5：只回显执行器自己的地址，不再用 *。
        port = int(getattr(self.server, "server_port", 0) or 8770)
        origin = self.headers.get("Origin") or ""
        allowed = {f"http://127.0.0.1:{port}", f"http://localhost:{port}",
                   f"http://[::1]:{port}"}
        if origin and origin in allowed:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, obj):
        body = json.dumps(obj, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        raw = self.rfile.read(n)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:  # noqa: BLE001
            return {}

    def _static(self, path: str):
        if not EXEC.serve_web:
            return self._json(404, {"ok": False, "error": "web server disabled"})
        rel = path.lstrip("/") or "index.html"
        base = EXEC.web_dir.resolve()
        fp = (base / rel).resolve()
        if not str(fp).startswith(str(base)) or not fp.is_file():
            return self._json(404, {"ok": False, "error": "not found"})
        data = fp.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", MIME.get(fp.suffix, "application/octet-stream"))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _events(self):
        qs = parse_qs(urlparse(self.path).query)
        client_id = (qs.get("client_id") or [""])[0]
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self._cors()
        self.end_headers()
        period = 1.0 / EXEC.push_hz
        try:
            while True:
                data = json.dumps(EXEC.state(), ensure_ascii=False, allow_nan=False)
                self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
                EXEC.touch(client_id)
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        except Exception:  # noqa: BLE001
            traceback.print_exc()
            return

    # ---------------- GET ----------------
    def do_GET(self):
        p = urlparse(self.path).path
        try:
            if p == "/api/state":
                qs = parse_qs(urlparse(self.path).query)
                cid = (qs.get("client_id") or [""])[0]
                EXEC.touch(cid)
                return self._json(200, EXEC.state())
            if p == "/api/events":
                return self._events()
            if p == "/api/config":
                return self._json(200, {"ok": True, **EXEC.config_payload()})
            if p == "/api/session":
                return self._json(200, {"ok": True,
                                        "session": (EXEC.session.id if EXEC.session else None),
                                        "dir": (str(EXEC.session.dir) if EXEC.session else None),
                                        "meta": (EXEC.session.meta if EXEC.session else None)})
            if p == "/api/debug/page_state":
                return self._json(200, EXEC.load_web_state())
            if p == "/api/kt/points":
                return self._json(200, EXEC.kt_points())
            if p == "/api/ping":
                return self._json(200, {"ok": True, "t": time.time()})
            if p == "/api/mock/faults":
                if not isinstance(EXEC.bus, MockJointBus):
                    raise MotionError("仅 mock 模式可用")
                return self._json(200, {"ok": True, **EXEC.bus.mock_faults()})
            if p == "/api/trace/status":
                return self._json(200, {"ok": True,
                                        "trace": (EXEC.trace.status() if EXEC.trace else None)})
            if p == "/api/sweep/status":
                return self._json(200, EXEC.sweep_status())
            if p == "/api/program/list":
                return self._json(200, EXEC.program_list())
            return self._static(p)
        except Exception as exc:  # noqa: BLE001
            return self._json(500, {"ok": False, "error": str(exc)})

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    # ---------------- POST ----------------
    def do_POST(self):
        p = urlparse(self.path).path
        body = self._body()
        try:
            # 控制权 / 心跳
            if p == "/api/acquire":
                r = EXEC.acquire(str(body.get("client_id") or ""),
                                 str(body.get("config_version") or ""),
                                 str(body.get("token") or ""))
                return self._json(200, {"ok": True, **r})
            if p == "/api/heartbeat":
                r = EXEC.heartbeat(str(body.get("token") or ""))
                return self._json(200, r)
            if p == "/api/release":
                return self._json(200, EXEC.release(str(body.get("token") or "")))
            if p == "/api/stop":
                # 停是安全动作：不要求控制权，任何客户端都能停
                return self._json(200, EXEC.stop_all(
                    "用户按停", source=body.get("source")))
            if p == "/api/test/write_profile":
                EXEC.require_control(body)
                nd = body.get("negative_decel")
                return self._json(200, EXEC.test_write_profile(
                    body.get("q"), None if nd is None else bool(nd)))
            if p == "/api/test/read_params":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_read_params(
                    body.get("joints"), body.get("cmds")))
            if p == "/api/test/write_targets":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_write_targets(body.get("q")))
            if p == "/api/test/skip_resend":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_skip_resend(bool(body.get("on", True))))
            if p == "/api/test/poll_hold":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_poll_hold(bool(body.get("on", True))))
            if p == "/api/test/cvp_crosscheck":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_cvp_crosscheck(
                    int(body.get("n") or 50), body.get("joints") or ("J2", "J3")))
            if p == "/api/test/read_profiles":
                EXEC.require_control(body)
                return self._json(200, EXEC.test_read_profiles())
            if p == "/api/mark":
                # 29.3 用户标记：安全动作，不要求控制权（任何客户端都能标记）
                return self._json(200, EXEC.mark(
                    str(body.get("text") or ""), str(body.get("kind") or "其他")))
            if p == "/api/trace/start":
                EXEC.require_control(body)
                try:
                    hz = float(body.get("hz") or EXEC.trace_hz)
                    sec = float(body.get("seconds") or EXEC.trace_seconds)
                except (TypeError, ValueError) as exc:
                    raise MotionError(f"hz/seconds 不是数字：{exc}")
                return self._json(200, {"ok": True,
                                        "trace": EXEC.start_trace(
                                            hz, sec, body.get("joints"),
                                            float(body.get("status_hz") or 5.0),
                                            bool(body.get("light")),
                                            str(body.get("mode") or ""))})
            if p == "/api/trace/stop":
                EXEC.require_control(body)
                return self._json(200, {"ok": True, "trace": EXEC.stop_trace()})
            if p == "/api/mock/fault":
                # 20.0：仅 mock 的故障注入开关
                if not isinstance(EXEC.bus, MockJointBus):
                    raise MotionError("仅 mock 模式可用")
                EXEC.require_control(body)
                if body.get("clear"):
                    EXEC.bus.clear_faults()
                else:
                    EXEC.bus.set_faults(
                        silent=body.get("silent"),
                        drop=body.get("drop"),
                        delay=body.get("delay"),
                        error_bits=body.get("error_bits"),
                        limits=body.get("limits"),
                        mode=body.get("mode"),
                        currents=body.get("currents"))
                return self._json(200, {"ok": True, **EXEC.bus.mock_faults()})
            if p == "/api/debug/page_state":
                # 调试快照：不要求控制权，任何页面（包括只读）都可写；只写最新一份
                return self._json(200, EXEC.save_web_state(body))
            if p == "/api/reach_tcp":
                return self._json(200, EXEC.reach_tcp(
                    body.get("target"), body.get("axis", (0, 0, -1)),
                    bool(body.get("free_yaw", True))))
            if p == "/api/clear_errors":
                EXEC.require_control(body)
                return self._json(200, EXEC.clear_errors(body.get("joint")))
            if p == "/api/ap1/pid_read":
                EXEC.require_control(body)
                return self._json(200, EXEC.read_pid_param(
                    str(body.get("joint") or "J3"), int(body.get("cmd") or 0x18)))
            if p == "/api/ap1/pid":
                EXEC.require_control(body)
                return self._json(200, EXEC.write_pid_param(
                    str(body.get("joint") or "J3"), int(body.get("cmd") or 0x11),
                    float(body.get("value", 0.0001)), bool(body.get("confirmed"))))
            if p == "/api/readonly/params":
                EXEC.require_control(body)
                return self._json(200, EXEC.readonly_params())
            if p == "/api/readonly/timing":
                EXEC.require_control(body)
                return self._json(200, EXEC.readonly_timing(
                    int(body.get("n") or 500), bool(body.get("confirmed"))))
            if p == "/api/hold_failure/disable_joint":
                EXEC.require_control(body)
                return self._json(200, EXEC.hold_disable_joint(
                    str(body.get("joint") or ""), bool(body.get("confirmed"))))
            if p == "/api/limits/write":
                EXEC.require_control(body)
                if not body.get("confirmed"):
                    raise MotionError("写限位要二次确认")
                return self._json(200, EXEC.write_limits_all())
            if p == "/api/drag/start":
                EXEC.require_control(body)
                return self._json(200, EXEC.drag_start(bool(body.get("confirmed"))))
            if p == "/api/drag/stop":
                EXEC.require_control(body)
                return self._json(200, EXEC.drag_stop())
            if p == "/api/measure/record":
                EXEC.require_control(body)
                return self._json(200, EXEC.record_measurement(
                    body.get("actual"), str(body.get("note") or "")))
            if p == "/api/measure/clear":
                EXEC.require_control(body)
                return self._json(200, EXEC.clear_measurements())
            if p == "/api/measure":
                return self._json(200, {"ok": True, "measurements": EXEC.measurements[-50:],
                                        "preset": EXEC.tcp_preset})
            # ---- 16.3 校准会话（只认控制权，不比对配置版本：校准本身会改配置） ----
            if p == "/api/calib/prepare":
                EXEC.require_token(body)
                EXEC._calib_active = True      # 准备期间先停掉普通轮询，别跟它抢 ECB
                try:
                    r = EXEC.calib.prepare(str(body.get("joint") or ""),
                                           bool(body.get("confirmed_supported")))
                except Exception:
                    EXEC._calib_active = False
                    raise
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/set_zero":
                EXEC.require_token(body)
                r = EXEC.calib.set_zero(str(body.get("joint") or ""))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/jog":
                EXEC.require_token(body)
                r = EXEC.calib.jog(float(body.get("delta", 5.0)))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/confirm_sign":
                EXEC.require_token(body)
                r = EXEC.calib.confirm_sign(int(body.get("sign", 1)))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/return":
                EXEC.require_token(body)
                r = EXEC.calib.return_zero()
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/abort":
                EXEC.require_token(body)
                r = EXEC.calib.abort(reason=str(body.get("reason") or "用户中止校准"))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/connect":
                EXEC.require_control(body)
                if EXEC.is_busy():
                    raise MotionError("正在运动/停机流程/预演，先按停再重连")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                r = EXEC.connect(body.get("mode", "mock"),
                                 body.get("ip") or EXEC.ecb_ip,
                                 int(body.get("port") or EXEC.ecb_port))
                return self._json(200, r)
            if p == "/api/enable":
                EXEC.require_control(body)
                if not EXEC.bus:
                    raise MotionError("还没连接")
                if EXEC.config_stale:
                    raise MotionError(EXEC.config_stale_msg or "配置已变，请重新连接")
                if EXEC.is_busy():
                    raise MotionError("正在运动/停机流程/预演，先按停")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                rep = EXEC.bus.enable_all(
                    bool(body.get("confirmed_supported")),
                    bool(body.get("power_cycle_confirmed")))
                # 35.2 第 4 条：启动前只读到的 0x1C/0x1D/0x1E 记进会话
                try:
                    pb = (rep or {}).get("profile_before") or {}
                    if pb and EXEC.recorder:
                        for n, row in pb.items():
                            EXEC.recorder.command(
                                "profile_readback_before",
                                " ".join(f"{k}={None if v is None else v['raw_hex']}"
                                         for k, v in row.items()), joint=n)
                except Exception:  # noqa: BLE001
                    pass
                EXEC.pending = None
                EXEC.preview = None
                EXEC._mark_operation("enable")
                EXEC._on_shelf = True
                EXEC.msg = "已使能（J6→J1），S 曲线模式就绪"
                check = {"has_parked": False, "ok": None}
                try:
                    q_now = EXEC.motion.read_q()
                    check = EXEC._park_check(q_now)
                    if check.get("has_parked") and check.get("ok") is False:
                        EXEC.park_block = {"max_diff_deg": check.get("max_diff_deg"),
                                           "diffs": check.get("diffs"),
                                           "reason": check.get("reason", ""),
                                           "ts": check.get("ts")}
                        if check.get("reason"):
                            EXEC.msg = (str(check["reason"])
                                        + "。已拒绝一切运动；看过实物后点“已确认”解除。")
                        else:
                            EXEC.msg = (f"启停比对不过：和 parked_q 差 "
                                        f"{check.get('max_diff_deg')}°（{check.get('diffs')}）。"
                                        "已拒绝一切运动；看过实物后点“已确认”解除。")
                    elif check.get("has_parked") and check.get("ok"):
                        EXEC.park_block = None
                        EXEC.msg = ("臂在停机姿态（搁架上）。先点“抬回预停机点”离开搁架，"
                                    "再回零或做别的。")
                except Exception as exc:  # noqa: BLE001
                    EXEC.msg = f"已使能，但读姿态失败：{exc}"
                return self._json(200, {"ok": True, "report": rep, "msg": EXEC.msg,
                                        "park_check": check})
            if p == "/api/ik_all":
                b = body
                return self._json(200, EXEC.ik_all(
                    b.get("target"), b.get("axis", (0, 0, -1)),
                    b.get("yaw"), b.get("preset"),
                    bool(b.get("free_yaw", False))))
            if p == "/api/plan_click":
                EXEC.require_control(body)
                return self._json(200, EXEC.plan_click(
                    float(body["x"]), float(body["y"]),
                    float(body.get("z", 20.0)), body.get("yaw")))
            if p == "/api/click":
                EXEC.require_control(body)
                return self._json(200, EXEC.click(
                    float(body["x"]), float(body["y"]),
                    float(body.get("z", 20.0)), body.get("yaw")))
            if p == "/api/execute":
                EXEC.require_control(body)
                return self._json(200, EXEC.execute(body.get("preview")))
            if p == "/api/jog":
                EXEC.require_control(body)
                sp = body.get("speed_deg_s")
                return self._json(200, EXEC.jog(
                    body["joint"], float(body.get("delta", 1.0)),
                    bool(body.get("preview", False)),
                    speed_deg_s=(None if sp is None else float(sp)),
                    single_target=bool(body.get("single_target", False))))
            if p == "/api/home":
                EXEC.require_control(body)
                return self._json(200, EXEC.home(bool(body.get("preview", False))))
            if p == "/api/move_joints":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_joints(
                    body["q"], bool(body.get("preview", False)),
                    bool(body.get("descend", False))))
            if p == "/api/move_tcp":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_tcp(
                    body["target"], body.get("axis", (0, 0, -1)),
                    body.get("yaw"), bool(body.get("preview", False)),
                    bool(body.get("descend", False)), body.get("preset"),
                    bool(body.get("free_yaw", False))))
            if p == "/api/move_through":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_through(
                    body["waypoints"], body.get("descend_flags"),
                    body.get("labels"), bool(body.get("preview", False))))
            if p == "/api/disable_all":
                EXEC.require_control(body)
                if not EXEC.bus:
                    raise MotionError("还没连接")
                if EXEC.park_active or (EXEC._park_thread and EXEC._park_thread.is_alive()):
                    raise MotionError("停机流程正在跑：先按停，它会停在保持")
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能全部失能；先按停")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                if not body.get("confirmed"):
                    raise MotionError("全部失能要二次确认")
                ok = EXEC.bus.disable_all(True, body.get("reason", "user"))
                EXEC._mark_operation("disable_all")
                EXEC.pending = None
                EXEC.preview = None
                EXEC.msg = "已全部失能"
                return self._json(200, {"ok": bool(ok)})
            if p == "/api/end_effector":
                EXEC.require_control(body)
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能换末端配置")
                return self._json(200, EXEC.set_end_effector(
                    str(body.get("end_effector") or ""), body.get("payload_kg")))
            if p == "/api/park/teach":
                EXEC.require_control(body)
                return self._json(200, EXEC.teach_park())
            if p == "/api/park":
                EXEC.require_control(body)
                return self._json(200, EXEC.park(
                    bool(body.get("preview", False)), body.get("prereq")))
            if p == "/api/park/ack":
                EXEC.require_control(body)
                return self._json(200, EXEC.park_ack())
            if p == "/api/park/unpark":
                EXEC.require_control(body)
                return self._json(200, EXEC.unpark(
                    bool(body.get("preview", False)), bool(body.get("confirmed"))))
            if p == "/api/recover":
                EXEC.require_control(body)
                return self._json(200, EXEC.recover())
            if p == "/api/park/drop":
                EXEC.require_control(body)
                return self._json(200, EXEC.park_drop(bool(body.get("confirmed"))))
            if p == "/api/park/support":
                EXEC.require_control(body)
                return self._json(200, EXEC.support_park(bool(body.get("confirmed"))))
            if p == "/api/debug/page_state":
                return self._json(200, EXEC.load_web_state())
            if p == "/api/sweep/plan":
                return self._json(200, {"ok": True,
                                        "plan": EXEC.sweep_plan(body.get("joint"))})
            if p == "/api/sweep/start":
                EXEC.require_control(body)
                return self._json(200, EXEC.start_sweep(
                    body.get("joint"), float(body.get("speed_deg_s", 3.0)),
                    bool(body.get("save", True))))
            if p == "/api/sweep/stop":
                EXEC.require_control(body)
                return self._json(200, EXEC.stop_sweep())
            if p == "/api/sweep/save":
                EXEC.require_control(body)
                return self._json(200, EXEC.sweep_save_fit())
            if p == "/api/program/record":
                EXEC.require_control(body)
                return self._json(200, EXEC.program_record(
                    str(body.get("name") or ""), float(body.get("dwell_s", 0.0))))
            if p == "/api/program/clear":
                EXEC.require_control(body)
                return self._json(200, EXEC.program_clear())
            if p == "/api/program/save":
                EXEC.require_control(body)
                return self._json(200, EXEC.program_save(
                    str(body.get("name") or ""), bool(body.get("overwrite", True))))
            if p == "/api/program/load":
                EXEC.require_control(body)
                return self._json(200, EXEC.program_load(str(body.get("name") or "")))
            if p == "/api/program/replay":
                EXEC.require_control(body)
                return self._json(200, EXEC.program_replay(
                    str(body.get("name") or ""), int(body.get("loops", 1)),
                    bool(body.get("preview", False)), bool(body.get("confirmed")),
                    body.get("point") if body.get("point") is not None else None))
            if p == "/api/kt/points":
                return self._json(200, EXEC.kt_points())
            if p == "/api/kt/goto":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_goto(body["q"],
                                                    bool(body.get("preview", False))))
            if p == "/api/kt/sample":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_sample(
                    body.get("joints"), float(body.get("load_kg", 0.0)),
                    str(body.get("label") or "")))
            if p == "/api/kt/fit":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_fit(bool(body.get("save", True))))
            if p == "/api/kt/reset":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_reset())
            if p == "/api/tcp":
                EXEC.require_control(body)
                EXEC._reject_if_parking()
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能换 TCP 预设")
                name = body.get("preset", "probe")
                if name not in EXEC.robot.tcp_presets:
                    raise MotionError(f"未知 TCP {name}")
                EXEC.tcp_preset = name
                pr = (EXEC.robot.tcp_presets or {}).get(name) or {}
                if pr.get("yaw_deg") is not None:
                    EXEC.yaw_deg = float(pr["yaw_deg"])
                    if EXEC.motion:
                        EXEC.motion.yaw_deg = EXEC.yaw_deg
                    EXEC.preview_motion.yaw_deg = EXEC.yaw_deg
                if EXEC.motion:
                    EXEC.motion.preset = name
                EXEC.preview_motion.preset = name
                EXEC.preview = None
                EXEC.pending = None
                return self._json(200, {"ok": True, "preset": name,
                                        "yaw": EXEC.yaw_deg})
            if p == "/api/yaw":
                EXEC.require_control(body)
                EXEC._reject_if_parking()
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能改偏航")
                EXEC.yaw_deg = float(body.get("yaw", 0.0))
                if EXEC.motion:
                    EXEC.motion.yaw_deg = EXEC.yaw_deg
                EXEC.preview_motion.yaw_deg = EXEC.yaw_deg
                return self._json(200, {"ok": True, "yaw": EXEC.yaw_deg})
            if p == "/api/idle/continue":
                EXEC.require_control(body)
                return self._json(200, EXEC.idle_continue())
            if p == "/api/speed_scale":
                EXEC.require_control(body)
                return self._json(200, EXEC.set_speed_scale(
                    float(body.get("scale", 0.5))))
            if p == "/api/exec_mode":
                EXEC.require_control(body)
                EXEC._reject_if_parking()
                mode = body.get("mode", "preview")
                if mode not in ("preview", "direct"):
                    raise MotionError("mode 只能是 preview 或 direct")
                EXEC.exec_mode = mode
                EXEC.msg = f"执行模式：{'直接执行' if mode == 'direct' else '预演'}"
                return self._json(200, {"ok": True, "exec_mode": mode})
            if p == "/api/reach":
                EXEC.require_control(body)
                return self._json(200, EXEC.reach(float(body.get("step", 20.0))))
            if p == "/api/msg":
                EXEC.msg = str(body.get("msg", ""))
                return self._json(200, {"ok": True})
            return self._json(404, {"ok": False, "error": "no such api"})
        except AuthError as exc:
            return self._json(409, {"ok": False, "error": str(exc)})
        except (MotionError, StartupRefused) as exc:
            EXEC.msg = str(exc)
            return self._json(400, {"ok": False, "error": str(exc)})
        except KeyError as exc:
            return self._json(400, {"ok": False, "error": f"缺字段 {exc}"})
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            return self._json(500, {"ok": False, "error": str(exc)})


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main(argv=None) -> int:
    global EXEC
    ap = argparse.ArgumentParser(description="阶段一 A 执行器（独立后台程序）")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--ecb-ip", default=None)
    ap.add_argument("--ecb-port", type=int, default=None)
    ap.add_argument("--mock", action="store_true", help="启动即连模拟臂")
    ap.add_argument("--open", action="store_true", help="打开浏览器")
    ap.add_argument("--no-web", action="store_true", help="只提供 API，不服务网页")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--config", default=None,
                    help="执行器配置文件（默认 config/executor.json）。"
                         "28.3 第 2 条：笔记本上用它指仓库外的 mock-only 配置")
    args = ap.parse_args(argv)

    if args.config:
        import json as _json
        from pathlib import Path as _Path
        _p = _Path(args.config)
        if not _p.exists():
            print(f"✗ 配置文件不存在：{_p}", file=sys.stderr)
            return 2
        try:
            cfg = dict(_json.loads(_p.read_text(encoding="utf-8")))
        except Exception as exc:  # noqa: BLE001
            print(f"✗ 配置文件读不了：{exc}", file=sys.stderr)
            return 2
    else:
        cfg = cfgmod.load_executor()
    if args.host:
        cfg["bind_host"] = args.host
    if args.port:
        cfg["bind_port"] = args.port
    if args.ecb_ip:
        cfg["ecb_ip"] = args.ecb_ip
    if args.ecb_port:
        cfg["ecb_udp_port"] = args.ecb_port
    if args.no_web:
        cfg["serve_web"] = False
    EXEC = Executor(cfg)
    EXEC.start_watchdog()
    EXEC.start_poll()
    if args.mock or cfg.get("mock_on_start"):
        EXEC.connect("mock")
    srv = Server((EXEC.bind_host, EXEC.bind_port), Handler)
    url = f"http://{EXEC.bind_host}:{EXEC.bind_port}/"
    print(f"执行器 {url}  config_version={EXEC.config_version}  mode={EXEC.mode}  {EXEC.msg}")
    if args.open:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        EXEC.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
