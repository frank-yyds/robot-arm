#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十九节 29.1/29.2：会话目录、三层记录、事故快照和自动 summary.md。

一个会话 = `runs/<日期时间>/`：
  * `meta.json`      会话头（29.1 第 4 条）：git 提交号/未提交改动、配置 sha、网页版本、
                     主机名、网口、六颗参数快照、上一份参数快照的差异
  * `state.csv`      逐拍状态（29.1 第 2 条）：一轮回读写一行
  * `events.jsonl`   事件（29.1 第 3 条）：一行一件
  * `incidents/<编号>/`  事故快照（29.2）：`summary.md` + `state_pre.csv`/`state_post.csv`
                     + `events_pre.jsonl`/`events_post.jsonl`（真抓包时再加 `pcap_slice.pcap`）

写盘全部走同一个后台线程和队列，不拖慢通信线程（29.1 第 2 条）。
所有时间戳都用同一台机器的时钟：`mono` = time.monotonic()，`wall` = 墙上时间。
"""
from __future__ import annotations

import collections
import datetime as _dt
import json
import os
import platform
import queue
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path

from . import config

STATE_HEADER = ("mono,wall,round,joint,target_deg,target_written_at_ms,pos_deg,"
                "vel_deg_s,current_a,rho,read_ok,mode,mode_age_s,enabled,error")
INCIDENT_PRE_S = 30.0
INCIDENT_POST_S = 30.0
INCIDENT_SUMMARY_S = 10.0
MAX_RUNS_BYTES = 20 * 1024 ** 3     # 29.7 第 4 条：记录目录超过 20 GB 删最旧会话


def _iso(ts: float | None = None) -> str:
    t = _dt.datetime.fromtimestamp(ts if ts is not None else time.time())
    return t.isoformat(timespec="milliseconds")


def _git_info(root: Path) -> dict:
    out = {}
    try:
        out["commit"] = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=str(root),
            stderr=subprocess.DEVNULL).decode().strip()
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=str(root),
            stderr=subprocess.DEVNULL).decode().strip()
        out["dirty"] = bool(dirty)
        out["dirty_list"] = [ln[3:] for ln in dirty.splitlines()[:40]]
    except Exception:  # noqa: BLE001
        out["commit"] = None
        out["dirty"] = None
    return out


def runs_bytes(root: Path) -> int:
    total = 0
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, fn))
            except OSError:
                pass
    return total


# 37.3 第 2 条查到：Executor.connect() 每次都调 enforce_quota，而 runs/ 大时
# runs_bytes() 要 0.5 s（笔记本 14.9 GB / 1346 个目录）；自检一进程里连 100 次就卡死。
# 这里限频：同一个 runs 根目录 10 分钟内最多扫一次。
_QUOTA_CACHE: dict[str, tuple[float, dict]] = {}
_QUOTA_MIN_INTERVAL_S = 600.0


class Session:
    """一次连接（会话）的记录器。线程安全；写盘在独立线程。"""

    def __init__(self, root: Path | None = None, tag: str = "",
                 session_id: str | None = None):
        self.root = Path(root or config.RUNS_DIR)
        stamp = session_id or (time.strftime("%Y%m%d_%H%M%S")
                               + (f"_{tag}" if tag else ""))
        self.dir = self.root / stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self.id = stamp
        self.meta_path = self.dir / "meta.json"
        self.state_path = self.dir / "state.csv"
        self.events_path = self.dir / "events.jsonl"
        self.incidents_dir = self.dir / "incidents"
        self._lock = threading.Lock()
        self._q: queue.Queue = queue.Queue(maxsize=20000)
        self._stop = threading.Event()
        self._drop = 0
        self._round = 0
        self._t0_mono = time.monotonic()
        self._state_fh = self.state_path.open("w", encoding="utf-8")
        self._state_fh.write(STATE_HEADER + "\n")
        self._state_fh.flush()
        self._events_fh = self.events_path.open("w", encoding="utf-8")
        self._thread = threading.Thread(target=self._writer, daemon=True)
        self._thread.start()
        # 事故快照：最近 30 s 的 state/event 行（内存环形缓冲）
        self._ring_state: collections.deque = collections.deque(maxlen=8000)
        self._ring_event: collections.deque = collections.deque(maxlen=4000)
        self._incident_n = 0
        self._incident_post: dict | None = None
        self.meta: dict = {"session": self.id, "started": _iso(),
                           "root": str(self.root)}
        self._param_snapshot: dict | None = None
        self._capture_pid: int | None = None
        # 当前流程/步号（执行器每帧状态推过来），给事故快照用
        self.ctx: dict = {"flow": "", "phase": "", "step": ""}
        self._write_meta()

    # ---------------- 会话头 ----------------
    def write_meta(self, **fields):
        with self._lock:
            self.meta.update(fields)
            self._write_meta()

    def _write_meta(self):
        try:
            tmp = self.meta_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(self.meta, ensure_ascii=False, indent=1),
                           encoding="utf-8")
            tmp.replace(self.meta_path)
        except Exception:  # noqa: BLE001
            pass

    def header(self, connector: str, config_version: str, web_version: str,
               joints_cfg: dict, bus_stats: dict | None = None,
               iface: str = "") -> None:
        params = self.params_snapshot(joints_cfg, bus_stats or {})
        with self._lock:
            self.meta.update({
                "connector": connector,
                "host": socket.gethostname(),
                "platform": platform.platform(),
                "python": platform.python_version(),
                "config_version": config_version,
                "config_shas": config.config_versions(),
                "web_version": web_version,
                "iface": iface,
                "git": _git_info(config.ROOT),
                "params": params["current"],
                "params_diff": params["diff"],
                "params_changed_outside": params["changed_outside"],
            })
            self._write_meta()

    def set_ctx(self, **kw):
        with self._lock:
            self.ctx.update({k: v for k, v in kw.items() if v is not None})

    def attach_capture(self, pid: int | None):
        self._capture_pid = pid
        with self._lock:
            self.meta["capture_pid"] = pid
            self._write_meta()

    def params_snapshot(self, joints_cfg: dict, bus_stats: dict) -> dict:
        """29.4：六颗能读到的参数快照 + 和上一份比。"""
        def pick(cfg):
            d = {}
            for name, c in (cfg or {}).items():
                d[name] = {
                    "current_limit_a": bus_stats.get("limits", {}).get(name),
                    "speed_limit_rpm": bus_stats.get("speed_limits", {}).get(name),
                    "pos_limits_r": [getattr(c, "eeprom_limits_deg", [None, None])[0] / 10.0
                                     if getattr(c, "eeprom_limits_deg", None) else None,
                                     getattr(c, "eeprom_limits_deg", [None, None])[1] / 10.0
                                     if getattr(c, "eeprom_limits_deg", None) else None],
                    "pos_kp": bus_stats.get("pid", {}).get(name, {}).get("pos_p"),
                    "vel_kp": bus_stats.get("pid", {}).get(name, {}).get("vel_p"),
                    "vel_ki": bus_stats.get("pid", {}).get(name, {}).get("vel_i"),
                }
            return d

        prev_path = self.root / "_last_params.json"
        prev = {}
        try:
            prev = json.loads(prev_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            prev = {}
        cur = pick(joints_cfg)
        diff = {}
        for name, vals in cur.items():
            old = prev.get(name) or {}
            for k, v in vals.items():
                if old.get(k) != v:
                    diff.setdefault(name, {})[k] = [old.get(k), v]
        try:
            prev_path.write_text(json.dumps(cur, ensure_ascii=False, indent=1),
                                 encoding="utf-8")
        except Exception:  # noqa: BLE001
            pass
        return {"current": cur, "diff": diff,
                "changed_outside": bool(diff) and bool(self.meta.get("params"))}

    # ---------------- 三层记录 ----------------
    def event(self, kind: str, text: str = "", joint: str = "", **fields):
        rec = {"mono": round(time.monotonic() - self._t0_mono, 4),
               "wall": _iso(), "kind": kind, "joint": joint, "text": text}
        rec.update({k: v for k, v in fields.items() if v is not None})
        line = json.dumps(rec, ensure_ascii=False, default=str)
        self._ring_event.append(line)
        self._push(("event", line))
        if self._incident_post is not None:
            self._incident_post["events"].append(line)
            if (time.monotonic() - self._incident_post["t0"]) >= INCIDENT_POST_S:
                self._finalize_incident()

    def state(self, states: dict, round_no: int | None = None,
              phase: str = "", step: str = "", flow: str = ""):
        """一轮回读写一行（每颗一行）。states: name -> JointState。"""
        self._round = (self._round + 1) if round_no is None else int(round_no)
        mono = round(time.monotonic() - self._t0_mono, 4)
        wall = _iso()
        lines = []
        for name, st in (states or {}).items():
            vals = [
                mono, wall, self._round, name,
                "" if getattr(st, "target_deg", None) is None else f"{st.target_deg:.4f}",
                getattr(st, "target_written_at_ms", ""),
                "" if getattr(st, "out_deg", None) is None else f"{st.out_deg:.4f}",
                "" if getattr(st, "velocity_deg_s", None) is None else f"{st.velocity_deg_s:.3f}",
                "" if getattr(st, "current_a", None) is None else f"{st.current_a:.4f}",
                getattr(st, "rho", ""),
                "" if getattr(st, "out_deg", None) is None else 1,
                "" if getattr(st, "mode", None) is None else st.mode,
                getattr(st, "mode_age_s", ""),
                "" if getattr(st, "enabled", None) is None else int(bool(st.enabled)),
                "" if getattr(st, "error", None) is None else f"0x{int(st.error):04X}",
            ]
            lines.append(",".join(str(v) for v in vals))
        if flow or phase:
            self.event("state_mark", f"{flow}.{step}".strip("."),
                       phase=phase, step=step, flow=flow)
        for ln in lines:
            self._ring_state.append(ln)
            self._push(("state", ln))
            if self._incident_post is not None:
                self._incident_post["state"].append(ln)
        if self._incident_post is not None and \
                (time.monotonic() - self._incident_post["t0"]) >= INCIDENT_POST_S:
            self._finalize_incident()

    def _push(self, item):
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self._drop += 1

    def _writer(self):
        while not self._stop.is_set() or not self._q.empty():
            try:
                kind, line = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if getattr(self, "_fh_closed", False):
                    continue
                if kind == "state":
                    self._state_fh.write(line + "\n")
                    if self._q.qsize() < 8:
                        self._state_fh.flush()
                else:
                    self._events_fh.write(line + "\n")
                    if self._q.qsize() < 8:
                        self._events_fh.flush()
            except Exception:  # noqa: BLE001
                pass

    # ---------------- 29.2 事故快照 ----------------
    def incident(self, reason: str, phase: str = "", step: str = "",
                 detail: dict | None = None, joints: dict | None = None,
                 trigger_ts: float | None = None) -> Path:
        self._incident_n += 1
        d = self.incidents_dir / f"{self._incident_n:03d}"
        d.mkdir(parents=True, exist_ok=True)
        now_mono = time.monotonic()
        pre_state = list(self._ring_state)
        pre_event = list(self._ring_event)
        (d / "state_pre.csv").write_text(
            STATE_HEADER + "\n" + "\n".join(pre_state) + "\n", encoding="utf-8")
        (d / "events_pre.jsonl").write_text("\n".join(pre_event) + "\n",
                                            encoding="utf-8")
        trig = {"mono": round(now_mono - self._t0_mono, 4),
                "wall": _iso(trigger_ts), "reason": reason, "phase": phase,
                "step": step, "detail": detail or {}, "joints": joints or {}}
        (d / "trigger.json").write_text(
            json.dumps(trig, ensure_ascii=False, indent=1), encoding="utf-8")
        self._incident_post = {"dir": d, "t0": now_mono, "trig": trig,
                               "state": [], "events": [],
                               "immediate": True}
        self._write_summary(d, trig, pre_state, pre_event)
        self.event("incident", reason, phase=phase, step=step, snapshot=str(d))
        if self._capture_pid:
            try:
                from .capture import slice_ring
                slice_ring(d, trigger_ts or time.time(),
                           before_s=INCIDENT_SUMMARY_S, after_s=INCIDENT_SUMMARY_S)
            except Exception:  # noqa: BLE001
                pass
        return d

    def _finalize_incident(self):
        post = self._incident_post
        if not post or not post.get("state") and not post.get("events"):
            self._incident_post = None
            return
        d = post["dir"]
        with open(d / "state_post.csv", "a", encoding="utf-8") as fh:
            fh.write("\n".join(post["state"]) + "\n")
        with open(d / "events_post.jsonl", "a", encoding="utf-8") as fh:
            fh.write("\n".join(post["events"]) + "\n")
        # 用“前 10 s / 后 10 s”窗口重写 summary.md，仍保证 ≤3 KB
        self._write_summary(d, post["trig"], post["state"], post["events"],
                            final=True)
        self._incident_post = None

    def _write_summary(self, d: Path, trig: dict, state_lines: list[str],
                       event_lines: list[str], final: bool = False):
        reason = trig.get("reason") or ""
        phase = trig.get("phase") or ""
        step = trig.get("step") or ""
        mono = trig.get("mono")
        joints = trig.get("joints") or {}
        detail = trig.get("detail") or {}
        # 事件时间线：触发前后 10 s
        ev = []
        for ln in event_lines:
            try:
                r = json.loads(ln)
            except Exception:  # noqa: BLE001
                continue
            if mono is not None and abs(float(r.get("mono", 0)) - float(mono)) > 10.0:
                continue
            ev.append(r)
        ev = ev[-15:]
        # 每颗在这一刻附近的值：取离触发时刻最近的一行
        per_joint: dict[str, list[str]] = {}
        best: dict[str, tuple[float, list[str]]] = {}
        for ln in state_lines:
            parts = ln.split(",")
            if len(parts) < 15:
                continue
            try:
                m = float(parts[0])
            except ValueError:
                continue
            name = parts[3]
            dist = abs(m - float(mono)) if mono is not None else 0.0
            if name not in best or dist < best[name][0]:
                best[name] = (dist, parts)
        for name, (_dist, parts) in best.items():
            per_joint[name] = parts
        lines = [
            f"# 事故快照 {d.name}",
            "",
            f"- 触发：{reason}",
            f"- 时刻：{trig.get('wall')}（会话内 mono={mono}s）",
            f"- 流程/步号：{phase or '—'}{('.' + step) if step else ''}",
        ]
        if not final:
            lines.append("- 状态：触发瞬间生成；后 30 s 的记录还在写入")
        if detail:
            kv = "，".join(f"{k}={v}" for k, v in detail.items())
            lines.append(f"- 关键量：{kv}")
        lines += ["", "## 每颗（触发时刻最近一行）", "```"]
        if not per_joint:
            lines.append("（触发前后没有状态行：这一段只有事件）")
        for name in sorted(per_joint):
            p = per_joint[name]
            lines.append(
                f"{name}: 目标 {p[4] or '—'} 实际 {p[6] or '—'} 差 "
                f"{_diff(p[4], p[6])} 速度 {p[7] or '—'} 电流 {p[8] or '—'} "
                f"ρ {p[9] or '—'} 模式 {p[11] or '—'} 使能 {p[13] or '—'} "
                f"报警 {p[14] or '—'}")
        lines += ["```", "", "## 事件时间线（触发前后 10 s）", "```"]
        for r in ev:
            lines.append(f"{r.get('mono')}s {r.get('kind')}"
                         + (f" {r.get('joint')}" if r.get("joint") else "")
                         + (f" {r.get('text')}" if r.get("text") else ""))
        lines += ["```", "", "## 抓包对账", "```",
                  f"- 抓包：{'有（capture_pid 见 meta.json）' if self._capture_pid else '没抓包'}"
                  f"；本段事件 {len(ev)} 条", "```"]
        text = "\n".join(lines) + "\n"
        if len(text.encode("utf-8")) > 3 * 1024:
            # 先减少事件时间线的行数（段落都保留），再不行才硬截
            for keep_n in (10, 6, 3, 1):
                ev2 = ev[-keep_n:]
                lines2 = lines[:lines.index("## 事件时间线（触发前后 10 s）") + 2]
                for r in ev2:
                    lines2.append(f"{r.get('mono')}s {r.get('kind')}"
                                  + (f" {r.get('joint')}" if r.get("joint") else "")
                                  + (f" {r.get('text')}" if r.get("text") else ""))
                tail_from = lines.index("## 抓包对账")
                lines2 += ["```"] + lines[tail_from:]
                text2 = "\n".join(lines2) + "\n"
                text = text2
                if len(text.encode("utf-8")) <= 3 * 1024:
                    break
            if len(text.encode("utf-8")) > 3 * 1024:
                text = text.encode("utf-8")[:3 * 1024 - 20].decode(
                    "utf-8", errors="ignore") + "\n（截断）\n"
        (d / "summary.md").write_text(text, encoding="utf-8")

    # ---------------- 磁盘保留 ----------------
    def enforce_quota(self, max_bytes: int = MAX_RUNS_BYTES) -> dict:
        """29.7 第 4 条：记录目录超过上限就删最旧的会话（快照目录不删）。

        37.3：同一次执行器进程里 10 分钟内只真扫一次（避免每次 connect 都扫整个 runs/）；
        超限时仍然会真删，删完把结果缓存。
        """
        key = str(self.root.resolve())
        now = time.monotonic()
        hit = _QUOTA_CACHE.get(key)
        if hit and now - hit[0] < _QUOTA_MIN_INTERVAL_S:
            return {**hit[1], "cached": True}
        total = runs_bytes(self.root)
        removed = []
        if total <= max_bytes:
            res = {"ok": True, "bytes": total, "removed": []}
            _QUOTA_CACHE[key] = (now, res)
            return res
        sessions = sorted([p for p in self.root.iterdir()
                           if p.is_dir() and p.name != "_pcap_ring"
                           and not p.name.startswith("_")],
                          key=lambda p: p.stat().st_mtime)
        for p in sessions:
            if p.resolve() == self.dir.resolve():
                break
            with_incidents = (p / "incidents")
            if with_incidents.exists() and any(with_incidents.iterdir()):
                continue        # 快照目录不删，等 PC 说删
            try:
                size = runs_bytes(p)
                shutil.rmtree(p)
                total -= size
                removed.append(p.name)
            except Exception:  # noqa: BLE001
                continue
            if total <= max_bytes:
                break
        res = {"ok": total <= max_bytes, "bytes": total, "removed": removed}
        _QUOTA_CACHE[key] = (now, res)
        return res

    def close(self):
        try:
            if self._incident_post is not None:
                self._finalize_incident()
        except Exception:  # noqa: BLE001
            pass
        self._stop.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:  # noqa: BLE001
            pass
        self._fh_closed = True
        for fh in (self._state_fh, self._events_fh):
            try:
                fh.flush()
                fh.close()
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            self.meta["ended"] = _iso()
            self.meta["dropped_lines"] = self._drop
            self._write_meta()


def _diff(a: str, b: str) -> str:
    try:
        return f"{float(b) - float(a):+.2f}"
    except (TypeError, ValueError):
        return "—"
