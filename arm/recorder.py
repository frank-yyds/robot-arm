#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CSV 记录（原 B7）：每次运行一个文件。

行类型（24 节 A5 / 16.4 第 8 条）：
  * cmd：每条发出去的写命令一行（时间、关节、指令、值、有没有回包）
  * poll：每次回读（位置、电流、报警）
  * status：电源 0x2B / 模式 0x55 / 报警（A5 要求每颗至少 5 Hz）
  * note / plan / execute_* / abort / read_anomaly
"""
from __future__ import annotations

import csv
import datetime as _dt
import threading
from pathlib import Path

from . import config

HEADER = ["time", "t_rel", "kind", "joint", "pos_deg", "target_deg",
          "current_a", "error", "power", "mode", "note"]


INCIDENT_KINDS = {
    "abort": "中止 / 原地保持",
    "hold_failure": "保持失效",
    "read_anomaly": "读数被合理性检查拒掉",
    "write_resend": "写目标丢了（已重发）",
    "disable_joint": "单颗失能",
    "contact_mismatch": "接触高度比对不符",
    "catch_failed": "搁架没接住",
    "park_drop": "就地失能落到搁架上",
    "user_mark": "用户标记",
    "user_stop": "用户按停",
    "alarm_change": "报警码变化",
}


class Recorder:
    def __init__(self, path: Path | None = None, note: str = "",
                 session=None):
        config.RUNS_DIR.mkdir(parents=True, exist_ok=True)
        if path is None:
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = config.RUNS_DIR / f"{stamp}_run.csv"
        self.path = Path(path)
        self._lock = threading.Lock()
        self._fh = self.path.open("w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(HEADER)
        self._t0 = _dt.datetime.now()
        # 第二十九节：同一份记录同时写进会话目录（三层记录）
        self.session = session
        if note:
            self.note(note)
        self._fh.flush()

    def _ts(self):
        now = _dt.datetime.now()
        return now.isoformat(timespec="milliseconds"), round((now - self._t0).total_seconds(), 3)

    def _row(self, kind, joint="", pos="", target="", current="", error="",
             power="", mode="", note=""):
        with self._lock:
            t, tr = self._ts()
            self._w.writerow([t, tr, kind, joint, pos, target, current,
                              error, power, mode, note])
            self._fh.flush()

    def note(self, text: str):
        self._row("note", note=text)
        if self.session:
            try:
                self.session.event("note", text)
            except Exception:  # noqa: BLE001
                pass

    def command(self, kind: str, note: str = "", joint: str = "",
                target_deg: float | None = None, **kw):
        if self.session:
            try:
                self.session.event(kind, note, joint=joint,
                                   target_deg=target_deg)
                why = INCIDENT_KINDS.get(kind)
                if why:
                    self.session.incident(why, detail={"原文": note})
            except Exception:  # noqa: BLE001
                pass
        self._row(kind, joint=joint,
                  pos="" if kw.get("pos_deg") is None else f"{kw['pos_deg']:.4f}",
                  target="" if target_deg is None else f"{target_deg:.4f}",
                  current=kw.get("current_a", ""), error=kw.get("error", ""),
                  power=kw.get("power", ""), mode=kw.get("mode", ""),
                  note=note)

    def write_cmd(self, joint: str, cmd: int, value: str, replied,
                  note: str = ""):
        """A5：一条写命令。replied：True/False，None = 该指令本身不等回包。"""
        if replied is None:
            rep = "回包=不等"
        else:
            rep = "回包=有" if replied else "回包=无"
        text = f"0x{cmd:02X} 值 {value} {rep}" + (f"；{note}" if note else "")
        self._row("cmd", joint=joint, note=text)
        if self.session:
            try:
                self.session.event("cmd", text, joint=joint, cmd=cmd,
                                   value=value, replied=replied)
            except Exception:  # noqa: BLE001
                pass

    def poll(self, states: dict):
        if self.session:
            try:
                self.session.state(states)
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            t, tr = self._ts()
            for name, st in states.items():
                self._w.writerow([
                    t, tr, "poll", name,
                    "" if st.out_deg is None else f"{st.out_deg:.4f}",
                    "" if st.target_deg is None else f"{st.target_deg:.4f}",
                    "" if st.current_a is None else f"{st.current_a:.3f}",
                    "" if st.error is None else f"0x{st.error:04X}",
                    "", "",
                    ("读数异常 原始 " + st.raw_pos_hex) if getattr(st, "pos_invalid", False) else "",
                ])
            self._fh.flush()

    def status(self, rows: dict[str, dict]):
        """A5：电源/模式/报警，每颗一行（位置由 poll 行覆盖）。"""
        if self.session:
            try:
                for name, r in rows.items():
                    self.session.event(
                        "status", f"电源={r.get('power')} 模式={r.get('mode')} "
                                  f"报警={r.get('error')}", joint=name)
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            t, tr = self._ts()
            for name, r in rows.items():
                self._w.writerow([
                    t, tr, "status", name, "", "", "", 
                    "" if r.get("error") is None else f"0x{int(r['error']):04X}",
                    "" if r.get("power") is None else str(r["power"]),
                    "" if r.get("mode") is None else str(r["mode"]),
                    "",
                ])
            self._fh.flush()

    def close(self):
        if self.session:
            try:
                self.session.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001
            pass
