#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""37.2 第 1 条：按动作的起止时刻，找"覆盖它的" trace 文件（不按最新文件拿）。

背景：36.3 c 组第一次跑那 6 段，分析脚本当时按"目录里最新的 trace 文件"取，
拿成了别的动作的 trace，段全部错位。正确做法：每个动作（execute_start ..
execute_done / abort）用自己的起止墙上时间去 trace 索引里找覆盖它的那一条。

"覆盖"的判据（trace 是 0x94 高频记录，不一定记到 execute_done 以后）：
  * trace 的起点不超过动作起点 + START_TOL_S，也不晚于动作结束；
  * trace 的终点至少到动作结束前 END_TOL_S（高频记录常比 execute_done 早
    一点结束：execute_done 是到位判定+记录落盘的时刻，trace 窗口可能 3 s
    而动作登记时长 2.9 s）；
  * 候选里取"起点离动作起点最近"的那一条；再用 trace 里确实出现本段的
    目标值（target_deg 变化）核对一遍。

trace 文件的 time 列是整秒（strftime），t_rel 是相对采样起点的高精度秒数，
所以这里用 first_time + t_rel 估算覆盖区间，只用于"选文件"，不用于精确对时。

只读；不碰真机、不写任何记录。
"""
from __future__ import annotations

import csv
import datetime as _dt
from pathlib import Path

START_TOL_S = 1.0     # trace 最多比动作起点晚 1 s 开始
END_TOL_S = 1.5       # trace 至少覆盖到动作结束前 1.5 s


class TraceFile:
    """一个 trace 文件。行数据按需加载（先只看首尾两行建索引，选文件快）。"""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.rows: dict[str, list[dict]] | None = None
        self.first_time: _dt.datetime | None = None
        self.last_time: _dt.datetime | None = None
        self.t_rel_first: float | None = None
        self.t_rel_last: float | None = None
        self.dt_median: float | None = None
        self._head_lines: list[str] = []
        self._load_head_tail()

    def _load_head_tail(self):
        with self.path.open(encoding="utf-8", errors="replace") as f:
            f.readline()  # 表头
            for _ in range(3):
                ln = f.readline()
                if not ln:
                    break
                self._head_lines.append(ln)
        # 尾行：从文件末尾找最后一个非空行
        size = self.path.stat().st_size
        with self.path.open("rb") as f:
            back = min(size, 65536)
            f.seek(size - back)
            chunk = f.read().decode("utf-8", "replace")
        lines = [ln for ln in chunk.splitlines() if ln.strip()]
        head_ok = bool(self._head_lines)
        tail_ok = bool(lines)
        if head_ok:
            r = _csv_head(self._head_lines[0])
            self.first_time = _parse_time(r[0] if r else "")
            self.t_rel_first = _parse_f(r[1] if len(r) > 1 else "")
        if tail_ok:
            r = _csv_head(lines[-1])
            self.t_rel_last = _parse_f(r[1] if len(r) > 1 else "")
        if self.first_time is not None and self.t_rel_last is not None:
            self.last_time = self.first_time + _dt.timedelta(seconds=self.t_rel_last)

    def _ensure_rows(self):
        if self.rows is not None:
            return
        self.rows = {}
        dt_list = []
        with self.path.open(encoding="utf-8") as f:
            rd = csv.DictReader(f)
            for r in rd:
                try:
                    tr = float(r["t_rel"])
                except (TypeError, ValueError):
                    continue
                self.t_rel_last = tr
                try:
                    dt_list.append(float(r["dt_s"]))
                except (TypeError, ValueError, KeyError):
                    pass
                row = {
                    "t": tr,
                    "pos": _f(r.get("pos_deg")),
                    "target": _f(r.get("target_deg")),
                    "vel": _f(r.get("velocity_deg_s")),
                }
                self.rows.setdefault(r.get("joint") or "?", []).append(row)
        dts = sorted(x for x in dt_list if x and x > 0)
        if dts:
            self.dt_median = dts[len(dts) // 2]

    # ---- 覆盖区间（估算） ----
    def start_est(self) -> _dt.datetime | None:
        """trace 起点估算：整秒时间 + 第一个 t_rel（误差 <1 s，足够选文件）。"""
        if self.first_time is None:
            return None
        return self.first_time + _dt.timedelta(seconds=self.t_rel_first or 0.0)

    def end_est(self) -> _dt.datetime | None:
        if self.first_time is None or self.t_rel_last is None:
            return None
        return self.first_time + _dt.timedelta(seconds=self.t_rel_last)

    def joints(self):
        self._ensure_rows()
        return sorted(self.rows or {})

    def samples(self, joint: str):
        self._ensure_rows()
        return (self.rows or {}).get(str(joint).upper(), [])

    def hz(self) -> float | None:
        self._ensure_rows()
        if not self.dt_median or self.dt_median <= 0:
            return None
        return round(1.0 / self.dt_median, 1)

    def switch_t(self, joint: str, target: float) -> float | None:
        """trace 里第一次出现这个目标值的 t_rel（写目标后的第一次采样）。"""
        rs = self.samples(joint)
        prev = None
        for r in rs:
            if prev is not None and r["target"] is not None \
                    and abs(r["target"] - float(target)) <= 1e-6:
                return r["t"]
            prev = r["target"]
        if rs and rs[0]["target"] is not None and abs(rs[0]["target"] - float(target)) <= 1e-6:
            return rs[0]["t"]
        return None


def _csv_head(line: str) -> list[str]:
    """只切第一行 CSV（够拿 time/t_rel 两列；字段里没有逗号）。"""
    return line.rstrip("\n").split(",")


def _parse_time(s: str) -> _dt.datetime | None:
    try:
        return _dt.datetime.fromisoformat(s)
    except (ValueError, TypeError):
        return None


def _parse_f(s: str) -> float | None:
    try:
        return float(s)
    except (ValueError, TypeError):
        return None


def _f(v):
    if v in (None, ""):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def build_index(patterns, base: Path | None = None) -> list[TraceFile]:
    """按 glob 模式建 trace 索引（去重，按起点时间排序）。"""
    base = Path(base or ".")
    files: list[Path] = []
    for pat in patterns:
        files.extend(base.glob(pat))
    uniq = sorted({p.resolve() for p in files})
    idx = [TraceFile(p) for p in uniq]
    idx = [t for t in idx if t.first_time is not None]
    idx.sort(key=lambda t: t.start_est() or _dt.datetime.min)
    return idx


def find_trace_covering(index: list[TraceFile],
                        t0: _dt.datetime, t1: _dt.datetime,
                        targets: dict[str, float] | None = None,
                        start_tol_s: float = START_TOL_S,
                        end_tol_s: float = END_TOL_S) -> TraceFile | None:
    """按动作 [t0, t1] 找覆盖它的 trace；targets 给了就核对目标值出现过。"""
    cands = []
    for tr in index:
        s, e = tr.start_est(), tr.end_est()
        if s is None or e is None:
            continue
        if s > t0 + _dt.timedelta(seconds=start_tol_s) or s > t1:
            continue
        if e < t1 - _dt.timedelta(seconds=end_tol_s) or e < t0:
            continue
        ds = abs((s - t0).total_seconds())
        cands.append((ds, -(e - t0).total_seconds(), tr))
    if not cands:
        return None
    cands.sort(key=lambda x: (x[0], x[1]))
    if not targets:
        return cands[0][2]
    for _ds, _cov, tr in cands:
        need = [(j, float(v)) for j, v in targets.items()]
        if any(tr.switch_t(j, v) is not None for j, v in need):
            return tr
    return None


def describe_match(tr: TraceFile, t0: _dt.datetime, t1: _dt.datetime) -> dict:
    s, e = tr.start_est(), tr.end_est()
    return {
        "trace": tr.path.name,
        "trace_start_est": s.isoformat() if s else None,
        "trace_end_est": e.isoformat() if e else None,
        "start_delta_s": None if s is None else round((s - t0).total_seconds(), 3),
        "end_delta_s": None if e is None else round((e - t1).total_seconds(), 3),
        "hz": tr.hz(),
    }
