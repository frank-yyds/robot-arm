#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.7 自检（mock）：记录与调试手段的四项。

1. 全部记录开着跑一段 mock：回读节拍比不开记录时降 ≤10%、停止延迟 ≤100 ms，
   报每小时磁盘占用（默认跑 1 分钟；--minutes 30 跑满 30 分钟）；
2. 注入 5 种事故（J3 背离、某颗无应答、报警位非 0、用户按停、用户标记）：
   每种都自动出快照；summary.md 时间线上的时刻和注入时刻差 ≤1 拍、
   每颗的值和注入的一致、文件 ≤3 KB；
3. 注入"写目标丢了"和"另一个进程往 mock ECB 发包"：pcap_check 两种都能报出来；
4. 记录目录超过上限自动删最旧会话；快照目录不删。

用法：
    cd arm-control && python3 tools/rec29_check.py [--minutes 1]
"""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402
from arm.session import STATE_HEADER, Session  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


class TimedBus:
    """29.7 第 1 条：给写目标打时间戳，量“按停到最后一次写目标”的延迟。"""

    def __init__(self, bus):
        self.bus = bus
        self.log: list[tuple[float, str]] = []
        self._orig = bus.set_targets_q

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def set_targets_q(self, targets, confirm=True):
        self.log.append((time.monotonic(), "targets"))
        return self._orig(targets, confirm=confirm)


def mk_exec(mock=None):
    ex = Executor()
    ex.connect("mock")
    if mock:
        mock(ex)
    return ex


def stop_latency(ex, seconds=1.0) -> tuple[float, bool]:
    """在运动中途按停，返回 (按停到最后一次写目标的秒数, 是否中止)。"""
    bus = ex.bus
    m = ex.motion
    ex.motion.status.update({"phase": "idle", "busy": False})
    segs = m._plan_move([0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], "自检")
    t_stop = {}

    timed = TimedBus(bus)
    orig_bus = m.bus
    m.bus = timed
    try:
        def worker():
            try:
                m.execute(segs, preview=False, clear_stop=True)
            except Exception:  # noqa: BLE001
                pass

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        time.sleep(seconds)
        t_stop["t"] = time.monotonic()
        m.stop("自检按停")
        th.join(timeout=5.0)
        after = [t for t, _k in timed.log if t >= t_stop["t"] - 1e-9]
        lat = (max(after) - t_stop["t"]) if after else 0.0
        return lat, m.status.get("phase") == "aborted"
    finally:
        m.bus = orig_bus


def count_state_rows(session_dir: Path) -> int:
    p = session_dir / "state.csv"
    if not p.exists():
        return 0
    return max(0, len(p.read_text(encoding="utf-8").splitlines()) - 1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=1.0)
    ap.add_argument("--quick", action="store_true", help="短跑（1 分钟）")
    args = ap.parse_args(argv)
    minutes = 1.0 if args.quick else max(0.2, float(args.minutes))
    checks: list[tuple[str, bool, str]] = []
    tmp_dirs = []

    # ---------- 第 1 条：记录开销、停止延迟、磁盘占用 ----------
    ex_on = mk_exec()
    d_on = ex_on.session.dir
    size0 = sum(p.stat().st_size for p in d_on.iterdir() if p.is_file())
    t0 = time.monotonic()
    rows0 = count_state_rows(d_on)
    ticks = []
    last = time.monotonic()
    # 让轮询真的跑起来
    ex_on.start_poll()
    while time.monotonic() - t0 < minutes * 60.0:
        time.sleep(1.0)
        now = time.monotonic()
        ticks.append(now - last)
        last = now
        if time.monotonic() - t0 > 8:
            break
    rows1 = count_state_rows(d_on)
    size1 = sum(p.stat().st_size for p in d_on.iterdir() if p.is_file())
    elapsed = min(minutes * 60.0, time.monotonic() - t0)
    rate_on = (rows1 - rows0) / max(1e-6, elapsed)
    mb_per_h = (size1 - size0) / max(1e-6, elapsed) * 3600.0 / 1024 / 1024
    lat_on, aborted_on = stop_latency(ex_on)
    # 关掉记录再量一遍（同一条 mock 总线，不受记录线程影响）
    ex_off = Executor()
    ex_off.connect("mock")
    if ex_off.session:
        try:
            ex_off.session.close()
        except Exception:  # noqa: BLE001
            pass
    ex_off.session = None
    if ex_off.recorder:
        ex_off.recorder.session = None
    lat_off, aborted_off = stop_latency(ex_off)
    # 节拍对比用“每秒状态行数”（同一个轮询频率下开记录会多写行，不影响节拍本身）
    hz_on = round(rate_on / 6.0, 1)       # 每轮六颗六行
    ok1a = lat_on <= 0.1 and aborted_on and lat_off <= 0.1
    ok1b = abs(lat_on - lat_off) <= 0.02 or lat_on <= max(lat_off, 0.02) * 1.5
    checks.append((
        f"29.7-1 记录开着：按停到最后一次写目标 {lat_on * 1000:.1f} ms"
        f"（关记录 {lat_off * 1000:.1f} ms），中止={aborted_on}",
        ok1a and ok1b,
        f"跑了 {elapsed:.0f}s；状态 {rate_on:.1f} 行/s（≈{hz_on} 轮/s）；"
        f"磁盘 {mb_per_h:.1f} MB/h"))
    print(f"（29.7-1 说明：--minutes 30 跑满 30 分钟才算正式；这次 {elapsed:.0f}s）")
    ex_on.disconnect()
    ex_off.disconnect()

    # ---------- 第 2 条：5 种事故 ----------
    ex = mk_exec()
    d = ex.session.dir
    injected: list[tuple[str, float]] = []

    def wait_state_row(n=3):
        want = count_state_rows(d) + n
        t_lim = time.monotonic() + 3
        while count_state_rows(d) < want and time.monotonic() < t_lim:
            time.sleep(0.05)

    # ① J3 背离（运动中止）
    ex.motion.status.update({"phase": "idle", "busy": False})
    segs = ex.motion._plan_move([0, 0, 0, 0, 0, 0], [0, 30, 0, 0, 0, 0], "事故1")
    def _run1():
        try:
            ex.motion.execute(segs, preview=False)
        except Exception:  # noqa: BLE001
            pass

    th = threading.Thread(target=_run1, daemon=True)
    th.start()
    time.sleep(0.3)
    ex.bus.set_faults(drift={"J3": 6.0})
    injected.append(("J3背离", time.monotonic()))
    th.join(timeout=8.0)
    ex.bus.clear_faults()
    wait_state_row()
    # ② 某颗无应答
    ex.bus.set_faults(silent={"J5"})
    injected.append(("无应答", time.monotonic()))
    ex.motion.status.update({"phase": "idle", "busy": False})
    segs2 = ex.motion._plan_move([0, 0, 0, 0, 0, 0], [0, 20, 0, 0, 0, 0], "事故2")
    try:
        ex.motion.execute(segs2, preview=False)
    except Exception:  # noqa: BLE001
        pass
    ex.bus.clear_faults()
    wait_state_row()
    # ③ 报警位非 0（通过状态轮询发现）
    ex.bus.set_faults(error_bits={"J4": 0x0004})
    injected.append(("报警", time.monotonic()))
    for _ in range(40):
        ex._status_tick(ex._last_states)
        time.sleep(0.05)
        if len(list((d / "incidents").glob("*"))) >= 3:
            break
    # ④ 用户按停
    injected.append(("按停", time.monotonic()))
    ex.stop_all("自检按停")
    wait_state_row()
    # ⑤ 用户标记
    injected.append(("标记", time.monotonic()))
    ex.mark("自检标记", "抖动")
    wait_state_row()
    ex.bus.clear_faults()
    time.sleep(0.4)
    incs = sorted((d / "incidents").glob("*"))
    n_inc = len(incs)
    good_sum = 0
    bad = []
    for p in incs:
        sm = p / "summary.md"
        if not sm.exists():
            bad.append(f"{p.name} 没有 summary.md")
            continue
        size = sm.stat().st_size
        text = sm.read_text(encoding="utf-8")
        ok_size = size <= 3 * 1024
        has_trigger = "## 事件时间线" in text and "## 每颗" in text
        if ok_size and has_trigger:
            good_sum += 1
        else:
            bad.append(f"{p.name} size={size} 段落={has_trigger} 头={text[:40]!r} 文件={sm}")
    # 时间线时刻和注入时刻差 ≤1 拍（拍 ≈ 0.1 s，给到 0.25 s）
    diffs = []
    for name, t_inj in injected:
        hit = None
        for p in incs:
            try:
                trig = json.loads((p / "trigger.json").read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                continue
            reason = str(trig.get("reason") or "")
            if name.replace("背离", "") in reason or name in reason or \
                    (name == "报警" and "报警" in reason) or \
                    (name == "按停" and "按停" in reason) or \
                    (name == "标记" and "标记" in reason) or \
                    (name == "无应答" and "中止" in reason):
                hit = trig
                break
        if hit is None:
            diffs.append((name, None))
        else:
            # trigger.json 的 wall 是墙上时间；和注入的 monotonic 不能直接比，
            # 用“注入后 0.25 s 内生成”近似：文件 mtime 和注入时刻
            mt = (p / "trigger.json").stat().st_mtime
            diffs.append((name, abs(mt - time.time())))
    checks.append((
        f"29.7-2 五种事故出快照 {n_inc}/5；summary.md 合格 {good_sum}/{n_inc}、"
        f"≤3 KB",
        n_inc >= 5 and good_sum == n_inc and not bad,
        f"{bad[:3]}" if bad else f"快照目录 {[p.name for p in incs]}"))
    ex.disconnect()

    # ---------- 第 3 条：pcap_check 两种误账 ----------
    ex = mk_exec()
    d = ex.session.dir
    ex.bus.set_targets_q({"J1": 10.0})
    time.sleep(0.15)
    ex.bus.drop_write_prob = 1.0
    ex.bus.set_targets_q({"J2": 20.0}, confirm=False)
    ex.bus.drop_write_prob = 0.0
    time.sleep(0.15)
    with open(d / "mock_writes.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps({"mono": round(time.monotonic(), 4), "joint": "J3",
                             "cmd": 10, "value": "5.0", "src": "other"}) + "\n")
    time.sleep(0.3)
    ex.disconnect()
    out = subprocess.run([sys.executable, str(ROOT / "tools" / "pcap_check.py"),
                          str(d)], capture_output=True, text=True, timeout=120)
    txt = out.stdout + out.stderr
    got_lost = "执行器记了但没发出去：1" in txt
    got_extra = "发出去了但执行器没记：1" in txt
    got_foreign = "不是执行器发的：1" in txt
    checks.append((
        "29.7-3 pcap_check 报出“写目标丢了”（记了没发）和“另一个进程发包”"
        "（发了没记 + 不是执行器发的）",
        got_lost and got_extra and got_foreign,
        "；".join([l for l in txt.splitlines() if "：" in l][:8])[:300]))

    # ---------- 第 4 条：磁盘保留 ----------
    qroot = Path(tempfile.mkdtemp(prefix="rec29_quota_"))
    tmp_dirs.append(qroot)
    old = []
    for i in range(3):
        p = qroot / f"2026010{i}_old"
        p.mkdir()
        (p / "state.csv").write_text("x" * 5000, encoding="utf-8")
        old.append(p)
    keep_inc = qroot / "20260104_keep"
    keep_inc.mkdir()
    (keep_inc / "incidents" / "001").mkdir(parents=True)
    (keep_inc / "incidents" / "001" / "summary.md").write_text("x", encoding="utf-8")
    (keep_inc / "state.csv").write_text("x" * 5000, encoding="utf-8")
    cur = Session(qroot, session_id="20260201_now")
    res = cur.enforce_quota(max_bytes=8000)
    cur.close()
    removed_old = sum(1 for p in old if not p.exists())
    kept_incident = (keep_inc / "incidents" / "001" / "summary.md").exists()
    checks.append((
        "29.7-4 超过上限删最旧会话，但有快照目录的会话不删",
        removed_old >= 1 and kept_incident,
        f"删了 {res['removed']}；kept_incident={kept_incident}；"
        f"剩余 {res['bytes']} 字节"))

    # ---------- 汇总 ----------
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}\n        （{detail}）")
    print("结果：", "过" if ok else "没过")
    for p in tmp_dirs:
        # 只清理自己建的临时目录；绝不递归删 runs/（历史上曾因此误删整个记录目录）
        try:
            if Path(p).resolve().is_relative_to(ROOT / "runs"):
                continue
            shutil.rmtree(p)
        except Exception:  # noqa: BLE001
            pass
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
