#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A5 自检（mock）：加上 A5 的记录（状态 5 Hz + 每条写命令一行）以后，
回读节拍掉了多少。

做法：mock 里跑同一段运动，先关掉状态记录、再打开，各取 6 s 的
`/api/state` 里 read_tick_hz 的平均值。真机节拍等 B2 再量。

用法：
    cd arm-control && python3 tools/readback_tick_check.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402


def sample_tick(ex, seconds=6.0):
    vals = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.2)
        v = ex.state().get("read_tick_hz")
        if v:
            vals.append(float(v))
    return (sum(vals) / len(vals)) if vals else 0.0


def run_phase(status_on: bool):
    ex = Executor()
    ex.connect("mock")
    ex.motion.speed_scale = 1.0
    ex._status_period_s = 0.2 if status_on else 1e9
    # 一段持续运动，让回读节拍有负载
    ex.move_joints([0.0, 20.0, 20.0, 10.0, 0.0, 0.0])
    time.sleep(0.3)
    hz = sample_tick(ex)
    ex.motion.stop("测节拍")
    ex.disconnect()
    return hz


def main():
    off = run_phase(False)
    on = run_phase(True)
    drop = 0.0 if off <= 0 else (off - on) / off * 100.0
    print(f"A5 回读节拍（mock）：不加状态记录 {off:.1f} Hz；"
          f"加 5 Hz 状态记录 {on:.1f} Hz；掉了 {drop:.1f}%")
    print("（真机节拍等 B2：read_all 500 次 + set_targets_q 耗时见 A8 只读工具）")
    print("结果：过（只要没把节拍压到 15 Hz 以下）" if on >= 15.0 else "结果：没过")
    return 0 if on >= 15.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
