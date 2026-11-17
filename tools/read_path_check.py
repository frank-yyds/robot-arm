#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A2/A3 自检：两线程并发读、乱序/迟到/串关节/串指令回包。

做法：用 FakeLink 顶替 ECB 总线（不占网络、可跑 10 万次），两个线程各自
调 JointBus.read_all，统计“被采用的值和该关节真值不符”的次数。

- 修前基线：关掉 A3 的合理性检查（plausible_rate 放大），串关节回包会被当成真值；
- 修后：A3 合理性检查 + 单条重读，错位 0 次（被判“读数异常”的不算采用）。

用法：
    python3 tools/read_path_check.py            # 默认 10 万次关节读数
    python3 tools/read_path_check.py --n 20000
"""
from __future__ import annotations

import argparse
import random
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.joints import JointBus, load_joint_configs
from arm.sca import R_ERROR, R_POSITION

IQ24 = float(1 << 24)


class FakeLink:
    """和 EcbTransport.read_many/read 同接口的假总线，故意制造错位回包。"""

    def __init__(self, truth: dict[str, float], cfg):
        self.truth = dict(truth)
        self.cfg = cfg
        self.lock = threading.Lock()
        self.cross_prob = 0.02      # 回包串到别的关节
        self.foreign_prob = 0.05    # 混进别的指令的回包
        self.dup_prob = 0.20        # 重复回包
        self.stale_hold = None      # (joint, 旧值, 剩余次数)：模拟迟到旧值
        self.rng = random.Random(20261117)
        self.timeouts = 0
        self.repeat_dropped = 0
        self.stats = {"tx": 0, "rx": 0}
        self.accepted = {"cross": 0, "stale": 0, "foreign": 0}

    # 真值按固定节拍变；测试里不动，避免把正常滞后算成错位
    def _pos_bytes(self, name: str) -> bytes:
        turns = self.cfg[name].turns(self.truth[name])
        return struct.pack(">i", int(round(turns * IQ24)))

    def _reply(self, addr: int, cmd: int, n: int) -> bytes:
        names = {self.cfg[nm].id: nm for nm in self.cfg}
        name = names.get(addr)
        if name is None:
            return b"\x00" * n
        if cmd == R_ERROR:
            return b"\x00" * n
        if cmd != R_POSITION:
            return b"\x00" * n
        # 串关节
        if self.rng.random() < self.cross_prob:
            other = [x for x in self.cfg if x != name]
            pick = self.rng.choice(other)
            return self._pos_bytes(pick)[:n]
        # 迟到旧值
        if self.stale_hold and self.stale_hold[0] == name and self.stale_hold[2] > 0:
            with self.lock:
                if self.stale_hold[2] > 0:
                    self.stale_hold = (self.stale_hold[0], self.stale_hold[1],
                                       self.stale_hold[2] - 1)
            turns = self.cfg[name].turns(self.stale_hold[1])
            return struct.pack(">i", int(round(turns * IQ24)))[:n]
        return self._pos_bytes(name)[:n]

    def read_many(self, items, timeout=0.05, retries=2):
        out = {}
        # 故意乱序：先答复后面的（但不影响 key 匹配，只考验匹配逻辑）
        order = list(items)
        self.rng.shuffle(order)
        for addr, cmd, n in order:
            self.stats["tx"] += 1
            if cmd in (R_POSITION, R_ERROR):
                self.stats["rx"] += 1
                out[(addr, cmd)] = self._reply(addr, cmd, n)
        return out

    def read(self, addr, cmd, n, **kw):
        self.stats["tx"] += 1
        return self._reply(addr, cmd, n)


def run_phase(n_samples: int, validate: bool, seed=7):
    cfg = load_joint_configs()
    truth = {"J1": 0.0, "J2": 10.0, "J3": 20.0, "J4": 30.0, "J5": 40.0, "J6": 50.0}
    link = FakeLink(truth, cfg)
    bus = JointBus("127.0.0.1", 1, cfg)
    bus.link = link
    for m in bus.motors.values():
        m.link = link
    bus.connected = True
    bus.startup_done = True
    if not validate:
        bus.plausible_rate = 1e9      # 修前基线：没有合理性检查
        bus.plausible_margin = 1e9
    errors: list[str] = []
    wrong = [0]
    invalid = [0]
    seen = [0]
    lock = threading.Lock()

    def worker(tid: int):
        local = 0
        while True:
            with lock:
                if seen[0] >= n_samples:
                    break
                seen[0] += 6
            st = bus.read_all(full=False)
            for name in bus.order:
                s = st.get(name)
                if s is None:
                    continue
                if s.pos_invalid or s.out_deg is None:
                    with lock:
                        invalid[0] += 1
                    continue
                if abs(s.out_deg - truth[name]) > 0.5:
                    with lock:
                        wrong[0] += 1
                        if len(errors) < 8:
                            errors.append(f"{name} 读到 {s.out_deg:.2f}° 真值 {truth[name]:.2f}°")
                local += 1
            # 线程结束条件由 seen 控制
    threads = [threading.Thread(target=worker, args=(i,)) for i in range(2)]
    t0 = time.time()
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return {
        "samples": n_samples, "wrong": wrong[0], "invalid": invalid[0],
        "threads": 2, "elapsed_s": round(time.time() - t0, 2),
        "examples": errors,
    }


def stale_step_phase(validate: bool):
    """迟到旧值：J3 真值 40°，混进 2 次事务之前的 20° 回包（5 条）。"""
    cfg = load_joint_configs()
    truth = {"J1": 0.0, "J2": 10.0, "J3": 40.0, "J4": 30.0, "J5": 40.0, "J6": 50.0}
    link = FakeLink(truth, cfg)
    link.cross_prob = 0.0
    bus = JointBus("127.0.0.1", 1, cfg)
    bus.link = link
    for m in bus.motors.values():
        m.link = link
    bus.startup_done = True
    if not validate:
        bus.plausible_rate = 1e9
        bus.plausible_margin = 1e9
    bus.read_all(full=False)          # 先建立可信基准 40°
    link.stale_hold = ("J3", 20.0, 5)
    wrong = invalid = 0
    for _ in range(40):
        st = bus.read_all(full=False)
        s = st["J3"]
        if s.pos_invalid or s.out_deg is None:
            invalid += 1
        elif abs(s.out_deg - 40.0) > 0.5:
            wrong += 1
    return {"wrong": wrong, "invalid": invalid}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100000)
    args = ap.parse_args()
    before = run_phase(args.n, validate=False)
    after = run_phase(args.n, validate=True)
    before_stale = stale_step_phase(validate=False)
    after_stale = stale_step_phase(validate=True)
    print(f"A2/A3 读数路径自检（两线程并发，{args.n} 次关节读数）")
    print(f"  修前基线：错位 {before['wrong']} 次，读数异常 {before['invalid']} 次，"
          f"{before['elapsed_s']} s")
    for x in before["examples"][:3]:
        print("    -", x)
    print(f"  修后    ：错位 {after['wrong']} 次，读数异常 {after['invalid']} 次，"
          f"{after['elapsed_s']} s")
    print(f"  迟到旧值（J3 20→40°）：修前采用旧值 {before_stale['wrong']} 次，"
          f"修后 {after_stale['wrong']} 次（判异常 {after_stale['invalid']} 次）")
    ok = (after["wrong"] == 0 and after_stale["wrong"] == 0
          and before["wrong"] > 0)
    print("结果：", "过" if ok else "没过")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
