#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.1 自检：0x94 的电流换算（IQ14 × 0x53 满量程，不是 × 限流）。

两种用法：
    python3 tools/cvp_current_check.py            # mock：注入已知电流，核对 A 和 ρ
    python3 tools/cvp_current_check.py --hw       # 真机：0x94 vs 单独读 0x04 对照（要臂已使能）

判据（30.1 第 3 条，真机）：
  * J2、J3 各 50 组，0x94 电流和单独读电流差 ≤ 满量程 2%；
  * 两组比值的**中位数**在 0.9–1.1；
  * 报六颗的 0x53 满量程（原始值和换算值）。

真机模式只读，不发任何写命令；执行器连着 ECB 时按 30.6 的独占锁会被拒绝（锁做好以后）。
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def mock_check() -> int:
    from arm.executor import Executor
    ex = Executor()
    ex.connect("mock")
    checks = []
    try:
        # 1) 满量程缓存有没有建起来（第一次读会去读 0x53）
        cvp0 = ex.bus.read_cvp_all()
        fs = dict(getattr(ex.bus, "current_full_scale_readback", {}) or {})
        ok_fs = all(v is not None for v in fs.values()) and len(fs) == 6
        checks.append(("满量程缓存六颗都有", ok_fs,
                       f"{ {k: round(v, 3) if v else v for k, v in fs.items()} }"))

        # 2) 注入已知电流（A），0x94 读出来的 A 要等于注入值，ρ = I / 限流
        inj = {"J2": 1.2, "J3": -0.8}
        ex.bus.set_faults(currents=inj)
        cvp = ex.bus.read_cvp_all()
        vals = {}
        for n, want in inj.items():
            got = (cvp.get(n) or {}).get("current_a")
            rho = (cvp.get(n) or {}).get("rho")
            lim = ex.bus.current_limit_readback.get(n)
            want_rho = None if not lim else abs(want) / lim
            vals[n] = (got, rho, want_rho)
        ok_cur = all(v[0] is not None and abs(v[0] - inj[n]) <= 0.02
                     for n, v in vals.items())
        ok_rho = all(v[2] is not None and v[1] is not None
                     and abs(v[1] - v[2]) <= 0.01 for v in vals.values())
        checks.append(("注入电流 0x94 读数 = 注入值（±0.02 A）", ok_cur,
                       {n: None if v[0] is None else round(v[0], 3)
                        for n, v in vals.items()}))
        checks.append(("ρ = |A| / 回读限流（不是 / 满量程）", ok_rho,
                       {n: (None if v[1] is None else round(v[1], 3),
                            None if v[2] is None else round(v[2], 3))
                        for n, v in vals.items()}))

        # 3) 0x94 的 A 和 read_all(full) 的 0x04 A 要一致（同一次回读，差 ≤ 满量程 2%）
        ex.bus.clear_faults()
        st = ex.bus.read_all(full=True)
        cvp = ex.bus.read_cvp_all()
        diffs = {}
        for n in ex.bus.order:
            a1 = (st.get(n).current_a if st.get(n) else None)
            a2 = (cvp.get(n) or {}).get("current_a")
            fs_n = ex.bus.current_full_scale_readback.get(n) or 33.0
            if a1 is None or a2 is None:
                diffs[n] = None
                continue
            diffs[n] = abs(a2 - a1) / fs_n * 100.0
        ok_same = all(v is not None and v <= 2.0 for v in diffs.values())
        checks.append(("0x94 电流 vs 0x04 电流差 ≤ 满量程 2%", ok_same,
                       {k: (None if v is None else round(v, 3)) for k, v in diffs.items()}))
    finally:
        ex.disconnect()
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    return 0 if ok else 1


def hw_check() -> int:
    from arm.executor import Executor
    ex = Executor()
    r = ex.connect("hw", "192.168.1.30", 2000)
    if not r.get("ok"):
        print("连不上 ECB：", r)
        return 2
    try:
        if not getattr(ex.bus, "startup_done", False):
            print("六颗还没完成启动/使能：先走启动序列（这条对照要求臂在使能保持里）")
            return 2
        # 满量程：0x53 原始值（IQ8）和换算值
        print("# 六颗 0x53 电流满量程")
        fs_rows = {}
        for n in ex.bus.order:
            m = ex.bus.motors[n]
            raw = None
            try:
                d = m.link.read_many([(m.mid, 0x53, 2)], timeout=0.1, retries=2)
                dd = d.get((m.mid, 0x53))
                raw = None if dd is None else int.from_bytes(dd, "big", signed=True)
            except Exception:  # noqa: BLE001
                raw = None
            val = m.current_max_a()
            ex.bus.current_full_scale_readback[n] = val
            fs_rows[n] = (raw, val)
            print(f"  {n}: 原始 {raw}（IQ8）→ {val} A")
        # 对照：0x94 的电流 vs 单独读 0x04（read_all(full=True) 的 current_a）
        res = {}
        for n in ("J2", "J3"):
            pairs = []
            t0 = time.monotonic()
            while len(pairs) < 50 and time.monotonic() - t0 < 20.0:
                st = ex.bus.read_all(full=True)
                cvp = ex.bus.read_cvp_all()
                a04 = st[n].current_a if st.get(n) else None
                a94 = (cvp.get(n) or {}).get("current_a")
                if a04 is not None and a94 is not None:
                    pairs.append((a04, a94))
            if not pairs:
                res[n] = None
                continue
            fs_n = fs_rows[n][1] or 1.0
            diffs = [abs(b - a) for a, b in pairs]
            ratios = [b / a for a, b in pairs if abs(a) > 0.2]
            res[n] = {
                "n": len(pairs),
                "max_diff_pct": max(diffs) / fs_n * 100.0,
                "median_ratio": statistics.median(ratios) if ratios else None,
            }
        ok = True
        for n, r0 in res.items():
            if r0 is None:
                print(f"  {n}: 没采到样本")
                ok = False
                continue
            good = (r0["max_diff_pct"] <= 2.0
                    and r0["median_ratio"] is not None
                    and 0.9 <= r0["median_ratio"] <= 1.1)
            ok &= good
            print(f"  {n}: {r0['n']} 组，差最大 {r0['max_diff_pct']:.2f}% 满量程，"
                  f"比值中位 {r0['median_ratio'] if r0['median_ratio'] is None else round(r0['median_ratio'], 3)}"
                  f" → {'过' if good else '没过'}")
        print("结果：", "过" if ok else "没过")
        return 0 if ok else 1
    finally:
        ex.disconnect()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hw", action="store_true", help="真机对照（默认只跑 mock）")
    args = ap.parse_args(argv)
    return hw_check() if args.hw else mock_check()


if __name__ == "__main__":
    raise SystemExit(main())
