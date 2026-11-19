#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.2 第 1 条 / P1：0x94 一条指令取电流/速度/位置，和分开读对拍。

静止 50 组；匀速 20°/s 的 50 组要真机运动（另由 tools/cvp_moving_check.py 做）。
每颗报：位置差、电流差（满量程 %）、速度和位置差分速度差。

用法（执行器停掉、臂在搁架上、已零扭矩使能）：
    python3 tools/cvp_check.py --static 50
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm.transport import EcbTransport  # noqa: E402
from arm.sca import ScaMotor, R_POSITION, R_VELOCITY, R_CURRENT, DEG_PER_MOTOR_TURN  # noqa: E402
from arm import config as cfgmod  # noqa: E402
from arm.joints import load_joint_configs  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--static", type=int, default=50)
    args = ap.parse_args()
    ecb = cfgmod.load_executor()
    ip = ecb.get("ecb_ip", "192.168.1.30")
    port = int(ecb.get("ecb_udp_port", 2000))
    link = EcbTransport(ip, port, timeout=0.15, retries=2)
    cfg = load_joint_configs()
    motors = {n: ScaMotor(link, c.id) for n, c in cfg.items()}
    out = {}
    for name, m in motors.items():
        c = cfg[name]
        diffs_pos, diffs_cur, diffs_vel = [], [], []
        cm = m.current_max_a() or 0.0
        prev_cvp = None
        samples = []
        for _ in range(args.static):
            cvp = m.read_cvp()
            pos = m.pos_turns()
            vel_rpm = m.velocity_rpm()
            cur = m.current_a()
            time.sleep(0.01)
            if cvp is None or pos is None:
                continue
            dp = abs(c.out_deg(cvp["pos_turns"]) - c.out_deg(pos))
            diffs_pos.append(dp)
            if cur is not None:
                diffs_cur.append(abs(cvp["current_a"] - cur) / cm * 100.0
                                 if cm else float("nan"))
            if vel_rpm is not None and prev_cvp is not None:
                # 位置差分速度（输出端 °/s）
                dt = samples[-1][0]
                dpv = abs(c.out_deg(cvp["pos_turns"]) - c.out_deg(prev_cvp["pos_turns"])) / dt
                cv = abs(c.sign * cvp["vel_rpm"] * DEG_PER_MOTOR_TURN / 60.0)
                if dpv > 0.5:
                    diffs_vel.append(abs(cv - dpv) / dpv * 100.0)
            samples.append((time.monotonic(), cvp))
            prev_cvp = cvp
        out[name] = {
            "n": len(diffs_pos),
            "pos_diff_max_deg": round(max(diffs_pos), 4) if diffs_pos else None,
            "pos_diff_median_deg": round(statistics.median(diffs_pos), 4) if diffs_pos else None,
            "cur_diff_max_pct": round(max(diffs_cur), 2) if diffs_cur else None,
            "vel_diff_max_pct": round(max(diffs_vel), 1) if diffs_vel else None,
            "usable": bool(diffs_pos and max(diffs_pos) <= 0.01
                           and (not diffs_cur or max(diffs_cur) <= 2.0)
                           and (not diffs_vel or max(diffs_vel) <= 10.0)),
        }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    Path("runs").mkdir(exist_ok=True)
    (Path("runs") / f"cvp_static_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    link.close()


if __name__ == "__main__":
    main()
