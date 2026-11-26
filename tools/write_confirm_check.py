#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.7 第 7 条 / 37.1 第 2 条：写目标重发改按位置判（mock，不碰真机）。

判据：写目标后 t_c=t_lag+sqrt(0.2/a) 内朝目标走了 ≥0.05° 算收到；
不到就重发一次（只记事件），重发后再等一个 t_c 还不动 → 存快照、原地保持。

mock 加上真机同量级的读数噪声：位置 ±0.005°、速度标准差 0.7°/s（36.3 第 2 条）。
另量 1000 次写目标排队时间。
"""
from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402

START = {"J1": 0.0, "J2": 30.0, "J3": 30.0, "J4": 90.0, "J5": 0.0, "J6": 0.0}
FAILS = []


def check(name, ok, detail=""):
    print(f"  {'过' if ok else '没过'}：{name}（{detail}）")
    if not ok:
        FAILS.append(name)
    return {"name": name, "ok": bool(ok), "detail": str(detail)}


def mk():
    e = Executor({"bind_host": "127.0.0.1", "bind_port": 0, "ecb_ip": "",
                  "idle_park_s": 600, "push_hz": 50, "state_push_hz": 50,
                  "runs_dir": tempfile.mkdtemp(prefix="runs37_wc_")})
    e.connect("mock")
    e.bus.q.update(START)
    e.bus.target.update(START)
    e.bus._targets.update(START)
    e.bus.pos_noise_deg = 0.005
    e.bus.vel_noise_std = 0.7
    e.bus.enable_all(True, allow_restart=True)
    return e


def kinds(e):
    out = {}
    p = Path(e.session.dir) / "events.jsonl"
    for ln in p.read_text(encoding="utf-8").splitlines():
        try:
            k = json.loads(ln).get("kind")
        except Exception:  # noqa: BLE001
            continue
        out[k] = out.get(k, 0) + 1
    return out


def move(e, target, timeout=25.0, recover=True):
    e.move_joints(target, preview=False)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not e.motion.status.get("busy"):
            break
        time.sleep(0.02)
    ph = e.motion.status.get("phase")
    if recover and ph in ("aborted", "error"):
        try:
            e.recover()
        except Exception:  # noqa: BLE001
            pass
    return ph


def main():
    out = {"checks": []}
    # 1) 25%、50% 各 50 段正常动作：0 重发
    for scale in (0.25, 0.5):
        e = mk()
        e.set_speed_scale(scale)
        for i in range(50):
            move(e, [0, 35 if i % 2 == 0 else 30, 30, 90, 0, 0])
        k = kinds(e)
        bad = k.get("target_confirm_resend", 0) + k.get("target_lost", 0)
        out["checks"].append(check(
            f"{int(scale * 100)}% 50 段正常动作重发 0 次", bad == 0,
            f"resend={k.get('target_confirm_resend', 0)} lost={k.get('target_lost', 0)}"))
        e.disconnect()
    # 2) 写丢一次：重发 1 次以后走完
    e = mk()
    e.set_speed_scale(0.25)
    e.bus.auto_resend_on_drop = False
    e.bus.drop_targets["J2"] = 1
    ph = move(e, [0, 40, 30, 90, 0, 0])
    k = kinds(e)
    out["checks"].append(check(
        "写丢一次：重发 1 次以后照常走完",
        k.get("target_confirm_resend", 0) == 1 and k.get("target_lost", 0) == 0
        and ph == "done", f"phase={ph} {k}"))
    e.disconnect()
    # 3) 连丢两次：存快照、原地保持
    e = mk()
    e.set_speed_scale(0.25)
    e.bus.auto_resend_on_drop = False
    e.bus.drop_targets["J2"] = 2
    ph = move(e, [0, 40, 30, 90, 0, 0], recover=False)
    k = kinds(e)
    out["checks"].append(check(
        "连丢两次：存快照、原地保持",
        k.get("target_lost", 0) >= 1 and k.get("incident", 0) >= 1 and ph == "aborted",
        f"phase={ph} {k}"))
    e.disconnect()
    # 4) 队列时间
    e = mk()
    q0 = dict(e.bus.q)
    times = []
    for _ in range(1000):
        t = time.monotonic()
        e.bus.set_targets_q({n: q0[n] for n in e.bus.order})
        times.append((time.monotonic() - t) * 1000.0)
    times.sort()
    out["checks"].append(check(
        "1000 次写目标排队时间 ≤50 ms", times[-1] <= 50.0,
        f"中位 {times[500]:.3f} / 99 {times[990]:.3f} / 最大 {times[-1]:.3f} ms"))
    e.disconnect()
    out["ok"] = all(c["ok"] for c in out["checks"])
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_37" / "write_confirm_37.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("结果：", "过" if out["ok"] else "没过")
    print(f"结果写 {p}")
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
