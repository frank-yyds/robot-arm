#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J3-HOLD-20261116 诊断：回零 → 长 J2 往复 → 回搁架。

trace 记 J2/J3（20 Hz，power 0x2B / mode 0x55 10 Hz），抓掉使能/切模式瞬间。
用法（用户在场、空法兰、手在空开旁）：
  python3 tools/j3_hold_stress.py --yes --cycles 20
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8770"
CLIENT = "j3-hold-stress"
SHELF = [92.2017, 0.0, 0.0, 0.0, 0.0, 0.0]  # J1 先转回 +92 的竖直姿态
LIFT = [92.2017, 75.5142, 0.0984, 68.7773, -0.0014, 0.0061]
SHELF_Q = [92.2017, 82.441, 2.512, 67.48, 0.008, 0.005]


class ApiError(RuntimeError):
    pass


def http(method: str, path: str, body: dict | None = None, timeout: float = 60.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode(errors="replace")
        try:
            msg = json.loads(raw).get("error") or raw
        except Exception:  # noqa: BLE001
            msg = raw
        raise ApiError(f"{path}: HTTP {exc.code} {msg}") from exc
    except Exception as exc:  # noqa: BLE001
        raise ApiError(f"{path}: {exc}") from exc


def get(p): return http("GET", p, timeout=15)
def post(p, b=None, t=60): return http("POST", p, b or {}, t)


def state(): return get("/api/state")


def wait_idle(timeout=40.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = state()
        m = s.get("motion") or {}
        if not m.get("busy"):
            time.sleep(0.25)
            return state()
        time.sleep(0.05)
    raise ApiError("等待运动结束超时")


def q_of(s): return [float(v) for v in (s.get("q") or [0] * 6)]


def parse_trace(path: Path, joint="J3"):
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    out = {"path": str(path), "samples": 0, "power_bad": [], "mode_bad": [],
           "max_jump": 0.0, "min": None, "max": None, "current_max": None}
    prev = None
    for r in rows:
        if r.get("joint") != joint:
            continue
        try:
            pos = float(r["pos_deg"])
        except (TypeError, ValueError):
            continue
        out["samples"] += 1
        out["min"] = pos if out["min"] is None else min(out["min"], pos)
        out["max"] = pos if out["max"] is None else max(out["max"], pos)
        if prev is not None:
            out["max_jump"] = max(out["max_jump"], abs(pos - prev))
        prev = pos
        pw = (r.get("power") or "").strip()
        md = (r.get("mode") or "").strip()
        if pw and pw not in ("1",):
            out["power_bad"].append((r["t_rel"], pw))
        if md and md not in ("6",):
            out["mode_bad"].append((r["t_rel"], md))
        try:
            c = abs(float(r["current_a"]))
            out["current_max"] = c if out["current_max"] is None else max(out["current_max"], c)
        except (TypeError, ValueError, KeyError):
            pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--cycles", type=int, default=20)
    ap.add_argument("--delta", type=float, default=20.0)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    global BASE
    BASE = f"http://127.0.0.1:{args.port}"
    if not args.yes:
        raise SystemExit("没有 --yes：确认空法兰、臂有支撑、人在直流空开旁")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    outdir = Path(args.out) if args.out else Path("runs") / f"j3_stress_{stamp}"
    outdir.mkdir(parents=True, exist_ok=True)
    rep = {"ts": stamp, "cycles": args.cycles, "delta": args.delta,
           "traces": [], "cycle_rows": [], "error": "", "notes": []}

    cfg = get("/api/config")
    s = state()
    if not s.get("connected"):
        a = post("/api/acquire", {"client_id": CLIENT,
                                  "config_version": cfg["config_version"]})
        tok = a["token"]
        body = {"token": tok, "config_version": cfg["config_version"]}
        ecb = cfg.get("ecb") or {}
        print("连接 ECB", ecb)
        post("/api/connect", {**body, "mode": "hw",
                              "ip": ecb.get("ip") or "192.168.1.30",
                              "port": ecb.get("port") or 2000}, 90)
        s = state()
    else:
        a = post("/api/acquire", {"client_id": CLIENT,
                                  "config_version": cfg["config_version"]})
        tok = a["token"]
        body = {"token": tok, "config_version": cfg["config_version"]}

    hb_stop = threading.Event()

    def hb():
        while not hb_stop.is_set():
            try:
                post("/api/heartbeat", {"token": tok}, 3)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.2)

    threading.Thread(target=hb, daemon=True).start()
    try:
        if not s.get("startup_done"):
            try:
                en = post("/api/enable", {**body, "confirmed_supported": True}, 120)
            except ApiError as exc:
                print("首次 enable 失败，重连后重试：", exc)
                ecb = cfg.get("ecb") or {}
                post("/api/connect", {**body, "mode": "hw",
                                      "ip": ecb.get("ip") or "192.168.1.30",
                                      "port": ecb.get("port") or 2000}, 90)
                en = post("/api/enable", {**body, "confirmed_supported": True}, 120)
            r = en.get("report") or {}
            if not r.get("ok"):
                raise ApiError(f"enable 未通过：{r}")
            print("使能：", en.get("msg"))
            time.sleep(1.0)
        s = state()
        print("起始 q", q_of(s))

        print("回零位（先预演，再抬高回零）...")
        post("/api/home", {**body, "preview": True}, 120)
        post("/api/home", {**body, "preview": False})
        s = wait_idle(90)
        m = s.get("motion") or {}
        print("home:", m.get("phase"), q_of(s))
        if m.get("phase") in ("aborted", "error"):
            raise ApiError(f"home 中止：{m.get('reason')}")
        if max(abs(v) for v in q_of(s)) > 1.0:
            raise ApiError(f"home 后不在零位：{q_of(s)}")

        trace = None

        def new_trace():
            nonlocal trace
            r = post("/api/trace/start", {**body, "hz": 20, "seconds": 55,
                                          "joints": ["J2", "J3"],
                                          "status_hz": 10}, 20)
            trace = (r.get("trace") or {}).get("path")
            print("  trace", trace)
            time.sleep(0.3)

        def stop_trace():
            nonlocal trace
            if trace:
                try:
                    post("/api/trace/stop", body, 10)
                except Exception:  # noqa: BLE001
                    pass
                rep["traces"].append(parse_trace(Path(trace)))
                trace = None

        new_trace()
        t_trace = time.time()
        for cyc in range(1, args.cycles + 1):
            for delta in (args.delta, -args.delta):
                if time.time() - t_trace > 50.0:
                    stop_trace(); new_trace(); t_trace = time.time()
                t0 = time.time()
                post("/api/jog", {**body, "joint": "J2", "delta": delta,
                                  "preview": False, "speed_deg_s": 10,
                                  "single_target": True})
                s = wait_idle(45)
                m = s.get("motion") or {}
                j3 = [j for j in s.get("joints", []) if j.get("name") == "J3"][0]
                row = {"cycle": cyc, "delta": delta, "phase": m.get("phase"),
                       "reason": m.get("reason"), "q": q_of(s),
                       "j3": j3.get("out_deg"), "j3_err": j3.get("error"),
                       "j3_mode": j3.get("mode"), "duration_s": round(time.time() - t0, 2)}
                rep["cycle_rows"].append(row)
                print(f"cycle {cyc} {delta:+.0f}: {row['phase']} q2={q_of(s)[1]:.3f} "
                      f"J3={row['j3']} err={row['j3_err']} dur={row['duration_s']}s")
                if row["phase"] in ("aborted", "error") or row["j3_err"]:
                    rep["error"] = f"cycle {cyc} {delta:+.0f} 中止：{row['reason']}；err={row['j3_err']}"
                    raise ApiError(rep["error"])
        stop_trace()

        print("回搁架...")
        post("/api/move_through", {**body, "waypoints": [SHELF, LIFT, SHELF_Q],
                                   "preview": False})
        s = wait_idle(90)
        m = s.get("motion") or {}
        print("shelf:", m.get("phase"), q_of(s))
        rep["shelf_q"] = q_of(s)
        rep["shelf_phase"] = m.get("phase")
    except Exception as exc:  # noqa: BLE001
        rep["error"] = rep["error"] or str(exc)
        print("异常：", exc)
    finally:
        hb_stop.set()
        try:
            if trace:
                post("/api/trace/stop", body, 10)
                rep["traces"].append(parse_trace(Path(trace)))
                trace = None
        except Exception:  # noqa: BLE001
            pass
        try:
            post("/api/release", {"token": tok}, 10)
        except Exception:  # noqa: BLE001
            pass
        bad_power = sum(len(t.get("power_bad") or []) for t in rep["traces"])
        bad_mode = sum(len(t.get("mode_bad") or []) for t in rep["traces"])
        max_jump = max([t.get("max_jump") or 0 for t in rep["traces"]] or [0])
        rep["summary"] = {"bad_power": bad_power, "bad_mode": bad_mode,
                          "max_jump_deg": round(max_jump, 3)}
        (outdir / "result.json").write_text(json.dumps(rep, ensure_ascii=False, indent=2),
                                            encoding="utf-8")
        lines = ["# J3-HOLD-20261116 压力测试", "",
                 f"- 时间：{stamp}", f"- 循环：{args.cycles} × J2 ±{args.delta:g}°",
                 f"- 结果：{rep.get('error') or '无中止'}",
                 f"- power 非 1：{bad_power}；mode 非 6：{bad_mode}；J3 最大相邻跳变：{max_jump:.3f}°",
                 "", "## 每轮"]
        for r in rep["cycle_rows"]:
            lines.append(f"- {r['cycle']} {r['delta']:+.0f}° {r['phase']} "
                         f"J3={r['j3']} err={r['j3_err']} mode={r['j3_mode']} {r['duration_s']}s")
        lines += ["", "## trace"] + [f"- {t['path']} samples={t['samples']} "
                                     f"power_bad={t['power_bad'][:5]} mode_bad={t['mode_bad'][:5]} "
                                     f"max_jump={t['max_jump']:.3f}" for t in rep["traces"]]
        (outdir / "result.md").write_text("\n".join(lines), encoding="utf-8")
        print("报告：", outdir / "result.md")
        print("summary", rep["summary"])
    return 0 if not rep.get("error") else 2


if __name__ == "__main__":
    sys.exit(main())
