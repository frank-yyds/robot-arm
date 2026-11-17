#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""21.3 真机试验：模式 6 下单关节一次给大角度目标，报不报 0x0200。

通过运行中的执行器 HTTP 接口操作（不直接碰 ECB）：
  * 逐颗清报警并回读 0x0000 → 使能 → 确认模式 6；
  * 每一步单关节、单目标（`single_target=true`，不拆 5° 段、不流式）、10°/s；
  * 20 Hz trace 只采被测关节，记位置/电流/报警；
  * 走完回起点；任何 0x0200 立即停止后面的步骤，原地保持。

用法（用户在场、手在空开旁）：
  python3 tools/task21_mode6_test.py --yes
  python3 tools/task21_mode6_test.py --yes --port 8770

不做：21.4 的改法、其它关节的动作、失能。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8770"
CLIENT = "task21-mode6"
STEPS = [
    ("J1", 8.0), ("J1", 12.0), ("J1", 20.0), ("J1", 40.0),
    ("J4", 8.0), ("J4", 12.0), ("J4", 20.0), ("J4", 40.0),
    ("J3", 8.0), ("J3", 12.0), ("J3", 20.0),
    ("J2", 8.0), ("J2", 12.0), ("J2", 20.0),
]
SPEED = 10.0
TRACE_HZ = 20.0


class ApiError(RuntimeError):
    pass


def http(method: str, path: str, body: dict | None = None,
         timeout: float = 20.0) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        BASE + path, data=data, method=method,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode()
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode(errors="replace")
        try:
            msg = json.loads(raw).get("error") or raw
        except Exception:  # noqa: BLE001
            msg = raw
        raise ApiError(f"{path}: HTTP {exc.code} {msg}") from exc
    except Exception as exc:  # noqa: BLE001
        raise ApiError(f"{path}: {exc}") from exc
    try:
        return json.loads(raw)
    except Exception as exc:  # noqa: BLE001
        raise ApiError(f"{path}: 回包不是 JSON：{raw[:200]}") from exc


def get(path: str) -> dict:
    return http("GET", path)


def post(path: str, body: dict | None = None, timeout: float = 60.0) -> dict:
    return http("POST", path, body or {}, timeout)


def state() -> dict:
    return get("/api/state")


def q_of(st: dict) -> list[float]:
    return [float(v) for v in (st.get("q") or [0, 0, 0, 0, 0, 0])]


def joint_of(st: dict, name: str) -> dict:
    for j in st.get("joints") or []:
        if j.get("name") == name:
            return j
    return {}


def wait_idle(timeout: float = 30.0) -> dict:
    t0 = time.time()
    last = state()
    while time.time() - t0 < timeout:
        last = state()
        if not (last.get("motion") or {}).get("busy"):
            # 运动刚结束时 /api/state 的 cache 可能还是上一拍的旧位置；
            # 等一轮 poll（50 ms）再读一次实际位置。
            time.sleep(0.30)
            return state()
        time.sleep(0.05)
    raise ApiError("等待运动结束超时")


def wait_q(start: list[float], tol: float = 0.8, timeout: float = 15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = state()
        q = q_of(st)
        if not (st.get("motion") or {}).get("busy"):
            if max(abs(a - b) for a, b in zip(q, start)) <= tol:
                return st
        time.sleep(0.1)
    raise ApiError(f"没有回到起点（目标 {start}）")


def parse_trace(path: Path, joint: str) -> dict:
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("joint") != joint:
                continue
            try:
                pos = float(r["pos_deg"])
            except (TypeError, ValueError):
                continue
            cur = None
            try:
                cur = float(r["current_a"])
            except (TypeError, ValueError):
                pass
            rows.append((pos, cur, r.get("error") or ""))
    out = {"samples": len(rows)}
    if not rows:
        return out
    pos = [x[0] for x in rows]
    cur = [abs(x[1]) for x in rows if x[1] is not None]
    moves = [pos[i] - pos[i - 1] for i in range(1, len(pos))]
    out.update({
        "start_deg": round(pos[0], 4),
        "end_deg": round(pos[-1], 4),
        "max_pos_deg": round(max(pos), 4),
        "min_pos_deg": round(min(pos), 4),
        "max_abs_current_a": round(max(cur), 4) if cur else None,
        "n_current": len(cur),
        "max_step_deg": round(max(abs(x) for x in moves), 4) if moves else None,
        "nonmono_steps": sum(1 for x in moves if x < -0.05),
        "biggest_back_deg": round(min([0.0] + moves), 4),
    })
    return out


def write_report(outdir: Path, payload: dict):
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 21.3 模式 6 大角度试验", "",
             f"- 时间：{payload.get('ts')}",
             f"- 执行器：{payload.get('web_version')} / {payload.get('config_version')}",
             f"- 结论：{payload.get('conclusion')}", ""]
    for r in payload.get("trials") or []:
        g = r.get("go") or {}
        b = r.get("back") or {}
        lines += [f"## {r['joint']} {r['delta']:+.0f}°",
                  f"- 结果：{r.get('result')}；报警 {r.get('error') or '无'}",
                  f"- 去程 {g.get('duration_s')} s，回程 {b.get('duration_s')} s；"
                  f"峰值电流 {g.get('trace', {}).get('max_abs_current_a')} A",
                  f"- trace：{r.get('trace_path')}",
                  ""]
    if payload.get("error"):
        lines += ["## 中止原因", payload["error"], ""]
    (outdir / "result.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--yes", action="store_true",
                    help="确认空法兰、臂在零位有支撑、人在直流空开旁")
    ap.add_argument("--out", default="")
    ap.add_argument("--already-enabled", action="store_true",
                    help="臂已使能并模式 6：跳过 clear+enable，避免零扭矩启动再次下垂")
    args = ap.parse_args()
    global BASE
    BASE = f"http://127.0.0.1:{args.port}"
    if not args.yes:
        raise SystemExit("没有 --yes：请确认空法兰、臂在零位有支撑、人在直流空开旁")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    outdir = Path(args.out) if args.out else Path("runs") / f"mode6_test_{stamp}"
    outdir.mkdir(parents=True, exist_ok=True)

    st = state()
    cfg = get("/api/config")
    if (cfg.get("end_effector") or "flange") != "flange":
        raise SystemExit(f"末端不是 flange：{cfg.get('end_effector')}")
    if st.get("connected") and st.get("mode") != "hw":
        raise SystemExit(f"连接的不是真机：{st.get('mode')}")

    acq = post("/api/acquire", {
        "client_id": CLIENT, "config_version": cfg.get("config_version")})
    token = acq.get("token")
    if not token:
        raise SystemExit(f"拿不到控制权：{acq}")
    body0 = {"token": token, "config_version": cfg.get("config_version")}
    if not st.get("connected"):
        ecb = cfg.get("ecb") or {}
        print("执行器还没连 ECB，先连接：", ecb)
        post("/api/connect", {**body0, "mode": "hw",
                              "ip": ecb.get("ip") or "192.168.1.30",
                              "port": ecb.get("port") or 2000}, timeout=90)
        st = state()
        cfg = get("/api/config")
        if not st.get("connected"):
            raise SystemExit(f"连接后仍未连上：{st.get('msg')}")
    print(f"状态：{st.get('msg')}  q={q_of(st)}")
    rep0 = st.get("startup_report") or {}
    pre_enabled = bool(st.get("startup_done")) and all(
        j.get("enabled") for j in st.get("joints") or []) and all(
        x.get("mode") == 6 for x in rep0.get("enabled") or [])
    if args.already_enabled and not pre_enabled:
        raise SystemExit("--already-enabled 但执行器未处于已使能/模式 6 状态")

    hb_stop = threading.Event()

    def hb():
        while not hb_stop.is_set():
            try:
                post("/api/heartbeat", {"token": token}, timeout=3)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.2)

    threading.Thread(target=hb, daemon=True).start()
    body = {"token": token, "config_version": cfg.get("config_version")}

    payload = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"),
               "web_version": cfg.get("web_version"),
               "config_version": cfg.get("config_version"),
               "trials": [], "conclusion": "", "error": ""}
    try:
        # 1) 清报警并回读。已使能且六颗 error=0 时不调用 clear_errors：
        #    20.x 的 clear_errors 在“已使能但个别 0x2B/0x55 读不到”时会把
        #    startup_done 置假，这里没必要冒这个险。
        st_now = state()
        err_now = {j.get("name"): j.get("error") for j in st_now.get("joints") or []}
        if args.already_enabled and not any(err_now.values()):
            rows = []
            print("已使能且六颗 error=0：跳过 clear_errors")
        else:
            cl = post("/api/clear_errors", body)
            rows = cl.get("rows") or []
            print("清报警：", ", ".join(
                f"{r.get('name')} {r.get('before_text')}→{r.get('after_text')}"
                for r in rows))
        bad = [r for r in rows if str(r.get("after")) not in ("0x0000", "0")]
        if bad:
            payload["error"] = ("清报警后仍有非零：" + "，".join(
                f"{r.get('name')}={r.get('after_text')}" for r in bad))
            payload["conclusion"] = "未开始：报警清不掉"
            return 1

        if args.already_enabled:
            rep = rep0
            print("已使能：跳过 enable（避免零扭矩启动再次下垂）")
        else:
            # 2) 使能（零扭矩启动 + 读核 + 成链）
            en = post("/api/enable", {**body, "confirmed_supported": True},
                      timeout=120)
            rep = en.get("report") or {}
            print("使能：", rep.get("ok"), en.get("msg"))
            if not rep.get("ok") or len(rep.get("enabled") or []) != 6:
                payload["error"] = f"使能未通过：{en.get('msg')}；{rep}"
                payload["conclusion"] = "未开始：使能未通过"
                return 1
            if en.get("park_check", {}).get("has_parked"):
                payload["error"] = f"启停比对拦住：{en.get('park_check')}"
                payload["conclusion"] = "未开始：启停比对"
                return 1

        # 2b) 读实际位置和启动时的模式回读
        order = cfg.get("joint_names") or ["J1", "J2", "J3", "J4", "J5", "J6"]
        by_name = {x.get("name"): x for x in rep.get("enabled") or []}
        if args.already_enabled:
            st = state()
            q_act = q_of(st)
        else:
            q_act = [float(by_name[n]["out_deg"]) for n in order]
        modes = {n: (by_name.get(n) or {}).get("mode") for n in order}
        print("使能后实际 q：", [round(v, 3) for v in q_act])
        print("启动时回读模式：", modes)
        if max(abs(v) for v in q_act) > 5.0:
            payload["error"] = f"使能后不在零位附近（>5°）：{q_act}"
            payload["conclusion"] = "未开始：不在零位附近"
            return 1
        if any(v != 6 for v in modes.values()):
            payload["error"] = f"模式不是 6：{modes}"
            payload["conclusion"] = "未开始：模式回读不是 6"
            return 1
        time.sleep(0.8)   # 让 /api/state 的 cache 从预演占位刷成实际位置

        # 4) 逐步试验
        for joint, delta in STEPS:
            st = state()
            q_start = q_of(st)
            j0 = joint_of(st, joint)
            if j0.get("error") not in (0, None):
                payload["error"] = f"{joint} 试验前已有报警 {j0.get('error')}"
                payload["conclusion"] = "中止：试验前报警"
                break
            tr = post("/api/trace/start", {
                **body, "hz": TRACE_HZ,
                "seconds": max(12.0, 2.2 * abs(delta) / SPEED + 6.0),
                "joints": [joint]})
            trace_path = (tr.get("trace") or {}).get("path")
            time.sleep(0.6)
            print(f"\n== {joint} {delta:+.0f}°（起点 {q_start}）trace={trace_path}")

            t_go = time.time()
            post("/api/jog", {**body, "joint": joint, "delta": delta,
                              "speed_deg_s": SPEED,
                              "single_target": True})
            st = wait_idle(timeout=max(20.0, 2.5 * abs(delta) / SPEED + 10))
            go_dur = round(time.time() - t_go, 3)
            m = st.get("motion") or {}
            j1 = joint_of(st, joint)
            go = {"duration_s": go_dur, "phase": m.get("phase"),
                  "reason": m.get("reason"),
                  "pos_deg": j1.get("out_deg"), "mode": j1.get("mode"),
                  "error": j1.get("error")}
            trs = post("/api/trace/stop", body)
            tr_info = trs.get("trace") or {}
            go["trace"] = (parse_trace(Path(trace_path), joint)
                           if trace_path else {})
            go["trace"]["actual_hz"] = tr_info.get("actual_hz")

            rec = {"joint": joint, "delta": delta, "q_start": q_start,
                   "go": go, "back": {}, "trace_path": trace_path,
                   "result": "待定", "error": go.get("error")}
            print(f"  去程 {go_dur}s  phase={go['phase']}  实际 {go['pos_deg']}  "
                  f"报警={go['error']}  峰值电流={go['trace'].get('max_abs_current_a')}")
            if str(m.get("phase")) in ("aborted", "error") or j1.get("error"):
                rec["result"] = "中止"
                payload["trials"].append(rec)
                payload["error"] = (f"{joint} {delta:+.0f}° 去程报警/中止："
                                    f"{m.get('reason')}；error={j1.get('error')}")
                payload["conclusion"] = "中止：出现报警/中止"
                break

            # 回起点
            t_back = time.time()
            post("/api/jog", {**body, "joint": joint, "delta": -delta,
                              "speed_deg_s": SPEED,
                              "single_target": True})
            st = wait_idle(timeout=max(20.0, 2.5 * abs(delta) / SPEED + 10))
            back_dur = round(time.time() - t_back, 3)
            j2 = joint_of(st, joint)
            rec["back"] = {"duration_s": back_dur, "pos_deg": j2.get("out_deg"),
                           "error": j2.get("error"),
                           "phase": (st.get("motion") or {}).get("phase")}
            print(f"  回程 {back_dur}s  实际 {j2.get('out_deg')}  "
                  f"报警={j2.get('error')}")
            if j2.get("error"):
                rec["result"] = "回程报警"
                payload["trials"].append(rec)
                payload["error"] = f"{joint} 回程报警 {j2.get('error')}"
                payload["conclusion"] = "中止：回程报警"
                break
            rec["result"] = "通过"
            payload["trials"].append(rec)
            wait_q(q_start, tol=0.8, timeout=5)

        if not payload["error"]:
            payload["conclusion"] = "全部步骤无 0x0200"
    except Exception as exc:  # noqa: BLE001
        payload["error"] = str(exc)
        payload["conclusion"] = "异常中止"
    finally:
        hb_stop.set()
        try:
            post("/api/release", {"token": token}, timeout=5)
        except Exception:  # noqa: BLE001
            pass
        write_report(outdir, payload)

    print("\n" + (payload["conclusion"] or ""))
    if payload.get("error"):
        print("错误：", payload["error"])
    print("报告：", outdir / "result.md")
    return 0 if payload["conclusion"] == "全部步骤无 0x0200" else 2


if __name__ == "__main__":
    sys.exit(main())
