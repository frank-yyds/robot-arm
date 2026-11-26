#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""37.1 自检（mock，不碰真机）：poll_hold、位置确认重发、按停来源、kept/pitch、0x21 符号、空闲自动停机。

用法：python3 tools/task37_selfcheck.py
结果写 runs/<日期>_37/selfcheck_37.json，控制台逐项过/不过。
"""
from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.executor import Executor  # noqa: E402
from arm.motion import MotionError  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  — {detail}")
    if not ok:
        FAILS.append(name)
    return {"name": name, "ok": bool(ok), "detail": str(detail)}


def mk_exec(idle=600, runs_dir=None):
    cfg = {"bind_host": "127.0.0.1", "bind_port": 0, "ecb_ip": "",
           "idle_park_s": idle, "push_hz": 50, "state_push_hz": 50,
           "runs_dir": str(runs_dir or tempfile.mkdtemp(prefix="runs37_"))}
    e = Executor(cfg)
    e.connect("mock")
    # 统一从起点姿态开始，避免每次测试都把 J3/J4 从 0 拖到 30/90（慢且不是被测项）
    start = {"J1": 0.0, "J2": 30.0, "J3": 30.0, "J4": 90.0, "J5": 0.0, "J6": 0.0}
    e.bus.q.update(start)
    e.bus.target.update(start)
    e.bus._targets.update(start)
    if runs_dir:
        e.runs_dir = Path(runs_dir)
    return e


def close(e):
    try:
        e.disconnect()
    except Exception:  # noqa: BLE001
        pass


def wait_until(pred, timeout=8.0, dt=0.02):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return time.monotonic() - t0
        time.sleep(dt)
    return None


def session_kinds(e):
    out = []
    try:
        p = Path(e.session.dir) / "events.jsonl"
        for ln in p.read_text(encoding="utf-8").splitlines():
            if ln.strip():
                out.append(json.loads(ln))
    except Exception:  # noqa: BLE001
        pass
    return out


# ---------------- 37.1 第 1 条：poll_hold ----------------
def poll_hold_case_drift(with_hold: bool):
    d = tempfile.mkdtemp(prefix="runs37_")
    e = mk_exec(runs_dir=d)
    e.bus.enable_all(True, allow_restart=True)
    e._on_shelf = False
    if with_hold:
        e.start_trace(hz=200, seconds=30, mode="cvp",
                      joints=["J1", "J2", "J3", "J4", "J5", "J6"])
        e.test_poll_hold(True)
    e.bus.drift = {"J3": 1.0}
    e.bus.target = dict(e.bus.q)
    e.bus._targets = dict(e.bus.q)
    t0 = time.monotonic()
    t = wait_until(lambda: e.hold_failure is not None, timeout=8.0)
    if with_hold:
        e.stop_trace()
    close(e)
    return t, e.hold_failure


def check_poll_hold():
    t_off, hf_off = poll_hold_case_drift(False)
    t_on, hf_on = poll_hold_case_drift(True)
    ok = (t_off is not None and t_on is not None
          and abs((t_on or 99) - (t_off or 0)) < 1.0
          and hf_on and hf_on.get("joint") == "J3")
    r = [check("37.1-1① poll_hold 开着 A3b 照常判保持失效（用时和不开时一样）", ok,
               f"不开 {None if t_off is None else round(t_off, 2)}s / 开 "
               f"{None if t_on is None else round(t_on, 2)}s，joint={hf_on.get('joint') if hf_on else None}")]
    # ② trace 结束自动关
    e = mk_exec()
    e.bus.enable_all(True, allow_restart=True)
    e.start_trace(hz=100, seconds=2, mode="cvp",
                  joints=["J1", "J2", "J3", "J4", "J5", "J6"])
    e.test_poll_hold(True)
    t_end = wait_until(lambda: not e._poll_hold, timeout=6.0)
    r.append(check("37.1-1② trace 结束以后 poll_hold 自动关", t_end is not None,
                   f"{round(t_end, 2)} s 后自动关" if t_end else "没自动关"))
    close(e)
    # ③ 拒绝：没 trace / 只采部分
    e = mk_exec()
    try:
        e.test_poll_hold(True)
        ok3a = False
    except MotionError:
        ok3a = True
    e.start_trace(hz=100, seconds=5, mode="cvp", joints=["J1"])
    try:
        e.test_poll_hold(True)
        ok3b = False
    except MotionError:
        ok3b = True
    e.stop_trace()
    close(e)
    r.append(check("37.1-1③ 没 trace / 只采部分关节时 poll_hold 被拒", ok3a and ok3b,
                   f"没 trace 拒={ok3a}，只采 1 颗拒={ok3b}"))
    return r


# ---------------- 37.1 第 2 条：位置确认重发 ----------------
def count_kinds(e, kinds):
    out = {}
    for ev in session_kinds(e):
        k = ev.get("kind")
        if k in kinds:
            out[k] = out.get(k, 0) + 1
    return out


def move_n(e, targets, n, wait=True, recover_on_abort=True):
    for _ in range(n):
        r = e.move_joints(targets, preview=False)
        if not wait:
            return r
        t = wait_until(lambda: not e.motion.status.get("busy"), timeout=20.0)
        if t is None:
            return r
        if recover_on_abort and e.motion.status.get("phase") in ("aborted", "error"):
            try:
                e.recover()
            except Exception:  # noqa: BLE001
                pass
    return r


def check_confirm():
    r = []
    for scale in (0.25, 0.5):
        e = mk_exec()
        e.bus.enable_all(True, allow_restart=True)
        e.set_speed_scale(scale)
        a = [0, 30, 30, 90, 0, 0]
        b = [0, 35, 30, 90, 0, 0]
        for i in range(50):
            move_n(e, b if i % 2 == 0 else a, 1)
        c = count_kinds(e, {"target_confirm_resend", "target_lost", "write_resend"})
        r.append(check(f"37.1-2 {int(scale * 100)}% 50 段正常动作重发 0 次", not c, str(c)))
        close(e)
    # 写丢一次
    e = mk_exec()
    e.bus.q["J2"] = 30.0
    e.bus.target["J2"] = 30.0
    e.bus._targets["J2"] = 30.0
    e.bus.enable_all(True, allow_restart=True)
    e.set_speed_scale(0.25)
    e.bus.auto_resend_on_drop = False
    e.bus.drop_targets["J2"] = 1
    move_n(e, [0, 40, 30, 90, 0, 0], 1)
    c1 = count_kinds(e, {"target_confirm_resend", "target_lost"})
    ok1 = c1.get("target_confirm_resend") == 1 and not c1.get("target_lost") \
        and e.motion.status.get("phase") == "done"
    r.append(check("37.1-2 写丢一次：重发 1 次以后照常走完", ok1, str(c1)))
    close(e)
    # 连丢两次
    e = mk_exec()
    e.bus.q["J2"] = 30.0
    e.bus.target["J2"] = 30.0
    e.bus._targets["J2"] = 30.0
    e.bus.enable_all(True, allow_restart=True)
    e.set_speed_scale(0.25)
    e.bus.auto_resend_on_drop = False
    e.bus.drop_targets["J2"] = 2
    move_n(e, [0, 40, 30, 90, 0, 0], 1, recover_on_abort=False)
    c2 = count_kinds(e, {"target_confirm_resend", "target_lost"})
    ok2 = c2.get("target_lost", 0) >= 1 and e.motion.status.get("phase") == "aborted"
    kept_in_abort = any(ev.get("kind") == "abort" and "kept=" in str(ev.get("text"))
                        for ev in session_kinds(e))
    r.append(check("37.1-2 连丢两次：存快照、原地保持", ok2, str(c2)))
    r.append(check("37.1-4 abort 事件里带 kept", kept_in_abort,
                   "abort note 含 kept=" if kept_in_abort else "abort note 没有 kept"))
    try:
        e.recover()
    except Exception:  # noqa: BLE001
        pass
    close(e)
    return r


# ---------------- 37.1 第 3、4 条 ----------------
def check_stop_and_pitch():
    r = []
    e = mk_exec()
    e.bus.enable_all(True, allow_restart=True)
    e.stop_all("用户按停", source="web")
    kinds = session_kinds(e)
    ev = [x for x in kinds if x.get("kind") == "user_stop"]
    ok = bool(ev) and "按停（网页）" in str(ev[-1].get("text")) and "kept=" in str(ev[-1].get("text"))
    r.append(check("37.1-3/4 网页按停：记来源 + kept", ok,
                   str(ev[-1].get("text"))[:80] if ev else "没有 user_stop"))
    # pitch_dev_max 写进 execute_done：mock 停机下降段带 pitch_check
    e2 = mk_exec()
    e2.bus.enable_all(True, allow_restart=True)
    e2._on_shelf = True
    try:
        e2.park(False, prereq={"gripper_empty": True, "workspace_clear": True,
                               "shelf_ready": True})
        wait_until(lambda: not e2.park_active, timeout=30.0)
    except Exception as exc:  # noqa: BLE001
        print("（pitch 测试 park 异常：", exc, "）")
    done = [x for x in session_kinds(e2) if x.get("kind") == "execute_done"]
    ok2 = any("pitch_dev_max=" in str(x.get("text")) for x in done)
    r.append(check("37.1-4 execute_done 带 pitch_dev_max", ok2,
                   next((str(x.get("text")) for x in done if "pitch_dev_max=" in str(x.get("text"))), "没有")))
    close(e2)
    close(e)
    return r


# ---------------- 37.1 第 5 条：0x21 全负数 ----------------
def check_0x21():
    e = mk_exec()
    e.bus.enable_all(True, allow_restart=True)
    e.set_speed_scale(0.5)
    move_n(e, [0, 40, 30, 90, 0, 0], 1)
    try:
        e.test_write_profile([0, 45, 30, 90, 0, 0])
    except Exception as exc:  # noqa: BLE001
        return [check("37.1-5 0x21 全负数（启动/动作/保持/停机/测试接口）", False, f"test_write_profile: {exc}")]
    e._on_shelf = False
    try:
        e.park(False, prereq={"gripper_empty": True, "workspace_clear": True,
                              "shelf_ready": True})
        wait_until(lambda: not e.park_active, timeout=30.0)
    except Exception:  # noqa: BLE001
        pass
    prof = dict(getattr(e.bus, "last_profiles", {}))
    bad = {n: v for n, v in prof.items() if v[2] is None or float(v[2]) >= 0}
    ok = bool(prof) and not bad
    close(e)
    return [check("37.1-5 0x21 全负数（启动/动作/保持/停机/测试接口）", ok,
                  f"{len(prof)} 颗有记录；非负 {bad}")]


# ---------------- 37.1 第 6 条：空闲自动停机 ----------------
def idle_case(kind):
    # ④ 要观察“动作/继续保持重置”，限时用 3 s，否则动作本身就要 1 s 多、会先到点
    e = mk_exec(idle=3 if kind == "reset" else 1)
    e.bus.enable_all(True, allow_restart=True)
    if kind == "shelf":
        e._on_shelf = True
        t = wait_until(lambda: bool(e.bus.latch_status().get("disabled_latch")), timeout=6.0)
        text = [x for x in session_kinds(e) if x.get("kind") == "idle_auto_disable"]
        ok = t is not None and bool(text)
        detail = f"{round(t, 2) if t else None}s，记录 {len(text)} 条"
        close(e)
        return check("37.1-6② 搁架上空闲到点：自动失能", ok, detail)
    if kind == "abnormal":
        e._on_shelf = False
        e.hold_failure = {"active": True, "joint": "J3", "since": time.time(),
                          "msg": "自检：保持失效"}
        t = wait_until(lambda: bool(e._idle_blocked_reason), timeout=6.0)
        ok = t is not None and not e._idle_triggered
        detail = f"blocked={e._idle_blocked_reason}"
        close(e)
        return check("37.1-6⑤ 状态异常（保持失效）：不自动停机、记一条", ok, detail)
    if kind == "aborted":
        e._on_shelf = False
        e.motion.status.update({"phase": "aborted", "reason": "自检", "busy": False})
        t = wait_until(lambda: e._idle_last_trigger_at > 0, timeout=6.0)
        t2 = wait_until(lambda: (e.park_step or {}).get("phase") == "parked", timeout=120.0)
        ok = t is not None and t2 is not None and "idle_auto_park" in [
            x.get("kind") for x in session_kinds(e)]
        detail = f"触发={t is not None}，恢复后停机={t2 is not None}"
        close(e)
        return check("37.1-6⑥ 中止保持空闲：先恢复再停机", ok, detail)
    # 空中正常停机 + 只发心跳/state 也计时 + 操作重置
    e._on_shelf = False
    if kind == "air":
        t = wait_until(lambda: e._idle_last_trigger_at > 0, timeout=6.0)
        t2 = wait_until(lambda: (e.park_step or {}).get("phase") == "parked", timeout=120.0)
        ok = t is not None and t2 is not None and "idle_auto_park" in [
            x.get("kind") for x in session_kinds(e)]
        detail = f"触发={round(t, 2) if t else None}s，停机={t2 is not None}"
        close(e)
        return check("37.1-6① 空中空闲到点：自动停机走完", ok, detail)
    if kind == "passive":
        e._on_shelf = False
        e.start_trace(hz=100, seconds=5, mode="cvp",
                      joints=["J1", "J2", "J3", "J4", "J5", "J6"])
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            e.state()
            time.sleep(0.1)
        e.stop_trace()
        t = wait_until(lambda: e._idle_last_trigger_at > 0, timeout=6.0)
        ok = t is not None
        close(e)
        return check("37.1-6③ 只读 state/开 trace 照样计时", ok,
                     f"触发={round(t, 2) if t else None}s")
    if kind == "reset":
        e._on_shelf = False
        time.sleep(0.3)
        move_n(e, [0, 32, 30, 90, 0, 0], 1)      # 动作重置
        e.idle_continue()                        # 继续保持重置
        t_short = wait_until(lambda: e._idle_last_trigger_at > 0, timeout=1.0)
        t = wait_until(lambda: e._idle_last_trigger_at > 0, timeout=6.0)
        ok = t_short is None and t is not None
        close(e)
        return check("37.1-6④ 动作/继续保持：计时从头算", ok,
                     f"0.4s 内没触发，之后 {round(t, 2) if t else None}s 触发")


def check_idle():
    r = [idle_case(k) for k in ("air", "shelf", "passive", "reset", "abnormal", "aborted")]
    # ⑦ 配置校验
    ok = True
    detail = []
    for v in (601, 0, -5):
        cfg = {"bind_host": "127.0.0.1", "bind_port": 0, "ecb_ip": "",
               "idle_park_s": v}
        e = Executor(cfg)
        try:
            e.connect("hw", ip="192.168.1.30")
            ok = False
            detail.append(f"{v}: 没拒")
        except MotionError as exc:
            detail.append(f"{v}: 拒（{'idle_park_s' in str(exc)}）")
        except Exception as exc:  # noqa: BLE001
            ok = False
            detail.append(f"{v}: 错异常 {exc}")
        finally:
            try:
                e.disconnect()
            except Exception:  # noqa: BLE001
                pass
    r.append(check("37.1-6⑦ 真 ECB 配置 idle_park_s>600 或 0：拒绝启动", ok,
                   "；".join(detail)))
    return r


def main():
    out = {"checks": []}
    for fn in (check_poll_hold, check_confirm, check_stop_and_pitch,
               check_0x21, check_idle):
        try:
            out["checks"].extend(fn())
        except Exception as exc:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            out["checks"].append({"name": fn.__name__, "ok": False,
                                  "detail": f"抛异常：{exc}"})
    out["ok"] = all(c["ok"] for c in out["checks"])
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = Path("runs") / f"{stamp[:8]}_37" / "selfcheck_37.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n结果写 {p}")
    print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
