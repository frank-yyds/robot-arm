#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.6 自检：ECB 独占锁、交接规则、重连接管（mock）。

① 锁被占时：第二个进程（或直接连 ECB 的脚本）拒绝启动；持锁进程被强行杀掉以后，
   锁自动释放，可以重新连。
② 注入“重连时六颗使能着、模式 6”：接管成功，全程没有 0x07 / 0x08 / 0x2A，
   各颗位置变化 ≤0.2°。
③ 注入“重连时 J3 在模式 1”：一条写指令都不发，给出提示。

用法：
    cd arm-control && python3 tools/lock_takeover_check.py
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def check(name, ok, detail=""):
    print(f"  {'过' if ok else '没过'}：{name}（{detail}）")
    return bool(ok)


# --------------------------------------------------------------------------- #
# ① 独占锁
# --------------------------------------------------------------------------- #
def test_lock() -> list[tuple[str, bool, str]]:
    out = []
    from arm import ecblock
    tmp = Path(tempfile.mkdtemp(prefix="ecb_lock_"))
    lockp = tmp / ".ecb.lock"
    child_code = (
        "import sys, time\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        "from arm import ecblock\n"
        f"l = ecblock.EcbLock('192.168.1.30:2000', path=__import__('pathlib').Path({str(lockp)!r}))\n"
        "l.acquire()\n"
        "print('HELD', flush=True)\n"
        "time.sleep(30)\n"
    )
    child = subprocess.Popen([sys.executable, "-c", child_code],
                             stdout=subprocess.PIPE, text=True)
    line = child.stdout.readline().strip()
    out.append(("①-1 子进程拿到锁", line == "HELD", line))
    # 第二个进程必须被拒
    code2 = (
        "import sys\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        "from arm import ecblock\n"
        "from pathlib import Path\n"
        "try:\n"
        f"    ecblock.EcbLock('x', path=Path({str(lockp)!r})).acquire()\n"
        "    print('ACQ')\n"
        "except ecblock.EcbLockBusy as e:\n"
        "    print('BUSY', str(e)[:80])\n"
    )
    r2 = subprocess.run([sys.executable, "-c", code2], capture_output=True, text=True)
    out.append(("①-2 第二个进程被拒（提示 PID 正连着 ECB）",
                r2.stdout.startswith("BUSY") and "PID" in r2.stdout,
                (r2.stdout or r2.stderr).strip()[:90]))
    # 直接连 ECB 的脚本：EcbTransport 也要被拒（用不可达 IP，锁检查要在建 socket 之前）
    code3 = (
        "import sys\n"
        f"sys.path.insert(0, {str(ROOT)!r})\n"
        "from arm import ecblock\n"
        "from arm.transport import EcbTransport\n"
        "from pathlib import Path\n"
        "import arm.transport as T\n"
        f"T.ECB_LOCK_PATH = Path({str(lockp)!r})\n"
        "try:\n"
        "    EcbTransport('192.0.2.1', 2000)\n"
        "    print('MADE')\n"
        "except Exception as e:\n"
        "    print('REFUSED', str(e)[:80])\n"
    )
    r3 = subprocess.run([sys.executable, "-c", code3], capture_output=True, text=True)
    out.append(("①-3 直接连 ECB 的脚本被拒",
                r3.stdout.startswith("REFUSED"), (r3.stdout or r3.stderr).strip()[:90]))
    # 杀掉持锁进程 → 锁自动释放
    child.send_signal(signal.SIGKILL)
    try:
        child.wait(timeout=5)
    except Exception:  # noqa: BLE001
        pass
    time.sleep(0.2)
    st = ecblock.status(path=lockp)
    ok_free = st.get("held") is False
    try:
        l = ecblock.EcbLock("x", path=lockp)
        l.acquire()
        l.release()
        ok_reacq = True
    except Exception as exc:  # noqa: BLE001
        ok_reacq = False
    out.append(("①-4 持锁进程被 kill 后锁自动释放、可以重新连",
                ok_free and ok_reacq, f"status={st.get('held')} 重新拿={ok_reacq}"))
    return out


# --------------------------------------------------------------------------- #
# ②／③ 重连接管（mock）
# --------------------------------------------------------------------------- #
def make_mock_exec():
    from arm.executor import Executor
    ex = Executor()
    # 先造一个“六颗已使能、模式 6”的 mock 总线（不经过 connect，避免自动接管）
    from arm.mock import MockJointBus
    bus = MockJointBus(initial_deg={"J1": 0.0, "J2": 0.0, "J3": 0.0,
                                    "J4": 0.0, "J5": 0.0, "J6": 0.0})
    bus.enabled = True
    bus.startup_done = True
    bus.force_mode = {n: 6 for n in bus.order}
    ex.bus = bus
    ex.mode = "mock"
    ex.connected = True
    ex.runs_dir = Path(tempfile.mkdtemp(prefix="takeover_"))
    from arm.session import Session
    ex.session = Session(ex.runs_dir, tag="mock")
    bus.write_log_path = str(ex.session.dir / "mock_writes.jsonl")
    from arm.recorder import Recorder
    ex.recorder = Recorder(ex.runs_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_run.csv",
                           session=ex.session)
    from arm.motion import Motion
    ex.motion = Motion(bus, ex.robot, ex.fence, ex.limits, recorder=ex.recorder)
    return ex


def writes_of(ex, cmds) -> list:
    """看会话记录里有没有发过这些写指令。"""
    rows = []
    p = Path(getattr(ex.bus, "write_log_path", "") or "")
    if p.exists():
        for ln in p.read_text(encoding="utf-8").splitlines():
            try:
                r = json.loads(ln)
            except Exception:  # noqa: BLE001
                continue
            if int(r.get("cmd", -1)) in cmds and r.get("src", "executor") == "executor":
                rows.append(r)
    return rows


def test_takeover() -> list[tuple[str, bool, str]]:
    out = []
    # ② 正常接管
    ex = make_mock_exec()
    try:
        before = {n: ex.bus.q[n] for n in ex.bus.order}
        n_write0 = ex.bus.write_log_count
        rep = ex.bus.try_takeover()
        time.sleep(0.3)
        after = {n: ex.bus.q[n] for n in ex.bus.order}
        moved = max(abs(after[n] - before[n]) for n in before)
        forbidden = writes_of(ex, {0x07, 0x08, 0x2A})
        out.append(("②-1 接管成功、进入保持",
                    bool(rep.get("took_over")) and ex.bus.startup_done,
                    f"ok={rep.get('ok')} msg={str(rep.get('msg'))[:60]}"))
        out.append(("②-2 接管全程没有 0x07 / 0x08 / 0x2A",
                    len(forbidden) == 0, f"违禁写 {len(forbidden)} 条"))
        out.append(("②-3 各颗位置变化 ≤0.2°（接管不该动臂）",
                    moved <= 0.2, f"最大变化 {moved:.3f}°"))
        # 接管以后出口和普通保持一样：能恢复/能停机（这里只查状态可读）
        out.append(("②-4 接管后 startup_done=True、目标=当前位置",
                    ex.bus.startup_done
                    and all(abs(ex.bus._targets.get(n, 1e9) - after[n]) <= 0.2
                            for n in ex.bus.order),
                    str({n: round(ex.bus._targets.get(n, 0), 2) for n in ex.bus.order})))
    finally:
        try:
            ex.session.close()
        except Exception:  # noqa: BLE001
            pass

    # ③ J3 在模式 1 → 一条写指令都不发
    ex = make_mock_exec()
    try:
        ex.bus.force_mode = {n: 6 for n in ex.bus.order}
        ex.bus.force_mode["J3"] = 1
        n_write0 = ex.bus.write_log_count
        msg = ""
        try:
            rep3 = ex.bus.try_takeover()
            msg = str(rep3.get("msg") or "") if not rep3.get("took_over") else ""
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
        after_writes = ex.bus.write_log_count - n_write0
        out.append(("③-1 J3 模式 1：拒绝接管并给出提示",
                    "接管" in msg and "J3" in msg, msg[:90]))
        out.append(("③-2 一条写指令都没发", after_writes == 0,
                    f"写命令 {after_writes} 条"))
    finally:
        try:
            ex.session.close()
        except Exception:  # noqa: BLE001
            pass
    return out


def main(argv=None) -> int:
    checks = []
    checks += test_lock()
    checks += test_takeover()
    ok = all(c[1] for c in checks)
    for name, passed, detail in checks:
        print(f"  {'过' if passed else '没过'}：{name}（{detail}）")
    print("结果：", "过" if ok else "没过")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
