"""六关节高频诊断采样。

只在明确调用时运行，不参与控制。默认 100 Hz，采：
位置 0x06、电流 0x04、速度 0x05（每 2 个样本）、错误 0xFF（每 5 个样本）。
写 runs/trace_YYYYmmdd_HHMMSS.csv，一行一个关节。

注意：ECB/CAN 是共享总线。采样率越高，越可能和运动/停止命令抢总线；
所以它不是常开功能，只在诊断时开 5–10 秒。
"""
from __future__ import annotations

import csv
import struct
import threading
import time
from pathlib import Path

from .sca import (DEG_PER_MOTOR_TURN, IQ24, R_CURRENT, R_ERROR, R_MODE,
                  R_POSITION, R_POWER, R_VELOCITY, VELOCITY_FULL_RPM)


class TraceSampler:
    def __init__(self, ex, hz: float = 100.0, seconds: float = 10.0,
                 joints=None, status_hz: float = 5.0):
        self.ex = ex
        self.joints = ([str(x).upper() for x in joints] if joints else None)
        # 0x2B power / 0x55 mode 的采样频率；默认 5 Hz，诊断掉使能时可调到 10 Hz
        self.status_hz = max(0.5, min(float(status_hz or 5.0), float(hz)))
        self.hz = max(5.0, min(200.0, float(hz)))
        self.seconds = max(1.0, min(60.0, float(seconds)))
        self.path: Path | None = None
        self.error = ""
        self.started = 0.0
        self.finished = 0.0
        self.samples = 0
        self.actual_hz = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self):
        if self._thread and self._thread.is_alive():
            raise RuntimeError("高频采样已经在跑")
        bus = self.ex.bus
        if bus is None or not getattr(bus, "connected", False):
            raise RuntimeError("还没有连接真机，不能高频采样")
        if not hasattr(bus, "link") or not hasattr(bus, "motors"):
            raise RuntimeError("当前不是真机 JointBus，不能高频采样")
        self._stop.clear()
        self.error = ""
        self.samples = 0
        self.actual_hz = 0.0
        self.started = time.time()
        self.finished = 0.0
        stamp = time.strftime("%Y%m%d_%H%M%S")
        runs = Path(self.ex.runs_dir)
        runs.mkdir(parents=True, exist_ok=True)
        self.path = runs / f"trace_{stamp}.csv"
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=1.5)
        with self._lock:
            active = bool(t and t.is_alive())
        return {"ok": True, "active": active}

    def status(self) -> dict:
        with self._lock:
            active = bool(self._thread and self._thread.is_alive())
            return {
                "active": active,
                "hz": self.hz,
                "seconds": self.seconds,
                "samples": self.samples,
                "actual_hz": round(self.actual_hz, 1),
                "started": self.started or None,
                "finished": self.finished or None,
                "path": (str(self.path) if self.path else None),
                "error": self.error,
            }

    def _run(self):
        bus = self.ex.bus
        order = [n for n in (self.joints or list(bus.order)) if n in bus.motors]
        cm: dict[str, float | None] = {}
        # 电流满量程 0x53 只读一次；读不到就记 None
        for n in order:
            try:
                cm[n] = bus.motors[n].current_max_a(timeout=0.05, retries=1)
            except Exception:  # noqa: BLE001
                cm[n] = None
        ids = {n: bus.motors[n].mid for n in order}
        period = 1.0 / self.hz
        status_every = max(1, int(round(self.hz / self.status_hz)))
        t0 = time.monotonic()
        next_t = t0
        last_sample = t0
        tick = 0
        try:
            assert self.path is not None
            with self.path.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["time", "t_rel", "sample", "joint", "pos_deg",
                            "target_deg", "current_a", "velocity_deg_s",
                            "error", "power", "mode", "dt_s"])
                f.flush()
                while not self._stop.is_set():
                    tick += 1
                    now = time.monotonic()
                    if now - t0 > self.seconds:
                        break
                    dt = max(0.0, now - last_sample)
                    last_sample = now
                    items = []
                    for n in order:
                        mid = ids[n]
                        items.append((mid, R_POSITION, 4))
                        items.append((mid, R_CURRENT, 4))
                        if tick % 2 == 0:
                            items.append((mid, R_VELOCITY, 4))
                        if tick % 5 == 0:
                            items.append((mid, R_ERROR, 2))
                        if tick % status_every == 0:
                            items.append((mid, R_POWER, 1))
                            items.append((mid, R_MODE, 1))
                    try:
                        got = bus.link.read_many(items, timeout=0.03, retries=1)
                    except Exception as exc:  # noqa: BLE001
                        got = {}
                        self.error = f"读总线失败：{exc}"
                    tr = now - t0
                    tstamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                    targets = dict(getattr(bus, "_targets", {}))
                    for n in order:
                        mid = ids[n]
                        cfg = bus.cfg[n]
                        p = got.get((mid, R_POSITION))
                        pos = None
                        if p and len(p) >= 4:
                            pos = cfg.out_deg(struct.unpack(">i", p)[0] / IQ24)
                        c = got.get((mid, R_CURRENT))
                        cur = None
                        if c and len(c) >= 4 and cm.get(n):
                            cur = struct.unpack(">i", c)[0] / IQ24 * cm[n]
                        v = got.get((mid, R_VELOCITY))
                        vel = None
                        if v and len(v) >= 4:
                            rpm = struct.unpack(">i", v)[0] / IQ24 * VELOCITY_FULL_RPM
                            vel = cfg.sign * rpm * DEG_PER_MOTOR_TURN / 60.0
                        e = got.get((mid, R_ERROR))
                        err = None
                        if e and len(e) >= 2:
                            err = struct.unpack(">H", e)[0]
                        pw = got.get((mid, R_POWER))
                        power = pw[0] if pw else None
                        md = got.get((mid, R_MODE))
                        mode = md[0] if md else None
                        w.writerow([tstamp, f"{tr:.4f}",
                                    tick, n,
                                    "" if pos is None else f"{pos:.4f}",
                                    "" if targets.get(n) is None else f"{targets[n]:.4f}",
                                    "" if cur is None else f"{cur:.4f}",
                                    "" if vel is None else f"{vel:.3f}",
                                    "" if err is None else f"0x{err:04X}",
                                    "" if power is None else str(power),
                                    "" if mode is None else str(mode),
                                    f"{dt:.4f}"])
                    self.samples = tick
                    if tick % 10 == 0:
                        f.flush()
                    elapsed = time.monotonic() - t0
                    self.actual_hz = tick / elapsed if elapsed > 0 else 0.0
                    next_t += period
                    sleep = next_t - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    elif sleep < -0.2:
                        # 总线太慢，别无限补旧 tick
                        next_t = time.monotonic()
                f.flush()
        except Exception as exc:  # noqa: BLE001
            self.error = f"采样异常：{exc}"
        finally:
            end_mono = time.monotonic()
            self.finished = time.time()
            with self._lock:
                self.actual_hz = self.samples / max(1e-6, end_mono - t0)
