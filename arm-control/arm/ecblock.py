#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.6 第 1 条：ECB 独占锁——同一时刻只允许一个进程和 ECB 通信。

16.4 第 10 条一直这么要求，现在做成硬的：
  * 任何真 ECB 通信都经过 `EcbTransport`，它在建立 socket 之前先拿这把锁；
  * 锁用操作系统的文件锁（fcntl.flock），进程退出或被 kill 时由内核自动释放，
    不需要手工清理；锁文件里另外写一份 PID / 开始时间 / 目标 IP，给人看；
  * 同一个进程里重复拿锁是允许的（比如一个工具里建两个 Transport），不自己挡自己；
  * 锁文件位置：`<repo>/.ecb.lock`，不入库（.gitignore）。

用法：
    from arm.ecblock import EcbLock, EcbLockBusy, status
    EcbLock("192.168.1.30:2000").acquire()     # 被占时抛 EcbLockBusy
    EcbLock("...").release()
"""
from __future__ import annotations

import json
import os
import socket
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOCK_PATH = ROOT / ".ecb.lock"

_mutex = threading.RLock()
# 同一个进程里已经拿过锁的标记：pid -> (路径, 计数)
_held_by_me: dict[int, tuple[Path, int]] = {}


class EcbLockBusy(RuntimeError):
    """ECB 已经被另一个进程占着。"""

    def __init__(self, holder: dict | None):
        self.holder = holder or {}
        pid = self.holder.get("pid")
        started = self.holder.get("started") or "?"
        ip = self.holder.get("ip") or "?"
        super().__init__(
            f"ECB 已经被另一个进程占着：PID {pid}（{started} 开始），目标 {ip}。"
            "同一时刻只允许一个进程和 ECB 通信（16.4 第 10 条 / 30.6）。"
            "要读数据请走执行器的接口（/api/state 等），不要另连 ECB。")


def _read_holder(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def _write_holder(path: Path, info: dict, fd: int | None = None) -> None:
    """把锁持有者写进锁文件。

    注意：不能用“写临时文件再 replace”的写法——replace 会换 inode，
    flock 锁在旧 inode 上就失效了，第二个进程会以为没人占。必须写同一个 fd。
    """
    data = json.dumps(info, ensure_ascii=False, indent=1).encode("utf-8")
    try:
        if fd is not None and fd >= 0:
            os.ftruncate(fd, 0)
            os.lseek(fd, 0, os.SEEK_SET)
            os.write(fd, data)
            return
        path.write_text(data.decode("utf-8"), encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass


class EcbLock:
    """一把独占锁。acquire() 成功以后要一直拿着 fd，直到 release()。"""

    def __init__(self, target: str = "192.168.1.30:2000",
                 path: Path | None = None):
        self.target = str(target or "")
        self.path = Path(path or LOCK_PATH)
        self._fd: int | None = None
        self.info: dict = {}

    # ---------------- 拿锁 ----------------
    def acquire(self, timeout_s: float = 0.0) -> dict:
        import fcntl
        pid = os.getpid()
        with _mutex:
            mine = _held_by_me.get(pid)
            if mine is not None:
                # 同一个进程：计数 +1，不重复 flock（flock 在同一 fd 上会自我覆盖）
                _held_by_me[pid] = (mine[0], mine[1] + 1)
                self._fd = -1            # -1 = 借用本进程已有的锁
                self.info = _read_holder(self.path)
                return self.info
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
            deadline = time.monotonic() + max(0.0, float(timeout_s))
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except OSError:
                    if time.monotonic() >= deadline:
                        holder = _read_holder(self.path)
                        os.close(fd)
                        raise EcbLockBusy(holder) from None
                    time.sleep(0.05)
            self._fd = fd
            self.info = {
                "pid": pid,
                "host": socket.gethostname(),
                "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "ip": self.target,
                "script": os.path.basename(
                    __import__("sys").argv[0] if __import__("sys").argv else "?"),
            }
            _write_holder(self.path, self.info, fd=fd)
            _held_by_me[pid] = (self.path, 1)
            return self.info

    # ---------------- 放锁 ----------------
    def release(self) -> None:
        import fcntl
        pid = os.getpid()
        with _mutex:
            mine = _held_by_me.get(pid)
            if mine is None:
                self._fd = None
                return
            count = mine[1] - 1
            if count > 0:
                _held_by_me[pid] = (mine[0], count)
                self._fd = None
                return
            _held_by_me.pop(pid, None)
            if self._fd is not None and self._fd >= 0:
                try:
                    fcntl.flock(self._fd, fcntl.LOCK_UN)
                except OSError:
                    pass
                try:
                    os.close(self._fd)
                except OSError:
                    pass
            self._fd = None
            # 锁文件留着（内容是最后一个持有者），下一次 acquire 会覆盖
            try:
                info = _read_holder(self.path)
                if int(info.get("pid") or 0) == pid:
                    _write_holder(self.path, {**info, "released": time.strftime(
                        "%Y-%m-%dT%H:%M:%S")}, fd=None)
            except Exception:  # noqa: BLE001
                pass

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()
        return False


def status(path: Path | None = None) -> dict:
    """给网页/工具看：现在锁有没有被占、占的是谁。只读，不拿锁。"""
    import fcntl
    p = Path(path or LOCK_PATH)
    holder = _read_holder(p)
    if not p.exists():
        return {"held": False, "holder": None, "path": str(p)}
    try:
        fd = os.open(p, os.O_RDWR | os.O_CREAT, 0o644)
    except OSError as exc:
        return {"held": None, "holder": holder, "path": str(p),
                "error": f"打不开锁文件：{exc}"}
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # 拿到了 → 说明没人占；立刻放掉
            fcntl.flock(fd, fcntl.LOCK_UN)
            return {"held": False, "holder": holder, "path": str(p)}
        except OSError:
            return {"held": True, "holder": holder, "path": str(p)}
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
