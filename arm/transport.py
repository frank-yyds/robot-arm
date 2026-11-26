#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ECB UDP 传输层：帧、CRC、握手、枚举、请求/应答、重试。

ECB 只是网口→CAN 网桥，线上跑的还是 SCA 指令。帧格式沿用 ecb-test/ecb_test.py
（已在实机上验证）：
    EE | addr | cmd | len_hi len_lo | data | crc16(data) 低字节在前 | ED
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass

HEAD = 0xEE
TAIL = 0xED
DEFAULT_IP = "192.168.1.30"
DEFAULT_PORT = 2000

# 30.6 第 1 条：ECB 独占锁的文件路径（测试里可以覆盖）。None = 用默认 .ecb.lock
ECB_LOCK_PATH = None


class EcbError(RuntimeError):
    pass


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_frame(addr: int, cmd: int, data: bytes = b"") -> bytes:
    if not 0 <= addr <= 0xFF:
        raise ValueError("地址超出 0x00~0xFF")
    body = bytes([HEAD, addr, cmd]) + len(data).to_bytes(2, "big") + data
    if data:
        body += struct.pack("<H", crc16(data))
    return body + bytes([TAIL])


@dataclass
class Frame:
    addr: int
    cmd: int
    data: bytes
    raw: bytes
    crc: bytes | None = None

    @property
    def crc_ok(self) -> bool | None:
        if self.crc is None:
            return None
        return struct.pack("<H", crc16(self.data)) == self.crc


def parse_frames(buf: bytes) -> list[Frame]:
    out: list[Frame] = []
    i = 0
    while i + 6 <= len(buf):
        if buf[i] != HEAD:
            i += 1
            continue
        n = int.from_bytes(buf[i + 3:i + 5], "big")
        j = i + 5 + n
        data = buf[i + 5:j]
        crc_frame = j + 3 <= len(buf) and buf[j + 2] == TAIL
        no_crc = j + 1 <= len(buf) and buf[j] == TAIL
        if crc_frame and buf[j:j + 2] == struct.pack("<H", crc16(data)):
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 3], buf[j:j + 2]))
            i = j + 3
        elif no_crc:
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 1], None))
            i = j + 1
        elif crc_frame:
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 3], buf[j:j + 2]))
            i = j + 3
        else:
            i += 1
    return out


class EcbTransport:
    """同步请求/应答 + 重试。线程安全（一把锁串行化请求）。"""

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 timeout: float = 0.25, retries: int = 3, debug: bool = False):
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.retries = int(retries)
        self.debug = debug
        # 30.6：真 ECB 通信前先拿独占锁；被别的进程占着就拒绝启动（16.4 第 10 条）
        self._ecb_lock = None
        try:
            from .ecblock import EcbLock
            lock = EcbLock(f"{ip}:{int(port)}", path=ECB_LOCK_PATH)
            lock.acquire()
            self._ecb_lock = lock
        except Exception as exc:  # noqa: BLE001
            from .ecblock import EcbLockBusy
            if isinstance(exc, EcbLockBusy):
                raise
            # 拿不到锁文件（例如只读文件系统）不挡通信，但记一句
            self._ecb_lock = None
            self._lock_error = str(exc)
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("", 0))
            # 26.7 7b：写目标走单独发送口，不排在读取的重试后面
            self._wsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._wsock.bind(("", 0))
        except Exception:  # noqa: BLE001
            if self._ecb_lock is not None:
                self._ecb_lock.release()
                self._ecb_lock = None
            raise
        # 29.1 修订（2026-11-26）：抓包转写钩子（执行器设；把发出去的帧抄一份成 pcap）
        self.tee_hook = None
        self._wlock = threading.Lock()
        self._lock = threading.RLock()
        self.tx = 0
        self.rx = 0
        self.tx_fail = 0
        self.timeouts = 0
        self.last_rx_time: float | None = None
        # 24 节 A2/A5：执行器运行期间所有收发都从这里走；命令钩子供记录用
        # fn(addr, cmd, data, ok) —— ok None 表示只发不等回包
        self.event_hook = None
        # 上一次事务超时：总线可能还有迟到回包，下一次事务先等一下再读空
        self._sync_lost = False
        # 26.7：按指令统计发送/回包/超时，供链路诊断
        self.by_cmd: dict[int, dict[str, int]] = {}
        self._resync_until = 0.0
        self.dup_window_s = 0.004     # 收到一条应答后这段窗口内同 key 的包按重复丢弃
        self.resync_wait_s = 0.03     # 超时之后先等这么久，再读空旧包
        self.repeat_dropped = 0

    def local_ip(self) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((self.ip, self.port))
            return s.getsockname()[0]
        except OSError:
            return "?"
        finally:
            s.close()

    # ---------------- 底层 ----------------
    def _stat(self, cmd: int, key: str, n: int = 1) -> None:
        d = self.by_cmd.setdefault(cmd, {"tx": 0, "rx": 0, "timeout": 0})
        d[key] = d.get(key, 0) + n

    def _tee(self, raw: bytes, sock=None) -> None:
        """把一帧发出去的原始字节交给抓包转写（失败不影响通信）。"""
        hook = getattr(self, "tee_hook", None)
        if hook is None:
            return
        try:
            sport = 0
            if sock is not None:
                try:
                    sport = sock.getsockname()[1]
                except Exception:  # noqa: BLE001
                    sport = 0
            hook(raw, self.ip, self.port, sport)
        except Exception:  # noqa: BLE001
            pass

    def _send(self, raw: bytes) -> None:
        try:
            self._sock.sendto(raw, (self.ip, self.port))
        except OSError as exc:
            self.tx_fail += 1
            raise EcbError(f"UDP 发不出去：{exc}") from exc
        self.tx += 1
        self._tee(raw, self._sock)

    def _collect(self, seconds: float) -> list[Frame]:
        frames: list[Frame] = []
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            self._sock.settimeout(min(0.02, left))
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            got = parse_frames(data)
            self.rx += len(got)
            self.last_rx_time = time.monotonic()
            frames.extend(got)
        return frames

    def _drain_rx(self) -> None:
        """发新请求前丢掉套接字里已经到的旧包。

        上一次超时请求的迟到回包如果留着，可能被当成这一次同指令的应答。
        只丢弃，不计入 rx；不保留历史缓冲。
        """
        self._sock.setblocking(False)
        try:
            while True:
                try:
                    self._sock.recvfrom(8192)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
        finally:
            self._sock.setblocking(True)

    def _resync(self) -> None:
        """上一次事务超时过：先等一段（让迟到回包落地），再读空。"""
        if not self._sync_lost:
            return
        left = self._resync_until - time.monotonic()
        if left > 0:
            time.sleep(left)
        self._drain_rx()
        self._sync_lost = False

    def _drain_dup(self, addr: int, cmd: int, window: float | None = None) -> int:
        """收到一条应答后，把紧跟其后的同 (addr, cmd) 重复回包丢掉。

        协议里回包不带序号，只能按“同一请求的重复/迟到回包”处理：
        保留先到的一条，其余计入 repeat_dropped。
        """
        win = self.dup_window_s if window is None else window
        dropped = 0
        end = time.monotonic() + win
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            self._sock.settimeout(min(0.004, left))
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            for f in parse_frames(data):
                if f.addr == addr and f.cmd == cmd:
                    dropped += 1
        self.repeat_dropped += dropped
        return dropped

    def request(self, addr: int, cmd: int, data: bytes = b"",
                timeout: float | None = None, retries: int | None = None) -> Frame | None:
        timeout = self.timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries
        raw = make_frame(addr, cmd, data)
        got_frame = None
        with self._lock:
            for _ in range(max(1, retries)):
                self._resync()
                self._drain_rx()
                self._send(raw)
                self._stat(cmd, "tx")
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    for f in self._collect(min(0.002, end - time.monotonic())):
                        # ECB 握手应答的 CRC 两字节是 00 00（不参与校验），
                        # 其它指令仍跳过 CRC 不符的帧。
                        if f.crc_ok is False and not (f.addr == 0 or f.cmd == 0x44):
                            continue
                        if f.addr == addr and f.cmd == cmd:
                            got_frame = f
                            break
                    if got_frame is not None:
                        break
                if got_frame is not None:
                    # A2：单个请求不等重复窗口（总线串行，20 ms 会把 trace
                    # 节拍压到 3 Hz）；重复包靠下一次 _drain_rx / _resync 清掉。
                    self._stat(cmd, "rx")
                    break
                self.timeouts += 1
                self._stat(cmd, "timeout")
                self._sync_lost = True
                self._resync_until = time.monotonic() + self.resync_wait_s
        if self.event_hook is not None:
            try:
                self.event_hook(addr, cmd, data, got_frame is not None)
            except Exception:  # noqa: BLE001
                pass
        return got_frame


    def send_now(self, addr: int, cmd: int, data: bytes = b"") -> None:
        """26.7 7b：立即发一条不等回包的写命令，不占读取的锁。"""
        raw = make_frame(addr, cmd, data)
        with self._wlock:
            self._wsock.sendto(raw, (self.ip, self.port))
        self.tx += 1
        self._tee(raw, self._wsock)
        self._stat(cmd, "tx")
        if self.event_hook is not None:
            try:
                self.event_hook(addr, cmd, data, None)
            except Exception:  # noqa: BLE001
                pass

    def send_cmd(self, addr: int, cmd: int, data: bytes = b"") -> None:
        """只发不等回（有些固件的使能/失能应答不可靠，但状态会变）。"""
        with self._lock:
            self._resync()
            self._send(make_frame(addr, cmd, data))
        if self.event_hook is not None:
            try:
                self.event_hook(addr, cmd, data, None)
            except Exception:  # noqa: BLE001
                pass

    def _drain_dup_any(self, keys, window: float = 0.006) -> None:
        end = time.monotonic() + window
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            self._sock.settimeout(min(0.004, left))
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            for f in parse_frames(data):
                if (f.addr, f.cmd) in keys:
                    self.repeat_dropped += 1

    def read_many(self, items, timeout: float = 0.05,
                  retries: int = 2) -> dict[tuple[int, int], bytes]:
        """一次发多颗的读请求，再收集回包（items=[(addr,cmd,n)]）。

        单条请求的固件/网桥回包约 20 ms；串行 6 颗要 130 ms。
        并发发 6 条通常 10~20 ms 收齐，丢的再补发一轮。
        """
        results: dict[tuple[int, int], bytes] = {}
        pending = list(items)
        want = {(a, c): n for a, c, n in pending}
        with self._lock:
            for _ in range(max(1, retries)):
                if not pending:
                    break
                self._resync()
                self._drain_rx()
                for addr, cmd, _n in pending:
                    self._send(make_frame(addr, cmd))
                    self._stat(cmd, "tx")
                end = time.monotonic() + timeout
                got_round: set[tuple[int, int]] = set()
                while time.monotonic() < end and len(got_round) < len(want):
                    for f in self._collect(min(0.002, end - time.monotonic())):
                        key = (f.addr, f.cmd)
                        if key in want and len(f.data) >= want[key] and key not in results:
                            results[key] = f.data[:want[key]]
                            got_round.add(key)
                            self._stat(f.cmd, "rx")
                pending = [it for it in pending if (it[0], it[1]) not in results]
                if pending:
                    self.timeouts += len(pending)
                    for _a, _c, _n in pending:
                        self._stat(_c, "timeout")
                    self._sync_lost = True
                    self._resync_until = time.monotonic() + self.resync_wait_s
                else:
                    # 这一轮要的都收到了；把紧跟着的重复回包丢掉
                    self._drain_dup_any(set(want), 0.001)
        if self.event_hook is not None:
            for addr, cmd, _n in items:
                try:
                    self.event_hook(addr, cmd, b"", (addr, cmd) in results)
                except Exception:  # noqa: BLE001
                    pass
        return results


    def read(self, addr: int, cmd: int, n: int, **kw) -> bytes | None:
        f = self.request(addr, cmd, b"", **kw)
        if f is None or len(f.data) < n:
            return None
        return f.data[:n]

    def write(self, addr: int, cmd: int, data: bytes = b"", timeout: float = 0.5,
              retries: int | None = None) -> bool:
        f = self.request(addr, cmd, data, timeout=timeout, retries=retries)
        return bool(f is not None and f.data and f.data[0] == 1)

    # ---------------- 高层 ----------------
    def hello(self, timeout: float = 0.8) -> bool:
        f = self.request(0x00, 0x44, timeout=timeout, retries=2)
        return bool(f is not None and f.cmd == 0x44
                    and (not f.data or f.data[0] == 1))

    def scan(self, wait: float = 2.0) -> dict[int, int]:
        with self._lock:
            self._drain_rx()
            self._send(make_frame(0x00, 0x02))
            frames = self._collect(wait)
        out: dict[int, int] = {}
        for f in frames:
            if f.addr > 0 and f.cmd == 0x02 and len(f.data) >= 4:
                out[f.addr] = int.from_bytes(f.data[:4], "big")
        return out

    def close(self) -> None:
        try:
            self._wsock.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._sock.close()
        except Exception:  # noqa: BLE001
            pass
        # 30.6：关掉连接时释放 ECB 独占锁
        if getattr(self, "_ecb_lock", None) is not None:
            try:
                self._ecb_lock.release()
            except Exception:  # noqa: BLE001
                pass
            self._ecb_lock = None

    @property
    def stats(self) -> dict:
        return {"tx": self.tx, "rx": self.rx, "tx_fail": self.tx_fail,
                "timeouts": self.timeouts,
                "last_rx_age_s": (None if self.last_rx_time is None
                                  else round(time.monotonic() - self.last_rx_time, 3))}
