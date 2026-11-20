#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十九节 29.1 第 1 条：连着真 ECB 时的 tcpdump 环形抓包（独立见证）。

由会话启动脚本 `tools/session_start.sh` 拉起、`tools/session_stop.sh` 结束，
执行器只读取状态的 pidfile 把它写进会话头、做事故快照切片；不需要 sudo 时
也可以由执行器在 connect(hw) 时自动拉起（配置里开 auto_start_capture）。

环形参数：每个文件 100 MB、留 20 个（-C 100 -W 20），只抓和 ECB（默认
192.168.1.30）之间的 UDP，不限端口（ECB 控制口 2000 + 现场可能用到的口）。
文件放在 runs/_pcap_ring/，文件名 ecb_<日期时间>.pcap；事故快照按时间窗口
切片后复制进 incidents/<编号>/pcap_slice.pcap。
"""
from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

PID_NAME = "tcpdump.pid"
LOG_NAME = "tcpdump.log"


def _pidfile(ring_dir: Path) -> Path:
    return Path(ring_dir) / PID_NAME


def _read_pid(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except Exception:  # noqa: BLE001
        return None


def _alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


class Capture:
    """tcpdump 环形抓包的启停和状态。只对真机/真网口有意义。"""

    def __init__(self, ring_dir, iface: str = "", ecb_ip: str = "192.168.1.30",
                 ecb_port: int = 2000, ring_mb: int = 100, ring_files: int = 20,
                 enabled: bool = True):
        self.ring_dir = Path(ring_dir)
        self.iface = str(iface or "")
        self.ecb_ip = str(ecb_ip)
        self.ecb_port = int(ecb_port)
        self.ring_mb = int(ring_mb)
        self.ring_files = int(ring_files)
        self.enabled = bool(enabled)

    # ---------------- 状态 ----------------
    def status(self) -> dict:
        pid = _read_pid(_pidfile(self.ring_dir))
        files = sorted(self.ring_dir.glob("*.pcap")) if self.ring_dir.is_dir() else []
        return {
            "available": bool(shutil.which("tcpdump")),
            "active": _alive(pid),
            "pid": pid,
            "iface": self.iface,
            "ring_dir": str(self.ring_dir),
            "ring_mb": self.ring_mb,
            "ring_files": self.ring_files,
            "files": [f.name for f in files],
            "files_count": len(files),
            "total_mb": round(sum(f.stat().st_size for f in files) / 1e6, 1),
            "ecb": f"{self.ecb_ip}:{self.ecb_port}",
        }

    # ---------------- 启停 ----------------
    def start(self) -> dict:
        if not self.enabled:
            return {"ok": False, "error": "配置里关掉了抓包（capture.enabled=false）"}
        if not shutil.which("tcpdump"):
            return {"ok": False, "error": "找不到 tcpdump"}
        st = self.status()
        if st["active"]:
            return {"ok": True, "already": True, **st}
        self.ring_dir.mkdir(parents=True, exist_ok=True)
        if not self.iface:
            return {"ok": False, "error": "没有配置抓包网口（capture.iface）"}
        # 只抓和 ECB 之间的 UDP；-C 单位是 1,000,000 字节；-W 环形文件数
        expr = f"udp and host {self.ecb_ip}"
        cmd = ["tcpdump", "-i", self.iface, "-n", "-s", "0",
               "-C", str(self.ring_mb), "-W", str(self.ring_files),
               "-w", str(self.ring_dir / "ecb_%Y%m%d_%H%M%S.pcap"),
               expr]
        sudo = shutil.which("sudo")
        launcher = cmd
        if sudo and os.geteuid() != 0:
            # 需要用户输密码时，由会话启动脚本执行；这里只试免密 sudo。
            check = subprocess.run([sudo, "-n", "true"], capture_output=True)
            if check.returncode != 0:
                return {"ok": False, "needs_sudo": True,
                        "error": "抓包要 sudo：请用 tools/session_start.sh 启动（会让你输一次密码）",
                        "cmd": " ".join(cmd)}
            launcher = [sudo, "-n"] + cmd
        log = (self.ring_dir / LOG_NAME).open("ab")
        try:
            p = subprocess.Popen(launcher, stdout=log, stderr=log,
                                 start_new_session=True)
        except Exception as exc:  # noqa: BLE001
            log.close()
            return {"ok": False, "error": f"启动 tcpdump 失败：{exc}"}
        _pidfile(self.ring_dir).write_text(str(p.pid))
        time.sleep(0.3)
        if not _alive(p.pid):
            return {"ok": False, "error": "tcpdump 起来就退出了，看 " + str(self.ring_dir / LOG_NAME),
                    "pid": p.pid}
        return {"ok": True, **self.status()}

    def stop(self) -> dict:
        pid = _read_pid(_pidfile(self.ring_dir))
        if not pid:
            return {"ok": True, "already": True, "msg": "没有在跑"}
        sudo = shutil.which("sudo")
        for sig in (signal.SIGTERM, signal.SIGINT):
            try:
                if sudo and os.geteuid() != 0:
                    subprocess.run([sudo, "-n", "kill", f"-{int(sig)}", str(pid)],
                                   capture_output=True)
                else:
                    os.kill(pid, sig)
            except OSError:
                pass
            for _ in range(20):
                if not _alive(pid):
                    break
                time.sleep(0.05)
            if not _alive(pid):
                break
        try:
            _pidfile(self.ring_dir).unlink()
        except OSError:
            pass
        return {"ok": not _alive(pid), "pid": pid, **self.status()}

    def slice_window(self, t_from: float, t_to: float, out_path) -> dict:
        """把环形文件里 [t_from, t_to] 的包切出来写一个 pcap。"""
        from . import pcapio
        files = sorted(self.ring_dir.glob("*.pcap"))
        if not files:
            return {"ok": False, "error": "没有抓包文件"}
        try:
            packets = []
            linktype = pcapio.LINKTYPE_ETHERNET
            for f in files:
                head = f.read_bytes()[:24]
                magic = head[:4]
                if magic not in pcapio._MAGIC:      # noqa: SLF001
                    continue
                endian = pcapio._MAGIC[magic][0]    # noqa: SLF001
                lt = int.from_bytes(head[20:24], endian)
                linktype = lt
                for ts, raw in pcapio.read_pcap(f):
                    if t_from <= ts <= t_to:
                        packets.append((ts, raw))
                    elif ts > t_to + 1.0:
                        break
            packets.sort(key=lambda x: x[0])
            if not packets:
                return {"ok": False, "error": "窗口内没有包"}
            pcapio.write_pcap(out_path, packets, linktype)
            return {"ok": True, "path": str(out_path), "packets": len(packets),
                    "files_used": len(files)}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}


def slice_ring(incident_dir, trigger_ts: float, before_s: float = 10.0,
               after_s: float = 10.0, ring_dir=None,
               ecb_ip: str = "192.168.1.30") -> dict:
    """29.2：把环形抓包里事故前后一段切进 incidents/<编号>/pcap_slice.pcap。"""
    d = Path(incident_dir)
    ring = Path(ring_dir) if ring_dir is not None else (d.parents[2]
                                                       / "_pcap_ring")
    cap = Capture(ring, ecb_ip=ecb_ip)
    try:
        return cap.slice_window(trigger_ts - float(before_s),
                                trigger_ts + float(after_s),
                                d / "pcap_slice.pcap")
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": str(exc)}
