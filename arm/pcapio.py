#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十九节 29.1/29.2/29.6：抓包的通用读写 + ECB 帧/写命令的独立解码。

这个模块只是 pcap 文件格式和 ECB 线上帧的“独立见证”解码，不 import
执行器的协议代码（arm.transport / arm.sca / arm.joints），所以执行器自己
记错了也能用它核对。tools/pcap_check.py 用它读包，但自己再对一遍写命令。

支持：
  * 经典 pcap（tcpdump 默认写的），大端/小端、微秒/纳秒时间戳；
  * Linux cooked SLL（tcpdump -i any）和 Ethernet；
  * IPv4 + UDP 负载里的 ECB 帧：
        EE | addr | cmd | len_hi len_lo | data | crc16(data) 低字节在前 | ED
    （和 ECB 网桥实测一致；解析不验证 CRC，只按帧长切）
  * 只把“发往 ECB 的 UDP 包”当请求（src/dst 由调用方给，默认 dst=192.168.1.30）。

注意：tcpdump 环形文件按 -C 100 -W 20 滚动，文件名不一定连续；读多个文件
时按时间戳排序去重（同一包可能跨文件重复）。
"""
from __future__ import annotations

import struct
from pathlib import Path

# pcap 全局头 magic：微秒（大端/小端）、纳秒（大端/小端）
# 文件头 4 字节和写入端字节序相反：d4c3b2a1 是小端文件（tcpdump 在 x86/Mac 上写的），
# a1b2c3d4 才是大端文件。原来这张表写反了，读 tcpdump 抓的文件会按大端解、
# caplen 乱、第一个包就 break，pcap_check 永远看不到包（11-24 真机上发现）。
_MAGIC = {
    b"\xd4\xc3\xb2\xa1": ("<", 1e-6),
    b"\xa1\xb2\xc3\xd4": (">", 1e-6),
    b"\x4d\x3c\xb2\xa1": ("<", 1e-9),
    b"\xa1\xb2\x3c\x4d": (">", 1e-9),
}

LINKTYPE_ETHERNET = 1
LINKTYPE_LINUX_SLL = 113
LINKTYPE_LINUX_SLL2 = 276


def read_pcap(path) -> list[tuple[float, bytes]]:
    """读一个经典 pcap，返回 [(unix 时间, 原始链路层包), ...]。"""
    data = Path(path).read_bytes()
    if len(data) < 24:
        raise ValueError(f"{path} 不是 pcap（太短）")
    magic = data[:4]
    if magic not in _MAGIC:
        raise ValueError(f"{path} 不是经典 pcap（magic {magic.hex()}；"
                         "pcapng 请先用 tcpdump -r 转一下）")
    endian, scale = _MAGIC[magic]
    out = []
    off = 24
    n = len(data)
    while off + 16 <= n:
        sec, frac, caplen, _origlen = struct.unpack_from(endian + "IIII", data, off)
        off += 16
        if off + caplen > n:
            break
        out.append((sec + frac * scale, data[off:off + caplen]))
        off += caplen
    return out


def write_pcap(path, packets, linktype: int = LINKTYPE_ETHERNET) -> None:
    """写回经典 pcap（微秒）。packets = [(unix 时间, 原始链路层包), ...]。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0,
                            65535, linktype))
        for ts, raw in packets:
            sec = int(ts)
            usec = max(0, min(999999, int(round((ts - sec) * 1e6))))
            f.write(struct.pack("<IIII", sec, usec, len(raw), len(raw)))
            f.write(raw)


def _ipv4_udp(linktype: int, raw: bytes):
    """从链路层包取 (src_ip, src_port, dst_ip, dst_port, payload)；不是 UDP 返回 None。"""
    try:
        if linktype == LINKTYPE_ETHERNET:
            if len(raw) < 14:
                return None
            eth_type = int.from_bytes(raw[12:14], "big")
            off = 14
            # VLAN tag
            while eth_type in (0x8100, 0x88A8) and len(raw) >= off + 4:
                eth_type = int.from_bytes(raw[off + 2:off + 4], "big")
                off += 4
            if eth_type != 0x0800:
                return None
        elif linktype in (LINKTYPE_LINUX_SLL, LINKTYPE_LINUX_SLL2):
            # SLL: 16 字节（2 pkttype + 2 arphrd + 2 addrlen + 8 addr + 2 proto）
            # SLL2: 20 字节（2 proto + 2 rsvd + 4 ifindex + 2 arphrd + ...）
            if linktype == LINKTYPE_LINUX_SLL2:
                if len(raw) < 20:
                    return None
                proto = int.from_bytes(raw[0:2], "big")
                off = 20
            else:
                if len(raw) < 16:
                    return None
                proto = int.from_bytes(raw[14:16], "big")
                off = 16
            if proto != 0x0800:
                return None
        else:
            return None
        if len(raw) < off + 20:
            return None
        ihl = (raw[off] & 0x0F) * 4
        if ihl < 20 or len(raw) < off + ihl + 8:
            return None
        if raw[off + 9] != 17:      # UDP
            return None
        src = ".".join(str(b) for b in raw[off + 12:off + 16])
        dst = ".".join(str(b) for b in raw[off + 16:off + 20])
        udp = off + ihl
        sport = int.from_bytes(raw[udp:udp + 2], "big")
        dport = int.from_bytes(raw[udp + 2:udp + 4], "big")
        ulen = int.from_bytes(raw[udp + 4:udp + 6], "big")
        payload = raw[udp + 8:udp + max(8, ulen)]
        return src, sport, dst, dport, payload
    except Exception:  # noqa: BLE001
        return None


def parse_ecb_frames(buf: bytes) -> list[tuple[int, int, bytes, bytes]]:
    """独立按帧长切 ECB 帧，返回 [(addr, cmd, data, raw_frame), ...]，不校验 CRC。"""
    out = []
    i = 0
    n = len(buf)
    while i + 6 <= n:
        if buf[i] != 0xEE:
            i += 1
            continue
        length = int.from_bytes(buf[i + 3:i + 5], "big")
        body_end = i + 5 + length
        if body_end > n:
            i += 1
            continue
        data = buf[i + 5:body_end]
        # 有数据时后面带 2 字节 CRC；无数据时后面直接是 ED
        if length and body_end + 3 <= n and buf[body_end + 2] == 0xED:
            raw = buf[i:body_end + 3]
        elif not length and body_end + 1 <= n and buf[body_end] == 0xED:
            raw = buf[i:body_end + 1]
        elif length and body_end + 1 <= n and buf[body_end] == 0xED:
            raw = buf[i:body_end + 1]
        else:
            i += 1
            continue
        out.append((buf[i + 1], buf[i + 2], data, raw))
        i = body_end + (3 if length and body_end + 2 < n and buf[body_end + 2] == 0xED else 1)
    return out


def iter_pcap_udp(paths, ecb_ip: str = "192.168.1.30",
                  ecb_port: int = 2000):
    """遍历 pcap 文件里发往 ECB 的 UDP 包。

    返回 [(unix 时间, src_ip, src_port, payload), ...]（按时间排序去重）。
    """
    if isinstance(paths, (str, Path)):
        paths = [paths]
    rows = []
    seen = set()
    for p in paths:
        try:
            pkts = read_pcap(p)
        except Exception:  # noqa: BLE001
            continue
        # 文件头 linktype 要读；read_pcap 不返回，单独读
        try:
            head = Path(p).read_bytes()[:24]
            endian = _MAGIC[head[:4]][0]
            linktype = struct.unpack_from(endian + "I", head, 20)[0]
        except Exception:  # noqa: BLE001
            linktype = LINKTYPE_ETHERNET
        for ts, raw in pkts:
            got = _ipv4_udp(linktype, raw)
            if not got:
                continue
            src_ip, sport, dst_ip, dport, payload = got
            if dst_ip != ecb_ip or dport != ecb_port:
                continue
            frames = parse_ecb_frames(payload)
            if not frames:
                continue
            key = (round(ts, 6), src_ip, sport, payload)
            if key in seen:
                continue
            seen.add(key)
            rows.append((ts, src_ip, sport, payload))
    rows.sort(key=lambda r: r[0])
    return rows


def iter_write_frames(paths, ecb_ip: str = "192.168.1.30",
                      ecb_port: int = 2000):
    """遍历发往 ECB 的写类命令帧：[(t, addr, cmd, data_hex, src_port), ...]。

    写类指令集合和第二十九节一致（读命令不参与对账）。
    """
    write_cmds = {
        0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
        0x14, 0x1F, 0x20, 0x21, 0x25, 0x26, 0x27, 0x2A, 0x2E, 0x2F, 0x30,
        0x31, 0x32, 0x33, 0x58, 0x5A, 0x5E, 0x70, 0x72, 0x74, 0x76, 0x78,
        0x7A, 0x7E, 0x83, 0x84, 0x87, 0x88, 0x89, 0x8C, 0x90, 0x91, 0xFE,
    }
    out = []
    for ts, src_ip, sport, payload in iter_pcap_udp(paths, ecb_ip, ecb_port):
        for addr, cmd, data, _raw in parse_ecb_frames(payload):
            if cmd in write_cmds:
                out.append((ts, addr, cmd, data.hex(), sport))
    out.sort(key=lambda r: r[0])
    return out
