#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""28.4 第 3 条的只读通信指标：**用记录里已有的真抓包离线算**（不碰真臂）。

28.4 第 3 条（原文）："先跑只读通信测试（26.7 第 1、3b、8 条；臂在搁架上、不使能）：
六颗都有应答，无应答 0，六颗一轮每秒 ≥100 轮。远程桌面开着、rviz 在转的时候再跑一次。"

28.4 本身要 G0 过关、PC 确认、用户在场才做，这里**不做**。本工具只回答
"这套指标在 Mac 上（也就是 26.7 那时的执行器）测出来是多少、抓包里能核对到什么"，
好在 28.3 的长跑和笔记本上做对比。

【2026-11-26 核对了一下转写文件的格式（不是 28 节的问题，顺手记）】
执行器内的抓包转写（arm/pcapio.py 的 PacketTee.feed）**每发一帧记一个包**：
自制的 Ethernet/IPv4 头（校验和算对了）+ 每帧一个 UDP 头（源端口写的是
`transport._tee` 取到的 socket 端口，但对"写口"取成了目标地址，
见 transport.py `_wsock` 那段；本次会话里全是 53100）+ 一个真正的 ECB 帧。
所以：
  * 一个 UDP 包里只有 1 帧是正常的，不是重复；
  * **同一时刻可以有好几帧**：read_cvp_all 六颗 0x94 在同一个微秒里全记下，
    因为发送前它们是一起发出去的，时间戳只到微秒，分辨不出先后 —— 所以本工具
    一轮的边界取"第一颗 J1 的 0x94"，节拍按单颗相邻两次的间隔算；
  * 转写是执行器**自己**记的（发送前），不是网络上的独立见证，回包不在里面，
    这和第 29.1 第 1 条的"独立见证"定位不同，见回报"需要 PC 定"。

指标怎么算：
  1. 一轮 = 六颗各一条 0x94（按下发顺序，J1…J6）。轮的边界取"第一颗 J1 的 0x94"。
  2. 节拍 = 1 ÷（同一颗关节相邻两次 0x94 的间隔），按关节分别取中位 / p95，
     再按窗口统计"每秒轮数 = 0x94 帧数 ÷ 6 ÷ 窗口时长"。
  3. 无应答 0 / 六颗都有应答：转写只抄发出去的帧，**看不到回包**，所以这一项
     只能由执行器自报（26.7 的 read_cvp_all 返回 ok=False 就是无应答）。这里只报
     "0x94 请求有没有断档"（相邻两次 0x94 间隔异常大的次数），作为链路问题的旁证。
"""
from __future__ import annotations

import argparse
import collections
import json
import statistics
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))

from arm import pcapio as P  # noqa: E402

TARGET_ROUNDS_HZ = 100.0
READ_CVP = 0x94
WRITE_CMDS = set((0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
                  0x14, 0x1F, 0x20, 0x21, 0x25, 0x26, 0x27, 0x2A, 0x2E, 0x2F, 0x30,
                  0x31, 0x32, 0x33, 0x58, 0x5A, 0x5E, 0x70, 0x72, 0x74, 0x76, 0x78,
                  0x7A, 0x7E, 0x83, 0x84, 0x87, 0x88, 0x89, 0x8C, 0x90, 0x91, 0xFE))


def find_pcaps(args) -> list[Path]:
    if args.pcap:
        out = []
        for p in args.pcap:
            q = Path(p)
            out.extend(sorted(q.glob("*.pcap")) if q.is_dir() else [q])
        return [p for p in out if p.is_file()]
    if args.session:
        for s in (ROOT / "runs" / args.session, Path(args.session)):
            if s.is_dir():
                got = sorted(s.glob("*.pcap"))
                if got:
                    return got
    ring = ROOT / "runs" / "_pcap_ring"
    files = sorted(ring.glob("ecb_tee_*.pcap"),
                   key=lambda p: p.stat().st_mtime) if ring.is_dir() else []
    return files[-1:] if files else []


def linktype_of(path: Path) -> int:
    try:
        head = path.read_bytes()[:24]
        endian = P._MAGIC[head[:4]][0]
        return struct.unpack_from(endian + "I", head, 20)[0]
    except Exception:  # noqa: BLE001
        return P.LINKTYPE_ETHERNET


def requests(pcaps):
    """按时间顺序返回 [(t, [(addr, cmd), ...])]；每个元素对应转写里的一个包（一帧）。"""
    out = []
    for p in pcaps:
        lt = linktype_of(p)
        for ts, raw in P.read_pcap(p):
            got = P._ipv4_udp(lt, raw)
            if not got:
                continue
            frames = P.parse_ecb_frames(got[4])
            if not frames:
                continue
            out.append((ts, [(a, c) for a, c, _d, _r in frames]))
    out.sort(key=lambda r: r[0])
    return out


def cvp_times(reqs):
    """所有 0x94 请求：(t, 关节, 一次 sendto 里第几个)。"""
    out = []
    for t, sig in reqs:
        k = 0
        for a, c in sig:
            if c == READ_CVP:
                out.append((t, a, k))
                k += 1
    return out


def periods(cvps):
    """每颗关节相邻两次 0x94 的间隔（ms），按关节分会话（间隔 > gap 视为换会话）。"""
    per = collections.defaultdict(list)
    last = {}
    for t, a, _k in cvps:
        if a in last:
            dt = (t - last[a]) * 1000.0
            if dt < 1000.0:
                per[a].append(dt)
        last[a] = t
    return per


def windows(cvps, gap_s):
    """0x94 连续出现的段（间隔 > gap_s 断开）。"""
    out, cur = [], []
    for t, a, k in cvps:
        if cur and t - cur[-1][0] > gap_s:
            out.append(cur)
            cur = []
        cur.append((t, a, k))
    if cur:
        out.append(cur)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="0x94 六颗一轮节拍（只读记录，不碰真臂）")
    ap.add_argument("--pcap", nargs="*")
    ap.add_argument("--session")
    ap.add_argument("--gap", type=float, default=0.5, help="0x94 段切分间隔 s")
    ap.add_argument("--out")
    args = ap.parse_args(argv)

    pcaps = find_pcaps(args)
    if not pcaps:
        print("没有 pcap 可读：给 --pcap 或 --session", file=sys.stderr)
        return 2
    reqs = requests(pcaps)
    cvps = cvp_times(reqs)
    per = periods(cvps)
    print(f"pcap：{', '.join(str(p) for p in pcaps)}")
    print(f"转写包 {len(reqs)} 个；0x94 请求 {len(cvps)} 条")
    if not cvps:
        print("记录里没有 0x94 请求（这段会话没走 26.2 的读法）")
        return 3

    print("\n每颗关节 0x94 的重复周期（相邻两次间隔）：")
    print(f"{'关节':>6} {'样本':>8} {'中位 ms':>9} {'p5 ms':>8} {'p95 ms':>8} {'中位折算 轮/s':>14}")
    joint_rows = {}
    for a in sorted(per):
        v = sorted(per[a])
        med = statistics.median(v)
        joint_rows[a] = {"n": len(v), "median_ms": round(med, 3),
                         "p5_ms": round(v[int(len(v) * 0.05)], 3),
                         "p95_ms": round(v[int(len(v) * 0.95)], 3)}
        print(f"{'J' + str(a):>6} {len(v):8d} {med:9.3f} {joint_rows[a]['p5_ms']:8.3f} "
              f"{joint_rows[a]['p95_ms']:8.3f} {1000.0 / med:14.1f}")

    ws = [w for w in windows(cvps, args.gap) if len(w) >= 60]
    rows = []
    for w in ws:
        dur = w[-1][0] - w[0][0]
        if dur < 1.0:
            continue
        # 一轮：从某一颗的 0x94 出发到这颗下一次 0x94 之前
        counts = collections.Counter(a for _t, a, _k in w)
        n_rounds = min(counts.values())
        rows.append({"t_rel_s": round(w[0][0] - reqs[0][0], 1),
                     "dur_s": round(dur, 2), "cvp_requests": len(w),
                     "rounds": n_rounds,
                     "hz": round(len(w) / 6.0 / dur, 1) if dur else 0.0,
                     "joints": len(counts)})
    print(f"\n0x94 连续段 {len(ws)} 段，≥1 s 的 {len(rows)} 段：")
    print(f"{'开始(rel s)':>11} {'时长s':>7} {'0x94条':>7} {'折算轮/s':>9} {'颗':>3}")
    for r in sorted(rows, key=lambda x: -x["hz"])[:12]:
        print(f"{r['t_rel_s']:11.1f} {r['dur_s']:7.2f} {r['cvp_requests']:7d} "
              f"{r['hz']:9.1f} {r['joints']:3d}")

    best = max(rows, key=lambda x: x["hz"]) if rows else None
    med_all = statistics.median([statistics.median(v) for v in per.values()]) if per else None
    print("\n口径（28.4 第 3 条：六颗一轮每秒 ≥100 轮）")
    if best:
        print(f"  记录里最好的连续段：{best['hz']:.1f} 轮/s（{best['dur_s']} s、"
              f"{best['cvp_requests']} 条 0x94、{best['joints']} 颗）  "
              f"{'≥100 ✓' if best['hz'] >= TARGET_ROUNDS_HZ else '<100 ✗'}")
    if med_all:
        print(f"  按单颗周期中位折算：{1000.0 / med_all:.1f} 轮/s")
    print("  这是 **Mac 上、26.7 那段执行器** 的离线统计，不是 28.4 的实测；"
          "笔记本上要重跑。")
    print("  无应答次数抓包里看不到（转写只抄发出去的帧），要以执行器 read_cvp_all 的 "
          "ok=False 计数为准。")

    result = {"pcaps": [str(p) for p in pcaps], "sendto": len(reqs), "cvp_requests": len(cvps),
"per_joint": {str(k): v for k, v in joint_rows.items()},
              "period_median_ms": med_all, "segments": rows, "best": best,
              "target_hz": TARGET_ROUNDS_HZ}
    if args.out:
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1),
                                  encoding="utf-8")
        print(f"\n已写 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
