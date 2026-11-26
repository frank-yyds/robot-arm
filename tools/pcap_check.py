#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""29.6：抓包对账（独立见证），报两类问题：

  * "执行器记了但没发出去"：events.jsonl 里有这条写命令，抓包里没有；
  * "发出去了但执行器没记"：抓包里有这条写命令，events.jsonl 里没有；
  * 另外报"不是执行器发出的包"（抓包里源端口不是执行器的发送口）。

mock 下没有真抓包，用 mock 总线自己的写命令日志（`mock_writes.jsonl`）代替，
第三种问题变成"src != executor"的日志行（模拟另一个进程往 mock ECB 发包）。

用法：
    python3 tools/pcap_check.py runs/<会话> [--pcap-dir DIR] [--ecb 192.168.1.30]
                                  [--from 12 --to 20] [--tol-ms 300]

只读。退出码：0 = 两类问题都是 0；1 = 有对不上的。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

WRITE_CMDS = None    # 由 arm.pcapio 提供同一张表


def load_events(session: Path) -> list[dict]:
    out = []
    p = session / "events.jsonl"
    for ln in (p.read_text(encoding="utf-8").splitlines() if p.exists() else []):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def load_mock_writes(session: Path) -> list[dict]:
    out = []
    p = session / "mock_writes.jsonl"
    for ln in (p.read_text(encoding="utf-8").splitlines() if p.exists() else []):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:  # noqa: BLE001
            continue
    return out


def _write_sign(p: dict) -> int | None:
    """写命令数据的符号：+1/0/−1；解不出返回 None。

    pcap 的 `data` 是 4 字节大端的十六进制（0x21 就是 int32 倍数），取最高位符号；
    mock 的 `data` 是已经换算好的十进制值（mock_writes.jsonl 的 value）。
    """
    data = p.get("data")
    if data in (None, ""):
        return None
    txt = str(data).strip()
    if len(txt) == 8 and all(c in "0123456789abcdefABCDEF" for c in txt):
        try:
            iv = int.from_bytes(bytes.fromhex(txt), "big", signed=True)
        except ValueError:
            return None
        return (iv > 0) - (iv < 0)
    try:
        fv = float(txt)
    except ValueError:
        return None
    return (fv > 0) - (fv < 0)


def sign_stats_0x21(packets: list[dict]) -> dict:
    """37.2 第 3 条：0x21 写指令的总条数、负数、正数（应当全是负数）。

    只加统计输出，不对账逻辑做任何改动。
    """
    tot = neg = pos = zero = unknown = 0
    for p in packets:
        if int(p.get("cmd", -1)) != 0x21:
            continue
        tot += 1
        sgn = _write_sign(p)
        if sgn is None:
            unknown += 1
        elif sgn < 0:
            neg += 1
        elif sgn > 0:
            pos += 1
        else:
            zero += 1
    return {"total": tot, "negative": neg, "positive": pos,
            "zero": zero, "unknown": unknown}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--pcap-dir", default="", help="抓包目录（默认 会话/_pcap 或 runs/_pcap_ring）")
    ap.add_argument("--ecb", default="192.168.1.30")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--tol-ms", type=float, default=300.0)
    args = ap.parse_args(argv)
    d = Path(args.session)
    if not d.exists():
        print(f"没有这个会话目录：{d}", file=sys.stderr)
        return 2

    events = load_events(d)
    writes_ev = [e for e in events if e.get("kind") == "cmd"]
    # 会话开始的墙上时间：用来把 pcap 的 unix 时间换算成相对秒
    meta = {}
    try:
        meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        meta = {}
    start_ts = None
    try:
        import datetime as _dt
        start_ts = _dt.datetime.fromisoformat(
            str(meta.get("started"))).timestamp()
    except Exception:  # noqa: BLE001
        start_ts = None

    # 会话结束时刻：events 最后一条的墙上时间（没有就用"会话开始 + 12 小时"兜底）
    end_ts = None
    for e in reversed(events):
        w = e.get("wall")
        if not w:
            continue
        try:
            import datetime as _dt
            end_ts = _dt.datetime.fromisoformat(str(w)).timestamp()
        except Exception:  # noqa: BLE001
            end_ts = None
        break
    if end_ts is None and start_ts is not None:
        end_ts = start_ts + 12 * 3600.0

    # mock 会话绝不能拿真抓包来对账（runs/_pcap_ring 里可能正有真机会话的包）
    is_mock = str(meta.get("connector") or "").lower().startswith("mock") or \
        bool(meta.get("mock"))
    pcap_files = []
    pcap_dir = None if is_mock else (Path(args.pcap_dir) if args.pcap_dir else None)
    if not is_mock and pcap_dir is None:
        for cand in (d / "_pcap", Path("runs/_pcap_ring")):
            if cand.exists():
                pcap_dir = cand
                break
    # 29.1 修订：会话目录里有执行器自己转写的 pcap（ecb_tee_*.pcap）就优先它，
    # 不再去公共环里翻（环里可能是几小时前 tcpdump 的老文件）
    own_tee = []
    if not is_mock:
        own_tee = sorted(d.glob("ecb_tee_*.pcap")) + sorted(d.glob("ecb_tee_*.pcap*"))
    if own_tee:
        pcap_files = list(dict.fromkeys(own_tee))
    elif pcap_dir is not None and pcap_dir.exists():
        pcap_files = sorted(pcap_dir.glob("*.pcap")) + \
            sorted(pcap_dir.glob("*.pcap*"))
        pcap_files = list(dict.fromkeys(pcap_files))
        # 29.1 修订（2026-11-26）：只拿本会话时间窗内的抓包文件。
        # 环里会同时留着 tcpdump 的老文件（可能是几天前的）和执行器自己的转写
        # （runs/_pcap_ring/ecb_tee_*.pcap）；不筛的话时间窗对不上、越跑越乱。
        try:
            import datetime as _dt
            lo_ts = _dt.datetime.fromisoformat(str(meta.get("started"))).timestamp() - 60.0
        except Exception:  # noqa: BLE001
            lo_ts = None
        # 上限不做过滤：环文件是滚动写的，mtime 是"最后写"的时刻，
        # 一场会话最后一个文件可能比会话结束晚十几分钟；按"包时刻"在下面筛就够了
        kept = []
        for f in pcap_files:
            try:
                mtime = f.stat().st_mtime
            except Exception:  # noqa: BLE001
                kept.append(f)
                continue
            if lo_ts is not None and mtime < lo_ts:
                continue
            kept.append(f)
        pcap_files = kept

    source = "mock"
    packets = []
    if pcap_files:
        from arm import pcapio
        source = f"pcap {len(pcap_files)} 个文件"
        try:
            frames = pcapio.iter_write_frames(pcap_files, ecb_ip=args.ecb)
        except Exception as exc:  # noqa: BLE001
            print(f"读抓包失败：{exc}", file=sys.stderr)
            frames = []
        for ts, addr, cmd, data_hex, sport in frames:
            rel = (ts - start_ts) if start_ts else None
            packets.append({"rel": rel, "addr": addr, "cmd": cmd,
                            "data": data_hex, "sport": sport})
    else:
        for w in load_mock_writes(d):
            packets.append({"rel": None, "addr": None, "cmd": int(w.get("cmd", -1)),
                            "data": str(w.get("value")), "sport": -1,
                            "src": w.get("src", "executor"),
                            "mono": w.get("mono")})

    # 36.5：按源端口分组；一个执行器可能有两个口（0x0A 的 send_now 单独一口）
    port_table = {}
    for p in packets:
        if p.get("sport") is None or p["sport"] < 0:
            continue
        row = port_table.setdefault(p["sport"], {"first": None, "last": None, "n": 0})
        row["n"] += 1
        r = p.get("rel")
        if r is None:
            continue
        if row["first"] is None or r < row["first"]:
            row["first"] = r
        if row["last"] is None or r > row["last"]:
            row["last"] = r

    exec_ports = set()
    if pcap_files and start_ts and port_table:
        lo_w = start_ts - 1.0
        hi_w = (end_ts if end_ts else start_ts + 1e9) + 2.0
        for sport, row in port_table.items():
            f = None if row["first"] is None else start_ts + row["first"]
            l = None if row["last"] is None else start_ts + row["last"]
            if f is not None and l is not None and l >= lo_w and f <= hi_w:
                exec_ports.add(sport)
        packets_for_match = [p for p in packets if p.get("sport") in exec_ports]
    else:
        packets_for_match = packets

    # ---- 1) events 里的写命令，抓包里有没有 ----
    missing_in_pcap = []
    if pcap_files:
        used = set()
        for e in writes_ev:
            cmd = int(e.get("cmd", -1))
            val = str(e.get("value"))
            lo = (e.get("mono") or 0.0) - args.tol_ms / 1000.0
            hi = (e.get("mono") or 0.0) + args.tol_ms / 1000.0
            hit = None
            for i, p in enumerate(packets_for_match):
                if i in used or p["cmd"] != cmd:
                    continue
                if p.get("rel") is not None and not (lo - 1.0 <= p["rel"] <= hi + 1.0):
                    continue
                hit = i
                break
            if hit is None:
                missing_in_pcap.append(e)
            else:
                used.add(hit)
    else:
        # mock：两边的时钟不同（events 是会话内相对秒，mock 日志是绝对 monotonic），
        # 按“顺序 + cmd + 值”对，不按时间
        used = set()
        j = 0
        for e in writes_ev:
            cmd = int(e.get("cmd", -1))
            val = str(e.get("value"))
            hit = None
            for i in range(j, len(packets)):
                if i in used or packets[i]["cmd"] != cmd:
                    continue
                if str(packets[i]["data"]) != val:
                    continue
                hit = i
                break
            if hit is None:
                # 再回头找一遍（允许跨过别的关节）
                for i, p in enumerate(packets):
                    if i in used or p["cmd"] != cmd or str(p["data"]) != val:
                        continue
                    hit = i
                    break
            if hit is None:
                missing_in_pcap.append(e)
            else:
                used.add(hit)
                j = hit + 1

    # ---- 2) 抓包里的写命令，events 里有没有 ----
    missing_in_events = [p for i, p in enumerate(packets_for_match) if i not in used]

    # ---- 3) 不是执行器发的 ----
    foreign = []
    if pcap_files:
        # 只报本会话时间窗里、源端口不属于本会话的包（别的会话的包不混进来）
        end_rel = None
        if start_ts and end_ts:
            end_rel = end_ts - start_ts
        foreign = [p for p in packets
                   if p.get("sport") not in exec_ports
                   and p.get("rel") is not None
                   and p["rel"] >= -5.0
                   and (end_rel is None or p["rel"] <= end_rel + 5.0)]
    else:
        foreign = [p for p in packets if p.get("src") != "executor"]

    print(f"# 会话 {d.name}；对账源：{source}")
    if port_table:
        print("# 36.5 按源端口分组：")
        for sport, row in sorted(port_table.items(), key=lambda kv: kv[1]["first"] or 0):
            f = "?" if row["first"] is None else f"{row['first']:.2f}"
            l = "?" if row["last"] is None else f"{row['last']:.2f}"
            mark = "本会话" if sport in exec_ports else "不属于本会话"
            print(f"  端口 {sport}：{row['n']} 条，{f}s → {l}s（{mark}）")
    print(f"# events 写命令 {len(writes_ev)} 条；抓包写命令 {len(packets_for_match)} 条"
          f"（本会话端口 {len(exec_ports)} 个）；窗口 ±{args.tol_ms:g} ms")
    st = sign_stats_0x21(packets_for_match)
    extra = ""
    if st["zero"]:
        extra += f"，零 {st['zero']}"
    if st["unknown"]:
        extra += f"，解不出符号 {st['unknown']}"
    print(f"# 0x21 写指令符号（37.2）：总 {st['total']} 条，负数 {st['negative']}，"
          f"正数 {st['positive']}{extra}（应全负）")
    print(f"执行器记了但没发出去：{len(missing_in_pcap)}")
    for e in missing_in_pcap[:20]:
        print(f"  - {e.get('mono')}s {e.get('joint')} cmd=0x{int(e.get('cmd', 0)):02X} "
              f"值 {e.get('value')}：{str(e.get('text'))[:80]}")
    print(f"发出去了但执行器没记：{len(missing_in_events)}")
    for p in missing_in_events[:20]:
        print(f"  - {p.get('cmd')} addr={p.get('addr')} 数据 {p.get('data')}")
    if missing_in_events:
        kinds = {e.get("kind") for e in events}
        notes = []
        if all(p.get("cmd") == 0x0A for p in missing_in_events):
            if "takeover" in kinds:
                notes.append("接管：30.6 接管只发 0x0A，事件里记 takeover 不是 cmd")
            if "test_write_targets" in kinds:
                notes.append("测试写目标：/api/test/write_targets 的 0x0A 记成 test_write_targets，不是 cmd")
        if notes:
            print("  说明：" + "；".join(notes))
    print(f"不是执行器发的：{len(foreign)}")
    for p in foreign[:20]:
        print(f"  - 源 {'端口 ' + str(p.get('sport')) if p.get('sport') is not None else p.get('src')}"
              f" cmd=0x{p.get('cmd', 0):02X} 数据 {p.get('data')}")
    ok = not missing_in_pcap and not missing_in_events
    print("结果：", "对得上" if ok else "有对不上的")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
