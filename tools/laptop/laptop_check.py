#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十八节：笔记本（执行器主机）的配置检查和电脑端新配置生成。**只读，不碰真臂。**

三个模式：

1. 默认（在任何一台机器上跑）——列出当前这台机器的 28 节相关状态：
   主机名、各网口地址、默认路由、`ip_forward`、执行器配置里的
   `ecb_ip / mock_on_start / bind_host`、`park_state.json` 里记的主机名、
   合盖 / 自动更新 / 时间同步 / 防火墙 / 远程桌面监听地址。

2. `--gen <文件.json>` —— 生成笔记本要用的执行器配置（**不写进仓库**）：
   把 `config/executor.json` 拷一份，强制成 28.3 第 2 条的样子：
   `ecb_ip` 留空、`mock_on_start` 打开、`bind_host` 改成 28.1 表里的地址。
   生成时打印每一项改了什么。生成的文件要放到笔记本的 `arm-control/config/`
   **外面**（比如 `~/28_generated/`），执行器用 `--config` 指过去，这样
   `git status` 在笔记本上仍然是干净的。

3. `--compare <文件> --base <config/executor.json>` —— 比对两份配置，
   只报 28 节相关的项（`ecb_ip`、`mock_on_start`、`bind_host`、`bind_port`），
   用来在部署后确认笔记本上跑的是"只连 mock"的那份。

为什么要有这个工具：28.4 那天要在笔记本上把 `ecb_ip` 从空改回真地址。
改仓库里的文件会留一个未提交改动，正好和 29.1 第 4 条"会话头写有没有未提交
的改动"混在一起。"配置在仓库外 + 只有 28 节相关的几项不同"是更容易核的。
"""
from __future__ import annotations

import argparse
import json
import platform
import shutil
import socket
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
CFG_DIR = ROOT / "config"

# 28.1 表里的地址（PC 原文；28.1 本身暂缓，这里只作为生成配置时的默认值）
ADDR_LAPTOP_SWITCH = "192.168.2.20"
ADDR_LAPTOP_ECB = "192.168.1.20"
ADDR_MAC_USB = "192.168.2.111"
ECB_ADDR = "192.168.1.30"

# 28 节关心的执行器配置项
KEYS_28 = ("ecb_ip", "ecb_udp_port", "mock_on_start", "bind_host", "bind_port")


def sh(cmd: list[str]) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=8).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f"<失败 {e}>"


def read_json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception as e:  # noqa: BLE001
        return {"_error": str(e)}


def host_facts() -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    rows.append(("主机名", socket.gethostname()))
    rows.append(("系统", f"{platform.system()} {platform.release()} {platform.machine()}"))

    # 网口和地址
    if platform.system() == "Linux":
        out = sh(["ip", "-4", "-o", "addr", "show"])
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4:
                iface = parts[1]
                cidr = parts[3]
                link = sh(["cat", f"/sys/class/net/{iface}/speed"])
                rows.append((f"网口 {iface}", f"{cidr}  速度 {link or '—'} Mb/s"))
        route = sh(["ip", "route", "show", "default"])
        rows.append(("默认路由", route or "（没有）"))
        rows.append(("ip_forward", sh(["cat", "/proc/sys/net/ipv4/ip_forward"]) or "?"))

        # 远程桌面监听地址（28.1 第 3 条：改成只监听交换机网口）
        vnc = sh(["bash", "-lc",
                  "ps -eo args | grep '[x]11vnc' | head -1"])
        listen = ""
        if vnc:
            for flag in ("-localhost", "-listen"):
                if flag in vnc:
                    listen = flag
        rows.append(("x11vnc", (vnc[:120] if vnc else "没在跑")
                     + (f"  （{listen}）" if vnc else "")))

        lid = ""
        for p in ("/etc/systemd/logind.conf",
                  "/etc/systemd/logind.conf.d/90-gluon-lid.conf"):
            try:
                for line in Path(p).read_text().splitlines():
                    if line.startswith("HandleLidSwitch"):
                        lid += line + " "
            except FileNotFoundError:
                pass
        rows.append(("合盖不休眠", lid or "没找到（HandleLidSwitch 没设）"))

        rows.append(("时间同步", sh(["timedatectl", "show", "-p",
                                    "NTP", "-p", "NTPSynchronized"]).replace("\n", " ")))
        uua = sh(["systemctl", "is-enabled", "unattended-upgrades"])
        rows.append(("自动更新", f"unattended-upgrades 服务 {uua or '?'}，"
                                f"apt-daily.timer {sh(['systemctl', 'is-enabled', 'apt-daily.timer']) or '?'}"))
        rows.append(("防火墙", sh(["bash", "-lc",
                                 "sudo -n ufw status 2>&1 | head -20"]) or "?"))
    else:
        # macOS
        route = sh(["netstat", "-rn", "-f", "inet"])
        for line in route.splitlines():
            if line.startswith("default"):
                rows.append(("默认路由", line.split()[1:]))
                break
        for line in sh(["ifconfig"]).splitlines():
            s = line.strip()
            if s.startswith("inet "):
                rows.append(("地址", s.split()[1:2]))
        rows.append(("（28.1 的 ip_forward / ufw 是笔记本侧的检查）", "—"))
    return rows


def cfg_rows(cfg_path: Path) -> list[tuple[str, str]]:
    cfg = read_json(cfg_path)
    if "_error" in cfg:
        return [(f"执行器配置 {cfg_path}", f"读不出：{cfg['_error']}")]
    rows = [(f"配置 {cfg_path.name}.{k}", repr(cfg.get(k))) for k in KEYS_28]
    return rows


def last_session_host() -> str:
    """runs 里最后一个会话的 meta/host，用来核对部署以后真跑过的是哪台机器。"""
    runs = sorted((ROOT / "runs").glob("*/meta.json"))
    if not runs:
        return "（runs 里没有会话）"
    m = read_json(runs[-1])
    return f"{runs[-1].parent.name} host={m.get('host')} connector={m.get('connector')}"


def cmd_status(args) -> int:
    cfg_path = Path(args.config) if args.config else CFG_DIR / "executor.json"
    print(f"== 第二十八节检查（{ROOT}）==")
    width = max(len(k) for k, _ in host_facts() + cfg_rows(cfg_path))
    for k, v in host_facts() + cfg_rows(cfg_path):
        print(f"  {k.ljust(width)} = {v}")
    print(f"  {'最近会话'.ljust(width)} = {last_session_host()}")
    return 0


def _load_cfg(path: Path) -> dict:
    cfg = read_json(path)
    if "_error" in cfg:
        raise SystemExit(f"读不出 {path}：{cfg['_error']}")
    return cfg


def cmd_gen(args) -> int:
    base = _load_cfg(Path(args.base) if args.base else CFG_DIR / "executor.json")
    out = Path(args.gen)
    new = dict(base)
    new["_comment"] = ("28.3：笔记本上执行器只连 mock（ecb_ip 空、mock_on_start 打开）。"
                       "这份文件在仓库外面，由 tools/laptop/laptop_setup.sh "
                       "写出来；执行器用 --config 指过来。")
    new["ecb_ip"] = ""
    new["mock_on_start"] = True
    new["bind_host"] = args.bind_host
    print(f"从 {args.base or CFG_DIR / 'executor.json'} 生成 {out}")
    for k in KEYS_28:
        old, cur = base.get(k), new.get(k)
        flag = "  " if old == cur else "改"
        print(f"  {flag} {k.ljust(14)} {old!r} -> {cur!r}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(new, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    print(f"已写 {out}（不在仓库里，git status 不受影响）")
    return 0


def cmd_compare(args) -> int:
    a = _load_cfg(Path(args.compare))
    b = _load_cfg(Path(args.base) if args.base else CFG_DIR / "executor.json")
    bad = 0
    for k in KEYS_28:
        same = a.get(k) == b.get(k)
        print(f"  {'同' if same else '异'} {k.ljust(14)} A={a.get(k)!r}  B={b.get(k)!r}")
        if not same:
            bad += 1
    # 28.3 第 2 条的硬条件
    ok_mock = a.get("mock_on_start") is True and not a.get("ecb_ip")
    print(f"28.3 第 2 条（只连 mock：ecb_ip 空 且 mock_on_start 真）："
          f"{'✓ 满足' if ok_mock else '✗ 不满足'}")
    if args.require_mock and not ok_mock:
        return 3
    return 0 if not args.require_mock or bad == 0 else 2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="28 节：笔记本配置检查 / 生成（只读，不碰真臂）")
    ap.add_argument("--config", help="要检查的执行器配置（默认 config/executor.json）")
    ap.add_argument("--gen", help="生成笔记本用的执行器配置到这个路径")
    ap.add_argument("--base", help="生成/比对时的基准配置")
    ap.add_argument("--bind-host", default=ADDR_LAPTOP_SWITCH,
                    help=f"生成时 bind_host（默认 {ADDR_LAPTOP_SWITCH}，28.1 表）")
    ap.add_argument("--compare", help="比对这份配置和 --base")
    ap.add_argument("--require-mock", action="store_true",
                    help="比对时，不满足 28.3 第 2 条就返回非 0")
    args = ap.parse_args(argv)
    if args.gen:
        return cmd_gen(args)
    if args.compare:
        return cmd_compare(args)
    return cmd_status(args)


if __name__ == "__main__":
    raise SystemExit(main())
