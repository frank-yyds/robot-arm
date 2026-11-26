#!/bin/bash
# 第二十八节：在笔记本上（从 Mac 上 ssh 过去跑）准备和自检"执行器搬到笔记本"。
#
# 只做 28.1–28.4 里**不碰真臂**的部分，一条命令一件事。所有条目只做软件能做的
# 那一段；要用户的、要 PC 重出原文的、要 G0 过关的，都不做，只在 check 里报状态。
#
#   laptop_setup.sh 281              28.1：只读检查 ip_forward、防火墙、地址现状。
#                                    **不改任何地址**（28.1 原文暂缓，用户 11-20 定"保持现状"）；
#                                    ufw 规则清单只打印，不执行。
#   laptop_setup.sh 282              28.2 第 2 条"你做"的三项（可重复跑）：
#                                    · 合盖不休眠：写 /etc/systemd/logind.conf.d/90-gluon-lid.conf
#                                      （不动发行版自带的 logind.conf）。重启 systemd-logind 会结束
#                                      当前图形会话、x11vnc 跟着没；默认**不**重启，打印"要重启才生效"，
#                                      加 --restart-logind 才重启（会先停 user@.service、做完再拉起并核对）。
#                                    · 关自动更新：mask unattended-upgrades 服务 + apt-daily /
#                                      apt-daily-upgrade 两个服务**和两个 timer**。
#                                      （11-24 只 mask 了服务，timer 还在按天跑 apt-get update，
#                                        和"只在臂断电时手动更新"不一致 —— 见回报"需要 PC 定"。）
#                                    · 时间同步：set-ntp true，等同步再报。
#                                    · 28.4 第 5 条要用 Mac 的地址：/etc/hosts 里加 gluon-mac。
#   laptop_setup.sh 283-deploy       28.3 第 1 条：用 git 把 arm-control 部署到笔记本，
#                                    **checkout 到 Mac 上打的 tag**（branch 名两边不用一样），
#                                    核对提交号一致。第一次笔记本上没仓库时，自动先做一次
#                                    rsync 引导（只拷受版本控制的文件以外的那些 + 建 .git）。
#                                    用法：283-deploy <源码目录> 或 283-deploy --tag <tag> [<源码目录>]
#   laptop_setup.sh 283-config       28.3 第 2 条：生成**仓库外**的只连 mock 的配置
#                                    （~/28_generated/executor_mock.json），并核对满足条件。
#   laptop_setup.sh 283-selfcheck    28.3 第 3 条里的基础项：Python、依赖（只用标准库？）、
#                                    已部署的代码和 --src 指的一份逐字节一致。
#   laptop_setup.sh 284-preflight    28.4 的**准备**：28.1 表里各口地址/速度的检查命令怎么跑、
#                                    只读通信测试（26.7）要抓的数、回滚到 Mac 的步骤。只打印，不动。
#   laptop_setup.sh check            默认：以上各项的现状，一屏看完（只读）。
#   laptop_setup.sh --ufw-plan       28.1 第 4 条的 ufw 规则清单（只打印）
#
# 红线：本脚本没有任何一步会连 ECB。283-config 生成出来的 ecb_ip 为空、mock_on_start 为真；
#       281 / check 只读 /proc、sudo -n ufw status、git 状态。
set -u
ROOT=$(cd "$(dirname "$0")/../.." && pwd)     # 笔记本上的 arm-control
CFG="$ROOT/config/executor.json"
GEN_DIR="${GEN_DIR:-$HOME/28_generated}"
MAC_IP="${MAC_IP:-192.168.2.111}"
SRC="${SRC:-$HOME/robot-arm/arm-control}"

say() { printf '%s\n' "$*"; }
ok()  { printf '✓ %s\n' "$*"; }
bad() { printf '✗ %s\n' "$*"; }

user_unit() {
  XDG_RUNTIME_DIR=/run/user/$(id -u) \
  DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus \
  systemctl --user "$@"
}

# ---------------------------------------------------------------- 28.1（只检查）
ufw_plan() {
  cat <<'EOF'
    28.1 第 4 条原文：先放行 SSH（来源 192.168.2.0/24、192.168.3.0/24），再启用；
    5900 和以后执行器的网页端口只放行 192.168.2.0/24。对应命令（**现在不执行**）：
      sudo ufw default deny incoming
      sudo ufw default allow outgoing
      sudo ufw allow from 192.168.2.0/24 to any port 22 proto tcp
      sudo ufw allow from 192.168.3.0/24 to any port 22 proto tcp
      sudo ufw enable
      sudo ufw allow from 192.168.2.0/24 to any port 5900 proto tcp
      sudo ufw allow from 192.168.2.0/24 to any port <执行器端口> proto tcp
    先放 SSH 再 enable 的顺序是对的（否则 enable 那一刻就把自己关在外面）。
    但 28.1 第 3 条要把 x11vnc 改成监听 192.168.2.20，等于把 5900 暴露给交换机网段；
    现在是 -localhost（只给 SSH 隧道、一次一个），要不要改见回报"需要 PC 定"。
EOF
}

cmd_281() {
  say "== 28.1 现状（28.1 暂缓：不改地址、不启用防火墙）=="
  fwd=$(cat /proc/sys/net/ipv4/ip_forward)
  [ "$fwd" = 0 ] && ok "ip_forward = 0" || bad "ip_forward = $fwd（28.1 第 2 条要求 0）"
  say "  持久化：$(grep -rhE '^net\.ipv4\.ip_forward' /etc/sysctl.conf /etc/sysctl.d/ 2>/dev/null | tr '\n' ' ' || echo '（没有配置文件，默认 0）')"
  say "-- 网口和地址："
  ip -4 -o addr show | awk '{printf "  %-10s %s\n", $2, $4}'
  say "-- 默认路由：$(ip route show default)"
  say "-- ufw："
  sudo -n ufw status verbose 2>&1 | sed 's/^/  /'
  say "-- ufw 规则清单（只打印）："
  ufw_plan
}

# ---------------------------------------------------------------- 28.2
cmd_282() {
  say "== 28.2 第 2 条（agent 侧）=="
  say "-- 合盖不休眠"
  sudo -n install -d -m 755 /etc/systemd/logind.conf.d \
    || { bad "写不了 /etc/systemd/logind.conf.d"; return 1; }
  printf '[Login]\n# 28.2 第 2 条：合盖不休眠（笔记本当执行器主机，合盖不能睡）\nHandleLidSwitch=ignore\nHandleLidSwitchExternalPower=ignore\nHandleLidSwitchDocked=ignore\n' \
    | sudo -n tee /etc/systemd/logind.conf.d/90-gluon-lid.conf >/dev/null
  ok "已写 /etc/systemd/logind.conf.d/90-gluon-lid.conf"
  say "  （这是 systemd 24x 起的 drop-in，改完要重启 systemd-logind 才生效）"
  if [ "${RESTART_LOGIND:-0}" = 1 ]; then
    others=$(loginctl list-sessions --no-legend 2>/dev/null | awk '{print $3}' \
             | sort -u | grep -v "^$(id -un)$" | grep -v '^$' || true)
    if [ -n "$others" ]; then
      bad "还有别的用户登录（$others），不重启 systemd-logind（会踢掉他们的会话）"
    else
      vnc_was=no
      user_unit is-active --quiet x11vnc.service && vnc_was=yes
      say "  重启 systemd-logind（当前图形会话会结束，x11vnc 会停；重启后拉起）"
      sudo -n systemctl restart systemd-logind
      sleep 3
      if [ "$vnc_was" = yes ]; then
        for _ in $(seq 1 20); do
          user_unit is-active --quiet x11vnc.service && break
          sleep 1
        done
      fi
      if user_unit is-active --quiet x11vnc.service; then
        ok "systemd-logind 已重启，x11vnc 已恢复"
      else
        bad "systemd-logind 已重启，x11vnc 没恢复。请用户在新会话里执行："
        say "     export XDG_RUNTIME_DIR=/run/user/$(id -u); systemctl --user start x11vnc.service"
      fi
    fi
  else
    say "  没有 --restart-logind，没重启 systemd-logind（下次重启笔记本后生效）"
  fi
  say "  当前生效值：$(systemd-analyze cat-config systemd/logind.conf 2>/dev/null | grep -E '^HandleLidSwitch' | tr '\n' ' ')"

  say "-- 关自动更新（28.2 第 2 条：以后系统更新只在臂断电时手动做）"
  sudo -n systemctl mask --now unattended-upgrades.service >/dev/null 2>&1
  sudo -n systemctl mask --now apt-daily.service apt-daily-upgrade.service \
       apt-daily.timer apt-daily-upgrade.timer >/dev/null 2>&1
  for u in unattended-upgrades.service apt-daily.service apt-daily-upgrade.service \
           apt-daily.timer apt-daily-upgrade.timer; do
    st=$(systemctl is-enabled "$u" 2>&1); act=$(systemctl is-active "$u" 2>&1)
    if [ "$st" = masked ] && [ "$act" != active ]; then ok "$u：$st / ${act}"
    else bad "$u：$st / ${act}"; fi
  done
  say "  /etc/apt/apt.conf.d/20auto-upgrades：$(tr '\n' ' ' < /etc/apt/apt.conf.d/20auto-upgrades 2>/dev/null)"
  say "  手动更新（臂断电时）：sudo apt update && sudo apt full-upgrade"

  say "-- 时间同步"
  sudo -n timedatectl set-ntp true 2>/dev/null || true
  for _ in $(seq 1 15); do
    [ "$(timedatectl show -p NTPSynchronized --value)" = yes ] && break
    sleep 2
  done
  synced=$(timedatectl show -p NTPSynchronized --value)
  say "  $(timedatectl show -p NTP -p NTPSynchronized -p Timezone --value | tr '\n' ' ')（本地 $(date '+%F %T %Z')）"
  [ "$synced" = yes ] && ok "已同步" || bad "还没同步"

  say "-- /etc/hosts 给 Mac 留个名字（28.1 表：$MAC_IP；28.4 第 5 条从笔记本打开 Mac 上的执行器用）"
  if grep -q "gluon-mac" /etc/hosts; then
    ok "已有：$(grep gluon-mac /etc/hosts | tr '\n' ' ')"
  elif printf '%s\n# 28.1 表里的 Mac USB 网卡地址（28.4 那天才用）\n' \
         | sudo -n tee -a /etc/hosts >/dev/null && \
       printf '%s   gluon-mac\n' "$MAC_IP" | sudo -n tee -a /etc/hosts >/dev/null; then
    ok "已写 /etc/hosts"
  else
    bad "写 /etc/hosts 失败（不影响别的项）"
  fi

  say "-- 28.2 第 1 条是用户在图形界面里做（自动登录 / 不锁屏 / 息屏从不 / 不休眠），现状："
  say "  自动登录：$(grep -hE 'AutomaticLogin|^\[daemon\]' /etc/gdm3/custom.conf 2>/dev/null | tr '\n' ' ' || echo 没记录)"
  say "  锁屏：$(sudo -n -u frank gsettings get org.gnome.desktop.screensaver lock-enabled 2>&1 | head -1)"
  say "  交流电自动挂起：$(sudo -n -u frank gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 2>&1 | head -1)"
}

# ---------------------------------------------------------------- 28.3
bootstrap_repo() {
  local src="$1"
  say "-- 笔记本上还没有 git 仓库，先做一次 rsync 引导（只拷源目录里的工作区内容，跳过 runs/docs/.git）"
  ( cd "$src" && tar -cf - --exclude='./runs' --exclude='./docs' --exclude='./.git' \
      --exclude='__pycache__' --exclude='.DS_Store' --exclude='.ecb.lock' . ) \
    | ( cd "$ROOT" && tar -xf - )
  ( cd "$ROOT" && git init -q -b deploy \
    && git remote add origin "$(git -C "$src" remote get-url origin)" \
    && git add -A \
    && git -c user.name=agent -c user.email=agent@local commit -q \
         -m "引导：rsync 一份 $src 的工作区（第二天起按 tag 部署）" ) \
    || { bad "引导失败"; return 1; }
  ok "引导完成：$ROOT（$(git -C "$ROOT" log --oneline -1)）"
}

cmd_283_deploy() {
  local tag="" src="$SRC"
  while [ $# -gt 0 ]; do
    case "$1" in
      --tag) tag="$2"; shift 2 ;;
      *) src="$1"; shift ;;
    esac
  done
  if [ -z "$tag" ]; then
    tag=$(git -C "$src" tag --points-at HEAD 2>/dev/null | head -1)
    [ -n "$tag" ] || { bad "$src 的 HEAD 没打 tag；先 git tag <名字> 再部署"; return 1; }
  fi
  say "== 28.3 第 1 条：git 部署 $src 的 tag $tag 到 $ROOT =="
  [ -d "$src/.git" ] || { bad "$src 不是 git 仓库"; return 1; }
  if [ ! -d "$ROOT/.git" ]; then bootstrap_repo "$src" || return 1; fi
  local before after mac
  before=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo none)
  mac=$(git -C "$src" rev-parse "$tag^{commit}")
  if git -C "$ROOT" fetch -q --tags origin 2>/dev/null && \
     git -C "$ROOT" rev-parse -q --verify "refs/tags/$tag" >/dev/null 2>&1; then
    git -C "$ROOT" checkout -q "refs/tags/$tag" || { bad "checkout $tag 失败"; return 1; }
  else
    say "  origin 上没有这个 tag（Mac 还没推送，按规矩不推）；从 Mac 直接把 tag 和对象传过来"
    # 把 Mac 的 tag 打包传过来再在本机 fetch。bundle 里带着完整历史，所以笔记本
    # 上的 origin 还是 GitHub（按规矩不推，从 GitHub 也取不到这个 tag）。
    local bundle="$ROOT/.deploy.bundle"
    git -C "$src" bundle create "$bundle.tmp" "$tag" >/dev/null 2>&1 \
      || { bad "生成 bundle 失败（$src）"; rm -f "$bundle.tmp"; return 1; }
    mv -f "$bundle.tmp" "$bundle"
    git -C "$ROOT" fetch -q "$bundle" "refs/tags/$tag:refs/tags/$tag" \
      || { bad "从 bundle 取 tag 失败"; rm -f "$bundle"; return 1; }
    rm -f "$bundle"
    git -C "$ROOT" checkout -q "refs/tags/$tag" || { bad "checkout $tag 失败"; return 1; }
  fi
  after=$(git -C "$ROOT" rev-parse HEAD)
  say "  笔记本：$before -> $after（tag $tag，detached）"
  say "  Mac 上 $tag 指向：$mac"
  [ "$after" = "$mac" ] && ok "提交号一致" || bad "提交号不一致"
  say "  git status：$(git -C "$ROOT" status --porcelain | wc -l) 个改动"
  git -C "$ROOT" status --porcelain | head -10 | sed 's/^/    /'
}

cmd_283_config() {
  say "== 28.3 第 2 条：生成仓库外的只连 mock 配置 =="
  mkdir -p "$GEN_DIR"
  # bind_host 用 127.0.0.1：28.1 表里的 192.168.2.20 要等硬件和 PC 重出 28.1；
  # 28.1 暂缓期间笔记本走 192.168.3.30（用户 11-20 定的临时接法），
  # 执行器只监听回环，远程看网页走 SSH 隧道。
  python3 "$ROOT/tools/laptop/laptop_check.py" --gen "$GEN_DIR/executor_mock.json" \
          --base "$CFG" --bind-host "${BIND_HOST:-127.0.0.1}" || return 1
  python3 "$ROOT/tools/laptop/laptop_check.py" --compare "$GEN_DIR/executor_mock.json" \
          --base "$CFG" --require-mock || { bad "不满足 28.3 第 2 条"; return 1; }
  ok "配置在 $GEN_DIR/executor_mock.json（仓库外，git status 不受影响）"
  python3 -m arm.executor --help 2>&1 | grep -q -- '--config' \
    && ok "执行器支持 --config" \
    || bad "执行器不支持 --config：只能直接改 config/executor.json（会留未提交改动）"
}

cmd_283_selfcheck() {
  local src="${1:-$SRC}"
  say "== 28.3：部署后自检 =="
  say "Python $(python3 -V 2>&1 | cut -d' ' -f2)  CPU $(nproc) 核  内存 $(free -g | awk '/内存|Mem/{print $2}') GB"
  say "已部署：$(git -C "$ROOT" describe --tags --always 2>/dev/null)  "
  python3 - "$ROOT" <<'PY'
import pathlib, sys
root = pathlib.Path(sys.argv[1]); std = set(sys.stdlib_module_names); mods = set()
for p in root.rglob("*.py"):
    if "runs" in p.parts: continue
    try: src = p.read_text(encoding="utf-8")
    except Exception: continue
    for line in src.splitlines():
        s = line.strip()
        if s.startswith("import "): mods.add(s.split()[1].split(".")[0])
        elif s.startswith("from ") and " import " in s: mods.add(s.split()[1].split(".")[0])
print("  第三方包：", sorted(m for m in mods if m not in std and m != "arm") or "（无，全部标准库）")
PY
  if [ -d "$src/.git" ]; then
    say "-- 已部署的代码和 $src 逐字节比对（.py/.js/.html/.css/.json/.sh/.md，跳过 runs/docs/.git）："
    python3 - "$src" "$ROOT" <<'PY'
import hashlib, pathlib, sys
src, dst = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
exts = {".py", ".js", ".html", ".css", ".json", ".sh", ".md"}
def manifest(root):
    out = {}
    for p in root.rglob("*"):
        if not p.is_file() or p.suffix not in exts: continue
        rel = p.relative_to(root)
        if rel.parts and rel.parts[0] in ("runs", "docs", ".git", "__pycache__"): continue
        out[str(rel)] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out
a, b = manifest(src), manifest(dst)
for k in sorted(set(a) | set(b)):
    if a.get(k) != b.get(k):
        print(f"  ✗ {k}: src={(a.get(k) or '-')[:12]} dst={(b.get(k) or '-')[:12]}")
n = sum(1 for k in a if a.get(k) == b.get(k))
print(f"  文件 {len(set(a) | set(b))} 个，一致 {n} 个")
PY
  fi
}

# ---------------------------------------------------------------- 28.4（只准备）
cmd_284_preflight() {
  say "== 28.4 准备清单（28.4 本身不做：要 G0 过关、PC 确认、用户在场）=="
  say "前置条件（逐条对照原文）："
  say "  1. 收敛路线'切换'关：G0 全过、PC 确认，回报在 待Mac确认 Y10/Y12；"
  say "  2. 用户在场（要拔插 ECB 网线、看启动和停机）；"
  say "  3. 交换机 + USB 网卡到货、PC 重出 28.1（现在没到，见 28.1）。"
  say "那天按顺序做（原文）："
  say "  a. Mac 的执行器改 mock；笔记本接 ECB 的口配 192.168.1.20（不设网关）；"
  say "     Mac 的 USB 网卡那一个口同时配 192.168.2.111 和 192.168.1.111（28.1 表）— 现在不做"
  say "  b. ECB 网线从 Mac 拔下 -> 笔记本接 ECB 的口；Mac USB 网卡插回交换机"
  say "  c. 笔记本执行器改成连真 ECB（把 executor_mock.json 的 ecb_ip 从空改成 192.168.1.30、"
  say "     mock_on_start 改成 false；同样在仓库外）"
  say "  d. 只读通信测试（26.7 第 1、3b、8 条；臂在架上、不使能）：六颗都有应答、无应答 0、"
  say "     六颗一轮 ≥100 轮；远程桌面开 + rviz 转的时候再跑一次"
  say "  e. 从搁架启动 -> 抬离 -> 停机回搁架，25% 一次"
  say "  f. Mac 浏览器连笔记本（192.168.2.20 或 SSH 隧道）：按停能停住不失能；关网页 0.5 s 内原地保持；"
  say "     重开能重连拿控制权"
  say "不做过就把网线插回 Mac、Mac 执行器改回真 ECB、回报（原文 28.4 第 6 条）。"
  say ""
  say "现在能先准备的（都只读，不执行）："
  say "-- Mac 那头的 USB 网卡地址（28.1 表）："
  ip -4 -o addr show | awk '$2!="lo" {printf "   %-10s %s\n", $2, $4}'
  say "-- ip_forward：$(cat /proc/sys/net/ipv4/ip_forward)（28.1 第 2 条要求 0）"
  say "-- 28.3 生成的 mock 配置：$(ls -l "$GEN_DIR/executor_mock.json" 2>/dev/null | awk '{print $5" 字节"}' || echo '还没生成（先跑 283-config）')"
}

# ---------------------------------------------------------------- check
cmd_check() {
  python3 "$ROOT/tools/laptop/laptop_check.py" --config "$CFG"
  if [ -f "$GEN_DIR/executor_mock.json" ]; then
    say "--- 仓库外的 mock 配置 $GEN_DIR/executor_mock.json ---"
    python3 "$ROOT/tools/laptop/laptop_check.py" --compare "$GEN_DIR/executor_mock.json" \
            --base "$CFG" --require-mock
  else
    say "（还没有 $GEN_DIR/executor_mock.json，先跑 283-config）"
  fi
  say "--- 部署 ---"
  if [ -d "$ROOT/.git" ]; then
    say "  git：$(git -C "$ROOT" log --oneline -1 2>&1)；改动 $(git -C "$ROOT" status --porcelain | wc -l) 个"
  else
    say "  笔记本上 $ROOT 还不是 git 仓库"
  fi
}

RESTART_LOGIND=0
for a in "$@"; do [ "$a" = "--restart-logind" ] && RESTART_LOGIND=1; done
case "${1:-check}" in
  281|--281)            cmd_281 ;;
  282|--282)            cmd_282 ;;
  283-deploy|--283-deploy) shift; cmd_283_deploy "$@" ;;
  283-config|--283-config) cmd_283_config ;;
  283-selfcheck|--283-selfcheck) shift; cmd_283_selfcheck "${1:-}" ;;
  284-preflight|--284-preflight) cmd_284_preflight ;;
  check|--check|"")      cmd_check ;;
  --ufw-plan)            ufw_plan ;;
  *) echo "用法见文件头（$0 check|281|282|283-deploy|283-config|283-selfcheck|284-preflight）" >&2; exit 64 ;;
esac
