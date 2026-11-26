#!/usr/bin/env bash
# 16.4a 回归全套（20 个自检程序）跑一遍，每个程序一行用时、退出码。
# 用法：cd arm-control && tools/reg16_4a_run.sh [--timeout 900] [--only pat1,pat2]
# 默认 timeout=900 s/程序；trend_guard_check 单独按 1800 s（笔记本上更慢）。
set -u
TIMEOUT=900
ONLY=""
LOG=${LOG:-/tmp/reg16_4a_$(date +%Y%m%d_%H%M%S).log}
while [ $# -gt 0 ]; do
  case "$1" in
    --timeout) TIMEOUT=$2; shift 2;;
    --only) ONLY=$2; shift 2;;
    -h|--help) sed -n '1,8p' "$0"; exit 0;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done
PROGS=(
  kinematics_selfcheck.py gravity_selfcheck.py task20_selfcheck.py
  task21_selfcheck.py task21_4_check.py task22_offline.py
  tomorrow_walkthrough.py a4_clear_check.py a6_latch_check.py
  a7_enable_check.py park_side_check.py hold_fail_check.py
  write_confirm_check.py read_path_check.py readback_tick_check.py
  cvp_check.py park_272_check.py lock_takeover_check.py rec29_check.py
  trend_guard_check.py
)
TALL=0
echo "16.4a runner start $(date +%Y-%m-%dT%H:%M:%S) timeout=${TIMEOUT}s log=$LOG"
for f in "${PROGS[@]}"; do
  if [ -n "$ONLY" ]; then
    case ",$ONLY," in
      *",$f,"*) ;;
      *) continue;;
    esac
  fi
  case "$f" in
    trend_guard_check.py) tmo=${TG_TIMEOUT:-1800};;
    *) tmo=$TIMEOUT;;
  esac
  t0=$(date +%s)
  echo "=== $f start $(date +%H:%M:%S) (timeout ${tmo}s)"
  timeout "$tmo" python3 -u "tools/$f" 2>&1 | tail -n 40
  rc=${PIPESTATUS[0]}
  t1=$(date +%s)
  dt=$((t1 - t0))
  TALL=$((TALL + dt))
  echo "=== $f end rc=$rc elapsed=${dt}s"
done
echo "16.4a runner end $(date +%Y-%m-%dT%H:%M:%S) total_elapsed=${TALL}s"
