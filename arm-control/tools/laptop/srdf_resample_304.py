#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.4 第 3 条：几何改了以后，按采样重生成 SRDF 的禁碰对。

做法：对当前 SRDF 里“没有禁用”的每一对 link，用 N 个随机关节状态
调 /check_state_validity，统计哪些对真的撞过。
  * 一次都没撞过的 → 建议加进 disable_collisions（reason="Never"，省时间）；
  * 撞过的 → 保留检查（真有可能碰）。
输出：~/gluon_ws/runs/30_4_srdf_resample.json
"""
import itertools
import json
import random
import re
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

from moveit_test_common import MoveItCheck, JOINTS

SRDF = "/home/frank/gluon_ws/src/gluon_moveit_config/config/gluon_2l6.srdf"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 10000
SEED = 20261121

links = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6",
         "flange", "gripper_center"]
pairs_all = [tuple(sorted(p)) for p in itertools.combinations(links, 2)]

txt = open(SRDF, encoding="utf-8").read()
disabled = set()
for m in re.finditer('<disable_collisions link1="([^"]+)" link2="([^"]+)"', txt):
    disabled.add(tuple(sorted((m.group(1), m.group(2)))))
enabled = [p for p in pairs_all if p not in disabled]

# 关节限位（执行器安全限位，和 joint_limits.yaml 同量级）
LIM = [(-2.3213, 2.3213), (-1.5359, 1.5359), (-2.3213, 2.3213),
       (-2.3213, 2.3213), (-2.3213, 2.3213), (-2.8449, 2.8449)]

rclpy.init()
node = MoveItCheck("srdf_resample_304")
assert node.wait_ready(), "move_group not ready"
random.seed(SEED)
hits = {p: 0 for p in enabled}
n_valid = 0


def match(name: str):
    for L in links:
        if name == L or name.startswith(L):
            return L
    return None


for i in range(N):
    q = [random.uniform(lo, hi) for lo, hi in LIM]
    req = GetStateValidity.Request()
    req.group_name = "arm"
    rs = RobotState()
    js = JointState()
    js.name = list(JOINTS)
    js.position = q
    rs.joint_state = js
    req.robot_state = rs
    fut = node.valid_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None:
        continue
    if res.valid:
        n_valid += 1
    for c in res.contacts:
        m1 = match(c.contact_body_1)
        m2 = match(c.contact_body_2)
        if not m1 or not m2:
            continue
        key = tuple(sorted((m1, m2)))
        if key in hits:
            hits[key] += 1

never = sorted([p for p, c in hits.items() if c == 0])
ever = sorted([p for p, c in hits.items() if c > 0])
out = {
    "n_samples": N, "n_valid": n_valid, "seed": SEED,
    "disabled_before": len(disabled),
    "enabled_before": len(enabled),
    "enabled_ever_collided": [[list(p), hits[p]] for p in ever],
    "enabled_never_collided": [list(p) for p in never],
    "suggested_disabled": sorted([list(p) for p in (disabled | set(never))]),
}
json.dump(out, open("/home/frank/gluon_ws/runs/30_4_srdf_resample.json", "w"),
          ensure_ascii=False, indent=1)
print(f"采样 {N} 组（valid {n_valid}）")
print(f"原来禁用 {len(disabled)} 对；现在建议禁用 {len(out['suggested_disabled'])} 对")
print("撞过的（保留检查）：")
for p in ever:
    print("   ", list(p), hits[p])
print("从没撞过的（建议加进 Never）：")
for p in never:
    print("   ", list(p))
node.destroy_node()
rclpy.shutdown()
