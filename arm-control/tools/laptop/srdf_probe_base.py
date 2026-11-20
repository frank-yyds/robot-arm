#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""30.4 补充：查 base_link 和哪些 link 在随机状态下还会碰，记下碰撞状态。

用法：python3 /tmp/srdf_probe_base.py [N]
输出：~/gluon_ws/runs/30_4_base_contacts.json
"""
import itertools
import json
import math
import random
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

from moveit_test_common import MoveItCheck, JOINTS

N = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
SEED = 20261121
LIM = [(-2.3213, 2.3213), (-1.5359, 1.5359), (-2.3213, 2.3213),
       (-2.3213, 2.3213), (-2.3213, 2.3213), (-2.8449, 2.8449)]
links = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6",
         "flange", "gripper_center"]
pairs = [tuple(sorted(p)) for p in itertools.combinations(links, 2)]
count = {p: 0 for p in pairs}
states = []

rclpy.init()
node = MoveItCheck("srdf_probe_base")
assert node.wait_ready()
random.seed(SEED)
n_valid = 0
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
        def match(name):
            for L in links:
                if name == L or name.startswith(L):
                    return L
            return None
        m1, m2 = match(c.contact_body_1), match(c.contact_body_2)
        if not m1 or not m2:
            continue
        key = tuple(sorted((m1, m2)))
        count[key] = count.get(key, 0) + 1
        if "base_link" in key:
            states.append({"q_deg": [round(math.degrees(v), 1) for v in q],
                           "pair": list(key),
                           "pos": [round(c.position.x, 3),
                                   round(c.position.y, 3),
                                   round(c.position.z, 3)]})
out = {"n_samples": N, "n_valid": n_valid, "seed": SEED,
       "pairs_ever_collided": [[list(p), count[p]] for p in pairs if count[p]],
       "base_link_contacts": states[:50]}
json.dump(out, open("/home/frank/gluon_ws/runs/30_4_base_contacts.json", "w"),
          ensure_ascii=False, indent=1)
print(f"采样 {N}（valid {n_valid}）")
for p in pairs:
    if count[p]:
        print("  ", list(p), count[p])
print("base_link 碰撞状态（最多列 10 个）：")
for s in states[:10]:
    print("   ", s["q_deg"], s["pos"])
node.destroy_node()
rclpy.shutdown()
