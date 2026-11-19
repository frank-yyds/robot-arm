#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""26.5 第 3 条 / A7：mock 注入“第一次使能无应答、第二次有”和“两次都无”各 20 次。"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm.joints import JointBus  # noqa: E402


class Fake:
    def __init__(self, fail_times):
        self.fail_times = fail_times
        self.calls = 0

    def enable(self):
        self.calls += 1
        return self.calls > self.fail_times


def main():
    ok_first = 0
    ok_both = 0
    for _ in range(20):
        report = {}
        m = Fake(1)
        got = JointBus._enable_with_retry(m, "J5", report)
        if got and report.get("enable_retry") == ["J5"] and m.calls == 2:
            ok_first += 1
    for _ in range(20):
        report = {}
        m = Fake(99)
        got = JointBus._enable_with_retry(m, "J5", report)
        if (not got) and report.get("enable_retry") == ["J5"] and m.calls == 2:
            ok_both += 1
    print(f"  {'过' if ok_first==20 else '没过'}：第一次没应答、第二次有  {ok_first}/20（通过并点名重试）")
    print(f"  {'过' if ok_both==20 else '没过'}：两次都无应答  {ok_both}/20（判失败并点名）")
    ok = ok_first == ok_both == 20
    print("结果：", "过" if ok else "没过")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
