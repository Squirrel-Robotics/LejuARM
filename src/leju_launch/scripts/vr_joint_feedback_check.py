#!/usr/bin/env python3
"""Observe joint feedback only. --check gates startup; --watch streams JSON."""
import argparse
import json
import time
from vr_workbench_support import FeedbackHealth, joint_reader


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--domain-id', type=int, default=0)
    p.add_argument('--timeout', type=float, default=12)
    p.add_argument('--watch', action='store_true')
    args = p.parse_args()
    participant, reader, kind = joint_reader(args.domain_id)
    health = FeedbackHealth()
    deadline, next_emit = time.monotonic()+args.timeout, 0
    while args.watch or time.monotonic() < deadline:
        for m in reader.take(N=32):
            if isinstance(m, kind):
                health.observe(m)
        if time.monotonic() >= next_emit:
            s = health.snapshot()
            if args.watch:
                print(json.dumps(s, allow_nan=False), flush=True)
            elif s['ready']:
                print(json.dumps(s, ensure_ascii=False, allow_nan=False), flush=True)
                return 0
            next_emit = time.monotonic()+.2
        time.sleep(.005)
    print(json.dumps(health.snapshot(), ensure_ascii=False, allow_nan=False), flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
