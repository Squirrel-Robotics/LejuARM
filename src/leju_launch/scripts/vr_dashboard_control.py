#!/usr/bin/env python3
"""Enqueue a recording/task request; no robot command publisher is created."""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid
from vr_workbench_support import atomic_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, default=Path('/home/lab/.cache/lejulab/vr_transfer'))
    args = p.parse_args()
    data = json.loads(sys.stdin.buffer.read(16385))
    if data.get('action') not in ('start', 'stop', 'task'):
        raise ValueError('Unsupported operation')
    status = json.loads((args.root/'vr_mcap_status.json').read_text())
    if time.time_ns()-status['updated_ns'] > 3_000_000_000 or status['state']=='stopped':
        raise ValueError('录制监听器未运行或心跳已过期')
    ident = data.get('request_id') or uuid.uuid4().hex
    if len(ident)!=32 or any(x not in '0123456789abcdef' for x in ident):
        raise ValueError('Invalid request ID')
    data['created_ns'] = time.time_ns()
    request = args.root/'dashboard/requests'/(ident+'.json')
    reply = args.root/'dashboard/responses'/(ident+'.json')
    if not reply.exists():
        atomic_json(request, data)
    deadline = time.monotonic()+6
    while time.monotonic()<deadline:
        if reply.exists():
            print(reply.read_text(), end='')
            return
        time.sleep(.02)
    # Do not let an unseen request execute later after a disconnected caller.
    request.unlink(missing_ok=True)
    print(json.dumps({'ok':False,'error':'下位机未及时确认，请检查实际状态后再操作'}))


if __name__=='__main__':
    try:
        main()
    except Exception as exc:
        print(json.dumps({'ok':False,'error':str(exc)}, ensure_ascii=False))
