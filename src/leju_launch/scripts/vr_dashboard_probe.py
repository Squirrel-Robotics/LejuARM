"""Read-only dashboard snapshot; never starts hardware or writes DDS."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import time
from vr_cache_cleanup import eligibility, pending_deletions

CACHE = Path('/home/lab/.cache/lejulab/vr_transfer')
SESSION = re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})?\Z')


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def command(pid):
    try:
        return [x.decode(errors='replace') for x in (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0') if x]
    except (OSError, ValueError):
        return []


def snapshot(cache=CACHE):
    now = time.time_ns()
    status = read_json(cache / 'vr_mcap_status.json')
    args = command(status.get('pid')) if status else []
    listener_live = any(a.endswith('/vr_mcap_recorder.py') for a in args)
    processes = {'pico': [], 'hardware': [], 'controller': []}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        args = command(p.name)
        if '--domain-id' in args:
            i = args.index('--domain-id')
            if i+1 < len(args) and args[i+1] != '0':
                continue
        for name, matched in [('pico', 'leju_pico_gmr.main' in args),
                              ('hardware', any(a == 'leju-hardware' or a.endswith('/leju-hardware') for a in args)),
                              ('controller', any(a == 'run_rl_controller' or a.endswith('/run_rl_controller') for a in args))]:
            if matched:
                processes[name].append(int(p.name))
    pending = []
    cache_media = 0
    if cache.exists():
        for p in cache.iterdir():
            if not p.is_dir() or p.is_symlink():
                continue
            if p.name.startswith('.deleting_'):
                pending.append({'session_id': p.name, 'state': 'deleting', 'published': False})
                continue
            if not SESSION.fullmatch(p.name):
                continue
            meta_files = list((p / 'Action_ObservationState').glob('*.json'))
            metadata = read_json(meta_files[0]) if len(meta_files) == 1 else None
            manifest = read_json(p / 'transfer_manifest.json')
            identity = (metadata or {}).get('name')
            can_delete, delete_reason = eligibility(cache, p.name, identity)
            pending.append({'session_id': p.name, 'can_delete_cache': can_delete, 'cache_delete_reason': delete_reason, 'recording_id': (metadata or {}).get('name'),
                'state': (metadata or {}).get('state', 'unknown'), 'published': bool(manifest),
                'bytes': sum(x['size'] for x in manifest.get('files', {}).values()) if manifest else None,
                'error': (metadata or {}).get('vision', {}).get('error') or (metadata or {}).get('transfer_error'),
                'start_button_ns': (metadata or {}).get('start_button_ns'),
                'recorder_ready_ns': (metadata or {}).get('recorder_ready_ns'),
                'stop_reason': (metadata or {}).get('stop_reason'),
                'stop_boundary_ns': (metadata or {}).get('stop_boundary_ns')})
        for job in pending_deletions(cache):
            previous = next((x for x in pending if x.get('recording_id') == job.get('recording_id')), {})
            pending = [x for x in pending if x.get('recording_id') != job.get('recording_id')]
            pending.append(dict(previous, **job))
        cache_media = sum(1 for pattern in ['*.mcap', '*.mp4'] for p in cache.rglob(pattern) if p.is_file())
    disk = shutil.disk_usage(cache if cache.exists() else cache.parent.parent)
    return {'schema_version': 1, 'now_ns': now, 'listener_live': listener_live,
            'status': status, 'processes': processes, 'pending_sessions': pending,
            'cache_media_files': cache_media, 'disk': {'free': disk.free, 'total': disk.total}}


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=CACHE);args=p.parse_args()
    print(json.dumps(snapshot(args.root), ensure_ascii=False, allow_nan=False))
