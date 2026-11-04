"""Closed-session handoff to the workstation; no DDS or video processing."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import time

DEFAULT_ROOT = Path('/home/lab/.cache/lejulab/vr_transfer')
MANIFEST = 'transfer_manifest.json'
SESSION_RE = re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})?\Z')
IDENTITY_RE = re.compile(r'vr_\d{8}_\d{6}_\d{9}_[0-9a-f]{6}\Z')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_write(path, value):
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)


def publish_session(session, config):
    root = Path(session['session_dir']).resolve()
    if session['state'] != 'closed' or not session.get('mcap_complete') or session.get('exit_code') != 0:
        raise ValueError('Only a normally closed MCAP can be exported')
    if session.get('vision', {}).get('status') != 'complete':
        raise ValueError('Raw RGB is not complete')
    if not SESSION_RE.fullmatch(root.name) or not IDENTITY_RE.fullmatch(session['name']):
        raise ValueError('Invalid recording identity')
    json_write(root / 'processing_config.json', config)
    entries = {}
    for p in sorted(root.rglob('*')):
        if p.is_symlink():
            raise ValueError('Symlinks are not allowed in an export')
        if p.is_file() and p.name != MANIFEST:
            entries[p.relative_to(root).as_posix()] = {'size': p.stat().st_size, 'sha256': digest(p)}
    manifest = {'version': 1, 'recording_id': session['name'], 'session_id': root.name,
                'source_session_dir': str(root), 'processing_location': 'workstation',
                'created_ns': time.time_ns(), 'files': entries}
    json_write(root / MANIFEST, manifest)
    return digest(root / MANIFEST)


def export_path(root, name):
    if not SESSION_RE.fullmatch(name):
        raise ValueError('Invalid session directory')
    path = root / name
    if path.is_symlink() or path.resolve().parent != root:
        raise ValueError('Session escaped transfer cache')
    return path


def scan(root):
    result = []
    for p in sorted(root.iterdir()):
        if not p.is_dir() or p.is_symlink() or not SESSION_RE.fullmatch(p.name):
            continue
        manifest = p / MANIFEST
        if manifest.is_file():
            data = json.loads(manifest.read_text())
            result.append({'session_id': p.name, 'recording_id': data['recording_id'],
                           'manifest_sha256': digest(manifest)})
    return result


def verify_source(path, name, identity, expected_hash):
    if digest(path / MANIFEST) != expected_hash:
        raise ValueError('Manifest changed before acknowledgement')
    manifest = json.loads((path / MANIFEST).read_text())
    if manifest['recording_id'] != identity or manifest['session_id'] != name:
        raise ValueError('Recording changed before acknowledgement')
    members = list(path.rglob('*'))
    if any(p.is_symlink() for p in members):
        raise ValueError('Raw source contains a symlink; refusing deletion')
    actual = {p.relative_to(path).as_posix() for p in members if p.is_file()}
    if actual != set(manifest['files']) | {MANIFEST}:
        raise ValueError('Raw source file list changed; refusing deletion')
    for relative, entry in manifest['files'].items():
        target = path / relative
        if Path(relative).is_absolute() or '..' in Path(relative).parts or target.is_symlink():
            raise ValueError('Unsafe export member')
        if target.stat().st_size != entry['size'] or digest(target) != entry['sha256']:
            raise ValueError('Raw source changed; refusing deletion: ' + relative)


def acknowledge(root, name, identity, expected_hash):
    if not IDENTITY_RE.fullmatch(identity):
        raise ValueError('Invalid recording identity')
    receipts = root / '.acknowledged'
    receipts.mkdir(exist_ok=True)
    receipt = receipts / (identity + '.json')
    path = export_path(root, name)
    if receipt.exists():
        saved = json.loads(receipt.read_text())
        if saved['manifest_sha256'] != expected_hash or saved['session_id'] != name:
            raise ValueError('Acknowledgement identity mismatch')
        # Resume interrupted cleanup, but never remove a new same-minute recording.
        deleting = root / ('.deleting_' + identity)
        if deleting.exists():
            shutil.rmtree(deleting)
        if path.exists() and (path / MANIFEST).exists() and digest(path / MANIFEST) == expected_hash:
            verify_source(path, name, identity, expected_hash)
            path.rename(deleting)
            shutil.rmtree(deleting)
        return dict(saved, already_acknowledged=True)
    verify_source(path, name, identity, expected_hash)
    saved = {'recording_id': identity, 'session_id': name, 'manifest_sha256': expected_hash,
             'acknowledged_ns': time.time_ns(), 'raw_data_removed': True}
    # Persist intent first so a disconnected acknowledgement is safely repeatable.
    json_write(receipt, saved)
    deleting = root / ('.deleting_' + identity)
    path.rename(deleting)
    shutil.rmtree(deleting)
    return saved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('command', choices=['scan', 'manifest', 'ack'])
    parser.add_argument('--session')
    parser.add_argument('--recording-id')
    parser.add_argument('--manifest-sha256')
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.transfer.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.command == 'scan':
            result = scan(root)
        elif args.command == 'manifest':
            result = json.loads((export_path(root, args.session or '') / MANIFEST).read_text())
        else:
            result = acknowledge(root, args.session or '', args.recording_id or '', args.manifest_sha256 or '')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
