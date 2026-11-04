#!/usr/bin/env python3
"""Delete one explicitly selected, unpublished failed recording over authenticated SSH.

Installed on both robots. The lower machine keeps a durable receipt, deletes the
upper cache first, then removes its own cache. No DDS or motor commands are used.
"""
import argparse
from contextlib import contextmanager, ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
import uuid

IDENTITY = re.compile(r'vr_\d{8}_\d{6}_\d{9}_[0-9a-f]{6}\Z')
SESSION = re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})*\Z')
ACTIVE = {'starting', 'recording', 'closing', 'finalizing_vision'}
LOWER_ROOT = Path('/home/lab/.cache/lejulab/vr_transfer')
UPPER_ROOT = Path('/home/leju_kuavo/.cache/vr_vision_sessions')


def identity_pair(name, identity):
    if not isinstance(name, str) or not SESSION.fullmatch(name):
        raise ValueError('会话编号无效')
    if not isinstance(identity, str) or not IDENTITY.fullmatch(identity):
        raise ValueError('录制编号无效')


def root_path(root):
    root = Path(root)
    if not root.is_absolute() or not root.is_dir() or root.is_symlink() or root.resolve() != root:
        raise ValueError('缓存根目录不存在或包含符号链接')
    return root


def child(root, name):
    path = root / name
    if Path(name).name != name or name in ('.', '..') or path.is_symlink() or path.resolve().parent != root:
        raise ValueError('缓存路径不安全')
    if path.exists() and not path.is_dir():
        raise ValueError('缓存路径不是目录')
    return path


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return default


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_json(path, data):
    temp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temp.open('w') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temp.replace(path); sync_dir(path.parent)


@contextmanager
def file_lock(path):
    if path.is_symlink():
        raise ValueError('锁文件不能为符号链接')
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('该段仍在录制、封装、传输或清理，请稍后重试')
        yield


@contextmanager
def recording_lock(root, identity, capture=False):
    """Shared with the upper RGB writer, from capture until encoding finishes."""
    if not isinstance(identity, str) or not IDENTITY.fullmatch(identity):
        raise ValueError('录制编号无效')
    root = root_path(root)
    locks = child(root, '.session_locks'); locks.mkdir(exist_ok=True)
    with file_lock(locks / (identity + '.lock')):
        if capture:
            write_json(locks / (identity + '.writer.json'), {'recording_id': identity, 'pid': os.getpid()})
        yield


def command(pid):
    try:
        return [x.decode(errors='replace') for x in (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0') if x]
    except OSError:
        return []


def listener_alive(root):
    status = read_json(root / 'vr_mcap_status.json', {}) or {}
    live = any(arg.endswith('/vr_mcap_recorder.py') for arg in command(status.get('pid')))
    return live, status


def eligibility(root, name, identity):
    """Cheap read-only eligibility for the probe; deletion rechecks under locks."""
    try:
        identity_pair(name, identity); root = root_path(root); path = child(root, name)
        metadata_dir = path / 'Action_ObservationState'
        if metadata_dir.is_symlink():
            raise ValueError('元数据目录不能为符号链接')
        files = list(metadata_dir.glob('*.json'))
        if len(files) != 1 or files[0].is_symlink():
            raise ValueError('缺少唯一的录制元数据，需人工检查')
        meta = read_json(files[0], {})
        if meta.get('name') != identity or meta.get('session_id') != name:
            raise ValueError('录制编号与缓存目录不一致')
        if (path / 'transfer_manifest.json').exists():
            raise ValueError('该段已进入正常传输流程，不能删除异常缓存')
        live, status = listener_alive(root)
        if live and (status.get('session') or {}).get('name') == identity:
            raise ValueError('该段仍由采集服务持有，请等待录制和收尾结束')
        state = meta.get('state')
        if state == 'error':
            if live and not meta.get('finalized_ns'):
                raise ValueError('异常片段尚未完成收尾')
        elif state in ACTIVE and not live:
            pass
        else:
            raise ValueError('仅支持已失败或录制服务退出后遗留的未发布片段')
        return True, ''
    except (ValueError, OSError, TypeError, AttributeError) as exc:
        return False, str(exc)


def ensure_unused(paths, legacy_upper=False):
    """Legacy captures have no lease; refuse them while an old RGB server lives."""
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        args = command(proc.name)
        if legacy_upper and any(a.endswith('/vr_rgb_server.py') for a in args):
            raise ValueError('旧片段无录制锁；请先结束摇操服务，再清理遗留视觉缓存')
        if any(str(path) in arg for arg in args for path in paths):
            raise ValueError('缓存仍被录制或封装进程使用')
        try:
            descriptors = list((proc / 'fd').iterdir())
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                value = Path(os.readlink(descriptor))
            except OSError:
                continue
            if any(value == path or path in value.parents for path in paths):
                raise ValueError('缓存仍有文件正在读写，请稍后重试')


def check_tree(path):
    if path.exists() and any(p.is_symlink() for p in path.rglob('*')):
        raise ValueError('缓存包含符号链接，需人工检查')


def delete_upper(root, name, identity):
    identity_pair(name, identity); root = root_path(root)
    path = child(root, identity); deleting = child(root, '.invalid_' + identity)
    legacy = not (root / '.session_locks' / (identity + '.writer.json')).is_file()
    with recording_lock(root, identity):
        ensure_unused([path, deleting], legacy_upper=legacy)
        check_tree(path); check_tree(deleting)
        for directory in (path, deleting):
            info = read_json(directory / 'export/vision_metadata.json')
            if info and (info.get('recording_id') != identity or info.get('session_id') != name):
                raise ValueError('上位机缓存元数据编号不一致')
        if path.exists():
            if deleting.exists():
                raise ValueError('上位机清理目录冲突，未覆盖数据')
            path.rename(deleting); sync_dir(root)
        if deleting.exists():
            shutil.rmtree(deleting); sync_dir(root)
        return {'ok': True, 'recording_id': identity, 'session_id': name,
                'upper_cache_removed': not path.exists() and not deleting.exists()}


def upper_rpc(name, identity):
    command_line = shlex.join(['python3', '/home/leju_kuavo/vr_vision_recorder/vr_cache_cleanup.py',
                              '--role', 'upper', '--root', str(UPPER_ROOT)])
    result = subprocess.run(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4',
        '-o', 'ServerAliveInterval=4', '-o', 'ServerAliveCountMax=2',
        'leju_kuavo@192.168.26.12', command_line],
        input=json.dumps({'session_id': name, 'recording_id': identity, 'confirm_permanent': True}),
        capture_output=True, text=True, timeout=40)
    if result.returncode:
        raise RuntimeError('上位机缓存清理失败：' + (result.stderr.strip()[-400:] or 'SSH连接失败'))
    data = json.loads(result.stdout)
    if data.get('recording_id') != identity or data.get('session_id') != name or not data.get('upper_cache_removed'):
        raise RuntimeError(data.get('error') or '未收到上位机缓存清理完成回执')
    return data


def delete_lower(root, name, identity, rpc=upper_rpc):
    identity_pair(name, identity); root = root_path(root)
    directory = child(root, '.invalid_deletions'); directory.mkdir(exist_ok=True)
    receipt_path = directory / (identity + '.json')
    with file_lock(root / '.transfer.lock'):
        receipt = read_json(receipt_path)
        if receipt and (receipt.get('recording_id') != identity or receipt.get('session_id') != name):
            raise ValueError('清理凭据与会话编号不一致')
        path = child(root, name); deleting = child(root, '.invalid_' + identity)
        if receipt and receipt.get('state') == 'complete':
            return dict(receipt, ok=True)
        with ExitStack() as stack:
            # An orphan cannot be concurrently adopted by a newly starting listener.
            live, _ = listener_alive(root)
            if not live:
                stack.enter_context(file_lock(root / '.vr_mcap_recorder.lock'))
            if not receipt:
                allowed, reason = eligibility(root, name, identity)
                if not allowed:
                    raise ValueError(reason)
                receipt = {'recording_id': identity, 'session_id': name,
                           'started_ns': time.time_ns(), 'upper_cache_removed': False,
                           'lower_cache_removed': False}
            elif path.exists():
                allowed, reason = eligibility(root, name, identity)
                if not allowed:
                    raise ValueError(reason)
            ensure_unused([path, deleting]); check_tree(path); check_tree(deleting)
            if deleting.exists() and path.exists():
                raise ValueError('下位机清理目录冲突，未覆盖数据')
            if deleting.exists():
                meta = read_json(deleting / 'Action_ObservationState' / (identity + '.json'), {})
                if not receipt.get('lower_delete_started') and (meta.get('name') != identity or meta.get('session_id') != name):
                    raise ValueError('待清理缓存编号不一致')
            receipt.update(state='deleting', pid=os.getpid(), updated_ns=time.time_ns(), error=None)
            write_json(receipt_path, receipt)
            try:
                # Repeat the idempotent upper check even if the previous reply was lost.
                upper = rpc(name, identity)
                if not upper.get('upper_cache_removed'):
                    raise RuntimeError('上位机缓存尚未清理完成')
                receipt.update(upper_cache_removed=True, upper_receipt=upper, updated_ns=time.time_ns())
                write_json(receipt_path, receipt)
                if path.exists():
                    # Recheck identity before rename; never remove a newer same-minute session.
                    allowed, reason = eligibility(root, name, identity)
                    if not allowed:
                        raise ValueError(reason)
                    ensure_unused([path, deleting]); check_tree(path)
                    receipt['lower_delete_started'] = True
                    write_json(receipt_path, receipt)
                    path.rename(deleting); sync_dir(root)
                if deleting.exists():
                    shutil.rmtree(deleting); sync_dir(root)
                receipt.update(lower_cache_removed=True, state='complete', completed_ns=time.time_ns(),
                               updated_ns=time.time_ns(), error=None)
                write_json(receipt_path, receipt)
                return dict(receipt, ok=True)
            except Exception as exc:
                receipt.update(state='failed', error=str(exc), updated_ns=time.time_ns())
                write_json(receipt_path, receipt)
                raise


def pending_deletions(root):
    result = []
    for path in sorted((root / '.invalid_deletions').glob('*.json')):
        try:
            data = read_json(path, {})
            identity_pair(data.get('session_id'), data.get('recording_id'))
            if data.get('state') == 'complete':
                continue
            running = data.get('state') == 'deleting' and any(
                a.endswith('/vr_cache_cleanup.py') for a in command(data.get('pid')))
            result.append(dict(data, state='cache_deleting' if running else 'cache_delete_failed',
                can_delete_cache=not running, published=False,
                error=data.get('error') or (None if running else '上次缓存清理中断，请重试')))
        except (ValueError, OSError, AttributeError):
            result.append({'recording_id': path.stem, 'session_id': path.stem, 'state': 'cache_delete_failed',
                           'can_delete_cache': False, 'published': False, 'error': '清理凭据损坏，需人工检查'})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['lower', 'upper'], required=True)
    parser.add_argument('--root', type=Path)
    args = parser.parse_args()
    try:
        data = json.loads(sys.stdin.read(16385))
        if not isinstance(data, dict) or data.get('confirm_permanent') is not True:
            raise ValueError('需要明确确认永久清理异常缓存')
        root = args.root or (LOWER_ROOT if args.role == 'lower' else UPPER_ROOT)
        action = delete_lower if args.role == 'lower' else delete_upper
        result = action(root, data.get('session_id'), data.get('recording_id'))
    except Exception as exc:
        result = {'ok': False, 'error': str(exc)}
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
