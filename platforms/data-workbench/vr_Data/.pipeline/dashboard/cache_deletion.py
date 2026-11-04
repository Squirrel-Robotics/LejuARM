"""Asynchronous, durable workstation view of remote invalid-cache cleanup."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shlex
import threading
import time

from vr_cache_cleanup import identity_pair
from vr_session_storage import episode_lock, write_json


class CacheDeletions:
    def __init__(self, root, config, ssh, refresh):
        self.root, self.config, self.ssh, self.refresh = Path(root), config, ssh, refresh
        self.directory = self.root / '.pipeline/cache_deletions'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='invalid_cache')
        self.jobs = {}
        for path in self.directory.glob('*.json'):
            job = json.loads(path.read_text())
            identity_pair(job.get('session_id'), job.get('recording_id'))
            if job['state'] == 'running':
                job.update(state='failed', error='网页服务重启，上次清理结果待确认，请重试', updated_ns=time.time_ns())
                write_json(path, job)
            self.jobs[job['recording_id']] = job

    def ensure_remote_only(self, identity):
        if (self.root / '.pipeline/transfers' / (identity + '.json')).exists():
            raise ValueError('该段已有本机传输记录，请使用本机数据操作')
        if (self.root / '.pipeline/incoming' / identity).exists():
            raise ValueError('本机已有正在接收的文件，不能清理异常缓存')

    def save(self, job):
        write_json(self.directory / (job['recording_id'] + '.json'), job)
        self.jobs[job['recording_id']] = dict(job)

    def submit(self, model, identity, confirmed):
        if confirmed is not True:
            raise ValueError('请先确认永久清理上下位机对应缓存')
        if not model.get('remote_fresh') or time.time_ns() - model.get('now_ns', 0) > 5_000_000_000:
            raise ValueError('下位机状态未确认，请连接恢复后重试')
        item = next((x for x in model.get('sessions', []) if x['recording_id'] == identity), None)
        if not item or item.get('local') or not item.get('can_delete_cache'):
            raise ValueError('仅能清理未传到本机、已经停止录制的异常缓存')
        name = item.get('session_id'); identity_pair(name, identity)
        with self.lock:
            existing = self.jobs.get(identity)
            if existing and existing['state'] == 'running':
                return {'ok': True, 'accepted': True, 'recording_id': identity}
            self.ensure_remote_only(identity)
            job = {'recording_id': identity, 'session_id': name, 'state': 'running',
                   'started_ns': time.time_ns(), 'updated_ns': time.time_ns(), 'error': None,
                   'start_button_ns': item.get('start_button_ns')}
            self.save(job)
            self.executor.submit(self.run, dict(job))
        self.refresh()
        return {'ok': True, 'accepted': True, 'recording_id': identity}

    def run(self, job):
        identity = job['recording_id']
        try:
            with episode_lock(self.root, identity, blocking=False):
                self.ensure_remote_only(identity)
                helper = str(Path(self.config['remote_helper']).with_name('vr_cache_cleanup.py'))
                command = shlex.join(['python3', helper, '--role', 'lower', '--root', self.config['remote_root']])
                result = json.loads(self.ssh(command, timeout=90, payload={
                    'recording_id': identity, 'session_id': job['session_id'], 'confirm_permanent': True}))
                if not result.get('ok'):
                    raise RuntimeError(result.get('error') or '机器人缓存清理失败')
                if (result.get('recording_id') != identity or result.get('session_id') != job['session_id']
                        or not result.get('lower_cache_removed') or not result.get('upper_cache_removed')):
                    raise RuntimeError('上下位机清理回执不完整，记录继续保留')
                job.update(state='complete', receipt=result, error=None)
        except Exception as exc:
            job.update(state='failed', error=str(exc))
        job['updated_ns'] = time.time_ns()
        with self.lock:
            self.save(job)
        self.refresh()

    def apply(self, model):
        with self.lock:
            jobs = [dict(x) for x in self.jobs.values()]
        items = {x['recording_id']: dict(x) for x in model.get('sessions', [])}
        remote_ids = {identity for identity, item in items.items() if not item.get('local')}
        pending = []
        for job in jobs:
            identity = job['recording_id']; current = items.get(identity)
            if job['state'] == 'complete':
                if current and not current.get('local') and model.get('lower_observed_ns', 0) < job['updated_ns']:
                    current.update(can_delete_cache=False, phase='cache_deleting', cache_delete_reason='已清理，等待状态刷新')
                continue
            pending.append(job)
            if current and current.get('local'):
                continue
            item = current or dict(recording_id=identity, session_id=job['session_id'], local=False,
                received=False, acknowledged=False, start_button_ns=job.get('start_button_ns'))
            retry_allowed = current.get('can_delete_cache', False) if current else True
            item.update(phase='cache_deleting' if job['state'] == 'running' else 'cache_delete_failed',
                cache_deletion=True, error=job.get('error'),
                can_delete_cache=bool(model.get('remote_fresh') and job['state'] != 'running' and retry_allowed))
            items[identity] = item
        if pending:
            model.update(safe_to_shutdown=False, shutdown_reason='异常缓存尚未清理完成，请查看记录并重试')
            model['queue_count'] += sum(j['recording_id'] not in remote_ids for j in pending)
        model['sessions'] = sorted(items.values(), key=lambda x: (x.get('start_button_ns') or 0, x['recording_id']), reverse=True)
        model['cache_deletions'] = jobs
        return model

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
