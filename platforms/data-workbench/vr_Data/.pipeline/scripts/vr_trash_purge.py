"""Confirmed, bounded, retryable permanent removal of local recycled episodes."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import threading
import time
import uuid
from vr_session_storage import (IDENTITY,SESSION,catalog_lock,episode_lock,read_record,
    relative_session,session_directory,write_json,sync_dir,recover_archives)

PLAN=re.compile(r'[0-9a-f]{32}\Z')
LEGACY_CACHE=re.compile(r'v1_[0-9a-f]{64}\Z')
BUSY={'pending','running'}


def _revision(record):
    return hashlib.sha256(json.dumps(record,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def _plan_path(root,identity):
    if not isinstance(identity,str) or not PLAN.fullmatch(identity):raise ValueError('永久删除确认编号无效')
    path=Path(root)/'.pipeline/trash_purge_plans'/(identity+'.json')
    if path.is_symlink() or path.parent.is_symlink():raise ValueError('永久删除确认目录不安全')
    return path


def _target(root,relative):
    parts=Path(relative).parts
    allowed=(len(parts)==2 and parts[0]=='.trash' and SESSION.fullmatch(parts[1])) or (
        len(parts)==3 and parts[:2]==('.pipeline','review_cache') and
        (IDENTITY.fullmatch(parts[2]) or LEGACY_CACHE.fullmatch(parts[2])))
    if not allowed or relative!=Path(relative).as_posix():raise ValueError('永久删除目标超出回收站或审核缓存范围')
    root=Path(root);target=root
    for part in parts:
        target=target/part
        if target.is_symlink():raise ValueError('永久删除拒绝符号链接目录')
        if target.exists() and target.stat().st_dev!=root.stat().st_dev:raise ValueError('永久删除拒绝跨磁盘目录')
    return target


def _scan(root,relatives):
    """Count physical bytes once; shared inodes are reclaimed only at last link."""
    inodes={};logical=0;files=0
    def visit(path):
        nonlocal logical,files
        info=path.lstat()
        if stat.S_ISLNK(info.st_mode):raise ValueError('永久删除目录中包含符号链接，请先检查')
        if info.st_dev!=Path(root).stat().st_dev or os.path.ismount(path):raise ValueError('永久删除目录包含其他挂载点')
        if stat.S_ISDIR(info.st_mode):
            with os.scandir(path) as entries:
                for item in entries:visit(Path(item.path))
        elif stat.S_ISREG(info.st_mode):
            logical+=info.st_size;files+=1;key=(info.st_dev,info.st_ino)
            value=inodes.setdefault(key,[0,info.st_nlink,info.st_blocks*512]);value[0]+=1
        else:raise ValueError('永久删除目录包含非普通文件')
    for relative in dict.fromkeys(relatives):
        path=_target(root,relative)
        if path.exists():visit(path)
    return dict(logical_bytes=logical,estimated_reclaim_bytes=sum(v[2] for v in inodes.values() if v[0]>=v[1]),file_count=files)


def _snapshot(root,record):
    if record.get('purge_snapshot'):return record['purge_snapshot']
    target=session_directory(root,record);identity=record['recording_id']
    path=target/'raw/Action_ObservationState'/(identity+'.json')
    metadata=json.loads(path.read_text()) if path.is_file() else {}
    return dict(recording_id=identity,session_id=record['local_session'],hand=record.get('hand',''),
        task_id=record.get('task_id',''),task=dict(name=record.get('task_name') or metadata.get('task',{}).get('name','')),
        start_button_ns=metadata.get('start_button_ns'),relative_path=relative_session(record),
        local=True,deleted=True,received=True,acknowledged=True)


def _check_record(root,record):
    if record.get('purged'):raise ValueError('该段已经永久删除，请刷新列表')
    if not record.get('deleted') or relative_session(record)!='.trash/'+record['local_session']:
        raise ValueError('只能永久删除回收站中的片段')
    if not record.get('received') or not record.get('acknowledged'):raise ValueError('传输和下位机缓存清理尚未完成，不能永久删除')
    if record.get('purge_state') in BUSY:raise ValueError('已有片段正在永久删除，请完成后再操作')
    if not record.get('purge_state'):
        target=session_directory(root,record)
        status=json.loads((target/'status.json').read_text())
        if status.get('state') not in ('ready','processing_failed'):raise ValueError('本机处理结束后才能永久删除')
        if not (target/'raw/transfer_manifest.json').is_file():raise ValueError('原始校验凭据缺失，未删除数据')


def preview_purge(root,identities=None,scope='selected'):
    root=Path(root);recover_archives(root)
    with catalog_lock(root):
        if scope=='all':
            identities=[]
            for path in sorted((root/'.pipeline/transfers').glob('*.json')):
                record=json.loads(path.read_text())
                if record.get('deleted') and not record.get('purged'):identities.append(record['recording_id'])
        elif scope!='selected':raise ValueError('永久删除范围无效')
        if not isinstance(identities,list) or not identities:raise ValueError('请选择回收站片段，或回收站已为空')
        if scope=='selected' and len(identities)>100:raise ValueError('单次所选最多100段，可使用清空回收站')
        if any(not isinstance(i,str) or not IDENTITY.fullmatch(i) for i in identities) or len(set(identities))!=len(identities):raise ValueError('录制编号无效或重复')
        entries=[]
        with ExitStack() as stack:
            for identity in sorted(identities):stack.enter_context(episode_lock(root,identity,blocking=False))
            for identity in identities:
                record=read_record(root,identity);_check_record(root,record)
                targets=[relative_session(record),'.pipeline/review_cache/'+identity]
                entries.append(dict(recording_id=identity,revision=_revision(record),snapshot=_snapshot(root,record),targets=targets))
            legacy=[];cache=root/'.pipeline/review_cache'
            if cache.is_symlink():raise ValueError('审核缓存目录不能是符号链接')
            for path in sorted(cache.glob('v1_*')):
                if LEGACY_CACHE.fullmatch(path.name):legacy.append(str(path.relative_to(root)))
            sizes=_scan(root,legacy+[p for e in entries for p in e['targets']])
            now=time.time_ns();plan=dict(id=uuid.uuid4().hex,state='preview',scope=scope,created_ns=now,
                expires_ns=now+300_000_000_000,entries=entries,legacy_targets=legacy,**sizes)
            write_json(_plan_path(root,plan['id']),plan)
            return dict(ok=True,plan_id=plan['id'],count=len(entries),items=[e['snapshot'] for e in entries],
                expires_ns=plan['expires_ns'],legacy_cache_count=len(legacy),**sizes)


def commit_purge(root,plan_id,confirmed):
    if confirmed is not True:raise ValueError('请明确确认永久删除且无法恢复')
    root=Path(root)
    with catalog_lock(root):
        path=_plan_path(root,plan_id);plan=json.loads(path.read_text())
        if plan.get('state')=='failed':raise ValueError('上次清理未完成，请重新查看剩余文件并确认重试')
        if plan.get('state')!='preview':return dict(ok=True,accepted=True,plan_id=plan_id,state=plan['state'])
        if time.time_ns()>plan['expires_ns']:raise ValueError('删除确认已过期，请重新查看片段和空间估算')
        with ExitStack() as stack:
            records=[]
            for identity in sorted(e['recording_id'] for e in plan['entries']):stack.enter_context(episode_lock(root,identity,blocking=False))
            for entry in plan['entries']:
                record=read_record(root,entry['recording_id']);_check_record(root,record)
                if _revision(record)!=entry['revision']:raise ValueError('回收站记录已变化，请重新确认；未删除任何数据')
                records.append(record)
            _scan(root,plan['legacy_targets']+[p for e in plan['entries'] for p in e['targets']])
            try:
                for entry,record in zip(plan['entries'],records):
                    record.update(purge_state='pending',purge_plan=plan_id,purge_error=None,purge_snapshot=entry['snapshot'])
                    write_json(root/'.pipeline/transfers'/(record['recording_id']+'.json'),record)
                plan.update(state='queued',confirmed_ns=time.time_ns());write_json(path,plan)
            except OSError:
                for record in records:
                    try:
                        current=read_record(root,record['recording_id'])
                        if current.get('purge_plan')==plan_id:
                            current.update(purge_state='failed',purge_error='提交删除时写入失败，尚未开始删除，请重试')
                            write_json(root/'.pipeline/transfers'/(current['recording_id']+'.json'),current)
                    except OSError:pass
                raise
    return dict(ok=True,accepted=True,plan_id=plan_id,state='queued',count=len(plan['entries']))


def _remove(root,relative):
    target=_target(root,relative)
    _scan(root,[relative])
    if target.exists():shutil.rmtree(target);sync_dir(target.parent)


def _set_failed(root,entry,plan_id,error):
    with catalog_lock(root),episode_lock(root,entry['recording_id']):
        record=read_record(root,entry['recording_id'])
        if record.get('purge_plan')==plan_id and not record.get('purged'):
            record.update(purge_state='failed',purge_error=str(error),purge_updated_ns=time.time_ns())
            write_json(Path(root)/'.pipeline/transfers'/(record['recording_id']+'.json'),record)


def run_purge(root,plan_id,refresh=lambda:None):
    root=Path(root);path=_plan_path(root,plan_id);plan=json.loads(path.read_text())
    if plan.get('state') not in ('queued','running'):return
    plan.update(state='running',updated_ns=time.time_ns());write_json(path,plan)
    errors=[]
    try:
        # Anonymous old previews cannot be attributed reliably. They are disposable,
        # listed in the confirmation, and all new previews use a recording namespace.
        for relative in plan['legacy_targets']:_remove(root,relative)
    except Exception as exc:
        for entry in plan['entries']:_set_failed(root,entry,plan_id,exc)
        errors.append(str(exc))
    if not errors:
        for entry in plan['entries']:
            identity=entry['recording_id']
            try:
                with catalog_lock(root),episode_lock(root,identity):
                    record=read_record(root,identity)
                    if record.get('purged'):continue
                    if record.get('purge_plan')!=plan_id:raise ValueError('永久删除任务已变更')
                    if not record.get('deleted') or relative_session(record)!='.trash/'+record['local_session']:raise ValueError('永久删除目录校验失败')
                    record.update(purge_state='running');write_json(root/'.pipeline/transfers'/(identity+'.json'),record)
                refresh()
                # No catalog lock during recursive IO: monitoring remains responsive.
                with episode_lock(root,identity):
                    for relative in entry['targets']:_remove(root,relative)
                with catalog_lock(root),episode_lock(root,identity):
                    record=read_record(root,identity)
                    if record.get('purge_plan')!=plan_id:raise ValueError('永久删除任务已变更')
                    tombstone={k:record[k] for k in ('recording_id','session_id','local_session','local_path','manifest_sha256','hand','task_id','task_name') if k in record}
                    tombstone.update(deleted=True,purged=True,purge_state='complete',purge_plan=plan_id,
                        purged_ns=time.time_ns(),received=False,acknowledged=True)
                    write_json(root/'.pipeline/transfers'/(identity+'.json'),tombstone)
            except Exception as exc:
                errors.append(identity+': '+str(exc));_set_failed(root,entry,plan_id,exc)
            refresh()
    plan.update(state='failed' if errors else 'complete',errors=errors,updated_ns=time.time_ns())
    write_json(path,plan);refresh()


def recover_purges(root):
    """A restart never resumes irreversible work silently; show a retry action."""
    root=Path(root)
    with catalog_lock(root):
        for path in (root/'.pipeline/transfers').glob('*.json'):
            record=json.loads(path.read_text())
            if record.get('purge_state') in BUSY:
                with episode_lock(root,record['recording_id']):
                    record.update(purge_state='failed',purge_error='网页服务重启，删除可能未完成，请确认后重试')
                    write_json(path,record)
        for path in (root/'.pipeline/trash_purge_plans').glob('*.json'):
            plan=json.loads(path.read_text())
            if plan.get('state') in ('queued','running'):
                plan.update(state='failed',errors=['服务重启，剩余文件等待确认重试'],updated_ns=time.time_ns());write_json(path,plan)


class TrashPurges:
    def __init__(self,root,refresh):
        self.root=Path(root);self.refresh=refresh;self.lock=threading.Lock();self.running=set()
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='trash-purge');recover_purges(self.root)
    def preview(self,identities=None,scope='selected'):return preview_purge(self.root,identities,scope)
    def submit(self,plan_id,confirmed):
        with self.lock:
            result=commit_purge(self.root,plan_id,confirmed)
            if result['state']=='queued' and plan_id not in self.running:
                self.running.add(plan_id);self.executor.submit(self._run,plan_id)
            self.refresh();return result
    def _run(self,plan_id):
        try:run_purge(self.root,plan_id,self.refresh)
        except Exception as exc:
            path=_plan_path(self.root,plan_id);plan=json.loads(path.read_text())
            for entry in plan['entries']:_set_failed(self.root,entry,plan_id,exc)
            plan.update(state='failed',errors=[str(exc)],updated_ns=time.time_ns());write_json(path,plan)
        finally:
            with self.lock:self.running.discard(plan_id)
            self.refresh()
    def close(self):self.executor.shutdown(wait=False,cancel_futures=True)
