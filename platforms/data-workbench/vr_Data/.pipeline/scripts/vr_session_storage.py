"""Stable episode identity, R/L locations, and recoverable atomic directory moves."""
from contextlib import contextmanager,ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import time
import uuid
import unicodedata

IDENTITY=re.compile(r'vr_\d{8}_\d{6}_\d{9}_[0-9a-f]{6}\Z')
SESSION=re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})*\Z')


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temp.open('w') as f:
        json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    temp.replace(path)
    sync_dir(path.parent)


def sync_dir(path):
    fd=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


@contextmanager
def catalog_lock(root,shared=False):
    directory=Path(root)/'.pipeline';directory.mkdir(parents=True,exist_ok=True)
    with (directory/'catalog.lock').open('a') as f:
        fcntl.flock(f,fcntl.LOCK_SH if shared else fcntl.LOCK_EX)
        yield


@contextmanager
def episode_lock(root,identity,blocking=True):
    if not isinstance(identity,str) or not IDENTITY.fullmatch(identity):raise ValueError('录制编号无效')
    directory=Path(root)/'.pipeline/episode_locks';directory.mkdir(parents=True,exist_ok=True)
    with (directory/(identity+'.lock')).open('a') as f:
        try:fcntl.flock(f,fcntl.LOCK_EX|(0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError:raise ValueError('该段正在处理或操作中，请完成后再归档')
        yield


def read_record(root,identity):
    if not isinstance(identity,str) or not IDENTITY.fullmatch(identity):raise ValueError('录制编号无效')
    value=json.loads((Path(root)/'.pipeline/transfers'/(identity+'.json')).read_text())
    if value.get('recording_id')!=identity:raise ValueError('录制编号与目录索引不一致')
    relative_session(value)
    return value


def split_session(value):
    if not isinstance(value,str):raise ValueError('会话路径无效')
    parts=Path(value).parts
    if value!=Path(value).as_posix() or len(parts) not in (1,2,3) or not SESSION.fullmatch(parts[-1]):
        raise ValueError('会话路径无效')
    if len(parts)==2 and parts[0] not in ('R','L','.trash'):raise ValueError('仅支持R或L目录')
    if len(parts)==3:
        task_name(parts[0])
        if parts[1] not in ('R','L','unfiled'):raise ValueError('任务下仅支持R、L或待归档目录')
    return parts


def task_name(value):
    if not isinstance(value,str):raise ValueError('请输入任务名称')
    name=unicodedata.normalize('NFC',value.strip())
    if not name or len(name)>80 or len(name.encode('utf-8'))>180:
        raise ValueError('任务名称需为1至80个字符，且不超过180字节')
    if name.startswith('.') or name.endswith('.') or any(unicodedata.category(c).startswith('C') or c in '<>:"/\\|?*' for c in name):
        raise ValueError('任务名称不能包含路径分隔符、控制字符或 <>:"\\|?*，也不能以点开头或结尾')
    if name.casefold() in ('r','l','unfiled') or SESSION.fullmatch(name):raise ValueError('该名称为数据目录保留名称')
    return name


def _tasks(root):
    path=Path(root)/'.pipeline/task_catalog.json'
    if path.is_symlink():raise ValueError('任务目录索引不能是符号链接')
    if not path.exists():return []
    data=json.loads(path.read_text());tasks=data.get('tasks')
    if data.get('schema')!=1 or not isinstance(tasks,list):raise ValueError('任务目录索引格式无效')
    seen=set()
    for task in tasks:
        if not re.fullmatch(r'task_[0-9a-f]{32}',task.get('id','')) or task_name(task.get('name'))!=task.get('folder'):
            raise ValueError('任务目录索引内容无效')
        if task['folder'].casefold() in seen:raise ValueError('任务名称重复')
        seen.add(task['folder'].casefold())
    return tasks


def list_tasks(root):
    with catalog_lock(root,shared=True):return _tasks(root)


def _get_task(root,identity):
    task=next((t for t in _tasks(root) if t['id']==identity),None)
    if not task:raise ValueError('任务不存在，请先新建或选择任务')
    return task


def create_task(root,name):
    root=Path(root);name=task_name(name)
    with catalog_lock(root):
        tasks=_tasks(root)
        if any(t['name'].casefold()==name.casefold() for t in tasks):raise ValueError('已有同名任务，请直接选择')
        if any(p.name.casefold()==name.casefold() for p in root.iterdir()):
            raise ValueError('该名称已被数据盘上的其他目录使用，请更换任务名称')
        task=dict(id='task_'+uuid.uuid4().hex,name=name,folder=name,created_ns=time.time_ns())
        write_json(root/'.pipeline/task_catalog.json',dict(schema=1,tasks=tasks+[task]))
        return task


def save_capture_assignment(root,task,task_id,hand):
    """Bind the lower recorder's immutable task version to this PC's catalog."""
    version=task.get('version','')
    if not isinstance(version,str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,100}',version):
        raise ValueError('下位机返回的任务版本无效')
    if hand not in ('','R','L'):raise ValueError('左右手设置无效')
    with catalog_lock(root):
        registered=_get_task(root,task_id)
        if registered['name']!=task.get('name'):raise ValueError('下位机任务名称与本机任务不一致')
        value=dict(task_id=task_id,name=registered['name'],hand=hand,version=version)
        write_json(Path(root)/'.pipeline/capture_tasks'/(version+'.json'),value)
        return value


def capture_assignment(root,task):
    version=task.get('version','')
    with catalog_lock(root,shared=True):
        tasks=_tasks(root)
        if isinstance(version,str) and re.fullmatch(r'[a-zA-Z0-9_-]{1,100}',version):
            path=Path(root)/'.pipeline/capture_tasks'/(version+'.json')
            if path.is_symlink():raise ValueError('采集任务索引不能为符号链接')
            if path.exists():
                value=json.loads(path.read_text())
                if (value.get('version')!=version or value.get('name')!=task.get('name') or
                    value.get('hand') not in ('','R','L') or
                    not any(t['id']==value.get('task_id') and t['name']==value['name'] for t in tasks)):
                    raise ValueError('采集任务索引与原始记录不一致')
                return value
        registered=next((t for t in tasks if t['name']==task.get('name')),None)
        return dict(task_id=registered['id'],name=registered['name'],hand='') if registered else None


def auto_classify_received(root,identity):
    """Run outside the receiver's episode lock, then recheck under move locks."""
    with catalog_lock(root,shared=True):
        record=read_record(root,identity)
        if record.get('task_id') or record.get('deleted') or not record.get('acknowledged'):return
        directory=session_directory(root,record)
        path=directory/'raw/Action_ObservationState'/(identity+'.json')
        if not path.exists():return
        task=json.loads(path.read_text()).get('task',{})
    assignment=capture_assignment(root,task)
    if assignment:
        # Preserve a hand explicitly assigned before reception finished.
        hand=record.get('hand') or assignment['hand']
        return assign_task(root,[identity],assignment['task_id'],hand=hand,only_unassigned=True)


def _safe_parents(root,relative,create=False):
    """Check every ancestor; nested task folders must never follow symlinks."""
    split_session(relative);root=Path(root);parent=root
    for part in Path(relative).parts[:-1]:
        parent=parent/part
        if parent.is_symlink():raise ValueError('归档目录不能为符号链接')
        if create:parent.mkdir(exist_ok=True)
        if parent.exists() and (not parent.is_dir() or parent.stat().st_dev!=root.stat().st_dev):
            raise ValueError('归档目标必须是同一数据盘内的目录')
    if (root/relative).is_symlink():raise ValueError('会话目录不能为符号链接')


def relative_session(record):
    name=record.get('local_session','')
    if not isinstance(name,str) or not SESSION.fullmatch(name):raise ValueError('会话编号无效')
    relative=record.get('local_path',name)
    if split_session(relative)[-1]!=name:raise ValueError('目录位置与会话编号不一致')
    return relative


def session_directory(root,record):
    if record.get('purged') or record.get('purge_state'):
        raise ValueError('该段已提交永久删除，不能恢复、归档或重新处理')
    root=Path(root);relative=relative_session(record);path=root/relative
    for part in [path]+list(path.parents)[:len(Path(relative).parts)]:
        if part.is_symlink():raise ValueError('不能使用符号链接会话目录')
    if not path.is_dir() or not path.resolve().is_relative_to(root.resolve()):raise ValueError('会话目录不存在')
    return path


def _complete_move(root,journal_path,journal):
    """Forward recovery: journal is durable before any rename or pointer update."""
    for entry in journal['entries']:
        old,new=root/entry['source'],root/entry['destination']
        _safe_parents(root,entry['source']);_safe_parents(root,entry['destination'],create=True)
        if old.exists() and new.exists():raise ValueError('归档源和目标同时存在，保留文件并停止操作')
        if old.exists():
            old.rename(new);sync_dir(old.parent);sync_dir(new.parent)
        elif not new.is_dir():raise ValueError('找不到待恢复的归档目录')
        record=entry['record_after']
        if relative_session(record)!=entry['destination']:raise ValueError('归档日志目录不一致')
        location=dict(recording_id=record['recording_id'],hand=record.get('hand',''),deleted=bool(record.get('deleted')),
            task_id=record.get('task_id',''),task_name=record.get('task_name',''),
            relative_path=entry['destination'],original_path=entry['record_before'].get('original_local_path',entry['source']),
            operation=journal['id'],updated_ns=journal['created_ns'])
        write_json(new/'location.json',location)
        status_path=new/'status.json'
        if status_path.is_file():
            status=json.loads(status_path.read_text())
            for key,suffix in [('raw_dir','raw'),('processed_dir','processed'),('audit_report','processed/vision/audit_report.json')]:
                if key in status:status[key]=str(new/suffix)
            write_json(status_path,status)
        write_json(new/'receipt.json',record)
        write_json(root/'.pipeline/transfers'/(record['recording_id']+'.json'),record)
    journal['state']='complete';journal['finished_ns']=time.time_ns();write_json(journal_path,journal)


def _recover_locked(root):
    directory=root/'.pipeline/archives'
    for path in sorted(directory.glob('*.json')):
        journal=json.loads(path.read_text())
        if journal.get('state')=='complete':continue
        if journal.get('state')!='prepared':raise ValueError('归档日志状态无效')
        with ExitStack() as stack:
            for identity in sorted(entry['record_after']['recording_id'] for entry in journal['entries']):
                stack.enter_context(episode_lock(root,identity))
            _complete_move(root,path,journal)


def recover_archives(root):
    root=Path(root)
    with catalog_lock(root):_recover_locked(root)


def classify_sessions(root,identities,hand):
    if hand not in ('R','L'):raise ValueError('请选择R（右手）或L（左手）')
    return _relocate_sessions(root,identities,hand=hand)


def assign_task(root,identities,task_id,hand=None,only_unassigned=False):
    if not task_id:raise ValueError('请选择要归入的任务')
    if hand not in (None,'','R','L'):raise ValueError('左右手设置无效')
    return _relocate_sessions(root,identities,hand,task_id,only_unassigned)


def _relocate_sessions(root,identities,hand=None,task_id=None,only_unassigned=False):
    root=Path(root)
    if not isinstance(identities,list) or not 1<=len(identities)<=100:
        raise ValueError('请选择1至100条不同记录')
    for identity in identities:
        if not isinstance(identity,str) or not IDENTITY.fullmatch(identity):raise ValueError('录制编号无效')
    if len(set(identities))!=len(identities):raise ValueError('不能重复选择记录')
    with catalog_lock(root):
        _recover_locked(root)
        task=_get_task(root,task_id) if task_id else None
        with ExitStack() as stack:
            for identity in sorted(identities):stack.enter_context(episode_lock(root,identity,blocking=False))
            entries=[];now=time.time_ns()
            for identity in identities:
                record=read_record(root,identity);source=session_directory(root,record)
                if only_unassigned and record.get('task_id'):continue
                if record.get('deleted'):raise ValueError('请先从回收站恢复，再进行R/L归档')
                status=json.loads((source/'status.json').read_text())
                if not record.get('received') or not record.get('acknowledged') or status.get('state') not in ('ready','processing_failed'):
                    raise ValueError('仅能归档传输校验完成、缓存清理已确认且本机处理结束的记录')
                current_task=task or (_get_task(root,record['task_id']) if record.get('task_id') else None)
                side=(record.get('hand') or hand or '') if only_unassigned else hand if hand is not None else record.get('hand','')
                relative=relative_session(record)
                destination=((current_task['folder']+'/'+(side or 'unfiled')+'/') if current_task else (side+'/' if side else ''))+record['local_session']
                if relative==destination:continue
                target=root/destination
                _safe_parents(root,destination)
                if target.exists():raise ValueError('目标目录已存在或不安全，未覆盖任何数据')
                if not (source/'raw/transfer_manifest.json').is_file():raise ValueError('原始校验凭据缺失')
                after=dict(record,local_path=destination,hand=side,classified_ns=now,
                    original_local_path=record.get('original_local_path',relative))
                if current_task:after.update(task_id=current_task['id'],task_name=current_task['name'],task_folder=current_task['folder'])
                entries.append(dict(source=relative,destination=destination,record_before=record,record_after=after))
            if not entries:return dict(ok=True,moved=0,hand=hand,paths=[])
            for entry in entries:_safe_parents(root,entry['destination'],create=True)
            journal=dict(id=uuid.uuid4().hex,state='prepared',created_ns=now,hand=hand,task_id=task_id,entries=entries)
            path=root/'.pipeline/archives'/(journal['id']+'.json');write_json(path,journal)
            _complete_move(root,path,journal)
            return dict(ok=True,moved=len(entries),hand=hand,operation=journal['id'],paths=[e['destination'] for e in entries])


def undo_archive(root,operation):
    """Restore a completed archive only if no newer user reclassification occurred."""
    root=Path(root)
    if not isinstance(operation,str) or not re.fullmatch('[0-9a-f]{32}',operation):raise ValueError('归档操作编号无效')
    with catalog_lock(root):
        _recover_locked(root)
        original=json.loads((root/'.pipeline/archives'/(operation+'.json')).read_text())
        if original['state']!='complete':raise ValueError('归档尚未完成')
        entries=[]
        with ExitStack() as stack:
            for identity in sorted(e['record_after']['recording_id'] for e in original['entries']):
                stack.enter_context(episode_lock(root,identity,blocking=False))
            for entry in original['entries']:
                current=read_record(root,entry['record_after']['recording_id'])
                if relative_session(current)!=entry['destination'] or current.get('classified_ns')!=original['created_ns']:
                    raise ValueError('记录已被再次调整，不自动覆盖后续操作')
                if (root/entry['source']).exists():raise ValueError('原位置已存在数据，停止还原')
                after=dict(entry['record_before'],hand=entry['record_before'].get('hand',''))
                entries.append(dict(source=entry['destination'],destination=entry['source'],record_before=current,record_after=after))
            journal=dict(id=uuid.uuid4().hex,state='prepared',created_ns=time.time_ns(),hand='',undo_of=operation,entries=entries)
            path=root/'.pipeline/archives'/(journal['id']+'.json');write_json(path,journal);_complete_move(root,path,journal)
            return dict(ok=True,moved=len(entries),operation=journal['id'])


def trash_sessions(root,identities,restore=False):
    """Soft deletion only: move the complete episode to/from a hidden recycle bin."""
    root=Path(root)
    if not isinstance(identities,list) or not 1<=len(identities)<=100:raise ValueError('请选择1至100条记录')
    if any(not isinstance(i,str) or not IDENTITY.fullmatch(i) for i in identities) or len(set(identities))!=len(identities):
        raise ValueError('录制编号无效或重复')
    with catalog_lock(root):
        _recover_locked(root)
        with ExitStack() as stack:
            for identity in sorted(identities):stack.enter_context(episode_lock(root,identity,blocking=False))
            entries=[];now=time.time_ns()
            for identity in identities:
                record=read_record(root,identity);source=session_directory(root,record)
                deleted=bool(record.get('deleted'))
                if restore!=deleted:continue
                status=json.loads((source/'status.json').read_text())
                if not record.get('received') or not record.get('acknowledged') or status.get('state') not in ('ready','processing_failed'):
                    raise ValueError('传输校验、缓存清理确认和本机处理完成后才能删除或恢复')
                relative=relative_session(record)
                destination=record.get('previous_local_path',record['local_session']) if restore else '.trash/'+record['local_session']
                split_session(destination)
                if restore and Path(destination).parts[0]=='.trash':raise ValueError('恢复目标无效')
                target=root/destination
                _safe_parents(root,destination)
                if target.exists():raise ValueError('目标目录已存在或不安全，未覆盖任何数据')
                if not (source/'raw/transfer_manifest.json').is_file():raise ValueError('原始校验凭据缺失')
                hand=record.get('hand','')
                after=dict(record,local_path=destination,hand=hand,deleted=not restore)
                if restore:after['restored_ns']=now
                else:after.update(previous_local_path=relative,deleted_ns=now)
                entries.append(dict(source=relative,destination=destination,record_before=record,record_after=after))
            if not entries:return dict(ok=True,moved=0,restored=restore,paths=[])
            for entry in entries:
                _safe_parents(root,entry['destination'],create=True)
            journal=dict(id=uuid.uuid4().hex,state='prepared',created_ns=now,kind='restore' if restore else 'trash',entries=entries)
            path=root/'.pipeline/archives'/(journal['id']+'.json');write_json(path,journal);_complete_move(root,path,journal)
            return dict(ok=True,moved=len(entries),restored=restore,operation=journal['id'],paths=[e['destination'] for e in entries])
