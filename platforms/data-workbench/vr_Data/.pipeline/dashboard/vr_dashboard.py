#!/usr/bin/env python3
"""Local Roban2 collection dashboard. Reads telemetry; never controls motors."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import struct
import uuid
import argparse
import json
import logging
import mimetypes
import os
from pathlib import Path
import re
import secrets
import shlex
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
IDENTITY = re.compile(r'vr_\d{8}_\d{6}_\d{9}_[0-9a-f]{6}\Z')
SESSION = re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})*\Z')
ACTIVE = {'starting', 'recording', 'closing', 'finalizing_vision'}
UNIT = 'roban-vr-workstation.service'
LOG = logging.getLogger('roban_dashboard')
from workbench import Streams, Review, atomic_json, STATE_ORDER, preview_is_fresh
from robot_model import model_spec, mesh_path
from cache_deletion import CacheDeletions
from vr_dashboard_depth import inflate_depth
from communication import CommunicationTracker, current_communication
from vr_trash_purge import TrashPurges
from vr_hand_alignment import load_hand_index
from vr_session_storage import (catalog_lock,episode_lock,read_record,relative_session,session_directory,
                                recover_archives,classify_sessions,split_session,trash_sessions,
                                list_tasks,create_task,assign_task,capture_assignment,save_capture_assignment)


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def service_state():
    env = dict(os.environ, XDG_RUNTIME_DIR='/run/user/%d' % os.getuid(),
               DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/%d/bus' % os.getuid())
    result = subprocess.run(['systemctl', '--user', 'show', UNIT, '-p', 'ActiveState', '-p', 'SubState',
                             '-p', 'UnitFileState', '-p', 'ExecMainStatus'], env=env,
                            capture_output=True, text=True, timeout=4)
    values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    return {'active': values.get('ActiveState') == 'active', 'state': values.get('ActiveState', 'unknown'),
            'enabled': values.get('UnitFileState') == 'enabled', 'detail': values.get('SubState', '')}


def storage_state(root):
    try:
        usage = shutil.disk_usage(root)
        attached = os.path.ismount(ROOT.parents[1])
        return {'available': attached and os.access(root, os.W_OK), 'mounted': attached,
                'free': usage.free, 'total': usage.total, 'path': str(root)}
    except OSError as exc:
        return {'available': False, 'mounted': False, 'free': 0, 'total': 0, 'path': str(root), 'error': str(exc)}


def local_sessions(root):
    recover_archives(root)
    with catalog_lock(root,shared=True):
        return _local_sessions(root)


def _local_sessions(root):
    result = []
    for file in sorted((root / '.pipeline/transfers').glob('*.json')):
        record = read_json(file, {})
        identity, name = record.get('recording_id', ''), record.get('local_session', '')
        if not IDENTITY.fullmatch(identity) or not SESSION.fullmatch(name):
            continue
        if record.get('purged'):continue
        if record.get('purge_state'):
            item=dict(record.get('purge_snapshot',{}),recording_id=identity,session_id=name,
                local=True,deleted=True,received=True,acknowledged=True,can_archive=False,can_restore=False,
                can_purge=record['purge_state']=='failed',purge_state=record['purge_state'],
                phase='purge_failed' if record['purge_state']=='failed' else 'purging',error=record.get('purge_error'))
            result.append(item);continue
        target = session_directory(root,record)
        relative = relative_session(record)
        metadata = read_json(target / 'raw/Action_ObservationState' / (identity + '.json'), {})
        status = read_json(target / 'status.json', {})
        report = read_json(target / 'processed/vision/audit_report.json', {})
        vision = read_json(target / 'raw/vision/vision_metadata.json', {})
        expected = sum(x['size'] for x in record.get('manifest', {}).get('files', {}).values())
        incoming = root / '.pipeline/incoming' / identity
        transferred = expected if record.get('received') else sum(
            p.stat().st_size for p in incoming.rglob('*') if p.is_file() and not p.is_symlink()) if incoming.exists() else 0
        phase = 'transferring'
        if record.get('received'):
            phase = 'cleanup_pending'
        if record.get('acknowledged'):
            phase = 'received'
        if status.get('state') in ('processing', 'processing_failed', 'ready'):
            phase = status['state']
        episode = read_json(target / 'episode.json', {})
        source_health = read_json(target / 'source_health.json', {})
        capture = read_json(target / 'processed/vision/capture_health.json', {})
        alignment = read_json(target / 'processed/vision/alignment_summary.json', {})
        hand_alignment,hand_error=None,None
        try:hand_alignment=load_hand_index(target/'processed/vision')
        except (OSError,ValueError,TypeError) as exc:hand_error=str(exc)
        hand_warning=bool(hand_error or hand_alignment and not hand_alignment.get('training_complete_and_aligned'))
        a, b = metadata.get('start_button_ns'), metadata.get('stop_boundary_ns')
        result.append({'recording_id': identity, 'session_id': name, 'relative_path':relative,
            'hand':record.get('hand','') or (Path(relative).parts[-2] if len(Path(relative).parts)>1 and Path(relative).parts[-2] in ('R','L') else ''),
            'task_id':record.get('task_id',''), 'task_folder':record.get('task_folder',''),
            'deleted':bool(record.get('deleted')),'deleted_ns':record.get('deleted_ns'),
            'previous_local_path':record.get('previous_local_path'),
            'can_restore':bool(record.get('deleted')),
            'can_purge':bool(record.get('deleted') and record.get('received') and record.get('acknowledged') and phase in ('ready','processing_failed')),
            'can_archive':bool(not record.get('deleted') and record.get('received') and record.get('acknowledged') and phase in ('ready','processing_failed')),
            'local': True, 'phase': phase,
            'received': bool(record.get('received')), 'acknowledged': bool(record.get('acknowledged')),
            'received_ns': record.get('received_ns'), 'acknowledged_ns': record.get('acknowledged_ns'),
            'raw_ready_ns': record.get('manifest', {}).get('created_ns'),
            'finished_ns': status.get('finished_ns'), 'start_button_ns': a, 'stop_boundary_ns': b,
            'stop_reason': metadata.get('stop_reason'), 'start_source': metadata.get('start_source','LT+Y'),
            'task': dict(metadata.get('task',{}),name=record['task_name']) if record.get('task_id') else metadata.get('task',{}),
            'capture_task':metadata.get('task',{}), 'annotation': episode, 'source_health': source_health,
            'capture_health': capture, 'alignment': alignment,
            'hand_alignment':hand_alignment,'hand_alignment_error':hand_error,
            'state_messages': report.get('state_messages_total'),
            'retained_duration_s': report.get('retained_window_duration_s'),
            'quality': ('attention' if hand_warning or report.get('warnings') or capture and not capture.get('no_frame_loss_observed') else 'pass') if report.get('complete_and_aligned') else 'failed' if status.get('state')=='processing_failed' else 'pending',
            'recorder_ready_ns': metadata.get('recorder_ready_ns'),
            'duration_s': (b-a)/1e9 if a and b else None, 'raw_frames': vision.get('frame_count'),
            'video_frames': report.get('video_frames'), 'action_messages': report.get('action_messages_total'),
            'samples': report.get('alignment_samples'), 'alignment_hz': report.get('alignment_hz'),
            'unmatched': report.get('unmatched_actions'), 'warnings': report.get('warnings', []),
            'error': status.get('error'), 'bytes_total': expected, 'bytes_received': min(expected, transferred),
            'progress': round(min(1, transferred / expected) * 100, 1) if expected else 0,
            'path': str(target), 'report': report})
    return result


def build_model(lower, lower_ok, observed_ns, camera, local, storage, sessions, now_ns=None, feedback=None):
    """Fail closed for stale/unknown remote state; processing is independent of ACK."""
    deleted=[s for s in sessions if s.get('deleted')]
    deleted_ids={s['recording_id'] for s in deleted}
    sessions=[s for s in sessions if not s.get('deleted')]
    now = time.time_ns() if now_ns is None else now_ns
    fresh = bool(lower_ok and observed_ns and 0 <= now-observed_ns < 7_000_000_000)
    source = lower or {}
    status = source.get('status') or {}
    live = bool(fresh and source.get('listener_live'))
    heartbeat_age = (source.get('now_ns', 0)-status.get('updated_ns', 0))/1e9
    heartbeat_ok = live and status.get('schema_version', 0) >= 2 and 0 <= heartbeat_age < 5
    telemetry = status.get('telemetry') or {}
    remote_now = source.get('now_ns', 0)
    def recent(key, seconds):
        stamp = telemetry.get(key)
        return bool(stamp and -.05 <= (remote_now-stamp)/1e9 < seconds)
    joy_ok = heartbeat_ok and recent('last_joy_ns', .5)
    ref_ok = heartbeat_ok and recent('last_reference_ns', .25) and telemetry.get('reference_valid')
    feedback = feedback if feedback is not None else telemetry.get('feedback', {})
    feedback_ok = bool(feedback.get('ready') and feedback.get('fresh'))
    camera_ok = bool(camera.get('active') and now-camera.get('observed_ns', 0) < 20_000_000_000)
    processes = source.get('processes', {})
    pico_ok = bool(fresh and processes.get('pico'))
    hardware_ok = bool(fresh and processes.get('hardware') and processes.get('controller'))
    idle = heartbeat_ok and status.get('state') in ('idle', 'error') and not status.get('session')
    active = bool(status.get('session') or status.get('state') in ACTIVE) if live else False
    cache = source.get('pending_sessions', [])
    unacknowledged = [s for s in sessions if not s['received'] or not s['acknowledged']]
    safe_shutdown = bool(fresh and storage.get('available') and not active and not cache and source.get('cache_media_files') == 0 and not unacknowledged
                         and (not live or heartbeat_ok))
    healthy_storage = bool(storage.get('available') and storage.get('free', 0) >= 1024**3)
    ready = bool(idle and local.get('active') and healthy_storage and camera_ok and status.get('vision_ready')
                 and pico_ok and joy_ok and ref_ok and hardware_ok and feedback_ok)
    services = [
        {'id': 'receiver', 'title': '本机接收与裁剪', 'state': 'good' if local.get('active') else 'bad',
         'detail': '运行中 · 登录后自动启动' if local.get('active') else '未运行，点击启动接收服务'},
        {'id': 'storage', 'title': '数据保存磁盘', 'state': 'good' if healthy_storage else 'bad',
         'detail': '已挂载 · 可写' if healthy_storage else '磁盘未挂载、不可写或空间不足'},
        {'id': 'lower', 'title': '下位机连接', 'state': 'good' if fresh else 'unknown',
         'detail': '连接正常 · 状态实时更新' if fresh else '连接中断或状态过期，正在重试'},
        {'id': 'hardware', 'title': '机器人硬件与主控', 'state': ('good' if hardware_ok else 'warn') if fresh else 'unknown',
         'detail': '硬件与主控进程已启动' if hardware_ok else '等待在下位机启动摇操程序'},
        {'id': 'listener', 'title': 'LT+Y / LT+X 采集监听',
         'state': 'good' if heartbeat_ok and status.get('vision_ready') else 'warn' if fresh else 'unknown',
         'detail': '监听中 · RGB已就绪' if heartbeat_ok and status.get('vision_ready') else status.get('vision_error') or '采集监听器尚未就绪'},
        {'id': 'pico', 'title': 'Pico与动作参考', 'state': 'good' if pico_ok and joy_ok and ref_ok else 'warn' if fresh else 'unknown',
         'detail': '手柄在线 · 已标定 · 参考实时更新' if pico_ok and joy_ok and ref_ok else '等待Pico数据或有效标定参考'},
        {'id': 'feedback', 'title': '23个关节反馈检查', 'state': 'good' if feedback_ok else 'warn',
         'detail': feedback.get('reason') or '等待硬件关节反馈，检查通过后再启动主控/摇操'},
        {'id': 'camera', 'title': '上位机RGB相机', 'state': 'good' if camera_ok else 'warn' if camera.get('ok') else 'unknown',
         'detail': '相机服务运行中 · 目标60 FPS' if camera_ok else camera.get('error') or '相机服务尚未运行'},
    ]
    merged = {s['recording_id']: dict(s) for s in sessions}
    for p in cache:
        identity = p.get('recording_id') or p['session_id']
        if identity in deleted_ids:continue
        if identity in merged:
            for key in ['start_button_ns', 'stop_boundary_ns']:
                if not merged[identity].get(key):
                    merged[identity][key] = p.get(key)
            continue
        phase = 'raw_ready' if p.get('published') else p.get('state', 'unknown')
        merged[identity] = dict(p, recording_id=identity, local=False, phase=phase, received=False, acknowledged=False)
    current = status.get('session') if live else None
    if current:
        identity = current['name']
        if identity not in merged:
            merged[identity] = dict(current, recording_id=identity, phase=status.get('state'), local=False,
                                   received=False, acknowledged=False)
        else:
            merged[identity]['phase'] = status.get('state')
            for key in ['start_button_ns', 'stop_boundary_ns', 'stop_reason', 'recorder_ready_ns']:
                if key in current:
                    merged[identity][key] = current[key]
    items = sorted(merged.values(), key=lambda s: (s.get('start_button_ns') or 0, s['recording_id']), reverse=True)
    if not fresh:
        for item in items:
            if not item.get('local'):
                item['phase'] = 'unknown'
    for item in items:
        if item.get('local'):
            item['can_delete_cache'] = False
        else:
            item['can_delete_cache'] = bool(fresh and item.get('can_delete_cache'))
            if item['can_delete_cache'] and item.get('phase') in ACTIVE:
                item['phase'] = 'interrupted'
    if not fresh:
        headline, hint, tone = '下位机状态待确认', '连接恢复前，无法确认能否开始下一段或关闭下位机。', 'unknown'
    elif active:
        phase = status.get('state')
        if phase == 'recording':
            headline, hint, tone = '正在采集', 'LT+Y已开始本段录制。完成动作后按LT+X结束。', 'recording'
        elif phase == 'starting':
            headline, hint, tone = '已收到LT+Y，正在准备采集', '等待MCAP与相机完成启动，请稍候。', 'busy'
        else:
            headline, hint, tone = '正在完成本段收尾', '已停止采集，正在封装原始数据。此时LT+Y不会开始新一段。', 'busy'
    elif ready:
        headline, hint, tone = '已就绪，可以开始下一段', '先松开手柄按键，再按LT+Y开始。无需等待本机裁剪完成。', 'ready'
    elif live and status.get('state') == 'error':
        headline, hint, tone = '采集出现异常', status.get('error') or '请查看下方异常信息，原始缓存会保留。', 'error'
    else:
        failed = [s for s in services if s['state'] != 'good']
        headline, hint, tone = '等待采集服务就绪', (failed[0]['detail'] if failed else '等待采集监听器恢复空闲。'), 'waiting'
    next_reason = '松开按键后按LT+Y即可' if ready else ('先按LT+X结束，再等待本段收尾' if status.get('state') == 'recording' else '本段仍在收尾，请等待') if active else '请先完成下方服务检查'
    shutdown_reason = ('原始数据已在本机保存；本机裁剪可继续在后台执行' if items else '当前没有录制中或待传的VR数据') if safe_shutdown else \
        ('状态未确认，请勿据此关闭' if not fresh else '仍有采集中、待传或待确认清理的数据')
    return {'now_ns': now, 'headline': headline, 'hint': hint, 'tone': tone, 'remote_fresh': fresh,
        'can_start_next': ready, 'next_reason': next_reason, 'lower_can_accept_start': bool(idle),
        'safe_to_shutdown': safe_shutdown, 'shutdown_reason': shutdown_reason,
        'services': services, 'sessions': items, 'trash':deleted, 'current': current, 'current_state': status.get('state') if live else 'offline',
        'telemetry': telemetry if heartbeat_ok else {}, 'reference_fresh': bool(ref_ok), 'storage': storage,
        'feedback': dict(feedback, joint_names=STATE_ORDER,
                         communication=current_communication(source.get('communication'), fresh, now)),
        'feedback_ready': feedback_ok, 'task': status.get('task', {}) if live else {},
        'lower_disk': source.get('disk') if fresh else None, 'queue_count': len(cache),
        'receiver': local, 'lower_observed_ns': observed_ns, 'lower_now_ns': remote_now,
        'counts': {'sessions': len(sessions), 'ready': sum(s['phase'] == 'ready' for s in sessions),
                   'processing': sum(s['phase'] == 'processing' for s in sessions),
                   'errors': sum(s['phase'] == 'processing_failed' for s in sessions)}}


class Monitor:
    def __init__(self, root=ROOT):
        self.root = root
        self.config = read_json(root / '.pipeline/receiver_config.json')
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.force_lower, self.force_camera, self.force_local = [threading.Event() for _ in range(3)]
        self.lower, self.camera, self.local = {}, {}, {}
        self.lower_ok, self.observed_ns = False, 0
        self.communication = CommunicationTracker()
        self.state = {}
        self.command_lock = threading.Lock()
        self.streams = Streams(self.config, self.stopping)
        self.review = Review(root, safe_file)
        self.event_path = root / '.pipeline/dashboard/events.jsonl'
        self.event_path.parent.mkdir(parents=True, exist_ok=True)
        self.events = deque(maxlen=300)
        try:
            for line in self.event_path.read_text().splitlines()[-300:]:
                self.events.append(json.loads(line))
        except (OSError, ValueError):
            pass
        self.event_ids = {e['id'] for e in self.events}
        self.threads = []
        self.cache_deletions = CacheDeletions(self.root, self.config, self.ssh, self.refresh)
        self.trash_purges = TrashPurges(self.root,self.refresh)

    def ssh(self, command, timeout=8, payload=None):
        opts = list(self.config['ssh_options'])
        result = subprocess.run(['ssh'] + opts + [self.config['ssh_host'], command], capture_output=True,
                                text=True, input=json.dumps(payload, ensure_ascii=False) if payload is not None else None,
                                timeout=timeout, start_new_session=True)
        if result.returncode:
            raise RuntimeError(result.stderr.strip()[-400:] or 'SSH连接失败')
        return result.stdout

    def lower_loop(self):
        while not self.stopping.is_set():
            try:
                probe_script = str(Path(self.config['remote_helper']).with_name('vr_dashboard_probe.py'))
                data = json.loads(self.ssh('python3 '+shlex.quote(probe_script)+' --root '+shlex.quote(self.config['remote_root'])))
                data['communication'] = self.communication.observe(data.get('communication'))
                with self.lock:
                    self.lower, self.lower_ok, self.observed_ns = data, True, time.time_ns()
            except Exception as exc:
                with self.lock:
                    self.lower_ok = False
                LOG.debug('Lower probe: %s', exc)
            self.force_lower.wait(2);self.force_lower.clear()

    def camera_loop(self):
        while not self.stopping.is_set():
            try:
                command = "ssh -T -o BatchMode=yes -o ConnectTimeout=3 leju_kuavo@192.168.26.12 'systemctl show vr-rgb-camera.service -p ActiveState --value'"
                text = self.ssh(command, timeout=9).strip()
                camera = {'ok': True, 'active': text == 'active', 'observed_ns': time.time_ns()}
            except Exception:
                camera = {'ok': False, 'active': False, 'error': '暂时无法确认相机服务', 'observed_ns': time.time_ns()}
            with self.lock:
                self.camera = camera
            self.force_camera.wait(8);self.force_camera.clear()

    def add_event(self, identity, stage, title, detail, stamp=None, level='info'):
        key = identity + ':' + stage
        if key in self.event_ids:
            return
        event = {'id': key, 'recording_id': identity, 'stage': stage, 'title': title, 'detail': detail,
                 'timestamp_ns': stamp or time.time_ns(), 'level': level}
        self.event_ids.add(key);self.events.append(event)
        with self.event_path.open('a') as f:
            f.write(json.dumps(event, ensure_ascii=False) + '\n')

    def update_events(self, model):
        for job in model.get('cache_deletions', []):
            if job['state'] == 'complete':
                self.add_event(job['recording_id'], 'invalid_cache_deleted', '异常缓存已清理',
                    job['session_id']+' · 上下位机对应缓存已删除，网页记录已移除', job['updated_ns'], 'good')
        for s in reversed(model['sessions']):
            identity = s['recording_id']
            source = '网页' if s.get('start_source')=='web' else 'LT+Y'
            if s.get('start_button_ns'):
                self.add_event(identity, 'start_request', '已收到'+source+'开始请求，准备采集', s.get('session_id', ''), s['start_button_ns'])
            if s.get('recorder_ready_ns'):
                self.add_event(identity, 'start', source+'：本段已开始采集', s.get('session_id', ''), s['recorder_ready_ns'])
            if s.get('stop_boundary_ns'):
                title = 'LT+X：已停止采集，正在收尾' if s.get('stop_reason') == 'LT+X' else '录制已停止，正在收尾'
                self.add_event(identity, 'stop', title, '等待原始数据封装完成', s['stop_boundary_ns'])
            if s['phase'] == 'raw_ready' or s.get('local'):
                self.add_event(identity, 'raw_ready', '下位机本段收尾完成', '采集服务就绪后，可按LT+Y开始下一段', s.get('raw_ready_ns'))
            if s.get('received'):
                self.add_event(identity, 'verified', '原始数据已传输并校验完成', '本机完整原片已保存', s.get('received_ns'), 'good')
            if s.get('acknowledged'):
                self.add_event(identity, 'cleanup', '下位机对应缓存已清理', '全部会话完成接收后，即可关闭下位机', s.get('acknowledged_ns'), 'good')
            if s['phase'] == 'ready':
                self.add_event(identity, 'ready', '本机裁剪与50Hz对齐通过', str(s.get('samples', 0)) + ' 条样本全部匹配', s.get('finished_ns'), 'good')
            if s['phase'] in ('error', 'processing_failed'):
                self.add_event(identity, 'error', '本段数据需要检查', s.get('error') or '处理出现异常，原始文件仍保留', level='bad')

    def local_loop(self):
        while not self.stopping.is_set():
            try:
                service, storage, sessions = service_state(), storage_state(self.root), local_sessions(self.root)
                tasks=list_tasks(self.root)
                purged_ids={read_json(p,{}).get('recording_id') for p in (self.root/'.pipeline/transfers').glob('*.json') if read_json(p,{}).get('purged')}
                with self.lock:
                    self.local = service
                    previous = self.state
                    preview, feedback = self.streams.snapshot()
                    camera = dict(self.camera)
                    camera['active'] = bool(camera.get('active') and preview.get('fresh'))
                    self.state = build_model(self.lower, self.lower_ok, self.observed_ns, camera,
                                             service, storage, sessions, feedback=feedback)
                    self.cache_deletions.apply(self.state)
                    self.state['sessions']=[s for s in self.state['sessions'] if s['recording_id'] not in purged_ids]
                    self.state['preview'] = preview
                    self.state['root_path'] = str(self.root)
                    self.state['task_catalog'] = tasks
                    assignment=capture_assignment(self.root,self.state.get('task') or {})
                    if assignment:self.state['task']=dict(self.state['task'],task_id=assignment['task_id'],hand=assignment['hand'])
                    for item in self.state['sessions']:
                        if not item.get('local'):
                            assignment=capture_assignment(self.root,item.get('task') or {})
                            if assignment:item.update(task_id=assignment['task_id'],hand=assignment['hand'])
                    for item in sessions:
                        if item['received'] and not item.get('deleted'):
                            try:
                                with episode_lock(self.root,item['recording_id'],blocking=False):
                                    current_record=read_record(self.root,item['recording_id'])
                                    path=session_directory(self.root,current_record)/'episode.json'
                                    if not path.exists():
                                        atomic_json(path, {'recording_id':item['recording_id'], 'task':item['task'],
                                            'result':'pending', 'use_for_training':True, 'note':''})
                            except ValueError:
                                # Long local processing must not stall readiness/shutdown telemetry.
                                continue
                    self.update_events(self.state)
                    items = self.state['sessions']
                    if items and self.state['can_start_next'] and not previous.get('can_start_next'):
                        self.add_event(items[0]['recording_id'], 'next_ready', '可以开始下一段采集',
                                       '先松开所有按键，再按LT+Y，无需等待本机裁剪完成', level='good')
                    if items and self.state['safe_to_shutdown'] and not previous.get('safe_to_shutdown'):
                        self.add_event(items[0]['recording_id'], 'shutdown_ready', '全部原始数据已校验，可以关闭下位机',
                                       '下位机缓存已清理，本机裁剪可继续在后台执行', level='good')
                    self.state['events'] = sorted(self.events, key=lambda e: e['timestamp_ns'], reverse=True)[:50]
            except Exception:
                LOG.exception('Dashboard local snapshot failed')
            self.force_local.wait(1);self.force_local.clear()

    def control(self, action, data):
        with self.command_lock:
            model = self.get()
            if action=='start' and (time.time_ns()-model.get('now_ns',0)>3_000_000_000 or not model.get('can_start_next')):
                raise ValueError('采集条件未全部就绪，请先通过环境及关节反馈检查')
            if action=='task' and model.get('current'):
                raise ValueError('本段收尾完成后才能更改任务')
            if action=='task':
                registered=next((t for t in list_tasks(self.root) if t['id']==data.get('task_id')),None)
                if not registered:raise ValueError('请先新建并选择任务')
                if data.get('hand','') not in ('','R','L'):raise ValueError('左右手设置无效')
                task=data.get('task')
                if not isinstance(task,dict):raise ValueError('任务设置无效')
                data=dict(data,task=dict(task,name=registered['name']))
            control_script = str(Path(self.config['remote_helper']).with_name('vr_dashboard_control.py'))
            command = 'python3 '+shlex.quote(control_script)+' --root '+shlex.quote(self.config['remote_root'])
            payload = dict(data, action=action, request_id=data.get('request_id') or uuid.uuid4().hex)
            result = json.loads(self.ssh(command, timeout=10, payload=payload))
            self.refresh()
            if not result.get('ok'):
                raise ValueError(result.get('error','下位机未接受操作'))
            if action=='task':
                assignment=save_capture_assignment(self.root,result['task'],registered['id'],data.get('hand',''))
                result['task']=dict(result['task'],task_id=assignment['task_id'],hand=assignment['hand'])
            return result

    def start(self):
        self.streams.start()
        for target in [self.lower_loop, self.camera_loop, self.local_loop]:
            t = threading.Thread(target=target, daemon=True);t.start();self.threads.append(t)

    def get(self):
        with self.lock:
            return dict(self.state)

    def refresh(self):
        for e in [self.force_lower, self.force_camera, self.force_local]:
            e.set()


def safe_file(root, value):
    relative = Path(value)
    if relative.is_absolute() or '..' in relative.parts or len(relative.parts) < 3:
        raise ValueError('Invalid file path')
    boundary=next((i for i,p in enumerate(relative.parts) if i in (1,2,3) and p in ('raw','processed')),None)
    if boundary is None or boundary+1>=len(relative.parts):
        raise ValueError('Only a recorded session may be opened')
    prefix=split_session(Path(*relative.parts[:boundary]).as_posix())
    if prefix[0]=='.trash':raise ValueError('Deleted recordings cannot be served')
    if relative.suffix not in ('.mp4', '.json', '.jsonl', '.csv', '.mcap'):
        raise ValueError('Unsupported recording file')
    path = root / relative
    if any(p.is_symlink() for p in [path] + list(path.parents)[:len(relative.parts)]):
        raise ValueError('Symlinks cannot be served')
    if not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('File not found')
    return path


class Handler(BaseHTTPRequestHandler):
    server_version = 'RobanDashboard/1.0'

    def log_message(self, fmt, *args):
        if len(args) > 1 and str(args[1]) not in ('200', '206', '304'):
            LOG.info(fmt, *args)

    def trusted_host(self):
        return self.headers.get('Host') in self.server.allowed_hosts

    def respond(self, status, body, content_type='application/json; charset=utf-8'):
        encoded = json.dumps(body, ensure_ascii=False, allow_nan=False).encode() if not isinstance(body, bytes) else body
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.end_headers();self.wfile.write(encoded)

    def do_GET(self):
        if not self.trusted_host():
            return self.respond(403, {'error': '请使用工作台配置的访问地址'})
        url = urlparse(self.path)
        if url.path == '/api/state':
            model = self.server.monitor.get()
            model['task_catalog'] = list_tasks(self.server.monitor.root)
            if time.time_ns()-model.get('now_ns', 0) > 5_000_000_000:
                model.update(can_start_next=False, safe_to_shutdown=False, remote_fresh=False,
                    feedback_ready=False, feedback={'ready':False,'fresh':False,'reason':'关节反馈状态过期，请重新检查'}, reference_fresh=False,
                    headline='实时状态更新中断', hint='正在恢复状态检查，请勿根据旧提示关闭下位机。',
                    tone='unknown', next_reason='请等待状态更新恢复', shutdown_reason='状态已过期，关闭条件待确认')
            model['csrf_token'] = self.server.token
            return self.respond(200, model)
        if url.path == '/api/preview':
            return self.preview_stream()
        if url.path == '/api/depth/preview':
            return self.preview_stream(depth=True, raw=parse_qs(url.query).get('raw') == ['1'])
        if url.path in ('/api/review/sample','/api/review/curve','/api/review/frame','/api/review/video'):
            try:
                query = parse_qs(url.query)
                session = query.get('session',[''])[0]
                item = next((s for s in self.server.monitor.get().get('sessions',[]) if s.get('session_id')==session and s.get('local') and not s.get('deleted')),None)
                if not item or not item.get('received') or item.get('phase') not in ('ready','processing_failed'):
                    raise ValueError('该段尚未完成本机处理')
                session=item.get('relative_path',session)
                review = self.server.monitor.review
                if url.path.endswith('/video'):
                    return self.send_file(review.preview_video(session), True)
                if url.path.endswith('/curve'):
                    return self.respond(200,review.curve(session))
                index = int(query.get('index',['0'])[0])
                if url.path.endswith('/frame'):
                    return self.respond(200,review.frame(session,index),'image/jpeg')
                return self.respond(200,review.sample(session,index))
            except (ValueError,OSError,StopIteration,subprocess.TimeoutExpired,subprocess.CalledProcessError) as exc:
                return self.respond(400,{'error':str(exc)})
        if url.path in ('/api/robot/model','/api/robot/mesh'):
            try:
                spec = model_spec()
                if url.path.endswith('/model'):
                    return self.respond(200,spec)
                name = parse_qs(url.query).get('name',[''])[0]
                if name not in spec['meshes']:
                    raise ValueError('Unknown model mesh')
                return self.send_file(mesh_path(name))
            except (ValueError,OSError) as exc:
                return self.respond(404,{'error':str(exc)})
        if url.path == '/api/file':
            try:
                file = safe_file(self.server.monitor.root, parse_qs(url.query).get('path', [''])[0])
                return self.send_file(file, True)
            except (ValueError, OSError) as exc:
                return self.respond(404, {'error': str(exc)})
        routes = {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css',
            '/depth.js':'depth.js','/robot3d.js':'robot3d.js','/robot_pose.mjs':'robot_pose.mjs',
            **{'/vendor/'+name:'vendor/'+name for name in
               ('three.module.js','three.core.js','OrbitControls.js','STLLoader.js','three-LICENSE.txt')}}
        if url.path not in routes:
            return self.respond(404, {'error': 'Not found'})
        self.send_file(HERE / 'static' / routes[url.path])

    def preview_stream(self, depth=False, raw=False):
        slots = self.server.depth_preview_slots if depth else self.server.preview_slots
        if not slots.acquire(blocking=False):
            return self.respond(429, {'error':'预览连接数已满'})
        self.send_response(200)
        self.send_header('Content-Type','application/x-roban-frames')
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        streams = self.server.monitor.streams
        counter_name, packet_name = ('depth_counter', 'depth_packet') if depth else ('counter', 'packet')
        last = -1
        self.connection.settimeout(3)
        try:
            while not self.server.monitor.stopping.is_set():
                with streams.condition:
                    streams.condition.wait_for(lambda:getattr(streams,counter_name)!=last or self.server.monitor.stopping.is_set(),timeout=1)
                    changed = last != getattr(streams,counter_name)
                    last = getattr(streams,counter_name)
                    packet = getattr(streams,packet_name)
                if changed and packet and preview_is_fresh(packet[0], time.time_ns()):
                    header, frame = packet
                else:
                    header,frame = {'size':0,'fresh':False},b''
                if depth and raw and frame:
                    frame = inflate_depth(frame, header['raw_size'])
                    header = dict(header, codec='raw-u16le', size=len(frame))
                encoded = json.dumps(header).encode()
                self.wfile.write(struct.pack('!I',len(encoded))+encoded+frame)
                self.wfile.flush()
        except (BrokenPipeError,ConnectionResetError,TimeoutError,OSError,ValueError):
            pass
        finally:
            slots.release()

    def send_file(self, path, recording=False):
        size = path.stat().st_size
        start, end, status = 0, size-1, 200
        value = self.headers.get('Range')
        if value:
            match = re.fullmatch(r'bytes=(\d*)-(\d*)', value)
            if not match or not size or not any(match.groups()):
                return self.respond(416, {'error': 'Invalid range'})
            left, right = match.groups()
            if left:
                start = int(left);end = min(size-1, int(right)) if right else size-1
            else:
                start, end = max(0, size-int(right)), size-1
            if start > end or start >= size:
                return self.respond(416, {'error': 'Range outside file'})
            status = 206
        self.send_response(status)
        self.send_header('Content-Type', mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
        self.send_header('Content-Length', str(max(0, end-start+1)))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Cache-Control', 'no-store' if recording else 'no-cache')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if status == 206:
            self.send_header('Content-Range', 'bytes %d-%d/%d' % (start, end, size))
        if recording and path.suffix != '.mp4':
            self.send_header('Content-Disposition', 'attachment; filename="' + path.name + '"')
        self.end_headers()
        try:
            with path.open('rb') as f:
                f.seek(start);remaining = end-start+1
                while remaining > 0:
                    block = f.read(min(256*1024, remaining))
                    if not block:
                        break
                    self.wfile.write(block);remaining -= len(block)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        expected = 'http://' + self.headers.get('Host', '')
        if not self.trusted_host() or self.headers.get('Origin') != expected or self.headers.get('X-Roban-Token') != self.server.token:
            return self.respond(403, {'error': '请从工作台网页操作'})
        try:
            size = int(self.headers.get('Content-Length','0'))
            if not 0<=size<=16384:
                raise ValueError('请求过大')
            body = json.loads(self.rfile.read(size)) if size else {}
            if not isinstance(body,dict):
                raise ValueError('请求格式错误')
            if self.path in ('/api/record/start','/api/record/stop','/api/task'):
                action = self.path.rsplit('/',1)[-1]
                return self.respond(200,self.server.monitor.control(action,body))
            if self.path == '/api/cache/delete':
                result = self.server.monitor.cache_deletions.submit(self.server.monitor.get(),
                    body.get('recording_id'), body.get('confirm_permanent'))
                self.server.monitor.refresh()
                return self.respond(202, result)
            if self.path == '/api/trash/purge/preview':
                return self.respond(200,self.server.monitor.trash_purges.preview(body.get('recording_ids'),body.get('scope','selected')))
            if self.path == '/api/trash/purge':
                result=self.server.monitor.trash_purges.submit(body.get('plan_id'),body.get('confirm_permanent'))
                self.server.monitor.refresh()
                return self.respond(202,result)
            if self.path == '/api/annotate':
                name=body.get('session','')
                item=next((s for s in local_sessions(self.server.monitor.root) if s['session_id']==name and not s.get('deleted')),None)
                if not item:raise ValueError('找不到对应会话')
                with episode_lock(self.server.monitor.root,item['recording_id']):
                    record=read_record(self.server.monitor.root,item['recording_id'])
                    data=self.server.monitor.review.annotate(relative_session(record),body)
                self.server.monitor.refresh()
                return self.respond(200,{'ok':True,'annotation':data})
            if self.path in ('/api/trash','/api/restore'):
                result=trash_sessions(self.server.monitor.root,body.get('recording_ids'),restore=self.path=='/api/restore')
                self.server.monitor.refresh()
                return self.respond(200,result)
            if self.path == '/api/archive':
                result=classify_sessions(self.server.monitor.root,body.get('recording_ids'),body.get('hand'))
                self.server.monitor.refresh()
                return self.respond(200,result)
            if self.path == '/api/tasks/create':
                task=create_task(self.server.monitor.root,body.get('name'))
                self.server.monitor.refresh()
                return self.respond(200,{'ok':True,'task':task})
            if self.path == '/api/tasks/assign':
                result=assign_task(self.server.monitor.root,body.get('recording_ids'),body.get('task_id'))
                self.server.monitor.refresh()
                return self.respond(200,result)
        except (ValueError,OSError,RuntimeError,subprocess.TimeoutExpired) as exc:
            return self.respond(409,{'error':str(exc)})
        if self.path == '/api/refresh':
            self.server.monitor.refresh()
            return self.respond(202, {'ok': True})
        if self.path == '/api/receiver/start':
            try:
                env = dict(os.environ, XDG_RUNTIME_DIR='/run/user/%d' % os.getuid(),
                           DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/%d/bus' % os.getuid())
                r = subprocess.run(['systemctl', '--user', 'start', UNIT], env=env, capture_output=True, text=True, timeout=10)
                if r.returncode:
                    raise RuntimeError(r.stderr[-500:])
                self.server.monitor.refresh()
                return self.respond(200, {'ok': True})
            except Exception as exc:
                return self.respond(500, {'error': str(exc)})
        return self.respond(404, {'error': 'Not found'})


def main():
    p = argparse.ArgumentParser();p.add_argument('--port', type=int, default=8877)
    p.add_argument('--bind', default='127.0.0.1', help='监听地址，默认仅本机；局域网服务使用0.0.0.0')
    p.add_argument('--allow-host', action='append', default=[], help='额外允许的访问IP或主机名（不含端口，可重复）')
    p.add_argument('--data-root', type=Path, default=ROOT);args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    monitor = Monitor(args.data_root.resolve())
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.allowed_hosts = {host + ':' + str(server.server_port)
                            for host in ['127.0.0.1', 'localhost', *args.allow_host]}
    server.monitor, server.token = monitor, secrets.token_urlsafe(32)
    server.preview_slots = threading.BoundedSemaphore(4)
    server.depth_preview_slots = threading.BoundedSemaphore(4)
    monitor.start()
    def stop(*_):
        monitor.stopping.set();monitor.refresh();monitor.streams.close()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop);signal.signal(signal.SIGINT, stop)
    LOG.info('Roban2 dashboard listening on %s:%d; allowed URLs: %s', args.bind, server.server_port,
             ', '.join('http://' + host + '/' for host in sorted(server.allowed_hosts)))
    try:
        server.serve_forever(poll_interval=.25)
    finally:
        monitor.stopping.set();monitor.refresh();monitor.streams.close();monitor.cache_deletions.close();monitor.trash_purges.close();server.server_close()


if __name__ == '__main__':
    main()
