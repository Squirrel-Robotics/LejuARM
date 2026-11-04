"""Read-only feedback validation and a bounded local recording command inbox."""
from collections import deque
import json
import math
from pathlib import Path
import re
import time
import uuid


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
    tmp.replace(path)


class FeedbackHealth:
    def __init__(self):
        self.samples = deque(maxlen=2000)
        self.last = None
        self.reason = '尚未收到 /rt/joint_state'
        self.q, self.v, self.tau = [], [], []
        self.valid = False

    def observe(self, msg, now=None):
        now = time.time_ns() if now is None else now
        stamp = int(msg.header_sec)*1_000_000_000 + int(msg.header_nanosec)
        vectors = [list(getattr(msg, k)) for k in ('q', 'v', 'vd', 'tau')]
        valid = all(len(v) == 23 and all(math.isfinite(x) for x in v) for v in vectors)
        valid = valid and -.05 <= (now-stamp)/1e9 <= .15
        if self.last is not None and stamp <= self.last:
            valid = False
        self.valid = valid
        self.reason = '' if valid else '关节反馈维度/数值/时间戳异常（需23维有限值及持续更新）'
        if valid:
            if self.last is not None and stamp-self.last > 150_000_000:
                self.samples.clear()
            self.samples.append(stamp)
            self.q, self.v, _, self.tau = vectors
        else:
            self.samples.clear()
            self.q, self.v, self.tau = [], [], []
        self.last = stamp

    def snapshot(self, now=None):
        now = time.time_ns() if now is None else now
        age = (now-self.last)/1e6 if self.last is not None else None
        times = [t for t in self.samples if now-2_000_000_000 <= t <= now+50_000_000]
        fresh = self.valid and age is not None and -50 <= age <= 150
        sustained = fresh and len(times) >= 3 and times[-1]-times[0] >= 200_000_000
        gaps = [(b-a)/1e6 for a,b in zip(times,times[1:])]
        return {'ready': bool(sustained), 'fresh': bool(fresh), 'valid': self.valid,
            'reason': '23个关节反馈持续更新，通信检查通过' if sustained else
                      ('反馈已过期' if self.valid and not fresh else self.reason or '正在确认反馈连续性（至少200ms）'),
            'last_state_ns': self.last, 'age_ms': age, 'dimensions': len(self.q),
            'hz': (len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else None,
            'max_interval_ms': max(gaps) if gaps else None,
            'q': self.q if fresh else [], 'v': self.v if fresh else [], 'tau': self.tau if fresh else [],
            'observed_ns': now}


def validate_task(value):
    if not isinstance(value, dict):
        raise ValueError('任务设置格式错误')
    out = {}
    for name, limit in [('name', 120), ('prompt', 2000), ('operator', 80)]:
        x = value.get(name, '')
        if not isinstance(x, str) or len(x) > limit or any(ord(c)<32 and c not in '\n\t' for c in x):
            raise ValueError('任务字段过长或包含无效字符')
        out[name] = x.strip()
    out['version'] = uuid.uuid4().hex
    out['applied_ns'] = time.time_ns()
    return out


class CommandInbox:
    def __init__(self, root):
        self.root = Path(root)/'dashboard'
        self.next_cleanup = 0
        for sub in ('requests', 'responses'):
            (self.root/sub).mkdir(parents=True, exist_ok=True)
        try:
            self.task = json.loads((self.root/'task.json').read_text())
        except (OSError, ValueError):
            self.task = {'name': '', 'prompt': '', 'operator': '', 'version': ''}

    def consume(self, recorder):
        recorder.task = dict(self.task)
        for path in sorted((self.root/'requests').glob('*.json'))[:8]:
            if not re.fullmatch(r'[0-9a-f]{32}\.json', path.name) or path.is_symlink():
                continue
            answer = self.root/'responses'/path.name
            try:
                if answer.exists():
                    continue
                data = json.loads(path.read_text())
                if abs(time.time_ns()-int(data['created_ns'])) > 5_000_000_000:
                    raise ValueError('请求已过期，请重新检查后操作')
                action = data['action']
                if action == 'task':
                    if recorder.session is not None:
                        raise ValueError('本段尚未收尾，任务设置已锁定')
                    self.task = validate_task(data.get('task'))
                    atomic_json(self.root/'task.json', self.task)
                    recorder.task = dict(self.task)
                elif action == 'start':
                    if not recorder.start(time.time_ns(), source='web'):
                        raise ValueError(getattr(recorder, 'start_error', '') or '采集条件未就绪或正在收尾')
                elif action == 'stop':
                    expected = data.get('recording_id')
                    if recorder.session and expected != recorder.session['name']:
                        raise ValueError('会话已变化，请刷新后停止当前本条')
                    recorder.request_stop(None, 'web')
                else:
                    raise ValueError('不支持的操作')
                result = {'ok': True, 'task': self.task, 'session': recorder.session}
            except Exception as exc:
                result = {'ok': False, 'error': str(exc)}
            finally:
                if not answer.exists():
                    atomic_json(answer, result)
                path.unlink(missing_ok=True)
        # Bound durable replies; request IDs are random and callers have a 5s TTL.
        if time.monotonic() >= self.next_cleanup:
            self.next_cleanup = time.monotonic()+60
            for p in (self.root/'responses').glob('*.json'):
                if time.time()-p.stat().st_mtime > 86400:
                    p.unlink(missing_ok=True)


def joint_reader(domain):
    import ctypes
    import os
    from dataclasses import dataclass
    lib = os.environ.get('CYCLONEDDS_LIB_DIR')
    if lib:
        ctypes.CDLL(str(Path(lib)/'libddsc.so'), mode=ctypes.RTLD_GLOBAL)
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import int32, uint32, sequence, float64
    @dataclass
    class JointState(IdlStruct, typename='leju::msgs::JointState'):
        header_sec: int32
        header_nanosec: uint32
        q: sequence[float64]
        v: sequence[float64]
        vd: sequence[float64]
        tau: sequence[float64]
    participant = DomainParticipant(domain)
    reader = DataReader(participant, Topic(participant, '/rt/joint_state', JointState),
        qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(32)))
    return participant, reader, JointState
