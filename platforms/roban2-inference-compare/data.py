"""Incremental, read-only ACT log indexing. No robot SDK or motion imports."""
from bisect import bisect_right
from collections import OrderedDict
import json
import math
from pathlib import Path
import re
import threading
import time

NAMES = ['waist_yaw_joint'] + [f'leg_{s}{i}_joint' for s in ('l', 'r') for i in range(1, 7)] + [f'zarm_{s}{i}_joint' for s in ('l', 'r') for i in range(1, 5)]
ACTIVE = [0, 17, 18, 19, 20]
RUN_ID = re.compile(r'^\d{8}_\d{6}_\d{6}$')


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def vector(v, n):
    return v if isinstance(v, list) and len(v) == n and all(type(x) in (int, float) and math.isfinite(x) for x in v) else None


def errors(a, b):
    if a is None or b is None:
        return None
    return [math.degrees(a[j] - b[j]) for j in ACTIVE]


class Tail:
    def __init__(self, path):
        self.path, self.offset, self.pending = path, 0, b''
        self.identity, self.invalid = None, 0

    def read(self):
        try:
            stat = self.path.stat()
        except OSError:
            return [], False
        identity = (stat.st_dev, stat.st_ino)
        reset = self.identity is not None and (identity != self.identity or stat.st_size < self.offset)
        if reset:
            self.offset, self.pending = 0, b''
        self.identity = identity
        if stat.st_size == self.offset:
            return [], reset
        with self.path.open('rb') as f:
            f.seek(self.offset)
            data = f.read(32 * 1024 * 1024)
            self.offset += len(data)
        lines = (self.pending + data).split(b'\n')
        self.pending = lines.pop()
        rows = []
        for line in lines:
            try:
                value = json.loads(line)
                if isinstance(value, dict):
                    rows.append(value)
            except (ValueError, UnicodeError):
                self.invalid += 1
        return rows, reset


class Run:
    def __init__(self, path):
        self.path = path
        self.execution, self.relay = Tail(path / 'execution.jsonl'), Tail(path / 'relay.jsonl')
        self.rows, self.acks, self.ack_times = [], [], []
        self.origin = None
        self.version, self.skipped = 0, 0
        self._images = []
        self._peak = None
        self._json_cache = {}
        self.last_refresh = 0

    def cached_json(self, name):
        path = self.path / name
        try:
            stat = path.stat()
            signature = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
        except OSError:
            return {}
        cached = self._json_cache.get(name)
        if cached is None or cached[0] != signature:
            cached = (signature, read_json(path))
            self._json_cache[name] = cached
        return cached[1]

    def refresh(self):
        if time.monotonic() - self.last_refresh < .25:
            return
        self.last_refresh = time.monotonic()
        incoming, reset = self.execution.read()
        if reset:
            self.rows, self.acks, self.ack_times = [], [], []
            self.relay = Tail(self.path / 'relay.jsonl')
            self.origin = None
            self._peak = None
            self.version += 1
        for r in incoming:
            q = r.get('measured', {}).get('q')
            request = r.get('bounded_request', {})
            if not all(vector(v, 21) is not None for v in (q, r.get('model_target_q'), request.get('q'))):
                self.skipped += 1
                continue
            if not all(type(r.get(k)) in (int, float) and math.isfinite(r[k]) for k in ('at', 'wall_elapsed_s', 'policy_elapsed_s')):
                self.skipped += 1
                continue
            if self.rows and r['wall_elapsed_s'] <= self.rows[-1]['t']:
                self.skipped += 1
                continue
            if self.origin is None:
                self.origin = r['at'] - r['wall_elapsed_s']
            self.rows.append(dict(t=r['wall_elapsed_s'], p=r['policy_elapsed_s'], at=r['at'], phase=r.get('phase', ''),
                model=r['model_target_q'], bounded=request['q'], actual=q,
                model_hand=vector(r.get('model_target_hand'), 12), bounded_hand=vector(request.get('hand'), 12),
                actual_hand=vector(r.get('measured_hand'), 12), joint_stamp=r.get('joint_source_stamp')))
        incoming, reset = self.relay.read()
        if reset:
            self.acks, self.ack_times = [], []
            self._peak = None
            self.version += 1
        for a in incoming:
            if a.get('event') != 'ack':
                continue
            received = a.get('_received_wall')
            q = vector(a.get('requested', {}).get('q'), 21)
            if q is None or type(received) not in (int, float) or not math.isfinite(received):
                self.skipped += 1
                continue
            if self.ack_times and received < self.ack_times[-1]:
                self.skipped += 1
                continue
            native = a.get('native_reference', {})
            self.ack_times.append(received)
            self.acks.append(dict(received=received, seq=a.get('seq'), ack=q,
                ack_hand=vector(a.get('requested', {}).get('hand'), 12), native=vector(native.get('hardware_q_rad'), 21),
                native_actual=vector(native.get('actual_hardware_q_rad'), 21),
                native_stamp=native.get('header_sec', 0) + native.get('header_nanosec', 0) * 1e-9,
                native_seq=native.get('seq'), packet_age_ms=a.get('packet_age_ms')))
            err = errors(self.acks[-1]['native'], self.acks[-1]['native_actual'])
            if err is not None:
                j = max(range(len(ACTIVE)), key=lambda i: abs(err[i]))
                if self._peak is None or abs(err[j]) > self._peak['error_deg']:
                    self._peak = dict(error_deg=abs(err[j]), joint=NAMES[ACTIVE[j]], seq=a.get('seq'), received=received)
        self.report = self.cached_json('report.json')
        self.initial = self.cached_json('initial.json')
        state = self.cached_json('state_start.json')
        joint = state.get('channels', {}).get('joint', {}).get('data', {})
        self.head = {n: v for n, v in zip(joint.get('names', []), joint.get('q', [])) if n.startswith('zhead_')}
        self._images = []
        for p in (self.path / 'images').glob('*.jpg'):
            try:
                stamp = int(p.stem) / 1e9
                self._images.append(dict(file=p.name, stamp=stamp, t=stamp - (self.origin or stamp)))
            except ValueError:
                continue
        self._images.sort(key=lambda x: x['stamp'])

    def meta(self):
        self.refresh()
        report = self.report
        try:
            mtime = (self.path / 'execution.jsonl').stat().st_mtime
        except OSError:
            mtime = self.path.stat().st_mtime
        age = max(0, time.time() - mtime)
        finished = bool(report.get('status'))
        status = report.get('status') or ('RUNNING' if age < 5 else 'WAITING_OR_INTERRUPTED')
        peak = dict(self._peak) if self._peak else None
        if peak:
            received = peak.pop('received')
            peak['t'] = received - (self.origin if self.origin is not None else received)
        return dict(id=self.path.name, names=self.initial.get('body_names') or NAMES, active=ACTIVE, head=self.head,
            status=status, finished=finished, live=not finished and age < 5, log_age_s=age,
            error=report.get('error'), mode=report.get('mode', 'pending'), frames=len(self.rows), acks=len(self.acks),
            duration=self.rows[-1]['t'] if self.rows else 0, policy_duration=self.rows[-1]['p'] if self.rows else 0,
            speed=report.get('applied_arm_speed_deg_s', report.get('configured_arm_speed_deg_s')),
            checkpoint=report.get('checkpoint_sha256'), physical_success=report.get('physical_grasp_success'),
            origin=self.origin, images=self._images, version=self.version, peak=peak,
            parse_errors=self.execution.invalid + self.relay.invalid + self.skipped,
            source=str(self.path), comparison='kinematic_reconstruction', physics_steps=0,
            hand_mapping='simulation_illustration_not_calibrated', motion_commands_sent_by_viewer=0)

    def frames(self, after=0):
        self.refresh()
        frames = []
        for i, row in enumerate(self.rows[after:], after):
            frame = dict(row, i=i)
            ai = bisect_right(self.ack_times, row['at']) - 1
            ack = self.acks[ai] if ai >= 0 else None
            frame.update(ack=ack['ack'] if ack else None, ack_hand=ack['ack_hand'] if ack else None,
                native=ack['native'] if ack else None, native_actual=ack['native_actual'] if ack else None,
                ack_seq=ack['seq'] if ack else None, native_seq=ack['native_seq'] if ack else None,
                ack_age_ms=(row['at']-ack['received'])*1000 if ack else None,
                model_error=errors(row['model'], row['actual']),
                native_error=errors(ack['native'], ack['native_actual']) if ack else None)
            frames.append(frame)
        return frames


class Store:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.lock, self.cache = threading.RLock(), OrderedDict()

    def path(self, run_id):
        if not RUN_ID.fullmatch(run_id):
            raise ValueError('Invalid run ID')
        p = (self.root / run_id).resolve()
        if p.parent != self.root or not p.is_dir():
            raise ValueError('Run not found')
        return p

    def get(self, run_id):
        with self.lock:
            if run_id not in self.cache:
                self.cache[run_id] = Run(self.path(run_id))
            self.cache.move_to_end(run_id)
            while len(self.cache) > 3:
                self.cache.popitem(last=False)
            return self.cache[run_id]

    def index(self):
        result = []
        for p in sorted(self.root.iterdir(), reverse=True):
            if not RUN_ID.fullmatch(p.name) or not p.is_dir() or p.is_symlink():
                continue
            report = read_json(p / 'report.json')
            execution = p / 'execution.jsonl'
            result.append(dict(id=p.name, status=report.get('status'), error=report.get('error'),
                speed=report.get('applied_arm_speed_deg_s', report.get('configured_arm_speed_deg_s')),
                has_motion_log=execution.exists() and execution.stat().st_size > 0,
                finished=bool(report.get('status'))))
        return result
