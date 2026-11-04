#!/usr/bin/env python3
"""Receive verified raw VR episodes, acknowledge cleanup, then process locally."""
import argparse
import fcntl
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from vr_session_storage import (episode_lock,read_record,relative_session,session_directory,recover_archives,auto_classify_received,capture_assignment)

PIPELINE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PIPELINE / 'deps'))
LOG = logging.getLogger('vr_workstation')
IDENTITY = re.compile(r'vr_\d{8}_\d{6}_\d{9}_[0-9a-f]{6}\Z')
SESSION = re.compile(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}(?:_\d{2,})?\Z')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def validate_manifest(manifest, identity, session):
    if not IDENTITY.fullmatch(identity) or not SESSION.fullmatch(session):
        raise ValueError('Invalid recording/session identity')
    if manifest.get('version') != 1 or manifest.get('processing_location') != 'workstation':
        raise ValueError('Not a workstation raw export')
    if manifest['recording_id'] != identity or manifest['session_id'] != session:
        raise ValueError('Manifest identity differs from scan')
    required = {'processing_config.json', 'vision/rgb.mp4', 'vision/frame_timestamps.csv',
                'vision/clock_sync.json', 'vision/camera_info.json', 'vision/vision_metadata.json',
                'vision/manifest.json', 'Action_ObservationState/' + identity + '.mcap',
                'Action_ObservationState/' + identity + '.json'}
    if not required <= set(manifest['files']):
        raise ValueError('Raw export lacks required data files')
    for name, item in manifest['files'].items():
        path = Path(name)
        if path.is_absolute() or '..' in path.parts or name != path.as_posix() or name == 'transfer_manifest.json':
            raise ValueError('Unsafe manifest member')
        if not isinstance(item['size'], int) or item['size'] < 0 or not re.fullmatch('[0-9a-f]{64}', item['sha256']):
            raise ValueError('Invalid file integrity record')


def verify_export(root, manifest, expected_hash):
    if digest(root / 'transfer_manifest.json') != expected_hash:
        raise ValueError('Transfer manifest SHA256 mismatch')
    if json.loads((root / 'transfer_manifest.json').read_text()) != manifest:
        raise ValueError('Transfer manifest changed during copy')
    members = list(root.rglob('*'))
    if any(p.is_symlink() for p in members):
        raise ValueError('Symlink in raw export')
    actual = {p.relative_to(root).as_posix() for p in members if p.is_file()}
    if actual != set(manifest['files']) | {'transfer_manifest.json'}:
        raise ValueError('Unexpected/missing raw export file')
    for name, item in manifest['files'].items():
        p = root / name
        if any(x.is_symlink() for x in [p] + list(p.parents)[:len(Path(name).parts)]):
            raise ValueError('Symlink in raw export')
        if p.stat().st_size != item['size'] or digest(p) != item['sha256']:
            raise ValueError('Transferred file failed integrity check: ' + name)
    metadata = json.loads((root / 'Action_ObservationState' / (manifest['recording_id'] + '.json')).read_text())
    if metadata.get('state') != 'closed' or metadata.get('exit_code') != 0 or not metadata.get('mcap_complete'):
        raise ValueError('Native recording did not finish normally')


class Receiver:
    def __init__(self, config):
        self.config = config
        self.root = Path(config['destination']).resolve()
        self.state = self.root / '.pipeline' / 'transfers'
        self.state.mkdir(parents=True, exist_ok=True)
        self.stopping = threading.Event()
        recover_archives(self.root)

    def rpc(self, command, **kwargs):
        args = ['python3', self.config['remote_helper'], '--root', self.config['remote_root'], command]
        for key, value in kwargs.items():
            args += ['--' + key.replace('_', '-'), value]
        remote = ' '.join(shlex.quote(str(s)) for s in args)
        result = subprocess.run(['ssh'] + self.config['ssh_options'] + [self.config['ssh_host'], remote],
                                capture_output=True, text=True, timeout=600 if command == 'ack' else 20,
                                start_new_session=True)
        if result.returncode:
            raise RuntimeError('Lower ' + command + ' failed: ' + result.stderr[-1000:])
        return json.loads(result.stdout)

    def reserve(self, item, manifest):
        record_path = self.state / (item['recording_id'] + '.json')
        if record_path.exists():
            record = json.loads(record_path.read_text())
            if record.get('purged') or record.get('purge_state'):raise ValueError('该录制已提交永久删除，禁止重新接收')
            if record['manifest_sha256'] != item['manifest_sha256']:
                raise ValueError('Recording identity reused with different contents')
            return record
        reserved = {json.loads(p.read_text())['local_session'] for p in self.state.glob('*.json')}
        for i in range(10000):
            name = item['session_id'] if i == 0 else item['session_id'] + '_%02d' % i
            target = self.root / name
            if name in reserved or any((self.root/hand/name).exists() for hand in ('R','L')):
                continue
            try:
                target.mkdir()
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError('Cannot reserve destination session')
        record = dict(item, local_session=name, manifest=manifest, received=False, acknowledged=False)
        json_write(record_path, record)
        return record

    def save_record(self, record):
        json_write(self.state / (record['recording_id'] + '.json'), record)
        json_write(session_directory(self.root,record) / 'receipt.json', record)

    def receive(self, item):
        with episode_lock(self.root,item['recording_id']):
            return self._receive(item)

    def _receive(self, item):
        identity, session = item['recording_id'], item['session_id']
        if not IDENTITY.fullmatch(identity) or not SESSION.fullmatch(session):
            raise ValueError('Unsafe scan identity')
        if (self.state/(identity+'.json')).exists():
            previous=read_record(self.root,identity)
            if previous.get('purged') or previous.get('purge_state'):raise ValueError('该录制已提交永久删除，禁止重新接收')
        manifest = self.rpc('manifest', session=session)
        validate_manifest(manifest, identity, session)
        record = self.reserve(item, manifest)
        raw = session_directory(self.root,record) / 'raw'
        if not raw.exists():
            incoming = self.root / '.pipeline' / 'incoming' / identity
            incoming.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(incoming).free < sum(i['size'] for i in manifest['files'].values()) + 128 * 1024 * 1024:
                raise OSError('Insufficient workstation space; lower raw cache retained')
            source = self.config['remote_root'].rstrip('/') + '/' + session + '/'
            transport = shlex.join(['ssh'] + self.config['ssh_options'])
            command = ['rsync', '-a', '--checksum', '--partial', '--fsync', '--protect-args', '--safe-links',
                       '-e', transport, self.config['ssh_host'] + ':' + source, str(incoming) + '/']
            result = subprocess.run(command, capture_output=True, text=True, timeout=1800, start_new_session=True)
            if result.returncode:
                raise RuntimeError('Raw copy failed; lower retained: ' + result.stderr[-1000:])
            verify_export(incoming, manifest, item['manifest_sha256'])
            incoming.rename(raw)
        verify_export(raw, manifest, item['manifest_sha256'])
        record.update(received=True, received_ns=time.time_ns())
        self.save_record(record)
        LOG.info('Raw received and SHA256 verified: %s', raw)
        return record

    def acknowledge(self, record):
        with episode_lock(self.root,record['recording_id']):
            current=read_record(self.root,record['recording_id'])
            if current.get('purged') or current.get('purge_state'):raise ValueError('该录制已提交永久删除')
            return self._acknowledge(current)

    def _acknowledge(self, record):
        raw = session_directory(self.root,record) / 'raw'
        # A durable receipt and matching local bytes are prerequisites for deletion.
        verify_export(raw, record['manifest'], record['manifest_sha256'])
        response = self.rpc('ack', session=record['session_id'], recording_id=record['recording_id'],
                            manifest_sha256=record['manifest_sha256'])
        if not response.get('raw_data_removed') or response['recording_id'] != record['recording_id']:
            raise ValueError('Lower cleanup acknowledgement differs')
        record.update(acknowledged=True, acknowledged_ns=time.time_ns(), lower_receipt=response)
        self.save_record(record)
        LOG.info('Lower temporary raw data removed: %s', record['recording_id'])

    def process(self, record, retry=False):
        with episode_lock(self.root,record['recording_id']):
            current=read_record(self.root,record['recording_id'])
            if current.get('purged') or current.get('purge_state'):
                if retry:raise ValueError('该录制已提交永久删除，不能重新处理')
                return
            if current.get('deleted'):
                if retry:raise ValueError('请先从回收站恢复该段数据')
                return
            result=self._process(current,retry)
        try:auto_classify_received(self.root,record['recording_id'])
        except (ValueError,OSError):LOG.exception('Task filing pending; received data retained, will retry next poll')
        return result

    def _process(self, record, retry=False):
        target = session_directory(self.root,record)
        status_path = target / 'status.json'
        if status_path.exists() and not retry:
            if json.loads(status_path.read_text()).get('state') in ('ready', 'processing_failed'):
                return
        raw, processed = target / 'raw', target / 'processed'
        json_write(status_path, {'state': 'processing', 'processing_location': 'workstation',
                                 'started_ns': time.time_ns()})
        try:
            verify_export(raw, record['manifest'], record['manifest_sha256'])
            if processed.exists():
                shutil.rmtree(processed)
            shutil.copytree(raw, processed, copy_function=os.link)
            (processed / 'transfer_manifest.json').unlink()
            meta = processed / 'Action_ObservationState' / (record['recording_id'] + '.json')
            session = json.loads(meta.read_text())
            session.update(source_session_dir=session['session_dir'], session_id=record['local_session'],
                session_dir=str(processed), mcap=str(meta.with_suffix('.mcap')), metadata=str(meta),
                vision_dir=str(processed / 'vision'), action_observation_dir=str(processed / 'Action_ObservationState'),
                processing_location='workstation', processing_state='processing')
            # Atomic replacement is essential: the raw snapshot shares unchanged inodes.
            json_write(meta, session)
            config = json.loads((processed / 'processing_config.json').read_text())
            assignment=capture_assignment(self.root,session.get('task') or {}) or {}
            hand=record.get('hand') or assignment.get('hand')
            config['hand_required_sides']=[hand] if hand in ('L','R') else ['L','R']
            # Keep the immutable raw processing configuration unchanged.
            json_write(processed/'processing_config.json',config)
            from vr_fixed_rate_alignment import finalize_fixed_rate_session, read_streams
            source_actions, source_states = read_streams(raw/'Action_ObservationState'/(record['recording_id']+'.mcap'), config)
            def stream_health(rows):
                times=[r['t'] for r in rows]
                gaps=[(b-a)/1e6 for a,b in zip(times,times[1:])]
                return {'messages':len(rows), 'observed_hz':(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
                        'max_interval_ms':max(gaps) if gaps else None, 'intervals_over_50ms':sum(g>50 for g in gaps),
                        'invalid_messages':sum(not r['valid'] for r in rows)}
            json_write(target/'source_health.json', {'action':stream_health(source_actions), 'state':stream_health(source_states)})
            del source_actions, source_states
            finalize_fixed_rate_session(session, config)
            report = json.loads((processed / 'vision/audit_report.json').read_text())
            if not report.get('complete_and_aligned'):
                raise ValueError('Independent local crop/alignment audit did not pass')
            verify_export(raw, record['manifest'], record['manifest_sha256'])
            finished_metadata = json.loads(meta.read_text())
            finished_metadata['processing_state'] = 'ready'
            json_write(meta, finished_metadata)
            json_write(status_path, {'state': 'ready', 'processing_location': 'workstation',
                'finished_ns': time.time_ns(), 'raw_dir': str(raw), 'processed_dir': str(processed),
                'alignment_samples': report['alignment_samples'], 'alignment_hz': report['alignment_hz'],
                'video_frames': report['video_frames'], 'unmatched_samples': report['unmatched_actions'],
                'hands_training_complete_and_aligned':report.get('hands_training_complete_and_aligned',False),
                'audit_report': str(processed / 'vision/audit_report.json')})
            LOG.info('Local crop/alignment PASSED: %s; samples=%s', target, report['alignment_samples'])
        except Exception as exc:
            json_write(status_path, {'state': 'processing_failed', 'processing_location': 'workstation',
                                     'error': str(exc), 'raw_preserved': True, 'finished_ns': time.time_ns()})
            LOG.exception('Local processing failed; verified raw data remains: %s', raw)

    def cycle(self, process=True):
        recover_archives(self.root)
        # Resolve lost ACK replies even when the source already disappeared from scan.
        for p in sorted(self.state.glob('*.json')):
            record = json.loads(p.read_text())
            if record['received'] and not record['acknowledged'] and not record.get('purge_state'):
                try:
                    self.acknowledge(record)
                except Exception:
                    LOG.exception('Cleanup not acknowledged; keep receipt and retry')
        for item in self.rpc('scan'):
            if self.stopping.is_set():
                break
            try:
                record = self.receive(item)
                self.acknowledge(record)
            except Exception:
                LOG.exception('Transfer incomplete; retry on next poll')
        if process and not self.stopping.is_set():
            for p in sorted(self.state.glob('*.json')):
                record = json.loads(p.read_text())
                if record['received'] and not record.get('purge_state'):
                    self.process(record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PIPELINE / 'receiver_config.json')
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--receive-only', action='store_true')
    parser.add_argument('--retry', help='Retry local processing of this local session directory')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    receiver = Receiver(json.loads(args.config.read_text()))
    lock = (receiver.root / '.pipeline' / 'receiver.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('Another receiver owns this destination')
    for sig in [signal.SIGINT, signal.SIGTERM]:
        signal.signal(sig, lambda *_: receiver.stopping.set())
    if args.retry:
        matches = [json.loads(p.read_text()) for p in receiver.state.glob('*.json')]
        record = next(r for r in matches if args.retry in (r['local_session'],relative_session(r)))
        receiver.process(record, retry=True)
        return
    while not receiver.stopping.is_set():
        try:
            receiver.cycle(not args.receive_only)
        except Exception as exc:
            LOG.warning('Receiver waiting/retrying: %s', exc)
        if args.once:
            break
        receiver.stopping.wait(float(receiver.config.get('poll_seconds', 5)))


if __name__ == '__main__':
    main()
