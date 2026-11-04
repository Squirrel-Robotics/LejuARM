"""Background SSH coordination, measured clock sync, verified transfer and alignment."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import logging
from pathlib import Path
import queue
import shlex
import subprocess
import threading
import time

from vr_time_alignment import build_alignment

LOG = logging.getLogger('vr_vision')


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class VisionRPC:
    def __init__(self, config, log_path):
        self.config, self.sequence = config, 0
        self.responses = queue.Queue()
        self.log = Path(log_path).open('a')
        command = [config['remote_script'], '--rgb-topic', config['rgb_topic'],
                   '--info-topic', config['info_topic'], '--fps', str(config['fps']),
                   '--buffer-seconds', str(config.get('buffer_seconds', 5))]
        if config.get('remote_cache_root'):
            command += ['--cache-root', config['remote_cache_root']]
        remote = ' '.join(shlex.quote(str(arg)) for arg in command)
        self.process = subprocess.Popen([
            'ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3',
            config['ssh_host'], remote,
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
            text=True, bufsize=1, start_new_session=True)
        self.reader = threading.Thread(target=self.read_output, daemon=True)
        self.reader.start()
        try:
            response, _ = self.responses.get(timeout=20)
            if response is None or response.get('event') != 'ready':
                raise RuntimeError('Upper RGB service failed to start; see ' + str(log_path))
            deadline = time.monotonic() + 8
            while True:
                try:
                    self.request('status')
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.2)
        except Exception:
            self.close()
            raise

    def read_output(self):
        for line in self.process.stdout:
            received_ns = time.time_ns()
            try:
                value = json.loads(line)
            except ValueError:
                self.log.write(line)
                self.log.flush()
                continue
            self.responses.put((value, received_ns))
        self.responses.put((None, time.time_ns()))

    def request(self, command, timeout=15, **kwargs):
        self.sequence += 1
        identity = self.sequence
        request = dict(kwargs, id=identity, command=command)
        sent_ns = time.time_ns()
        self.process.stdin.write(json.dumps(request) + '\n')
        self.process.stdin.flush()
        try:
            response, received_ns = self.responses.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError('Upper vision request timed out: ' + command)
        if response is None:
            raise RuntimeError('Upper vision SSH connection closed')
        if response.get('id') != identity:
            raise RuntimeError('Upper vision response id mismatch')
        if not response['ok']:
            raise RuntimeError(response.get('error', 'Upper vision request failed'))
        return response['result'], sent_ns, received_ns

    def measure_clock(self, count=9):
        samples = []
        for _ in range(count):
            result, t0, t3 = self.request('clock')
            t1, t2 = result['receive_ns'], result['send_ns']
            delay = max(0, (t3 - t0) - (t2 - t1))
            offset = ((t1 - t0) + (t2 - t3)) // 2
            samples.append({'lower_ns': (t0 + t3) // 2, 'offset_ns': offset,
                            'round_trip_ns': delay, 'uncertainty_ns': (delay + 1) // 2})
        best = dict(min(samples, key=lambda item: item['round_trip_ns']))
        best['samples'] = samples
        best['offset_definition'] = 'upper clock minus lower clock'
        return best

    def close(self):
        try:
            if self.process.stdin and not self.process.stdin.closed:
                self.process.stdin.close()
            self.process.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.reader.join(timeout=1)
        self.log.close()


class VisionCoordinator:
    def __init__(self, config, output_root):
        self.config = config
        self.root = Path(output_root)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vr_vision')
        self.rpc = None
        self.ready = False
        self.error = 'Connecting to upper RGB camera'
        self.connection_future = None
        self.start_future = None
        self.last_attempt = 0
        self.tick()

    def connect(self):
        try:
            if self.rpc:
                self.rpc.close()
                self.rpc = None
            self.rpc = VisionRPC(self.config, self.root / '.vision_connection.log')
            self.rpc.measure_clock()
            self.ready, self.error = True, None
            LOG.info('Upper RGB ready; buffered camera, SSH and clock exchange verified')
        except Exception as exc:
            self.ready, self.error = False, str(exc)
            LOG.error('Upper RGB unavailable: %s', exc)

    def tick(self):
        if not self.ready and (self.connection_future is None or self.connection_future.done()):
            if time.monotonic() - self.last_attempt >= 5:
                self.last_attempt = time.monotonic()
                self.connection_future = self.executor.submit(self.connect)

    def begin(self, session):
        self.start_future = self.executor.submit(self.start, dict(session))

    def start(self, session):
        try:
            self.rpc.request('status')
            clock = self.rpc.measure_clock()
            result, _, _ = self.rpc.request('start', recording_id=session['name'],
                session_id=session['session_id'], start_button_ns=session['start_button_ns'],
                clock_start=clock)
            return {'clock_start': clock, 'remote': result}
        except Exception as exc:
            self.ready, self.error = False, str(exc)
            raise

    def finish(self, session):
        return self.executor.submit(self.finalize, dict(session), self.start_future)

    def finalize(self, session, start_future):
        start = start_future.result()
        try:
            end_clock = self.rpc.measure_clock()
            result, _, _ = self.rpc.request('stop', timeout=600,
                recording_id=session['name'], stop_button_ns=session['stop_boundary_ns'],
                clock_end=end_clock)
            remote = result['remote_export']
            # Remote paths and identities originate from our authenticated SSH service.
            if not remote.startswith('/') or any(ch in remote for ch in ['\n', '\r', '\0']):
                raise ValueError('Invalid remote export path')
            vision = Path(session['vision_dir'])
            incoming = vision / '.incoming'
            incoming.mkdir()
            command = ['rsync', '-a', '--partial', '--protect-args', '-e',
                       'ssh -o BatchMode=yes -o ConnectTimeout=5',
                       self.config['ssh_host'] + ':' + remote + '/', str(incoming) + '/']
            subprocess.run(command, check=True, timeout=600,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           start_new_session=True)
            manifest_path = incoming / 'manifest.json'
            if file_hash(manifest_path) != result['manifest_sha256']:
                raise RuntimeError('Transferred vision manifest checksum mismatch')
            manifest = json.loads(manifest_path.read_text())
            for name, info in manifest['files'].items():
                if Path(name).name != name or name in ('.', '..'):
                    raise ValueError('Invalid filename in vision manifest')
                path = incoming / name
                if path.stat().st_size != info['size'] or file_hash(path) != info['sha256']:
                    raise RuntimeError('Transferred file checksum mismatch: ' + name)
            for path in incoming.iterdir():
                if (vision / path.name).exists():
                    raise FileExistsError('Vision destination already contains ' + path.name)
            for path in list(incoming.iterdir()):
                path.replace(vision / path.name)
            incoming.rmdir()
            if self.config.get('processing_location') == 'workstation':
                # This is a verified raw export. Trimming/IK-grid acceptance runs
                # on the workstation after its independent transfer checks.
                metadata = dict(result['metadata'], status='complete',
                    transfer_verified=True, destination=str(vision),
                    manifest_sha256=result['manifest_sha256'],
                    processing_location='workstation',
                    processing_state='awaiting_transfer',
                    alignment_pending=True)
            elif self.config.get('cleanup_unmatched', True):
                from vr_session_cleanup import finalize_aligned_session
                metadata = finalize_aligned_session(session, self.config, result['metadata'])
            else:
                alignment = build_alignment(session, self.config)
                metadata = result['metadata']
                metadata.update(transfer_verified=True, destination=str(vision),
                                manifest_sha256=result['manifest_sha256'],
                                alignment=alignment,
                                status='complete' if alignment['status'] == 'aligned' else 'alignment_error')
            if metadata['status'] == 'complete':
                try:
                    cleanup, _, _ = self.rpc.request('ack', recording_id=session['name'],
                        manifest_sha256=result['manifest_sha256'])
                    metadata['upper_cache_cleanup'] = cleanup
                except Exception as exc:
                    # Verified local data is complete even if upper cache cleanup fails.
                    metadata['upper_cache_cleanup'] = {'error': str(exc)}
                    LOG.warning('Upper temporary JPEG cleanup deferred: %s', exc)
            return metadata
        except Exception as exc:
            self.ready, self.error = False, str(exc)
            raise RuntimeError('Vision finalization failed; upper cache retained at %s: %s' %
                               (start['remote']['remote_cache'], exc)) from exc

    def close(self):
        self.executor.shutdown(wait=True)
        if self.rpc:
            self.rpc.close()
            self.rpc = None
