#!/usr/bin/env python3
"""LAN observation console. SSH readers feed a bounded, shared live cache."""
import argparse
import asyncio
import base64
from collections import deque
from contextlib import suppress
from datetime import datetime, timezone, timedelta
import hashlib
import json
import logging
import math
from pathlib import Path
import shlex
import shutil
import socket
import time
import xml.etree.ElementTree as ET

from aiohttp import web

ROOT = Path(__file__).resolve().parent
SHANGHAI = timezone(timedelta(hours=8))
LOG = logging.getLogger('roban.console')


def dumps(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def ssh_args(host):
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
            '-o', 'ServerAliveCountMax=2', host]


def reader_command(config, name):
    if name == 'lower':
        return ssh_args(config['lower_host']) + [shlex.join(['env', 'PYTHONDONTWRITEBYTECODE=1',
                'ROBAN_JOINT_HZ=' + str(config['joint_hz']), 'python3', '-u', '-'])]
    environment = {
        'ROBAN_COLOR_TOPIC': config['color_topic'], 'ROBAN_DEPTH_TOPIC': config['depth_topic'],
        'ROBAN_INFO_TOPIC': config['camera_info_topic'], 'ROBAN_CAMERA_FPS': config['camera_fps'],
        'ROBAN_JPEG_QUALITY': config['jpeg_quality'], 'PYTHONDONTWRITEBYTECODE': 1,
    }
    script = ('source ' + shlex.quote(config['ros_setup']) + ' && source ' +
              shlex.quote(config['camera_workspace']) + ' && exec ' +
              shlex.join(['env'] + [k + '=' + str(v) for k, v in environment.items()] + ['python3', '-u', '-']))
    command = shlex.join(['bash', '-lc', script])
    upper = ssh_args(config['upper_host'])
    upper[1:1] = ['-o', 'HostKeyAlias=' + config['upper_host_key_alias']]
    upper.append(command)
    return ssh_args(config['lower_host']) + [shlex.join(upper)] if config['upper_via_lower'] else upper


def model_metadata():
    path = ROOT / 'assets/model/robot.urdf'
    robot = ET.parse(path).getroot()
    joints = []
    hand_joints = []
    for j in robot.findall('joint'):
        if j.get('type') == 'fixed':
            continue
        limit = j.find('limit')
        entry = {'name': j.get('name'), 'type': j.get('type'),
                       'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
                       'lower': float(limit.get('lower')) if limit is not None and limit.get('lower') else None,
                       'upper': float(limit.get('upper')) if limit is not None and limit.get('upper') else None,
                       'effort': float(limit.get('effort')) if limit is not None and limit.get('effort') else None}
        mimic = j.find('mimic')
        if mimic is not None:
            entry['mimic'] = dict(mimic.attrib)
        (hand_joints if j.get('name').startswith(('left_', 'right_')) else joints).append(entry)
    manifest_path = ROOT / 'assets/model/manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    return {'name': 'Roban-2 / S17 + Revo-2 × 2', 'joints': joints, 'joint_count': len(joints),
            'hand_joints': hand_joints, 'hand_joint_count': len(hand_joints),
            'kinematic_joint_count': len(joints) + len(hand_joints), 'hand_channels_per_side': 6,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'url': '/model/robot.urdf',
            'base_mode': 'fixed_display', 'source': '本地 S17 + 双 Revo-2；继承当前工程的腕部安装变换',
            'hand_mounts': manifest.get('mounts'), 'hand_finger_pose': 'reference_open',
            'physical_calibration_verified': False, 'hand_geometry_driven': False}


class Hub:
    def __init__(self, config):
        self.config = config
        self.started = time.monotonic()
        self.channels = {}
        self.frames = {}
        self.frame_events = {s: asyncio.Event() for s in ('rgb', 'depth')}
        self.rates = {}
        self.bridges = {n: {'connected': False, 'restarts': 0, 'error': None} for n in ('lower', 'upper')}
        self.events = deque(maxlen=100)
        self.tasks = {}
        self.processes = {}
        self.clients = set()
        self.recording = None
        self.last_record = None
        self.record_bytes = 0
        self.record_count = 0
        self.record_started = None
        self.model = model_metadata()
        self.event('info', '观察台已启动', '仅建立反馈订阅，所有浏览器共享同一条采集连接。')

    def event(self, level, title, detail=''):
        self.events.appendleft({'time': datetime.now(SHANGHAI).isoformat(timespec='seconds'),
                                'level': level, 'title': title, 'detail': str(detail)[:350]})
        LOG.info('%s: %s', title, detail)

    def ingest(self, packet):
        kind = packet.get('kind')
        if kind not in {'joint', 'hand', 'hardware', 'rgb', 'depth', 'camera_info', 'controller', 'ros_status', 'lower_status', 'bridge_error'}:
            raise ValueError('Unsupported channel')
        if kind == 'joint':
            data = packet.get('data', {})
            names, q = data.get('names', []), data.get('q', [])
            if len(names) != 23 or len(set(names)) != 23 or len(q) != 23 or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in q):
                raise ValueError('Rejected invalid native joint feedback')
            expected = {j['name'] for j in self.model['joints']}
            if set(names) != expected:
                raise ValueError('Native joint names do not match the selected S17 URDF')
            for key in ('v', 'vd', 'tau'):
                values = data.get(key, [])
                if len(values) != 23 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
                    raise ValueError('Invalid joint ' + key)
            previous = self.channels.get(kind)
            if previous and packet.get('stamp', 0) <= previous.get('stamp', 0):
                raise ValueError('Duplicate/out-of-order joint feedback')
        now = time.monotonic()
        if kind in ('rgb', 'depth'):
            jpeg = base64.b64decode(packet.pop('jpeg'), validate=True)
            if not jpeg.startswith(b'\xff\xd8') or len(jpeg) > 3 * 1024 * 1024:
                raise ValueError('Invalid JPEG')
            sequence = self.frames.get(kind, {}).get('seq', 0) + 1
            self.frames[kind] = {'jpeg': jpeg, 'seq': sequence}
            self.frame_events[kind].set()
            self.frame_events[kind] = asyncio.Event()
        packet['_received_mono'] = now
        self.channels[kind] = packet
        rate = self.rates.setdefault(kind, deque(maxlen=180))
        rate.append(now)
        if self.recording and kind in ('joint', 'hand', 'hardware', 'controller', 'camera_info', 'rgb', 'depth'):
            record = {k: v for k, v in packet.items() if not k.startswith('_')}
            record['dashboard_received_at'] = time.time()
            line = dumps(record) + '\n'
            self.recording.write(line)
            self.record_bytes += len(line.encode())
            self.record_count += 1
            if (self.record_bytes >= self.config['record_limit_mb'] * 1024 * 1024 or
                    now - self.record_started >= self.config['record_limit_seconds']):
                self.stop_recording('已达到记录容量或时长上限')

    def channel(self, kind):
        packet = self.channels.get(kind)
        if not packet:
            return {'status': 'waiting', 'age_ms': None, 'source_age_ms': None, 'hz': 0, 'data': None}
        age = (time.monotonic() - packet['_received_mono']) * 1000
        source = packet.get('source_age_ms')
        source_age = source + age if isinstance(source, (int, float)) else None
        threshold = self.config['joint_stale_ms'] if kind in ('joint', 'hand') else self.config['camera_stale_ms'] if kind in ('rgb', 'depth') else 6000
        stale = age > threshold or (source_age is not None and source_age > threshold)
        clock_invalid = source is not None and source < -50
        rate = [t for t in self.rates.get(kind, ()) if time.monotonic() - t <= 3]
        hz = (len(rate) - 1) / (rate[-1] - rate[0]) if len(rate) > 1 and rate[-1] > rate[0] else 0
        return {**{k: v for k, v in packet.items() if not k.startswith('_')},
                'status': 'clock_error' if clock_invalid else 'stale' if stale else 'live',
                'age_ms': round(age, 1), 'source_age_ms': round(source_age, 1) if source_age is not None else None,
                'hz': round(hz, 1)}

    def snapshot(self):
        free = shutil.disk_usage(ROOT).free
        return {'schema': 'roban2_observation_v1', 'server_time': time.time(),
                'uptime_s': round(time.monotonic() - self.started), 'host': socket.gethostname(),
                'read_only_robot': True, 'motion_commands_sent': 0, 'viewer_count': len(self.clients),
                'channels': {k: self.channel(k) for k in ('joint', 'hand', 'hardware', 'rgb', 'depth', 'camera_info', 'controller', 'ros_status', 'lower_status')},
                'bridges': self.bridges, 'events': list(self.events),
                'recording': {'active': self.recording is not None, 'file': self.last_record,
                              'bytes': self.record_bytes, 'samples': self.record_count,
                              'duration_s': round(time.monotonic() - self.record_started) if self.recording else 0,
                              'video_recorded': False},
                'disk_free_mb': round(free / 1024 / 1024),
                'limits': {'joint_stale_ms': self.config['joint_stale_ms'], 'camera_stale_ms': self.config['camera_stale_ms']}}

    def start_recording(self):
        if self.recording:
            return
        if shutil.disk_usage(ROOT).free < 300 * 1024 * 1024:
            raise ValueError('剩余空间不足 300 MB，无法开始记录')
        name = datetime.now(SHANGHAI).strftime('%Y%m%d_%H%M%S_%f') + '.jsonl'
        self.recording = (ROOT / 'state/records' / name).open('x', buffering=1)
        self.last_record = name
        self.record_bytes = 0
        self.record_count = 0
        self.record_started = time.monotonic()
        self.event('info', '开始记录反馈', name + '（JSONL；包含图像时间戳，不包含视频像素）')

    def stop_recording(self, reason='用户结束记录'):
        if self.recording:
            self.recording.close()
            self.recording = None
            self.event('info', '反馈记录已保存', reason)

    async def run_bridge(self, name):
        source = (ROOT / 'bridges' / (name + '_reader.py')).read_bytes()
        backoff = 2
        while True:
            process = None
            error_task = None
            try:
                process = await asyncio.create_subprocess_exec(*reader_command(self.config, name),
                        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                        stderr=asyncio.subprocess.PIPE, limit=4 * 1024 * 1024)
                self.processes[name] = process
                process.stdin.write(source)
                await process.stdin.drain()
                process.stdin.close()
                async def stderr_pump():
                    async for line in process.stderr:
                        message = line.decode(errors='replace').strip()
                        if message:
                            self.bridges[name]['error'] = message[:350]
                            LOG.warning('%s: %s', name, message[:350])
                error_task = asyncio.create_task(stderr_pump())
                async for line in process.stdout:
                    if not line.startswith(b'{'):
                        continue
                    try:
                        packet = json.loads(line)
                        self.ingest(packet)
                        if not self.bridges[name]['connected']:
                            self.event('success', ('下位机' if name == 'lower' else '上位机') + '订阅已连接')
                        self.bridges[name]['connected'] = True
                        self.bridges[name]['error'] = None
                        backoff = 2
                    except (ValueError, KeyError, TypeError) as exc:
                        self.bridges[name]['error'] = str(exc)[:300]
                await process.wait()
                raise RuntimeError('SSH 采集进程退出：' + str(process.returncode))
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.bridges[name]['connected'] = False
                if not self.bridges[name]['error']:
                    self.bridges[name]['error'] = str(exc)[:350]
                self.bridges[name]['restarts'] += 1
                self.event('warning', ('下位机' if name == 'lower' else '上位机') + '采集重连', self.bridges[name]['error'])
            finally:
                if process and process.returncode is None:
                    process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), 3)
                    except asyncio.TimeoutError:
                        process.kill()
                        await process.wait()
                if error_task:
                    error_task.cancel()
                    with suppress(asyncio.CancelledError):
                        await error_task
                self.processes.pop(name, None)
            await asyncio.sleep(backoff)
            backoff = min(30, backoff * 2)


HUB_KEY = web.AppKey('hub', Hub)


@web.middleware
async def protection(request, handler):
    if request.method not in ('GET', 'HEAD', 'OPTIONS'):
        origin = request.headers.get('Origin')
        if origin and origin != request.scheme + '://' + request.host:
            raise web.HTTPForbidden(text='仅允许同源操作')
        if request.headers.get('Sec-Fetch-Site') == 'cross-site':
            raise web.HTTPForbidden(text='Cross-site mutation denied')
    response = await handler(request)
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['Referrer-Policy'] = 'same-origin'
    if request.path.startswith('/api/'):
        response.headers['Cache-Control'] = 'no-store'
    return response


def create_app(config, readers=True):
    hub = Hub(config)
    app = web.Application(middlewares=[protection], client_max_size=1024 * 1024)
    app[HUB_KEY] = hub

    async def index(request):
        return web.FileResponse(ROOT / 'web/index.html', headers={'Cache-Control': 'no-cache'})

    async def status(request):
        return web.json_response(hub.snapshot(), dumps=dumps)

    async def model(request):
        return web.json_response(hub.model, dumps=dumps)

    async def configuration(request):
        return web.json_response({k: v for k, v in config.items() if k not in ('ros_setup', 'camera_workspace')}, dumps=dumps)

    async def websocket(request):
        origin = request.headers.get('Origin')
        if origin and origin != request.scheme + '://' + request.host:
            raise web.HTTPForbidden()
        ws = web.WebSocketResponse(heartbeat=15, max_msg_size=1024)
        await ws.prepare(request)
        hub.clients.add(ws)
        async def send():
            while not ws.closed:
                await asyncio.wait_for(ws.send_str(dumps(hub.snapshot())), 3)
                await asyncio.sleep(.1)
        sender = asyncio.create_task(send())
        try:
            async for _ in ws:
                pass
        finally:
            sender.cancel()
            with suppress(asyncio.CancelledError, ConnectionError, asyncio.TimeoutError):
                await sender
            hub.clients.discard(ws)
        return ws

    async def stream(request):
        kind = request.match_info['kind']
        if kind not in ('rgb', 'depth'):
            raise web.HTTPNotFound()
        response = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                                      'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})
        await response.prepare(request)
        sequence = -1
        try:
            while True:
                frame = hub.frames.get(kind)
                if frame and frame['seq'] != sequence:
                    jpeg = frame['jpeg']
                    sequence = frame['seq']
                    await asyncio.wait_for(response.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n'), 3)
                event = hub.frame_events[kind]
                try:
                    await asyncio.wait_for(event.wait(), 1)
                except asyncio.TimeoutError:
                    if request.transport is None or request.transport.is_closing():
                        break
        except (ConnectionError, asyncio.TimeoutError, asyncio.CancelledError):
            pass
        return response

    async def snapshot_image(request):
        kind = request.match_info['kind']
        frame = hub.frames.get(kind)
        if not frame:
            raise web.HTTPServiceUnavailable(text='尚未收到图像')
        state = hub.channel(kind)
        if state['status'] != 'live':
            raise web.HTTPConflict(text='当前图像已过期，不能作为实时截图保存')
        name = datetime.now(SHANGHAI).strftime('roban2_' + kind + '_%Y%m%d_%H%M%S.jpg')
        return web.Response(body=frame['jpeg'], content_type='image/jpeg', headers={
            'Content-Disposition': 'attachment; filename="' + name + '"',
            'X-Frame-Stamp': str(state.get('stamp')), 'Cache-Control': 'no-store'})

    async def action(request):
        name = request.match_info['name']
        try:
            if name == 'record-start':
                hub.start_recording()
            elif name == 'record-stop':
                hub.stop_recording()
            elif name == 'reconnect':
                if readers:
                    for key in ('lower', 'upper'):
                        hub.tasks[key].cancel()
                    await asyncio.gather(*(hub.tasks[k] for k in ('lower', 'upper')), return_exceptions=True)
                    for key in ('lower', 'upper'):
                        hub.bridges[key]['connected'] = False
                        hub.tasks[key] = asyncio.create_task(hub.run_bridge(key))
                    hub.event('info', '正在重新连接采集器')
            else:
                raise web.HTTPNotFound()
        except ValueError as exc:
            return web.json_response({'ok': False, 'error': str(exc)}, status=409)
        return web.json_response({'ok': True, 'recording': hub.snapshot()['recording']})

    async def download_state(request):
        return web.Response(text=json.dumps(hub.snapshot(), ensure_ascii=False, indent=2, allow_nan=False),
                            content_type='application/json', headers={'Content-Disposition': 'attachment; filename="roban2_state.json"'})

    async def records(request):
        files = sorted((ROOT / 'state/records').glob('*.jsonl'), reverse=True)[:50]
        return web.json_response([{'name': p.name, 'bytes': p.stat().st_size, 'active': p.name == hub.last_record and hub.recording is not None} for p in files])

    async def record_file(request):
        name = request.match_info['name']
        if Path(name).name != name or not name.endswith('.jsonl'):
            raise web.HTTPNotFound()
        path = ROOT / 'state/records' / name
        if not path.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={'Content-Disposition': 'attachment; filename="' + name + '"'})

    async def lifetime(app):
        async def maintain():
            while True:
                await asyncio.sleep(1)
                if hub.recording and (time.monotonic() - hub.record_started >= config['record_limit_seconds'] or
                                      shutil.disk_usage(ROOT).free < 250 * 1024 * 1024):
                    hub.stop_recording('达到时长上限或剩余空间低于 250 MB')
        hub.tasks['maintenance'] = asyncio.create_task(maintain())
        if readers:
            for name in ('lower', 'upper'):
                hub.tasks[name] = asyncio.create_task(hub.run_bridge(name))
        yield
        hub.stop_recording('服务退出，文件已关闭')
        for task in hub.tasks.values():
            task.cancel()
        await asyncio.gather(*hub.tasks.values(), return_exceptions=True)
        await asyncio.gather(*(ws.close(code=1001, message=b'Server shutdown') for ws in list(hub.clients)), return_exceptions=True)

    app.cleanup_ctx.append(lifetime)
    async def shutdown(app):
        await asyncio.gather(*(ws.close(code=1001, message=b'Server shutdown') for ws in list(hub.clients)), return_exceptions=True)
    app.on_shutdown.append(shutdown)
    app.add_routes([web.get('/', index), web.get('/api/state', status), web.get('/api/model', model),
                    web.get('/api/config', configuration), web.get('/api/ws', websocket),
                    web.get('/api/stream/{kind}', stream), web.get('/api/snapshot/{kind}', snapshot_image),
                    web.get('/api/export', download_state), web.get('/api/records', records),
                    web.get('/api/records/{name}', record_file), web.post('/api/action/{name}', action)])
    app.router.add_static('/static/', ROOT / 'web', show_index=False)
    app.router.add_static('/model/', ROOT / 'assets/model', show_index=False)
    app.router.add_static('/vendor/three/build/', ROOT / 'node_modules/three/build', show_index=False)
    app.router.add_static('/vendor/three/addons/', ROOT / 'node_modules/three/examples/jsm', show_index=False)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config.json')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--no-readers', action='store_true', help='离线检查页面，不连接机器人')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    LOG.info('Robot observation only. No motion publisher, RPC, or SDK initialization.')
    web.run_app(create_app(config, readers=not args.no_readers), host=args.host or config['listen'], port=args.port or config['port'], access_log=None)


if __name__ == '__main__':
    main()
