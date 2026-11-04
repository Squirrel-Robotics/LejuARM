#!/usr/bin/env python3
"""Local comparison viewer: reads existing logs and the existing camera cache."""
from functools import partial
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import argparse
import csv
import gzip
import io
import json
import mimetypes
from pathlib import Path
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

from data import Store, ACTIVE, NAMES

ROOT = Path(__file__).resolve().parent
CONFIG = json.loads((ROOT / 'config.json').read_text())
CONSOLE = Path(CONFIG['console_root']).resolve()


def workcell():
    root = ET.parse(CONFIG['simulation_scene']).getroot()
    geoms = []
    for g in root.findall('worldbody/geom'):
        name = g.get('name', '')
        if name.startswith(('table_leg', 'table_visual', 'black_left', 'white_right')) and g.get('type') == 'box':
            geoms.append(dict(name=name, pos=list(map(float, g.get('pos').split())),
                size=list(map(float, g.get('size').split())), rgba=list(map(float, g.get('rgba', '.7 .7 .7 1').split()))))
    return dict(geoms=geoms, source=CONFIG['simulation_scene'], static_reference=True, bag_pose_localized=False)


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, store, **kwargs):
        self.store = store
        super().__init__(*args, **kwargs)

    def send(self, body, mime='application/json; charset=utf-8', code=200, cache=False, attachment=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
        elif isinstance(body, str):
            body = body.encode()
        compressed = len(body) > 1500 and ('json' in mime or 'javascript' in mime or mime.startswith('text/')) and 'gzip' in self.headers.get('Accept-Encoding', '')
        if compressed:
            body = gzip.compress(body, compresslevel=2)
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'public, max-age=3600' if cache else 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if compressed:
            self.send_header('Content-Encoding', 'gzip')
            self.send_header('Vary', 'Accept-Encoding')
        if attachment:
            self.send_header('Content-Disposition', f'attachment; filename="{attachment}"')
        self.end_headers()
        self.wfile.write(body)

    def file(self, root, relative):
        root = Path(root).resolve()
        p = (root / relative).resolve()
        if not p.is_relative_to(root) or not p.is_file():
            return self.send({'error': 'File not found'}, code=404)
        return self.send(p.read_bytes(), mimetypes.guess_type(p.name)[0] or 'application/octet-stream',
                         cache=root != (ROOT / 'web').resolve())

    def do_GET(self):
        try:
            self.route()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (ValueError, KeyError) as exc:
            self.send({'error': str(exc)}, code=400)
        except Exception as exc:
            self.send({'error': type(exc).__name__ + ': ' + str(exc)}, code=503)

    def route(self):
        url = urllib.parse.urlsplit(self.path)
        path = urllib.parse.unquote(url.path)
        query = urllib.parse.parse_qs(url.query)
        if path == '/':
            return self.file(ROOT / 'web', 'index.html')
        for prefix, directory in [('/static/', ROOT / 'web'), ('/model/', CONSOLE / 'assets/model'),
                ('/vendor/three/build/', CONSOLE / 'node_modules/three/build'),
                ('/vendor/three/addons/', CONSOLE / 'node_modules/three/examples/jsm')]:
            if path.startswith(prefix):
                return self.file(directory, path[len(prefix):])
        if path == '/api/health':
            return self.send(dict(service='roban-inference-compare', read_only=True, motion_commands_sent=0,
                                  listen_host=self.server.server_address[0], port=self.server.server_port))
        if path == '/api/workcell':
            return self.send(workcell())
        if path == '/api/runs':
            with self.store.lock:
                result = self.store.index()
            return self.send(result)
        if path == '/api/live':
            with urllib.request.urlopen(CONFIG['observation_url'] + '/api/state', timeout=2) as response:
                data = json.load(response)
            return self.send(dict(channels={k: data.get('channels', {}).get(k) for k in ('joint', 'hand', 'rgb')},
                                  server_time=data.get('server_time'), read_only=True))
        if path == '/api/camera':
            with urllib.request.urlopen(CONFIG['observation_url'] + '/api/snapshot/rgb', timeout=2) as response:
                return self.send(response.read(3 * 1024 * 1024), 'image/jpeg')
        parts = path.strip('/').split('/')
        if len(parts) >= 4 and parts[:2] == ['api', 'run']:
            run_id, action = parts[2:4]
            with self.store.lock:
                run = self.store.get(run_id)
            if action == 'data':
                with self.store.lock:
                    after = max(0, int(query.get('after', ['0'])[0]))
                    meta = run.meta()
                    version = int(query.get('version', [str(meta['version'])])[0])
                    reset = after > meta['frames'] or version != meta['version']
                    result = dict(meta=meta, frames=run.frames(0 if reset else after), reset=reset)
                return self.send(result)
            if action == 'image' and len(parts) == 5:
                return self.file(run.path / 'images', parts[4])
            if action == 'csv':
                with self.store.lock:
                    rows = run.frames()
                output = io.StringIO(); writer = csv.writer(output)
                writer.writerow(['wall_seconds', 'policy_seconds', 'phase', 'ack_sequence'] +
                    [kind + '_' + NAMES[j] + '_rad' for kind in ('model', 'bounded', 'ack', 'native', 'native_actual', 'actual') for j in ACTIVE])
                for row in rows:
                    writer.writerow([row['t'], row['p'], row['phase'], row['ack_seq']] +
                        [row[kind][j] if row[kind] is not None else '' for kind in ('model', 'bounded', 'ack', 'native', 'native_actual', 'actual') for j in ACTIVE])
                return self.send(output.getvalue(), 'text/csv; charset=utf-8', attachment=run_id + '_comparison.csv')
        return self.send({'error': 'Not found'}, code=404)

    def log_message(self, fmt, *args):
        if not (args and str(args[1]) == '200'):
            super().log_message(fmt, *args)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), partial(Handler, store=Store(CONFIG['runs_root'])))
    print(f'ACT comparison viewer: http://{args.host}:{server.server_port}/', flush=True)
    server.serve_forever()
