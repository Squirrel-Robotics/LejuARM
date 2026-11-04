#!/usr/bin/env python3
"""Manage only the local comparison HTTP process."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parent
STATE = ROOT / 'state'
URL = 'http://127.0.0.1:8766/'
LISTEN_HOST = '0.0.0.0'


def lan_urls():
    """List active LAN addresses, excluding loopback and point-to-point VPNs."""
    try:
        result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'up'],
                                capture_output=True, text=True, check=True, timeout=2)
        networks = [ipaddress.ip_network(cidr) for cidr in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16')]
        addresses = []
        for interface in json.loads(result.stdout):
            if 'BROADCAST' not in interface.get('flags', []) or 'LOWER_UP' not in interface.get('flags', []):
                continue
            for info in interface.get('addr_info', []):
                address = info.get('local', '')
                if info.get('scope') == 'global' and any(ipaddress.ip_address(address) in net for net in networks):
                    addresses.append(f'http://{address}:8766/')
        return sorted(set(addresses))
    except (OSError, ValueError, subprocess.SubprocessError):
        return []


def healthy():
    try:
        with urllib.request.urlopen(URL + 'api/health', timeout=.4) as r:
            return json.load(r).get('service') == 'roban-inference-compare'
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['start', 'stop', 'status'], default='start', nargs='?')
    args = ap.parse_args()
    STATE.mkdir(exist_ok=True)
    if args.action == 'status':
        print(json.dumps(dict(running=healthy(), url=URL, lan_urls=lan_urls(), read_only=True), ensure_ascii=False))
        return 0
    if args.action == 'stop':
        try:
            pid = int((STATE / 'server.pid').read_text())
            command = (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0')
            if str(ROOT / 'server.py').encode() not in command:
                raise RuntimeError('PID does not belong to this viewer')
            os.kill(pid, signal.SIGTERM)
            print('对比网页服务已停止。')
        except FileNotFoundError:
            print('对比网页服务未运行。')
        return 0
    if not healthy():
        with (STATE / 'server.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, str(ROOT / 'server.py'), '--host', LISTEN_HOST, '--port', '8766'],
                stdin=subprocess.DEVNULL, stdout=log, stderr=log, cwd=ROOT, start_new_session=True)
        for _ in range(50):
            if process.poll() is not None:
                raise RuntimeError('网页启动失败，请查看 ' + str(STATE / 'server.log'))
            if healthy():
                (STATE / 'server.pid').write_text(str(process.pid) + '\n')
                break
            time.sleep(.1)
        else:
            process.terminate()
            raise RuntimeError('网页启动超时')
    print('ACT 仿真 / 真机运动对比网页：' + URL)
    for url in lan_urls():
        print('局域网访问地址：' + url)
    print('只读取推理日志和已有相机缓存，不启动推理、不发送机器人动作。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
