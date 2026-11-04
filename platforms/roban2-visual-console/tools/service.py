#!/usr/bin/env python3
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
PIDFILE = ROOT / 'state/server.pid'


def owned_pid():
    try:
        pid = int(PIDFILE.read_text())
        raw = Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
        if str(ROOT / 'server.py').encode() in raw:
            return pid
    except (OSError, ValueError):
        pass
    return None


def main():
    c = json.loads((ROOT / 'config.json').read_text())
    action = sys.argv[1]
    pid = owned_pid()
    if action == 'start':
        if pid:
            print('观察台已运行，PID=%d' % pid)
        else:
            with socket.socket() as sock:
                if sock.connect_ex(('127.0.0.1', c['port'])) == 0:
                    raise SystemExit('端口已被其他服务占用，请先修改 config.json 中的 port')
            with (ROOT / 'state/server.log').open('ab') as log:
                process = subprocess.Popen([sys.executable, '-u', str(ROOT / 'server.py')], cwd=ROOT,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
            PIDFILE.write_text(str(process.pid) + '\n')
            ready = False
            for _ in range(50):
                try:
                    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                    with opener.open('http://127.0.0.1:%d/api/model' % c['port'], timeout=1) as response:
                        ready = response.status == 200
                    break
                except OSError:
                    if process.poll() is not None:
                        break
                    time.sleep(.1)
            if not ready:
                raise SystemExit('服务启动失败，请查看 ' + str(ROOT / 'state/server.log'))
            print('观察台已启动，PID=%d' % process.pid)
    elif action == 'stop':
        if pid:
            # The service starts in its own session. Stop only that session's
            # observation SSH children together with the HTTP process.
            os.killpg(pid, signal.SIGTERM)
            for _ in range(100):
                if not owned_pid():
                    break
                time.sleep(.1)
            if owned_pid():
                os.kill(pid, signal.SIGKILL)
            print('观察台及其反馈订阅已停止。')
        else:
            print('观察台未运行。')
        return
    else:
        print('运行中 PID=%d' % pid if pid else '观察台未运行')
    print('本机：http://127.0.0.1:%d' % c['port'])
    try:
        interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address'], text=True))
        for interface in interfaces:
            if interface.get('operstate') != 'UP':
                continue
            for info in interface.get('addr_info', []):
                if info.get('scope') == 'global':
                    print('局域网：http://%s:%d (%s)' % (info['local'], c['port'], interface['ifname']))
    except (OSError, ValueError):
        pass
    print('日志：' + str(ROOT / 'state/server.log'))


if __name__ == '__main__':
    main()
