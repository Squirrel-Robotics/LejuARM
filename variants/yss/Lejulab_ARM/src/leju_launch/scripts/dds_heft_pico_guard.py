"""Stop only this project's optional Pico frontend before HEFT warmup in AMP.

No DDS publications or controller changes. Process identity is rechecked under
the required privilege and SIGTERM is sent through a pidfd, never a PID group.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def read_process(pid, proc=Path('/proc')):
    folder = proc / str(pid)
    try:
        fields = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
        argv = [os.fsdecode(x) for x in (folder / 'cmdline').read_bytes().split(b'\0') if x]
        return {'pid': int(pid), 'ppid': int(fields[1]),
                'start_ticks': int(fields[19]), 'state': fields[0], 'argv': argv}
    except (FileNotFoundError, ProcessLookupError):
        return None


def optional_source_hashes(root):
    scripts = Path(root) / 'src/leju_launch/scripts'
    checks = {}
    for name, function in (('_pico_teleop_common.sh', 'launch_pico_gmr'),
                           ('_launch_common.sh', 'wait_and_cleanup')):
        path = scripts / name
        content = path.read_bytes()
        match = re.search(r'^' + function + r'\(\)\s*\{\n(.*?)^\}',
                          content.decode(), re.M | re.S)
        if match is None:
            raise RuntimeError(f'无法核实Pico可选进程的启动方式：{path}')
        body = match.group(1)
        if function == 'launch_pico_gmr':
            valid = (re.search(r'^\s*ALL_PIDS\+=\(', body, re.M)
                     and 'REQUIRED_PIDS' not in body)
        else:
            valid = ('"${REQUIRED_PIDS[@]}"' in body
                     and '"${ALL_PIDS[@]}"' not in body)
        if not valid:
            raise RuntimeError('Pico可能已被列为必需进程；拒绝自动停止，请检查启动配置')
        checks[str(path.resolve())] = hashlib.sha256(content).hexdigest()
    return checks


def find_optional_pico(root, proc=Path('/proc')):
    root = Path(root).resolve()
    launcher = root / 'src/leju_launch/scripts/launch_teleop_real.sh'
    candidates = []
    for folder in proc.iterdir():
        if not folder.name.isdecimal():
            continue
        try:
            child = read_process(int(folder.name), proc)
        except PermissionError:
            continue  # Any unseen DDS publisher is still rejected by PublicationGuard.
        if child is None or child['state'] == 'Z':
            continue
        argv = child['argv']
        if len(argv) < 3 or argv[1:3] != ['-m', 'leju_pico_gmr.main']:
            continue
        parent = read_process(child['ppid'], proc)
        pargv = parent['argv'] if parent else []
        if (not re.fullmatch(r'python(?:3(?:\.\d+)?)?', Path(argv[0]).name)
                or len(pargv) < 2 or Path(pargv[0]).name != 'bash'):
            raise RuntimeError(f'Pico PID {child["pid"]} 不是已核实的可选启动方式；拒绝自动停止')
        parent_script = Path(pargv[1])
        if not parent_script.is_absolute():
            parent_script = root / parent_script
        if parent_script.resolve() != launcher.resolve():
            raise RuntimeError(f'Pico PID {child["pid"]} 不属于本工程主启动器；拒绝自动停止')
        candidates.append({'child': child, 'parent': parent, 'launcher': str(launcher.resolve())})
    if candidates:
        hashes = optional_source_hashes(root)
        for item in candidates:
            item['source_hashes'] = hashes
    return candidates


# Runs under the privilege needed to signal a root-owned frontend. The pidfd is
# opened first; identity, parent cwd and source checks precede the only signal.
_STOP_CODE = r'''
import hashlib, json, os, pathlib, signal, sys, time
expected = json.load(sys.stdin)
def identity(pid):
    folder = pathlib.Path('/proc') / str(pid)
    fields = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
    return {'pid': pid, 'ppid': int(fields[1]), 'start_ticks': int(fields[19]),
            'argv': [os.fsdecode(x) for x in (folder / 'cmdline').read_bytes().split(b'\0') if x]}
child = expected['child']
fd = os.pidfd_open(child['pid'])
try:
    for key in ('child', 'parent'):
        value = expected[key]
        actual = identity(value['pid'])
        if any(actual[field] != value[field] for field in ('pid', 'ppid', 'start_ticks', 'argv')):
            raise RuntimeError('process identity changed; no signal sent')
    parent = expected['parent']
    script = pathlib.Path(parent['argv'][1])
    if not script.is_absolute():
        script = pathlib.Path(os.readlink('/proc/' + str(parent['pid']) + '/cwd')) / script
    if str(script.resolve()) != expected['launcher']:
        raise RuntimeError('launcher working directory differs; no signal sent')
    for filename, digest in expected['source_hashes'].items():
        if hashlib.sha256(pathlib.Path(filename).read_bytes()).hexdigest() != digest:
            raise RuntimeError('launcher source changed; no signal sent')
    if time.monotonic() - expected['amp_observed_monotonic'] > 0.75:
        raise RuntimeError('AMP observation expired; no signal sent')
    signal.pidfd_send_signal(fd, signal.SIGTERM)
finally:
    os.close(fd)
'''


def signal_verified_pico(identity):
    command = [sys.executable, '-c', _STOP_CODE]
    if os.geteuid() != 0:
        command = ['sudo', '-n', '--'] + command
    result = subprocess.run(command, input=json.dumps(identity), text=True,
                            capture_output=True, timeout=4)
    if result.returncode:
        detail = result.stderr.strip()[-1500:]
        raise RuntimeError('无法停止已核实的Pico/GMR：' + detail +
                           '\n若提示需要sudo密码，请先在本机终端运行 sudo -v，再重试本脚本。')


def stop_optional_pico(root, ensure_amp, events):
    candidates = find_optional_pico(root)
    for identity in candidates:
        state = ensure_amp()  # Must be fresh AMP immediately before any signal.
        child = identity['child']
        event = {'pid': child['pid'], 'parent_pid': child['ppid'],
                 'start_ticks': child['start_ticks'], 'requested_at': time.time(),
                 'controller_observation': state, 'status': 'checking'}
        events.append(event)
        print(f'AMP已确认：停止主启动器的可选 Pico/GMR（PID {child["pid"]}）……', flush=True)
        signal_verified_pico({**identity, 'amp_observed_monotonic': state['received_monotonic']})
        event['status'] = 'sigterm_sent'
        deadline = time.monotonic() + 3.0
        while True:
            ensure_amp()
            current = read_process(child['pid'])
            if current is None or current['state'] == 'Z' or current['start_ticks'] != child['start_ticks']:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Pico/GMR未按时退出；不强杀、不创建新的参考发布器')
            time.sleep(0.05)
        event.update(status='stopped', stopped_at=time.time())
    return events
