#!/usr/bin/env python3
"""Read-only CAN/serial inventory. Never opens a serial port or sends CAN frames."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

CONFIG = Path('/home/lab/.config/lejuconfig/canbus_device_cofig.yaml')


def joint_index(name):
    for prefix, start, count in [('Lleg_joint_', 0, 6), ('Rleg_joint_', 6, 6),
                                 ('Larm_joint_', 13, 4), ('Rarm_joint_', 17, 4)]:
        if name.startswith(prefix) and name[len(prefix):].isdigit():
            n = int(name[len(prefix):])
            return start+n-1 if 1 <= n <= count else None
    return {'Waist_joint_01': 12, 'Head_yaw': 21, 'Head_pitch': 22}.get(name)


def interface_mapping(config):
    result = []
    for key, entry in config.get('canbus_interfaces', {}).items():
        if entry.get('type', '').upper() != 'SOCKETCAN':
            continue
        devices, parts, joints = [], {}, []
        for device in config.get(key+'_devices', []):
            if device.get('ignore', False):
                continue
            name = device.get('name', '')
            devices.append({k: device.get(k) for k in ('name', 'class', 'device_id')})
            index = joint_index(name)
            if index is not None:
                joints.append(index)
            part = next((label for prefix, label in [('Lleg_', '左腿'), ('Rleg_', '右腿'),
                ('Waist_', '腰'), ('Larm_', '左臂'), ('Rarm_', '右臂'), ('Head_', '头部'),
                ('Lhand_', '左手'), ('Rhand_', '右手')] if name.startswith(prefix)), '其他')
            parts[part] = parts.get(part, 0)+1
        result.append({'interface': entry.get('name', key), 'config_key': key,
            'enabled': entry.get('enable', True), 'devices': devices,
            'joint_indices': sorted(joints), 'body': ' + '.join(
                label+'（末端）' if label in ('左手', '右手') else label+str(count)
                for label, count in parts.items()) or '配置未标注部位'})
    return result


def device_owners(path):
    # fuser inspects existing file descriptors; opening an IMU serial port to
    # inspect it could change DTR or consume feedback, so do not do that.
    if not re.fullmatch(r'/dev/tty(?:USB|ACM)\d+', path):
        return {'owners_known': False, 'owners': []}
    command = ([] if os.geteuid() == 0 else ['sudo', '-n'])+['/usr/bin/fuser', path]
    try:
        p = subprocess.run(command, capture_output=True, text=True, timeout=.8,
                           env=dict(os.environ, LC_ALL='C'))
        if p.returncode not in (0, 1) or (p.returncode == 1 and p.stderr.strip()):
            return {'owners_known': False, 'owners': []}
        owners = []
        for pid in re.findall(r'\b\d+\b', p.stdout):
            try:
                name = (Path('/proc')/pid/'comm').read_text().strip()
            except OSError:
                name = '进程已退出'
            owners.append({'pid': int(pid), 'name': name})
        return {'owners_known': True, 'owners': owners}
    except (OSError, subprocess.TimeoutExpired):
        return {'owners_known': False, 'owners': []}


def serial_inventory(hardware_pids, dev_root=Path('/dev')):
    rows, by_path = [], {}
    imu = dev_root/'HIPNUC_IMU'
    if imu.is_symlink() or imu.exists():
        path = str(imu.resolve())
        by_path[path] = {'id': 'imu', 'label': 'HIPNUC IMU', 'path': path,
            'alias': str(imu), 'body': '姿态传感器（IMU）', 'required': True}
    for link in sorted((dev_root/'serial/by-id').glob('*')):
        path = str(link.resolve())
        if path in by_path:
            by_path[path]['usb_id'] = link.name
            continue
        match = re.search(r'CANable\d+', link.name)
        label = match.group(0) if match else link.name.split('_')[1] if '_' in link.name else 'USB串口'
        by_path[path] = {'id': link.name, 'label': label, 'path': path, 'usb_id': link.name,
            'body': 'USB转CAN，当前SocketCAN关节配置未使用' if match else '未确认部位',
            'required': False}
    # Keep the required sensor visible when it is unplugged.
    if not any(r['required'] for r in by_path.values()):
        by_path[str(imu)] = {'id': 'imu', 'label': 'HIPNUC IMU', 'path': str(imu),
            'alias': str(imu), 'body': '姿态传感器（IMU）', 'required': True}
    for path, row in by_path.items():
        row.update(connected=Path(path).exists())
        row.update(device_owners(path) if row['connected'] else {'owners_known': True, 'owners': []})
        row['used_by_hardware'] = any(x['pid'] in hardware_pids for x in row['owners'])
        rows.append(row)
    return sorted(rows, key=lambda r: (not r['required'], r['label']))


def snapshot(hardware_pids=(), config_path=CONFIG):
    started = time.monotonic_ns()
    errors, mapping, config_hash = [], [], None
    try:
        import yaml
        content = config_path.read_bytes()
        mapping = interface_mapping(yaml.safe_load(content) or {})
        config_hash = hashlib.sha256(content).hexdigest()
    except (OSError, ValueError, ImportError) as exc:
        errors.append('无法确认部位配置：'+str(exc)[:160])
    try:
        p = subprocess.run(['ip', '-j', '-d', '-s', 'link', 'show', 'type', 'can'],
            check=True, capture_output=True, text=True, timeout=1)
        links = {row['ifname']: row for row in json.loads(p.stdout)}
        can_available = True
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        links, can_available = {}, False
        errors.append('无法读取CAN接口状态：'+str(exc)[:160])
    configured = {row['interface'] for row in mapping}
    for name in sorted(set(links)-configured):
        mapping.append({'interface': name, 'enabled': None, 'body': '配置未标注部位',
                        'devices': [], 'joint_indices': []})
    for row in mapping:
        link = links.get(row['interface'])
        row.update(present=bool(link) if can_available else None)
        if not link:
            continue
        info, stats = link.get('linkinfo', {}), link.get('stats64', link.get('stats', {}))
        data, extended = info.get('info_data', {}), info.get('info_xstats', {})
        row.update(ifindex=link['ifindex'], up='UP' in link['flags'],
            kernel_state=data.get('state', 'UNKNOWN'), kind='CAN FD' if link['mtu'] == 72 else 'CAN',
            bitrate=data.get('bittiming', {}).get('bitrate'),
            data_bitrate=data.get('data_bittiming', {}).get('bitrate'),
            tec=data.get('berr_counter', {}).get('tx'), rec=data.get('berr_counter', {}).get('rx'))
        row['counters'] = {direction+'_'+key: stats.get(direction, {}).get(key, 0)
            for direction in ('rx', 'tx') for key in ('packets', 'errors', 'dropped')}
        row['counters'].update({key: extended.get(key, 0) for key in
            ('restarts', 'bus_error', 'arbitration_lost', 'error_warning', 'error_passive', 'bus_off')})
    try:
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        boot = None
    return {'schema_version': 1, 'available': can_available, 'now_ns': time.time_ns(),
        'monotonic_ns': time.monotonic_ns(), 'boot_id': boot, 'hardware_running': bool(hardware_pids),
        'config_path': str(config_path), 'config_sha256': config_hash, 'mapping_known': bool(config_hash),
        'can': mapping, 'serial': serial_inventory(set(hardware_pids)), 'errors': errors,
        'probe_ms': (time.monotonic_ns()-started)/1e6}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, default=CONFIG)
    args = p.parse_args()
    print(json.dumps(snapshot(config_path=args.config), ensure_ascii=False, allow_nan=False))
