"""Compare read-only kernel snapshots; never infer motor freshness from DDS Hz."""
from copy import deepcopy
import time

ERRORS = ('rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped', 'bus_error',
          'error_warning', 'error_passive', 'bus_off')


def current_communication(model, remote_fresh=True, now_ns=None):
    now = time.time_ns() if now_ns is None else now_ns
    result = deepcopy(model or {})
    fresh = bool(remote_fresh and result.get('available') and
                 0 <= now-result.get('observed_ns', 0) < 7_000_000_000)
    result['fresh'] = fresh
    if not fresh:
        result.update(state='muted', summary='通信接口状态待确认')
        for row in result.get('can', [])+result.get('serial', []):
            row.update(state='muted', status='状态过期 / 待确认', rx_fps=None, tx_fps=None,
                       new_errors=None)
    return result


class CommunicationTracker:
    def __init__(self):
        self.previous = None
        self.activity = {}

    def observe(self, raw, now_ns=None):
        now = time.time_ns() if now_ns is None else now_ns
        if not raw or not raw.get('available'):
            return {'available': False, 'observed_ns': now, 'can': [], 'serial': [],
                    'errors': (raw or {}).get('errors', ['等待下位机接口探测'])}
        model = deepcopy(raw)
        model.update(observed_ns=now, fresh=True)
        previous = self.previous or {}
        delta_ns = raw['monotonic_ns']-previous.get('monotonic_ns', raw['monotonic_ns'])
        same_run = (raw.get('boot_id') == previous.get('boot_id') and
                    raw.get('hardware_running') == previous.get('hardware_running') and
                    .1e9 <= delta_ns <= 6e9)
        before = {r['interface']: r for r in previous.get('can', [])} if same_run else {}
        if not same_run:
            self.activity.clear()
        for row in model['can']:
            name, counts = row['interface'], row.get('counters', {})
            old = before.get(name, {})
            old_counts = old.get('counters', {})
            comparable = bool(counts and old_counts and old.get('ifindex') == row.get('ifindex') and
                              all(counts.get(k, 0) >= old_counts.get(k, 0) for k in counts))
            row.update(rx_fps=None, tx_fps=None, new_errors=None)
            if comparable:
                row['rx_fps'] = (counts['rx_packets']-old_counts['rx_packets'])*1e9/delta_ns
                row['tx_fps'] = (counts['tx_packets']-old_counts['tx_packets'])*1e9/delta_ns
                row['new_errors'] = {k: counts.get(k, 0)-old_counts.get(k, 0) for k in ERRORS}
            else:
                self.activity[name] = {'rx': raw['monotonic_ns'], 'tx': raw['monotonic_ns']}
            activity = self.activity.setdefault(name, {'rx': raw['monotonic_ns'], 'tx': raw['monotonic_ns']})
            for direction in ('rx', 'tx'):
                if (row.get(direction+'_fps') or 0) > 0:
                    activity[direction] = raw['monotonic_ns']
                row[direction+'_quiet_s'] = (raw['monotonic_ns']-activity[direction])/1e9
            if row.get('present') is None:
                state, status = 'muted', '无法确认接口'
            elif row.get('enabled') is False:
                state, status = 'muted', '配置已停用'
            elif not row['present']:
                state, status = 'bad', '接口不存在'
            elif not row.get('up') or row.get('kernel_state') in ('BUS-OFF', 'STOPPED', 'SLEEPING'):
                state, status = 'bad', '接口未运行' if not row.get('up') else '总线 '+row['kernel_state']
            elif not raw.get('hardware_running'):
                state, status = 'muted', '接口已开启，硬件程序未运行'
            elif not comparable:
                state, status = 'muted', '正在测量收发'
            elif row['rx_quiet_s'] >= 4:
                state, status = 'bad', '接收已停止'
            elif row['tx_quiet_s'] >= 4:
                state, status = 'bad', '发送已停止'
            elif not row['rx_fps'] or not row['tx_fps']:
                state, status = 'warn', '暂无新增接收' if not row['rx_fps'] else '暂无新增发送'
            elif any(row['new_errors'].values()) or row.get('tec', 0) or row.get('rec', 0):
                state, status = 'warn', '有收发，错误计数有提示'
            else:
                state, status = 'good', '有收发'
            row.update(state=state, status=status,
                       kernel_warning=row.get('kernel_state') in ('ERROR-WARNING', 'ERROR-PASSIVE'))
        for row in model.get('serial', []):
            if not row['connected']:
                state, status = ('bad' if row['required'] else 'muted'), '设备未连接'
            elif not row['owners_known']:
                state, status = 'muted', '已连接，占用状态待确认'
            elif row['required'] and row['used_by_hardware']:
                state, status = 'good', '硬件程序已打开'
            elif row['owners']:
                state, status = 'warn' if row['required'] else 'muted', '已被其他程序打开'
            else:
                state, status = 'warn' if row['required'] and raw.get('hardware_running') else 'muted', '已连接，未占用'
            row.update(state=state, status=status)
        enabled = [r for r in model['can'] if r.get('enabled') is not False]
        flowing = sum((r.get('rx_fps') or 0)>0 and (r.get('tx_fps') or 0)>0 for r in enabled)
        warnings = sum(r['kernel_warning'] for r in enabled)
        required_serial = [r for r in model.get('serial', []) if r.get('required')]
        model['summary'] = (f'CAN收发 {flowing}/{len(enabled)}' if raw.get('hardware_running') else
                            f'CAN接口 {len(enabled)}路 · 硬件未运行')
        if required_serial:
            imu_state = ('已打开' if all(r['state']=='good' for r in required_serial) else
                         '已连接（未占用）' if all(r['connected'] and r['owners_known'] and not r['owners'] for r in required_serial) else '待确认')
            model['summary'] += ' · IMU'+imu_state
        if warnings:
            model['summary'] += f' · {warnings}路总线有状态提示'
        if not raw.get('mapping_known'):
            model['summary'] += ' · 部位映射待确认'
        rows = enabled+required_serial
        model['state'] = 'bad' if any(r['state']=='bad' for r in rows) else 'warn' if warnings or not raw.get('mapping_known') or any(r['state']=='warn' for r in rows) else 'good' if rows and all(r['state']=='good' for r in rows) else 'muted'
        self.previous = deepcopy(raw)
        return model
