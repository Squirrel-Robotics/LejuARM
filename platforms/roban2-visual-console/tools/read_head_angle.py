#!/usr/bin/env python3
"""Read head encoder feedback from the observation console; never send commands."""
import argparse
from datetime import datetime
import json
import math
import time
import urllib.request


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def extract(first, second):
    current = second.get('channels', {}).get('joint', {})
    previous = first.get('channels', {}).get('joint', {})
    stamp = current.get('stamp')
    ages = [current.get('age_ms'), current.get('source_age_ms')]
    result = dict(status='UNAVAILABLE', source_topic='/rt/joint_state',
                  source_stamp=stamp, source_age_ms=current.get('source_age_ms'),
                  head_pitch_deg=None, head_yaw_deg=None,
                  angle_reference='current_motor_zero',
                  physical_down_from_horizontal_deg=None,
                  physical_zero_reference_verified=False, motion_commands_sent=0)
    if (current.get('status') not in ('live', 'stale')
            or not number(stamp) or stamp <= 0
            or any(not number(age) or not 0 <= age <= 1000 for age in ages)):
        result['reason'] = 'JOINT_FEEDBACK_MISSING_OR_STALE'
        return result
    if not number(previous.get('stamp')) or stamp <= previous['stamp']:
        result['reason'] = 'JOINT_SOURCE_NOT_ADVANCING'
        return result
    data = current.get('data') or {}
    names, positions = data.get('names'), data.get('q')
    if (not isinstance(names, list) or not isinstance(positions, list)
            or len(names) != 23 or len(positions) != 23
            or any(not isinstance(name, str) for name in names)
            or len(set(names)) != 23
            or any(not number(q) for q in positions)
            or any(name not in names for name in ('zhead_1_joint', 'zhead_2_joint'))):
        result['reason'] = 'INVALID_NAMED_JOINT_FEEDBACK'
        return result
    result.update(status='LIVE', head_pitch_deg=math.degrees(positions[names.index('zhead_2_joint')]),
                  head_yaw_deg=math.degrees(positions[names.index('zhead_1_joint')]))
    return result


def show(result):
    print(datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %z'))
    if result['status'] != 'LIVE':
        age = result.get('source_age_ms')
        suffix = '；最近关节数据已过去 %.1f 秒' % (age / 1000.) if number(age) else ''
        print('当前头部角度不可用：' + result.get('reason', 'UNKNOWN') + suffix)
        print('需要关节反馈恢复并持续更新；相机画面更新不代表关节反馈正常。', flush=True)
        return
    print('头部俯仰反馈（相对当前电机零位）：%+.2f°' % result['head_pitch_deg'])
    print('头部左右转角（相对当前电机零位）：%+.2f°' % result['head_yaw_deg'])
    print('反馈年龄：%.1f ms；已确认时间戳持续更新。' % result['source_age_ms'])
    print('重设零位后，此反馈不能直接当作相对水平面的真实俯角；须先恢复物理角度参考。', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', action='store_true', help='每秒刷新；Ctrl+C 退出')
    parser.add_argument('--json', action='store_true', help='输出结构化只读结果')
    args = parser.parse_args(argv)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def fetch():
        with opener.open('http://127.0.0.1:8765/api/state', timeout=3) as response:
            return json.load(response)

    try:
        while True:
            start = time.monotonic()
            try:
                first = fetch()
                time.sleep(.3)
                result = extract(first, fetch())
            except (OSError, ValueError, TypeError) as exc:
                result = dict(status='UNAVAILABLE', reason='CONSOLE_READ_FAILED: ' + str(exc),
                              physical_down_from_horizontal_deg=None, motion_commands_sent=0)
            if args.json:
                print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
            else:
                show(result)
            if not args.watch:
                return 0 if result['status'] == 'LIVE' else 2
            time.sleep(max(0., 1. - (time.monotonic() - start)))
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
