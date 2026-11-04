#!/usr/bin/env python3
"""Read-only independent audit of one finished real MCAP/RGB recording."""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import subprocess
import time


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def stamp(message):
    data = message.data
    if len(data) >= 12 and data[:2] in (b'\x00\x00', b'\x00\x01'):
        endian = '<' if data[1] else '>'
        sec, nsec = struct.unpack_from(endian + 'iI', data, 4)
        if 946684800 <= sec < 4102444800 and nsec < 1000000000:
            return sec * 1000000000 + nsec
    return message.publish_time or message.log_time


def describe(values):
    if not values:
        return None
    values = sorted(values)
    return {'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values) - 1, int(len(values) * .95))], 'max': values[-1]}


def inspect_joint_payload(data, heft=False, return_values=False):
    """Independent field walker, separate from the production action decoder."""
    endian, position = ('<' if data[1] else '>'), 4
    def read(fmt, alignment):
        nonlocal position
        position += (-(position - 4)) % alignment
        result = struct.unpack_from(endian + fmt, data, position)
        position += struct.calcsize(endian + fmt)
        return result
    read('iI', 4)
    valid = True
    if heft:
        version, = read('I', 4)
        length, = read('I', 4)
        robot = data[position:position + length].rstrip(b'\0').decode()
        position += length
        read('Q', 8)
        read('I', 4)
        read('q', 8)
        live, calibrated = read('BB', 1)
        read('7d', 8)
        valid = version == 1 and robot == 'roban_s17' and live == 1 and calibrated == 1
    size, = read('I', 4)
    if size > 1024:
        raise ValueError('Unreasonable joint sequence size')
    q = read(str(size) + 'd', 8) if size else ()
    valid = valid and all(math.isfinite(value) for value in q)
    return (len(q), valid, list(q)) if return_values else (len(q), valid)


def inspect(directory):
    from mcap.reader import make_reader
    errors, warnings = [], []
    report = {'session_dir': str(directory), 'checked_ns': time.time_ns(),
              'errors': errors, 'warnings': warnings, 'checks': {}}
    files = list((directory / 'Action_ObservationState').glob('*.mcap'))
    if len(files) != 1:
        errors.append('Expected exactly one MCAP; found %d' % len(files))
        return report
    mcap = files[0]
    metadata = json.loads(mcap.with_suffix('.json').read_text())
    if metadata.get('alignment_hz'):
        from verify_fixed_rate_recording import audit_fixed_session
        return audit_fixed_session(directory)
    report['metadata_state'] = metadata.get('state')
    report['mcap'] = str(mcap)
    if metadata.get('state') not in ('closed', 'error') or 'finalized_ns' not in metadata:
        errors.append('Recording has not finished finalizing; no final verdict yet')
        return report
    vision = directory / 'vision'
    required = ['rgb.mp4', 'frame_timestamps.csv', 'alignment.csv', 'alignment_summary.json',
                'clock_sync.json', 'camera_info.json', 'vision_metadata.json', 'manifest.json']
    report['files'] = {name: {'exists': (vision / name).exists(),
                             'bytes': (vision / name).stat().st_size if (vision / name).exists() else 0}
                       for name in required}
    missing = [name for name in required if not (vision / name).is_file()]
    if missing:
        errors.append('Missing vision files: ' + ', '.join(missing))
    if metadata.get('state') != 'closed':
        errors.append('Recorder metadata reports an incomplete/error segment')
    if missing:
        return report
    manifest = json.loads((vision / 'manifest.json').read_text())
    manifest_ok = True
    for name, expected in manifest['files'].items():
        path = vision / name
        if Path(name).name != name or not path.exists() or path.stat().st_size != expected['size'] or digest(path) != expected['sha256']:
            manifest_ok = False
            errors.append('Vision manifest verification failed: ' + name)
    report['checks']['vision_sha256'] = manifest_ok
    alignment_saved = json.loads((vision / 'alignment_summary.json').read_text())
    action_topic = alignment_saved['action_topic']
    state_topic = alignment_saved['state_topic']
    actions, states, topic_counts, dimensions = [], [], Counter(), Counter()
    invalid_actions, invalid_states = 0, 0
    outside_trim = Counter()
    cleanup = metadata.get('cleanup', {})
    crc_ok = True
    before = mcap.stat()
    try:
        with mcap.open('rb') as stream:
            for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                topic_counts[channel.topic] += 1
                if cleanup:
                    lower_bound = cleanup.get('requested_start_ns', metadata['effective_start_ns'])
                    upper_bound = cleanup.get('requested_end_ns', metadata['effective_end_ns'])
                    if not lower_bound <= stamp(message) <= upper_bound:
                        outside_trim[channel.topic] += 1
                if channel.topic in (action_topic, state_topic):
                    target = actions if channel.topic == action_topic else states
                    target.append((stamp(message), len(target), message.log_time))
                    if channel.topic == action_topic:
                        dim, valid = inspect_joint_payload(message.data, heft=bool(
                            schema and schema.name == 'leju::msgs::HeftReference'))
                        dimensions[dim] += 1
                        invalid_actions += int(not valid or dim != alignment_saved.get('action_dimensions', 21))
                    else:
                        dim, valid = inspect_joint_payload(message.data)
                        invalid_states += int(not valid or dim != 23)
    except Exception as exc:
        crc_ok = False
        errors.append('MCAP read/CRC failure: ' + str(exc))
    after = mcap.stat()
    if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
        errors.append('MCAP changed during inspection')
    report['checks']['mcap_full_read_crc'] = crc_ok
    report['mcap_total_messages'] = sum(topic_counts.values())
    report['mcap_topic_counts'] = dict(topic_counts)
    report['action_topic'] = action_topic
    report['state_topic'] = state_topic
    report['action_dimensions'] = dict(dimensions)
    report['invalid_action_messages'] = invalid_actions
    report['invalid_state_messages'] = invalid_states
    report['messages_outside_trimmed_window'] = dict(outside_trim)
    if outside_trim:
        errors.append('MCAP contains messages inside the discarded head/tail interval')
    if invalid_actions or invalid_states:
        errors.append('Invalid joint dimensions, nonfinite values or invalid reference flags')
    if report['mcap_total_messages'] != metadata.get('message_count'):
        errors.append('MCAP message total differs from metadata')
    start, stop = metadata['start_button_ns'], metadata['stop_boundary_ns']
    actions.sort()
    states.sort()
    selected = [row for row in actions if start <= row[0] <= stop]
    report['action_frames'] = len(selected)
    report['action_messages_total'] = len(actions)
    report['state_messages_total'] = len(states)
    report['button_window_duration_s'] = (stop - start) / 1e9
    if metadata.get('cleanup'):
        report['effective_episode_duration_s'] = (metadata['effective_end_ns'] - metadata['effective_start_ns']) / 1e9
        report['cleanup'] = metadata['cleanup']
    report['action_interval_ms'] = describe([(b[0] - a[0]) / 1e6 for a, b in zip(selected, selected[1:])])
    if not selected:
        errors.append('No actual action messages in button window')
    if metadata.get('action_frames') != len(selected):
        errors.append('Action frame count differs from metadata')
    with (vision / 'frame_timestamps.csv').open(newline='') as stream:
        frames = list(csv.DictReader(stream))
    with (vision / 'alignment.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    video = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
        '-show_entries', 'stream=nb_read_frames,width,height,codec_name,avg_frame_rate,duration',
        '-of', 'json', str(vision / 'rgb.mp4')], capture_output=True, text=True, timeout=600)
    if video.returncode or video.stderr.strip():
        errors.append('Video probe/decode error: ' + video.stderr[-2000:])
    probe = json.loads(video.stdout)['streams'][0]
    decoded = int(probe['nb_read_frames'])
    report['video'] = probe
    report['video_frames'] = decoded
    report['timestamp_rows'] = len(frames)
    report['checks']['video_frame_count_matches_csv'] = decoded == len(frames) == metadata['vision']['frame_count']
    if not report['checks']['video_frame_count_matches_csv']:
        errors.append('Video decoded frame count, timestamp rows and metadata do not match')
    if [int(row['frame_index']) for row in frames] != list(range(len(frames))):
        errors.append('Video frame indices are not contiguous from zero')
    upper = [int(row['header_stamp_ns']) for row in frames]
    lower = [int(row['estimated_lower_stamp_ns']) for row in frames]
    if any(b <= a for a, b in zip(upper, upper[1:])) or any(b <= a for a, b in zip(lower, lower[1:])):
        errors.append('Frame timestamps repeat or go backwards')
    if not lower or lower[0] < start or lower[-1] > stop:
        errors.append('Video timestamp range exceeds button window')
    if cleanup:
        lower_bound = cleanup.get('requested_start_ns', metadata['effective_start_ns'])
        upper_bound = cleanup.get('requested_end_ns', metadata['effective_end_ns'])
        if any(t < lower_bound or t > upper_bound for t in lower):
            errors.append('Video contains frames inside discarded head/tail interval')
        if any(int(row.get('relative_episode_ns', t - metadata['effective_start_ns'])) !=
               t - metadata['effective_start_ns'] for row, t in zip(frames, lower)):
            errors.append('Episode-relative frame timestamps disagree with retained window')
    gaps = [int(b['ros_seq']) - int(a['ros_seq']) - 1 for a, b in zip(frames, frames[1:])]
    intentional = [int(b.get('original_frame_index', b['frame_index'])) -
                   int(a.get('original_frame_index', a['frame_index'])) - 1
                   for a, b in zip(frames, frames[1:])]
    report['video_frames_intentionally_removed_between_retained_frames'] = sum(max(0, gap) for gap in intentional)
    report['camera_seq_missing_frames'] = sum(max(0, gap - removed) for gap, removed in zip(gaps, intentional))
    report['camera_seq_backwards_events'] = sum(1 for gap in gaps if gap < 0)
    report['video_capture_interval_ms'] = describe([(b - a) / 1e6 for a, b in zip(upper, upper[1:])])
    report['video_capture_intervals_over_100ms'] = sum(b - a > 100_000_000 for a, b in zip(upper, upper[1:]))
    if report['camera_seq_missing_frames']:
        warnings.append('Camera ROS sequence indicates missing frames')
    if report['camera_seq_backwards_events']:
        errors.append('Camera ROS sequence repeats or goes backwards')
    if report['video_capture_intervals_over_100ms']:
        warnings.append('Camera capture interval exceeds 100 ms; continuous ROS sequence does not prove uniform frame cadence')
    clock = json.loads((vision / 'clock_sync.json').read_text())
    c0, c1 = clock['start'], clock['end']
    u0, u1 = c0['lower_ns'] + c0['offset_ns'], c1['lower_ns'] + c1['offset_ns']
    mapped = [c0['lower_ns'] + (value - u0) * (c1['lower_ns'] - c0['lower_ns']) // (u1 - u0) for value in upper]
    mapping_error = max([abs(a - b) for a, b in zip(mapped, lower)] or [0])
    report['clock_mapping_error_ns'] = mapping_error
    report['clock_offset_start_ms'] = c0['offset_ns'] / 1e6
    report['clock_offset_end_ms'] = c1['offset_ns'] / 1e6
    report['clock_offset_change_ms'] = abs(c1['offset_ns'] - c0['offset_ns']) / 1e6
    report['clock_uncertainty_ms'] = max(c0['uncertainty_ns'], c1['uncertainty_ns']) / 1e6
    if mapping_error > 1:
        errors.append('Saved corrected frame timestamps disagree with clock anchors')
    state_times = [row[0] for row in states]
    matched, mismatch_rows, rgb_ages, state_ages = 0, 0, [], []
    used_frames = set()
    if len(rows) != len(selected):
        errors.append('Alignment row count differs from actual action count')
    for actual, row in zip(selected, rows):
        stamp_ns, action_index, log_ns = actual
        fi = bisect.bisect_right(lower, stamp_ns) - 1
        si = bisect.bisect_right(state_times, stamp_ns) - 1
        rgb_ok = fi >= 0 and stamp_ns - lower[fi] <= alignment_saved['max_rgb_age_ms'] * 1e6
        state_ok = si >= 0 and stamp_ns - state_times[si] <= alignment_saved['max_state_age_ms'] * 1e6
        expected_match = rgb_ok and state_ok
        mismatch = int(row['action_timestamp_ns']) != stamp_ns or int(row['action_index']) != action_index
        mismatch |= int(row['matched']) != int(expected_match)
        if rgb_ok:
            mismatch |= row['rgb_frame_index'] != str(fi) or int(row['rgb_timestamp_lower_ns']) != lower[fi]
            rgb_ages.append((stamp_ns - lower[fi]) / 1e6)
            used_frames.add(fi)
        elif row['rgb_frame_index']:
            mismatch = True
        if state_ok:
            mismatch |= int(row['state_index']) != states[si][1] or int(row['state_timestamp_ns']) != state_times[si]
            state_ages.append((stamp_ns - state_times[si]) / 1e6)
        elif row['state_index']:
            mismatch = True
        mismatch_rows += int(bool(mismatch))
        matched += int(expected_match)
    report['matched_actions'] = matched
    report['unmatched_actions'] = len(selected) - matched
    report['alignment_recalculation_mismatch_rows'] = mismatch_rows
    report['rgb_age_ms'] = describe(rgb_ages)
    report['state_age_ms'] = describe(state_ages)
    report['video_frames_used_by_actions'] = len(used_frames)
    report['checks']['causal_alignment_independently_verified'] = bool(selected) and mismatch_rows == 0
    if mismatch_rows:
        errors.append('Saved alignment table disagrees with independently recalculated matches')
    if matched != alignment_saved.get('matched_action_frames'):
        errors.append('Matched action count differs from alignment summary')
    if report['clock_uncertainty_ms'] > 5:
        errors.append('Clock exchange uncertainty exceeds configured 5 ms limit')
    if report['clock_offset_change_ms'] > 50:
        errors.append('Clock offset changes by more than configured 50 ms limit')
    if len(selected) and matched / len(selected) < alignment_saved['minimum_matched_fraction']:
        errors.append('Matched fraction below configured acceptance threshold')
    if report['unmatched_actions']:
        warnings.append('%d actions lack an acceptable image/state match' % report['unmatched_actions'])
    if metadata.get('cleanup') and (report['unmatched_actions'] or len(used_frames) != len(frames)):
        errors.append('Cleaned episode contains an unmatched action or unused video frame')
    report['complete_and_aligned'] = not errors
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    result = inspect(args.session.resolve())
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    concise = {key: value for key, value in result.items() if key not in ['mcap_topic_counts', 'files']}
    print(json.dumps(concise, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
