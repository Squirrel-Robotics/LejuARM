"""Timestamp mapping and explicit, causal MCAP-to-video correspondences."""
import bisect
import csv
import json
import math
import statistics
import struct
from pathlib import Path


class CDRFields:
    """Plain CDR field decoder; alignment is relative to the encapsulation."""
    def __init__(self, data):
        if data[:2] not in (b'\0\0', b'\0\1'):
            raise ValueError('Expected plain CDR payload')
        self.data, self.pos = data, 4
        self.endian = '<' if data[1] else '>'

    def scalar(self, fmt, size):
        self.pos += (-(self.pos - 4)) % size
        value = struct.unpack_from(self.endian + fmt, self.data, self.pos)[0]
        self.pos += size
        return value

    def string(self):
        size = self.scalar('I', 4)
        if size > len(self.data) - self.pos:
            raise ValueError('Truncated CDR string')
        result = self.data[self.pos:self.pos + size].rstrip(b'\0').decode()
        self.pos += size
        return result

    def vector(self):
        size = self.scalar('I', 4)
        if size > 1024:
            raise ValueError('Unexpected joint sequence size')
        return [self.scalar('d', 8) for _ in range(size)]


def action_details(message, config):
    """Read the selected action's actual q, including HeftReference validity."""
    reader = CDRFields(message.data)
    reader.scalar('i', 4)
    reader.scalar('I', 4)
    valid, reason, seq = True, '', None
    if config.get('action_kind') == 'heft_reference':
        version = reader.scalar('I', 4)
        robot = reader.string()
        seq = reader.scalar('Q', 8)
        reader.scalar('I', 4)  # calibration epoch
        reader.scalar('q', 8)  # Pico device clock; never used as lower clock
        live_valid = reader.scalar('?', 1)
        calibrated = reader.scalar('?', 1)
        for _ in range(7):
            reader.scalar('d', 8)
        q = reader.vector()
        reader.vector()  # qd
        if reader.pos != len(message.data):
            raise ValueError('Unexpected trailing HeftReference fields')
        valid = version == 1 and robot == 'roban_s17' and live_valid and calibrated
        if not valid:
            reason = 'invalid_or_uncalibrated_reference'
    else:
        q = reader.vector()
    if len(q) != config.get('action_dimensions', 21):
        valid, reason = False, 'wrong_action_dimensions'
    elif any(not math.isfinite(value) for value in q):
        valid, reason = False, 'nonfinite_action'
    return {'valid': valid, 'reason': reason, 'dimensions': len(q), 'seq': seq, 'q': q}


def map_upper_to_lower(stamp, start, end):
    """Affine mapping through two measured clock anchors; integer nanoseconds."""
    upper0 = start['lower_ns'] + start['offset_ns']
    upper1 = end['lower_ns'] + end['offset_ns']
    if upper1 <= upper0 or end['lower_ns'] <= start['lower_ns']:
        return int(stamp - start['offset_ns'])
    return start['lower_ns'] + ((int(stamp) - upper0) *
        (end['lower_ns'] - start['lower_ns'])) // (upper1 - upper0)


def source_stamp(message):
    # Deployed DDS types begin with int32 header_sec, uint32 header_nanosec.
    data = message.data
    if len(data) >= 12 and data[:2] in (b'\x00\x00', b'\x00\x01'):
        endian = '<' if data[1] else '>'
        sec, nsec = struct.unpack_from(endian + 'iI', data, 4)
        if 946684800 <= sec < 4102444800 and 0 <= nsec < 1000000000:
            return sec * 1000000000 + nsec
    return message.publish_time or message.log_time


def build_alignment(session, config):
    if config.get('alignment_hz'):
        from vr_fixed_rate_alignment import build_fixed_alignment
        return build_fixed_alignment(session, config)
    from mcap.reader import make_reader
    vision = Path(session['vision_dir'])
    action_topic = config.get('action_topic', '/rt/pico/heft_reference')
    state_topic = config.get('state_topic', '/rt/joint_state')
    action, state = [], []
    with Path(session['mcap']).open('rb') as stream:
        for _, channel, message in make_reader(stream).iter_messages(
                topics=[action_topic, state_topic], log_time_order=False):
            target = action if channel.topic == action_topic else state
            if channel.topic == action_topic and config.get('action_kind'):
                detail = action_details(message, config)
                if not detail['valid']:
                    raise ValueError('Invalid action survived cleanup: ' + detail['reason'])
            target.append((source_stamp(message), len(target), message.log_time))
    action.sort()
    state.sort()
    state_times = [row[0] for row in state]
    with (vision / 'frame_timestamps.csv').open(newline='') as stream:
        frames = list(csv.DictReader(stream))
    frame_times = [int(row['estimated_lower_stamp_ns']) for row in frames]
    if any(b <= a for a, b in zip(frame_times, frame_times[1:])):
        raise ValueError('Video timestamps are not strictly increasing')
    max_rgb_age = int(config.get('max_rgb_age_ms', 100) * 1e6)
    max_state_age = int(config.get('max_state_age_ms', 50) * 1e6)
    begin, end = session['start_button_ns'], session['stop_boundary_ns']
    fields = ['action_index', 'action_timestamp_ns', 'action_mcap_log_time_ns',
              'state_index', 'state_timestamp_ns', 'state_age_ns',
              'rgb_frame_index', 'rgb_timestamp_lower_ns', 'rgb_age_ns', 'matched']
    count, matched, rgb_ages, state_ages = 0, 0, [], []
    with (vision / 'alignment.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for stamp, index, log_stamp in action:
            if not begin <= stamp <= end:
                continue
            count += 1
            row = dict.fromkeys(fields, '')
            row.update(action_index=index, action_timestamp_ns=stamp,
                       action_mcap_log_time_ns=log_stamp)
            si, fi = bisect.bisect_right(state_times, stamp) - 1, bisect.bisect_right(frame_times, stamp) - 1
            state_ok = si >= 0 and stamp - state_times[si] <= max_state_age
            rgb_ok = fi >= 0 and stamp - frame_times[fi] <= max_rgb_age
            if state_ok:
                age = stamp - state_times[si]
                row.update(state_index=state[si][1], state_timestamp_ns=state_times[si], state_age_ns=age)
                state_ages.append(age / 1e6)
            if rgb_ok:
                age = stamp - frame_times[fi]
                row.update(rgb_frame_index=frames[fi]['frame_index'],
                           rgb_timestamp_lower_ns=frame_times[fi], rgb_age_ns=age)
                rgb_ages.append(age / 1e6)
            row['matched'] = int(state_ok and rgb_ok)
            matched += row['matched']
            writer.writerow(row)
    def stats(values):
        return {'median_ms': statistics.median(values), 'max_ms': max(values)} if values else None
    clock = json.loads((vision / 'clock_sync.json').read_text())
    uncertainty = max(clock['start']['uncertainty_ns'], clock['end']['uncertainty_ns'])
    offset_change = abs(clock['start']['offset_ns'] - clock['end']['offset_ns'])
    timing_ok = uncertainty <= int(config.get('max_clock_uncertainty_ms', 5) * 1e6) and offset_change <= 50_000_000
    fraction = matched / count if count else 0.0
    minimum_fraction = config.get('min_match_fraction', .95)
    result = {
        'status': 'aligned' if fraction >= minimum_fraction and timing_ok else 'error',
        'action_topic': action_topic, 'state_topic': state_topic,
        'action_kind': config.get('action_kind', 'float64_array'),
        'action_field': 'q' if config.get('action_kind') == 'heft_reference' else 'data',
        'action_dimensions': config.get('action_dimensions', 21),
        'action_semantics': config.get('action_semantics', 'configured joint target'),
        'action_messages_in_mcap': len(action), 'state_messages_in_mcap': len(state),
        'action_frames_in_button_window': count, 'rgb_video_frames': len(frames),
        'matched_action_frames': matched, 'unmatched_action_frames': count - matched,
        'matched_fraction': fraction, 'minimum_matched_fraction': minimum_fraction,
        'matching_rule': 'latest state and RGB at or before action; no future frames',
        'max_rgb_age_ms': max_rgb_age / 1e6, 'max_state_age_ms': max_state_age / 1e6,
        'rgb_age': stats(rgb_ages), 'state_age': stats(state_ages),
        'clock_uncertainty_bound_ms': uncertainty / 1e6,
        'clock_offset_change_ms': offset_change / 1e6,
        'clock_quality_accepted': timing_ok,
        'note': 'Clock uncertainty is measured transport uncertainty, not a guarantee of exposure timing. '
                'Unmatched rows remain explicit; no images or actions are invented.',
    }
    (vision / 'alignment_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result
