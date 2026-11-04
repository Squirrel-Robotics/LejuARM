"""Finalize an episode by retaining actual causal action/state/RGB matches.

Prepare and audit a complete sibling directory before publishing it. The original
session is retained under .raw_backups, including the original video and MCAP.
"""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import logging
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from vr_time_alignment import CDRFields, action_details, build_alignment, source_stamp

LOG = logging.getLogger('vr_cleanup')
REVISION = 2


def policy_signature(config):
    keys = ('action_topic', 'action_kind', 'action_dimensions', 'state_topic',
            'state_dimensions', 'max_rgb_age_ms', 'max_state_age_ms',
            'trim_start_seconds', 'trim_end_seconds', 'max_clock_uncertainty_ms')
    return hashlib.sha256(json.dumps({k: config.get(k) for k in keys},
                                    sort_keys=True).encode()).hexdigest()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_write(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.new')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def refresh_manifest(vision):
    entries = {p.name: {'sha256': digest(p), 'size': p.stat().st_size}
               for p in sorted(vision.iterdir())
               if p.is_file() and not p.name.startswith('.') and p.name != 'manifest.json'}
    json_write(vision / 'manifest.json', {'files': entries})
    return digest(vision / 'manifest.json')


def verify_manifest(vision):
    manifest = json.loads((vision / 'manifest.json').read_text())
    for name, entry in manifest['files'].items():
        if Path(name).name != name or name in ('.', '..'):
            raise ValueError('Invalid manifest filename')
        p = vision / name
        if p.stat().st_size != entry['size'] or digest(p) != entry['sha256']:
            raise ValueError('Source manifest verification failed: ' + name)


def plan_cleanup(session, config):
    from mcap.reader import make_reader
    vision = Path(session['vision_dir'])
    with (vision / 'frame_timestamps.csv').open(newline='') as f:
        frames = list(csv.DictReader(f))
    if not frames or [int(r['frame_index']) for r in frames] != list(range(len(frames))):
        raise ValueError('Source video frame indices must be contiguous from zero')
    frame_times = [int(row['estimated_lower_stamp_ns']) for row in frames]
    if any(b <= a for a, b in zip(frame_times, frame_times[1:])):
        raise ValueError('Video timestamps must be strictly increasing')
    action_topic, state_topic = config['action_topic'], config['state_topic']
    button_start, button_stop = session['start_button_ns'], session['stop_boundary_ns']
    trim_start = int(config.get('trim_start_seconds', 0) * 1e9)
    trim_end = int(config.get('trim_end_seconds', 0) * 1e9)
    if trim_start < 0 or trim_end < 0:
        raise ValueError('Trim seconds must be nonnegative')
    start, stop = button_start + trim_start, button_stop - trim_end
    if stop <= start:
        raise ValueError('Recording too short for head/tail trim: duration %.3fs, requested %.3fs + %.3fs' %
                         ((button_stop - button_start) / 1e9, trim_start / 1e9, trim_end / 1e9))
    eligible_frames = [i for i, t in enumerate(frame_times) if start <= t <= stop]
    eligible_frame_times = [frame_times[i] for i in eligible_frames]
    if not eligible_frames:
        raise ValueError('No video remains after head/tail trim')
    actions, states, invalid_states = [], [], set()
    counters = Counter()
    with Path(session['mcap']).open('rb') as f:
        for schema, channel, message in make_reader(f, validate_crcs=True).iter_messages(
                topics=[action_topic, state_topic], log_time_order=False):
            topic = channel.topic
            index = counters[topic]
            counters[topic] += 1
            stamp = source_stamp(message)
            if topic == action_topic:
                if config.get('action_kind') == 'heft_reference' and (
                        not schema or schema.name != 'leju::msgs::HeftReference'):
                    raise ValueError('Configured Pico action has unexpected MCAP schema')
                detail = action_details(message, config)
                actions.append(dict(detail, stamp=stamp, index=index))
            else:
                reader = CDRFields(message.data)
                reader.scalar('i', 4)
                reader.scalar('I', 4)
                q = reader.vector()
                import math
                if len(q) != config.get('state_dimensions', 23) or any(not math.isfinite(v) for v in q):
                    invalid_states.add(index)
                else:
                    states.append((stamp, index))
    if not actions or not states:
        raise ValueError('No configured action or valid state data; cannot create an aligned episode')
    actions.sort(key=lambda row: (row['stamp'], row['index']))
    states.sort()
    states = [r for r in states if start <= r[0] <= stop]
    state_times = [r[0] for r in states]
    max_rgb = int(config.get('max_rgb_age_ms', 100) * 1e6)
    max_state = int(config.get('max_state_age_ms', 50) * 1e6)
    keep, used_frames, used_states, dropped = [], set(), [], []
    seen = set()
    in_window = 0
    for row in actions:
        t = row['stamp']
        eligible_fi = bisect.bisect_right(eligible_frame_times, t) - 1
        fi = eligible_frames[eligible_fi] if eligible_fi >= 0 else -1
        si = bisect.bisect_right(state_times, t) - 1
        ra = t - frame_times[fi] if fi >= 0 else None
        sa = t - state_times[si] if si >= 0 else None
        reason = row['reason']
        if button_start <= t <= button_stop:
            in_window += 1
        if not button_start <= t <= button_stop:
            reason = 'outside_button_window'
        elif t < start:
            reason = 'trim_start'
        elif t > stop:
            reason = 'trim_end'
        else:
            if t in seen:
                reason = 'duplicate_action_timestamp'
            elif not reason and (ra is None or ra > max_rgb):
                reason = 'no_rgb_within_age_limit'
            elif not reason and (sa is None or sa > max_state):
                reason = 'no_state_within_age_limit'
        seen.add(t)
        if reason:
            dropped.append({'source_action_index': row['index'], 'action_timestamp_ns': t,
                            'reference_seq': row['seq'], 'reason': reason,
                            'rgb_age_ns': ra, 'state_age_ns': sa})
        else:
            keep.append(row)
            used_frames.add(fi)
            used_states.append(state_times[si])
    if not keep:
        raise ValueError('All actions lack a valid causal RGB/state match; source data retained')
    used_frames = sorted(used_frames)
    effective_start = min(frame_times[used_frames[0]], keep[0]['stamp'], min(used_states))
    effective_end = max(frame_times[used_frames[-1]], keep[-1]['stamp'], max(used_states))
    if effective_start < start or effective_end > stop:
        raise ValueError('Source frame/state dependency falls outside button window')
    return {'keep_action_indices': {r['index'] for r in keep},
            'invalid_state_indices': invalid_states, 'keep_frame_indices': used_frames,
            'frames': frames, 'dropped': dropped,
            'effective_start_ns': effective_start, 'effective_end_ns': effective_end,
            'first_action_ns': keep[0]['stamp'], 'last_action_ns': keep[-1]['stamp'],
            'requested_start_ns': start, 'requested_end_ns': stop,
            'trim_start_seconds': trim_start / 1e9, 'trim_end_seconds': trim_end / 1e9,
            'source_action_messages': len(actions), 'source_actions_in_button_window': in_window,
            'kept_actions': len(keep), 'source_video_frames': len(frames),
            'kept_video_frames': len(used_frames)}


def rewrite_mcap(source, destination, plan, config):
    from mcap.reader import make_reader
    from mcap.writer import Writer
    destination.unlink()
    counters, schema_ids, channel_ids = Counter(), {}, {}
    with source.open('rb') as src, destination.open('wb') as dst:
        reader = make_reader(src, validate_crcs=True)
        header = reader.get_header()
        writer = Writer(dst, enable_data_crcs=True)
        writer.start(profile=header.profile, library='vr-session-cleanup/1')
        for schema, channel, message in reader.iter_messages(log_time_order=False):
            index = counters[channel.topic]
            counters[channel.topic] += 1
            t = source_stamp(message)
            if not plan['effective_start_ns'] <= t <= plan['effective_end_ns']:
                continue
            if channel.topic == config['action_topic'] and index not in plan['keep_action_indices']:
                continue
            if channel.topic == config['state_topic'] and index in plan['invalid_state_indices']:
                continue
            if channel.id not in channel_ids:
                sid = 0
                if schema:
                    if schema.id not in schema_ids:
                        schema_ids[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
                    sid = schema_ids[schema.id]
                channel_ids[channel.id] = writer.register_channel(
                    channel.topic, channel.message_encoding, sid, channel.metadata)
            writer.add_message(channel_ids[channel.id], message.log_time, message.data,
                               message.publish_time, message.sequence)
        for metadata in reader.iter_metadata():
            writer.add_metadata(metadata.name, metadata.metadata)
        for attachment in reader.iter_attachments():
            writer.add_attachment(attachment.create_time, attachment.log_time, attachment.name,
                                  attachment.media_type, attachment.data)
        writer.finish()


def video_selection(indices):
    groups = []
    a = b = indices[0]
    for n in indices[1:]:
        if n == b + 1:
            b = n
        else:
            groups.append((a, b))
            a = b = n
    groups.append((a, b))
    return '+'.join('between(n,%d,%d)' % (a, b) for a, b in groups)


def rewrite_video(source, destination, indices, fps):
    destination.unlink()
    expression = "select='%s',setpts=N/(%s*TB)" % (video_selection(indices), fps)
    result = subprocess.run([
        'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
        '-progress', 'pipe:1', '-nostats',
        '-threads', '2', '-filter_threads', '1', '-i', str(source),
        # The selected images already have exact N/fps PTS. Explicit CFR gives
        # FFmpeg 7 a duration for the final packet too. The independent audit
        # requires exactly one encoded frame per selected source timestamp.
        '-vf', expression, '-fps_mode', 'cfr', '-r', str(fps), '-an', '-c:v', 'libx264',
        '-threads', '2', '-preset', 'veryfast', '-crf', '18',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(destination)
    ], capture_output=True, text=True, timeout=600)
    if result.returncode:
        raise RuntimeError('Video pruning failed: ' + result.stderr[-2000:])
    changes = [int(line.split('=', 1)[1]) for line in result.stdout.splitlines()
               if line.startswith(('dup_frames=', 'drop_frames='))]
    if not changes or any(changes):
        raise RuntimeError('Video encoder duplicated/dropped frames or did not report frame preservation')


def finalize_aligned_session(session, config, vision_metadata=None):
    """Validate a staged replacement, then publish it with a recoverable backup."""
    if config.get('alignment_hz'):
        from vr_fixed_rate_alignment import finalize_fixed_rate_session
        return finalize_fixed_rate_session(session, config, vision_metadata)
    root = Path(session['session_dir']).resolve()
    vision = root / 'vision'
    metadata = json.loads(Path(session['metadata']).read_text())
    signature = policy_signature(config)
    previous = metadata.get('cleanup', {})
    if previous.get('revision') == REVISION and previous.get('policy_signature') == signature:
        from verify_real_recording import inspect
        audit = inspect(root)
        if not audit.get('complete_and_aligned'):
            raise ValueError('Previously cleaned episode no longer passes audit')
        result = dict(metadata['vision'])
        result['mcap_update'] = {k: metadata[k] for k in (
            'message_count', 'topics', 'mcap_start_ns', 'mcap_end_ns',
            'effective_start_ns', 'effective_end_ns', 'cleanup', 'mcap_complete')}
        return result
    source_root = root
    if previous:
        source_root = Path(previous['source_backup']).resolve()
        if source_root.parent != (root.parent / '.raw_backups').resolve():
            raise ValueError('Previous raw source backup is outside this recording root')
        if not source_root.is_dir():
            raise ValueError('Original raw backup is unavailable; refusing repeated trimming')
    source_mcap = source_root / 'Action_ObservationState' / Path(session['mcap']).name
    source_vision = source_root / 'vision'
    if previous and digest(source_mcap) != previous['source_mcap_sha256']:
        raise ValueError('Original raw backup hash changed')
    metadata = json.loads(source_mcap.with_suffix('.json').read_text())
    if not metadata.get('mcap_complete') or metadata.get('exit_code') != 0:
        raise ValueError('Native MCAP did not close successfully')
    verify_manifest(source_vision)
    clock = json.loads((source_vision / 'clock_sync.json').read_text())
    uncertainty = max(clock[k]['uncertainty_ns'] for k in ('start', 'end'))
    if uncertainty > config.get('max_clock_uncertainty_ms', 5) * 1e6:
        raise ValueError('Clock uncertainty exceeds acceptance limit')
    if abs(clock['end']['offset_ns'] - clock['start']['offset_ns']) > 50_000_000:
        raise ValueError('Clock offset changed too much')
    source_session = dict(session, mcap=str(source_mcap), vision_dir=str(source_vision))
    plan = plan_cleanup(source_session, config)
    required = source_mcap.stat().st_size + (source_vision / 'rgb.mp4').stat().st_size
    if shutil.disk_usage(root).free < required + 128 * 1024 * 1024:
        raise OSError('Insufficient free space for staged cleanup')
    identity = uuid.uuid4().hex[:8]
    stage = root.with_name('.' + root.name + '.cleanup_' + identity)
    backup = root.parent / '.raw_backups' / (root.name + '_' + identity)
    source_hash = digest(source_mcap)
    source_manifest_hash = digest(source_vision / 'manifest.json')
    moved = False
    try:
        # Unmodified files share inodes; all writes below unlink or replace first.
        shutil.copytree(source_root, stage, copy_function=os.link)
        sv = stage / 'vision'
        staged_mcap = stage / 'Action_ObservationState' / Path(session['mcap']).name
        rewrite_mcap(source_mcap, staged_mcap, plan, config)
        raw_vm = vision_metadata or json.loads((source_vision / 'vision_metadata.json').read_text())
        rewrite_video(source_vision / 'rgb.mp4', sv / 'rgb.mp4', plan['keep_frame_indices'],
                      raw_vm.get('container_fps', config.get('fps', 15)))
        rows = []
        for index, original in enumerate(plan['keep_frame_indices']):
            row = dict(plan['frames'][original])
            row['original_frame_index'] = row.get('original_frame_index', row['frame_index'])
            row['frame_index'] = index
            row['relative_episode_ns'] = int(row['estimated_lower_stamp_ns']) - plan['effective_start_ns']
            rows.append(row)
        (sv / 'frame_timestamps.csv').unlink()
        with (sv / 'frame_timestamps.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        drop_path = sv / 'dropped_actions.csv'
        if drop_path.exists():
            drop_path.unlink()
        with drop_path.open('w', newline='') as f:
            fields = ['source_action_index', 'action_timestamp_ns', 'reference_seq',
                      'reason', 'rgb_age_ns', 'state_age_ns']
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(plan['dropped'])
        dropped_counts = Counter(row['reason'] for row in plan['dropped'])
        cleanup = {k: v for k, v in plan.items() if k not in (
            'keep_action_indices', 'invalid_state_indices', 'keep_frame_indices', 'frames', 'dropped')}
        cleanup.update(revision=REVISION, policy_signature=signature,
                       policy='trim head/tail by original button times, then drop unmatched actions and unused RGB',
                       action_topic=config['action_topic'], action_kind=config.get('action_kind'),
                       action_semantics=config.get('action_semantics'),
                       removed_video_frames=plan['source_video_frames'] - len(rows),
                       removed_action_messages=len(plan['dropped']), drop_reasons=dict(dropped_counts),
                       dropped_invalid_states=len(plan['invalid_state_indices']),
                       source_mcap_sha256=source_hash, source_manifest_sha256=source_manifest_hash,
                       source_backup=str(backup if source_root == root else source_root),
                       replaced_session_backup=str(backup), processed_ns=time.time_ns())
        json_write(sv / 'cleanup_report.json', cleanup)
        staged_session = dict(session, mcap=str(staged_mcap), vision_dir=str(sv))
        for name in ('alignment.csv', 'alignment_summary.json'):
            if (sv / name).exists():
                (sv / name).unlink()
        strict = dict(config, min_match_fraction=1.0)
        alignment = build_alignment(staged_session, strict)
        if alignment['status'] != 'aligned' or alignment['unmatched_action_frames']:
            raise ValueError('Pruned episode still contains unmatched actions')
        vm = dict(raw_vm, frame_count=len(rows), first_upper_stamp_ns=int(rows[0]['header_stamp_ns']),
                  last_upper_stamp_ns=int(rows[-1]['header_stamp_ns']),
                  first_lower_stamp_ns=int(rows[0]['estimated_lower_stamp_ns']),
                  last_lower_stamp_ns=int(rows[-1]['estimated_lower_stamp_ns']),
                  raw_frame_count=plan['source_video_frames'], status='complete',
                  destination=str(vision), transfer_verified=True, cleanup=cleanup, alignment=alignment)
        vm.pop('manifest_sha256', None)
        vm['source_manifest_sha256'] = source_manifest_hash
        json_write(sv / 'vision_metadata.json', vm)
        from mcap.reader import make_reader
        with staged_mcap.open('rb') as f:
            summary = make_reader(f).get_summary()
        updates = dict(message_count=summary.statistics.message_count,
                       topics=sorted({c.topic for c in summary.channels.values()}),
                       mcap_start_ns=summary.statistics.message_start_time,
                       mcap_end_ns=summary.statistics.message_end_time, mcap_complete=True,
                       effective_start_ns=plan['effective_start_ns'], effective_end_ns=plan['effective_end_ns'],
                       cleanup=cleanup)
        metadata.update(updates, vision=vm, action_frames=plan['kept_actions'],
                        state='closed', finalized_ns=time.time_ns())
        stage_meta = staged_mcap.with_suffix('.json')
        json_write(stage_meta, metadata)
        refresh_manifest(sv)
        from verify_real_recording import inspect
        audit = inspect(stage)
        if not audit.get('complete_and_aligned') or audit.get('unmatched_actions'):
            raise ValueError('Independent staged audit failed: ' + '; '.join(audit['errors']))
        audit.update(session_dir=str(root), mcap=session['mcap'])
        json_write(sv / 'audit_report.json', audit)
        vm['manifest_sha256'] = refresh_manifest(sv)
        metadata['vision'] = vm
        json_write(stage_meta, metadata)
        backup.parent.mkdir(exist_ok=True)
        root.rename(backup)
        moved = True
        stage.rename(root)
        moved = False
        LOG.info('Pruned episode: actions=%d RGB=%d; source retained in %s',
                 plan['kept_actions'], len(rows), backup)
        return dict(vm, mcap_update=updates)
    except Exception:
        if moved:
            backup.rename(root)
        if stage.exists():
            shutil.rmtree(stage)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    root = args.session.resolve()
    files = list((root / 'Action_ObservationState').glob('*.mcap'))
    if len(files) != 1:
        raise ValueError('Expected one MCAP')
    session = json.loads(files[0].with_suffix('.json').read_text())
    # Refuse to alter an episode still owned by a running recorder.
    status_path = root.parent / 'vr_mcap_status.json'
    if status_path.exists():
        status = json.loads(status_path.read_text())
        if status.get('session') and status['session'].get('session_dir') == str(root):
            if Path('/proc', str(status['pid'])).exists():
                raise RuntimeError('Session is still recording/finalizing')
    result = finalize_aligned_session(session, json.loads(args.config.read_text()))
    if status_path.exists():
        status = json.loads(status_path.read_text())
        if (status.get('last_session') or {}).get('session_dir') == str(root):
            status['last_session'] = json.loads(files[0].with_suffix('.json').read_text())
            status.update(updated_ns=time.time_ns(), error=None)
            json_write(status_path, status)
    print(json.dumps({k: result[k] for k in ('status', 'frame_count', 'alignment', 'cleanup')}, indent=2))


if __name__ == '__main__':
    main()
