"""Preserve native camera frames and align actual MCAP samples to a fixed clock."""
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import time
import uuid

from vr_time_alignment import CDRFields, action_details, source_stamp

REVISION = 4


def timing_health(frames, fps):
    times = [int(r['header_stamp_ns']) for r in frames]
    gaps = [b-a for a,b in zip(times,times[1:])]
    seq = [int(b['ros_seq'])-int(a['ros_seq'])-1 for a,b in zip(frames,frames[1:])]
    period = 1e9/fps
    ordered = sorted(gaps)
    return {
        'frames': len(frames), 'expected_fps': fps,
        'observed_fps': (len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else None,
        'median_period_fps': 1e9/statistics.median(gaps) if gaps else None,
        'interval_ms': {'median':statistics.median(gaps)/1e6,
                        'p95':ordered[min(len(ordered)-1,int(len(ordered)*.95))]/1e6,
                        'max':max(gaps)/1e6} if gaps else None,
        'nonincreasing_timestamps': sum(g<=0 for g in gaps),
        'sequence_missing_frames': sum(max(0,g) for g in seq),
        'sequence_backwards_events': sum(g<0 for g in seq),
        'intervals_over_1_5_period': sum(g>1.5*period for g in gaps),
        'estimated_missing_by_timestamp': sum(max(0,round(g/period)-1) for g in gaps)}


def read_streams(path, config):
    from mcap.reader import make_reader
    actions, states = [], []
    with Path(path).open('rb') as f:
        for schema, channel, msg in make_reader(f, validate_crcs=True).iter_messages(
                topics=[config['action_topic'],config['state_topic']],log_time_order=False):
            if channel.topic == config['action_topic']:
                if config.get('action_kind') == 'heft_reference' and (
                        not schema or schema.name != 'leju::msgs::HeftReference'):
                    raise ValueError('Unexpected action schema')
                detail = action_details(msg,config)
                actions.append(dict(detail,t=source_stamp(msg),i=len(actions),log=msg.log_time))
            else:
                decoder = CDRFields(msg.data)
                decoder.scalar('i',4)
                decoder.scalar('I',4)
                q = decoder.vector()
                states.append({'t':source_stamp(msg),'i':len(states),'q':q,
                               'valid':len(q)==config.get('state_dimensions',23) and
                                       all(math.isfinite(v) for v in q)})
    actions.sort(key=lambda r:(r['t'],r['i']))
    states.sort(key=lambda r:(r['t'],r['i']))
    return actions,states


def build_fixed_alignment(session, config):
    actions,states = read_streams(session['mcap'],config)
    vision = Path(session['vision_dir'])
    with (vision/'frame_timestamps.csv').open(newline='') as f:
        frames = list(csv.DictReader(f))
    if not actions or not states or not frames:
        raise ValueError('No action, state or RGB data for fixed-rate alignment')
    hz = int(config['alignment_hz'])
    if hz <= 0 or 1_000_000_000 % hz:
        raise ValueError('Alignment rate must have an exact integer-nanosecond period')
    period = 1_000_000_000//hz
    begin = session['start_button_ns']+int(config.get('trim_start_seconds',5)*1e9)
    end = session['stop_boundary_ns']-int(config.get('trim_end_seconds',5)*1e9)
    at,st = [r['t'] for r in actions],[r['t'] for r in states]
    ft = [int(r['estimated_lower_stamp_ns']) for r in frames]
    first_support = max(begin,at[0],st[0],ft[0])
    grid_begin = begin + ((first_support-begin+period-1)//period)*period
    if grid_begin>end:
        raise ValueError('No supported fixed-rate samples after head/tail trim')
    limits = {k:int(config.get('max_'+k+'_age_ms',v)*1e6)
              for k,v in [('action',50),('state',50),('rgb',40)]}
    fields = ['sample_index','sample_timestamp_ns','sample_relative_ns',
              'action_index','action_timestamp_ns','action_age_ns','action_mcap_log_time_ns',
              'state_index','state_timestamp_ns','state_age_ns',
              'rgb_frame_index','rgb_timestamp_lower_ns','rgb_age_ns','matched','reason']
    matched,used,ages,reused,previous = 0,set(),{'action':[],'state':[],'rgb':[]},0,None
    rows = []
    with (vision/'aligned_50hz.jsonl').open('w') as samples:
        for index,t in enumerate(range(grid_begin,end+1,period)):
            ai,si,fi = bisect.bisect_right(at,t)-1,bisect.bisect_right(st,t)-1,bisect.bisect_right(ft,t)-1
            a,s = actions[ai] if ai>=0 else None,states[si] if si>=0 else None
            aa,sa,ra = t-a['t'] if a else None,t-s['t'] if s else None,t-ft[fi] if fi>=0 else None
            reasons = []
            for key,value,valid in [('action',aa,a and a['valid']),('state',sa,s and s['valid']),('rgb',ra,fi>=0)]:
                if not valid:reasons.append('missing_or_invalid_'+key)
                elif value>limits[key]:reasons.append('stale_'+key)
            ok = not reasons
            row = dict(sample_index=index,sample_timestamp_ns=t,sample_relative_ns=t-grid_begin,
                       action_index=a['i'] if a else '',action_timestamp_ns=a['t'] if a else '',
                       action_age_ns=aa,action_mcap_log_time_ns=a['log'] if a else '',
                       state_index=s['i'] if s else '',state_timestamp_ns=s['t'] if s else '',
                       state_age_ns=sa,rgb_frame_index=fi if fi>=0 else '',
                       rgb_timestamp_lower_ns=ft[fi] if fi>=0 else '',rgb_age_ns=ra,
                       matched=int(ok),reason=';'.join(reasons))
            rows.append(row)
            samples.write(json.dumps({'sample_index':index,'timestamp_ns':t,
                'timestamp_s':(t-grid_begin)/1e9,'matched':ok,'rgb_frame_index':fi,
                'action_index':a['i'] if a else None,'state_index':s['i'] if s else None,
                'action':a['q'] if ok else None,'observation_state':s['q'] if ok else None},
                allow_nan=False,separators=(',',':'))+'\n')
            if ok:
                matched+=1
                used.add(fi)
                for key,value in [('action',aa),('state',sa),('rgb',ra)]:ages[key].append(value/1e6)
                reused+=int(previous==a['i'])
                previous=a['i']
    with (vision/'alignment.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    def stats(values):
        return {'median_ms':statistics.median(values),'max_ms':max(values)} if values else None
    clock=json.loads((vision/'clock_sync.json').read_text())
    uncertainty=max(clock[k]['uncertainty_ns'] for k in ('start','end'))/1e6
    change=abs(clock['start']['offset_ns']-clock['end']['offset_ns'])/1e6
    clock_ok=uncertainty<=config.get('max_clock_uncertainty_ms',5) and change<=50
    result={'status':'aligned' if matched==len(rows) and clock_ok else 'error',
            'alignment_mode':'fixed_grid','alignment_hz':hz,'sample_period_ns':period,
            'grid_start_ns':grid_begin,'grid_end_ns':rows[-1]['sample_timestamp_ns'],
            'requested_start_ns':begin,'requested_end_ns':end,
            'initial_grid_points_skipped':(grid_begin-begin)//period,
            'action_topic':config['action_topic'],'state_topic':config['state_topic'],
            'action_kind':config.get('action_kind'),'action_field':'q',
            'action_dimensions':config.get('action_dimensions',21),
            'state_dimensions':config.get('state_dimensions',23),
            'action_semantics':config.get('action_semantics','Pico/GMR reference'),
            'action_messages_in_mcap':len(actions),'state_messages_in_mcap':len(states),
            'action_frames_in_button_window':len(rows),'alignment_samples':len(rows),
            'matched_action_frames':matched,'unmatched_action_frames':len(rows)-matched,
            'matched_fraction':matched/len(rows),'minimum_matched_fraction':1.0,
            'rgb_video_frames':len(frames),'used_rgb_frames':len(used),
            'unused_native_rgb_frames_preserved':len(frames)-len(used),
            'reused_action_rows':reused,'resampling':'causal_zero_order_hold',
            'matching_rule':'latest actual action/state/RGB at or before the fixed-grid sample timestamp',
            'max_action_age_ms':limits['action']/1e6,'max_state_age_ms':limits['state']/1e6,
            'max_rgb_age_ms':limits['rgb']/1e6,'action_age':stats(ages['action']),
            'state_age':stats(ages['state']),'rgb_age':stats(ages['rgb']),
            'clock_uncertainty_bound_ms':uncertainty,'clock_offset_change_ms':change,
            'clock_quality_accepted':clock_ok,
            'note':'Native RGB and MCAP timestamps are preserved. Repeated source actions are explicit bounded holds. '
                   'An unmatched interior sample fails the episode instead of breaking the fixed-rate grid.'}
    from vr_session_cleanup import json_write
    json_write(vision/'alignment_summary.json',result)
    json_write(vision/'aligned_50hz_schema.json',{
        'alignment_hz':hz,'units':'radians','action_topic':config['action_topic'],
        'action_field':'q','action_dimensions':21,'observation_state_dimensions':23,
        'state_order':'Original /rt/joint_state hardware order, including two head joints',
        'action_order':['waist_yaw_joint']+['leg_'+side+str(i)+'_joint' for side in ('l','r') for i in range(1,7)]
                       +['zarm_'+side+str(i)+'_joint' for side in ('l','r') for i in range(1,5)],
        'timestamp_origin_ns':str(grid_begin),'resampling':'causal_zero_order_hold',
        'source_index_rule':'indices refer to topic messages in the trimmed MCAP; indices may repeat'})
    return result


def make_manifest(vision):
    from vr_session_cleanup import digest,json_write
    entries={str(p.relative_to(vision)):{'size':p.stat().st_size,'sha256':digest(p)}
             for p in sorted(vision.rglob('*')) if p.is_file() and
             p!=vision/'manifest.json' and not any(part.startswith('.') for part in p.relative_to(vision).parts)}
    json_write(vision/'manifest.json',{'files':entries})
    return digest(vision/'manifest.json')


def finalize_fixed_rate_session(session,config,vision_metadata=None):
    from mcap.reader import make_reader
    from vr_session_cleanup import digest,json_write,rewrite_mcap,rewrite_video,verify_manifest
    root=Path(session['session_dir']).resolve()
    metadata=json.loads(Path(session['metadata']).read_text())
    signature=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
    previous=metadata.get('cleanup',{})
    update_keys=('message_count','topics','mcap_start_ns','mcap_end_ns','mcap_complete',
                 'effective_start_ns','effective_end_ns','cleanup','alignment_hz','alignment_samples')
    if previous.get('revision')==REVISION and previous.get('policy_signature')==signature:
        from verify_fixed_rate_recording import audit_fixed_session
        audit=audit_fixed_session(root)
        if not audit['complete_and_aligned']:raise ValueError('Existing fixed-rate episode failed audit')
        return dict(metadata['vision'],mcap_update={k:metadata[k] for k in update_keys})
    source_root=root
    if previous:
        source_root=Path(previous['source_backup']).resolve()
        if source_root.parent!=(root.parent/'.raw_backups').resolve():
            raise ValueError('Invalid raw source backup')
    source_mcap=source_root/'Action_ObservationState'/Path(session['mcap']).name
    source_vision=source_root/'vision'
    metadata=json.loads(source_mcap.with_suffix('.json').read_text())
    if not metadata.get('mcap_complete') or metadata.get('exit_code')!=0:
        raise ValueError('Native MCAP did not close successfully')
    if previous and digest(source_mcap)!=previous['source_mcap_sha256']:
        raise ValueError('Original raw MCAP changed')
    verify_manifest(source_vision)
    raw_vm=vision_metadata or json.loads((source_vision/'vision_metadata.json').read_text())
    expected=float(config.get('fps',60))
    if abs(float(raw_vm['container_fps'])-expected)>.5:
        raise ValueError('Native RGB FPS is %s, expected %s; refusing relabeling' %(raw_vm['container_fps'],expected))
    clock=json.loads((source_vision/'clock_sync.json').read_text())
    if max(clock[k]['uncertainty_ns'] for k in ('start','end'))>config.get('max_clock_uncertainty_ms',5)*1e6:
        raise ValueError('Clock uncertainty exceeds limit')
    if abs(clock['start']['offset_ns']-clock['end']['offset_ns'])>50_000_000:
        raise ValueError('Clock offset changed too much')
    start=session['start_button_ns']+int(config.get('trim_start_seconds',5)*1e9)
    stop=session['stop_boundary_ns']-int(config.get('trim_end_seconds',5)*1e9)
    if stop<=start:raise ValueError('Recording too short for head/tail trim')
    with (source_vision/'frame_timestamps.csv').open(newline='') as f:frames=list(csv.DictReader(f))
    raw_health=timing_health(frames,expected)
    if raw_health['nonincreasing_timestamps'] or raw_health['sequence_backwards_events']:
        raise ValueError('Camera source timestamps/sequence go backwards')
    if raw_health['observed_fps'] is None or abs(raw_health['observed_fps']/expected-1)>.05:
        raise ValueError('Actual RGB rate is not within 5 percent of expected native FPS')
    indices=[i for i,r in enumerate(frames) if start<=int(r['estimated_lower_stamp_ns'])<=stop]
    if not indices:raise ValueError('No camera frames survive head/tail trim')
    required=source_mcap.stat().st_size+(source_vision/'rgb.mp4').stat().st_size+128*1024*1024
    if shutil.disk_usage(root).free<required:
        raise OSError('Insufficient disk space for verified native-video finalization')
    identity=uuid.uuid4().hex[:8]
    stage=root.with_name('.'+root.name+'.fixed_'+identity)
    backup=root.parent/'.raw_backups'/(root.name+'_'+identity)
    moved=False
    try:
        shutil.copytree(source_root,stage,copy_function=os.link)
        sv=stage/'vision'
        raw=sv/'raw'
        raw.mkdir(exist_ok=True)
        for name in ('rgb.mp4','frame_timestamps.csv','vision_metadata.json','clock_sync.json','camera_info.json','manifest.json'):
            target=raw/name
            if not target.exists():os.link(source_vision/name,target)
        rows=[]
        for index,old in enumerate(indices):
            row=dict(frames[old])
            row['original_frame_index']=row.get('original_frame_index',row['frame_index'])
            row['frame_index']=index
            row['relative_episode_ns']=int(row['estimated_lower_stamp_ns'])-start
            rows.append(row)
        rewrite_video(source_vision/'rgb.mp4',sv/'rgb.mp4',indices,raw_vm['container_fps'])
        (sv/'frame_timestamps.csv').unlink()
        with (sv/'frame_timestamps.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
        staged_mcap=stage/'Action_ObservationState'/source_mcap.name
        source_actions,_=read_streams(source_mcap,config)
        rewrite_mcap(source_mcap,staged_mcap,{'effective_start_ns':start,'effective_end_ns':stop,
            'keep_action_indices':{a['i'] for a in source_actions},
            'invalid_state_indices':set()},config)
        for name in ('alignment.csv','alignment_summary.json','aligned_50hz.jsonl','aligned_50hz_schema.json'):
            if (sv/name).exists():(sv/name).unlink()
        alignment=build_fixed_alignment(dict(session,mcap=str(staged_mcap),vision_dir=str(sv)),config)
        if alignment['status']!='aligned':
            json_write(root/'vision/alignment_error.json',alignment)
            raise ValueError('Fixed 50 Hz alignment has %d unmatched samples; raw data retained'%
                             alignment['unmatched_action_frames'])
        from vr_hand_alignment import build_hand_alignment
        hand_alignment=build_hand_alignment(staged_mcap,sv,config)
        trim_health=timing_health(rows,expected)
        capture_health={'raw':raw_health,'trimmed':trim_health,
                        'queue_dropped_frames':raw_vm.get('queue_dropped_frames',0),
                        'note':'Sequence loss and long timestamp gaps are reported separately; no unused native RGB frame is deleted.'}
        capture_health['no_frame_loss_observed']=not(raw_health['sequence_missing_frames'] or
            raw_health['intervals_over_1_5_period'] or capture_health['queue_dropped_frames'])
        json_write(sv/'capture_health.json',capture_health)
        cleanup={'revision':REVISION,'policy_signature':signature,
                 'policy':'trim head/tail; preserve all native RGB and raw video; fixed-grid causal alignment',
                 'requested_start_ns':start,'requested_end_ns':stop,
                 'effective_start_ns':start,'effective_end_ns':stop,
                 'trim_start_seconds':config.get('trim_start_seconds',5),
                 'trim_end_seconds':config.get('trim_end_seconds',5),
                 'source_video_frames':len(frames),'kept_video_frames':len(rows),
                 'removed_video_frames':len(frames)-len(rows),
                 'source_action_messages':len(source_actions),'alignment_samples':alignment['alignment_samples'],
                 'source_mcap_sha256':digest(source_mcap),
                 'source_video_sha256':digest(source_vision/'rgb.mp4'),
                 'source_manifest_sha256':digest(source_vision/'manifest.json'),
                 'source_backup':str(backup if source_root==root else source_root),
                 'replaced_session_backup':str(backup),'raw_vision_dir':str(root/'vision/raw'),
                 'processed_ns':time.time_ns()}
        json_write(sv/'cleanup_report.json',cleanup)
        vm=dict(raw_vm,status='complete',frame_count=len(rows),raw_frame_count=len(frames),
                container_fps=raw_vm['container_fps'],native_frames_preserved=True,
                alignment_hz=int(config['alignment_hz']),alignment=alignment,cleanup=cleanup,
                capture_health=capture_health,hand_alignment=hand_alignment['summary'],
                destination=str(root/'vision'),transfer_verified=True,
                raw_video='raw/rgb.mp4',raw_timestamps='raw/frame_timestamps.csv',
                source_manifest_sha256=cleanup['source_manifest_sha256'],
                first_upper_stamp_ns=int(rows[0]['header_stamp_ns']),
                last_upper_stamp_ns=int(rows[-1]['header_stamp_ns']),
                first_lower_stamp_ns=int(rows[0]['estimated_lower_stamp_ns']),
                last_lower_stamp_ns=int(rows[-1]['estimated_lower_stamp_ns']))
        vm.pop('manifest_sha256',None)
        json_write(sv/'vision_metadata.json',vm)
        with staged_mcap.open('rb') as f:summary=make_reader(f).get_summary()
        updates={'message_count':summary.statistics.message_count,
                 'topics':sorted({c.topic for c in summary.channels.values()}),
                 'mcap_start_ns':summary.statistics.message_start_time,
                 'mcap_end_ns':summary.statistics.message_end_time,'mcap_complete':True,
                 'effective_start_ns':start,'effective_end_ns':stop,'cleanup':cleanup,
                 'alignment_hz':int(config['alignment_hz']),'alignment_samples':alignment['alignment_samples']}
        metadata.update(updates,vision=vm,action_frames=alignment['alignment_samples'],
                        state='closed',finalized_ns=time.time_ns())
        json_write(staged_mcap.with_suffix('.json'),metadata)
        make_manifest(sv)
        from verify_fixed_rate_recording import audit_fixed_session
        audit=audit_fixed_session(stage)
        if not audit['complete_and_aligned']:
            raise ValueError('Independent fixed-rate audit failed: '+'; '.join(audit['errors']))
        audit.update(session_dir=str(root),mcap=session['mcap'])
        json_write(sv/'audit_report.json',audit)
        vm['manifest_sha256']=make_manifest(sv)
        metadata['vision']=vm
        json_write(staged_mcap.with_suffix('.json'),metadata)
        backup.parent.mkdir(exist_ok=True)
        root.rename(backup);moved=True
        stage.rename(root);moved=False
        return dict(vm,mcap_update=updates)
    except Exception:
        if moved:backup.rename(root)
        if stage.exists():shutil.rmtree(stage)
        raise
