"""Independent validation of native-video / fixed-rate MCAP episode exports."""
import bisect
from collections import Counter
import csv
import json
from pathlib import Path
import statistics
import subprocess
import time


def video_probe(path):
    result=subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
        '-show_entries','stream=nb_read_frames,width,height,codec_name,avg_frame_rate,duration',
        '-of','json',str(path)],capture_output=True,text=True,timeout=600)
    if result.returncode or result.stderr.strip():
        raise ValueError('Video decode/probe failed: '+result.stderr[-1000:])
    return json.loads(result.stdout)['streams'][0]


def audit_fixed_session(directory):
    from mcap.reader import make_reader
    from verify_real_recording import digest,stamp,describe,inspect_joint_payload
    root=Path(directory)
    errors,warnings=[],[]
    report={'session_dir':str(root),'checked_ns':time.time_ns(),'errors':errors,'warnings':warnings,'checks':{}}
    def require(condition,message):
        if not condition:errors.append(message)
    files=list((root/'Action_ObservationState').glob('*.mcap'))
    if len(files)!=1:
        errors.append('Expected exactly one MCAP')
        report['complete_and_aligned']=False
        return report
    mcap=files[0];vision=root/'vision'
    metadata=json.loads(mcap.with_suffix('.json').read_text())
    report.update(mcap=str(mcap),metadata_state=metadata.get('state'))
    require(metadata.get('state')=='closed','Episode metadata is not closed')
    names=['rgb.mp4','frame_timestamps.csv','alignment.csv','alignment_summary.json',
           'aligned_50hz.jsonl','aligned_50hz_schema.json','manifest.json','clock_sync.json',
           'camera_info.json','cleanup_report.json','vision_metadata.json',
           'raw/rgb.mp4','raw/frame_timestamps.csv','raw/manifest.json']
    missing=[n for n in names if not (vision/n).is_file()]
    if missing:
        errors.append('Missing files: '+', '.join(missing))
        report['complete_and_aligned']=False
        return report
    manifest=json.loads((vision/'manifest.json').read_text())
    for name,entry in manifest['files'].items():
        relative=Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts,'Unsafe manifest path')
        if relative.is_absolute() or '..' in relative.parts:continue
        path=vision/relative
        require(path.is_file() and path.stat().st_size==entry['size'] and digest(path)==entry['sha256'],
                'Manifest hash/size mismatch: '+name)
    report['checks']['vision_sha256']=not errors
    cleanup=metadata['cleanup']
    begin=metadata['start_button_ns']+int(cleanup['trim_start_seconds']*1e9)
    end=metadata['stop_boundary_ns']-int(cleanup['trim_end_seconds']*1e9)
    require(cleanup['requested_start_ns']==begin and cleanup['requested_end_ns']==end,'Incorrect head/tail trim boundaries')
    alignment=json.loads((vision/'alignment_summary.json').read_text())
    action_topic,state_topic=alignment['action_topic'],alignment['state_topic']
    actions,states,counts,dimensions,invalid_sources=[],[],Counter(),Counter(),Counter()
    before=mcap.stat()
    outside=Counter()
    try:
        with mcap.open('rb') as f:
            for schema,channel,message in make_reader(f,validate_crcs=True).iter_messages(log_time_order=False):
                counts[channel.topic]+=1
                t=stamp(message)
                if not begin<=t<=end:outside[channel.topic]+=1
                if channel.topic not in (action_topic,state_topic):continue
                is_action=channel.topic==action_topic
                dim,valid,q=inspect_joint_payload(message.data,heft=bool(
                    is_action and schema and schema.name=='leju::msgs::HeftReference'),return_values=True)
                target=actions if is_action else states
                valid=valid and dim==(alignment['action_dimensions'] if is_action else alignment['state_dimensions'])
                if not valid:invalid_sources[channel.topic]+=1
                if is_action:dimensions[dim]+=1
                target.append({'t':t,'i':len(target),'q':q,'valid':valid,'log':message.log_time})
        report['checks']['mcap_full_read_crc']=True
    except Exception as exc:
        report['checks']['mcap_full_read_crc']=False
        errors.append('MCAP parse/CRC failure: '+str(exc))
    after=mcap.stat()
    require((before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns),'MCAP changed during inspection')
    require(not outside,'MCAP messages outside trimmed boundaries')
    require(sum(counts.values())==metadata['message_count'],'MCAP total count differs from metadata')
    report.update(mcap_total_messages=sum(counts.values()),mcap_topic_counts=dict(counts),
                  messages_outside_trimmed_window=dict(outside),invalid_source_messages=dict(invalid_sources),
                  action_topic=action_topic,state_topic=state_topic,action_dimensions=dict(dimensions),
                  action_messages_total=len(actions),state_messages_total=len(states))
    actions.sort(key=lambda r:(r['t'],r['i']));states.sort(key=lambda r:(r['t'],r['i']))
    with (vision/'frame_timestamps.csv').open(newline='') as f:frames=list(csv.DictReader(f))
    with (vision/'raw/frame_timestamps.csv').open(newline='') as f:raw_frames=list(csv.DictReader(f))
    with (vision/'alignment.csv').open(newline='') as f:rows=list(csv.DictReader(f))
    with (vision/'aligned_50hz.jsonl').open() as f:numeric=[json.loads(line) for line in f]
    if not actions or not states or not frames or not rows or not raw_frames:
        errors.append('Empty action/state/RGB/grid stream')
        report['complete_and_aligned']=False
        return report
    fps=float(metadata['vision']['container_fps'])
    probes={}
    for name,selected in [('trimmed',frames),('raw',raw_frames)]:
        try:
            probe=video_probe(vision/('rgb.mp4' if name=='trimmed' else 'raw/rgb.mp4'))
            probes[name]=probe
            count=int(probe['nb_read_frames'])
            require(count==len(selected),name+' video frame count differs from timestamps')
            numerator,denominator=map(int,probe['avg_frame_rate'].split('/'))
            require(abs(numerator/denominator-fps)<.01,name+' video frame rate was changed')
        except Exception as exc:errors.append(str(exc))
    require(len(frames)==metadata['vision']['frame_count'],'Trimmed frame count differs from metadata')
    require(len(raw_frames)==metadata['vision']['raw_frame_count'],'Raw frame count differs from metadata')
    require([int(r['frame_index']) for r in frames]==list(range(len(frames))),'Trimmed frame indices not contiguous')
    require([int(r['frame_index']) for r in raw_frames]==list(range(len(raw_frames))),'Raw frame indices not contiguous')
    expected_frames=[r for r in raw_frames if begin<=int(r['estimated_lower_stamp_ns'])<=end]
    require(len(expected_frames)==len(frames),'Unused native RGB frames were removed or extra frames retained')
    for a,b in zip(expected_frames,frames):
        require(a['header_stamp_ns']==b['header_stamp_ns'] and
                a['estimated_lower_stamp_ns']==b['estimated_lower_stamp_ns'] and
                int(a['frame_index'])==int(b['original_frame_index']),'Trimmed frame is not the expected original frame')
    ft=[int(r['estimated_lower_stamp_ns']) for r in frames]
    rt=[int(r['header_stamp_ns']) for r in raw_frames]
    require(all(b>a for a,b in zip(ft,ft[1:])) and all(b>a for a,b in zip(rt,rt[1:])),'Nonincreasing camera timestamps')
    require(all(begin<=t<=end for t in ft),'Video frame outside head/tail trim')
    clock=json.loads((vision/'clock_sync.json').read_text())
    c0,c1=clock['start'],clock['end']
    u0,u1=c0['lower_ns']+c0['offset_ns'],c1['lower_ns']+c1['offset_ns']
    require(u1>u0 and c1['lower_ns']>c0['lower_ns'],'Invalid clock anchors')
    mapped=[c0['lower_ns']+(t-u0)*(c1['lower_ns']-c0['lower_ns'])//(u1-u0) for t in rt]
    mapping_error=max(abs(a-int(r['estimated_lower_stamp_ns'])) for a,r in zip(mapped,raw_frames))
    require(mapping_error<=1,'Clock mapping disagrees with saved timestamps')
    uncertainty=max(c0['uncertainty_ns'],c1['uncertainty_ns'])/1e6
    require(uncertainty<=5 and abs(c1['offset_ns']-c0['offset_ns'])<=50_000_000,'Clock quality failed')
    at,st=[r['t'] for r in actions],[r['t'] for r in states]
    hz=int(metadata['alignment_hz']);period=1_000_000_000//hz
    grid_start=begin+((max(begin,at[0],st[0],ft[0])-begin+period-1)//period)*period
    expected_times=list(range(grid_start,end+1,period))
    require(len(rows)==len(expected_times)==len(numeric),'Fixed-grid sample count or numeric row count differs')
    require(alignment['sample_period_ns']==period and alignment['grid_start_ns']==grid_start,'Grid definition differs from independent calculation')
    mismatch,matched,used=0,0,set()
    aa,sa,ra=[],[],[]
    for k,(row,t,values) in enumerate(zip(rows,expected_times,numeric)):
        ai,si,fi=bisect.bisect_right(at,t)-1,bisect.bisect_right(st,t)-1,bisect.bisect_right(ft,t)-1
        a,s=actions[ai],states[si]
        action_age,state_age,rgb_age=t-a['t'],t-s['t'],t-ft[fi]
        ok=(a['valid'] and s['valid'] and 0<=action_age<=alignment['max_action_age_ms']*1e6 and
            0<=state_age<=alignment['max_state_age_ms']*1e6 and 0<=rgb_age<=alignment['max_rgb_age_ms']*1e6)
        expected={'sample_index':k,'sample_timestamp_ns':t,'sample_relative_ns':t-grid_start,
                  'action_index':a['i'],'action_timestamp_ns':a['t'],'action_age_ns':action_age,
                  'action_mcap_log_time_ns':a['log'],'state_index':s['i'],'state_timestamp_ns':s['t'],
                  'state_age_ns':state_age,'rgb_frame_index':fi,'rgb_timestamp_lower_ns':ft[fi],
                  'rgb_age_ns':rgb_age,'matched':int(ok)}
        mismatch+=int(any(int(row[key])!=value for key,value in expected.items()) or
            values['sample_index']!=k or values['timestamp_ns']!=t or
            values['action_index']!=a['i'] or values['state_index']!=s['i'] or values['rgb_frame_index']!=fi or
            values['matched']!=ok or values['action']!=(a['q'] if ok else None) or
            values['observation_state']!=(s['q'] if ok else None))
        matched+=int(ok);used.add(fi)
        aa.append(action_age/1e6);sa.append(state_age/1e6);ra.append(rgb_age/1e6)
    require(mismatch==0,'Alignment/numeric values differ from independently recalculated MCAP matches')
    require(matched==len(rows),'Fixed grid contains unmatched samples')
    require(metadata['action_frames']==len(rows)==metadata['alignment_samples'],'Metadata sample count differs')
    require(alignment['alignment_samples']==len(rows) and alignment['matched_action_frames']==matched,'Alignment summary counts differ')
    require(digest(vision/'raw/rgb.mp4')==cleanup['source_video_sha256'],'Original raw RGB video bytes changed')
    raw_manifest=json.loads((vision/'raw/manifest.json').read_text())
    require(digest(vision/'raw/rgb.mp4')==raw_manifest['files']['rgb.mp4']['sha256'],'Raw video differs from upper transfer manifest')
    gaps=[b-a for a,b in zip(rt,rt[1:])]
    seq=[int(b['ros_seq'])-int(a['ros_seq'])-1 for a,b in zip(raw_frames,raw_frames[1:])]
    missing=sum(max(0,v) for v in seq)
    long_gaps=sum(v>1.5e9/fps for v in gaps)
    observed=(len(rt)-1)*1e9/(rt[-1]-rt[0])
    require(abs(observed/fps-1)<=.05,'Actual camera source rate differs from native FPS')
    if missing:warnings.append('Raw camera sequence has missing frames')
    if long_gaps:warnings.append('Raw camera timestamp gaps exceed 1.5 frame periods')
    report.update(alignment_mode='fixed_grid',alignment_hz=hz,sample_period_ns=period,
        alignment_samples=len(rows),action_frames=len(rows),matched_actions=matched,
        unmatched_actions=len(rows)-matched,alignment_recalculation_mismatch_rows=mismatch,
        video_frames=len(frames),timestamp_rows=len(frames),raw_video_frames=len(raw_frames),
        video_frames_used_by_samples=len(used),unused_native_frames_preserved=len(frames)-len(used),
        video=probes,clock_mapping_error_ns=mapping_error,clock_uncertainty_ms=uncertainty,
        clock_offset_change_ms=abs(c1['offset_ns']-c0['offset_ns'])/1e6,
        camera_observed_fps=observed,camera_seq_missing_frames=missing,
        camera_timestamp_gap_events=long_gaps,video_capture_interval_ms=describe([v/1e6 for v in gaps]),
        action_age_ms=describe(aa),state_age_ms=describe(sa),rgb_age_ms=describe(ra),
        trim_start_seconds=cleanup['trim_start_seconds'],trim_end_seconds=cleanup['trim_end_seconds'],
        retained_window_duration_s=(end-begin)/1e9,
        initial_grid_points_skipped=(grid_start-begin)//period)
    report['checks'].update(native_and_raw_video_preserved=True,
        video_frame_count_matches_csv=all(name in probes for name in ('raw','trimmed')),
        causal_alignment_independently_verified=mismatch==0,
        fixed_grid_verified=[int(r['sample_timestamp_ns']) for r in rows]==expected_times)
    report['complete_and_aligned']=not errors
    return report
