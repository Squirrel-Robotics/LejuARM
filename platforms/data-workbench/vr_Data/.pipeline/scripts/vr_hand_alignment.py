"""Six channels per hand, causally aligned to the existing body/RGB 50 Hz grid.

This adds an immutable versioned export; original body arrays and media are untouched.
Positions retain the Leju interface's 0..100 opening/closing scale, never radians.
"""
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import statistics
import struct
import uuid
from vr_time_alignment import CDRFields,source_stamp
from vr_session_storage import write_json,sync_dir

REVISION=1
COMMAND_TOPIC='/rt/hand_cmd'
STATE_TOPIC='/rt/hand_state'
SIDES=('left','right')
VERSION=re.compile(r'hand_alignment/v1_[0-9a-f]{64}\Z')
FIELDS=['hand_command_index','hand_command_timestamp_ns','hand_command_age_ns',
        'hand_state_index','hand_state_timestamp_ns','hand_state_message_age_ns',
        'left_hand_driver_valid','left_hand_sample_age_ms','left_hand_effective_age_ns',
        'left_hand_age_upper_bound_ns','left_hand_matched','left_hand_reason',
        'right_hand_driver_valid','right_hand_sample_age_ms','right_hand_effective_age_ns',
        'right_hand_age_upper_bound_ns','right_hand_matched','right_hand_reason',
        'required_hands_matched','training_matched']


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def clean(values):return [v if not isinstance(v,float) or math.isfinite(v) else None for v in values] if values is not None else None

def side_values(row,key,side):
    values=row.get(key) if row else None
    return values[side*6:(side+1)*6] if values is not None and len(values)==12 else None


def finite(values,bounded=False):
    return values is not None and len(values)==6 and all(isinstance(v,(int,float)) and math.isfinite(v) and (not bounded or 0<=v<=100) for v in values)


def policy_config(config=None,required_sides=None):
    config=config or {}
    required=required_sides if required_sides is not None else config.get('hand_required_sides',['left','right'])
    if isinstance(required,str):required=[required]
    aliases={'L':'left','R':'right','left':'left','right':'right'}
    if not isinstance(required,list) or not required or any(not isinstance(s,str) or s not in aliases for s in required):raise ValueError('Required hand sides must be L/R or left/right')
    required=list(dict.fromkeys(aliases[s] for s in required))
    command=float(config.get('max_hand_command_age_ms',50));state=float(config.get('max_hand_state_age_ms',50))
    if not math.isfinite(command) or not math.isfinite(state) or command<=0 or state<=0:raise ValueError('Invalid hand age thresholds')
    return dict(revision=REVISION,command_topic=COMMAND_TOPIC,state_topic=STATE_TOPIC,dimensions_per_hand=6,
        command_dimensions=12,state_dimensions=12,required_sides=required,max_command_age_ms=command,
        max_state_age_ms=state,sample_age_quantization_bound_ns=1_000_000,
        matching_rule='latest source message at or before the existing 50Hz grid; never search past an invalid newer message',
        feedback_age_rule='grid minus hand_state header stamp plus driver sample_age_ms; validity uses an additional 1ms quantization upper bound',
        resampling='causal_zero_order_hold',units='percent_open_close_0_100',zero='open',hundred='closed')


def read_hand_streams(path):
    from mcap.reader import make_reader
    commands,states=[],[]
    schemas={};decode_errors=Counter()
    with Path(path).open('rb') as f:
        for schema,channel,msg in make_reader(f,validate_crcs=True).iter_messages(topics=[COMMAND_TOPIC,STATE_TOPIC],log_time_order=False):
            target=commands if channel.topic==COMMAND_TOPIC else states
            row=dict(t=source_stamp(msg),i=len(target),log=msg.log_time,decode_valid=False)
            try:
                expected='leju::msgs::Float64Array' if target is commands else 'leju::msgs::HandState'
                if not schema or schema.name!=expected or channel.message_encoding!='cdr':raise ValueError('unexpected_schema_or_encoding')
                schemas[channel.topic]=dict(name=schema.name,sha256=hashlib.sha256(schema.data).hexdigest())
                d=CDRFields(msg.data);sec=d.scalar('i',4);ns=d.scalar('I',4)
                if not 946684800<=sec<4102444800 or not 0<=ns<1_000_000_000:raise ValueError('invalid_source_header_stamp')
                row['t']=sec*1_000_000_000+ns
                if target is commands:row['q']=d.vector()
                else:
                    flags=[d.scalar('B',1),d.scalar('B',1)]
                    if any(v not in (0,1) for v in flags):raise ValueError('invalid_boolean_flag')
                    row.update(flags=[bool(v) for v in flags],sample_age_ms=[d.scalar('I',4),d.scalar('I',4)],
                        q=d.vector(),velocity=d.vector(),current=d.vector())
                    count=d.scalar('I',4)
                    if count>1024:raise ValueError('invalid_status_dimensions')
                    row['state_code']=[d.scalar('B',1) for _ in range(count)]
                if d.pos!=len(msg.data):raise ValueError('unexpected_trailing_fields')
                row['decode_valid']=True
            except (ValueError,struct.error,UnicodeError) as exc:
                row['decode_error']=str(exc);decode_errors[channel.topic]+=1
            target.append(row)
    health={}
    for name,rows in [('command',commands),('state',states)]:
        nonincreasing=sum(b['t']<=a['t'] for a,b in zip(rows,rows[1:]));times=[r['t'] for r in rows]
        health[name]=dict(messages=len(rows),source_time_order_ok=not nonincreasing,
            nonincreasing_source_timestamps=nonincreasing,decode_errors=sum(not r['decode_valid'] for r in rows),
            observed_publisher_hz=(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
            max_header_interval_ms=max((b-a)/1e6 for a,b in zip(times,times[1:])) if len(times)>1 else None,
            dimensions=dict(Counter(str(len(r.get('q',[]))) for r in rows)))
        rows.sort(key=lambda r:(r['t'],r['i']))
    health['schemas']=schemas
    return commands,states,health


def select_hand(t,commands,states,ct,st,policy,health):
    ci=bisect.bisect_right(ct,t)-1;si=bisect.bisect_right(st,t)-1
    command=commands[ci] if ci>=0 else None;state=states[si] if si>=0 else None
    ca=t-command['t'] if command else None;sa=t-state['t'] if state else None
    result=dict(command_index=command['i'] if command else None,command_timestamp_ns=command['t'] if command else None,
        command_age_ns=ca,state_index=state['i'] if state else None,state_timestamp_ns=state['t'] if state else None,state_message_age_ns=sa)
    for side,name in enumerate(SIDES):
        reasons=[];cq=side_values(command,'q',side);sq=side_values(state,'q',side)
        velocity=side_values(state,'velocity',side);current=side_values(state,'current',side);codes=side_values(state,'state_code',side)
        command_ok=bool(command and command['decode_valid'] and finite(cq,True))
        if not command_ok:reasons.append('missing_or_invalid_hand_command')
        elif not health['command']['source_time_order_ok']:command_ok=False;reasons.append('nonmonotonic_hand_command')
        elif not 0<=ca<=policy['max_command_age_ms']*1e6:command_ok=False;reasons.append('stale_hand_command')
        flag=state.get('flags',[None,None])[side] if state else None
        age=state.get('sample_age_ms',[None,None])[side] if state else None
        effective=sa+age*1_000_000 if sa is not None and age is not None else None
        upper=effective+policy['sample_age_quantization_bound_ns'] if effective is not None else None
        state_ok=bool(state and state['decode_valid'] and finite(sq,True) and finite(velocity) and finite(current) and codes is not None)
        if not state_ok:reasons.append('missing_or_invalid_hand_state')
        elif not flag:state_ok=False;reasons.append('hand_driver_invalid')
        elif not health['state']['source_time_order_ok']:state_ok=False;reasons.append('nonmonotonic_hand_state')
        elif upper is None or not 0<=upper<=policy['max_state_age_ms']*1e6:state_ok=False;reasons.append('stale_hand_feedback')
        result[name]=dict(action=clean(cq) if command_ok else None,observation_state=clean(sq) if state_ok else None,
            command_valid=command_ok,state_valid=state_ok,driver_valid=flag,sample_age_ms=age,
            effective_age_ns=effective,age_upper_bound_ns=upper,
            estimated_measurement_stamp_ns=state['t']-age*1_000_000 if state and age is not None else None,
            raw_command=clean(cq),raw_position=clean(sq),velocity_native=clean(velocity),current_native=clean(current),
            state_code=codes,matched=command_ok and state_ok,reason=';'.join(reasons))
    result['required_matched']=all(result[s]['matched'] for s in policy['required_sides'])
    result['both_matched']=all(result[s]['matched'] for s in SIDES)
    return result


def timing_columns(hands,body_matched):
    row=dict(hand_command_index=hands['command_index'],hand_command_timestamp_ns=hands['command_timestamp_ns'],
        hand_command_age_ns=hands['command_age_ns'],hand_state_index=hands['state_index'],
        hand_state_timestamp_ns=hands['state_timestamp_ns'],hand_state_message_age_ns=hands['state_message_age_ns'],
        required_hands_matched=int(hands['required_matched']),training_matched=int(body_matched and hands['required_matched']))
    for side in SIDES:
        value=hands[side]
        row.update({side+'_hand_driver_valid':int(value['driver_valid']) if value['driver_valid'] is not None else None,
            side+'_hand_sample_age_ms':value['sample_age_ms'],side+'_hand_effective_age_ns':value['effective_age_ns'],
            side+'_hand_age_upper_bound_ns':value['age_upper_bound_ns'],side+'_hand_matched':int(value['matched']),side+'_hand_reason':value['reason']})
    return row


def _range(values):
    return [dict(min=min(column),max=max(column)) if column else None for column in values]


def _stats(values):
    return dict(median_ms=statistics.median(values)/1e6,max_ms=max(values)/1e6) if values else None


def _schema(body_schema,policy):
    return dict(schema_version=1,body_schema=body_schema,alignment_hz=50,body_fields_unchanged=True,
        hands=dict(policy=policy,channel_order=dict(left_source_indices=list(range(6)),right_source_indices=list(range(6,12)),
            left_channels=['left_hand_channel_'+str(i+1) for i in range(6)],right_channels=['right_hand_channel_'+str(i+1) for i in range(6)]),
            position_units='0..100: 0=open, 100=closed; not radians or degrees',
            velocity_current_units='native driver values, retained without an unverified physical-unit conversion',
            state_code_semantics='native SDK uint8 status codes retained; a nonzero code is not assumed to be a communication fault',
            sample_age_ms='driver-reported age, included in feedback freshness; publisher frequency is not sensor refresh frequency',
            invalid_value_rule='invalid or stale training action/state is null; raw values and reasons remain for inspection; no interpolation or zero filling',
            zero_rule='a valid stationary zero is retained; zero alone is not an invalidity indicator',
            training_rule='use only rows where hands.training_matched is true; select the configured side(s); no implicit 21+12 concatenation',
            source_index_rule='zero-based topic-message indices in the trimmed MCAP; holds may repeat indices'))


def build_hand_alignment(mcap,vision,config=None,required_sides=None):
    mcap,vision=Path(mcap),Path(vision);policy=policy_config(config,required_sides)
    source=dict(mcap_sha256=digest(mcap),body_jsonl_sha256=digest(vision/'aligned_50hz.jsonl'),
        body_alignment_sha256=digest(vision/'alignment.csv'))
    key=hashlib.sha256(json.dumps(dict(policy=policy,source=source),sort_keys=True).encode()).hexdigest()
    relative='hand_alignment/v1_'+key;destination=vision/relative
    previous=load_hand_index(vision)
    if previous and previous.get('version_directory')==relative:
        verify_hand_manifest(vision,previous);return previous
    commands,states,health=read_hand_streams(mcap);ct=[r['t'] for r in commands];st=[r['t'] for r in states]
    with (vision/'aligned_50hz.jsonl').open() as f:body=[json.loads(l) for l in f]
    with (vision/'alignment.csv').open(newline='') as f:reader=csv.DictReader(f);fields=list(reader.fieldnames);times=list(reader)
    if not body or len(body)!=len(times):raise ValueError('Body alignment and numeric rows differ')
    if any(r['sample_index']!=i or r['timestamp_ns']!=int(times[i]['sample_timestamp_ns']) or (i and r['timestamp_ns']-body[i-1]['timestamp_ns']!=20_000_000) for i,r in enumerate(body)):
        raise ValueError('Hands require the unchanged continuous 50Hz body grid')
    parent=vision/'hand_alignment'
    if parent.is_symlink() or destination.is_symlink():raise ValueError('Unsafe hand alignment directory')
    parent.mkdir(exist_ok=True);stage=parent/('.'+key+'.'+uuid.uuid4().hex+'.tmp');stage.mkdir()
    try:
        counts={side:Counter() for side in SIDES};matched={s:0 for s in SIDES};ages={s:[] for s in SIDES};bounds={s:[] for s in SIDES}
        driver_invalid=Counter();body_matched=0;required_matched=0;training_matched=0
        with (stage/'aligned_50hz_with_hands.jsonl').open('w') as numeric,(stage/'alignment.csv').open('w',newline='') as timing:
            writer=csv.DictWriter(timing,fieldnames=fields+FIELDS);writer.writeheader()
            for row,time_row in zip(body,times):
                hands=select_hand(row['timestamp_ns'],commands,states,ct,st,policy,health)
                hands['training_matched']=bool(row['matched'] and hands['required_matched']);values=dict(row,hands=hands)
                numeric.write(json.dumps(values,allow_nan=False,separators=(',',':'))+'\n')
                writer.writerow(dict(time_row,**timing_columns(hands,row['matched'])))
                body_matched+=bool(row['matched']);required_matched+=hands['required_matched'];training_matched+=hands['training_matched']
                for side in SIDES:
                    value=hands[side];matched[side]+=value['matched'];counts[side].update(filter(None,value['reason'].split(';')))
                    if value['driver_valid'] is False:driver_invalid[side]+=1
                    if value['effective_age_ns'] is not None:ages[side].append(value['effective_age_ns'])
                    if value['age_upper_bound_ns'] is not None:bounds[side].append(value['age_upper_bound_ns'])
            for stream in (numeric,timing):stream.flush();os.fsync(stream.fileno())
        summary=dict(version=1,policy=policy,source=source,source_health=health,alignment_hz=50,alignment_samples=len(body),
            grid_start_ns=body[0]['timestamp_ns'],grid_end_ns=body[-1]['timestamp_ns'],body_matched_samples=body_matched,
            required_matched_samples=required_matched,training_matched_samples=training_matched,
            unmatched_required_samples=len(body)-required_matched,training_complete_and_aligned=training_matched==len(body),
            required_sides=policy['required_sides'],sides={})
        for si,side in enumerate(SIDES):
            cq=[[] for _ in range(6)];sq=[[] for _ in range(6)]
            for rows,key,columns in [(commands,'q',cq),(states,'q',sq)]:
                for row in rows:
                    values=side_values(row,key,si)
                    if values is not None:
                        for i,v in enumerate(values):
                            if math.isfinite(v):columns[i].append(v)
            summary['sides'][side]=dict(matched_samples=matched[side],unmatched_samples=len(body)-matched[side],
                driver_invalid_samples=driver_invalid[side],reasons=dict(counts[side]),effective_age=_stats(ages[side]),
                age_upper_bound=_stats(bounds[side]),command_ranges=_range(cq),position_ranges=_range(sq),
                raw_state_codes=dict(Counter(str(v) for row in states for v in (side_values(row,'state_code',si) or []))))
        body_schema=json.loads((vision/'aligned_50hz_schema.json').read_text())
        write_json(stage/'summary.json',summary);write_json(stage/'schema.json',_schema(body_schema,policy))
        audit=audit_hand_alignment(mcap,vision,stage,policy)
        if not audit['integrity_passed']:raise ValueError('Hand alignment independent audit failed: '+'; '.join(audit['errors']))
        write_json(stage/'audit_report.json',audit)
        entries={p.name:dict(size=p.stat().st_size,sha256=digest(p)) for p in sorted(stage.iterdir()) if p.is_file()}
        write_json(stage/'manifest.json',dict(files=entries))
        if destination.exists():
            # Only the same immutable source/policy key can use this name.
            verify_hand_manifest(vision,dict(version_directory=relative,manifest_sha256=digest(stage/'manifest.json')))
            shutil.rmtree(stage)
        else:stage.rename(destination);sync_dir(parent)
        index=dict(version=1,version_directory=relative,manifest_sha256=digest(destination/'manifest.json'),summary=summary,
            files={name:relative+'/'+name for name in entries},integrity_passed=True,
            training_complete_and_aligned=summary['training_complete_and_aligned'])
        if digest(mcap)!=source['mcap_sha256'] or digest(vision/'aligned_50hz.jsonl')!=source['body_jsonl_sha256'] or digest(vision/'alignment.csv')!=source['body_alignment_sha256']:
            raise ValueError('Source files changed while hand alignment was built')
        write_json(vision/'hand_alignment.json',index)
        return index
    except Exception:
        if stage.exists():shutil.rmtree(stage)
        raise


def load_hand_index(vision):
    vision=Path(vision);path=vision/'hand_alignment.json'
    if not path.exists():return None
    if path.is_symlink():raise ValueError('Unsafe hand alignment index')
    index=json.loads(path.read_text());relative=index.get('version_directory','')
    if index.get('version')!=1 or not VERSION.fullmatch(relative):raise ValueError('Invalid hand alignment version path')
    for p in [vision/relative,vision/'hand_alignment']:
        if p.is_symlink():raise ValueError('Unsafe hand alignment version directory')
    return index


def verify_hand_manifest(vision,index):
    relative=index.get('version_directory','')
    if not VERSION.fullmatch(relative):raise ValueError('Invalid hand alignment path')
    directory=Path(vision)/relative;manifest=directory/'manifest.json'
    if manifest.is_symlink() or directory.is_symlink() or (Path(vision)/'hand_alignment').is_symlink():raise ValueError('Unsafe hand alignment manifest')
    if digest(manifest)!=index['manifest_sha256']:raise ValueError('Hand alignment manifest SHA256 differs')
    data=json.loads(manifest.read_text())
    expected={'aligned_50hz_with_hands.jsonl','alignment.csv','schema.json','summary.json','audit_report.json'}
    if set(data['files'])!=expected:raise ValueError('Hand alignment artifact set differs')
    for name,entry in data['files'].items():
        path=directory/name
        if path.is_symlink() or not path.is_file() or path.stat().st_size!=entry['size'] or digest(path)!=entry['sha256']:raise ValueError('Hand alignment file failed SHA256: '+name)
    summary=json.loads((directory/'summary.json').read_text())
    if 'summary' in index and index['summary']!=summary:raise ValueError('Hand index summary differs from its immutable version')
    if 'training_complete_and_aligned' in index and index['training_complete_and_aligned']!=summary['training_complete_and_aligned']:raise ValueError('Hand index validity differs')
    if 'files' in index and index['files']!={name:relative+'/'+name for name in data['files']}:raise ValueError('Hand index file paths differ')
    return True


def audit_hand_alignment(mcap,vision,directory,policy=None):
    """Re-read MCAP and independently recompute source choices and freshness."""
    mcap,vision,directory=Path(mcap),Path(vision),Path(directory)
    saved=json.loads((directory/'summary.json').read_text());policy=policy or saved['policy'];errors=[]
    commands,states,health=read_hand_streams(mcap);ct=[r['t'] for r in commands];st=[r['t'] for r in states]
    with (vision/'aligned_50hz.jsonl').open() as f:body=[json.loads(l) for l in f]
    with (directory/'aligned_50hz_with_hands.jsonl').open() as f:numeric=[json.loads(l) for l in f]
    with (vision/'alignment.csv').open(newline='') as f:base=list(csv.DictReader(f))
    with (directory/'alignment.csv').open(newline='') as f:timing=list(csv.DictReader(f))
    if not body or len(body)!=len(numeric) or len(body)!=len(timing) or len(body)!=len(base):errors.append('Row count differs from the existing grid')
    bad=0;matched=Counter();required=0;training=0
    for row,value,line,base_line in zip(body,numeric,timing,base):
        t=row['timestamp_ns'];ci=bisect.bisect_right(ct,t)-1;si=bisect.bisect_right(st,t)-1
        c=commands[ci] if ci>=0 else None;s=states[si] if si>=0 else None;h=value.get('hands',{});issue=False
        issue|={k:v for k,v in value.items() if k!='hands'}!=row
        issue|=any(line.get(k)!=v for k,v in base_line.items())
        ca=t-c['t'] if c else None;sa=t-s['t'] if s else None
        choices=dict(command_index=c['i'] if c else None,command_timestamp_ns=c['t'] if c else None,command_age_ns=ca,
            state_index=s['i'] if s else None,state_timestamp_ns=s['t'] if s else None,state_message_age_ns=sa)
        issue|=any(h.get(k)!=v for k,v in choices.items())
        expected_sides={}
        for side,name in enumerate(SIDES):
            v=h.get(name,{});cq=side_values(c,'q',side);sq=side_values(s,'q',side);vel=side_values(s,'velocity',side);cur=side_values(s,'current',side);codes=side_values(s,'state_code',side)
            flag=s.get('flags',[None,None])[side] if s else None;age=s.get('sample_age_ms',[None,None])[side] if s else None
            effective=sa+age*1_000_000 if age is not None and sa is not None else None
            upper=effective+policy['sample_age_quantization_bound_ns'] if effective is not None else None
            co=bool(c and c['decode_valid'] and finite(cq,True) and health['command']['source_time_order_ok'] and 0<=ca<=policy['max_command_age_ms']*1e6)
            so=bool(s and s['decode_valid'] and finite(sq,True) and finite(vel) and finite(cur) and codes is not None and flag and health['state']['source_time_order_ok'] and upper is not None and 0<=upper<=policy['max_state_age_ms']*1e6)
            reasons=[]
            if not (c and c['decode_valid'] and finite(cq,True)):reasons.append('missing_or_invalid_hand_command')
            elif not health['command']['source_time_order_ok']:reasons.append('nonmonotonic_hand_command')
            elif not 0<=ca<=policy['max_command_age_ms']*1e6:reasons.append('stale_hand_command')
            if not (s and s['decode_valid'] and finite(sq,True) and finite(vel) and finite(cur) and codes is not None):reasons.append('missing_or_invalid_hand_state')
            elif not flag:reasons.append('hand_driver_invalid')
            elif not health['state']['source_time_order_ok']:reasons.append('nonmonotonic_hand_state')
            elif upper is None or not 0<=upper<=policy['max_state_age_ms']*1e6:reasons.append('stale_hand_feedback')
            expected=dict(action=clean(cq) if co else None,observation_state=clean(sq) if so else None,command_valid=co,state_valid=so,
                driver_valid=flag,sample_age_ms=age,effective_age_ns=effective,age_upper_bound_ns=upper,
                estimated_measurement_stamp_ns=s['t']-age*1_000_000 if s and age is not None else None,
                raw_command=clean(cq),raw_position=clean(sq),velocity_native=clean(vel),current_native=clean(cur),state_code=codes,matched=co and so,reason=';'.join(reasons))
            issue|=any(v.get(k)!=z for k,z in expected.items());expected_sides[name]=co and so;matched[name]+=co and so
        rm=all(expected_sides[name] for name in policy['required_sides']);tm=bool(row['matched'] and rm)
        issue|=h.get('required_matched')!=rm or h.get('both_matched')!=all(expected_sides.values()) or h.get('training_matched')!=tm
        expected_columns=timing_columns(h,row['matched']) if all(name in h for name in SIDES) else {}
        issue|=any(line.get(k)!=(str(v) if v is not None else '') for k,v in expected_columns.items())
        bad+=bool(issue);required+=rm;training+=tm
    if bad:errors.append(str(bad)+' rows disagree with independently selected MCAP source/values/ages')
    if saved['source']!=dict(mcap_sha256=digest(mcap),body_jsonl_sha256=digest(vision/'aligned_50hz.jsonl'),body_alignment_sha256=digest(vision/'alignment.csv')):errors.append('Source SHA256 differs')
    if saved['policy']!=policy or saved['alignment_samples']!=len(body) or saved['required_matched_samples']!=required or saved['unmatched_required_samples']!=len(body)-required or saved['training_matched_samples']!=training or any(saved['sides'][name]['matched_samples']!=matched[name] or saved['sides'][name]['unmatched_samples']!=len(body)-matched[name] for name in SIDES):errors.append('Summary counts differ')
    if saved['training_complete_and_aligned']!=(training==len(body)):errors.append('Training completeness differs')
    return dict(integrity_passed=not errors,errors=errors,alignment_samples=len(body),source_values_mismatches=bad,
        matched_by_side=dict(matched),required_matched_samples=required,training_matched_samples=training,
        training_complete_and_aligned=training==len(body),mcap_read_crc=True,source_time_order_ok=all(health[k]['source_time_order_ok'] for k in ('command','state')))


def extend_received_episode(root,identity):
    """Add or refresh hands on a closed local episode, without re-cropping it."""
    from vr_session_storage import episode_lock,read_record,session_directory
    root=Path(root)
    with episode_lock(root,identity,blocking=False):
        record=read_record(root,identity)
        if record.get('deleted') or record.get('purged') or record.get('purge_state'):raise ValueError('Cannot extend a deleted episode')
        if not record.get('received') or not record.get('acknowledged'):raise ValueError('Transfer and cache cleanup must finish first')
        target=session_directory(root,record)
        if json.loads((target/'status.json').read_text()).get('state')!='ready':raise ValueError('Episode has no completed body/RGB alignment')
        processed=target/'processed';config=json.loads((processed/'processing_config.json').read_text())
        hand=record.get('hand')
        return build_hand_alignment(processed/'Action_ObservationState'/(identity+'.mcap'),processed/'vision',config,
            required_sides=[hand] if hand in ('L','R') else ['L','R'])


if __name__=='__main__':
    import argparse,sys
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'deps'))
    parser=argparse.ArgumentParser(description='Extend a ready episode with 6D-per-hand 50Hz data, without re-cropping media')
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--recording-id',action='append',required=True)
    args=parser.parse_args()
    for identity in args.recording_id:
        index=extend_received_episode(args.root,identity);summary=index['summary']
        print(json.dumps(dict(recording_id=identity,version_directory=index['version_directory'],integrity_passed=index['integrity_passed'],
            total=summary['alignment_samples'],training_matched=summary['training_matched_samples'],required_sides=summary['required_sides']),ensure_ascii=False))
