"""Workbench streaming, annotations and source-index review helpers."""
from collections import deque, OrderedDict
import csv
import json
import math
from pathlib import Path
import shlex
import struct
import subprocess
import threading
import time
import uuid
from review_alignment import diagnostic, signature, render_preview
from vr_session_storage import split_session,relative_session
from vr_dashboard_depth import validate_header as validate_depth_header


ACTION_ORDER = ['waist_yaw_joint']+['leg_'+side+str(i)+'_joint' for side in ('l','r') for i in range(1,7)]+['zarm_'+side+str(i)+'_joint' for side in ('l','r') for i in range(1,5)]
STATE_ORDER = ACTION_ORDER[1:13]+ACTION_ORDER[:1]+ACTION_ORDER[13:]+['head_yaw','head_pitch']
STATE_INDICES = [12]+list(range(12))+list(range(13,21))

def display_state(values):
    return [values[j]*(-1 if i==0 else 1) for i,j in enumerate(STATE_INDICES)]

def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False)+'\n')
    temp.replace(path)


def load_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


PREVIEW_MAX_AGE_MS = 700


def preview_age_ms(header, now_ns):
    return header.get('latency_bound_ms', 0) + max(0, now_ns-header.get('received_ns', 0))/1e6


def preview_is_fresh(header, now_ns):
    return bool(header.get('fresh') and preview_age_ms(header, now_ns) < PREVIEW_MAX_AGE_MS)


class Streams:
    def __init__(self, config, stopping):
        self.config, self.stopping = config, stopping
        self.condition = threading.Condition()
        self.packet, self.counter = None, 0
        self.depth_packet, self.depth_counter = None, 0
        self.depth, self.depth_received = {}, deque(maxlen=180)
        self.preview, self.feedback = {}, {}
        self.received = deque(maxlen=180)
        self.processes = []

    def process(self, command, input_pipe=False, host=None, options=None):
        options = self.config['ssh_options'] if options is None else options
        p = subprocess.Popen(['ssh']+options+[host or self.config['ssh_host'], command],
            stdin=subprocess.PIPE if input_pipe else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True)
        self.processes.append(p)
        return p

    def feedback_loop(self):
        domain = int(self.config.get('dashboard_domain_id', 0))
        feedback_script = str(Path(self.config['remote_helper']).with_name('run_vr_joint_feedback_check.sh'))
        command = ('bash '+shlex.quote(feedback_script)+' --watch --domain-id '+str(domain))
        if domain:
            command = 'env CYCLONEDDS_URI='+shlex.quote(self.config['dashboard_dds_uri'])+' '+command
        while not self.stopping.is_set():
            p = self.process(command)
            try:
                for line in p.stdout:
                    data = json.loads(line)
                    data['received_ns'] = time.time_ns()
                    with self.condition:
                        self.feedback = data
                    if self.stopping.is_set():
                        break
            except (OSError, ValueError):
                pass
            finally:
                p.terminate();p.wait(timeout=5);self.processes.remove(p)
            self.stopping.wait(2)

    def preview_loop(self, depth=False):
        script = 'vr_dashboard_depth.py' if depth else 'vr_dashboard_preview.py'
        upper_command = ("bash -c 'source /opt/ros/noetic/setup.bash; source /home/leju_kuavo/ros_ws/devel/setup.bash; "
            "exec python3 /home/leju_kuavo/vr_vision_recorder/"+script+" --pull'")
        command = ("ssh -T -o BatchMode=yes -o ConnectTimeout=4 -o ServerAliveInterval=5 "
            "leju_kuavo@192.168.26.12 " + shlex.quote(upper_command))
        direct_options = ['-T', '-o', 'HostName=roban-upper.local', '-o', 'ProxyJump=none',
            '-o', 'HostKeyAlias=192.168.26.12', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', '-o', 'ServerAliveInterval=5',
            '-o', 'ServerAliveCountMax=3']
        direct = True
        def read_exact(stream, n):
            out = bytearray()
            while len(out)<n:
                part = stream.read(n-len(out))
                if not part:
                    raise EOFError()
                out.extend(part)
            return bytes(out)
        while not self.stopping.is_set():
            p = self.process(upper_command, input_pipe=True, host='roban-upper', options=direct_options) if direct else self.process(command, input_pipe=True)
            try:
                while not self.stopping.is_set():
                    # One request in flight: SSH/TCP cannot accumulate seconds of frames.
                    request_ns = time.monotonic_ns()
                    p.stdin.write((str(request_ns)+'\n').encode())
                    p.stdin.flush()
                    size = struct.unpack('!I', read_exact(p.stdout,4))[0]
                    if not 0<size<16384:
                        raise ValueError('Invalid preview packet')
                    header = json.loads(read_exact(p.stdout,size))
                    if depth:
                        validate_depth_header(header)
                    if not 0<=header['size']<=4*1024*1024:
                        raise ValueError('Oversized preview frame')
                    frame = read_exact(p.stdout,header['size'])
                    now = time.time_ns()
                    if header.get('request_id') != str(request_ns):
                        raise ValueError('Unexpected preview response')
                    elapsed_ms = (time.monotonic_ns()-request_ns)/1e6
                    source_age = header.get('source_age_ms')
                    if frame and (not isinstance(source_age, (int,float)) or not math.isfinite(source_age)):
                        raise ValueError('Invalid preview source age')
                    header['request_elapsed_ms'] = elapsed_ms
                    header['transport'] = 'direct' if direct else 'via_lower'
                    # Upper bound uses a local monotonic RTT plus age on the upper
                    # computer, so it does not assume that the two clocks agree.
                    header['latency_bound_ms'] = max(0, source_age or 0) + elapsed_ms
                    header['fresh'] = bool(header.get('fresh') and
                        (not frame or source_age >= -50) and header['latency_bound_ms'] < PREVIEW_MAX_AGE_MS)
                    with self.condition:
                        header['received_ns'] = now
                        received = self.depth_received if depth else self.received
                        if frame and header['fresh']:
                            received.append(now)
                            if depth:
                                self.depth_counter += 1
                                header['relay_index'] = self.depth_counter
                                self.depth_packet = (header, frame)
                            else:
                                self.counter += 1
                                header['relay_index'] = self.counter
                                self.packet = (header, frame)
                        recent = list(received)
                        header['relay_fps'] = (len(recent)-1)*1e9/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]-recent[0]>=1_500_000_000 else None
                        if depth:
                            self.depth = header
                        else:
                            self.preview = header
                        self.condition.notify_all()
            except (EOFError, OSError, ValueError):
                pass
            finally:
                try:
                    p.stdin.close()
                except OSError:
                    pass
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill();p.wait()
                self.processes.remove(p)
            direct = not direct
            self.stopping.wait(2)

    def snapshot(self):
        with self.condition:
            p, f, d = dict(self.preview), dict(self.feedback), dict(self.depth)
        now = time.time_ns()
        p['frame_age_ms'] = preview_age_ms(p, now)
        d['frame_age_ms'] = preview_age_ms(d, now)
        if not preview_is_fresh(p, now):
            p.update(fresh=False, source_fps=None, relay_fps=None)
        if now-f.get('received_ns',0)>1_000_000_000:
            f.update(ready=False, fresh=False, reason='关节反馈连接中断或状态过期', q=[], v=[], tau=[])
        if not preview_is_fresh(d, now):
            d.update(fresh=False, source_fps=None, relay_fps=None)
        p['depth'] = d
        return p, f

    def start(self):
        for target in [self.preview_loop, self.feedback_loop, lambda:self.preview_loop(depth=True)]:
            threading.Thread(target=target,daemon=True).start()

    def close(self):
        for p in list(self.processes):
            try:
                p.terminate()
            except ProcessLookupError:
                pass
        with self.condition:
            self.condition.notify_all()


class Review:
    def __init__(self, root, safe_file):
        self.root, self.safe_file = root, safe_file
        self.lock = threading.RLock()
        self.cache = OrderedDict()
        self.frames = OrderedDict()
        self.decode_slots = threading.BoundedSemaphore(2)
        self.load_lock = threading.Lock()
        self.preview_lock = threading.Lock()

    def resolve_session(self,session):
        parts=split_session(session)
        if len(parts)==1:
            records=[]
            for path in (self.root/'.pipeline/transfers').glob('*.json'):
                record=load_json(path,{})
                if record.get('local_session')==session:records.append(record)
            if len(records)>1:raise ValueError('会话编号重复，请检查索引')
            if records:return relative_session(records[0])
        return session

    def path(self, session, name):
        return self.safe_file(self.root, self.resolve_session(session)+'/processed/vision/'+name)

    def dataset(self, session):
        session=self.resolve_session(session)
        status_path = self.root/session/'status.json'
        status = load_json(status_path,{})
        mode = 'aligned' if status.get('state')=='ready' else 'diagnostic'
        version = 'processed' if mode=='aligned' else 'raw'
        video = self.safe_file(self.root,session+'/'+version+'/vision/rgb.mp4')
        timestamps = self.safe_file(self.root,session+'/'+version+'/vision/frame_timestamps.csv')
        config_path = self.safe_file(self.root,session+'/raw/processing_config.json')
        config = load_json(config_path,{})
        if mode=='aligned':
            sources = [self.path(session,name) for name in
                ('aligned_50hz.jsonl','alignment.csv','aligned_50hz_schema.json','alignment_summary.json')]
        else:
            if status.get('state')!='processing_failed':
                raise ValueError('片段尚未处理完成，请稍后查看')
            metadata_files = list((self.root/session/'raw/Action_ObservationState').glob('*.json'))
            if len(metadata_files)!=1:
                raise ValueError('录制元数据不唯一')
            metadata_path = self.safe_file(self.root,str(metadata_files[0].relative_to(self.root)))
            mcap = self.safe_file(self.root,str(metadata_path.with_suffix('.mcap').relative_to(self.root)))
            sources = [metadata_path,mcap]
        key = signature(sources+[video,timestamps,config_path,status_path])
        with self.load_lock:
            with self.lock:
                if key in self.cache:
                    self.cache.move_to_end(key)
                    return self.cache[key]
            with timestamps.open(newline='') as f:
                frames = list(csv.DictReader(f))
            if not frames or any(int(r['frame_index'])!=i for i,r in enumerate(frames)):
                raise ValueError('视频帧索引不连续')
            times = [int(r['estimated_lower_stamp_ns']) for r in frames]
            if any(b<=a for a,b in zip(times,times[1:])):
                raise ValueError('RGB源时间戳不严格递增，拒绝显示误导性的审核预览')
            source_health = load_json(self.root/session/'source_health.json',{}).get('action',{})
            if mode=='aligned':
                with sources[0].open() as f:
                    numeric = [json.loads(line) for line in f]
                with sources[1].open(newline='') as f:
                    timing = list(csv.DictReader(f))
                schema,summary = load_json(sources[2],{}),load_json(sources[3],{})
            else:
                numeric,timing,source_health = diagnostic(mcap,load_json(metadata_path,{}),config,frames)
                schema = {'action_order':ACTION_ORDER,'alignment_hz':50,'units':'radians',
                    'source_index_rule':'Diagnostic indices refer to original raw MCAP and raw video'}
                clock = load_json(video.parent/'clock_sync.json',{})
                bounds = [clock.get(k,{}).get('uncertainty_ns') for k in ('start','end')]
                offsets = [clock.get(k,{}).get('offset_ns') for k in ('start','end')]
                uncertainty = max(bounds)/1e6 if all(v is not None for v in bounds) else None
                change = abs(offsets[0]-offsets[1])/1e6 if all(v is not None for v in offsets) else None
                summary = {'clock_uncertainty_bound_ms':uncertainty,'clock_offset_change_ms':change,
                    'clock_quality_accepted':uncertainty is not None and change is not None and
                    uncertainty<=config.get('max_clock_uncertainty_ms',5) and change<=50}
            if len(numeric)!=len(timing) or not numeric:
                raise ValueError('对齐表与数值样本不一致')
            for i,(row,t) in enumerate(zip(numeric,timing)):
                if row['sample_index']!=i or int(t['sample_index'])!=i or row['timestamp_ns']!=int(t['sample_timestamp_ns']):
                    raise ValueError('样本与对齐表索引/时间不一致')
                if i and row['timestamp_ns']-numeric[i-1]['timestamp_ns']!=20_000_000:
                    raise ValueError('审核网格不是连续50Hz')
                fi = row['rgb_frame_index']
                if not 0<=fi<len(frames) or fi!=int(t['rgb_frame_index']) or int(t['rgb_timestamp_lower_ns'])!=int(frames[fi]['estimated_lower_stamp_ns']):
                    raise ValueError('样本RGB源帧与时间戳不一致')
                if bool(row['matched'])!=bool(int(t['matched'])):
                    raise ValueError('样本质量标记不一致')
            value = dict(numeric=numeric,timing=timing,schema=schema,summary=summary,frames=frames,
                mode=mode,version=version,video=video,key=key,config=config,source_health=source_health)
            with self.lock:
                self.cache[key] = value
                while len(self.cache)>3:
                    self.cache.popitem(last=False)
            return value

    def data(self, session):
        data = self.dataset(session)
        return data['numeric'],data['timing']

    def sample(self, session, index):
        numeric,timing = self.data(session)
        if not 0<=index<len(numeric):
            raise ValueError('样本索引越界')
        row = dict(numeric[index], timestamp_ns=str(numeric[index]['timestamp_ns']), timing=timing[index])
        row['total_samples'] = len(numeric)
        row['display_state'] = display_state(row['observation_state']) if row['observation_state'] else [None]*21
        row['display_joint_names'] = ACTION_ORDER
        row['display_state_source_indices'] = STATE_INDICES
        return row

    def curve(self, session):
        data = self.dataset(session)
        numeric,timing = data['numeric'],data['timing']
        step = max(1,(len(numeric)+999)//1000)
        indices = sorted(set(range(0,len(numeric),step))|{len(numeric)-1})
        schema = dict(data['schema'])
        schema.update(state_joint_order=STATE_ORDER, display_state_indices=STATE_INDICES,
            state_display_sign=[-1 if i==12 else 1 for i in range(23)],
            display_mapping_note='Roban S17 hardware order; waist direction -1; display only')
        samples = []
        for row,t in zip(numeric,timing):
            fi = row['rgb_frame_index']
            samples.append(dict(row,timestamp_ns=str(row['timestamp_ns']),
                timing={k:str(v) if v is not None else '' for k,v in t.items()},
                original_rgb_frame_index=int(data['frames'][fi].get('original_frame_index',fi))))
        bad = [r['sample_index'] for r in numeric if not r['matched']]
        return {'total_samples':len(numeric),'samples':samples,'points':[samples[i] for i in indices],
            'schema':schema,'mode':data['mode'],'bad_indices':bad,'summary':data['summary'],
            'source_action_health':data['source_health'],
            'trim_start_seconds':data['config'].get('trim_start_seconds',5),
            'trim_end_seconds':data['config'].get('trim_end_seconds',5),
            'thresholds_ms':{k:data['config'].get('max_'+k+'_age_ms',v) for k,v in [('action',50),('state',50),('rgb',40)]}}

    def preview_video(self, session):
        session=self.resolve_session(session)
        data = self.dataset(session)
        # Only this server-derived cache path can be served, never client paths.
        metadata=list((self.root/session/'raw/Action_ObservationState').glob('vr_*.json'))
        from vr_session_storage import IDENTITY
        identity=metadata[0].stem if len(metadata)==1 and IDENTITY.fullmatch(metadata[0].stem) else 'session_'+data['key']
        directory = self.root/'.pipeline/review_cache'/identity/('v2_'+data['key'])
        if any(p.is_symlink() for p in (directory,directory.parent,directory.parent.parent)):
            raise ValueError('Invalid review cache path')
        output = directory/'aligned_preview.mp4'
        with self.preview_lock:
            if not output.is_file():
                render_preview(data['video'],[r['rgb_frame_index'] for r in data['numeric']],output)
        return output

    def frame(self, session, sample):
        row = self.sample(session,sample)
        index = row['rgb_frame_index']
        path = self.dataset(session)['video']
        key = (str(path),path.stat().st_mtime_ns,index)
        with self.lock:
            if key in self.frames:
                self.frames.move_to_end(key)
                return self.frames[key]
        if not self.decode_slots.acquire(timeout=.2):
            raise ValueError('图像解码忙，请稍后重试')
        try:
            result = subprocess.run(['ffmpeg','-v','error','-i',str(path),'-vf','select=eq(n\\,%d)'%index,
                '-frames:v','1','-fps_mode','passthrough','-f','image2pipe','-c:v','mjpeg','pipe:1'],
                capture_output=True,timeout=30)
            if result.returncode or not result.stdout:
                raise ValueError('对应视频帧解码失败')
            with self.lock:
                self.frames[key] = result.stdout
                while len(self.frames)>32:
                    self.frames.popitem(last=False)
            return result.stdout
        finally:
            self.decode_slots.release()

    def annotate(self, session, data):
        session=self.resolve_session(session)
        # Validate the session using its immutable raw video; no arbitrary path writes.
        self.safe_file(self.root, session+'/raw/vision/rgb.mp4')
        path = self.root/session/'episode.json'
        current = load_json(path,{})
        if data.get('result') not in ('pending','success','failure') or data.get('use_for_training') not in (True,False):
            raise ValueError('标注格式错误')
        note = data.get('note','')
        if not isinstance(note,str) or len(note)>4000:
            raise ValueError('备注最多4000字')
        with self.lock:
            current = load_json(path,{})
            current.update(result=data['result'],use_for_training=data['use_for_training'],note=note,updated_ns=time.time_ns())
            atomic_json(path,current)
        return current
