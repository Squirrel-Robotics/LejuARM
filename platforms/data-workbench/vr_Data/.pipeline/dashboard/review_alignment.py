"""Read-only review diagnostics and a video whose frame N is 50 Hz sample N."""
import bisect
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
DEPS = Path(__file__).resolve().parents[1]/'deps'
for directory in (SCRIPTS, DEPS):
    if str(directory) not in sys.path:
        sys.path.append(str(directory))


def read_json(path):
    return json.loads(path.read_text())


def diagnostic(mcap, metadata, config, frames):
    from vr_fixed_rate_alignment import read_streams
    begin = metadata['start_button_ns']+round(config.get('trim_start_seconds',5)*1e9)
    end = metadata['stop_boundary_ns']-round(config.get('trim_end_seconds',5)*1e9)
    if end <= begin:
        raise ValueError('片段不足首尾裁剪时长，无法建立审核网格')
    actions, states = read_streams(mcap, config)
    intervals = [b['t']-a['t'] for a,b in zip(actions,actions[1:])]
    source_health = dict(messages=len(actions),scope='raw',
        observed_hz=(len(actions)-1)*1e9/(actions[-1]['t']-actions[0]['t']) if len(actions)>1 and actions[-1]['t']>actions[0]['t'] else None,
        max_interval_ms=max(intervals)/1e6 if intervals else None)
    actions = [r for r in actions if begin <= r['t'] <= end]
    states = [r for r in states if begin <= r['t'] <= end]
    selected = [(i,r) for i,r in enumerate(frames) if begin <= int(r['estimated_lower_stamp_ns']) <= end]
    if not actions or not states or not selected:
        raise ValueError('裁剪区间内缺少动作、反馈或图像')
    hz = int(config.get('alignment_hz',50))
    if hz != 50:
        raise ValueError('审核要求50Hz网格')
    period = 1_000_000_000//hz
    at, st = [r['t'] for r in actions], [r['t'] for r in states]
    ft = [int(r['estimated_lower_stamp_ns']) for _,r in selected]
    first = max(begin,at[0],st[0],ft[0])
    origin = begin+((first-begin+period-1)//period)*period
    numeric, timing = [], []
    limits = {key:round(config.get('max_'+key+'_age_ms',default)*1e6)
              for key,default in [('action',50),('state',50),('rgb',40)]}
    for index,t in enumerate(range(origin,end+1,period)):
        ai,si,fi = [bisect.bisect_right(times,t)-1 for times in (at,st,ft)]
        a,s = actions[ai],states[si]
        ages = {'action':t-a['t'],'state':t-s['t'],'rgb':t-ft[fi]}
        reasons = []
        for key,valid in [('action',a['valid']),('state',s['valid']),('rgb',True)]:
            if not valid:
                reasons.append('missing_or_invalid_'+key)
            elif ages[key]>limits[key]:
                reasons.append('stale_'+key)
        # Invalid/stale samples remain visibly failed. Valid source values are shown
        # for diagnosis only; they are never written into a training output.
        numeric.append(dict(sample_index=index,timestamp_ns=t,timestamp_s=(t-origin)/1e9,
            matched=not reasons,rgb_frame_index=selected[fi][0],action_index=a['i'],state_index=s['i'],
            action=a['q'] if a['valid'] else None,observation_state=s['q'] if s['valid'] else None))
        timing.append(dict(sample_index=index,sample_timestamp_ns=t,sample_relative_ns=t-origin,
            action_index=a['i'],action_timestamp_ns=a['t'],action_age_ns=ages['action'],
            action_mcap_log_time_ns=a['log'],state_index=s['i'],state_timestamp_ns=s['t'],
            state_age_ns=ages['state'],rgb_frame_index=selected[fi][0],rgb_timestamp_lower_ns=ft[fi],
            rgb_age_ns=ages['rgb'],matched=int(not reasons),reason=';'.join(reasons)))
    if not numeric:
        raise ValueError('裁剪区间内没有可查看的50Hz样本')
    return numeric,timing,source_health


def signature(paths):
    return hashlib.sha256(repr([(str(p),p.stat().st_size,p.stat().st_mtime_ns) for p in paths]).encode()).hexdigest()


def render_preview(video, indices, output):
    """Decode real source frames sequentially, emit exactly one mapped frame/sample.

    This is a disposable review artifact, not a replacement of native 60 FPS data.
    No interpolation or interpolation of joint angles is performed.
    """
    if not indices or min(indices)<0 or indices != sorted(indices):
        raise ValueError('审核图像索引缺失或倒退')
    probe = subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries',
        'stream=width,height','-of','json',str(video)],capture_output=True,text=True,timeout=15,check=True)
    stream = json.loads(probe.stdout)['streams'][0]
    w,h = int(stream['width']),int(stream['height'])
    width = min(640,w)//2*2
    height = max(2,round(h*width/w/2)*2)
    frame_size = width*height*3
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary = output.with_name('.'+uuid.uuid4().hex+'.mp4')
    decoder = encoder = None
    started = time.monotonic()
    try:
        with tempfile.TemporaryFile() as errors:
            decoder = subprocess.Popen(['ffmpeg','-v','error','-threads','1','-i',str(video),
                '-an','-vf',f'scale={width}:{height}','-pix_fmt','rgb24','-fps_mode','passthrough',
                '-f','rawvideo','pipe:1'],stdout=subprocess.PIPE,stderr=errors)
            encoder = subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24',
                '-s',f'{width}x{height}','-r','50','-i','pipe:0','-an','-c:v','libx264','-threads','2',
                '-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)],
                stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=errors)
            current, frame = -1,None
            for target in indices:
                if time.monotonic()-started>300:
                    raise ValueError('审核预览生成超时，请重试')
                while current<target:
                    frame = decoder.stdout.read(frame_size)
                    if len(frame)!=frame_size:
                        raise ValueError('视频源帧不足，无法生成完整审核预览')
                    current+=1
                encoder.stdin.write(frame)
            encoder.stdin.close()
            if encoder.wait(timeout=30):
                raise ValueError('审核预览编码失败')
            decoder.stdout.close()
            decoder.terminate()
            decoder.wait(timeout=10)
        probe = subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
            '-show_entries','stream=nb_read_frames,r_frame_rate','-of','json',str(temporary)],
            capture_output=True,text=True,timeout=30,check=True)
        info = json.loads(probe.stdout)['streams'][0]
        if int(info['nb_read_frames'])!=len(indices) or info['r_frame_rate']!='50/1':
            raise ValueError('审核预览帧数检查失败')
        temporary.replace(output)
    finally:
        for process in (decoder,encoder):
            if process:
                if process.poll() is None:
                    process.kill();process.wait()
                for stream in (process.stdin,process.stdout):
                    if stream and not stream.closed:
                        try:
                            stream.close()
                        except OSError:
                            pass
        temporary.unlink(missing_ok=True)
