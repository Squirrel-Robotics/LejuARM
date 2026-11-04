#!/usr/bin/env python3
"""Read-only RGB-D tabletop inclination; independent of head motor zero."""
import argparse
import base64
from collections import deque
from datetime import datetime
import io
import json
import math
from pathlib import Path
import select
import shlex
import subprocess
import sys
import threading
import time

import numpy as np

# Verified clear tabletop patches in the 2026-11-02 640x480 aligned depth view.
# These are measurement regions, not an automatic semantic table detector.
DEFAULT_ROIS = [(115, 210, 215, 320), (225, 210, 335, 320), (140, 335, 330, 445)]
MAX_SOURCE_AGE_MS = 1500.0


def fit_plane(depth, k, roi):
    x0, y0, x1, y1 = roi
    if not (0 <= x0 < x1 <= depth.shape[1] and 0 <= y0 < y1 <= depth.shape[0]):
        raise ValueError('ROI_OUTSIDE_IMAGE')
    v, u = np.mgrid[y0:y1:3, x0:x1:3]
    z = depth[v, u].astype(float)
    good = np.isfinite(z) & (z > 0)
    if good.sum() < 250 or good.mean() < .6:
        raise ValueError('INSUFFICIENT_VALID_TABLE_DEPTH')
    u, v, z = u[good], v[good], z[good]
    p = np.column_stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z))
    scale = float(np.median(z))
    threshold = scale * .006
    best = np.zeros(len(p), dtype=bool)
    rng = np.random.default_rng(314)
    for _ in range(200):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-10:
            continue
        n /= length
        inside = np.abs((p-a) @ n) < threshold
        if inside.sum() > best.sum():
            best = inside
    if best.sum() < 250:
        raise ValueError('TABLE_PLANE_NOT_FOUND')
    for _ in range(3):
        center = p[best].mean(axis=0)
        _, singular, vh = np.linalg.svd(p[best]-center, full_matrices=False)
        n = vh[-1]
        residual = np.abs((p-center) @ n)
        best = residual < threshold
        if best.sum() < 250:
            raise ValueError('TABLE_PLANE_NOT_FOUND')
    fraction = float(best.mean())
    rms = float(np.sqrt(np.mean(residual[best]**2)) / scale)
    if fraction < .85 or rms > .005 or singular[1] / max(singular[0], 1e-12) < .05:
        raise ValueError('TABLE_PLANE_POOR_FIT: roi=%s inliers=%.3f rms=%.5f' % (roi, fraction, rms))
    if n[2] > 0:
        n = -n
    # Camera optical axis is +Z. Inclination to the plane is asin(|normal.z|).
    return dict(angle_deg=math.degrees(math.asin(float(np.clip(abs(n[2]), 0, 1)))),
                normal_camera=n.tolist(), roi=list(roi), points=len(p),
                valid_depth_fraction=float(good.mean()), inlier_fraction=fraction,
                rms_relative_depth=rms)


def estimate(frames, info, samples, rois):
    if len(frames) < 3 or len(frames) != len(samples):
        raise ValueError('NEED_MULTIPLE_DEPTH_FRAMES')
    k = np.asarray(info['K'], dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('INVALID_DEPTH_INTRINSICS')
    if np.any(np.abs(np.asarray(info.get('D', []), dtype=float)) > 1e-10):
        raise ValueError('DEPTH_RAYS_REQUIRE_UNDISTORTION')
    if not str(info.get('frame_id', '')).endswith('color_optical_frame'):
        raise ValueError('DEPTH_NOT_IN_COLOR_OPTICAL_FRAME')
    stamps = [s['stamp'] for s in samples]
    if not all(math.isfinite(s) and s > 0 for s in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('DEPTH_SOURCE_NOT_ADVANCING')
    for frame, sample in zip(frames, samples):
        age = sample['age_ms']
        if not math.isfinite(age) or not 0 <= age <= MAX_SOURCE_AGE_MS:
            raise ValueError('DEPTH_SOURCE_STALE_OR_FUTURE')
        if (sample['frame_id'] != info['frame_id'] or sample['encoding'].upper() != '16UC1'
                or frame.shape != (info['height'], info['width'])):
            raise ValueError('DEPTH_SCHEMA_MISMATCH')
    fits = [dict(fit_plane(frame, k, roi), frame_index=i)
            for i, frame in enumerate(frames) for roi in rois]
    angles = [f['angle_deg'] for f in fits]
    if max(angles)-min(angles) > 1.5:
        raise ValueError('TABLE_PATCHES_OR_CAMERA_MOTION_INCONSISTENT')
    return dict(status='LIVE', angle_deg=float(np.median(angles)),
                angle_min_deg=min(angles), angle_max_deg=max(angles),
                frame_count=len(frames), patch_count=len(rois),
                source_stamp=stamps[-1], source_age_ms=samples[-1]['age_ms'],
                angle_reference='camera_optical_axis_to_selected_tabletop_plane',
                convention='0_parallel_to_table_90_perpendicular_to_table',
                independent_of_motor_zero=True, automatic_table_detection=False,
                depth_info=info, samples=samples, fits=fits, motion_commands_sent=0)


def unavailable(reason):
    return dict(status='UNAVAILABLE', angle_deg=None, reason=reason, motion_commands_sent=0)


def remote_main(config):
    # This process ONLY subscribes to image and CameraInfo messages.
    import rospy
    from sensor_msgs.msg import Image, CameraInfo
    rospy.init_node('table_optical_angle_readonly', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    lock = threading.Lock()
    frames = deque(maxlen=5)
    state = {'info': None, 'last_received': -1.0, 'error': None}

    def on_info(msg):
        with lock:
            state['info'] = dict(K=list(msg.K), D=list(msg.D), width=msg.width,
                                 height=msg.height, frame_id=msg.header.frame_id)

    def on_depth(msg):
        received = time.monotonic()
        with lock:
            if received-state['last_received'] < .08:
                return
            state['last_received'] = received
            if msg.encoding.upper() != '16UC1':
                state['error'] = 'UNSUPPORTED_DEPTH_ENCODING'
                return
            try:
                dtype = np.dtype(np.uint16).newbyteorder('>' if msg.is_bigendian else '<')
                depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                                   buffer=msg.data, strides=(msg.step, 2)).copy()
                sample = dict(stamp=msg.header.stamp.to_sec(), encoding=msg.encoding,
                              frame_id=msg.header.frame_id)
                frames.append((depth, sample))
                state['error'] = None
            except (ValueError, TypeError) as exc:
                state['error'] = 'DEPTH_DECODE_FAILED: '+str(exc)

    prefix = '/roban2_stage0_camera'
    subscribers = [rospy.Subscriber(prefix+'/depth/camera_info', CameraInfo, on_info, queue_size=1),
                   rospy.Subscriber(prefix+'/depth/image_raw', Image, on_depth, queue_size=1,
                                    buff_size=2**22)]
    last_stamp = 0.0
    count = 0
    captured = False
    while not rospy.is_shutdown():
        deadline = time.monotonic()+8
        batch, info = [], None
        while time.monotonic() < deadline:
            with lock:
                info = state['info']
                batch = [(d, dict(s)) for d, s in frames if s['stamp'] > last_stamp]
                error = state['error']
            if info is not None and len(batch) == 5:
                break
            time.sleep(.03)
        try:
            if error:
                raise ValueError(error)
            if info is None or len(batch) < 5:
                raise ValueError('DEPTH_CAMERA_INFO_OR_FRESH_FRAMES_UNAVAILABLE')
            now = time.time()
            for _, sample in batch:
                sample['age_ms'] = (now-sample['stamp'])*1000
            arrays, samples = [d for d, _ in batch], [s for _, s in batch]
            result = estimate(arrays, info, samples, config['rois'])
            result['source_age_ms'] = (time.time()-samples[-1]['stamp'])*1000
            if result['source_age_ms'] > MAX_SOURCE_AGE_MS:
                raise ValueError('DEPTH_STALE_AFTER_FITTING')
            last_stamp = samples[-1]['stamp']
        except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            result = unavailable(str(exc))
        if not captured and info is not None and len(batch) == 5:
            buf = io.BytesIO()
            np.savez_compressed(buf, depth=np.stack([d for d, _ in batch]), metadata=json.dumps(
                dict(depth_info=info, samples=[s for _, s in batch], rois=config['rois'])))
            result['capture_npz_base64'] = base64.b64encode(buf.getvalue()).decode('ascii')
            captured = True
        result['measured_at_epoch'] = time.time()
        if result['status'] == 'LIVE':
            result['source_age_ms'] = (result['measured_at_epoch']-result['source_stamp'])*1000
            if result['source_age_ms'] > MAX_SOURCE_AGE_MS:
                result = dict(unavailable('DEPTH_STALE_AFTER_ENCODING'),
                              measured_at_epoch=result['measured_at_epoch'])
        print(json.dumps(result, allow_nan=False), flush=True)
        count += 1
        if not config['watch'] or (config['count'] and count >= config['count']):
            break
        time.sleep(config['interval'])
    for sub in subscribers:
        sub.unregister()


def ssh_command(config):
    flags = ['-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
             '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
             '-o', 'ServerAliveCountMax=2']
    remote_python = ['env', 'PYTHONDONTWRITEBYTECODE=1', 'OMP_NUM_THREADS=1',
                     'OPENBLAS_NUM_THREADS=1', 'MKL_NUM_THREADS=1',
                     'python3', '-u', '-', '--remote', json.dumps(config)]
    setup = ('source /opt/ros/noetic/setup.bash && '
             'source /home/leju_kuavo/ros_ws/devel/setup.bash && exec '+shlex.join(remote_python))
    upper = ['ssh']+flags+['-o', 'HostKeyAlias=roban-upper.local',
                           'leju_kuavo@192.168.26.12', shlex.join(['bash', '-lc', setup])]
    return ['ssh']+flags+['-o', 'HostKeyAlias=roban-lower.local',
                          'lab@192.168.110.84', shlex.join(upper)]


def show(result):
    print(result['received_at'])
    if result['status'] != 'LIVE':
        print('当前相机相对桌面俯角不可用：'+result.get('reason', 'UNKNOWN'), flush=True)
        return
    print('相机光轴相对桌面俯角：%.2f°' % result['angle_deg'])
    print('多帧/多区域结果范围：%.2f°～%.2f°；源数据年龄：%.0f ms' % (
        result['angle_min_deg'], result['angle_max_deg'], result['source_age_ms']))
    print('0° = 平行桌面；90° = 垂直朝向桌面。该读数不依赖头部电机零位。')
    print('使用预选的裸露桌面区域；物体遮挡区域或相机移动后需重新选择 --roi。', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', action='store_true', help='保持单个只读订阅连接，连续刷新')
    parser.add_argument('--count', type=int, default=0, help='连续模式刷新次数；0 为直到 Ctrl+C')
    parser.add_argument('--interval', type=float, default=2., help='连续模式刷新间隔，秒')
    parser.add_argument('--json', action='store_true', help='输出 JSON')
    parser.add_argument('--roi', action='append', metavar='X0,Y0,X1,Y1',
                        help='裸露桌面区域，可多次提供；坐标基于 640x480 图像')
    parser.add_argument('--remote', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.remote:
        remote_main(json.loads(args.remote))
        return 0
    if args.count < 0 or not math.isfinite(args.interval) or args.interval < .5:
        parser.error('count 必须非负，interval 必须至少 0.5 秒')
    try:
        rois = [tuple(map(int, s.split(','))) for s in args.roi] if args.roi else DEFAULT_ROIS
        if len(rois) < 2 or any(len(r) != 4 for r in rois):
            raise ValueError()
    except ValueError:
        parser.error('至少提供两个裸露桌面区域，每个格式为 X0,Y0,X1,Y1')
    config = dict(watch=args.watch, count=args.count, interval=args.interval, rois=rois)
    directory = Path(__file__).resolve().parents[1]/'state'/'table_angle'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    last_status = 'UNAVAILABLE'
    process = None
    try:
        with (directory/'ssh.stderr').open('w') as stderr, (directory/'readings.jsonl').open('w') as log:
            process = subprocess.Popen(ssh_command(config), stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=stderr)
            process.stdin.write(Path(__file__).read_bytes())
            process.stdin.close()
            buffer = b''
            deadline = time.monotonic()+30
            while True:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('READONLY_CAMERA_CONNECTION_TIMEOUT')
                ready, _, _ = select.select([process.stdout], [], [], min(remaining, 1.))
                if not ready:
                    continue
                import os
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    if not buffer and process.wait(timeout=3) == 0:
                        break
                    raise OSError('SSH_CAMERA_READER_EXITED; details: '+str(directory/'ssh.stderr'))
                buffer += chunk
                if len(buffer) > 8*1024*1024:
                    raise ValueError('CAMERA_REPLY_TOO_LARGE')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    result = json.loads(line)
                    # Clock agreement is independently required before labeling a network result LIVE.
                    transport_age_ms = (time.time()-result['measured_at_epoch'])*1000
                    if not -100 <= transport_age_ms <= 1500:
                        result = unavailable('RESULT_DELAY_OR_HOST_CLOCK_DIFFERENCE')
                    elif result['status'] == 'LIVE':
                        result['source_age_ms'] += max(0., transport_age_ms)
                        if result['source_age_ms'] > MAX_SOURCE_AGE_MS:
                            result = unavailable('DEPTH_STALE_ON_RECEIPT')
                    capture = result.pop('capture_npz_base64', None)
                    if capture:
                        (directory/'depth_frames.npz').write_bytes(base64.b64decode(capture, validate=True))
                    result.update(received_at=datetime.now().astimezone().isoformat(),
                                  evidence_directory=str(directory))
                    text = json.dumps(result, ensure_ascii=False, allow_nan=False)
                    log.write(text+'\n')
                    log.flush()
                    (directory/'latest.json').write_text(text+'\n', encoding='utf-8')
                    if args.json:
                        print(text, flush=True)
                    else:
                        show(result)
                    last_status = result['status']
                    deadline = time.monotonic()+max(15, args.interval+12)
        if not args.json:
            print('只读测量记录：'+str(directory))
        return 0 if last_status == 'LIVE' else 2
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        result = unavailable(str(exc))
        result.update(received_at=datetime.now().astimezone().isoformat(), evidence_directory=str(directory))
        (directory/'latest.json').write_text(json.dumps(result, ensure_ascii=False)+'\n', encoding='utf-8')
        if args.json:
            print(json.dumps(result, ensure_ascii=False), flush=True)
        else:
            show(result)
        return 2
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


if __name__ == '__main__':
    raise SystemExit(main())
