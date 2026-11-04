#!/usr/bin/env python3
"""SSH stdio service: buffered ROS RGB -> MP4 + original frame timestamps."""
import argparse
from collections import deque
import csv
import hashlib
import json
import logging
from pathlib import Path
import queue
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time

from vr_time_alignment import map_upper_to_lower
from vr_cache_cleanup import recording_lock

LOG = logging.getLogger('vr_rgb')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class Capture:
    def __init__(self, directory, request, info, fps):
        self.directory, self.request, self.info, self.fps = directory, request, info, fps
        self.cache_lease = recording_lock(directory.parent, request['recording_id'], capture=True)
        self.cache_lease.__enter__()
        try:
            (directory / 'frames').mkdir(parents=True)
            self.queue = queue.Queue(maxsize=max(600, int(fps * 10)))
            self.records = []
            self.dropped = 0
            self.error = None
            self.thread = threading.Thread(target=self.write_frames, daemon=True)
            self.thread.start()
        except Exception:
            self.release_cache()
            raise

    def enqueue(self, frame):
        try:
            self.queue.put_nowait(frame)
        except queue.Full:
            self.dropped += 1

    def write_frames(self):
        try:
            with (self.directory / 'raw_timestamps.jsonl').open('w') as catalog:
                while True:
                    frame = self.queue.get()
                    if frame is None:
                        break
                    index = len(self.records)
                    path = self.directory / 'frames' / ('%08d.jpg' % index)
                    path.write_bytes(frame['jpeg'])
                    row = {key: value for key, value in frame.items() if key != 'jpeg'}
                    row['source_index'] = index
                    self.records.append(row)
                    catalog.write(json.dumps(row) + '\n')
        except Exception as exc:
            self.error = str(exc)
            LOG.exception('RGB frame spool failed')

    def release_cache(self):
        if self.cache_lease is not None:
            self.cache_lease.__exit__(None, None, None)
            self.cache_lease = None

    def finish(self, end_clock, stop_ns, start_ns):
        try:
            return self._finish(end_clock, stop_ns, start_ns)
        finally:
            if not self.thread.is_alive():
                self.release_cache()

    def _finish(self, end_clock, stop_ns, start_ns):
        # Do not block forever if the writer failed with a full queue.
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.2)
                break
            except queue.Full:
                pass
        self.thread.join(timeout=10)
        if self.thread.is_alive() or self.error or self.dropped:
            raise RuntimeError('Capture incomplete: error=%r dropped=%d' % (self.error, self.dropped))
        start_clock = self.request['clock_start']
        selected = []
        for source in self.records:
            corrected = map_upper_to_lower(source['header_stamp_ns'], start_clock, end_clock)
            if start_ns <= corrected <= stop_ns:
                row = dict(source)
                row['estimated_lower_stamp_ns'] = corrected
                row['relative_lower_ns'] = corrected - start_ns
                selected.append(row)
        selected.sort(key=lambda row: row['header_stamp_ns'])
        if not selected:
            raise RuntimeError('No RGB frames inside requested timestamp window')
        if any(b['header_stamp_ns'] <= a['header_stamp_ns'] for a, b in zip(selected, selected[1:])):
            raise RuntimeError('Camera timestamps repeated or went backwards')
        output = self.directory / 'export'
        output.mkdir()
        fields = ['frame_index', 'header_stamp_ns', 'estimated_lower_stamp_ns',
                  'arrival_upper_ns', 'relative_lower_ns', 'ros_seq', 'source_index']
        with (output / 'frame_timestamps.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index, row in enumerate(selected):
                row['frame_index'] = index
                writer.writerow({key: row[key] for key in fields})
        (output / 'camera_info.json').write_text(json.dumps(self.info, indent=2) + '\n')
        clock = {'start': start_clock, 'end': end_clock,
                 'mapping': 'linear interpolation between upper/lower clock anchors',
                 'start_button_lower_ns': start_ns, 'stop_button_lower_ns': stop_ns}
        (output / 'clock_sync.json').write_text(json.dumps(clock, indent=2) + '\n')
        with (self.directory / 'ffmpeg.log').open('w') as log:
            encoder = subprocess.Popen([
                'ffmpeg', '-hide_banner', '-loglevel', 'warning', '-y',
                '-f', 'image2pipe', '-framerate', str(self.fps), '-c:v', 'mjpeg', '-i', 'pipe:0',
                '-an', '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '18',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output / 'rgb.mp4')
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
            try:
                for row in selected:
                    encoder.stdin.write((self.directory / 'frames' / ('%08d.jpg' % row['source_index'])).read_bytes())
                encoder.stdin.close()
                if encoder.wait(timeout=300) != 0:
                    raise RuntimeError('RGB encoding failed; see ' + str(self.directory / 'ffmpeg.log'))
            finally:
                if encoder.poll() is None:
                    encoder.terminate()
                    encoder.wait(timeout=5)
        probe = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
            '-show_entries', 'stream=nb_read_frames,width,height,codec_name', '-of', 'json',
            str(output / 'rgb.mp4')], timeout=120))['streams'][0]
        if int(probe['nb_read_frames']) != len(selected):
            raise RuntimeError('Video frame count differs from timestamp row count')
        result = {
            'recording_id': self.request['recording_id'], 'session_id': self.request['session_id'],
            'status': 'complete', 'frame_count': len(selected), 'video': 'rgb.mp4',
            'codec': probe['codec_name'], 'width': probe['width'], 'height': probe['height'],
            'container_fps': self.fps, 'source_topic': self.request['rgb_topic'],
            'first_upper_stamp_ns': selected[0]['header_stamp_ns'],
            'last_upper_stamp_ns': selected[-1]['header_stamp_ns'],
            'first_lower_stamp_ns': selected[0]['estimated_lower_stamp_ns'],
            'last_lower_stamp_ns': selected[-1]['estimated_lower_stamp_ns'],
            'queue_dropped_frames': self.dropped,
            'camera_seq_gaps': sum(max(0, b['ros_seq'] - a['ros_seq'] - 1) for a, b in zip(selected, selected[1:])),
            'remote_cache': str(self.directory),
            'timestamp_note': 'MP4 plays at container_fps; original capture times are in frame_timestamps.csv.',
        }
        intervals = [b['header_stamp_ns'] - a['header_stamp_ns'] for a, b in zip(selected, selected[1:])]
        result['observed_fps'] = ((len(selected)-1)*1e9 /
            (selected[-1]['header_stamp_ns']-selected[0]['header_stamp_ns'])) if len(selected)>1 else None
        result['camera_timestamp_gap_events'] = sum(value > 1.5e9/self.fps for value in intervals)
        result['max_camera_interval_ms'] = max(intervals)/1e6 if intervals else None
        (output / 'vision_metadata.json').write_text(json.dumps(result, indent=2) + '\n')
        manifest = {'files': {p.name: {'sha256': sha256(p), 'size': p.stat().st_size}
                              for p in output.iterdir() if p.is_file()}}
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        return {'remote_export': str(output), 'metadata': result,
                'manifest_sha256': sha256(output / 'manifest.json')}


class CameraService:
    def __init__(self, args):
        import rospy
        from sensor_msgs.msg import CompressedImage, CameraInfo
        self.rospy, self.args = rospy, args
        self.lock = threading.Lock()
        self.buffer = deque(maxlen=int(args.fps * args.buffer_seconds) + 30)
        self.info, self.active = None, None
        self.last_stamp = None
        args.cache_root.mkdir(parents=True, exist_ok=True)
        rospy.init_node('vr_rgb_recorder', anonymous=True, disable_signals=True)
        self.sub = rospy.Subscriber(args.rgb_topic, CompressedImage, self.image, queue_size=120,
                                    buff_size=16 * 1024 * 1024)
        self.info_sub = rospy.Subscriber(args.info_topic, CameraInfo, self.camera_info, queue_size=1)

    def image(self, msg):
        if 'jpeg' not in msg.format.lower():
            return
        frame = {'header_stamp_ns': msg.header.stamp.to_nsec(), 'ros_seq': msg.header.seq,
                 'arrival_upper_ns': time.time_ns(), 'jpeg': bytes(msg.data)}
        with self.lock:
            self.buffer.append(frame)
            if self.active:
                self.active.enqueue(frame)

    def camera_info(self, msg):
        self.info = {'width': msg.width, 'height': msg.height, 'frame_id': msg.header.frame_id,
                     'K': list(msg.K), 'D': list(msg.D), 'R': list(msg.R), 'P': list(msg.P),
                     'distortion_model': msg.distortion_model}

    def check_camera(self):
        with self.lock:
            if not self.buffer or time.time_ns() - self.buffer[-1]['arrival_upper_ns'] > 1_500_000_000:
                raise RuntimeError('No fresh camera RGB data on ' + self.args.rgb_topic)
            if not self.info:
                raise RuntimeError('No camera calibration on ' + self.args.info_topic)
            stamps = [frame['header_stamp_ns'] for frame in self.buffer]
            if len(stamps) < max(5, int(self.args.fps * .25)):
                raise RuntimeError('Waiting for enough RGB frames to verify native FPS')
        gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
        fps = 1e9 / statistics.median(gaps) if gaps else self.args.fps
        if abs(fps / self.args.fps - 1) > .05:
            raise RuntimeError('Actual RGB rate %.3f FPS differs from requested %.3f FPS' %
                               (fps, self.args.fps))
        return {'camera_ready': True, 'width': self.info['width'], 'height': self.info['height'],
                'measured_fps': fps}

    def start(self, request):
        camera = self.check_camera()
        identity = request['recording_id']
        if not re.fullmatch(r'[A-Za-z0-9_\-]+', identity):
            raise ValueError('Invalid recording id')
        if shutil.disk_usage(self.args.cache_root).free < 1024 * 1024 * 1024:
            raise OSError('Upper computer has less than 1 GiB free')
        directory = self.args.cache_root / identity
        directory.mkdir()
        request = dict(request, rgb_topic=self.args.rgb_topic)
        with self.lock:
            if self.active:
                raise RuntimeError('Another RGB recording is active')
            cutoff = request['start_button_ns'] + request['clock_start']['offset_ns'] - 250_000_000
            if not self.buffer or self.buffer[0]['header_stamp_ns'] > cutoff + 250_000_000:
                raise RuntimeError('Start request arrived outside buffered camera history')
            capture = Capture(directory, request, dict(self.info), self.args.fps)
            for frame in self.buffer:
                if frame['header_stamp_ns'] >= cutoff:
                    capture.enqueue(frame)
            self.active = capture
        return {'recording_id': identity, 'remote_cache': str(directory)}

    def stop(self, request):
        with self.lock:
            capture = self.active
        if not capture or capture.request['recording_id'] != request['recording_id']:
            raise ValueError('No matching RGB recording')
        # Wait for camera transport latency, then trim by original timestamps.
        time.sleep(.25)
        with self.lock:
            self.active = None
        return capture.finish(request['clock_end'], request['stop_button_ns'],
                              capture.request['start_button_ns'])

    def acknowledge_transfer(self, request):
        identity = request['recording_id']
        if not re.fullmatch(r'[A-Za-z0-9_\-]+', identity):
            raise ValueError('Invalid recording id')
        directory = self.args.cache_root / identity
        manifest = directory / 'export' / 'manifest.json'
        if sha256(manifest) != request['manifest_sha256']:
            raise ValueError('Transfer acknowledgement has wrong manifest hash')
        # Only generated JPEG intermediates are removed, after verified transfer
        # and accepted alignment. Keep the upper MP4/CSV export as a recovery copy.
        frames = directory / 'frames'
        if frames.exists():
            shutil.rmtree(str(frames))
        return {'temporary_jpegs_removed': True, 'upper_export_retained': str(directory / 'export')}

    def close(self):
        with self.lock:
            capture, self.active = self.active, None
        if capture:
            # Preserve JPEG spool on connection loss; never call it a complete video.
            while capture.thread.is_alive():
                try:
                    capture.queue.put(None, timeout=.2)
                    break
                except queue.Full:
                    pass
            capture.thread.join(timeout=5)
            (capture.directory / 'interrupted.json').write_text(json.dumps({
                'status': 'interrupted', 'reason': 'SSH/controller disconnected',
                'frames_spooled': len(capture.records)}))
            if not capture.thread.is_alive():
                capture.release_cache()
        self.sub.unregister()
        self.info_sub.unregister()
        self.rospy.signal_shutdown('VR vision connection closed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cache-root', type=Path, default=Path.home() / '.cache/vr_vision_sessions')
    parser.add_argument('--rgb-topic', default='/roban2_stage0_camera/color/image_raw/compressed')
    parser.add_argument('--info-topic', default='/roban2_stage0_camera/color/camera_info')
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--buffer-seconds', type=float, default=5.0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, stream=sys.stderr)
    service = CameraService(args)
    try:
        print(json.dumps({'event': 'ready'}), flush=True)
        for line in sys.stdin:
            received = time.time_ns()
            request = {}
            try:
                request = json.loads(line)
                command = request['command']
                if command == 'clock':
                    result = {'receive_ns': received, 'send_ns': time.time_ns()}
                elif command == 'status':
                    result = service.check_camera()
                elif command == 'start':
                    result = service.start(request)
                elif command == 'stop':
                    result = service.stop(request)
                elif command == 'ack':
                    result = service.acknowledge_transfer(request)
                else:
                    raise ValueError('Unknown command')
                response = {'id': request['id'], 'ok': True, 'result': result}
            except Exception as exc:
                LOG.exception('Vision request failed')
                response = {'id': request.get('id'), 'ok': False, 'error': str(exc)}
            print(json.dumps(response), flush=True)
    finally:
        service.close()


if __name__ == '__main__':
    main()
