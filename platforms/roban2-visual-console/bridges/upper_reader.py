#!/usr/bin/env python3
"""Subscribe to existing ROS camera and diagnostic topics; no ROS publishers."""
import base64
import json
import math
import os
import signal
import threading
import time

import cv2
import numpy as np
import rospy
import roslib.message
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

LOCK = threading.Lock()
LATEST = {}
COUNTS = {}


def emit(value):
    print(json.dumps(value, allow_nan=False, separators=(',', ':')), flush=True)


def primitive(value):
    if isinstance(value, (bool, str, int)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [primitive(x) for x in value[:200]]
    if hasattr(value, '__slots__'):
        return {k: primitive(getattr(value, k)) for k in value.__slots__}
    return str(value)[:300]


def receive(msg, key):
    with LOCK:
        LATEST[key] = (msg, time.time())
        COUNTS[key] = COUNTS.get(key, 0) + 1


def encode(msg, depth=False):
    if isinstance(msg, CompressedImage):
        arr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise ValueError('Compressed image decode failed')
    else:
        enc = msg.encoding.lower()
        if depth:
            dtype = {'16uc1': np.uint16, 'mono16': np.uint16, '32fc1': np.float32}.get(enc)
            if dtype is None:
                raise ValueError('Unsupported depth encoding: ' + msg.encoding)
            dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
            raw = np.ndarray((msg.height, msg.width), dtype=dt, buffer=msg.data, strides=(msg.step, dt.itemsize)).astype(np.float32)
            valid = np.isfinite(raw) & (raw > 0)
            near, far = np.percentile(raw[valid], [3, 97]) if valid.any() else (0., 1.)
            gray = np.zeros(raw.shape, np.uint8)
            if far > near:
                gray[valid] = (255 * (1 - np.clip((raw[valid] - near) / (far - near), 0, 1))).astype(np.uint8)
            arr = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
            arr[~valid] = 0
        else:
            channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc)
            if channels is None:
                raise ValueError('Unsupported color encoding: ' + msg.encoding)
            arr = np.ndarray((msg.height, msg.width, channels), np.uint8, buffer=msg.data, strides=(msg.step, channels, 1))
            if enc == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc == 'rgba8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            elif enc == 'bgra8':
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            elif enc == 'mono8':
                arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    h, w = arr.shape[:2]
    if w > 960:
        arr = cv2.resize(arr, (960, round(h * 960 / w)))
    ok, jpeg = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, int(os.environ.get('ROBAN_JPEG_QUALITY', '78'))])
    if not ok:
        raise ValueError('JPEG encode failed')
    return jpeg.tobytes(), w, h


def main():
    rospy.init_node('roban2_web_observer', anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    subscribers = {}
    chosen = {}
    preferred = {'rgb': os.environ.get('ROBAN_COLOR_TOPIC', ''), 'depth': os.environ.get('ROBAN_DEPTH_TOPIC', ''),
                 'camera_info': os.environ.get('ROBAN_INFO_TOPIC', '')}
    last_sent = {}
    next_discovery = 0
    errors = {}
    period = 1. / float(os.environ.get('ROBAN_CAMERA_FPS', '12'))
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now >= next_discovery:
            try:
                topics = dict(rospy.get_published_topics())
                candidates = {
                    'rgb': sorted(t for t, typ in topics.items() if typ in ('sensor_msgs/Image', 'sensor_msgs/CompressedImage') and not any(x in t.lower() for x in ('depth', 'infra', 'ir_', 'confidence'))),
                    'depth': sorted(t for t, typ in topics.items() if typ == 'sensor_msgs/Image' and 'depth' in t.lower()),
                    'camera_info': sorted(t for t, typ in topics.items() if typ == 'sensor_msgs/CameraInfo' and 'color' in t),
                    'controller': ['/controller_state'] if '/controller_state' in topics else [],
                }
                for key, available in candidates.items():
                    target = preferred.get(key) if preferred.get(key) in topics else (available[0] if available else None)
                    if target == chosen.get(key):
                        continue
                    if key in subscribers:
                        subscribers.pop(key).unregister()
                    chosen[key] = target
                    if target:
                        cls = roslib.message.get_message_class(topics[target])
                        if cls:
                            subscribers[key] = rospy.Subscriber(target, cls, receive, callback_args=key, queue_size=1, buff_size=16 * 1024 * 1024, tcp_nodelay=True)
                emit({'kind': 'ros_status', 'data': {'topics': topics, 'selected': chosen, 'counts': dict(COUNTS),
                      'errors': dict(errors), 'remote_wall': time.time(), 'motion_commands_sent': 0}})
            except Exception as exc:
                emit({'kind': 'bridge_error', 'data': {'message': str(exc)[:300]}})
            next_discovery = now + 3
        with LOCK:
            current = dict(LATEST)
        for key, (msg, received_at) in current.items():
            stamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') else received_at
            # Never re-emit the same capture as if it were a new live frame.
            unique = (stamp, received_at)
            if unique == last_sent.get(key):
                continue
            last_sent[key] = unique
            try:
                packet = {'kind': key, 'topic': chosen.get(key), 'stamp': stamp,
                          'source_age_ms': (time.time() - stamp) * 1000 if stamp > 0 else None}
                if key in ('rgb', 'depth'):
                    jpeg, w, h = encode(msg, depth=key == 'depth')
                    packet['jpeg'] = base64.b64encode(jpeg).decode('ascii')
                    packet['data'] = {'width': w, 'height': h, 'frame_id': msg.header.frame_id,
                                      'encoding': getattr(msg, 'encoding', getattr(msg, 'format', 'jpeg')),
                                      'depth_scale_verified': False}
                else:
                    packet['data'] = primitive(msg)
                emit(packet)
                errors.pop(key, None)
            except Exception as exc:
                errors[key] = str(exc)[:200]
        # Keep the configured cadence instead of adding the encode/SSH work on top of it.
        delay = period - (time.monotonic() - now)
        if delay > 0:
            time.sleep(delay)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: rospy.signal_shutdown('observer stopped'))
    main()
