#!/usr/bin/env python3
"""Read-only live 16-bit depth over SSH; no device settings or recording writes."""
from collections import deque
import json
import signal
import struct
import sys
import threading
import time
import zlib

TOPIC = '/roban2_stage0_camera/depth/image_raw'
MAX_BYTES = 4 * 1024 * 1024


def canonical_depth(data, width, height, step, bigendian=False, encoding='16UC1'):
    if encoding != '16UC1':
        raise ValueError('Expected millimetre 16UC1 depth from the configured Orbbec driver')
    if not 0 < width <= 1920 or not 0 < height <= 1080 or width * height * 2 > MAX_BYTES:
        raise ValueError('Invalid depth dimensions')
    if step < width * 2 or len(data) != step * height:
        raise ValueError('Depth buffer or row stride differs from its dimensions')
    raw = bytes(data) if step == width * 2 else b''.join(
        data[row * step:row * step + width * 2] for row in range(height))
    if bigendian:
        swapped = bytearray(len(raw))
        swapped[0::2], swapped[1::2] = raw[1::2], raw[0::2]
        raw = bytes(swapped)
    return raw


def validate_header(header):
    size = header.get('size')
    if not isinstance(size, int) or isinstance(size, bool) or not 0 <= size <= MAX_BYTES:
        raise ValueError('Invalid depth payload length')
    if not size:
        return
    width, height = header.get('width'), header.get('height')
    if (not isinstance(width, int) or not isinstance(height, int) or
            not 0 < width <= 1920 or not 0 < height <= 1080 or width * height * 2 > MAX_BYTES):
        raise ValueError('Invalid depth image dimensions')
    if (header.get('codec') != 'zlib-u16le' or header.get('unit_m') != .001 or
            header.get('raw_size') != width * height * 2):
        raise ValueError('Unsupported depth encoding or unit')


def inflate_depth(frame, expected):
    decoder = zlib.decompressobj()
    try:
        raw = decoder.decompress(frame, expected + 1)
    except zlib.error as exc:
        raise ValueError('Invalid compressed depth frame') from exc
    if len(raw) != expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('Depth decompression length or framing mismatch')
    return raw


def main():
    import rospy
    from sensor_msgs.msg import Image
    pull = '--pull' in sys.argv[1:]
    condition = threading.Condition()
    stamps = deque(maxlen=180)
    state = {'counter': 0, 'message': None}

    def receive(msg):
        with condition:
            stamps.append(msg.header.stamp.to_nsec())
            state.update(counter=state['counter'] + 1, message=msg, arrival_ns=time.time_ns())
            condition.notify_all()

    rospy.init_node('roban_workbench_depth_preview', anonymous=True, disable_signals=True)
    sub = rospy.Subscriber(TOPIC, Image, receive, queue_size=1, buff_size=4 * 1024 * 1024)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    last = -1
    try:
        while True:
            request_id = None
            if pull:
                request = sys.stdin.buffer.readline(64)
                if not request:
                    break
                request_id = str(int(request))
            with condition:
                condition.wait_for(lambda: state['counter'] != last, timeout=1)
                fresh = state.get('arrival_ns') and time.time_ns() - state['arrival_ns'] < 500_000_000
                msg = state['message'] if fresh and state['counter'] != last else None
                previous, last = last, state['counter']
                recent = list(stamps)
            if msg is not None:
                started = time.monotonic_ns()
                raw = canonical_depth(msg.data, msg.width, msg.height, msg.step, msg.is_bigendian, msg.encoding)
                frame = zlib.compress(raw, level=1)
                header = dict(size=len(frame), raw_size=len(raw), width=msg.width, height=msg.height,
                    codec='zlib-u16le', unit_m=.001, invalid_value=0, frame_id=msg.header.frame_id,
                    stamp_ns=str(msg.header.stamp.to_nsec()), ros_seq=msg.header.seq, fresh=True,
                    source_topic=TOPIC, source_fps=(len(recent)-1)*1e9/(recent[-1]-recent[0])
                    if len(recent)>1 and recent[-1]>recent[0] else None,
                    encode_ms=(time.monotonic_ns()-started)/1e6,
                    preview_skipped=max(0, last-previous-1) if previous >= 0 else 0)
            else:
                header, frame = dict(size=0, fresh=False), b''
            header['request_id'] = request_id
            if frame:
                header['source_age_ms'] = (time.time_ns()-int(header['stamp_ns']))/1e6
            encoded = json.dumps(header, allow_nan=False).encode()
            sys.stdout.buffer.write(struct.pack('!I', len(encoded)) + encoded + frame)
            sys.stdout.buffer.flush()
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        sub.unregister(); rospy.signal_shutdown('Depth preview closed')


if __name__ == '__main__':
    main()
