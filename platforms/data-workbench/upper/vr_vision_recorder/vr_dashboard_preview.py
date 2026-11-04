#!/usr/bin/env python3
"""Stream latest native compressed RGB over SSH. No device or recording writes."""
from collections import deque
import json
import signal
import struct
import sys
import threading
import time


def main():
    import rospy
    from sensor_msgs.msg import CompressedImage, CameraInfo
    pull = '--pull' in sys.argv[1:]
    condition = threading.Condition()
    stamps = deque(maxlen=180)
    state = {'counter': 0, 'frame': None, 'width': 640, 'height': 480, 'seq_gaps': 0}
    previous_seq = None
    def image(msg):
        nonlocal previous_seq
        if 'jpeg' not in msg.format.lower():
            return
        with condition:
            stamp = msg.header.stamp.to_nsec()
            stamps.append(stamp)
            if previous_seq is not None and msg.header.seq > previous_seq+1:
                state['seq_gaps'] += msg.header.seq-previous_seq-1
            previous_seq = msg.header.seq
            state.update(counter=state['counter']+1, frame=bytes(msg.data), stamp_ns=str(stamp),
                         ros_seq=msg.header.seq, arrival_ns=time.time_ns())
            condition.notify_all()
    def info(msg):
        with condition:
            state.update(width=msg.width, height=msg.height)
    rospy.init_node('roban_workbench_preview', anonymous=True, disable_signals=True)
    rgb = rospy.Subscriber('/roban2_stage0_camera/color/image_raw/compressed', CompressedImage,
                           image, queue_size=2, buff_size=4*1024*1024)
    camera = rospy.Subscriber('/roban2_stage0_camera/color/camera_info', CameraInfo, info, queue_size=1)
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
                condition.wait_for(lambda:state['counter'] != last, timeout=1)
                fresh = state.get('arrival_ns') and time.time_ns()-state['arrival_ns'] < 500_000_000
                frame = state['frame'] if fresh and state['counter'] != last else b''
                last = state['counter']
                recent = list(stamps)
                header = {k:v for k,v in state.items() if k!='frame'}
                header.update(size=len(frame), source_fps=(len(recent)-1)*1e9/(recent[-1]-recent[0])
                    if fresh and len(recent)>1 and recent[-1]>recent[0] else None, fresh=bool(fresh))
            header['request_id'] = request_id
            if frame:
                header['source_age_ms'] = (time.time_ns()-int(header['stamp_ns']))/1e6
            encoded = json.dumps(header, allow_nan=False).encode()
            sys.stdout.buffer.write(struct.pack('!I',len(encoded))+encoded+frame)
            sys.stdout.buffer.flush()
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        rgb.unregister();camera.unregister();rospy.signal_shutdown('preview closed')


if __name__=='__main__':
    main()
