"""Passive RGB health check after restoring camera startup; no DDS writers."""
import json
from pathlib import Path
import statistics
import threading
import time

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo

rospy.init_node('vr_rgb_startup_health', anonymous=True, disable_signals=True)
rows, info = [], {}
lock = threading.Lock()

def frame(message):
    with lock:
        rows.append((message.header.stamp.to_nsec(), message.header.seq, time.time_ns()))

def calibration(message):
    info.update(width=message.width, height=message.height)

rgb = rospy.Subscriber('/roban2_stage0_camera/color/image_raw/compressed', CompressedImage,
                       frame, queue_size=120, buff_size=16*1024*1024)
camera = rospy.Subscriber('/roban2_stage0_camera/color/camera_info', CameraInfo,
                          calibration, queue_size=1)
time.sleep(20)
rgb.unregister()
camera.unregister()
assert len(rows) > 900, len(rows)
gaps = [b[0]-a[0] for a,b in zip(rows, rows[1:])]
result = {'frames':len(rows), 'observed_fps':(len(rows)-1)*1e9/(rows[-1][0]-rows[0][0]),
          'median_interval_ms':statistics.median(gaps)/1e6,
          'max_interval_ms':max(gaps)/1e6,
          'sequence_missing_frames':sum(max(0,b[1]-a[1]-1) for a,b in zip(rows,rows[1:])),
          'timestamp_gap_events':sum(t>25_000_000 for t in gaps),
          'calibration':info}
assert 57 <= result['observed_fps'] <= 63, result
assert info == {'width':640,'height':480}, info
result['passed'] = True
directory = Path('/home/leju_kuavo/vr_vision_recorder/validation_20261026')
directory.mkdir(exist_ok=True)
(directory/'camera_health.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
rospy.signal_shutdown('health check complete')
