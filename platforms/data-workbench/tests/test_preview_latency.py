from pathlib import Path
import json
import io
import os
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'vr_Data/.pipeline/dashboard'), str(ROOT/'vr_Data/.pipeline/scripts')]
from workbench import Streams, preview_is_fresh

FAKE_ROS = '''
import threading, time, struct
from types import SimpleNamespace as S
class Subscriber:
 def __init__(self, topic, kind, callback, **kw):
  self.running = True
  def emit():
   seq = 0
   while self.running:
    seq += 1
    stamp = time.time_ns()
    h = S(seq=seq, stamp=S(to_nsec=lambda stamp=stamp:stamp), frame_id='color_optical_frame')
    msg = S(header=h, width=2, height=1, step=4, is_bigendian=False,
            encoding='16UC1', format='jpeg', data=struct.pack('<2H',1000,2000))
    callback(msg)
    time.sleep(.005)
  threading.Thread(target=emit, daemon=True).start()
 def unregister(self): self.running = False
def init_node(*a, **kw): pass
def signal_shutdown(*a, **kw): pass
'''


def packet(pipe):
    def read(n):
        data = bytearray()
        while len(data) < n:
            if not select.select([pipe], [], [], 3)[0]:
                raise TimeoutError('Sender did not answer a frame request')
            part = os.read(pipe.fileno(), n-len(data))
            if not part:
                raise EOFError()
            data.extend(part)
        return bytes(data)
    size = struct.unpack('!I', read(4))[0]
    header = json.loads(read(size))
    return header, read(header['size'])


class PreviewLatencyTests(unittest.TestCase):
    def test_direct_connection_failure_falls_back_to_the_lower_route(self):
        stopping = threading.Event()
        stream = Streams({}, stopping)
        calls = []
        class Output(io.BytesIO):
            def read(self, n):
                value = super().read(n)
                if self.tell() == len(self.getvalue()):
                    stopping.set()
                return value
        class Process:
            def __init__(self, stdout):
                self.stdin, self.stdout = io.BytesIO(), stdout
            def terminate(self): pass
            def wait(self, timeout=None): pass
        header = dict(size=1, fresh=True, source_age_ms=1, request_id='123')
        encoded = json.dumps(header).encode()
        processes = [Process(io.BytesIO()), Process(Output(struct.pack('!I', len(encoded))+encoded+b'x'))]
        def process(command, **kwargs):
            calls.append(kwargs)
            proc = processes[len(calls)-1]
            stream.processes.append(proc)
            return proc
        stream.process = process
        with patch('workbench.time.monotonic_ns', return_value=123), patch.object(stopping, 'wait', return_value=False), \
                patch('workbench.read_preview_bytes', side_effect=lambda stream, n, deadline: stream.read(n) if n == 0 or stream.tell() < len(stream.getvalue()) else (_ for _ in ()).throw(EOFError())):
            stream.preview_loop()
        self.assertEqual(calls[0]['host'], 'roban-upper')
        self.assertIn('StrictHostKeyChecking=yes', calls[0]['options'])
        self.assertNotIn('host', calls[1])
        self.assertEqual(stream.preview['transport'], 'via_lower')
        self.assertTrue(stream.preview['fresh'])

    def test_receiving_a_buffered_frame_does_not_make_it_fresh(self):
        now = time.time_ns()
        self.assertFalse(preview_is_fresh(dict(fresh=True, received_ns=now, latency_bound_ms=2000), now))
        self.assertFalse(preview_is_fresh(dict(fresh=True, received_ns=now-600_000_000, latency_bound_ms=120), now))
        self.assertTrue(preview_is_fresh(dict(fresh=True, received_ns=now-20_000_000, latency_bound_ms=100), now))

    def test_snapshot_marks_rgb_and_depth_stale_independently(self):
        stream = Streams({}, threading.Event())
        now = time.time_ns()
        stream.preview = dict(fresh=True, received_ns=now, latency_bound_ms=80)
        stream.depth = dict(fresh=True, received_ns=now, latency_bound_ms=11000, source_fps=60)
        preview, _ = stream.snapshot()
        self.assertTrue(preview['fresh'])
        self.assertFalse(preview['depth']['fresh'])
        self.assertIsNone(preview['depth']['source_fps'])
        self.assertTrue(stream.depth['fresh'])

    def test_slow_receivers_get_latest_frame_without_unsolicited_queue(self):
        for filename in ('vr_dashboard_preview.py', 'vr_dashboard_depth.py'):
            with self.subTest(sender=filename), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                (root/'rospy.py').write_text(FAKE_ROS)
                (root/'sensor_msgs').mkdir()
                (root/'sensor_msgs/__init__.py').write_text('')
                (root/'sensor_msgs/msg.py').write_text('class Image: pass\nclass CompressedImage: pass\nclass CameraInfo: pass\n')
                env = dict(os.environ, PYTHONPATH=str(root))
                path = ROOT/'upper/vr_vision_recorder'/filename
                proc = subprocess.Popen([sys.executable, '-u', str(path), '--pull'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                try:
                    self.assertFalse(select.select([proc.stdout], [], [], .1)[0])
                    proc.stdin.write(b'100\n'); proc.stdin.flush()
                    first, frame = packet(proc.stdout)
                    self.assertEqual(first['request_id'], '100')
                    self.assertTrue(frame)
                    # Simulate a consumer stalled for many camera periods.
                    time.sleep(.15)
                    self.assertFalse(select.select([proc.stdout], [], [], 0)[0])
                    proc.stdin.write(b'200\n'); proc.stdin.flush()
                    latest, frame = packet(proc.stdout)
                    self.assertEqual(latest['request_id'], '200')
                    self.assertGreater(latest['ros_seq']-first['ros_seq'], 10)
                    self.assertLess(latest['source_age_ms'], 50)
                    proc.stdin.close()
                    proc.wait(timeout=3)
                    self.assertEqual(proc.returncode, 0, proc.stderr.read().decode())
                finally:
                    if proc.poll() is None:
                        proc.kill(); proc.wait()
                    proc.stdout.close(); proc.stderr.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
