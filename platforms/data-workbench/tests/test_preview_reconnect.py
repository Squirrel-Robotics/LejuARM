import io
import json
import os
from pathlib import Path
import struct
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'vr_Data/.pipeline/dashboard'), str(ROOT/'vr_Data/.pipeline/scripts')]
from workbench import Streams, read_preview_bytes


class PreviewReconnectTests(unittest.TestCase):
    def test_partial_packet_has_one_deadline(self):
        reader, writer = os.pipe()
        try:
            with os.fdopen(reader, 'rb') as stream:
                os.write(writer, b'123')
                with self.assertRaises(TimeoutError):
                    read_preview_bytes(stream, 4, time.monotonic()+.03)
        finally:
            os.close(writer)

    def test_empty_responses_trigger_reconnect_even_while_ssh_is_alive(self):
        stop = threading.Event()
        streams = Streams({'upper_preview_host': '192.168.110.85'}, stop)
        calls, clock = [], [100.0]

        def response(fresh):
            frame = b'jpeg' if fresh else b''
            header = json.dumps(dict(size=len(frame), fresh=fresh, source_age_ms=10,
                                     request_id='123')).encode()
            return struct.pack('!I', len(header))+header+frame

        class Process:
            def __init__(self, fresh):
                self.fresh = fresh
                self.stdin, self.stdout = io.BytesIO(), io.BytesIO(response(fresh)*3)
                self.terminated = False
            def terminate(self):
                self.terminated = True
            def wait(self, timeout=None):
                return 0

        procs = [Process(False), Process(True)]
        def process(command, **kwargs):
            p = procs[len(calls)]
            calls.append(kwargs)
            streams.processes.append(p)
            return p

        def read(stream, length, deadline):
            data = stream.read(length)
            if len(data) != length:
                raise AssertionError('Expected watchdog to reconnect before EOF')
            if length == 0:
                clock[0] += 2
            elif data == b'jpeg':
                stop.set()
            return data

        with patch.object(streams, 'process', side_effect=process), \
             patch('workbench.time.monotonic', side_effect=lambda: clock[0]), \
             patch('workbench.time.monotonic_ns', return_value=123), \
             patch('workbench.read_preview_bytes', side_effect=read), \
             patch.object(stop, 'wait', return_value=False):
            streams.preview_loop()
        self.assertTrue(procs[0].terminated)
        self.assertIn('HostName=192.168.110.85', calls[0]['options'])
        self.assertIn('StrictHostKeyChecking=yes', calls[0]['options'])
        self.assertNotIn('host', calls[1])
        self.assertEqual(streams.packet[1], b'jpeg')
        self.assertEqual(streams.preview['transport'], 'via_lower')


if __name__ == '__main__':
    unittest.main()
