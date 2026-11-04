"""Exercise a growing inference log through HTTP using an isolated temp directory."""
from functools import partial
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from data import Store
from server import Handler
from test_data import row, ack, line


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.run_id = '20261103_120000_000000'
        self.path = self.root / self.run_id
        self.path.mkdir()
        self.store = Store(self.root)
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Handler, store=self.store))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:' + str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.tmp.cleanup()

    def get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=3) as r:
            return json.load(r)

    def data(self, after=0, version=0):
        self.store.get(self.run_id).last_refresh = 0
        return self.get(f'/api/run/{self.run_id}/data?after={after}&version={version}')

    def test_new_run_append_partial_line_and_finish(self):
        # No robot connection: the complete live lifecycle comes from this temp log.
        self.assertFalse(self.get('/api/runs')[0]['has_motion_log'])
        self.assertEqual(self.data()['frames'], [])
        execution = self.path / 'execution.jsonl'
        line(execution, row())
        first = self.data()
        self.assertEqual(len(first['frames']), 1)
        self.assertTrue(first['meta']['live'])
        self.assertIsNone(first['frames'][0]['ack'])
        a = ack(100.1)
        line(self.path / 'relay.jsonl', a)
        payload = json.dumps(row(100.2, .2))
        with execution.open('a') as f:
            f.write(payload[:80])
        self.assertEqual(self.data(after=1)['frames'], [])
        with execution.open('a') as f:
            f.write(payload[80:] + '\n')
        second = self.data(after=1)
        self.assertEqual([f['i'] for f in second['frames']], [1])
        self.assertEqual(second['frames'][0]['ack_seq'], 1)
        (self.path / 'report.json').write_text(json.dumps({'status': 'BLOCKED_OR_STOPPED', 'error': 'TEST_ONLY'}))
        finished = self.data(after=2)
        self.assertTrue(finished['meta']['finished'])
        self.assertFalse(finished['meta']['live'])
        self.assertEqual(finished['frames'], [])
        self.assertTrue(self.get('/api/runs')[0]['finished'])

    def test_reset_csv_and_read_only_http_contract(self):
        execution = self.path / 'execution.jsonl'
        line(execution, row())
        line(execution, row(100.2, .2))
        self.assertEqual(len(self.data()['frames']), 2)
        execution.write_text(json.dumps(row()) + '\n')
        reset = self.data(after=2)
        self.assertTrue(reset['reset'])
        self.assertEqual(reset['meta']['version'], 1)
        self.assertEqual(len(reset['frames']), 1)
        with urllib.request.urlopen(self.url + f'/api/run/{self.run_id}/csv') as r:
            csv = r.read().decode()
        self.assertIn('native_actual_zarm_r2_joint_rad', csv)
        self.assertEqual(len(csv.splitlines()), 2)
        before = execution.read_bytes()
        request = urllib.request.Request(self.url + '/api/run/' + self.run_id, method='POST', data=b'{}')
        with self.assertRaises(urllib.error.HTTPError) as raised:
            urllib.request.urlopen(request, timeout=3)
        self.assertEqual(raised.exception.code, 501)
        raised.exception.close()
        self.assertEqual(execution.read_bytes(), before)
        self.assertTrue(self.get('/api/health')['read_only'])


if __name__ == '__main__':
    unittest.main()
