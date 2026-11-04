import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from data import Store, Tail, Run


def line(path, value):
    with path.open('a') as f:
        f.write(json.dumps(value) + '\n')


def row(at=100., t=0.):
    return dict(at=at, wall_elapsed_s=t, policy_elapsed_s=t/2, phase='body_chunk',
        model_target_q=[.3]*21, measured={'q':[.1]*21}, model_target_hand=[0.]*12,
        measured_hand=[1.]*12, bounded_request={'q':[.2]*21, 'hand':[0.]*12}, joint_source_stamp=at-.02)


def ack(received=100.1):
    return dict(event='ack', _received_wall=received, seq=1, requested={'q':[.2]*21,'hand':[0.]*12},
        native_reference={'hardware_q_rad':[.15]*21,'actual_hardware_q_rad':[.14]*21,
                          'header_sec':100, 'header_nanosec':0,'seq':50})


class DataTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.path=self.root/'20261103_140035_404398';self.path.mkdir()
        self.exec=self.path/'execution.jsonl';self.relay=self.path/'relay.jsonl'

    def test_partial_json_line_is_not_lost_or_published_early(self):
        tail=Tail(self.exec);self.exec.write_bytes(b'{"x":')
        self.assertEqual(tail.read()[0],[])
        with self.exec.open('ab') as f:f.write(b'1}\n')
        self.assertEqual(tail.read()[0],[{'x':1}]);self.assertEqual(tail.read()[0],[])

    def test_future_ack_never_appears_as_delivered(self):
        line(self.exec,row());line(self.exec,row(100.2,.2));line(self.relay,ack())
        frames=Run(self.path).frames()
        self.assertIsNone(frames[0]['ack']);self.assertEqual(frames[1]['ack_seq'],1)
        self.assertEqual(frames[1]['ack'],[.2]*21)

    def test_tracking_error_uses_paired_native_feedback_not_candidate(self):
        line(self.exec,row(100.2,.2));line(self.relay,ack())
        f=Run(self.path).frames()[0]
        self.assertAlmostEqual(f['native_error'][2],math.degrees(.01))
        self.assertAlmostEqual(f['model_error'][2],math.degrees(.2))
        self.assertNotEqual(f['native_error'][2],f['model_error'][2])

    def test_missing_native_feedback_is_unknown_not_zero(self):
        line(self.exec,row(100.2,.2));a=ack();del a['native_reference']['actual_hardware_q_rad'];line(self.relay,a)
        f=Run(self.path).frames()[0];self.assertIsNone(f['native_error']);self.assertIsNone(f['native_actual'])

    def test_incomplete_final_execution_line_is_retained_across_growth(self):
        r=Run(self.path);line(self.exec,row());self.assertEqual(len(r.frames()),1)
        new=json.dumps(row(100.2,.2)).encode()
        with self.exec.open('ab') as f:f.write(new[:100])
        r.last_refresh=0;self.assertEqual(len(r.frames(1)),0)
        with self.exec.open('ab') as f:f.write(new[100:]+b'\n')
        r.last_refresh=0;self.assertEqual(len(r.frames(1)),1)

    def test_truncation_resets_cursor_and_version(self):
        r=Run(self.path);line(self.exec,row());line(self.exec,row(100.2,.2));r.refresh()
        self.exec.write_text(json.dumps(row())+'\n');r.last_refresh=0;r.refresh()
        self.assertEqual(len(r.rows),1);self.assertEqual(r.version,1)

    def test_invalid_body_values_are_not_rendered(self):
        bad=row();bad['measured']['q'][0]=float('nan');line(self.exec,bad)
        r=Run(self.path);self.assertEqual(r.frames(),[]);self.assertEqual(r.skipped,1)

    def test_reports_and_raw_logs_remain_unchanged(self):
        line(self.exec,row());original=self.exec.read_bytes()
        (self.path/'report.json').write_text(json.dumps({'status':'BLOCKED_OR_STOPPED','error':'BODY_TRACKING_ERROR','applied_arm_speed_deg_s':60.}))
        r=Run(self.path);m=r.meta();self.assertTrue(m['finished']);self.assertFalse(m['live']);self.assertEqual(m['speed'],60.)
        r.frames();self.assertEqual(self.exec.read_bytes(),original)

    def test_run_path_traversal_and_external_symlink_rejected(self):
        s=Store(self.root)
        for name in ['../../etc','../20261103_140035_404398','20261103_140035_404398/../']:
            with self.subTest(name=name),self.assertRaises(ValueError):s.path(name)
        (self.root/'20261103_140000_000000').symlink_to('/tmp',target_is_directory=True)
        with self.assertRaises(ValueError):s.path('20261103_140000_000000')


if __name__=='__main__':unittest.main()
