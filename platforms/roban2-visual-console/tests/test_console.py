"""Observation invariants and HTTP behavior, using synthetic samples only."""
import base64
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server

CONFIG = json.loads((server.ROOT / 'config.json').read_text())


def joint_packet(hub, stamp=1000., source_age_ms=5.):
    # Test fixtures never connect to hardware and never enter the running hub.
    names = [j['name'] for j in hub.model['joints']]
    return {'kind': 'joint', 'topic': '/rt/joint_state', 'stamp': stamp, 'source_age_ms': source_age_ms,
            'data': {'names': names, 'q': [0.01 * i for i in range(23)],
                     'v': [0.] * 23, 'vd': [0.] * 23, 'tau': [0.] * 23}}


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.hub = server.Hub(deepcopy(CONFIG))

    def test_missing_feedback_never_becomes_zero_pose_feedback(self):
        c = self.hub.channel('joint')
        self.assertEqual(c['status'], 'waiting')
        self.assertIsNone(c['data'])
        self.assertEqual(self.hub.model['joint_count'], 23)
        self.assertTrue(self.hub.snapshot()['read_only_robot'])

    def test_two_revo2_hands_have_correct_mounts_and_six_active_channels(self):
        model = self.hub.model
        self.assertEqual(model['hand_joint_count'], 22)
        self.assertEqual(model['kinematic_joint_count'], 45)
        urdf = ET.parse(server.ROOT / 'assets/model/robot.urdf').getroot()
        for side in ('left', 'right'):
            hand = [j for j in model['hand_joints'] if j['name'].startswith(side + '_')]
            self.assertEqual(len(hand), 11)
            self.assertEqual(len([j for j in hand if 'mimic' not in j]), 6)
            mount = urdf.find("joint[@name='%s_revo2_mount']" % side)
            self.assertIsNotNone(mount)
            self.assertEqual(mount.find('child').get('link'), side + '_hand_base_link')
            self.assertEqual(mount.find('parent').get('link'), 'zarm_' + side[0] + '7_end_effector')
        for mesh in urdf.iter('mesh'):
            path = server.ROOT / 'assets/model' / mesh.get('filename')
            self.assertTrue(path.is_file(), str(path))
        self.assertFalse(model['hand_geometry_driven'])

    def test_native_names_required_and_bad_numbers_rejected(self):
        for mutate in [lambda p: p['data']['q'].pop(),
                       lambda p: p['data']['q'].__setitem__(0, float('nan')),
                       lambda p: p['data']['names'].__setitem__(0, 'unverified_joint'),
                       lambda p: p['data']['v'].pop()]:
            p = joint_packet(self.hub)
            mutate(p)
            with self.assertRaises(ValueError):
                self.hub.ingest(p)
        self.assertIsNone(self.hub.channel('joint')['data'])

    def test_duplicate_and_out_of_order_feedback_rejected(self):
        self.hub.ingest(joint_packet(self.hub))
        self.assertEqual(self.hub.channel('joint')['status'], 'live')
        for stamp in (1000., 999.):
            with self.assertRaises(ValueError):
                self.hub.ingest(joint_packet(self.hub, stamp))
        self.assertEqual(self.hub.channel('joint')['stamp'], 1000.)

    def test_stale_source_and_transport_are_visible_preserving_last_values(self):
        self.hub.ingest(joint_packet(self.hub, source_age_ms=350.))
        self.assertEqual(self.hub.channel('joint')['status'], 'stale')
        self.hub.ingest(joint_packet(self.hub, stamp=1001.))
        self.hub.channels['joint']['_received_mono'] -= .3
        self.assertEqual(self.hub.channel('joint')['status'], 'stale')
        self.assertEqual(self.hub.channel('joint')['data']['q'][1], .01)

    def test_future_clock_is_not_reported_as_fresh(self):
        self.hub.ingest(joint_packet(self.hub, source_age_ms=-200.))
        self.assertEqual(self.hub.channel('joint')['status'], 'clock_error')

    def test_recording_limit_and_metadata_without_video(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(server, 'ROOT', Path(tmp)):
            (Path(tmp) / 'state/records').mkdir(parents=True)
            self.hub.config['record_limit_mb'] = .00001
            self.hub.start_recording()
            self.hub.ingest(joint_packet(self.hub))
            self.assertIsNone(self.hub.recording)
            record = json.loads((Path(tmp) / 'state/records' / self.hub.last_record).read_text())
            self.assertEqual(record['kind'], 'joint')
            self.assertNotIn('jpeg', record)
            self.assertEqual(self.hub.record_count, 1)


class HttpTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.app = server.create_app(deepcopy(CONFIG), readers=False)
        self.hub = self.app[server.HUB_KEY]
        self.client = TestClient(TestServer(self.app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()

    async def test_page_local_model_and_no_motion_api(self):
        for path in ('/', '/static/app.js', '/model/robot.urdf', '/api/state', '/api/model'):
            response = await self.client.get(path)
            self.assertEqual(response.status, 200, path)
            await response.read()
        response = await self.client.post('/api/action/move')
        self.assertEqual(response.status, 404)
        response = await self.client.post('/api/ingest')
        self.assertEqual(response.status, 404)

    async def test_cross_origin_recording_and_websocket_denied(self):
        response = await self.client.post('/api/action/record-start', headers={'Origin': 'https://unexpected.example'})
        self.assertEqual(response.status, 403)
        self.assertIsNone(self.hub.recording)
        response = await self.client.get('/api/ws', headers={'Origin': 'https://unexpected.example'})
        self.assertEqual(response.status, 403)

    async def test_multiple_websocket_viewers_share_cache_and_have_no_readers(self):
        first = await self.client.ws_connect('/api/ws')
        second = await self.client.ws_connect('/api/ws')
        self.hub.ingest(joint_packet(self.hub))
        for ws in (first, second):
            for _ in range(4):
                packet = await ws.receive_json(timeout=2)
                if packet['channels']['joint']['data']:
                    break
            self.assertEqual(len(packet['channels']['joint']['data']['q']), 23)
        self.assertEqual(len(self.hub.processes), 0)
        self.assertEqual(len(self.hub.clients), 2)
        await first.close()
        await second.close()

    async def test_missing_and_stale_camera_cannot_be_saved_as_live_snapshot(self):
        response = await self.client.get('/api/snapshot/rgb')
        self.assertEqual(response.status, 503)
        self.hub.ingest({'kind': 'rgb', 'stamp': 1000., 'source_age_ms': 2000.,
                         'jpeg': base64.b64encode(b'\xff\xd8\xff\xd9').decode(),
                         'data': {'width': 1, 'height': 1}})
        response = await self.client.get('/api/snapshot/rgb')
        self.assertEqual(response.status, 409)


if __name__ == '__main__':
    unittest.main()
