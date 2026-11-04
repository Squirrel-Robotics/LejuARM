"""Geometry and invalid-observation checks for the read-only tabletop reader."""
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('table_angle', Path(__file__).resolve().parents[1]/'tools/read_table_angle.py')
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)
K = np.array([[460.6, 0, 323.5], [0, 460.7, 236.4], [0, 0, 1.]])
INFO = dict(K=K.ravel().tolist(), D=[0.]*8, width=640, height=480,
            frame_id='roban2_stage0_camera_color_optical_frame')


def plane(angle, roll=0., outliers=0.):
    v, u = np.mgrid[:480, :640]
    angle, roll = np.deg2rad([angle, roll])
    n = np.array([np.cos(angle)*np.sin(roll), np.cos(angle)*np.cos(roll), np.sin(angle)])
    z = 800/(n[0]*(u-K[0, 2])/K[0, 0]+n[1]*(v-K[1, 2])/K[1, 1]+n[2])
    rng = np.random.default_rng(99)
    z += rng.normal(0, .7, size=z.shape)
    bad = rng.random(z.shape) < outliers
    z[bad] *= rng.uniform(.7, 1.3, size=bad.sum())
    return z


def samples():
    return [dict(stamp=100+i*.1, age_ms=500-i*100,
                 frame_id=INFO['frame_id'], encoding='16UC1') for i in range(3)]


class TableAngleTests(unittest.TestCase):
    def test_known_plane_angles(self):
        for expected in (20., 35., 56., 80., 90.):
            with self.subTest(angle=expected):
                result = reader.fit_plane(plane(expected), K, (300, 140, 440, 320))
                self.assertAlmostEqual(result['angle_deg'], expected, delta=.08)

    def test_roll_and_depth_scale_do_not_change_inclination(self):
        for roll in (-25., 25.):
            result = reader.fit_plane(plane(56., roll)*.001, K, (300, 140, 440, 320))
            self.assertAlmostEqual(result['angle_deg'], 56., delta=.08)

    def test_outliers_rejected_by_robust_plane(self):
        result = reader.fit_plane(plane(56., outliers=.1), K, (300, 140, 440, 320))
        self.assertAlmostEqual(result['angle_deg'], 56., delta=.1)
        self.assertGreater(result['inlier_fraction'], .85)

    def test_multiframe_result_and_json_finite(self):
        result = reader.estimate([plane(56.)]*3, INFO, samples(), reader.DEFAULT_ROIS)
        self.assertAlmostEqual(result['angle_deg'], 56., delta=.08)
        self.assertEqual(result['motion_commands_sent'], 0)
        json.dumps(result, allow_nan=False)

    def test_blank_depth_rejected(self):
        with self.assertRaisesRegex(ValueError, 'INSUFFICIENT_VALID_TABLE_DEPTH'):
            reader.fit_plane(np.zeros((480, 640)), K, reader.DEFAULT_ROIS[0])

    def test_stale_or_duplicate_frame_rejected(self):
        for field, value, message in [('age_ms', 1501, 'STALE'), ('stamp', 99., 'ADVANCING')]:
            batch = samples()
            batch[-1][field] = value
            with self.assertRaisesRegex(ValueError, message):
                reader.estimate([plane(56.)]*3, INFO, batch, reader.DEFAULT_ROIS)

    def test_unaligned_or_distorted_depth_rejected(self):
        for changes, message in [({'frame_id': 'depth_frame'}, 'OPTICAL_FRAME'),
                                  ({'D': [.01]}, 'UNDISTORTION')]:
            with self.assertRaisesRegex(ValueError, message):
                reader.estimate([plane(56.)]*3, dict(INFO, **changes), samples(), reader.DEFAULT_ROIS)

    def test_conflicting_planes_rejected(self):
        depth = plane(56.)
        depth[:, 400:] = plane(40.)[:, 400:]
        with self.assertRaisesRegex(ValueError, 'INCONSISTENT'):
            reader.estimate([depth]*3, INFO, samples(), [(280, 200, 380, 400), (440, 200, 540, 400)])


if __name__ == '__main__':
    unittest.main()
