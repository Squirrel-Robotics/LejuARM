"""Exercise the source boundary through real S17 IK and the HEFT contract."""

import time
from types import SimpleNamespace

import numpy as np
import pytest
import mujoco as mj

from test_xsens_bvh import payload
from leju_xsens_gmr.xsens_bvh import calibration_from_payload
from leju_xsens_gmr.motion_retarget import MotionRetargeter
from leju_gmr_common.heft_extractor import HeftJointExtractor
from leju_gmr_common.params import HEFT_JOINT_NAMES


def test_bvh_calibration_retargets_s17_with_limits(payload):
    pose, height = calibration_from_payload(payload)
    rt = MotionRetargeter("bvh_xsense", "roban_s17",
                          actual_human_height=height, cali_pose=pose)
    extractor = HeftJointExtractor(rt.model)
    for step in range(12):
        frame = {name: [p + [step * .001, 0., .9], q.copy()]
                 for name, (p, q) in pose.items()}
        qpos = rt.retarget(frame)
        assert np.isfinite(qpos).all()
        joints = extractor.extract_qpos(qpos)
        assert joints.shape == (21,)
        for i, name in enumerate(HEFT_JOINT_NAMES):
            jid = mj.mj_name2id(rt.model, mj.mjtObj.mjOBJ_JOINT, name)
            np.testing.assert_equal(joints[i], qpos[rt.model.jnt_qposadr[jid]])
            if rt.model.jnt_limited[jid]:
                lo, hi = rt.model.jnt_range[jid]
                assert lo - 1e-6 <= joints[i] <= hi + 1e-6


class CapturedDds:
    def __init__(self, **kwargs):
        self.references = []
        self.bones = []
        self.resume_ready = []

    def publish_heft_reference(self, **kwargs):
        self.references.append(kwargs)

    def publish_head(self, *args):
        pass

    def publish_bone_poses(self, *args, **kwargs):
        self.bones.append(args)

    def publish_heft_resume_ready(self):
        self.resume_ready.append(True)


@pytest.fixture
def app(monkeypatch):
    # Actual shared consumer and solver; only external transport is replaced.
    from leju_gmr_common import app as common_app
    from leju_xsens_gmr.main import XsensGmrApp, build_arg_parser
    monkeypatch.setattr(common_app, "DdsPublishers", CapturedDds)
    monkeypatch.setattr(XsensGmrApp, "_create_receiver", lambda self, args:
                        SimpleNamespace(total_frames=1, stop=lambda: None))
    args = build_arg_parser().parse_args(["--control-topic", ""])
    result = XsensGmrApp(args)
    yield result
    result.stop()


def test_no_ik_or_reference_before_bvh(app, payload):
    pose, _ = calibration_from_payload(payload)
    app._on_xsens_frame(pose, time.time())
    app._tick()
    assert app.retarget is None
    assert app.dds.references == []
    assert not app.calibrated


def test_xsens_to_shared_heft_publication_and_freeze(app, payload):
    pose, height = calibration_from_payload(payload)
    app._on_bvh(pose, height)
    app._tick()
    assert app.calibrated and app.calibration_epoch == 1
    assert app.dds.references == []
    for _ in range(10):
        frame = {name: [p + [0., 0., .9], q.copy()]
                 for name, (p, q) in pose.items()}
        app._on_xsens_frame(frame, time.time())
        app._tick()
    msg = app.dds.references[-1]
    assert msg["valid"] and msg["calibrated"]
    assert msg["robot_version"] == "roban_s17"
    assert msg["schema_version"] == 1
    assert len(msg["q"]) == 21 and len(msg["root_quat_wxyz"]) == 4
    expected = app.extractor.extract_qpos(app.retarget.configuration.data.qpos)
    expected[0] = -expected[0]  # Existing S17 deployment waist direction.
    np.testing.assert_allclose(msg["q"], expected)
    assert not app.dds.bones  # Never label Xsens 20-bone data as Quest 24-bone.

    count = len(app.dds.references)
    app._on_xsens_frame(frame, time.time() - 1.)
    app._tick()
    assert len(app.dds.references) == count  # A stale source cannot become fresh.
    app._freeze_stream()
    app._tick()
    held = app.dds.references[-1]
    assert held["valid"]
    np.testing.assert_allclose(held["q"], msg["q"])
    assert held["qd"] is None  # Shared contract omits velocity for held frames.
    app._resume_stream()
    app._on_xsens_frame(frame, time.time())
    app._tick()
    np.testing.assert_allclose(app.dds.references[-1]["root_quat_wxyz"],
                               held["root_quat_wxyz"], atol=1e-5)
    assert app.dds.resume_ready


def test_bvh_recalibration_resets_heading_and_epoch(app, payload):
    pose, height = calibration_from_payload(payload)
    app._on_bvh(pose, height)
    app._tick()
    app._run_calibration("upper_body", 0.)
    assert app.calibration_epoch == 2
    assert not app.heading.initialized
    assert app.prev_dof is None


def test_mvn_binary_to_bvh_calibrated_heft(app, payload):
    from scipy.spatial.transform import Rotation
    from test_xsens_receiver import full_pose, pose_record, calibrate
    from leju_xsens_gmr.xsens_receiver import XsensFrameDecoder, BODY_MAPPING
    pose, height = calibration_from_payload(payload)
    app._on_bvh(pose, height)
    app._tick()
    decoder = XsensFrameDecoder(expected_character_id=0)
    assert decoder.feed(full_pose(), 0.) is None
    calibrate(decoder, now=0., left=(0., 0., 0.), right=(0., 0., 0.))
    # Generate source rotations from independent basis arithmetic. The output
    # canonical rotation is R_raw * B.T, while positions return to raw axes.
    basis = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
    records = {}
    for name, sid, _ in BODY_MAPPING:
        p, q = pose[name]
        raw_rotation = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix() @ basis
        xyzw = Rotation.from_matrix(raw_rotation).as_quat()
        records[sid] = pose_record(sid, p + [0., 0., .9], xyzw[[3, 0, 1, 2]])
    for sample in range(10):
        frame = decoder.feed(full_pose(sample=sample, overrides=records), .01 * sample)
        app._on_xsens_frame(frame, time.time())
        app._tick()
    reference = app.dds.references[-1]
    assert reference["valid"] and len(reference["q"]) == 21
    assert np.isfinite(reference["q"]).all()


def test_failed_calibration_invalidates_stream(app, payload, monkeypatch):
    from leju_xsens_gmr import main as xsens_main
    app._on_bvh(*calibration_from_payload(payload))
    app._tick()
    def fail(*args, **kwargs):
        raise ValueError("bad robot config")
    monkeypatch.setattr(xsens_main, "MotionRetargeter", fail)
    with pytest.raises(ValueError, match="bad robot config"):
        app._run_calibration("whole_body", 0.)
    assert not app.calibrated
