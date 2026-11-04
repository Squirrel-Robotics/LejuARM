"""Unit tests for leju_pico_gmr: model loading, IK config validation,
HEFT joint extraction permutation, and retargeting sanity."""

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")

from leju_pico_gmr.params import (HEFT_JOINT_NAMES, IK_CONFIG_DICT,
                                  ROBOT_XML_DICT, resolve_robot_name)


def test_resolve_robot_aliases():
    assert resolve_robot_name("roban_s17") == "roban_s17"
    assert resolve_robot_name("robans17") == "roban_s17"
    assert resolve_robot_name("S17") == "roban_s17"
    with pytest.raises(KeyError):
        resolve_robot_name("kuavo_s52")


def test_model_loads():
    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT["roban_s17"]))
    assert model.nq == 7 + 23  # free joint + 23 hinges (21 policy + 2 head)


def test_ik_config_bodies_exist():
    import json
    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT["roban_s17"]))
    with open(IK_CONFIG_DICT["bvh_motive_streaming"]["roban_s17"]) as f:
        cfg = json.load(f)
    for table_name in ("ik_match_table1", "ik_match_table2"):
        for body in cfg.get(table_name, {}):
            assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body) >= 0, \
                f"{table_name} body '{body}' missing from S17 model"


def test_heft_extractor_permutation():
    from leju_gmr_common.heft_extractor import HeftJointExtractor

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT["roban_s17"]))
    ext = HeftJointExtractor(model)
    assert ext.num_joints == 21

    # Fill qpos with a unique marker per joint and verify extraction order.
    qpos = np.zeros(model.nq)
    expected = np.zeros(21)
    for i, name in enumerate(HEFT_JOINT_NAMES):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        addr = model.jnt_qposadr[jid]
        qpos[addr] = 100.0 + i
        expected[i] = 100.0 + i

    out = ext.extract_qpos(qpos)
    np.testing.assert_array_equal(out, expected)


def test_heft_extractor_rejects_missing_joint():
    from leju_gmr_common.heft_extractor import HeftJointExtractor

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT["roban_s17"]))
    with pytest.raises(ValueError):
        HeftJointExtractor(model, joint_names=["nonexistent_joint"])


def test_retarget_neutral_frame_finite():
    """A synthetic neutral standing frame retargets to finite qpos within limits."""
    from leju_pico_gmr.motion_retarget import MotionRetargeter

    rt = MotionRetargeter(
        src_human="bvh_motive_streaming",
        tgt_robot="roban_s17",
        actual_human_height=None,
        cali_pose=None,
    )

    # Synthetic T-pose-like human frame (approx positions in meters, identity rot)
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    frame = {
        "Hips": [np.array([0.0, 0.0, 0.95]), identity.copy()],
        "Spine": [np.array([0.0, 0.0, 1.15]), identity.copy()],
        "Neck": [np.array([0.0, 0.0, 1.45]), identity.copy()],
        "Head": [np.array([0.0, 0.0, 1.60]), identity.copy()],
        "LeftShoulder": [np.array([0.0, 0.10, 1.40]), identity.copy()],
        "LeftArm": [np.array([0.0, 0.20, 1.40]), identity.copy()],
        "LeftForeArm": [np.array([0.0, 0.25, 1.15]), identity.copy()],
        "LeftHand": [np.array([0.0, 0.28, 0.90]), identity.copy()],
        "RightShoulder": [np.array([0.0, -0.10, 1.40]), identity.copy()],
        "RightArm": [np.array([0.0, -0.20, 1.40]), identity.copy()],
        "RightForeArm": [np.array([0.0, -0.25, 1.15]), identity.copy()],
        "RightHand": [np.array([0.0, -0.28, 0.90]), identity.copy()],
        "LeftUpLeg": [np.array([0.0, 0.10, 0.90]), identity.copy()],
        "LeftLeg": [np.array([0.0, 0.10, 0.50]), identity.copy()],
        "LeftFoot": [np.array([0.0, 0.10, 0.08]), identity.copy()],
        "LeftToeBase": [np.array([0.10, 0.10, 0.02]), identity.copy()],
        "RightUpLeg": [np.array([0.0, -0.10, 0.90]), identity.copy()],
        "RightLeg": [np.array([0.0, -0.10, 0.50]), identity.copy()],
        "RightFoot": [np.array([0.0, -0.10, 0.08]), identity.copy()],
        "RightToeBase": [np.array([0.10, -0.10, 0.02]), identity.copy()],
    }

    qpos = rt.retarget(frame)
    assert np.all(np.isfinite(qpos))

    # joint limits check for hinge joints
    model = rt.model
    for jid in range(model.njnt):
        if model.jnt_type[jid] != mj.mjtJoint.mjJNT_HINGE:
            continue
        if not model.jnt_limited[jid]:
            continue
        addr = model.jnt_qposadr[jid]
        lo, hi = model.jnt_range[jid]
        margin = 1e-6
        assert lo - margin <= qpos[addr] <= hi + margin, \
            f"joint {mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jid)} out of range"


def test_forearm_position_offset_is_hips_anchored():
    """Elbow rotation must not rotate its calibrated position offset."""
    from leju_pico_gmr.motion_retarget import MotionRetargeter

    identity = np.array([1.0, 0.0, 0.0, 0.0])
    cali_pose = {
        "Hips": [np.array([0.2, -0.1, 1.0]), identity.copy()],
        "LeftForeArm": [np.array([0.45, 0.3, 1.25]), identity.copy()],
    }
    rt = MotionRetargeter(
        src_human="bvh_motive_streaming",
        tgt_robot="roban_s17",
        cali_pose=cali_pose,
        cali_option="upper_body",
    )
    assert "LeftForeArm" in rt.parent_frame_position_bodies

    data = mj.MjData(rt.model)
    mj.mj_forward(rt.model, data)
    expected_offset = data.body("base_link").xmat.reshape(3, 3).T @ (
        cali_pose["LeftForeArm"][0] - data.body("zarm_l4_link").xpos)
    np.testing.assert_allclose(
        rt.pos_offsets1["LeftForeArm"], expected_offset, atol=1e-12)

    quarter_turn_z = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
    forearm_pos = np.array([0.55, 0.25, 1.3])

    def converted(hips_quat, forearm_quat):
        frame = {
            "Hips": [np.array([0.2, -0.1, 1.0]), hips_quat],
            "LeftForeArm": [forearm_pos.copy(), forearm_quat],
        }
        return rt.offset_human_data_robotframe(
            frame, rt.pos_offsets1, rt.rot_offsets1)["LeftForeArm"]

    base_pos, base_quat = converted(identity, identity)
    elbow_rot_pos, elbow_rot_quat = converted(identity, quarter_turn_z)
    hips_rot_pos, _ = converted(quarter_turn_z, identity)

    np.testing.assert_allclose(elbow_rot_pos, base_pos, atol=1e-12)
    assert not np.allclose(elbow_rot_quat, base_quat)
    assert not np.allclose(hips_rot_pos, base_pos)


def test_viewer_draws_scaled_offset_human_targets():
    from leju_gmr_common.robot_viewer import (SKELETON_CONNECTIONS,
                                             draw_human_targets)

    model = mj.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
    scene = mj.MjvScene(model, maxgeom=16)
    targets = {
        "Hips": [np.array([0.1, 0.2, 0.3]),
                 np.array([1.0, 0.0, 0.0, 0.0])],
        "Spine": [np.array([0.4, 0.5, 0.6]),
                  np.array([1.0, 0.0, 0.0, 0.0])],
    }

    draw_human_targets(scene, targets)

    assert scene.ngeom == len(targets) + 1
    np.testing.assert_allclose(scene.geoms[0].pos, targets["Hips"][0])
    np.testing.assert_allclose(scene.geoms[1].pos, targets["Spine"][0])
    assert ("Hips", "Spine") in SKELETON_CONNECTIONS


def test_viewer_uses_sim_scene_and_stands_on_floor():
    from leju_gmr_common.params import ROBOT_XML_DICT
    from leju_gmr_common.robot_viewer import (
        apply_viewer_pose, reset_viewer_to_home, resolve_viewer_scene_xml)

    robot_xml = ROBOT_XML_DICT["roban_s17"]
    scene_xml = resolve_viewer_scene_xml(robot_xml)
    assert scene_xml.endswith("scene_rl.xml")

    robot = mj.MjModel.from_xml_path(str(robot_xml))
    vis = mj.MjModel.from_xml_path(scene_xml)
    assert vis.nq == robot.nq
    floor_id = mj.mj_name2id(vis, mj.mjtObj.mjOBJ_GEOM, "floor")
    assert floor_id >= 0
    assert vis.geom_rgba[floor_id, 3] > 0.5
    assert mj.mj_name2id(vis, mj.mjtObj.mjOBJ_TEXTURE, "groundplane") >= 0

    data = mj.MjData(vis)
    base_id = mj.mj_name2id(vis, mj.mjtObj.mjOBJ_BODY, "base_link")

    def _assert_standing(sole_z):
        waist_z = float(data.xpos[base_id, 2])
        np.testing.assert_allclose(sole_z, 0.001, atol=0.008)
        assert waist_z > 0.6
        assert waist_z - sole_z > 0.6

    # Window-open pose: qpos0 parks the waist on the floor.
    _dz, sole_z = reset_viewer_to_home(vis, data)
    _assert_standing(sole_z)
    assert vis.geom_rgba[floor_id, 3] > 0.5

    for root_z in (0.0, 0.73, -0.2):
        qpos = vis.qpos0.copy()
        qpos[2] = root_z
        _dz, sole_z = apply_viewer_pose(vis, data, qpos)
        _assert_standing(sole_z)


def test_pico_frame_mapping():
    from leju_pico_gmr.calibration import (PICO_TO_LAFAN, pico_body_names,
                                           world_matrices_to_gmr_frame)

    # 25 identity matrices (24 bones + FOOT_STATE)
    mats = np.tile(np.eye(4), (25, 1, 1))
    for i in range(25):
        mats[i][:3, 3] = [0.0, 0.0, float(i) * 0.1]

    frame = world_matrices_to_gmr_frame(mats)
    assert set(frame.keys()) == set(PICO_TO_LAFAN.values())
    # pelvis (index 0) maps to Hips
    np.testing.assert_allclose(frame["Hips"][0], [0.0, 0.0, 0.0])
    # head (index 15) maps to Head
    np.testing.assert_allclose(frame["Head"][0], [0.0, 0.0, 1.5])
