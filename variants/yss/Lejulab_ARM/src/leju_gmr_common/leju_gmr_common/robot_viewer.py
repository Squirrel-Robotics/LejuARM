"""MuJoCo viewer for debugging robot motion and human IK targets."""

from pathlib import Path

import leju_gmr_common.gl_backend  # noqa: F401  sets MUJOCO_GL before mujoco
import mujoco as mj
import numpy as np


SKELETON_CONNECTIONS = [
    ("Hips", "Spine"), ("Spine", "Neck"), ("Neck", "Head"),
    ("Spine", "LeftShoulder"), ("LeftShoulder", "LeftArm"),
    ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
    ("Spine", "RightShoulder"), ("RightShoulder", "RightArm"),
    ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ("Hips", "LeftUpLeg"), ("LeftUpLeg", "LeftLeg"),
    ("LeftLeg", "LeftFoot"), ("LeftFoot", "LeftToeBase"),
    ("Hips", "RightUpLeg"), ("RightUpLeg", "RightLeg"),
    ("RightLeg", "RightFoot"), ("RightFoot", "RightToeBase"),
]


def draw_human_targets(scene, human_motion_data, pos_offset=None):
    """Draw the scaled, calibration-offset GMR targets into a user scene."""
    scene.ngeom = 0
    if not human_motion_data:
        return

    offset = (np.zeros(3, dtype=np.float64) if pos_offset is None
              else np.asarray(pos_offset, dtype=np.float64))
    identity = np.eye(3).reshape(-1)

    for pos, _ in human_motion_data.values():
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_SPHERE,
            size=[0.025, 0.025, 0.025],
            pos=np.asarray(pos, dtype=np.float64) + offset,
            mat=identity,
            rgba=[0.2, 0.6, 1.0, 0.8],
        )
        scene.ngeom += 1

    for start_name, end_name in SKELETON_CONNECTIONS:
        if start_name not in human_motion_data or end_name not in human_motion_data:
            continue
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mj.mjv_connector(
            geom,
            type=mj.mjtGeom.mjGEOM_CAPSULE,
            width=0.008,
            from_=np.asarray(human_motion_data[start_name][0]) + offset,
            to=np.asarray(human_motion_data[end_name][0]) + offset,
        )
        geom.rgba[:] = [0.8, 0.8, 0.8, 0.6]
        scene.ngeom += 1


def resolve_viewer_scene_xml(robot_xml):
    """Use the same scene as leju-mujoco-sim (scene_rl.xml), then scene.xml."""
    if not robot_xml:
        return None
    robot_path = Path(robot_xml)
    for name in ("scene_rl.xml", "scene.xml"):
        candidate = robot_path.with_name(name)
        if candidate.is_file():
            return str(candidate)
    return None


FOOT_BODY_NAMES = ("leg_l6_link", "leg_r6_link")
# S17 sole is ~4 cm below ankle (collision box at local z=-0.035, half 0.005).
SOLE_BELOW_ANKLE = 0.040
SOLE_CLEARANCE = 0.001


def _geom_world_zmin(model, data, geom_id):
    """Lowest world-z of a primitive geom (meshes are skipped by the caller)."""
    pos = data.geom_xpos[geom_id]
    size = model.geom_size[geom_id]
    mat = data.geom_xmat[geom_id].reshape(3, 3)
    gtype = int(model.geom_type[geom_id])
    if gtype == mj.mjtGeom.mjGEOM_BOX:
        return float(pos[2] - (np.abs(mat) @ size)[2])
    if gtype == mj.mjtGeom.mjGEOM_CYLINDER:
        axis_z = abs(float(mat[2, 2]))
        radius = float(size[0])
        half_h = float(size[1])
        return float(pos[2] - (axis_z * half_h + np.sqrt(max(0.0, 1.0 - axis_z * axis_z)) * radius))
    if gtype == mj.mjtGeom.mjGEOM_SPHERE:
        return float(pos[2] - size[0])
    return float(pos[2])


def foot_sole_z(model, data, foot_geom_ids=None):
    """Lowest sole z from ankle bodies, then collision geoms if needed."""
    zs = []
    for name in FOOT_BODY_NAMES:
        bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            zs.append(float(data.xpos[bid, 2]) - SOLE_BELOW_ANKLE)
    if zs:
        return min(zs)
    if not foot_geom_ids:
        return None
    zmin = None
    for geom_id in foot_geom_ids:
        z = _geom_world_zmin(model, data, geom_id)
        zmin = z if zmin is None else min(zmin, z)
    return zmin


def collect_foot_collision_geom_ids(model):
    ids = []
    for i in range(model.ngeom):
        body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[i])
        if body not in FOOT_BODY_NAMES:
            continue
        if int(model.geom_type[i]) == mj.mjtGeom.mjGEOM_MESH:
            continue
        ids.append(i)
    return tuple(ids)


def display_height_offset(sole_z, clearance=SOLE_CLEARANCE):
    """Lift amount that puts the lowest sole on the z=0 visual floor."""
    if sole_z is None:
        return 0.0
    return -float(sole_z) + float(clearance)


def apply_viewer_pose(model, data, qpos, foot_geom_ids=None):
    """Write IK qpos, then lift so the lowest sole sits on the z=0 scene floor.

    Same visual contract as leju-mujoco-sim ``home``: checkerboard at z=0,
    robot standing on it. Returns ``(dz, sole_z)`` after the lift.
    """
    n = min(len(qpos), model.nq)
    data.qpos[:n] = np.asarray(qpos, dtype=np.float64)[:n]
    mj.mj_forward(model, data)
    dz = display_height_offset(foot_sole_z(model, data, foot_geom_ids))
    if model.nq >= 3:
        data.qpos[2] = float(data.qpos[2]) + dz
        mj.mj_forward(model, data)
    return dz, foot_sole_z(model, data, foot_geom_ids)


def reset_viewer_to_home(model, data, foot_geom_ids=None):
    """Match sim startup: home keyframe, then snap soles onto z=0."""
    key_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        mj.mj_resetDataKeyframe(model, data, key_id)
        qpos = data.qpos.copy()
    else:
        qpos = np.asarray(model.qpos0, dtype=np.float64).copy()
    return apply_viewer_pose(model, data, qpos, foot_geom_ids)


class RobotMotionViewer:
    def __init__(self, model: mj.MjModel, cam_distance: float = 3.0,
                 scene_xml=None):
        import mujoco.viewer
        vis_model = model
        if scene_xml:
            vis_model = mj.MjModel.from_xml_path(str(scene_xml))
        self.model = vis_model
        self.data = mj.MjData(self.model)
        self._foot_geom_ids = collect_foot_collision_geom_ids(self.model)
        # Paint a standing pose before the window opens. qpos0 has the waist
        # on the z=0 floor; sim uses the home keyframe instead.
        reset_viewer_to_home(self.model, self.data, self._foot_geom_ids)
        self._cam_inited = False
        self.handle = mujoco.viewer.launch_passive(self.model, self.data)
        # Match scene_rl.xml <visual><global azimuth elevation>
        self.handle.cam.distance = cam_distance
        self.handle.cam.lookat[:] = [
            float(self.data.qpos[0]),
            float(self.data.qpos[1]),
            1.0,
        ]
        self.handle.cam.azimuth = 160
        self.handle.cam.elevation = -20

    def step(self, qpos: np.ndarray, human_motion_data=None):
        dz, _sole_z = apply_viewer_pose(
            self.model, self.data, qpos, self._foot_geom_ids)
        if not self._cam_inited and self.model.nq >= 2:
            self.handle.cam.lookat[:] = [
                float(self.data.qpos[0]),
                float(self.data.qpos[1]),
                1.0,
            ]
            self._cam_inited = True
        draw_human_targets(
            self.handle.user_scn,
            human_motion_data,
            pos_offset=np.array([0.0, 0.0, dz], dtype=np.float64),
        )
        self.handle.sync()

    def close(self):
        try:
            self.handle.close()
        except Exception:
            pass
