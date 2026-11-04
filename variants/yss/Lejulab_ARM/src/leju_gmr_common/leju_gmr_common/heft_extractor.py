"""Name-based extraction of HEFT-ordered joint positions from MuJoCo qpos.

Never slice qpos by fixed index ranges: build the permutation from joint
names via ``jnt_qposadr`` and validate at startup.
"""

from typing import List, Tuple

import leju_gmr_common.gl_backend  # noqa: F401  sets MUJOCO_GL before mujoco
import mujoco as mj
import numpy as np

from leju_gmr_common.params import HEFT_JOINT_NAMES

HEFT_ROOT_BODY_NAME = "base_link"


class HeftJointExtractor:
    """Maps a full MuJoCo qpos vector to the 21-dim HEFT-order joint vector."""

    def __init__(self, model: mj.MjModel, joint_names: List[str] = None):
        self.joint_names = list(joint_names or HEFT_JOINT_NAMES)
        self._qpos_addr = np.zeros(len(self.joint_names), dtype=np.int64)
        self._dof_addr = np.zeros(len(self.joint_names), dtype=np.int64)

        missing = []
        for i, name in enumerate(self.joint_names):
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                missing.append(name)
                continue
            jtype = model.jnt_type[jid]
            if jtype not in (mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_SLIDE):
                raise ValueError(
                    f"Joint '{name}' has unsupported type {jtype} "
                    "(expected hinge/slide)")
            self._qpos_addr[i] = model.jnt_qposadr[jid]
            self._dof_addr[i] = model.jnt_dofadr[jid]

        if missing:
            raise ValueError(
                "HEFT joints missing from MuJoCo model: " + ", ".join(missing))

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    def extract_qpos(self, qpos: np.ndarray) -> np.ndarray:
        """Extract HEFT-order joint positions from a full qpos vector."""
        qpos = np.asarray(qpos, dtype=np.float64)
        return qpos[self._qpos_addr].copy()

    def extract_qvel(self, qvel: np.ndarray) -> np.ndarray:
        """Extract HEFT-order joint velocities from a full qvel vector."""
        qvel = np.asarray(qvel, dtype=np.float64)
        return qvel[self._dof_addr].copy()


class BodyPoseExtractor:
    """FK a named body pose (world xpos + xquat wxyz) from a full qpos.

    HEFT ``root_*`` defaults to freejoint ``base_link`` (training root), not
    torso. Waist twist must not appear as whole-body yaw in the reference.
    """

    def __init__(self, model: mj.MjModel, body_name: str = HEFT_ROOT_BODY_NAME):
        self.model = model
        self.body_name = body_name
        self._body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
        if self._body_id < 0:
            raise ValueError(f"Body '{body_name}' missing from MuJoCo model")
        self._data = mj.MjData(model)

    def pose_wxyz(self, qpos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return (xpos[3], xquat_wxyz[4]) for the configured body."""
        qpos = np.asarray(qpos, dtype=np.float64)
        if qpos.shape != (self.model.nq,):
            raise ValueError(
                f"qpos length {qpos.shape} != model.nq={self.model.nq}")
        self._data.qpos[:] = qpos
        mj.mj_forward(self.model, self._data)
        pos = self._data.xpos[self._body_id].copy()
        quat = self._data.xquat[self._body_id].copy()  # MuJoCo: wxyz
        return pos, quat
