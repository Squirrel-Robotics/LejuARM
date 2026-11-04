"""Pico body order and mapping to canonical GMR frames."""
from typing import Dict, List
import numpy as np
from scipy.spatial.transform import Rotation as R
from leju_gmr_common.calibration import FrameDict

pico_body_names: List[str] = [
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
]

PICO_TO_LAFAN: Dict[str, str] = {
    # torso
    "pelvis": "Hips",
    "spine3": "Spine",
    "neck": "Neck",
    "head": "Head",
    # arms
    "left_collar": "LeftShoulder",
    "left_shoulder": "LeftArm",
    "left_elbow": "LeftForeArm",
    "left_wrist": "LeftHand",
    "right_collar": "RightShoulder",
    "right_shoulder": "RightArm",
    "right_elbow": "RightForeArm",
    "right_wrist": "RightHand",
    # legs
    "left_hip": "LeftUpLeg",
    "left_knee": "LeftLeg",
    "left_ankle": "LeftFoot",
    "left_foot": "LeftToeBase",
    "right_hip": "RightUpLeg",
    "right_knee": "RightLeg",
    "right_ankle": "RightFoot",
    "right_foot": "RightToeBase",
}

def map_pico_frame_to_gmr(frame_data: FrameDict) -> FrameDict:
    """Map canonicalized PICO frame into the source skeleton expected by GMR."""
    out: FrameDict = {}
    for pico_name, lafan_name in PICO_TO_LAFAN.items():
        if pico_name not in frame_data:
            raise ValueError(f"Missing expected PICO joint '{pico_name}' in frame data")
        pos, quat_wxyz = frame_data[pico_name]
        out[lafan_name] = [np.asarray(pos, dtype=np.float64), quat_wxyz]
    return out

def world_matrices_to_gmr_frame(world_matrices: np.ndarray) -> FrameDict:
    """Convert PicoReceiver world matrices [N,4,4] to a GMR FrameDict.

    Index order follows BODY_TRACKER_ROLES (same order as pico_body_names
    for the first 24 entries).
    """
    # One batch conversion avoids 24 native/Python transitions per input frame.
    # Keep independent float64 outputs: the receiver reuses its matrix buffer.
    positions = np.asarray(world_matrices[:len(pico_body_names), :3, 3], dtype=np.float64).copy()
    quaternions_xyzw = R.from_matrix(world_matrices[:len(pico_body_names), :3, :3]).as_quat()
    frame_data: FrameDict = {}
    for bone_idx, pico_name in enumerate(pico_body_names):
        q = quaternions_xyzw[bone_idx]
        frame_data[pico_name] = [positions[bone_idx].copy(),
                                np.array([q[3], q[0], q[1], q[2]], dtype=np.float64)]
    return map_pico_frame_to_gmr(frame_data)
