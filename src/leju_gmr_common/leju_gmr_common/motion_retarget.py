"""Device-independent General Motion Retargeting (GMR).

ROS-free port of kuavo_pico_gmr's ``motion_retarget.py`` (GeneralMotionRetargeting)
and ``GMRmodule_bvh.py`` (VMR_bvh). Algorithms are kept verbatim from the
hardware-proven Kuavo pipeline; only imports/registries changed.
"""

import json
import os
import pathlib
from typing import Dict, List, Optional, Sequence

import leju_gmr_common.gl_backend  # noqa: F401  sets MUJOCO_GL before mujoco
import mink
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R

from leju_gmr_common.params import ROBOT_XML_DICT


# ---------------------------------------------------------------------------
# Pure-numpy quaternion helpers (wxyz convention) - avoid per-frame scipy cost
# ---------------------------------------------------------------------------
def _qmul_wxyz(q1, q2):
    """Hamilton product of two quaternions in (w,x,y,z) order. Pure numpy."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def _quat_wxyz_to_mat(q):
    """Convert quaternion (w,x,y,z) -> 3x3 rotation matrix. Pure numpy."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    tx, ty, tz = 2.0 * x, 2.0 * y, 2.0 * z
    twx, twy, twz = tx * w, ty * w, tz * w
    txx, txy, txz = tx * x, ty * x, tz * x
    tyy, tyz, tzz = ty * y, ty * z, tz * z
    return np.array([
        [1.0 - (tyy + tzz), txy - twz, txz + twy],
        [txy + twz, 1.0 - (txx + tzz), tyz - twx],
        [txz - twy, tyz + twx, 1.0 - (txx + tyy)],
    ], dtype=np.float64)


def _mat_to_quat_wxyz(M):
    """Convert 3x3 rotation matrix -> quaternion (w,x,y,z). Pure numpy.
    Uses Shepperd's method for numerical stability."""
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
        w = (M[2, 1] - M[1, 2]) / s
        x = 0.25 * s
        y = (M[0, 1] + M[1, 0]) / s
        z = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
        w = (M[0, 2] - M[2, 0]) / s
        x = (M[0, 1] + M[1, 0]) / s
        y = 0.25 * s
        z = (M[1, 2] + M[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
        w = (M[1, 0] - M[0, 1]) / s
        x = (M[0, 2] + M[2, 0]) / s
        y = (M[1, 2] + M[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    if q[0] < 0.0:
        q = -q
    q /= np.linalg.norm(q)
    return q


def _qnorm_wxyz(q):
    """Normalize quaternion (w,x,y,z), enforce w>=0."""
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q = q / n
    if q[0] < 0.0:
        q = -q
    return q


def _avg_existing(vals: Sequence[Optional[float]]) -> Optional[float]:
    xs = [float(v) for v in vals if v is not None]
    return (sum(xs) / len(xs)) if xs else None


def get_body_z(d: "mj.MjData", body_name: str) -> Optional[float]:
    try:
        return float(d.body(body_name).xpos[2])
    except Exception:
        return None


def get_pose_z(pose: Optional[Dict[str, tuple]], key: str) -> Optional[float]:
    if not pose or key not in pose:
        return None
    p = pose[key][0]  # (pos, quat)
    if p is None:
        return None
    return float(p[2])


# default upper-body-only calibration joints
upper_body_cali_part = [
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
]

position_parent = {
    "LeftForeArm": "Hips",
    "LeftHand": "Hips",
    "RightForeArm": "Hips",
    "RightHand": "Hips",
}


class MotionRetargeter:
    """Realtime streaming motion retargeter.

    This class keeps the original online IK/retargeting interface from
    ``motion_retarget.py`` while using the robot-hierarchy calibration and
    position reconstruction logic from the updated VMR_bvh implementation.

    Position semantics
    ------------------
    * The human root stores one world-frame source-to-robot translation.
    * Bodies listed in ``position_hierarchy`` are reconstructed parent-first
      using robot-sized link vectors expressed in the controlling robot
      parent's frame.
    * Bodies outside the hierarchy keep the legacy per-body offset fallback.
    * Orientations remain independently calibrated per mapped body.
    """

    def __init__(
        self,
        src_human: str,
        tgt_robot: str,
        actual_human_height: float = None,
        solver: str = "daqp",
        damping: float = 5e-1,
        verbose: bool = False,
        use_velocity_limit: bool = False,
        cali_pose: Dict[str, tuple] = None,
        cali_option: str = "whole_body",
        cali_part: List[str] = None,
        contact_sequence: Dict[str, np.ndarray] = None,
        save_ik_config: Optional[str] = None,
        ik_config_path: Optional[str] = None,
    ) -> None:
        # Kept for compatibility with the original VMR_bvh interface.
        self.contact_sequence = contact_sequence
        self.previous_human_data = None

        # ------------------------------------------------------------------
        # Robot model
        # ------------------------------------------------------------------
        self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
        if verbose:
            print("Use robot model: ", self.xml_file)
        self.model = mj.MjModel.from_xml_path(self.xml_file)

        if verbose:
            print("[GMR] Robot Degrees of Freedom (DoF) names and their order:")
        self.robot_dof_names = {}
        for i in range(self.model.nv):
            dof_name = mj.mj_id2name(
                self.model,
                mj.mjtObj.mjOBJ_JOINT,
                self.model.dof_jntid[i],
            )
            self.robot_dof_names[dof_name] = i
            if verbose:
                print(f"DoF {i}: {dof_name}")

        if verbose:
            print("[GMR] Robot Body names and their IDs:")
        self.robot_body_names = {}
        for i in range(self.model.nbody):
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            if verbose:
                print(f"Body ID {i}: {body_name}")

        if verbose:
            print("[GMR] Robot Motor (Actuator) names and their IDs:")
        self.robot_motor_names = {}
        for i in range(self.model.nu):
            motor_name = mj.mj_id2name(
                self.model, mj.mjtObj.mjOBJ_ACTUATOR, i
            )
            self.robot_motor_names[motor_name] = i
            if verbose:
                print(f"Motor ID {i}: {motor_name}")

        # ------------------------------------------------------------------
        # IK configuration
        # ------------------------------------------------------------------
        if ik_config_path is None:
            raise ValueError("The frontend must provide its IK configuration path")
        self.ik_config_path = str(ik_config_path)
        with open(self.ik_config_path) as f:
            ik_config = json.load(f)
        self.ik_config = ik_config

        if verbose:
            print("Use IK config: ", self.ik_config_path)

        if actual_human_height is not None:
            ratio = (
                actual_human_height / ik_config["human_height_assumption"]
            )
        else:
            ratio = 1.0

        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = (
                ik_config["human_scale_table"][key] * ratio
            )

        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config.get("ik_match_table2", {})
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config.get(
            "use_ik_match_table2", False
        )
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        # New hierarchy-aware offset semantics from the updated VMR_bvh.
        self.offset_convention = ik_config.get(
            "offset_convention", "legacy_inverse"
        )
        self.position_hierarchy = ik_config.get("position_hierarchy", {})
        stored_root_offset = ik_config.get("root_position_offset")
        self.root_position_offset = (
            None
            if stored_root_offset is None
            else np.asarray(stored_root_offset, dtype=np.float64)
        )
        self.position_link_vectors = {}

        # Validate robot references before building hierarchy geometry.
        self._validate_ik_tables()
        self.position_hierarchy_order = self._validate_position_hierarchy()
        self._cache_robot_position_geometry()

        self.max_iter = 10
        self.solver = solver
        self.damping = damping

        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}

        self.task_errors1 = {}
        self.task_errors2 = {}

        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            velocity_limits = {
                k: 3 * np.pi for k in self.robot_motor_names.keys()
            }
            self.ik_limits.append(
                mink.VelocityLimit(self.model, velocity_limits)
            )

        # Convert offsets from the JSON storage convention into the runtime
        # convention before optional calibration.  Autocalibration then
        # overwrites the corresponding runtime values.
        self._process_table(
            self.ik_match_table1,
            self.pos_offsets1,
            self.rot_offsets1,
            "ik_match_table1",
        )
        self._process_table(
            self.ik_match_table2,
            self.pos_offsets2,
            self.rot_offsets2,
            "ik_match_table2",
        )

        if cali_pose is not None:
            self.autocalibrate_offsets_from_default_pose(
                cali_pose,
                align_feet=True,
                cali_option=cali_option,
                cali_part=cali_part,
                verbose=verbose,
            )

        # Build the fast per-frame numpy cache only after all offsets have
        # reached their final runtime values.
        self._build_offset_cache()
        self.setup_retarget_configuration()

        if save_ik_config is not None:
            self.save_ik_config(save_ik_config)

        self.ground_offset = 0.0
        self.min_ground_clearance = 0.01
        self.flag = 0

    # ------------------------------------------------------------------
    # Validation and robot position geometry
    # ------------------------------------------------------------------
    def _validate_ik_tables(self):
        missing = []
        for table_name in ("ik_match_table1", "ik_match_table2"):
            table = getattr(self, table_name)
            for frame_name in table.keys():
                if frame_name not in self.robot_body_names:
                    missing.append(f"{table_name}: body '{frame_name}'")
        if missing:
            raise ValueError(
                "IK config references bodies missing from robot model "
                f"{self.xml_file}:\n  " + "\n  ".join(missing)
            )

    def _validate_position_hierarchy(self):
        """Validate and topologically order the optional robot target skeleton."""
        if not self.position_hierarchy:
            return []

        mapped_robot_bodies = {}
        for table in (self.ik_match_table1, self.ik_match_table2):
            for robot_body, entry in table.items():
                if isinstance(entry, (list, tuple)) and entry:
                    mapped_robot_bodies[entry[0]] = robot_body

        pending = dict(self.position_hierarchy)
        resolved = {self.human_root_name}
        ordered = []

        while pending:
            progressed = False
            for human_body, spec in list(pending.items()):
                required = {"human_parent", "robot_parent", "robot_body"}
                if not isinstance(spec, dict) or not required.issubset(spec):
                    raise ValueError(
                        f"Invalid position_hierarchy entry for "
                        f"{human_body}: {spec}"
                    )

                if spec["human_parent"] not in resolved:
                    continue

                for robot_body in (spec["robot_parent"], spec["robot_body"]):
                    if robot_body not in self.robot_body_names:
                        raise ValueError(
                            f"Unknown robot body {robot_body!r} in "
                            "position hierarchy"
                        )

                mapped_robot_body = mapped_robot_bodies.get(human_body)
                if mapped_robot_body != spec["robot_body"]:
                    raise ValueError(
                        f"Position hierarchy maps {human_body} to "
                        f"{spec['robot_body']}, but the IK table maps it to "
                        f"{mapped_robot_body}"
                    )

                ordered.append(human_body)
                resolved.add(human_body)
                del pending[human_body]
                progressed = True

            if not progressed:
                unresolved = ", ".join(sorted(pending))
                raise ValueError(
                    "Position hierarchy contains a cycle or missing human "
                    f"parent: {unresolved}"
                )

        return ordered

    def _cache_robot_position_geometry(self, data=None):
        """Cache target origins relative to their position-controlling parents."""
        if not self.position_hierarchy_order:
            return

        if data is None:
            data = mj.MjData(self.model)
            data.qpos[:] = self.model.qpos0
            mj.mj_forward(self.model, data)

        self.position_link_vectors.clear()
        for human_body in self.position_hierarchy_order:
            spec = self.position_hierarchy[human_body]
            parent = data.body(spec["robot_parent"])
            body = data.body(spec["robot_body"])
            r_parent = parent.xmat.reshape(3, 3).copy()
            self.position_link_vectors[human_body] = (
                r_parent.T @ (body.xpos.copy() - parent.xpos.copy())
            )

    # ------------------------------------------------------------------
    # IK tasks
    # ------------------------------------------------------------------
    def setup_retarget_configuration(self):
        self.configuration = mink.Configuration(self.model)

        self.tasks1 = []
        self.tasks2 = []

        for frame_name, entry in self.ik_match_table1.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task1[body_name] = task
                self.tasks1.append(task)
                self.task_errors1[task] = []

        for frame_name, entry in self.ik_match_table2.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task2[body_name] = task
                self.tasks2.append(task)
                self.task_errors2[task] = []

    # ------------------------------------------------------------------
    # Offset processing and persistence
    # ------------------------------------------------------------------
    def _process_table(
        self,
        table: dict,
        pos_offsets: dict,
        rot_offsets: dict,
        tag: str,
        verbose: bool = False,
    ):
        """Convert stored IK offsets into the runtime direct convention."""
        pos_offsets.clear()
        rot_offsets.clear()

        for frame_name, entry in table.items():
            if not (isinstance(entry, list) and len(entry) >= 5):
                continue

            human_name = entry[0]
            pos = np.asarray(entry[3], dtype=np.float64)
            quat_wxyz = np.asarray(entry[4], dtype=np.float64)

            if pos.shape != (3,) or quat_wxyz.shape != (4,):
                continue

            if self.offset_convention == "robot_hierarchy_v1":
                # Auto-calibrated offsets are already stored in runtime form.
                pos_conv = pos.copy()
                quat_conv = quat_wxyz.copy()
            else:
                # Legacy JSON stores the inverse transform.
                pos_conv = -pos
                quat_conv = quat_wxyz.copy()
                quat_conv[1:] *= -1.0

            norm = np.linalg.norm(quat_conv)
            if norm < 1e-12:
                continue
            quat_conv /= norm

            # Keep the in-memory table synchronized with runtime values.
            entry[3] = pos_conv.tolist()
            entry[4] = quat_conv.tolist()

            pos_offsets[human_name] = pos_conv
            # wxyz -> scipy xyzw, compatible with older SciPy releases.
            rot_offsets[human_name] = R.from_quat(
                quat_conv[[1, 2, 3, 0]]
            )

        if verbose:
            print(
                f"[InitOffsets] processed {tag}: "
                f"{len(pos_offsets)} bodies"
            )

    def save_ik_config(self, path: str) -> None:
        """Persist the in-memory, runtime-form IK configuration as JSON."""
        if not hasattr(self, "ik_config"):
            raise RuntimeError("no ik_config available to save")

        # At this point the table entries are in direct runtime form.  Mark
        # that explicitly so a future load does not negate/invert them again.
        self.offset_convention = "robot_hierarchy_v1"
        self.ik_config["offset_convention"] = self.offset_convention
        self.ik_config["ik_match_table1"] = self.ik_match_table1
        self.ik_config["ik_match_table2"] = self.ik_match_table2

        if self.root_position_offset is not None:
            self.ik_config["root_position_offset"] = [
                float(x) for x in self.root_position_offset
            ]
        if self.position_hierarchy_order:
            self.ik_config["position_link_vectors"] = {
                body_name: [
                    float(x)
                    for x in self.position_link_vectors[body_name]
                ]
                for body_name in self.position_hierarchy_order
            }

        path = os.fspath(path)
        save_dir = os.path.dirname(path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        with open(path, "w") as f:
            json.dump(self.ik_config, f, indent=4)

        print(f"[MotionRetargeter] IK configuration dumped to {path}")

    # ------------------------------------------------------------------
    # Automatic calibration
    # ------------------------------------------------------------------
    def autocalibrate_offsets_from_default_pose(
        self,
        cali_pose: Dict[str, tuple],
        align_feet: bool = True,
        cali_option: Optional[str] = None,
        cali_part: List[str] = None,
        verbose: bool = True,
        save_json: bool = False,
        save_path: Optional[str] = None,
    ) -> None:
        """Calibrate orientation offsets and hierarchy-aware position geometry.

        The position logic follows the updated VMR_bvh implementation:

        * Hips/root stores one world-frame source-to-robot translation.
        * Hierarchy bodies store the robot parent-to-child vector expressed in
          the controlling robot parent's frame.
        * Targets outside ``position_hierarchy`` keep a compatibility fallback
          ``R_robot.T @ (p_source - p_robot)``.

        ``cali_option='upper_body'`` and ``cali_part`` are retained for
        compatibility with the previous realtime API.  They limit which
        orientation offsets are recalibrated.  Hierarchy position geometry is
        still rebuilt for the whole robot because it is robot-model geometry,
        not source-body calibration.
        """
        d = mj.MjData(self.model)
        d.qpos[:] = self.model.qpos0
        mj.mj_forward(self.model, d)

        # Work on copies: callers may reuse the captured calibration pose.
        cali_pose = {
            body_name: [
                np.asarray(pos, dtype=np.float64).copy(),
                np.asarray(quat, dtype=np.float64).copy(),
            ]
            for body_name, (pos, quat) in cali_pose.items()
        }
        cali_pose = self.scale_human_data(
            cali_pose, self.human_root_name, self.human_scale_table
        )

        if align_feet:
            hz = _avg_existing(
                [
                    get_pose_z(cali_pose, "LeftToeBase"),
                    get_pose_z(cali_pose, "RightToeBase"),
                ]
            )
            rz = _avg_existing(
                [
                    get_body_z(d, "leg_l6_link"),
                    get_body_z(d, "leg_r6_link"),
                ]
            )
            if hz is not None and rz is not None:
                dz = float(hz - rz)
                d.qpos[2] += dz
                mj.mj_forward(self.model, d)
                if verbose:
                    print(
                        f"[AutoCalib] base z += {dz:+.4f} "
                        f"(human_toe_z={hz:.4f}, robot_foot_z={rz:.4f})"
                    )
            elif verbose:
                print(
                    "[AutoCalib] skip feet alignment "
                    "(missing toe/foot bodies)."
                )

        # Cache geometry from exactly the same neutral/aligned robot state.
        self._cache_robot_position_geometry(d)

        if self.human_root_name not in cali_pose:
            raise ValueError(
                "Calibration pose is missing root body "
                f"{self.human_root_name!r}"
            )

        root_source_pos = np.asarray(
            cali_pose[self.human_root_name][0], dtype=np.float64
        )
        root_robot_pos = d.body(self.robot_root_name).xpos.copy()
        self.root_position_offset = root_source_pos - root_robot_pos

        self.ik_config["root_position_offset"] = [
            float(x) for x in self.root_position_offset
        ]
        self.ik_config["position_link_vectors"] = {
            body_name: [
                float(x) for x in self.position_link_vectors[body_name]
            ]
            for body_name in self.position_hierarchy_order
        }

        if cali_option == "upper_body" and cali_part is None:
            cali_part = upper_body_cali_part
        orientation_selection = (
            set(cali_part) if cali_part is not None else None
        )

        def _update_table(
            table: dict,
            pos_offsets: dict,
            rot_offsets: dict,
            tag: str,
        ):
            updated_count = 0

            for frame_name, entry in table.items():
                if not isinstance(entry, (list, tuple)) or len(entry) < 5:
                    continue

                human_name = entry[0]

                # Position values for hierarchy bodies are robot geometry and
                # can be refreshed even if the source pose omits that body.
                position_value = None
                if human_name == self.human_root_name:
                    position_value = self.root_position_offset
                elif human_name in self.position_link_vectors:
                    position_value = self.position_link_vectors[human_name]
                elif human_name in cali_pose:
                    try:
                        pr = d.body(frame_name).xpos.copy()
                        rr = d.body(frame_name).xmat.reshape(3, 3).copy()
                        ps = np.asarray(
                            cali_pose[human_name][0], dtype=np.float64
                        )
                        position_value = rr.T @ (ps - pr)
                    except Exception:
                        position_value = None

                if position_value is not None:
                    entry[3] = [
                        float(x) for x in position_value.tolist()
                    ]
                    pos_offsets[human_name] = np.asarray(
                        position_value, dtype=np.float64
                    )

                # Orientation stays independently calibrated per body.
                should_recalibrate_orientation = (
                    human_name in cali_pose
                    and (
                        orientation_selection is None
                        or human_name in orientation_selection
                    )
                )

                if should_recalibrate_orientation:
                    try:
                        rr = d.body(frame_name).xmat.reshape(3, 3).copy()
                    except Exception:
                        continue

                    qs_wxyz = np.asarray(
                        cali_pose[human_name][1], dtype=np.float64
                    )
                    rs = R.from_quat(
                        qs_wxyz[[1, 2, 3, 0]]
                    ).as_matrix()
                    r_s2r = rr.T @ rs
                    q_xyzw = R.from_matrix(r_s2r).as_quat()
                    q_wxyz = q_xyzw[[3, 0, 1, 2]]

                    entry[4] = [
                        float(x) for x in q_wxyz.tolist()
                    ]
                    rot_offsets[human_name] = R.from_quat(q_xyzw)
                    updated_count += 1
                elif human_name in rot_offsets:
                    # _process_table() already converted this body to runtime
                    # convention.  Keep the JSON entry synchronized before we
                    # switch the config-wide convention below.
                    q_xyzw = rot_offsets[human_name].as_quat()
                    q_wxyz = q_xyzw[[3, 0, 1, 2]]
                    entry[4] = [
                        float(x) for x in q_wxyz.tolist()
                    ]

            if verbose:
                print(
                    f"[AutoCalib] updated {tag}: "
                    f"{updated_count} orientation bodies; "
                    f"{len(pos_offsets)} position offsets available"
                )

        _update_table(
            self.ik_match_table1,
            self.pos_offsets1,
            self.rot_offsets1,
            "ik_match_table1",
        )
        _update_table(
            self.ik_match_table2,
            self.pos_offsets2,
            self.rot_offsets2,
            "ik_match_table2",
        )

        # Calibration writes direct runtime-form values into the tables.
        self.offset_convention = "robot_hierarchy_v1"
        self.ik_config["offset_convention"] = self.offset_convention
        self.ik_config["ik_match_table1"] = self.ik_match_table1
        self.ik_config["ik_match_table2"] = self.ik_match_table2

        # Refresh fast caches in case this method is called again at runtime
        # after __init__ has already completed.
        self._build_offset_cache()

        if save_json:
            if save_path is None:
                base = pathlib.Path(self.ik_config_path)
                save_path = str(
                    base.with_name(base.stem + "_updated.json")
                )
            self.save_ik_config(save_path)

    def autocalibrate_offsets_from_bvh_default_pose(
        self,
        cali_pose: Dict[str, tuple],
        align_feet: bool = True,
        verbose: bool = True,
    ) -> None:
        """Compatibility alias matching the updated VMR_bvh API."""
        self.autocalibrate_offsets_from_default_pose(
            cali_pose,
            align_feet=align_feet,
            cali_option="whole_body",
            cali_part=None,
            verbose=verbose,
        )

    # ------------------------------------------------------------------
    # Fast runtime caches and source->robot target conversion
    # ------------------------------------------------------------------
    def _build_offset_cache(self):
        """Pre-compute rotation matrices and offsets for the per-frame path."""
        self._rot_offsets1_inv_mat = {}
        self._pos_offsets1_arr = {}
        for body_name, rot_obj in self.rot_offsets1.items():
            self._rot_offsets1_inv_mat[body_name] = (
                rot_obj.inv().as_matrix().astype(np.float64)
            )
            if body_name in self.pos_offsets1:
                self._pos_offsets1_arr[body_name] = np.asarray(
                    self.pos_offsets1[body_name], dtype=np.float64
                )

        self._rot_offsets2_inv_mat = {}
        self._pos_offsets2_arr = {}
        for body_name, rot_obj in self.rot_offsets2.items():
            self._rot_offsets2_inv_mat[body_name] = (
                rot_obj.inv().as_matrix().astype(np.float64)
            )
            if body_name in self.pos_offsets2:
                self._pos_offsets2_arr[body_name] = np.asarray(
                    self.pos_offsets2[body_name], dtype=np.float64
                )

    def offset_human_data_robotframe(
        self, human_data, pos_offsets, rot_offsets
    ):
        """Convert mocap global poses into robot-frame global target poses.

        Orientations are calibrated independently for every body.  Positions
        are then reconstructed parent-first from the root translation and
        cached robot link geometry.  This prevents a body's own rotation from
        moving its joint origin.
        """
        if rot_offsets is self.rot_offsets1:
            inv_mat_cache = self._rot_offsets1_inv_mat
            pos_cache = self._pos_offsets1_arr
        elif rot_offsets is self.rot_offsets2:
            inv_mat_cache = self._rot_offsets2_inv_mat
            pos_cache = self._pos_offsets2_arr
        else:
            # Defensive fallback for external callers passing compatible dicts.
            inv_mat_cache = {
                body_name: rot_obj.inv().as_matrix().astype(np.float64)
                for body_name, rot_obj in rot_offsets.items()
            }
            pos_cache = {
                body_name: np.asarray(offset, dtype=np.float64)
                for body_name, offset in pos_offsets.items()
            }

        out = {
            body_name: [
                np.asarray(pos_s, dtype=np.float64).copy(),
                np.asarray(quat_s, dtype=np.float64).copy(),
            ]
            for body_name, (pos_s, quat_s) in human_data.items()
        }

        target_rotation_mats = {}

        # Pass 1: independent orientation calibration for every mapped body.
        for body_name, (_, quat_s) in human_data.items():
            if body_name not in inv_mat_cache:
                continue

            quat_s = _qnorm_wxyz(quat_s)
            r_s_mat = _quat_wxyz_to_mat(quat_s)
            r_r_mat = r_s_mat @ inv_mat_cache[body_name]
            target_rotation_mats[body_name] = r_r_mat
            out[body_name][1] = _mat_to_quat_wxyz(r_r_mat)

        hierarchy_active = (
            self.root_position_offset is not None
            and self.human_root_name in out
            and self.human_root_name in target_rotation_mats
            and all(
                body_name in self.position_link_vectors
                for body_name in self.position_hierarchy_order
            )
        )

        hierarchy_bodies = set()

        if hierarchy_active:
            # Root translation is a world-space quantity; never rotate it.
            root_source_pos = np.asarray(
                human_data[self.human_root_name][0], dtype=np.float64
            )
            out[self.human_root_name][0] = (
                root_source_pos - self.root_position_offset
            )
            hierarchy_bodies.add(self.human_root_name)

            # Pass 2: synthesize each joint origin from its controlling parent.
            for body_name in self.position_hierarchy_order:
                spec = self.position_hierarchy[body_name]
                parent_name = spec["human_parent"]

                if (
                    body_name not in out
                    or parent_name not in hierarchy_bodies
                    or parent_name not in target_rotation_mats
                ):
                    continue

                out[body_name][0] = (
                    out[parent_name][0]
                    + target_rotation_mats[parent_name]
                    @ self.position_link_vectors[body_name]
                )
                hierarchy_bodies.add(body_name)

        # Legacy fallback for targets outside the configured hierarchy, or for
        # all targets when hierarchy reconstruction is unavailable.
        for body_name, (pos_s, _) in human_data.items():
            if body_name in hierarchy_bodies:
                continue

            if hierarchy_active and (
                body_name == self.human_root_name
                or body_name in self.position_hierarchy
            ):
                # Do not reinterpret a hierarchy link vector as an absolute
                # source-to-robot offset when a hierarchy parent is missing.
                continue

            if (
                body_name not in pos_cache
                or body_name not in target_rotation_mats
            ):
                continue

            out[body_name][0] = (
                np.asarray(pos_s, dtype=np.float64)
                - target_rotation_mats[body_name] @ pos_cache[body_name]
            )

        return out

    # ------------------------------------------------------------------
    # Per-frame target update and IK solve
    # ------------------------------------------------------------------
    def update_targets(self, human_data, offset_to_ground=False):
        human_data = self.to_numpy(human_data)
        human_data = self.scale_human_data(
            human_data,
            self.human_root_name,
            self.human_scale_table,
        )
        human_data = self.offset_human_data_robotframe(
            human_data,
            self.pos_offsets1,
            self.rot_offsets1,
        )

        # Expose only actual robot targets to comparison/debug viewers.
        # Source-only joints (for example clavicles without robot targets) are
        # intentionally omitted.
        visualized_bodies = dict.fromkeys(
            [*self.human_body_to_task1, *self.human_body_to_task2]
        )
        self.scaled_human_data = {
            body_name: human_data[body_name]
            for body_name in visualized_bodies
            if body_name in human_data
        }

        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(
                    mink.SE3.from_rotation_and_translation(
                        mink.SO3(rot), pos
                    )
                )

        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(
                    mink.SE3.from_rotation_and_translation(
                        mink.SO3(rot), pos
                    )
                )

    def retarget(self, human_data, offset_to_ground=False):
        self.update_targets(human_data, offset_to_ground)

        if self.use_ik_match_table1:
            if self.flag == 0:
                # First frame: iterate until converged.
                curr_error = self.error1()
                dt = self.configuration.model.opt.timestep
                vel1 = mink.solve_ik(
                    self.configuration,
                    self.tasks1,
                    dt,
                    self.solver,
                    self.damping,
                    self.ik_limits,
                )
                self.configuration.integrate_inplace(vel1, dt)
                next_error = self.error1()
                num_iter = 0

                while (
                    curr_error - next_error > 0.001
                    and num_iter < self.max_iter
                ):
                    curr_error = next_error
                    dt = self.configuration.model.opt.timestep
                    vel1 = mink.solve_ik(
                        self.configuration,
                        self.tasks1,
                        dt,
                        self.solver,
                        self.damping,
                        self.ik_limits,
                    )
                    self.configuration.integrate_inplace(vel1, dt)
                    next_error = self.error1()
                    num_iter += 1

                self.flag = 1
            else:
                dt = self.configuration.model.opt.timestep
                vel1 = mink.solve_ik(
                    self.configuration,
                    self.tasks1,
                    dt,
                    self.solver,
                    self.damping,
                    self.ik_limits,
                )
                self.configuration.integrate_inplace(vel1, dt)

        if self.use_ik_match_table2:
            curr_error = self.error2()
            dt = self.configuration.model.opt.timestep
            vel2 = mink.solve_ik(
                self.configuration,
                self.tasks2,
                dt,
                self.solver,
                self.damping,
                self.ik_limits,
            )
            self.configuration.integrate_inplace(vel2, dt)
            next_error = self.error2()
            num_iter = 0

            while (
                curr_error - next_error > 0.001
                and num_iter < self.max_iter
            ):
                curr_error = next_error
                dt = self.configuration.model.opt.timestep
                vel2 = mink.solve_ik(
                    self.configuration,
                    self.tasks2,
                    dt,
                    self.solver,
                    self.damping,
                    self.ik_limits,
                )
                self.configuration.integrate_inplace(vel2, dt)
                next_error = self.error2()
                num_iter += 1

        return self.configuration.data.qpos.copy()

    def error1(self):
        return np.linalg.norm(
            np.concatenate(
                [
                    task.compute_error(self.configuration)
                    for task in self.tasks1
                ]
            )
        )

    def error2(self):
        return np.linalg.norm(
            np.concatenate(
                [
                    task.compute_error(self.configuration)
                    for task in self.tasks2
                ]
            )
        )

    def to_numpy(self, human_data):
        for body_name in human_data.keys():
            human_data[body_name] = [
                np.asarray(human_data[body_name][0]),
                np.asarray(human_data[body_name][1]),
            ]
        return human_data

    def scale_human_data(
        self,
        human_data,
        human_root_name,
        human_scale_table,
    ):
        human_data_local = {}
        root_pos, root_quat = human_data[human_root_name]

        scaled_root_pos = human_scale_table[human_root_name] * root_pos

        for body_name in human_data.keys():
            if body_name not in human_scale_table:
                continue
            if body_name == human_root_name:
                continue

            human_data_local[body_name] = (
                (human_data[body_name][0] - root_pos)
                * human_scale_table[body_name]
            )

        human_data_global = {
            human_root_name: (scaled_root_pos, root_quat)
        }
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (
                human_data_local[body_name] + scaled_root_pos,
                human_data[body_name][1],
            )

        return human_data_global
