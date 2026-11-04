"""Canonical frame calibration and heading alignment for motion frontends.

Canonical-frame averaging, heading alignment and yaw continuity math,
originally ported from kuavo_pico_gmr with ROS removed. Device-specific
joint order and coordinate conversion belong to the input adapters.
"""

import logging

import threading

from collections import deque

from typing import Dict, List, Optional, Tuple

import numpy as np

from scipy.spatial.transform import Rotation as R

log = logging.getLogger("leju_gmr_common.calibration")

FrameDict = Dict[str, Tuple[np.ndarray, np.ndarray]]

upper_body_cali_part = [
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "Neck", "Head",
]

def _quat_normalize_wxyz(q: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4,)
    n = np.linalg.norm(q)
    if n < eps:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n

def average_quat_wxyz(quats: np.ndarray) -> np.ndarray:
    quats = np.asarray(quats, dtype=np.float64)
    if quats.ndim != 2 or quats.shape[1] != 4:
        raise ValueError(f"Expected (N,4), got {quats.shape}")

    ref = quats[0].copy()
    aligned = []
    for q in quats:
        qn = _quat_normalize_wxyz(q)
        if np.dot(qn, ref) < 0.0:
            qn = -qn
        aligned.append(qn)

    q_mean = np.mean(np.asarray(aligned), axis=0)
    return _quat_normalize_wxyz(q_mean)

def build_cali_pose_from_frames(frames: List[FrameDict]) -> FrameDict:
    if len(frames) == 0:
        raise ValueError("No frames provided for calibration")

    keys = list(frames[0].keys())
    cali_pose: FrameDict = {}

    for k in keys:
        pos_list = []
        quat_list = []
        for f in frames:
            if k not in f:
                continue
            p, q = f[k]
            pos_list.append(np.asarray(p, dtype=np.float64))
            quat_list.append(np.asarray(q, dtype=np.float64))

        if len(pos_list) == 0:
            continue

        pos_mean = np.mean(np.stack(pos_list, axis=0), axis=0)
        quat_mean = average_quat_wxyz(np.stack(quat_list, axis=0))
        cali_pose[k] = [pos_mean, quat_mean]

    return cali_pose

def _angular_velocity_wxyz(q_prev_wxyz: np.ndarray, q_curr_wxyz: np.ndarray,
                           inv_dt: float) -> np.ndarray:
    """Compute angular velocity from two unit quaternions (wxyz)."""
    qp = np.asarray(q_prev_wxyz, dtype=np.float64)
    qc = np.asarray(q_curr_wxyz, dtype=np.float64)
    qp = qp / max(np.linalg.norm(qp), 1e-12)
    qc = qc / max(np.linalg.norm(qc), 1e-12)

    pw, px, py, pz = qp[0], -qp[1], -qp[2], -qp[3]
    cw, cx, cy, cz = qc[0], qc[1], qc[2], qc[3]
    dw = cw * pw - cx * px - cy * py - cz * pz
    dx = cw * px + cx * pw + cy * pz - cz * py
    dy = cw * py - cx * pz + cy * pw + cz * px
    dz = cw * pz + cx * py - cy * px + cz * pw

    if dw < 0.0:
        dw, dx, dy, dz = -dw, -dx, -dy, -dz

    sin_half = np.sqrt(dx * dx + dy * dy + dz * dz)
    if sin_half < 1e-10:
        return np.zeros(3, dtype=np.float64)

    half_angle = np.arctan2(sin_half, dw)
    angle = 2.0 * half_angle
    axis = np.array([dx, dy, dz], dtype=np.float64) / sin_half
    return axis * angle * inv_dt

def _wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

def _yaw_from_quat_wxyz(q_wxyz: np.ndarray) -> float:
    q_wxyz = _quat_normalize_wxyz(q_wxyz)
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    fwd_xy = R.from_quat(q_xyzw).apply([1.0, 0.0, 0.0])[:2]
    n = np.linalg.norm(fwd_xy)
    if n < 1e-8 or not np.all(np.isfinite(fwd_xy)):
        return 0.0
    return _wrap_to_pi(np.arctan2(fwd_xy[1], fwd_xy[0]))

def _left_apply_yaw_to_quat_wxyz(q_wxyz: np.ndarray, yaw: float) -> np.ndarray:
    q_wxyz = _quat_normalize_wxyz(q_wxyz)
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    q_corr_xyzw = (R.from_euler("z", yaw) * R.from_quat(q_xyzw)).as_quat()
    q_corr_xyzw /= np.clip(np.linalg.norm(q_corr_xyzw), 1e-12, None)
    return np.array(
        [q_corr_xyzw[3], q_corr_xyzw[0], q_corr_xyzw[1], q_corr_xyzw[2]],
        dtype=np.float64,
    )

def estimate_human_height_from_frame(frame_gmr: FrameDict) -> float:
    try:
        head_z = float(frame_gmr["Head"][0][2])
        l_foot_z = float(frame_gmr["LeftToeBase"][0][2])
        r_foot_z = float(frame_gmr["RightToeBase"][0][2])
        height = head_z - 0.5 * (l_foot_z + r_foot_z)
        return float(np.clip(height, 1.2, 2.2))
    except Exception:
        return 1.75

class HeadingAligner:
    """Root heading alignment (ported from the OptiTrack/Pico node)."""

    def __init__(self, init_buf_len: int = 6):
        self._R_delta: Optional[R] = None
        self._p0: Optional[np.ndarray] = None
        self._init_qpos_buf = deque(maxlen=init_buf_len)

    @property
    def initialized(self) -> bool:
        return self._R_delta is not None

    def reset(self):
        self._R_delta = None
        self._p0 = None
        self._init_qpos_buf.clear()

    def align(self, qpos_arr: np.ndarray) -> Optional[np.ndarray]:
        qpos_arr = np.asarray(qpos_arr, dtype=np.float64).copy()

        try:
            if self._R_delta is None:
                self._init_qpos_buf.append(qpos_arr.copy())
                if len(self._init_qpos_buf) < self._init_qpos_buf.maxlen:
                    return None

                pos_xy = []
                fwd_xy_list = []

                for qk in self._init_qpos_buf:
                    pos_xy.append(np.asarray(qk[:2], dtype=np.float64))
                    q_wxyz = np.asarray(qk[3:7], dtype=np.float64)
                    q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
                    q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

                    Rk = R.from_quat(q_xyzw)
                    fwd = Rk.apply([1.0, 0.0, 0.0])[:2]
                    n = np.linalg.norm(fwd)
                    if n > 1e-8 and np.all(np.isfinite(fwd)):
                        fwd_xy_list.append(fwd / n)

                body_samples = fwd_xy_list[1:]
                if len(body_samples) == 0:
                    body_vec = np.array([1.0, 0.0], dtype=np.float64)
                else:
                    seed = np.sum(np.asarray(body_samples), axis=0)
                    seed_n = np.linalg.norm(seed)
                    seed = (np.array([1.0, 0.0], dtype=np.float64)
                            if seed_n < 1e-8 else seed / seed_n)
                    keep = [v for v in body_samples if np.dot(v, seed) > 0.5]
                    body_vec = np.sum(
                        np.asarray(keep if len(keep) >= 3 else body_samples), axis=0)
                    body_n = np.linalg.norm(body_vec)
                    body_vec = seed if body_n < 1e-8 else body_vec / body_n

                d_xy = pos_xy[-1] - pos_xy[0]
                d_norm = np.linalg.norm(d_xy)
                move_thresh = 0.05
                full_traj_thresh = 0.1

                if d_norm > move_thresh and np.all(np.isfinite(d_xy)):
                    traj_vec = d_xy / d_norm
                    alpha = np.clip(
                        (d_norm - move_thresh) / (full_traj_thresh - move_thresh),
                        0.0, 1.0)
                    v = (1.0 - alpha) * body_vec + alpha * traj_vec
                    v_n = np.linalg.norm(v)
                    v = traj_vec if v_n < 1e-8 else v / v_n
                else:
                    v = body_vec

                yaw0 = np.arctan2(v[1], v[0])
                yaw0 = (yaw0 + np.pi) % (2.0 * np.pi) - np.pi

                self._R_delta = R.from_euler("z", -yaw0)
                self._p0 = self._init_qpos_buf[0][:3].copy()
                self._init_qpos_buf.clear()

                log.info("[HeadingAlign] initialized: yaw0=%.6f rad (%.3f deg)",
                         yaw0, np.degrees(yaw0))

            root_loc = qpos_arr[:3]
            q_wxyz = qpos_arr[3:7]
            q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
            q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

            root_loc_aligned = (self._R_delta.apply(root_loc - self._p0) + self._p0)
            root_rot_xyzw_aligned = (self._R_delta * R.from_quat(q_xyzw)).as_quat()
            root_rot_xyzw_aligned /= np.clip(
                np.linalg.norm(root_rot_xyzw_aligned), 1e-12, None)
            root_rot_wxyz_aligned = np.array(
                [
                    root_rot_xyzw_aligned[3],
                    root_rot_xyzw_aligned[0],
                    root_rot_xyzw_aligned[1],
                    root_rot_xyzw_aligned[2],
                ],
                dtype=np.float64,
            )

            qpos_arr[:3] = root_loc_aligned
            qpos_arr[3:7] = root_rot_wxyz_aligned
            return qpos_arr

        except Exception as e:
            log.warning("[HeadingAlign] failed: %s", e)
            return None if self._R_delta is None else qpos_arr

    def apply_to_point(self, pos: np.ndarray) -> np.ndarray:
        """Apply the heading correction to a world point (for FK targets)."""
        if self._R_delta is None or self._p0 is None:
            return pos
        return self._R_delta.apply(pos - self._p0) + self._p0

class YawInvariant:
    """Freeze/resume yaw continuity (RT+Y freeze, RT+X resume)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._correction = 0.0
        self._freeze_yaw: Optional[float] = None
        self._resume_pending = False
        self._latest_published_root_rot: Optional[np.ndarray] = None

    def reset(self):
        with self._lock:
            self._correction = 0.0
            self._freeze_yaw = None
            self._resume_pending = False
            self._latest_published_root_rot = None

    def mark_freeze(self):
        with self._lock:
            latest = (None if self._latest_published_root_rot is None
                      else self._latest_published_root_rot.copy())
        if latest is None:
            log.warning("[YawInvariant] freeze ignored: no published root yet")
            return
        freeze_yaw = _yaw_from_quat_wxyz(latest)
        with self._lock:
            self._freeze_yaw = freeze_yaw
            self._resume_pending = False
        log.info("[YawInvariant] freeze yaw captured: %.6f rad (%.2f deg)",
                 freeze_yaw, np.degrees(freeze_yaw))

    def request_resume(self):
        with self._lock:
            has = self._freeze_yaw is not None
            if has:
                self._resume_pending = True
        if not has:
            log.warning("[YawInvariant] resume ignored: freeze yaw not captured")

    def complete_resume_if_needed(self, root_rot_wxyz: np.ndarray) -> bool:
        """Returns True if a resume completed (caller should reset velocity state)."""
        with self._lock:
            pending = self._resume_pending
            freeze_yaw = self._freeze_yaw
        if not pending or freeze_yaw is None:
            return False

        resume_yaw = _yaw_from_quat_wxyz(root_rot_wxyz)
        yaw_correction = _wrap_to_pi(freeze_yaw - resume_yaw)

        with self._lock:
            self._correction = yaw_correction
            self._resume_pending = False
            self._freeze_yaw = None

        log.info("[YawInvariant] resume yaw=%.6f rad, correction=%.6f rad (%.2f deg)",
                 resume_yaw, yaw_correction, np.degrees(yaw_correction))
        return True

    @property
    def correction(self) -> float:
        with self._lock:
            return self._correction

    def apply(self, root_rot_wxyz: np.ndarray) -> np.ndarray:
        root_rot_wxyz = _quat_normalize_wxyz(root_rot_wxyz)
        with self._lock:
            yaw_correction = self._correction
        corrected = _left_apply_yaw_to_quat_wxyz(root_rot_wxyz, yaw_correction)
        with self._lock:
            self._latest_published_root_rot = corrected.copy()
        return corrected
