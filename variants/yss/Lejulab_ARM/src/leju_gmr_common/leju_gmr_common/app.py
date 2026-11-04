"""Source-independent canonical-frame consumer and S17 DDS HEFT output.

Input adapters provide their receiver, IK config and calibration policy.
The shared layer never imports a device package or interprets raw packets.
"""
import argparse
import logging
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple
import numpy as np
import leju_gmr_common.gl_backend  # noqa: F401  sets MUJOCO_GL before mujoco
from .calibration import FrameDict, HeadingAligner, YawInvariant
from .dds_io import DdsPublishers
from .heft_extractor import BodyPoseExtractor, HeftJointExtractor
from .params import HEFT_SCHEMA_VERSION, resolve_robot_name
from .drift_compensator import SupportFootDriftCompensator

log = logging.getLogger(__name__)


class MotionGmrApp:
    frontend_name = "Motion"
    retargeter_type = None
    def __init__(self, args):
        self.robot = resolve_robot_name(args.robot)
        self.fps = float(args.fps)
        self.enable_viewer = bool(args.viewer)
        self.trigger_threshold = 0.5

        # Latest-frame producer/consumer buffer
        self.frame_buf: deque = deque(maxlen=max(1, int(args.frame_buffer_len)))
        self.frame_buf_lock = threading.Lock()

        # Calibration state
        self.cali_pose: Optional[FrameDict] = None
        self._cali_pending_option: Optional[str] = None
        self._cali_running = False
        self.calibrated = False
        self.calibration_epoch = 0

        # Stream freeze state (RT+Y / RT+X)
        self.stream_frozen = False
        self._resume_zero_vel_frames = 0
        self._resume_ready_frames_remaining = 0

        # Heading / yaw helpers
        self.heading = HeadingAligner()
        self.yaw_inv = YawInvariant()
        #drift_compensator
        # Contact-aware XY drift compensation
        self.enable_drift_comp = bool(args.drift_comp)

        self.drift_comp = SupportFootDriftCompensator(
            fps=self.fps,

            height_enter=0.040,
            height_keep=0.070,

            vz_enter=0.45,
            vz_keep=0.80,

            vxy_enter=0.45,
            vxy_keep=0.80,

            enter_frames=3,
            exit_frames=3,

            # V2 correction parameters
            correction_tau=0.15,
            max_correction_rate=0.12,
            max_correction=0.08,

            # V2 step gate
            step_update_thresh=0.08,
        )

        log.info(
            "[DRIFT_CFG] "
            "tau=%.3f "
            "max_rate=%.3f "
            "max_corr=%.3f "
            "step_thresh=%.3f",
            self.drift_comp.correction_tau,
            self.drift_comp.max_correction_rate,
            self.drift_comp.max_correction,
            self.drift_comp.step_update_thresh,
        )

        self._drift_log_count = 0

        # Retargeter (created lazily on first calibration; also created
        # uncalibrated at startup so the model is validated early)
        self.retarget: Optional[object] = None
        self.retarget_lock = threading.Lock()
        self.extractor: Optional[HeftJointExtractor] = None
        self.root_pose: Optional[BodyPoseExtractor] = None

        # Prev-state for velocity estimation / freeze hold publish
        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof = None
        self.prev_pub_time = None
        self.last_head_yaw: Optional[float] = None
        self.last_head_pitch: Optional[float] = None

        # Button edge detection
        self._prev_btn = {"X": False, "Y": False, "A": False, "B": False}

        # DDS
        self.dds = self._create_publishers(args)
        self.seq = 0

        self.receiver = self._create_receiver(args)

        # Viewer (optional, requires display)
        self.viewer = None

        self._stop = threading.Event()

        # Statistics
        self.total_frames_processed = 0

    def handle_control(self, joy):
        # Frontend-internal combos (edge-triggered)
        curr_x = bool(joy.left_first_button_pressed)
        curr_y = bool(joy.left_second_button_pressed)
        curr_b = bool(joy.right_second_button_pressed)

        rt = joy.right_trigger >= self.trigger_threshold
        lt = joy.left_trigger >= self.trigger_threshold

        y_rising = curr_y and not self._prev_btn["Y"]
        x_rising = curr_x and not self._prev_btn["X"]
        b_rising = curr_b and not self._prev_btn["B"]

        if rt and y_rising:
            self._freeze_stream()
        if rt and x_rising:
            self._resume_stream()
        if rt and b_rising:
            self._request_calibration("upper_body")
        if lt and b_rising:
            self._request_calibration("whole_body")
        # LT+Y / LT+X CSV: handled in C++ PicoTeleopAdapter (policy 50 Hz)

        self._prev_btn["X"] = curr_x
        self._prev_btn["Y"] = curr_y
        self._prev_btn["A"] = bool(joy.right_first_button_pressed)
        self._prev_btn["B"] = curr_b

    def _freeze_stream(self):
        if self.stream_frozen:
            return
        self.yaw_inv.mark_freeze()
        self.stream_frozen = True
        log.info("\033[93m[GMR] stream FROZEN (RT+Y); still publishing live "
                 "(C++ holds consumption)\033[0m")

    def _resume_stream(self):
        if not self.stream_frozen:
            return
        self.yaw_inv.request_resume()
        self.stream_frozen = False
        log.info("\033[92m[GMR] stream RESUMED (RT+X)\033[0m")

    def _publish_heft_reference(
        self,
        pub_time: float,
        capture_stamp: Optional[float],
        root_loc: np.ndarray,
        root_rot_wxyz: np.ndarray,
        heft_q: np.ndarray,
        heft_qd: Optional[List[float]],
    ) -> bool:
        finite = (np.all(np.isfinite(root_loc))
                  and np.all(np.isfinite(root_rot_wxyz))
                  and np.all(np.isfinite(heft_q)))
        valid = bool(self.calibrated and self.heading.initialized and finite)

        self.seq += 1
        try:
            self.dds.publish_heft_reference(
                schema_version=HEFT_SCHEMA_VERSION,
                robot_version=self.robot,
                seq=self.seq,
                calibration_epoch=self.calibration_epoch,
                capture_timestamp_ms=int((capture_stamp or pub_time) * 1000),
                valid=valid,
                calibrated=self.calibrated,
                root_pos=root_loc,
                root_quat_wxyz=root_rot_wxyz,
                q=heft_q.tolist(),
                qd=heft_qd,
            )
        except Exception as e:
            log.error("DDS publish failed: %s", e)
        return valid

    def _request_calibration(self, option: str):
        if self._cali_running or self._cali_pending_option:
            log.warning("[GMR] calibration already running/pending")
            return
        self._cali_pending_option = option
        label = "upper_body" if option == "upper_body" else "whole_body"
        log.info("[GMR] calibration requested: %s", label)

    def _pop_latest_frame(self) -> Tuple[Optional[float], Optional[FrameDict]]:
        with self.frame_buf_lock:
            if not self.frame_buf:
                return None, None
            stamp, frame = self.frame_buf[-1]
            self.frame_buf.clear()
            return stamp, frame

    def run(self):
        log.info("=" * 70)
        log.info("%s → GMR → DDS HEFT reference frontend", self.frontend_name)
        log.info("Robot: %s", self.robot)
        log.info("FPS: %.1f", self.fps)
        log.info("Combos: RT+B=upper cali, LT+B=whole cali, "
                 "RT+Y=freeze(C++ hold), RT+X=resume+yaw; "
                 "LT+Y/X CSV → C++ heft_pico policy 50Hz")
        log.info("=" * 70)

        # Validate model + IK config + joint extraction at startup
        with self.retarget_lock:
            self.retarget = self.retargeter_type(
                src_human=self.src_format,
                tgt_robot=self.robot,
                actual_human_height=None,
                cali_pose=None,
            )
            self.extractor = HeftJointExtractor(self.retarget.model)
            self.root_pose = BodyPoseExtractor(self.retarget.model)
        log.info("Model validated: %d HEFT joints mapped; root body=%s",
                 self.extractor.num_joints, self.root_pose.body_name)

        if self.enable_viewer:
            try:
                from leju_gmr_common.robot_viewer import (
                    RobotMotionViewer, resolve_viewer_scene_xml)
                scene_xml = resolve_viewer_scene_xml(
                    getattr(self.retarget, "xml_file", None))
                self.viewer = RobotMotionViewer(
                    self.retarget.model, scene_xml=scene_xml)
                if scene_xml:
                    log.info("Viewer scene: %s", scene_xml)
            except Exception as e:
                log.warning("viewer unavailable: %s", e)
                self.viewer = None

        self.receiver.start()

        dt = 1.0 / self.fps
        tick_sleep = dt * 0.5
        next_tick = time.monotonic()

        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(next_tick - now, dt))
                    continue
                next_tick += dt
                if now - next_tick > 1.0:  # fell far behind; resync
                    next_tick = now + dt

                # Pending calibration runs inside the main loop
                if self._cali_pending_option:
                    option = self._cali_pending_option
                    self._cali_pending_option = None
                    self._run_calibration(option, tick_sleep)
                    continue

                self._tick()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _tick(self):
        pub_time = time.time()

        # ============================================================
        # 1. Get latest canonical frame
        # ============================================================
        capture_stamp, frame_gmr_raw = self._pop_latest_frame()

        if frame_gmr_raw is None:
            if self.receiver.total_frames == 0:
                if int(pub_time) % 5 == 0:
                    log.debug("[GMR] waiting for stream ...")
            return

        # ============================================================
        # 2. Contact-aware XY drift compensation
        # ============================================================
        if self.enable_drift_comp:
            try:
                frame_gmr, drift_dbg = self.drift_comp.apply(
                    frame_gmr_raw,
                    timestamp=(
                        capture_stamp
                        if capture_stamp is not None
                        else pub_time
                    ),
                )
            except Exception as e:
                log.warning("[DRIFT] compensation failed: %s", e)

                # fail-safe:
                # 如果漂移补偿出问题，不要让整个重定向挂掉
                frame_gmr = frame_gmr_raw
                drift_dbg = None

        else:
            frame_gmr = frame_gmr_raw
            drift_dbg = None

        # ============================================================
        # 3. Retarget corrected canonical frame
        # ============================================================
        try:
            with self.retarget_lock:
                qpos = self.retarget.retarget(frame_gmr)

        except Exception as e:
            log.warning("retarget() failed: %s", e)
            return

        # ============================================================
        # 4. Drift debug log
        # ============================================================
        self._drift_log_count += 1

        if (
            self.enable_drift_comp
            and drift_dbg is not None
            and self._drift_log_count % 20 == 0
        ):
            hips_raw = frame_gmr_raw["Hips"][0]
            hips_corr = frame_gmr["Hips"][0]

            active = drift_dbg.get(
                "active_support",
                None,
            )

            log.info(
                "[DRIFT] "
                "contact=L%d R%d "
                "active=%s "
                "corr_x=%.4f "
                "desired_x=%.4f "
                "hips_raw_x=%.4f "
                "hips_corr_x=%.4f",
                int(drift_dbg["left_contact"]),
                int(drift_dbg["right_contact"]),
                str(active),
                drift_dbg["corr_x"],
                drift_dbg["desired_x"],
                hips_raw[0],
                hips_corr[0],
            )

        if (
            self.enable_drift_comp
            and drift_dbg is not None
        ):
            if drift_dbg["left_step_event"]:
                log.info(
                    "[DRIFT_STEP] "
                    "side=Left "
                    "dist=%.4f "
                    "anchor_update=%d "
                    "reason=%s "
                    "anchor_x=%.4f",
                    drift_dbg["left_step_dist"],
                    int(
                        drift_dbg[
                            "left_anchor_updated"
                        ]
                    ),
                    drift_dbg["left_step_reason"],
                    drift_dbg["left_anchor_x"],
                )

            if drift_dbg["right_step_event"]:
                log.info(
                    "[DRIFT_STEP] "
                    "side=Right "
                    "dist=%.4f "
                    "anchor_update=%d "
                    "reason=%s "
                    "anchor_x=%.4f",
                    drift_dbg["right_step_dist"],
                    int(
                        drift_dbg[
                            "right_anchor_updated"
                        ]
                    ),
                    drift_dbg["right_step_reason"],
                    drift_dbg["right_anchor_x"],
                )

        # ============================================================
        # 5. Raw GMR / IK output
        # ============================================================
        qpos_raw = np.asarray(
            qpos,
            dtype=np.float64,
        ).copy()

        if self._drift_log_count % 20 == 0:
            log.info(
                "[IK_RAW] root=(%.4f %.4f %.4f)",
                qpos_raw[0],
                qpos_raw[1],
                qpos_raw[2],
            )

        # ============================================================
        # 6. Heading alignment
        # ============================================================
        qpos_pub = self.heading.align(
            qpos_raw.copy()
        )

        if qpos_pub is None:
            return

        if self.root_pose is None:
            return
        root_loc, root_rot_wxyz = self.root_pose.pose_wxyz(qpos_pub)

        if self._drift_log_count % 20 == 0:
            log.info(
                "[PUB] root/base_link=(%.4f %.4f %.4f)",
                root_loc[0],
                root_loc[1],
                root_loc[2],
            )

        # ============================================================
        # 7. Viewer (RT+Y freeze only holds controller-side
        # consumption; GMR keeps publishing live with yaw continuity)
        # ============================================================
        if self.viewer is not None:
            try:
                self.viewer.step(
                    qpos_raw,
                    self.retarget.scaled_human_data,
                )
            except Exception as e:
                log.warning("viewer.step failed: %s", e)

        # ============================================================
        # 8. Resume yaw continuity
        # ============================================================
        resume_completed = self.yaw_inv.complete_resume_if_needed(
            root_rot_wxyz
        )
        if resume_completed:
            self.prev_root_loc = None
            self.prev_root_rot_wxyz = None
            self.prev_dof = None
            self.prev_pub_time = None

            self._resume_zero_vel_frames = 10

        root_rot_wxyz = self.yaw_inv.apply(
            root_rot_wxyz
        )

        # ============================================================
        # 9. HEFT joints
        # ============================================================
        heft_q = self.extractor.extract_qpos(
            qpos_raw
        )
        heft_q[0] = -heft_q[0]

        # ============================================================
        # 10. Velocity estimate
        # ============================================================
        heft_qd = None

        if self._resume_zero_vel_frames > 0:

            heft_qd = [0.0] * int(
                heft_q.size
            )

            self._resume_zero_vel_frames -= 1

        elif (
            self.prev_pub_time is not None
            and self.prev_dof is not None
        ):
            dt_sec = pub_time - self.prev_pub_time

            if 1e-4 < dt_sec < 0.25:
                heft_qd = (
                    (heft_q - self.prev_dof)
                    / dt_sec
                ).tolist()

        self.prev_dof = heft_q.copy()
        self.prev_pub_time = pub_time

        self.prev_root_loc = root_loc.copy()
        self.prev_root_rot_wxyz = (
            root_rot_wxyz.copy()
        )

        # ============================================================
        # 11. DDS publish (always live; C++ freezes consumption on RT+Y)
        # ============================================================
        valid = self._publish_heft_reference(
            pub_time,
            capture_stamp,
            root_loc,
            root_rot_wxyz,
            heft_q,
            heft_qd,
        )

        # Publish only after the corrected HEFT frame. Repeat to tolerate
        # best-effort loss; the controller still waits for a newer frame.
        if resume_completed:
            self._resume_ready_frames_remaining = 10
        if self._resume_ready_frames_remaining > 0:
            try:
                self.dds.publish_heft_resume_ready()
                self._resume_ready_frames_remaining -= 1
            except Exception as e:
                log.warning("resume-ready publish failed: %s", e)

        # ============================================================
        # 12. Head passthrough
        # ============================================================
        try:
            full_q = qpos_raw
            model = self.retarget.model

            import mujoco as mj

            zid1 = mj.mj_name2id(
                model,
                mj.mjtObj.mjOBJ_JOINT,
                "zhead_1_joint",
            )

            zid2 = mj.mj_name2id(
                model,
                mj.mjtObj.mjOBJ_JOINT,
                "zhead_2_joint",
            )

            if zid1 >= 0 and zid2 >= 0:

                yaw = float(
                    full_q[
                        model.jnt_qposadr[zid1]
                    ]
                )

                pitch = float(
                    full_q[
                        model.jnt_qposadr[zid2]
                    ]
                )

                self.last_head_yaw = yaw
                self.last_head_pitch = pitch

                self.dds.publish_head(
                    yaw,
                    pitch,
                )

        except Exception:
            pass

        # ============================================================
        # Statistics
        # ============================================================
        self.total_frames_processed += 1

        if self.total_frames_processed % 500 == 1:
            log.info(
                "[GMR] frames processed=%d "
                "published seq=%d valid=%s",
                self.total_frames_processed,
                self.seq,
                valid,
            )

    def stop(self):
        self._stop.set()
        self.receiver.stop()
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass

    def _create_publishers(self, args):
        return DdsPublishers(domain_id=args.domain_id, heft_topic=args.heft_topic,
                             joy_topic=None, bone_topic=None,
                             head_topic=args.head_topic or None)

    def _create_receiver(self, args):
        raise NotImplementedError

    def _run_calibration(self, option, tick_sleep):
        raise NotImplementedError


def build_common_arg_parser(description):
    """Only device-independent options; each frontend owns its input options."""
    from .dds_io import HEFT_REFERENCE_TOPIC, HEAD_TRAJECTORY_TOPIC
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--robot', default='roban_s17')
    parser.add_argument('--fps', type=float, default=100.)
    parser.add_argument('--domain-id', type=int, default=0)
    parser.add_argument('--heft-topic', default=HEFT_REFERENCE_TOPIC)
    parser.add_argument('--head-topic', default=HEAD_TRAJECTORY_TOPIC)
    parser.add_argument('--frame-buffer-len', type=int, default=3)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--drift-comp', action='store_true')
    parser.add_argument('-v', '--verbose', action='store_true')
    return parser
