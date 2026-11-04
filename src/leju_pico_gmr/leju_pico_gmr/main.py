"""Pico UDP body tracking → GMR → DDS HEFT frontend for S17."""
import argparse
import logging
import sys
import time
from typing import List
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation
from leju_gmr_common.app import MotionGmrApp, build_common_arg_parser
from leju_gmr_common.calibration import (FrameDict, build_cali_pose_from_frames,
    estimate_human_height_from_frame, upper_body_cali_part)
from leju_gmr_common.dds_io import DdsPublishers, Pose
from leju_gmr_common.heft_extractor import BodyPoseExtractor, HeftJointExtractor
from .motion_retarget import MotionRetargeter
from .calibration import world_matrices_to_gmr_frame
from .pico_receiver import PicoReceiver, JoyState, NUM_POSE_BONES

log = logging.getLogger(__name__)


def _world_matrices_to_bone_poses(world_matrices: np.ndarray) -> List[Pose]:
    """Convert Unity→world 4x4 bone matrices to QuestBonePoses Pose list.

    Index order matches Pico BODY_TRACKER_ROLES / NUM_POSE_BONES (24).
    Quaternion order is xyzw (qx,qy,qz,qw), matching QuestBonePoses.idl.
    """
    n = min(int(world_matrices.shape[0]), NUM_POSE_BONES)
    poses: List[Pose] = []
    quaternions = SciRotation.from_matrix(world_matrices[:n, :3, :3]).as_quat() if n else []
    for i in range(n):
        M = world_matrices[i]
        q_xyzw = quaternions[i]
        poses.append(Pose(
            x=float(M[0, 3]),
            y=float(M[1, 3]),
            z=float(M[2, 3]),
            qx=float(q_xyzw[0]),
            qy=float(q_xyzw[1]),
            qz=float(q_xyzw[2]),
            qw=float(q_xyzw[3]),
        ))
    return poses


class PicoGmrApp(MotionGmrApp):
    src_format = "bvh_motive_streaming"
    frontend_name = "Pico"
    retargeter_type = MotionRetargeter

    def _create_publishers(self, args):
        return DdsPublishers(domain_id=args.domain_id, heft_topic=args.heft_topic,
                             joy_topic=args.joy_topic, bone_topic=args.bone_topic or None,
                             head_topic=args.head_topic or None)

    def _on_joy(self, joy):
        try:
            self.dds.publish_joysticks(joy)
        except Exception as exc:
            log.debug("joystick publish failed: %s", exc)
        self.handle_control(joy)

    def _create_receiver(self, args):
        self.cali_frame_count = int(args.cali_frame_count)
        self.cali_timeout_sec = float(args.cali_timeout_sec)
        return PicoReceiver(
            host=args.host, port=int(args.port),
            enable_ip_broadcast=not args.no_broadcast,
            robot_name=args.broadcast_name,
            frame_callback=self._on_frame,
            joy_callback=self._on_joy,
        )

    def _on_frame(self, world_matrices: np.ndarray, capture_time: float):
        try:
            frame_gmr = world_matrices_to_gmr_frame(world_matrices)
        except Exception as e:
            log.warning("frame conversion failed: %s", e)
            return
        with self.frame_buf_lock:
            self.frame_buf.append((capture_time, frame_gmr))

        # Always republish world-frame raw bones for record/debug (QuestBonePoses).
        try:
            poses = _world_matrices_to_bone_poses(world_matrices)
            self.dds.publish_bone_poses(
                poses,
                capture_timestamp_ms=int(capture_time * 1000),
            )
        except Exception as e:
            log.debug("bone_poses publish failed: %s", e)

    def _run_calibration(self, option: str, tick_sleep: float):
        self._cali_running = True
        try:
            with self.frame_buf_lock:
                self.frame_buf.clear()
            log.info("[Pico] collecting calibration frames: target=%d timeout=%.1fs",
                     self.cali_frame_count, self.cali_timeout_sec)

            frames: List[FrameDict] = []
            t0 = time.time()
            while not self._stop.is_set():
                _, frame = self._pop_latest_frame()
                if frame is not None:
                    frames.append(frame)
                    if len(frames) >= self.cali_frame_count:
                        break
                if (time.time() - t0) > self.cali_timeout_sec:
                    break
                time.sleep(tick_sleep)

            if len(frames) == 0:
                log.warning("[Pico] calibration failed: no frames collected")
                return

            log.info("[Pico] collected %d calibration frames", len(frames))
            self.cali_pose = build_cali_pose_from_frames(frames)

            heights = []
            for f in frames:
                try:
                    heights.append(estimate_human_height_from_frame(f))
                except Exception:
                    pass
            human_height = float(np.mean(heights)) if heights else None
            log.info("[Pico] estimated human height: %s", human_height)

            with self.retarget_lock:
                if self.retarget is None:
                    self.retarget = MotionRetargeter(
                        src_human=self.src_format,
                        tgt_robot=self.robot,
                        actual_human_height=human_height,
                        cali_pose=self.cali_pose,
                        cali_option=option,
                        cali_part=upper_body_cali_part,
                    )
                    self.extractor = HeftJointExtractor(self.retarget.model)
                    self.root_pose = BodyPoseExtractor(self.retarget.model)
                else:
                    self.retarget.autocalibrate_offsets_from_default_pose(
                        self.cali_pose,
                        align_feet=True,
                        cali_option=option,
                        cali_part=upper_body_cali_part,
                    )
                    self.retarget._build_offset_cache()

            # Reset runtime state after calibration
            self.prev_root_loc = None
            self.prev_root_rot_wxyz = None
            self.prev_dof = None
            self.prev_pub_time = None
            self.heading.reset()
            self.yaw_inv.reset()
            #drift_compensator
            self.drift_comp.reset()
            self.stream_frozen = False

            self.calibrated = True
            self.calibration_epoch += 1
            log.info("\033[92m[Pico] ✅ %s calibration finished (epoch=%d)\033[0m",
                     option, self.calibration_epoch)
        except Exception as e:
            log.error("[Pico] calibration failed: %s", e, exc_info=True)
        finally:
            self._cali_running = False

def build_arg_parser():
    parser = build_common_arg_parser(__doc__)
    parser.set_defaults(fps=50.)
    parser.add_argument('--python-switch-ms', type=float, default=1.,
                        help='Python thread handoff interval for receiver/IK concurrency')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=12345)
    parser.add_argument('--joy-topic', default='/rt/pico/joysticks')
    parser.add_argument('--bone-topic', default='/rt/pico/bone_poses')
    parser.add_argument('--cali-frame-count', type=int, default=10)
    parser.add_argument('--cali-timeout-sec', type=float, default=3.)
    parser.add_argument('--no-broadcast', action='store_true')
    parser.add_argument('--broadcast-name', default='ROBAN')
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    if not 0 < args.python_switch_ms <= 5:
        parser.error('--python-switch-ms must be in (0, 5]')
    # NumPy, MuJoCo and DDS release/reacquire the GIL frequently. Short handoffs
    # prevent the 100 Hz input thread from delaying the reference consumer.
    sys.setswitchinterval(args.python_switch_ms / 1000.)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    app = PicoGmrApp(args)
    app.run()

if __name__ == "__main__":
    main()
