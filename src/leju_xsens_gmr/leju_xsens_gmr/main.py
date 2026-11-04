"""Independent Xsens/MVN + BVH → S17 GMR → DDS HEFT frontend.

Optional controls arrive as DDS joystick messages from a separate process.
No Pico implementation, Protobuf decoder or Pico virtualenv is required.
"""

import logging
import threading
import time

from leju_gmr_common.app import MotionGmrApp, build_common_arg_parser
from leju_gmr_common.dds_io import JoystickSubscriber, JOYSTICK_TOPIC
from .motion_retarget import MotionRetargeter
from leju_gmr_common.heft_extractor import BodyPoseExtractor, HeftJointExtractor
from .xsens_receiver import XsensReceiver
from .xsens_bvh import BvhDescriptionReceiver

log = logging.getLogger(__name__)


class XsensGmrApp(MotionGmrApp):
    src_format = "bvh_xsense"
    frontend_name = "Xsens"
    retargeter_type = MotionRetargeter

    def __init__(self, args):
        self._bvh_lock = threading.Lock()
        self._bvh_calibration = None
        self._bvh_pending = False
        self.control_reader = None
        self.bvh_receiver = None
        self.xsens_args = args
        super().__init__(args)
        self.bvh_receiver = BvhDescriptionReceiver(
            host=args.host, port=args.bvh_port,
            callback=self._on_bvh, arm_pose_deg=args.cali_arm_pose_deg,
        )
        if args.control_topic:
            self.control_reader = JoystickSubscriber(self.dds.participant, args.control_topic)

    def _create_receiver(self, args):
        return XsensReceiver(
            host=args.host, port=args.port,
            frame_callback=self._on_xsens_frame,
            expected_character_id=args.character_id,
            publish_hz=args.pose_publish_hz,
            queue_target_depth=args.pose_queue_target_depth,
            queue_max_size=args.pose_queue_max_size,
            stale_timeout=args.pose_stale_timeout,
        )

    def _on_xsens_frame(self, frame_gmr, capture_time):
        # These are canonical named Xsens bones, not the 24-index Quest schema.
        with self.frame_buf_lock:
            self.frame_buf.append((capture_time, frame_gmr))

    def _on_bvh(self, cali_pose, human_height):
        # Network thread only stages data. MuJoCo state changes on the consumer.
        with self._bvh_lock:
            self._bvh_calibration = (cali_pose, human_height)
            self._bvh_pending = True

    def _run_calibration(self, option, tick_sleep):
        # BVH defines the static reference. Button re-calibration reapplies this
        # reference rather than averaging the actor's current moving pose.
        with self._bvh_lock:
            calibration = self._bvh_calibration
            self._bvh_pending = False
        if calibration is None:
            log.warning("Waiting for actor BVH on TCP %d", self.xsens_args.bvh_port)
            return
        self._cali_running = True
        try:
            cali_pose, height = calibration
            retarget = MotionRetargeter(
                src_human=self.src_format, tgt_robot=self.robot,
                actual_human_height=height, cali_pose=cali_pose,
                cali_option="whole_body",
            )
            with self.retarget_lock:
                self.retarget = retarget
                self.extractor = HeftJointExtractor(retarget.model)
                self.root_pose = BodyPoseExtractor(retarget.model)
            self.cali_pose = cali_pose
            self.prev_root_loc = self.prev_root_rot_wxyz = None
            self.prev_dof = self.prev_pub_time = None
            self.last_head_yaw = self.last_head_pitch = None
            self._resume_zero_vel_frames = 0
            self.heading.reset()
            self.yaw_inv.reset()
            self.drift_comp.reset()
            self.stream_frozen = False
            with self.frame_buf_lock:
                self.frame_buf.clear()
            self.calibrated = True
            self.calibration_epoch += 1
            log.info("Xsens BVH calibrated: height=%.3fm epoch=%d; waiting for heading",
                     height, self.calibration_epoch)
        except Exception:
            # Never continue publishing an old valid reference after failed calibration.
            self.calibrated = False
            raise
        finally:
            self._cali_running = False

    def _pop_latest_frame(self):
        stamp, frame = super()._pop_latest_frame()
        if stamp is not None and time.time() - stamp > self.xsens_args.pose_stale_timeout:
            return None, None
        return stamp, frame

    def _tick(self):
        if self.control_reader is not None:
            for joy in self.control_reader.take_fresh():
                self.handle_control(joy)
        with self._bvh_lock:
            pending = self._bvh_pending
            self._bvh_pending = False
        if pending:
            self._run_calibration("whole_body", 0.0)
        if self.calibrated:
            super()._tick()
        else:
            # Do not solve or publish until the actor's static skeleton is ready.
            with self.frame_buf_lock:
                self.frame_buf.clear()

    def run(self):
        try:
            self.bvh_receiver.start()
            log.info("Waiting for actor BVH on TCP %s:%d", self.xsens_args.host,
                     self.bvh_receiver.port)
            log.info("MVN requires Quaternion + Scale for character %d; BVH calibrates once",
                     self.xsens_args.character_id)
            log.info("RT+B / LT+B reapply the BVH reference; RT+Y/X freeze/resume")
            super().run()
        finally:
            self.stop()

    def stop(self):
        if self.bvh_receiver is not None:
            self.bvh_receiver.stop()
        self.control_reader = None
        super().stop()


def build_arg_parser():
    parser = build_common_arg_parser(__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9763)
    parser.add_argument("--control-topic", default=JOYSTICK_TOPIC,
                        help="DDS joystick control topic; empty disables controls")
    parser.add_argument("--bvh-port", type=int, default=9764)
    parser.add_argument("--character-id", type=int, default=0,
                        help="MVN actor ID (one actor per frontend)")
    parser.add_argument("--cali-arm-pose-deg", type=float, default=90.0)
    parser.add_argument("--pose-publish-hz", type=float, default=120.0)
    parser.add_argument("--pose-queue-target-depth", type=int, default=10)
    parser.add_argument("--pose-queue-max-size", type=int, default=15)
    parser.add_argument("--pose-stale-timeout", type=float, default=0.2)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    import math
    for field in ("fps", "pose_publish_hz", "pose_stale_timeout"):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(field + " must be finite and positive")
    if not math.isfinite(args.cali_arm_pose_deg):
        parser.error("cali_arm_pose_deg must be finite")
    if not 0 <= args.character_id <= 255:
        parser.error("character-id must be in [0, 255]")
    if not 1 <= args.pose_queue_target_depth <= args.pose_queue_max_size:
        parser.error("require 1 <= pose-queue-target-depth <= pose-queue-max-size")
    for field in ("port", "bvh_port"):
        if not 1 <= getattr(args, field) <= 65535:
            parser.error(field + " must be in [1, 65535]")
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    XsensGmrApp(args).run()


if __name__ == "__main__":
    main()
