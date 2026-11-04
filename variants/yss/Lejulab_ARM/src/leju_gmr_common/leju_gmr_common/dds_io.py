"""Shared DDS wire contracts and optional motion/control publishers.

Python IdlStruct mirrors of the lejusdk IDL types:
- leju::msgs::HeftReference (HeftReference.idl)
- leju::msgs::QuestJoysticks (QuestJoysticks.idl, reused for Pico)
- leju::msgs::StringData (RT+X corrected-reference acknowledgement)
- leju::msgs::QuestBonePoses (QuestBonePoses.idl, reused for Pico raw bones)
- leju::msgs::JointTrajectoryPoint (JointTrajectory.idl, head passthrough)

Field order and types must match the IDL exactly.
"""

import ctypes
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (array, float32, float64, int32, int64,
                                  sequence, uint32, uint64)
from cyclonedds.pub import DataWriter
from cyclonedds.topic import Topic

HEFT_REFERENCE_TOPIC = "/rt/pico/heft_reference"
HEFT_RESUME_READY_TOPIC = "/rt/pico/heft_resume_ready"
# Existing platform bindings use these topic names. Device implementations
# are not dependencies of this transport layer.
JOYSTICK_TOPIC = "/rt/pico/joysticks"
BONE_POSES_TOPIC = "/rt/pico/bone_poses"
HEAD_TRAJECTORY_TOPIC = "/rt/head_trajectory"


def preload_cyclonedds() -> None:
    """Preload libddsc from CYCLONEDDS_LIB_DIR if provided (ARM deployments)."""
    if os.environ.get("CYCLONEDDS_PRELOADED") == "1":
        return
    lib_dir = os.environ.get("CYCLONEDDS_LIB_DIR")
    if not lib_dir:
        return
    lib_path = Path(lib_dir) / "libddsc.so"
    if lib_path.is_file():
        ctypes.CDLL(str(lib_path))
        os.environ["CYCLONEDDS_PRELOADED"] = "1"


@dataclass
class HeftReference(IdlStruct, typename="leju::msgs::HeftReference"):
    header_sec: int32
    header_nanosec: uint32
    schema_version: uint32
    robot_version: str
    seq: uint64
    calibration_epoch: uint32
    capture_timestamp_ms: int64
    valid: bool
    calibrated: bool
    root_pos: array[float64, 3]
    root_quat: array[float64, 4]
    q: sequence[float64]
    qd: sequence[float64]


@dataclass
class QuestJoysticks(IdlStruct, typename="leju::msgs::QuestJoysticks"):
    header_sec: int32
    header_nanosec: uint32
    left_x: float32
    left_y: float32
    left_trigger: float32
    left_grip: float32
    left_first_button_pressed: bool
    left_second_button_pressed: bool
    left_first_button_touched: bool
    left_second_button_touched: bool
    right_x: float32
    right_y: float32
    right_trigger: float32
    right_grip: float32
    right_first_button_pressed: bool
    right_second_button_pressed: bool
    right_first_button_touched: bool
    right_second_button_touched: bool


@dataclass
class StringData(IdlStruct, typename="leju::msgs::StringData"):
    header_sec: int32
    header_nanosec: uint32
    data: str


@dataclass
class JointTrajectoryPoint(IdlStruct, typename="leju::msgs::JointTrajectoryPoint"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    acc: sequence[float64]


@dataclass
class Pose(IdlStruct, typename="leju::msgs::Pose"):
    x: float32
    y: float32
    z: float32
    qx: float32
    qy: float32
    qz: float32
    qw: float32


@dataclass
class QuestBonePoses(IdlStruct, typename="leju::msgs::QuestBonePoses"):
    header_sec: int32
    header_nanosec: uint32
    timestamp_ms: int64
    is_high_confidence: bool
    is_hand_tracking: bool
    poses: sequence[Pose]


def _stamp() -> tuple:
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    return sec, nsec


class DdsPublishers:
    """Owns the DomainParticipant and all frontend DataWriters."""

    def __init__(self, domain_id: int = 0,
                 heft_topic: Optional[str] = HEFT_REFERENCE_TOPIC,
                 joy_topic: Optional[str] = None,
                 bone_topic: Optional[str] = None,
                 head_topic: Optional[str] = HEAD_TRAJECTORY_TOPIC):
        preload_cyclonedds()
        self.participant = DomainParticipant(domain_id)
        self.heft_writer = (DataWriter(
            self.participant, Topic(self.participant, heft_topic, HeftReference))
            if heft_topic else None)
        self.resume_ready_writer = (DataWriter(
            self.participant,
            Topic(self.participant, HEFT_RESUME_READY_TOPIC, StringData))
            if heft_topic else None)
        self.joy_writer = (DataWriter(
            self.participant, Topic(self.participant, joy_topic, QuestJoysticks))
            if joy_topic else None)
        self.bone_writer = None
        if bone_topic:
            self.bone_writer = DataWriter(
                self.participant,
                Topic(self.participant, bone_topic, QuestBonePoses))
        self.head_writer = None
        if head_topic:
            self.head_writer = DataWriter(
                self.participant,
                Topic(self.participant, head_topic, JointTrajectoryPoint))

    def publish_heft_reference(self, *, schema_version: int, robot_version: str,
                               seq: int, calibration_epoch: int,
                               capture_timestamp_ms: int, valid: bool,
                               calibrated: bool, root_pos, root_quat_wxyz,
                               q: List[float], qd: Optional[List[float]] = None):
        sec, nsec = _stamp()
        msg = HeftReference(
            header_sec=sec,
            header_nanosec=nsec,
            schema_version=schema_version,
            robot_version=robot_version,
            seq=seq,
            calibration_epoch=calibration_epoch,
            capture_timestamp_ms=capture_timestamp_ms,
            valid=valid,
            calibrated=calibrated,
            root_pos=[float(x) for x in root_pos],
            root_quat=[float(x) for x in root_quat_wxyz],
            q=[float(x) for x in q],
            qd=[float(x) for x in (qd if qd is not None else [])],
        )
        self.heft_writer.write(msg)

    def publish_bone_poses(self, poses: Sequence[Pose], *,
                           capture_timestamp_ms: int,
                           is_high_confidence: bool = True,
                           is_hand_tracking: bool = False) -> None:
        """Publish Pico world-frame bone poses (QuestBonePoses layout)."""
        if self.bone_writer is None:
            return
        sec, nsec = _stamp()
        msg = QuestBonePoses(
            header_sec=sec,
            header_nanosec=nsec,
            timestamp_ms=int(capture_timestamp_ms),
            is_high_confidence=bool(is_high_confidence),
            is_hand_tracking=bool(is_hand_tracking),
            poses=list(poses),
        )
        self.bone_writer.write(msg)

    def publish_joysticks(self, joy) -> None:
        """Publish an object exposing the QuestJoysticks control fields."""
        if self.joy_writer is None:
            return
        sec, nsec = _stamp()
        msg = QuestJoysticks(
            header_sec=sec,
            header_nanosec=nsec,
            left_x=float(joy.left_x),
            left_y=float(joy.left_y),
            left_trigger=float(joy.left_trigger),
            left_grip=float(joy.left_grip),
            left_first_button_pressed=bool(joy.left_first_button_pressed),
            left_second_button_pressed=bool(joy.left_second_button_pressed),
            left_first_button_touched=False,
            left_second_button_touched=False,
            right_x=float(joy.right_x),
            right_y=float(joy.right_y),
            right_trigger=float(joy.right_trigger),
            right_grip=float(joy.right_grip),
            right_first_button_pressed=bool(joy.right_first_button_pressed),
            right_second_button_pressed=bool(joy.right_second_button_pressed),
            right_first_button_touched=False,
            right_second_button_touched=False,
        )
        self.joy_writer.write(msg)

    def publish_heft_resume_ready(self) -> None:
        """Acknowledge that a yaw-corrected reference is on the wire."""
        if self.resume_ready_writer is None:
            return
        sec, nsec = _stamp()
        self.resume_ready_writer.write(StringData(
            header_sec=sec,
            header_nanosec=nsec,
            data="ready",
        ))

    def publish_head(self, yaw_rad: float, pitch_rad: float) -> None:
        if self.head_writer is None:
            return
        sec, nsec = _stamp()
        msg = JointTrajectoryPoint(
            header_sec=sec,
            header_nanosec=nsec,
            q=[float(yaw_rad), float(pitch_rad)],
            v=[0.0, 0.0],
            acc=[0.0, 0.0],
        )
        self.head_writer.write(msg)


class JoystickSubscriber:
    """Consumer-thread DDS controls; no device receiver and no rebroadcast.

    Old queued controls must not trigger calibration or resume on startup.
    The optional reader is polled by the motion consumer, avoiding an extra
    application thread and concurrent control-state mutations.
    """

    def __init__(self, participant, topic=JOYSTICK_TOPIC, max_age=0.5):
        from cyclonedds.sub import DataReader
        self.reader = DataReader(participant, Topic(participant, topic, QuestJoysticks))
        self.max_age = max_age

    def take_fresh(self, now=None):
        messages = self.reader.take(N=100)
        now = time.time() if now is None else now
        fresh = []
        for msg in messages:
            if not isinstance(msg, QuestJoysticks):
                continue
            stamp = msg.header_sec + msg.header_nanosec * 1e-9
            if 0. <= now - stamp <= self.max_age:
                fresh.append(msg)
        return fresh
