"""Pico wire-contract regression tests."""

import json

import numpy as np

from leju_pico_gmr.pico_receiver import (
    BODY_TRACKER_ROLES,
    NUM_POSE_BONES,
    PicoReceiver,
    RobotInfoBroadcaster,
    _effective_grip,
)
from leju_pico_gmr.protos import body_tracking_extended_pb2 as proto


def test_discovery_payload_matches_existing_pico_app(monkeypatch):
    monkeypatch.setattr(
        "leju_pico_gmr.pico_receiver.get_localip_and_broadcast_ips",
        lambda: [("192.168.30.182", "192.168.30.255")],
    )
    broadcaster = RobotInfoBroadcaster(robot_name="ROBAN")
    payload = json.loads(broadcaster._encode_message("192.168.30.182"))
    assert payload == {
        "data": {
            "robot_name": "ROBAN",
            "robot_ip": "192.168.30.182",
            "eef_type": None,
        }
    }


def _receiver_without_socket():
    receiver = PicoReceiver.__new__(PicoReceiver)
    receiver.cached_matrices = np.tile(
        np.eye(4), (len(BODY_TRACKER_ROLES), 1, 1))
    return receiver


def _body_message(count):
    msg = proto.VRData()
    for i in range(count):
        pose = msg.full_body.full_body.add()
        pose.pos_x = float(i)
        pose.rot_qw = 1.0
    return msg


def test_incomplete_body_frame_is_rejected():
    receiver = _receiver_without_socket()
    assert receiver._parse_full_body_to_matrix(
        _body_message(NUM_POSE_BONES - 1)) is None


def test_complete_body_frame_is_accepted():
    receiver = _receiver_without_socket()
    matrices = receiver._parse_full_body_to_matrix(
        _body_message(NUM_POSE_BONES))
    assert matrices is not None
    assert matrices[NUM_POSE_BONES - 1, 0, 3] == NUM_POSE_BONES - 1


class _ControllerState:
    thumbstick = [0.0, 0.0]
    primary_button = False
    secondary_button = False
    grip_button = True
    trigger_button = False
    grip_value = 0.0
    trigger_value = 0.0


def test_digital_grip_is_preserved_for_switch_combo():
    parsed = PicoReceiver._parse_controller_state(_ControllerState())
    assert _effective_grip(parsed["axes"][2], parsed["buttons"][2]) == 1.0
