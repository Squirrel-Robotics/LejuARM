"""Pure calibration tests followed by real localhost TCP lifecycle tests."""

import ast
import copy
import json
import socket
import struct
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from leju_xsens_gmr import xsens_bvh as bvh


@pytest.fixture
def payload():
    joints = []

    def add(name, parent, offset):
        pid = next((j["id"] for j in joints if j["name"] == parent), 0)
        joints.append(dict(id=len(joints) + 1, name=name, parent_id=pid,
                           offset=list(offset)))

    add("Hips", None, [12, 34, 56])  # Must be ignored, not a world translation.
    add("Chest", "Hips", [0, .4, 0])
    add("Neck", "Chest", [0, .15, 0])
    add("Head", "Neck", [0, .2, 0])
    for side, sign in (("Left", 1), ("Right", -1)):
        add(side + "Collar", "Chest", [sign * .1, .1, 0])
        add(side + "Shoulder", side + "Collar", [sign * .15, 0, 0])
        add(side + "Elbow", side + "Shoulder", [sign * .3, 0, 0])
        add(side + "Wrist", side + "Elbow", [sign * .25, 0, 0])
        add(side + "Hip", "Hips", [sign * .1, -.1, 0])
        add(side + "Knee", side + "Hip", [0, -.4, 0])
        add(side + "Ankle", side + "Knee", [0, -.4, 0])
        add(side + "Toe", side + "Ankle", [0, -.05, .2])
    return dict(protocol=bvh.PROTOCOL, unit="m", joints=joints)


def test_validation_copies_and_normalizes(payload):
    for joint in payload["joints"]:
        joint["name"] = "actor_" + joint["name"]
    original = copy.deepcopy(payload)
    frame, joints = bvh.validate_bvh_payload(payload)
    assert frame == "xsense_bvh"
    assert joints[0]["name"] == "Hips"
    joints[0]["offset"][0] = -999
    assert payload == original
    payload["frame_id"] = " actor frame "
    assert bvh.validate_bvh_payload(payload)[0] == "actor frame"


@pytest.mark.parametrize("name", list(bvh.XS_NAME_MAPPING))
def test_every_required_name_is_checked(payload, name):
    next(j for j in payload["joints"] if j["name"] == name)["name"] = "Unmapped"
    with pytest.raises(bvh.BvhPayloadError, match=name):
        bvh.validate_bvh_payload(payload)


@pytest.mark.parametrize("key,value", [
    ("protocol", "wrong"), ("unit", "cm"), ("frame_id", " "),
    ("frame_id", 12), ("joints", []), ("joints", {}), ("joints", [None]),
])
def test_bad_payload_fields(payload, key, value):
    payload[key] = value
    with pytest.raises(bvh.BvhPayloadError):
        bvh.validate_bvh_payload(payload)


@pytest.mark.parametrize("key,value", [
    ("id", True), ("id", 1.0), ("id", 2),
    ("parent_id", False), ("parent_id", 0.0), ("parent_id", -1),
    ("parent_id", 1), ("parent_id", 999),
    ("name", ""), ("name", None), ("name", "actor_"),
    ("offset", [1, 2]), ("offset", "123"), ("offset", None),
])
def test_bad_joint_fields(payload, key, value):
    payload["joints"][0][key] = value
    with pytest.raises(bvh.BvhPayloadError):
        bvh.validate_bvh_payload(payload)


@pytest.mark.parametrize("value", [True, False, "1", None, float("nan"),
                                  float("inf"), -float("inf"), 10 ** 400])
def test_offsets_require_finite_nonbool_numbers(payload, value):
    payload["joints"][0]["offset"][0] = value
    with pytest.raises(bvh.BvhPayloadError):
        bvh.validate_bvh_payload(payload)


@pytest.mark.parametrize("value", [None, [], "payload", 7])
def test_payload_requires_object(value):
    with pytest.raises(bvh.BvhPayloadError):
        bvh.validate_bvh_payload(value)


def test_normalized_name_collision(payload):
    payload["joints"][1]["name"] = "actor_Hips"
    with pytest.raises(bvh.BvhPayloadError, match="duplicate"):
        bvh.validate_bvh_payload(payload)


def test_fk_canonical_positions_orientations_and_height(payload):
    original = copy.deepcopy(payload)
    pose, height = bvh.calibration_from_payload(payload)
    assert payload == original
    assert set(pose) == set(bvh.XS_NAME_MAPPING.values()) | {"LeftFootMod", "RightFootMod"}
    np.testing.assert_allclose(pose["Hips"][0], [0, 0, 0])
    np.testing.assert_allclose(pose["Hips"][1], [.5, .5, .5, .5])
    np.testing.assert_allclose(pose["Spine"][0], [0, 0, .4])
    np.testing.assert_allclose(pose["Head"][0], [0, 0, .75])
    assert height == pytest.approx(1.65)  # Ankle, not toe or skull-tip height.
    canonical = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    for side, sign in (("Left", 1), ("Right", -1)):
        np.testing.assert_allclose(pose[side + "Arm"][0], [0, sign * .25, .5])
        np.testing.assert_allclose(pose[side + "ForeArm"][0], [0, sign * .25, .2], atol=1e-15)
        np.testing.assert_allclose(pose[side + "Hand"][0], [0, sign * .25, -.05], atol=1e-15)
        expected = canonical @ Rotation.from_euler("z", -sign * 90, degrees=True).as_matrix()
        for name in ("Arm", "ForeArm", "Hand"):
            q = pose[side + name][1]
            np.testing.assert_allclose(Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix(), expected, atol=1e-15)
        np.testing.assert_allclose(pose[side + "ToeBase"][0], [.2, sign * .1, -.95])
        np.testing.assert_allclose(pose[side + "FootMod"][0], pose[side + "Foot"][0])
        np.testing.assert_allclose(pose[side + "FootMod"][1], pose[side + "ToeBase"][1])
    for pos, quat in pose.values():
        assert pos.shape == (3,) and quat.shape == (4,)
        assert np.all(np.isfinite(pos))
        assert np.linalg.norm(quat) == pytest.approx(1)


def test_custom_arm_angle(payload):
    pose, height = bvh.calibration_from_payload(payload, arm_pose_deg=0)
    np.testing.assert_allclose(pose["LeftHand"][0], [0, .8, .5])
    np.testing.assert_allclose(pose["RightHand"][0], [0, -.8, .5])
    assert height == pytest.approx(1.65)


def test_extra_joint_participates_in_fk_and_all_roots_zero(payload):
    payload["joints"].append(dict(id=21, name="Aux", parent_id=1, offset=[0, .1, 0]))
    # Move Aux before Chest, renumbering to retain the wire ordering contract.
    extra = payload["joints"].pop()
    payload["joints"].insert(1, extra)
    for index, joint in enumerate(payload["joints"], 1):
        joint["id"] = index
        if joint["parent_id"] > 1:
            joint["parent_id"] += 1
    payload["joints"][2]["parent_id"] = 2
    pose, height = bvh.calibration_from_payload(payload)
    assert "Aux" not in pose
    assert height == pytest.approx(1.75)
    payload["joints"][1]["parent_id"] = 0
    pose, height = bvh.calibration_from_payload(payload)
    assert height == pytest.approx(1.65)


@pytest.mark.parametrize("scale", [0, -1, .01, 1000])
def test_implausible_height_rejected(payload, scale):
    for joint in payload["joints"]:
        joint["offset"] = [v * scale for v in joint["offset"]]
    with pytest.raises(bvh.BvhPayloadError, match="height"):
        bvh.calibration_from_payload(payload)


def test_fk_overflow_rejected(payload):
    payload["joints"][1]["offset"] = [1e308, 0, 0]
    payload["joints"][2]["offset"] = [1e308, 0, 0]
    with pytest.raises(bvh.BvhPayloadError, match="nonfinite"):
        bvh.calibration_from_payload(payload)


@pytest.mark.parametrize("angle", [True, "90", float("nan"), float("inf")])
def test_invalid_arm_angle(payload, angle):
    with pytest.raises(bvh.BvhPayloadError):
        bvh.calibration_from_payload(payload, angle)


def test_python38_syntax_and_no_ros_dds_imports():
    tree = ast.parse(Path(bvh.__file__).read_text(), feature_version=(3, 8))
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append(node.module or "")
    assert not any(name.startswith(("rospy", "rclpy", "geometry_msgs", "kuavo_msgs", "cyclonedds"))
                   for name in imports)


# Network tests use independent framing helpers, never the implementation's I/O.
def _frame(payload):
    raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw


def _exact(client, size):
    result = b""
    while len(result) < size:
        chunk = client.recv(size - len(result))
        assert chunk, "unexpected EOF in ack"
        result += chunk
    return result


def _ack(client):
    size = struct.unpack(">I", _exact(client, 4))[0]
    assert 0 < size < 65536
    return json.loads(_exact(client, size).decode("utf-8"))


def _request(receiver, data):
    with socket.create_connection(("127.0.0.1", receiver.port), timeout=2) as client:
        client.sendall(data)
        return _ack(client)


@pytest.fixture
def receiver():
    instance = bvh.BvhDescriptionReceiver("127.0.0.1", 0)
    instance.start()
    try:
        yield instance
    finally:
        instance.stop()


def test_tcp_valid_fragmented_utf8_and_callback(receiver, payload):
    received = []
    receiver.callback = lambda pose, height: received.append((pose, height))
    payload["frame_id"] = "骨架"
    data = _frame(payload)
    with socket.create_connection(("127.0.0.1", receiver.port), timeout=2) as client:
        for chunk in (data[:1], data[1:3], data[3:17], data[17:]):
            client.sendall(chunk)
        assert _ack(client) == {"ok": True, "message": "BVH skeleton received", "joint_count": 20}
    assert len(received) == 1
    expected, height = bvh.calibration_from_payload(payload)
    assert received[0][1] == height
    for name in expected:
        for actual, wanted in zip(received[0][0][name], expected[name]):
            np.testing.assert_allclose(actual, wanted)


@pytest.mark.parametrize("data", [
    struct.pack(">I", 0), struct.pack(">I", bvh.MAX_PAYLOAD_BYTES + 1),
    struct.pack(">I", 1) + b"{", struct.pack(">I", 1) + b"\xff",
    _frame([]), _frame({"protocol": "wrong"}),
])
def test_tcp_bad_wire_allows_retry(receiver, payload, data):
    ack = _request(receiver, data)
    assert ack["ok"] is False and ack["message"]
    assert _request(receiver, _frame(payload))["ok"] is True


@pytest.mark.parametrize("kind", ["missing", "nonfinite", "height"])
def test_tcp_bad_skeleton_allows_retry(receiver, payload, kind):
    invalid = copy.deepcopy(payload)
    if kind == "missing":
        invalid["joints"][-1]["name"] = "Other"
    elif kind == "nonfinite":
        invalid["joints"][0]["offset"][0] = float("nan")
    else:
        for joint in invalid["joints"]:
            joint["offset"] = [0, 0, 0]
    assert _request(receiver, _frame(invalid))["ok"] is False
    assert _request(receiver, _frame(payload))["ok"] is True


def test_first_valid_only_concurrent_and_restart(receiver, payload):
    calls = []
    receiver.callback = lambda *args: calls.append(args)
    receiver.start()  # Idempotent: must not reset the latch.
    with ThreadPoolExecutor(max_workers=6) as pool:
        replies = list(pool.map(lambda _: _request(receiver, _frame(payload)), range(6)))
    assert sum(reply["ok"] for reply in replies) == 1
    assert len(calls) == 1
    receiver.start()
    assert _request(receiver, _frame(payload))["ok"] is False
    receiver.stop()
    receiver.stop()
    receiver.start()
    assert _request(receiver, _frame(payload))["ok"] is True
    assert len(calls) == 2


def test_callback_failure_does_not_reconfigure(receiver, payload):
    calls = []

    def fail(*args):
        calls.append(args)
        raise RuntimeError("callback failed")

    receiver.callback = fail
    assert _request(receiver, _frame(payload)) == {"ok": False, "message": "callback failed"}
    assert _request(receiver, _frame(payload))["ok"] is False
    assert len(calls) == 1


@pytest.mark.parametrize("partial", [b"", b"\x00\x00", struct.pack(">I", 100) + b"{"])
def test_stop_unblocks_active_reads_and_joins(receiver, partial):
    with socket.create_connection(("127.0.0.1", receiver.port), timeout=2) as client:
        if partial:
            client.sendall(partial)
        deadline = time.monotonic() + 2
        workers = []
        while time.monotonic() < deadline:
            with receiver._lock:
                workers = list(receiver._workers)
            if workers:
                break
            threading.Event().wait(.005)
        assert workers, "client was not accepted"
        listener = receiver._thread
        start = time.monotonic()
        receiver.stop()
        assert time.monotonic() - start < 1
        assert not listener.is_alive()
        assert all(not worker.is_alive() for worker in workers)
        assert not receiver._clients


def test_partial_client_does_not_block_valid_client(receiver, payload):
    with socket.create_connection(("127.0.0.1", receiver.port), timeout=2) as stalled:
        stalled.sendall(b"\x00")
        assert _request(receiver, _frame(payload))["ok"] is True


def test_truncated_body_error_and_retry(receiver, payload):
    with socket.create_connection(("127.0.0.1", receiver.port), timeout=2) as client:
        client.sendall(struct.pack(">I", 100) + b"{")
        client.shutdown(socket.SHUT_WR)
        assert _ack(client)["ok"] is False
    assert _request(receiver, _frame(payload))["ok"] is True


def test_bind_failure_is_synchronous_and_recoverable(receiver):
    other = bvh.BvhDescriptionReceiver("127.0.0.1", receiver.port)
    try:
        with pytest.raises(OSError):
            other.start()
        receiver.stop()
        other.start()
    finally:
        other.stop()


def _add_windows_end_sites(payload, prefix):
    for parent_name in ('Head', 'LeftWrist', 'RightWrist', 'LeftToe', 'RightToe'):
        parent = next(j for j in payload['joints'] if j['name'] == parent_name)
        payload['joints'].append(dict(id=len(payload['joints']) + 1,
            name=parent_name + '_EndSite', parent_id=parent['id'], offset=[0., .02, 0.]))
    if prefix:
        for joint in payload['joints']:
            joint['name'] = prefix + joint['name']


@pytest.mark.parametrize('prefix', ['', 'actor_'])
def test_windows_end_sites_preserve_identity_and_calibration(payload, prefix):
    expected, expected_height = bvh.calibration_from_payload(payload)
    _add_windows_end_sites(payload, prefix)
    _, joints = bvh.validate_bvh_payload(payload)
    assert len({j['name'] for j in joints}) == 25
    assert prefix + 'Head_EndSite' in {j['name'] for j in joints}
    actual, height = bvh.calibration_from_payload(payload)
    assert height == expected_height
    for name in expected:
        for lhs, rhs in zip(actual[name], expected[name]):
            np.testing.assert_allclose(lhs, rhs)


def test_duplicate_terminal_wire_names_still_rejected(payload):
    _add_windows_end_sites(payload, '')
    payload['joints'][-1]['name'] = payload['joints'][-2]['name']
    with pytest.raises(bvh.BvhPayloadError, match='duplicate'):
        bvh.validate_bvh_payload(payload)


def test_tcp_accepts_windows_end_sites(receiver, payload):
    _add_windows_end_sites(payload, '')
    response = _request(receiver, _frame(payload))
    assert response['ok'] is True
    assert response['joint_count'] == 25
