"""Standalone sender tests; no ROS, numpy, or package import helpers required."""

import importlib.util
import json
from pathlib import Path
import socket
import struct

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "send_xsens_bvh.py"
SPEC = importlib.util.spec_from_file_location("xsens_bvh_sender", SCRIPT)
sender = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sender)

BVH = """HIERARCHY
ROOT Full Body:骨盆 {
 OFFSET 100 -200 3e1
 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
 JOINT Left Arm {
  OFFSET 0 50 0
  CHANNELS 3 Zrotation Xrotation Yrotation
  JOINT Hand{OFFSET 0 20 0 CHANNELS 0 End Site{OFFSET 0 5 0}}
 }
 JOINT RightArm { OFFSET 0 -50 0 CHANNELS 3 Xrotation Yrotation Zrotation }
}
MOTION
Frames: nonsense deliberately ignored
Frame Time: not a number
{ garbage samples
"""


def test_hierarchy_preorder_names_end_sites_and_default_scaling():
    payload = sender.parse_bvh(BVH)
    assert payload["protocol"] == "xsens_bvh_desc_v1"
    assert payload["unit"] == "m"
    assert payload["joints"] == [
        {"id": 1, "name": "Full Body:骨盆", "parent_id": 0, "offset": [1, -2, .3]},
        {"id": 2, "name": "Left Arm", "parent_id": 1, "offset": [0, .5, 0]},
        {"id": 3, "name": "Hand", "parent_id": 2, "offset": [0, .2, 0]},
        {"id": 4, "name": "RightArm", "parent_id": 1, "offset": [0, -.5, 0]},
    ]


@pytest.mark.parametrize("scale", [1, .001, 2])
def test_configurable_scaling(scale):
    joints = sender.parse_bvh(BVH, scale)["joints"]
    assert joints[0]["offset"] == pytest.approx([100 * scale, -200 * scale, 30 * scale])


@pytest.mark.parametrize("scale", [0, -1, float("nan"), float("inf")])
def test_invalid_scale(scale):
    with pytest.raises(ValueError):
        sender.parse_bvh(BVH, scale)


@pytest.mark.parametrize("text", [
    "", "ROOT Hips {}", "HIERARCHY JOINT Hips {}",
    "HIERARCHY ROOT {OFFSET 0 0 0 CHANNELS 0}",
    "HIERARCHY ROOT Hips {CHANNELS 0}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 CHANNELS 0}",
    "HIERARCHY ROOT Hips {OFFSET nan 0 0 CHANNELS 0}",
    "HIERARCHY ROOT Hips {OFFSET 1e309 0 0 CHANNELS 0}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 OFFSET 0 0 0 CHANNELS 0}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS -1}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS 1.0 Xrotation}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS 2 Xrotation}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS 1 Xrotation Yrotation}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS 1 Wrotation}",
    "HIERARCHY ROOT Hips {OFFSET 0 0 0 CHANNELS 2 Xrotation Xrotation}",
    BVH.replace("OFFSET 0 50 0", ""),
    BVH.replace("JOINT RightArm", "JOINT Hand"),
    BVH.replace("End Site{OFFSET 0 5 0}", "End Site{OFFSET 0 5}"),
    BVH.replace("}\nMOTION", "MOTION"),
    BVH.replace("MOTION", "unexpected"),
    BVH.replace("MOTION", "ROOT Second {} MOTION"),
])
def test_malformed_hierarchy(text):
    with pytest.raises(sender.BvhError):
        sender.parse_bvh(text)


def test_file_bom_crlf_and_read_only(tmp_path):
    path = tmp_path / "actor.bvh"
    original = ("\ufeff" + BVH.replace("\n", "\r\n")).encode("utf-8")
    path.write_bytes(original)
    assert sender.load_bvh(path) == sender.parse_bvh(BVH)
    assert path.read_bytes() == original
    assert sender.parse_bvh(BVH.split("MOTION")[0]) == sender.parse_bvh(BVH)


def frame(value):
    raw = json.dumps(value, ensure_ascii=False).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw


class FakeSocket:
    def __init__(self, response):
        self.response = response
        self.sent = b""
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def sendall(self, data):
        self.sent += data

    def recv(self, size):
        # Fragment even the four-byte header to exercise recv_exact.
        result = self.response[:min(size, 1)]
        self.response = self.response[len(result):]
        return result


def connect_fake(monkeypatch, response):
    sock = FakeSocket(response)
    calls = []

    def connect(address, timeout):
        calls.append((address, timeout))
        return sock

    monkeypatch.setattr(sender.socket, "create_connection", connect)
    return sock, calls


def test_tcp_framing_utf8_and_fragmented_ack(monkeypatch):
    ack = {"ok": True, "message": "已接收", "joint_count": 4}
    sock, calls = connect_fake(monkeypatch, frame(ack))
    payload = sender.parse_bvh(BVH)
    assert sender.send_payload(payload, "robot", 9764, 2) == ack
    assert calls == [(("robot", 9764), 2)]
    assert struct.unpack(">I", sock.sent[:4])[0] == len(sock.sent[4:])
    assert json.loads(sock.sent[4:].decode("utf-8")) == payload
    assert "骨盆".encode("utf-8") in sock.sent
    assert sock.closed


@pytest.mark.parametrize("response", [
    b"", b"\x00\x00", struct.pack(">I", 10) + b"{}",
    struct.pack(">I", 0), struct.pack(">I", sender.MAX_FRAME_BYTES + 1),
    struct.pack(">I", 1) + b"\xff", struct.pack(">I", 1) + b"{",
    frame({"ok": False, "message": "bad skeleton"}), frame({}),
    frame({"ok": 1}), frame({"ok": "true"}), frame([]),
])
def test_invalid_ack(monkeypatch, response):
    sock, _ = connect_fake(monkeypatch, response)
    with pytest.raises(sender.BvhError):
        sender.send_payload(sender.parse_bvh(BVH), "robot")
    assert sock.closed


def test_cli_defaults_and_meter_option(monkeypatch, tmp_path, capsys):
    path = tmp_path / "actor.bvh"
    path.write_text(BVH, encoding="utf-8")
    for option, expected in [([], 1), (["--offset-scale", "1"], 100)]:
        sock, calls = connect_fake(monkeypatch, frame({"ok": True}))
        assert sender.main([str(path), "--host", "robot"] + option) == 0
        assert json.loads(sock.sent[4:])["joints"][0]["offset"][0] == expected
        assert calls == [(("robot", 9764), 10)]
        assert "Sent 4 joints" in capsys.readouterr().out


def test_cli_errors(monkeypatch, tmp_path, capsys):
    path = tmp_path / "actor.bvh"
    assert sender.main([str(path), "--host", "robot"]) == 1
    path.write_text("bad hierarchy", encoding="utf-8")
    assert sender.main([str(path), "--host", "robot"]) == 1
    path.write_text(BVH, encoding="utf-8")
    connect_fake(monkeypatch, frame({"ok": False, "message": "denied"}))
    assert sender.main([str(path), "--host", "robot"]) == 1
    assert "denied" in capsys.readouterr().err

    def fail(*args, **kwargs):
        raise socket.timeout("timed out")

    monkeypatch.setattr(sender.socket, "create_connection", fail)
    assert sender.main([str(path), "--host", "robot"]) == 1
    assert "timed out" in capsys.readouterr().err


@pytest.mark.parametrize("option", [
    ["--port", "0"], ["--port", "65536"], ["--offset-scale", "nan"],
    ["--offset-scale", "0"], ["--timeout", "-1"],
])
def test_cli_invalid_arguments(option):
    with pytest.raises(SystemExit) as exc:
        sender.main(["actor.bvh", "--host", "robot"] + option)
    assert exc.value.code != 0
