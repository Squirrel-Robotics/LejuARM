"""ROS/DDS-free Xsens BVH calibration and length-prefixed JSON transport.

``validate_bvh_payload(payload)`` returns ``(frame_id, normalized_joints)``.
``calibration_from_payload(payload, arm_pose_deg=90)`` returns
``(FrameDict, height_m)``; positions are metres and quaternions are wxyz.
Both raise BvhPayloadError (ValueError) for invalid input, without mutation.
Mapped names may have an ``actor_`` prefix. Unmapped names (including
``Head_EndSite`` and ``LeftToe_EndSite``) retain their full identity. Extra joints take
part in FK but only the 20 mapped joints and two FootMod joints are returned.
Height is Head.z minus the lower ankle.z, required to be in [0.2, 3.0] m.
"""

import errno
import json
import math
import socket
import struct
import threading
from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation


FrameDict = Dict[str, Tuple[np.ndarray, np.ndarray]]
PROTOCOL = "xsens_bvh_desc_v1"
MAX_PAYLOAD_BYTES = 8 * 1024 * 1024
XS_NAME_MAPPING = {
    "Hips": "Hips", "Chest": "Spine", "Neck": "Neck", "Head": "Head",
    "LeftCollar": "LeftShoulder", "LeftShoulder": "LeftArm",
    "LeftElbow": "LeftForeArm", "LeftWrist": "LeftHand",
    "RightCollar": "RightShoulder", "RightShoulder": "RightArm",
    "RightElbow": "RightForeArm", "RightWrist": "RightHand",
    "LeftHip": "LeftUpLeg", "LeftKnee": "LeftLeg",
    "LeftAnkle": "LeftFoot", "LeftToe": "LeftToeBase",
    "RightHip": "RightUpLeg", "RightKnee": "RightLeg",
    "RightAnkle": "RightFoot", "RightToe": "RightToeBase",
}
_CANONICAL = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


class BvhPayloadError(ValueError):
    """Malformed skeleton, framing, or calibration geometry."""


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BvhPayloadError("{} must be a finite number".format(label))
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise BvhPayloadError("{} must be finite".format(label)) from exc
    if not math.isfinite(number):
        raise BvhPayloadError("{} must be finite".format(label))
    return number


def validate_bvh_payload(payload):
    """Validate wire schema/names; return copied, prefix-normalized joints.

    IDs must be consecutive from 1, with parent 0 or an earlier ID, as in
    the publisher protocol. Multiple roots are allowed and zeroed during FK.
    Geometry/height validation is performed by calibration_from_payload.
    """
    if not isinstance(payload, dict):
        raise BvhPayloadError("payload must be an object")
    if payload.get("protocol") != PROTOCOL:
        raise BvhPayloadError("protocol must be " + PROTOCOL)
    if payload.get("unit") != "m":
        raise BvhPayloadError("unit must be m")
    frame_id = payload.get("frame_id")
    if frame_id is None:
        frame_id = "xsense_bvh"
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise BvhPayloadError("frame_id must be a nonempty string")
    joints = payload.get("joints")
    if not isinstance(joints, list) or not joints:
        raise BvhPayloadError("joints must be a nonempty list")
    normalized, names = [], set()
    for expected_id, joint in enumerate(joints, 1):
        if not isinstance(joint, dict):
            raise BvhPayloadError("joint must be an object")
        jid, pid = joint.get("id"), joint.get("parent_id")
        if type(jid) is not int or jid != expected_id:
            raise BvhPayloadError("joint IDs must be consecutive from 1")
        if type(pid) is not int or not 0 <= pid < jid:
            raise BvhPayloadError("parent_id must be 0 or an earlier joint ID")
        name = joint.get("name")
        if not isinstance(name, str) or not name.strip():
            raise BvhPayloadError("joint name must be a nonempty string")
        name = name.strip()
        # Only canonical body names need actor-prefix removal. The Windows
        # sender emits unique terminal names such as Head_EndSite; stripping
        # those prefixes collapses distinct terminal joints to "EndSite".
        candidate = name.split("_", 1)[-1]
        if candidate in XS_NAME_MAPPING:
            name = candidate
        if not name or name in names:
            raise BvhPayloadError("empty or duplicate normalized joint name: " + name)
        names.add(name)
        offset = joint.get("offset")
        if not isinstance(offset, list) or len(offset) != 3:
            raise BvhPayloadError("offset must be a list of three numbers")
        normalized.append({"id": jid, "name": name, "parent_id": pid,
                           "offset": [_number(v, name + " offset") for v in offset]})
    missing = set(XS_NAME_MAPPING) - names
    if missing:
        raise BvhPayloadError("missing required joints: " + ", ".join(sorted(missing)))
    return frame_id.strip(), normalized


def calibration_from_payload(payload, arm_pose_deg=90):
    """BVH FK with left/right shoulder local -/+arm_pose_deg about Z.

    Root offsets are ignored. Apply the canonical matrix to global positions
    and left-compose it onto global orientations. No scale/clamping is used.
    """
    _, joints = validate_bvh_payload(payload)
    angle = _number(arm_pose_deg, "arm_pose_deg")
    canonical = Rotation.from_matrix(_CANONICAL)
    positions, rotations, pose = {}, {}, {}
    with np.errstate(over="ignore", invalid="ignore"):
        for joint in joints:
            jid, pid, name = joint["id"], joint["parent_id"], joint["name"]
            degrees = -angle if name == "LeftShoulder" else (
                angle if name == "RightShoulder" else 0.)
            local = Rotation.from_euler("z", degrees, degrees=True)
            if pid == 0:
                pos, rot = np.zeros(3), local
            else:
                pos = positions[pid] + rotations[pid].apply(joint["offset"])
                rot = rotations[pid] * local
            positions[jid], rotations[jid] = pos, rot
            converted = _CANONICAL @ pos
            if not np.all(np.isfinite(converted)):
                raise BvhPayloadError("nonfinite FK position")
            if name in XS_NAME_MAPPING:
                xyzw = (canonical * rot).as_quat()
                pose[XS_NAME_MAPPING[name]] = (converted, xyzw[[3, 0, 1, 2]])
    for side in ("Left", "Right"):
        pose[side + "FootMod"] = (pose[side + "Foot"][0].copy(),
                                  pose[side + "ToeBase"][1].copy())
    height = float(pose["Head"][0][2] - min(
        pose["LeftFootMod"][0][2], pose["RightFootMod"][0][2]))
    if not math.isfinite(height) or not 0.2 <= height <= 3.0:
        raise BvhPayloadError("human height must be between 0.2 and 3.0 metres")
    return pose, height


def _recv_exact(sock, length):
    chunks = []
    while length:
        chunk = sock.recv(length)
        if not chunk:
            raise BvhPayloadError("connection closed before complete frame")
        chunks.append(chunk)
        length -= len(chunk)
    return b"".join(chunks)


def _read_json(sock):
    size = struct.unpack(">I", _recv_exact(sock, 4))[0]
    if not 0 < size <= MAX_PAYLOAD_BYTES:
        raise BvhPayloadError("payload length must be 1..{} bytes".format(MAX_PAYLOAD_BYTES))
    return json.loads(_recv_exact(sock, size).decode("utf-8"))


def _close(sock):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    sock.close()


class BvhDescriptionReceiver:
    """Background TCP server; one request/ack per connection (retry reconnects).

    start() binds synchronously, then returns without waiting for a skeleton.
    port=0 selects an ephemeral port, available as ``port`` after start().
    Repeated start/stop calls are harmless. stop() closes listener and active
    clients and joins all threads (except itself if called inside callback).
    Call lifecycle methods serially; callbacks must return for stop() to finish.

    The first valid skeleton is latched BEFORE callback(pose, height), which
    runs on a client thread. Later requests get ok=false until stop()/start().
    A callback exception gives an error ack but keeps the latch, preventing
    duplicate side effects. Invalid skeletons never consume the latch.
    Acks use uint32 big-endian UTF-8 JSON byte lengths, as do requests.
    """

    def __init__(self, host="0.0.0.0", port=9764, callback=None, arm_pose_deg=90):
        self.host, self.port = host, port
        self.callback = callback
        self.arm_pose_deg = _number(arm_pose_deg, "arm_pose_deg")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._server = None
        self._thread = None
        self._clients = set()
        self._workers = []
        self._accepted = False
        self._stopping = False

    def start(self):
        with self._lock:
            if self._stopping:
                raise RuntimeError("receiver is still stopping")
            if self._server is not None:
                return
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(8)
                server.settimeout(0.2)
            except OSError as exc:
                server.close()
                if exc.errno == errno.EADDRINUSE:
                    raise OSError(
                        errno.EADDRINUSE,
                        "BVH TCP {}:{} already in use; stop leftover leju_xsens_gmr"
                        .format(self.host, self.port),
                    ) from exc
                raise
            except BaseException:
                server.close()
                raise
            self.port = server.getsockname()[1]
            self._server = server
            self._accepted = False
            self._stop.clear()
            self._thread = threading.Thread(target=self._serve, args=(server,),
                                            name="xsens-bvh-listener", daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            self._stopping = True
            self._stop.set()
            server, listener = self._server, self._thread
            clients, workers = list(self._clients), list(self._workers)
        if server is not None:
            _close(server)
        for client in clients:
            _close(client)
        for thread in [listener] + workers:
            if thread is not None and thread is not threading.current_thread():
                thread.join()
        with self._lock:
            self._server = None
            self._workers = [t for t in workers if t.is_alive()]
            self._stopping = False

    def _serve(self, server):
        while not self._stop.is_set():
            try:
                client, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                if self._stop.is_set():
                    _close(client)
                    break
                client.settimeout(10.)
                self._clients.add(client)
                self._workers = [t for t in self._workers if t.is_alive()]
                worker = threading.Thread(target=self._handle, args=(client,),
                                          name="xsens-bvh-client", daemon=True)
                self._workers.append(worker)
                worker.start()

    def _handle(self, client):
        try:
            try:
                payload = _read_json(client)
                pose, height = calibration_from_payload(payload, self.arm_pose_deg)
                with self._lock:
                    if self._stop.is_set():
                        return
                    if self._accepted:
                        raise BvhPayloadError("skeleton already accepted; restart to recalibrate")
                    self._accepted = True
                if self.callback is not None:
                    self.callback(pose, height)
                ack = {"ok": True, "message": "BVH skeleton received",
                       "joint_count": len(payload["joints"])}
            except Exception as exc:
                ack = {"ok": False, "message": str(exc)}
            data = json.dumps(ack, ensure_ascii=False).encode("utf-8")
            client.sendall(struct.pack(">I", len(data)) + data)
        except OSError:
            pass
        finally:
            with self._lock:
                self._clients.discard(client)
            _close(client)
