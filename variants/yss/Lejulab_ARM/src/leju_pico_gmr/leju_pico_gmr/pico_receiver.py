"""Pico UDP receiver (ROS-free port of kuavo_pico_gmr pico_comm_minimal.py).

Receives protobuf VRData over UDP from the Pico headset, converts bone
matrices from Unity to the robot world frame (algorithm verbatim from the
hardware-proven Kuavo pipeline), and delivers frames/joystick states via
plain Python callbacks.
"""

import logging
import queue
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from leju_pico_gmr.protos import body_tracking_extended_pb2 as proto

try:
    import netifaces
    NETIFACES_AVAILABLE = True
except ImportError:
    NETIFACES_AVAILABLE = False

log = logging.getLogger("leju_pico_gmr.receiver")

# ---------------------------------------------------------------------------
# Bone role definitions (verbatim)
# ---------------------------------------------------------------------------
BODY_TRACKER_ROLES = [
    "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
    "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
    "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND",
    "FOOT_STATE",
]

INDEX_TO_BONE_NAME = {index: name for index, name in enumerate(BODY_TRACKER_ROLES)}
BONE_NAME_TO_INDEX = {name: index for index, name in enumerate(BODY_TRACKER_ROLES)}

LEFT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND"
]]
RIGHT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND"
]]

# Number of pose bones expected in the FullBody stream (FOOT_STATE excluded
# from poses in some firmware; we tolerate >= 24).
NUM_POSE_BONES = 24


def _effective_grip(analog_value: float, digital_pressed: bool) -> float:
    """Merge Pico analog/digital grip reports into the DDS analog field."""
    return max(float(analog_value), 1.0 if digital_pressed else 0.0)


def _quaternion_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    """4x4 homogeneous rotation matrix from xyzw quaternion (tf-compatible)."""
    x, y, z, w = q_xyzw
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    M = np.eye(4)
    M[0, 0] = 1.0 - (yy + zz)
    M[0, 1] = xy - wz
    M[0, 2] = xz + wy
    M[1, 0] = xy + wz
    M[1, 1] = 1.0 - (xx + zz)
    M[1, 2] = yz - wx
    M[2, 0] = xz - wy
    M[2, 1] = yz + wx
    M[2, 2] = 1.0 - (xx + yy)
    return M


def _quaternion_from_matrix(M: np.ndarray) -> np.ndarray:
    """xyzw quaternion from 4x4/3x3 rotation matrix (Shepperd)."""
    m = M[:3, :3]
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def get_localip_and_broadcast_ips() -> List[Tuple[str, str]]:
    """Return list of (local_ip, broadcast_ip) tuples for all interfaces."""
    results: List[Tuple[str, str]] = []
    if not NETIFACES_AVAILABLE:
        log.warning("netifaces not available; IP broadcast disabled")
        return results
    try:
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface)
            for addr in addrs.get(netifaces.AF_INET, []):
                ip = addr.get("addr")
                bcast = addr.get("broadcast")
                if not ip or not bcast or ip.startswith("127."):
                    continue
                results.append((ip, bcast))
    except Exception as e:  # pragma: no cover
        log.error("Error getting local/broadcast IPs: %s", e)
    return results


class RobotInfoBroadcaster:
    """UDP broadcast so the Pico headset can auto-discover this host."""

    def __init__(self, robot_name: str = "ROBAN", broadcast_port: int = 8443,
                 extra: Optional[Dict] = None):
        self.broadcast_port = broadcast_port
        self.running = True
        self.is_paused = False
        self.robot_name = robot_name
        self.extra = dict(extra or {})
        self._ips = get_localip_and_broadcast_ips()
        for local_ip, bcast in self._ips:
            log.info("Will broadcast robot info to %s:%d", bcast, self.broadcast_port)
            log.debug("Robot discovery payload: %s",
                      self._encode_message(local_ip).decode("utf-8"))

    def _encode_message(self, local_ip: str) -> bytes:
        """Encode the wire format expected by the existing Pico application."""
        import json
        data = {
            "robot_name": self.robot_name,
            "robot_ip": local_ip,
            "eef_type": None,
        }
        data.update(self.extra)
        return json.dumps({"data": data}).encode("utf-8")

    def quit(self):
        self.running = False

    def broadcast(self):
        if not self._ips:
            log.warning("No valid IPs found; robot info broadcast disabled")
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            while self.running:
                if self.is_paused:
                    time.sleep(0.5)
                    continue
                for local_ip, bcast in self._ips:
                    try:
                        sock.sendto(self._encode_message(local_ip),
                                    (bcast, self.broadcast_port))
                    except OSError as e:
                        log.debug("broadcast to %s failed: %s", bcast, e)
                time.sleep(1.0)
        finally:
            sock.close()
            log.info("Robot info broadcast thread exited")


class JoyState:
    """Snapshot of Pico controller state (QuestJoysticks-compatible layout)."""

    __slots__ = (
        "left_x", "left_y", "left_trigger", "left_grip",
        "left_first_button_pressed", "left_second_button_pressed",
        "right_x", "right_y", "right_trigger", "right_grip",
        "right_first_button_pressed", "right_second_button_pressed",
        "right_grip_button", "left_grip_button",
    )

    def __init__(self):
        self.left_x = 0.0
        self.left_y = 0.0
        self.left_trigger = 0.0
        self.left_grip = 0.0
        self.left_first_button_pressed = False   # X
        self.left_second_button_pressed = False  # Y
        self.right_x = 0.0
        self.right_y = 0.0
        self.right_trigger = 0.0
        self.right_grip = 0.0
        self.right_first_button_pressed = False   # A
        self.right_second_button_pressed = False  # B
        self.right_grip_button = False
        self.left_grip_button = False


FrameCallback = Callable[[np.ndarray, float], None]   # (world_matrices[N,4,4], capture_time_s)
JoyCallback = Callable[[JoyState], None]


class PicoReceiver:
    """Receives Pico UDP protobuf data and produces world-frame bone matrices.

    Coordinate conversion pipeline is byte-identical to
    kuavo_pico_gmr/pico_comm_minimal.py (Unity -> ROS -> robot URDF frame,
    x-position reversal, y/z rotation reflection, arm roll corrections).
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 12345,
                 enable_ip_broadcast: bool = True,
                 robot_name: str = "ROBAN",
                 broadcast_port: int = 8443,
                 frame_callback: Optional[FrameCallback] = None,
                 joy_callback: Optional[JoyCallback] = None,
                 receive_body: bool = True):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.settimeout(1.0)

        self.running = False
        self.data_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=10)

        self.frame_callback = frame_callback
        self.joy_callback = joy_callback
        self.receive_body = receive_body

        self.cached_matrices = np.tile(np.eye(4), (len(BODY_TRACKER_ROLES), 1, 1))
        self.left_arm_idxs = LEFT_ARM_IDXS
        self.right_arm_idxs = RIGHT_ARM_IDXS

        self.total_packets = 0
        self.total_frames = 0

        self.broadcaster: Optional[RobotInfoBroadcaster] = None
        self._broadcast_thread: Optional[threading.Thread] = None
        if enable_ip_broadcast:
            self.broadcaster = RobotInfoBroadcaster(
                robot_name=robot_name, broadcast_port=broadcast_port)
            self._broadcast_thread = threading.Thread(
                target=self.broadcaster.broadcast, daemon=True)
            self._broadcast_thread.start()
            log.info("IP broadcast enabled")

        self._recv_thread: Optional[threading.Thread] = None
        self._proc_thread: Optional[threading.Thread] = None
        log.info("PicoReceiver initialized on %s:%d", host, port)

    # ------------------------------------------------------------------
    # Protobuf parsing
    # ------------------------------------------------------------------
    def _parse_full_body_to_matrix(self, msg: "proto.VRData") -> Optional[np.ndarray]:
        poses = msg.full_body.full_body
        if len(poses) < NUM_POSE_BONES:
            log.warning("Ignoring incomplete Pico body frame: got %d poses, need at least %d",
                        len(poses), NUM_POSE_BONES)
            return None
        for i, pose in enumerate(poses):
            if i >= len(self.cached_matrices):
                break
            translation = np.array([pose.pos_x, pose.pos_y, pose.pos_z])
            rotation = np.array([pose.rot_qx, pose.rot_qy, pose.rot_qz, pose.rot_qw])
            self.cached_matrices[i][:3, :3] = _quaternion_matrix(rotation)[:3, :3]
            self.cached_matrices[i][:3, 3] = translation
        return self.cached_matrices

    @staticmethod
    def _parse_controller_state(state) -> dict:
        thumbstick = state.thumbstick
        thumb_x = float(thumbstick[0]) if len(thumbstick) > 0 else 0.0
        thumb_y = float(thumbstick[1]) if len(thumbstick) > 1 else 0.0
        return {
            "buttons": [
                bool(state.primary_button),
                bool(state.secondary_button),
                bool(state.grip_button),
                bool(state.trigger_button),
            ],
            "axes": [
                thumb_x,
                thumb_y,
                float(state.grip_value),
                float(state.trigger_value),
            ],
        }

    def _process_controller_data(self, controller_data):
        if not controller_data.HasField("controllers"):
            return
        states = controller_data.controllers
        left = (self._parse_controller_state(states.left)
                if states.HasField("left")
                else {"buttons": [False] * 4, "axes": [0.0] * 4})
        right = (self._parse_controller_state(states.right)
                 if states.HasField("right")
                 else {"buttons": [False] * 4, "axes": [0.0] * 4})

        joy = JoyState()
        joy.left_x = left["axes"][0]
        joy.left_y = left["axes"][1]
        # Some Pico firmware reports the digital grip button without updating
        # the analog grip value. Preserve the working VMP semantics for combos.
        joy.left_grip = _effective_grip(
            left["axes"][2], left["buttons"][2])
        joy.left_trigger = left["axes"][3]
        joy.left_first_button_pressed = left["buttons"][0]   # X
        joy.left_second_button_pressed = left["buttons"][1]  # Y
        joy.left_grip_button = left["buttons"][2]

        joy.right_x = right["axes"][0]
        joy.right_y = right["axes"][1]
        joy.right_grip = _effective_grip(
            right["axes"][2], right["buttons"][2])
        joy.right_trigger = right["axes"][3]
        joy.right_first_button_pressed = right["buttons"][0]   # A
        joy.right_second_button_pressed = right["buttons"][1]  # B
        joy.right_grip_button = right["buttons"][2]

        if self.joy_callback:
            self.joy_callback(joy)

    # ------------------------------------------------------------------
    # Unity -> world coordinate conversion (verbatim algorithms)
    # ------------------------------------------------------------------
    @staticmethod
    def _homogeneous_matrix_roll(angle_degrees: float) -> np.ndarray:
        angle_radians = np.radians(angle_degrees)
        c = np.cos(angle_radians)
        s = np.sin(angle_radians)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1],
        ])

    @staticmethod
    def _reverse_y_z_rotation(ros_matrices: np.ndarray) -> np.ndarray:
        result = ros_matrices.copy()
        reflection_matrix = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ])
        rotation_matrices = ros_matrices[:, :3, :3]
        result[:, :3, :3] = np.einsum(
            "ij,njk,kl->nil", reflection_matrix, rotation_matrices,
            reflection_matrix.T)
        return result

    def transform_matrix_to_world(self, matrix: np.ndarray) -> np.ndarray:
        """Transform Unity coordinate system matrices to robot world frame."""
        T_unity_2_ros = np.array([[0, 0, -1, 0],
                                  [-1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])
        T_ros_2_robot_urdf = np.array([[0, -1, 0, 0],
                                       [0, 0, 1, 0],
                                       [-1, 0, 0, 0],
                                       [0, 0, 0, 1]])

        T_LEFT_ARM_CORRECTION = self._homogeneous_matrix_roll(90)
        T_RIGHT_ARM_CORRECTION = self._homogeneous_matrix_roll(-90)

        ros_matrices = np.matmul(T_unity_2_ros, matrix)
        ros_matrices = np.matmul(ros_matrices, T_ros_2_robot_urdf)

        # Reverse x position
        ros_matrices[:, 0, 3] = -ros_matrices[:, 0, 3]

        # Reverse y and z rotation via reflection
        ros_matrices = self._reverse_y_z_rotation(ros_matrices)

        # Align arms to robot urdf
        for idx in self.left_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_LEFT_ARM_CORRECTION
        for idx in self.right_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_RIGHT_ARM_CORRECTION
        return ros_matrices

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------
    def _recv_loop(self):
        log.info("UDP receive thread started")
        while self.running:
            try:
                data, _addr = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.total_packets += 1
            try:
                self.data_queue.put_nowait(data)
            except queue.Full:
                try:
                    self.data_queue.get_nowait()
                    self.data_queue.put_nowait(data)
                except (queue.Empty, queue.Full):
                    pass
        log.info("UDP receive thread exited")

    def _proc_loop(self):
        log.info("Data processing thread started")
        while self.running:
            try:
                data = self.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._handle_packet(data)
            except Exception as e:
                log.error("Error handling Pico packet: %s", e, exc_info=True)
        log.info("Data processing thread exited")

    def _handle_packet(self, data: bytes):
        msg = proto.VRData()
        msg.ParseFromString(data)

        if msg.HasField("controller"):
            self._process_controller_data(msg.controller)

        if not self.receive_body or not msg.HasField("full_body"):
            return

        matrices = self._parse_full_body_to_matrix(msg)
        if matrices is None:
            return

        # capture timestamp from protobuf header (fallback: local time)
        capture_time = time.time()
        try:
            ts = msg.header.timestamp
            if ts.seconds > 0:
                capture_time = ts.seconds + ts.nanos * 1e-9
        except Exception:
            pass

        world = self.transform_matrix_to_world(matrices.copy())
        self.total_frames += 1
        if self.frame_callback:
            self.frame_callback(world, capture_time)

    # ------------------------------------------------------------------
    def start(self):
        if self.running:
            return
        self.running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._proc_thread = threading.Thread(target=self._proc_loop, daemon=True)
        self._recv_thread.start()
        self._proc_thread.start()
        log.info("PicoReceiver started")

    def stop(self):
        self.running = False
        if self.broadcaster:
            self.broadcaster.quit()
        try:
            self.socket.close()
        except OSError:
            pass
        log.info("PicoReceiver stopped")
