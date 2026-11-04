"""ROS-free MVN Quaternion/Scale UDP input, requiring only NumPy.

Port of kuavo_xsense_gmr/src/xsens_udp/{parser,fragment_assembler,
elbow_landmark_cache,xsens_pose_builder}.cpp and xsense_udp_receiver_node.cpp;
GMR names/canonicalization follow scripts/xsense_streaming_node.py.

Wire floats/quaternions are big-endian xyz/wxyz. Parsed data is in BVH axes:
B = [[0,1,0],[0,0,1],[1,0,0]], p_bvh=B p_raw, R_bvh=B R_raw B.T.
Output is {GMR_name: (float64 xyz[3], float64 wxyz[4])}, with p=C p_bvh
and R=C R_bvh (LEFT multiplication, C=B.T). Elbow positions use upper-arm
landmarks; elbow rotations remain forearm rotations. No Scale means no poses.

Deliberate differences from C++:
* Only types 02 and 13 are supported. Invalid input raises PacketError; the
  live receiver drops it. All floats must be finite and zero quaternions are
  rejected (the C++ matrix round trip can accidentally repair a zero input).
* Pending fragments have both idle and absolute timeouts, a key limit and a
  65535-byte per-key budget. Scale candidates expire after one second; valid
  committed calibration persists until reset (at most 256 character IDs).
  Malformed Scale cancels that character's candidate, never its committed pair.
* FIFO uses monotonic time and expires individual frames, including prefill and
  held frames, after stale_timeout. New arrivals cannot prolong an old held
  frame's life. C++ checks last-input age and can drain stale queued frames.
* capture_time is local Unix arrival time of the datagram completing the pose,
  never MVN time_code or publication time. Holds keep this original timestamp.
  total_frames counts accepted complete poses, excluding repeat publications.
* No ROS topics, pause controls, DDS, or timestamp rewriting. Callback failures
  are logged and isolated. Callbacks run serially on a publication thread and
  should return promptly. Each invocation gets its own writable array copies.

Pure APIs: XsensPacketParser.parse_header/parse_packet(bytes),
FragmentAssembler.add_datagram(bytes, now), ElbowLandmarkCache.consume(packet,
now)/get_offsets(character_id), XsensPoseBuilder.build(packet, offsets),
XsensFrameDecoder.feed(bytes, now), and PoseFIFO.push/pop(...).
Explicit ``now`` values are monotonic seconds, enabling deterministic tests.
"""

from collections import OrderedDict, deque
from dataclasses import dataclass, field
import logging
import math
import socket
import struct
import threading
import time
from typing import Dict, Tuple

import numpy as np

log = logging.getLogger(__name__)
FrameDict = Dict[str, Tuple[np.ndarray, np.ndarray]]
B = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
C = B.T.copy()
B.setflags(write=False)
C.setflags(write=False)
_HEADER = struct.Struct('>6sIBBIBBBBHH')
_POSE = struct.Struct('>I7f')
# GMR name, orientation/origin segment ID, optional landmark parent segment ID.
BODY_MAPPING = (
    ('Hips', 1, None), ('Spine', 2, None), ('Neck', 6, None), ('Head', 7, None),
    ('LeftShoulder', 12, None), ('LeftArm', 13, None),
    ('LeftForeArm', 14, 13), ('LeftHand', 15, None),
    ('RightShoulder', 8, None), ('RightArm', 9, None),
    ('RightForeArm', 10, 9), ('RightHand', 11, None),
    ('LeftUpLeg', 20, None), ('LeftLeg', 21, None),
    ('LeftFoot', 22, None), ('LeftToeBase', 23, None),
    ('RightUpLeg', 16, None), ('RightLeg', 17, None),
    ('RightFoot', 18, None), ('RightToeBase', 19, None),
)
_TARGETS = {9: 'pRightArmLatEpicondyle', 13: 'pLeftArmLatEpicondyle'}


class PacketError(ValueError):
    """Malformed/unsupported MVN packet or incomplete/invalid pose."""


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')


def _integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')


def _finite(values):
    value = np.asarray(values, dtype=np.float64)
    if not np.isfinite(value).all():
        raise PacketError('non-finite data')
    return value


def quaternion_matrix(q):
    """Rotation matrix from finite nonzero wxyz; normalizes the input."""
    q = _finite(q)
    if q.shape != (4,):
        raise PacketError('quaternion must have four components')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm == 0:
        raise PacketError('zero or invalid quaternion norm')
    w, x, y, z = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def matrix_quaternion(r):
    """Unit wxyz, using the same trace/largest-diagonal branches as C++."""
    r = _finite(r)
    q = np.empty(4)
    trace = np.trace(r)
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q[:] = [s/4, (r[2, 1]-r[1, 2])/s,
                (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * np.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
        q[0] = (r[k, j]-r[j, k])/s
        q[i+1] = s/4
        q[j+1] = (r[i, j]+r[j, i])/s
        q[k+1] = (r[i, k]+r[k, i])/s
    return q / np.linalg.norm(q)


@dataclass(frozen=True)
class PacketHeader:
    message_type: str
    sample_counter: int
    datagram_counter: int
    number_of_items: int
    time_code: int
    character_id: int
    body_segments: int
    props: int
    finger_tracking_segments: int
    reserved: int
    payload_size: int

    @property
    def fragment_index(self):
        return self.datagram_counter & 0x7f

    @property
    def is_last_fragment(self):
        return bool(self.datagram_counter & 0x80)


@dataclass(frozen=True)
class SegmentPose:
    segment_id: int
    position_m: np.ndarray
    orientation: np.ndarray


@dataclass(frozen=True)
class ScaleSegment:
    name: str
    origin_m: np.ndarray


@dataclass(frozen=True)
class ScalePoint:
    segment_id: int
    point_id: int
    name: str
    flags: int
    position_m: np.ndarray


@dataclass(frozen=True)
class ParsedPacket:
    header: PacketHeader
    pose_quaternion: tuple = ()
    scale_segments: tuple = ()
    scale_points: tuple = ()


class _Reader:
    def __init__(self, data):
        self.data, self.offset = data, 24

    def read(self, fmt):
        size = struct.calcsize(fmt)
        if len(self.data) - self.offset < size:
            raise PacketError('record exceeds payload')
        result = struct.unpack_from(fmt, self.data, self.offset)
        self.offset += size
        return result

    def string(self):
        length, = self.read('>i')  # signed int32, NOT uint16/UTF length
        if length < 0 or length > len(self.data) - self.offset:
            raise PacketError('invalid string length')
        result = self.data[self.offset:self.offset+length]
        self.offset += length
        # C++ std::string preserves arbitrary bytes; Latin-1 is lossless.
        return result.decode('latin1')

    def vector(self):
        return B @ _finite(self.read('>3f'))


class XsensPacketParser:
    @staticmethod
    def parse_header(datagram):
        if len(datagram) < 24:
            raise PacketError('datagram shorter than 24-byte header')
        magic, *fields = _HEADER.unpack_from(datagram)
        if magic[:4] != b'MXTP' or not all(48 <= c <= 57 for c in magic[4:]):
            raise PacketError('invalid MXTP ID or ASCII message type')
        header = PacketHeader(magic[4:].decode('ascii'), *fields)
        if len(datagram) != 24 + header.payload_size:
            raise PacketError('payload size does not match datagram size')
        return header

    @classmethod
    def parse_packet(cls, datagram):
        h = cls.parse_header(datagram)
        reader = _Reader(datagram)
        if h.message_type == '02':
            if h.payload_size % 32 or h.number_of_items != h.payload_size // 32:
                raise PacketError('pose record size/item count mismatch')
            poses = []
            for _ in range(h.number_of_items):
                sid, *values = reader.read('>I7f')
                values = _finite(values)
                r = B @ quaternion_matrix(values[3:]) @ B.T
                poses.append(SegmentPose(sid, B @ values[:3], matrix_quaternion(r)))
            return ParsedPacket(h, pose_quaternion=tuple(poses))
        if h.message_type != '13':
            raise PacketError('unsupported Xsens message type')
        segments = []
        count, = reader.read('>I')
        if count > (len(datagram)-reader.offset) // 16:
            raise PacketError('scale segment count exceeds payload')
        for _ in range(count):
            segments.append(ScaleSegment(reader.string(), reader.vector()))
        count, = reader.read('>I')  # mandatory even when zero
        if count > (len(datagram)-reader.offset) // 24:
            raise PacketError('scale point count exceeds payload')
        points = []
        for _ in range(count):
            sid, pid = reader.read('>HH')
            name = reader.string()
            flags, = reader.read('>I')
            points.append(ScalePoint(sid, pid, name, flags, reader.vector()))
        if reader.offset != len(datagram):
            raise PacketError('scale payload has trailing bytes')
        return ParsedPacket(h, scale_segments=tuple(segments), scale_points=tuple(points))


@dataclass
class _Fragments:
    first: bytes
    created: float
    updated: float
    parts: dict = field(default_factory=dict)
    last: object = None


class FragmentAssembler:
    """Ordinary byte fragments keyed by (character, type, sample); Scale bypasses."""
    def __init__(self, timeout=1.0, max_pending=64):
        _positive(timeout, 'timeout')
        _integer(max_pending, 1, 65536, 'max_pending')
        self.timeout, self.max_pending = timeout, max_pending
        self.pending = OrderedDict()

    def cleanup(self, now):
        for key, p in list(self.pending.items()):
            if now-p.updated > self.timeout or now-p.created > self.timeout:
                del self.pending[key]

    def add_datagram(self, datagram, now):
        self.cleanup(now)
        h = XsensPacketParser.parse_header(datagram)
        if h.message_type == '13' or h.datagram_counter == 0x80:
            return datagram
        key = (h.character_id, h.message_type, h.sample_counter)
        if key not in self.pending:
            if len(self.pending) >= self.max_pending:
                self.pending.popitem(last=False)
            self.pending[key] = _Fragments(bytes(datagram[:24]), now, now)
        p = self.pending[key]
        p.updated = now
        self.pending.move_to_end(key)
        if h.fragment_index == 0:
            p.first = bytes(datagram[:24])
        p.parts[h.fragment_index] = (bytes(datagram[24:]), h.number_of_items)
        if h.is_last_fragment:
            p.last = h.fragment_index
        if sum(len(part[0]) for part in p.parts.values()) > 65535:
            del self.pending[key]
            raise PacketError('reconstructed payload exceeds uint16')
        if p.last is None or any(i not in p.parts for i in range(p.last+1)):
            return None
        del self.pending[key]
        items = sum(p.parts[i][1] for i in range(p.last+1))
        if items > 255:
            raise PacketError('reconstructed item count exceeds uint8')
        payload = b''.join(p.parts[i][0] for i in range(p.last+1))
        header = bytearray(p.first)
        header[10:12] = bytes((0x80, items))
        struct.pack_into('>H', header, 22, len(payload))
        return bytes(header) + payload


@dataclass
class _Candidate:
    created: float
    points: dict = field(default_factory=dict)
    valid: bool = True


class ElbowLandmarkCache:
    """Atomic per-character BVH offsets {9: right_xyz, 13: left_xyz}."""
    def __init__(self, timeout=1.0):
        _positive(timeout, 'timeout')
        self.timeout = timeout
        self.candidates = {}
        self._committed = {}

    def cleanup(self, now):
        for char, candidate in list(self.candidates.items()):
            if now - candidate.created > self.timeout:
                del self.candidates[char]

    def invalidate(self, character_id):
        self.candidates.pop(character_id, None)

    def get_offsets(self, character_id):
        offsets = self._committed.get(character_id)
        return None if offsets is None else {k: v.copy() for k, v in offsets.items()}

    def consume(self, packet, now):
        self.cleanup(now)
        h = packet.header
        if h.message_type != '13':
            return 'ignored'
        char = h.character_id
        if packet.scale_segments:
            self.invalidate(char)
            if h.datagram_counter != 0:
                return 'rejected'
            self.candidates[char] = _Candidate(now)
        candidate = self.candidates.get(char)
        if candidate is None:
            return 'ignored'
        for point in packet.scale_points:
            sid = point.segment_id
            id_match = sid in _TARGETS and point.point_id == 3
            name_match = point.name in _TARGETS.values()
            if not id_match and not name_match:
                continue
            if not id_match or point.name != _TARGETS[sid] or not np.isfinite(point.position_m).all():
                candidate.valid = False
                continue
            previous = candidate.points.get(sid)
            if previous is not None and not np.all(np.abs(previous-point.position_m) <= 1e-6):
                candidate.valid = False
            candidate.points[sid] = point.position_m.copy()
        if not h.is_last_fragment:
            return 'collecting'
        del self.candidates[char]
        if not candidate.valid or set(candidate.points) != set(_TARGETS):
            return 'rejected'
        old = self._committed.get(char)
        if old is not None and all(np.all(np.abs(old[k]-candidate.points[k]) <= 1e-6) for k in old):
            return 'unchanged'
        self._committed[char] = candidate.points
        return 'ready' if old is None else 'updated'


class XsensPoseBuilder:
    @staticmethod
    def build(packet, offsets):
        if packet.header.message_type != '02':
            raise PacketError('expected quaternion pose')
        if offsets is None or not all(k in offsets for k in _TARGETS):
            raise PacketError('both elbow Scale offsets are required')
        # C++ maps ID 0 to Pelvis too, with the last occurrence winning.
        segments = {p.segment_id or 1: p for p in packet.pose_quaternion}
        frame = {}
        for name, sid, parent in BODY_MAPPING:
            if sid not in segments:
                raise PacketError(f'missing Xsens segment {sid} ({name})')
            source = segments[sid]
            position = _finite(source.position_m)
            rotation = quaternion_matrix(source.orientation)
            if parent is not None:
                upper = segments[parent]
                position = (_finite(upper.position_m) +
                            quaternion_matrix(upper.orientation) @ _finite(offsets[parent]))
            frame[name] = (_finite(C @ position), matrix_quaternion(C @ rotation))
        return frame


class XsensFrameDecoder:
    """Single-threaded pure pipeline; feed returns a complete GMR dict or None.

    -1 accepts all characters; each character keeps separate Scale/fragments.
    State is bounded by 64 pending packets and 256 character calibrations.
    """
    def __init__(self, expected_character_id=0):
        _integer(expected_character_id, -1, 255, 'expected_character_id')
        self.expected_character_id = expected_character_id
        self.assembler = FragmentAssembler()
        self.scale_cache = ElbowLandmarkCache()
        self._last_scale_warning = -math.inf

    def cleanup(self, now):
        self.assembler.cleanup(now)
        self.scale_cache.cleanup(now)

    def feed(self, datagram, now):
        self.cleanup(now)
        try:
            h = XsensPacketParser.parse_header(datagram)
        except PacketError:
            # A damaged Scale length/header must not let a partial batch commit.
            if len(datagram) >= 17 and datagram[:6] == b'MXTP13':
                self.scale_cache.invalidate(datagram[16])
            raise
        if self.expected_character_id >= 0 and h.character_id != self.expected_character_id:
            return None
        if h.message_type not in ('02', '13'):
            return None
        try:
            complete = self.assembler.add_datagram(datagram, now)
            if complete is None:
                return None
            packet = XsensPacketParser.parse_packet(complete)
        except PacketError:
            if h.message_type == '13':
                self.scale_cache.invalidate(h.character_id)
            raise
        if h.message_type == '13':
            event = self.scale_cache.consume(packet, now)
            if event in ('ready', 'updated'):
                log.info('Xsens character %d elbow Scale %s', h.character_id, event)
            elif event == 'rejected':
                log.warning('Rejected Xsens character %d Scale batch; keeping prior calibration',
                            h.character_id)
            return None
        offsets = self.scale_cache.get_offsets(h.character_id)
        if offsets is None:
            if now - self._last_scale_warning >= 2.0:
                log.warning('Waiting for complete left/right elbow Scale for character %d; '
                            'enable MVN Quaternion and Scale together', h.character_id)
                self._last_scale_warning = now
            return None
        return XsensPoseBuilder.build(packet, offsets)


@dataclass(frozen=True)
class CapturedFrame:
    frame_gmr: FrameDict
    capture_time: float
    arrival_monotonic: float


class PoseFIFO:
    """Pure FIFO/prefill/hold policy. External locking required across threads."""
    def __init__(self, queue_target_depth=10, queue_max_size=15, stale_timeout=0.2):
        _integer(queue_max_size, 1, 65536, 'queue_max_size')
        _integer(queue_target_depth, 1, queue_max_size, 'queue_target_depth')
        _positive(stale_timeout, 'stale_timeout')
        self.target = queue_target_depth
        self.queue = deque(maxlen=queue_max_size)
        self.stale_timeout = stale_timeout
        self.prefill_ready = False
        self.last = None

    def push(self, frame_gmr, capture_time, arrival_monotonic):
        self.queue.append(CapturedFrame(frame_gmr, capture_time, arrival_monotonic))

    def pop(self, now):
        while self.queue and now-self.queue[0].arrival_monotonic > self.stale_timeout:
            self.queue.popleft()
        if self.last is not None and now-self.last.arrival_monotonic > self.stale_timeout:
            self.last = None
        if not self.prefill_ready and len(self.queue) >= self.target:
            self.prefill_ready = True
        if self.prefill_ready:
            if self.queue:
                self.last = self.queue.popleft()
                return self.last
            self.prefill_ready = False
        return self.last


class XsensReceiver:
    """UDP receiver; start() binds synchronously, stop() closes and joins workers.

    Constructor: host='0.0.0.0', port=9763, frame_callback=None,
    expected_character_id=0, publish_hz=120, queue_target_depth=10,
    queue_max_size=15, stale_timeout=0.2. Port 0 requests an ephemeral port;
    ``port`` holds the actual bound port after start(). start/stop are idempotent
    and restart clears Scale/fragments/FIFO, retaining lifetime total_frames.
    Callback signature: callback(frame_gmr, original_capture_time).
    """
    def __init__(self, host='0.0.0.0', port=9763, frame_callback=None,
                 expected_character_id=0, publish_hz=120, queue_target_depth=10,
                 queue_max_size=15, stale_timeout=0.2):
        _integer(port, 0, 65535, 'port')
        _positive(publish_hz, 'publish_hz')
        if frame_callback is not None and not callable(frame_callback):
            raise ValueError('frame_callback must be callable or None')
        self.host, self.port = host, port
        self._requested_port = port
        self.frame_callback = frame_callback
        self.expected_character_id = expected_character_id
        self.publish_hz = publish_hz
        self._queue_args = (queue_target_depth, queue_max_size, stale_timeout)
        self._decoder = XsensFrameDecoder(expected_character_id)
        self._fifo = PoseFIFO(*self._queue_args)
        self.total_frames = 0
        self.running = False
        self.socket = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._threads = []

    def start(self):
        if self.running:
            return
        if any(t.is_alive() for t in self._threads):
            raise RuntimeError('previous receiver workers have not stopped')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.host, self._requested_port))
            sock.settimeout(0.05)
        except BaseException:
            sock.close()
            raise
        self.socket = sock
        self.port = sock.getsockname()[1]
        self._decoder = XsensFrameDecoder(self.expected_character_id)
        self._fifo = PoseFIFO(*self._queue_args)
        self._stop.clear()
        self.running = True
        self._threads = [threading.Thread(target=target, name=name, daemon=True)
                         for target, name in ((self._receive, 'xsens-udp'),
                                              (self._publish, 'xsens-publish'))]
        for thread in self._threads:
            thread.start()
        log.info('Xsens UDP listening on %s:%d, FIFO %.1f Hz depth=%d/%d',
                 self.host, self.port, self.publish_hz, self._queue_args[0], self._queue_args[1])

    def stop(self):
        self.running = False
        self._stop.set()
        if self.socket is not None:
            self.socket.close()
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join()

    def _receive(self):
        while not self._stop.is_set():
            try:
                data, _ = self.socket.recvfrom(65535)
            except socket.timeout:
                self._decoder.cleanup(time.monotonic())
                continue
            except OSError:
                self.running = False
                self._stop.set()
                break
            arrival = time.monotonic()
            capture_time = time.time()
            try:
                frame = self._decoder.feed(data, arrival)
            except PacketError as exc:
                log.debug('Dropping Xsens packet: %s', exc)
                continue
            if frame is not None:
                with self._lock:
                    self._fifo.push(frame, capture_time, arrival)
                    self.total_frames += 1

    def _publish(self):
        period = 1.0 / self.publish_hz
        deadline = time.monotonic() + period
        while not self._stop.wait(max(0., deadline-time.monotonic())):
            now = time.monotonic()
            with self._lock:
                captured = self._fifo.pop(now)
            if captured is not None and self.frame_callback is not None:
                frame = {name: (p.copy(), q.copy())
                         for name, (p, q) in captured.frame_gmr.items()}
                try:
                    self.frame_callback(frame, captured.capture_time)
                except Exception:
                    log.exception('Xsens frame callback failed')
            # Keep phase without catch-up bursts after a slow callback.
            now = time.monotonic()
            deadline += max(1, math.floor((now-deadline)/period)+1) * period
