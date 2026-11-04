#!/usr/bin/env python3
"""Executed over SSH on the lower host. DDS subscribers only, no writers/RPC."""
import ctypes
from dataclasses import asdict, dataclass, field
import json
import math
import os
import platform
from pathlib import Path
import signal
import socket
import time
import xml.etree.ElementTree as ET


def emit(value):
    print(json.dumps(value, allow_nan=False, separators=(',', ':')), flush=True)


def main():
    # Only this observer's environment changes; never touch robot services/SDK.
    libroot = Path('/home/lab/Lejulab_ARM/src/lejusdk/3rd_party') / platform.machine() / 'cyclonedds-0.10.2'
    os.environ.setdefault('CYCLONEDDS_LIB_DIR', str(libroot / 'lib'))
    os.environ.setdefault('CYCLONEDDS_HOME', str(libroot))
    os.environ.setdefault('CYCLONEDDS_URI', Path('/home/lab/roban2-gamepad/dds-network.xml').read_text())
    root = ET.fromstring(os.environ['CYCLONEDDS_URI'])
    for domain in root:
        shm = next((x for x in domain if x.tag.rsplit('}', 1)[-1] == 'SharedMemory'), None)
        if shm is None:
            shm = ET.SubElement(domain, 'SharedMemory')
        enable = next((x for x in shm if x.tag.rsplit('}', 1)[-1] == 'Enable'), None)
        if enable is None:
            enable = ET.SubElement(shm, 'Enable')
        enable.text = 'false'
    os.environ['CYCLONEDDS_URI'] = ET.tostring(root, encoding='unicode')
    ctypes.CDLL(os.path.join(os.environ['CYCLONEDDS_LIB_DIR'], 'libddsc.so'))
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.annotations import final
    from cyclonedds.idl.types import int32, uint32, uint8, sequence
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    @final
    @dataclass
    class JointState(IdlStruct, typename='leju::msgs::JointState'):
        header_sec: int32 = 0
        header_nanosec: uint32 = 0
        q: sequence[float] = field(default_factory=list)
        v: sequence[float] = field(default_factory=list)
        vd: sequence[float] = field(default_factory=list)
        tau: sequence[float] = field(default_factory=list)

    @final
    @dataclass
    class HandState(IdlStruct, typename='leju::msgs::HandState'):
        header_sec: int32 = 0
        header_nanosec: uint32 = 0
        left_valid: bool = False
        right_valid: bool = False
        left_sample_age_ms: uint32 = 0
        right_sample_age_ms: uint32 = 0
        position: sequence[float] = field(default_factory=list)
        velocity: sequence[float] = field(default_factory=list)
        current: sequence[float] = field(default_factory=list)
        state: sequence[uint8] = field(default_factory=list)

    @final
    @dataclass
    class StringData(IdlStruct, typename='leju::msgs::StringData'):
        header_sec: int32 = 0
        header_nanosec: uint32 = 0
        data: str = ''

    participant = DomainParticipant(0)
    qos = Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(4))
    definitions = [('joint', '/rt/joint_state', JointState),
                   ('hand', '/rt/hand_state', HandState),
                   ('hardware', '/rt/hardware/state', StringData)]
    readers = [(key, topic, cls, DataReader(participant, Topic(participant, topic, cls), qos=qos))
               for key, topic, cls in definitions]
    names = (['leg_%s%d_joint' % (s, i) for s in ('l', 'r') for i in range(1, 7)]
             + ['waist_yaw_joint']
             + ['zarm_%s%d_joint' % (s, i) for s in ('l', 'r') for i in range(1, 5)]
             + ['zhead_1_joint', 'zhead_2_joint'])
    # This is the audited S17 native 23 channel layout; do not apply to ROS's
    # legacy JointState names. Native head link aliases are named as URDF joints.
    period = 1. / float(os.environ.get('ROBAN_JOINT_HZ', '25'))
    last_stamp = {}
    next_beat = 0
    counts = {key: 0 for key, _, _ in definitions}
    rejected = 0
    while True:
        now = time.monotonic()
        for key, topic, cls, reader in readers:
            samples = [s for s in reader.take(N=100) if isinstance(s, cls)]
            counts[key] += len(samples)
            if not samples:
                continue
            sample = samples[-1]
            stamp_ns = sample.header_sec * 1000000000 + sample.header_nanosec
            if not 0 <= sample.header_nanosec < 1000000000 or stamp_ns <= last_stamp.get(key, -1):
                rejected += 1
                continue
            if key == 'joint' and not all(len(getattr(sample, k)) == 23 and all(math.isfinite(x) for x in getattr(sample, k)) for k in ('q', 'v', 'vd', 'tau')):
                rejected += 1
                continue
            if key == 'hand' and not all(math.isfinite(x) for k in ('position', 'velocity', 'current') for x in getattr(sample, k)):
                rejected += 1
                continue
            last_stamp[key] = stamp_ns
            data = asdict(sample)
            if key == 'joint':
                data['names'] = names
                data['mapping'] = 'S17 native 23: left_leg6,right_leg6,waist,left_arm4,right_arm4,head2'
            emit({'kind': key, 'topic': topic, 'stamp': stamp_ns / 1e9,
                  'source_age_ms': (time.time() - stamp_ns / 1e9) * 1000,
                  'data': data})
        if now >= next_beat:
            emit({'kind': 'lower_status', 'data': {'host': socket.gethostname(), 'counts': counts,
                  'rejected': rejected, 'remote_wall': time.time(), 'motion_commands_sent': 0,
                  'control_publishers_created': False, 'rpc_calls': 0}})
            next_beat = now + 1
        time.sleep(period)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: exit(0))
    main()
