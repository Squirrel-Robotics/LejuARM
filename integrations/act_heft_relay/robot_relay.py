#!/usr/bin/env python3
"""Lower-host ACT relay. Default is a reader-only probe; --execute is explicit.

Body targets use the existing native HEFT bridge. Only hand commands get a new
DDS writer. This module never starts motors, switches modes, or restarts CAN.
"""
import argparse
from dataclasses import dataclass, field, asdict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import time

from execution_core import BODY_SPEED_ENVELOPE_DEG_S, NAMES, PERIOD, finite, hands, hand_feedback, hand_command_from_feedback, validate_packet, within, require_hand_samples, require_body_tracking, relay_timeouts, feedback_age_limit_ms, command_packet_age_limit_ms, feedback_guard_indices
from can_guards import CanWatch
from command_ownership import HandCommandOwner


def emit(value):
    print(json.dumps(value, allow_nan=False), flush=True)


def last_line(path):
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size-16384))
        for line in reversed(stream.read().splitlines()):
            try:
                return json.loads(line)
            except (ValueError, UnicodeError):
                pass
    raise RuntimeError('NATIVE_REFERENCE_LOG_EMPTY')


def can_status():
    return json.loads(subprocess.check_output(
        ['ip', '-j', '-d', '-s', 'link', 'show', 'type', 'can'], text=True, timeout=.15))


def native_bridge_preflight_errors(native):
    """Check native process arguments against the executor's finite envelope."""
    speed, rate = native.max_speed_deg_s, native.rate
    if (any(type(value) not in (int, float) or not math.isfinite(value)
            for value in (speed, rate))
            or not 0 < speed <= BODY_SPEED_ENVELOPE_DEG_S or rate != 1. / PERIOD):
        return [f'BRIDGE_REQUIRES_FINITE_SPEED_UP_TO_{BODY_SPEED_ENVELOPE_DEG_S:g}'
                f'_AT_{1. / PERIOD:g}_HZ']
    return []


def require_reference_tracking(reference, config):
    return require_body_tracking(reference['actual_hardware_q_rad'], reference['hardware_q_rad'],
                                 [0,17,18,19,20], config['tracking_abort_deg'], config=config)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, required=True)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--transport-check', action='store_true', help='Validate stationary targets and ack; no UDP socket or command DataWriter')
    args = ap.parse_args()
    config = json.loads(args.config.read_text())
    feedback_guard_indices(config, [0,17,18,19,20])
    feedback_ms = feedback_age_limit_ms(config)
    command_age_ms = command_packet_age_limit_ms(config)
    feedback_s = feedback_ms / 1000.
    ack_timeout_s, input_timeout_s = relay_timeouts(config)
    root = Path(config['native_root'])
    scripts = root/'src/leju_launch/scripts'
    sys.path.insert(0, str(scripts))
    from dds_heft_external_control import parse_args, fresh_controller, active, udp_only_uri, PublicationGuard
    for name, sha in config['native_source_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != sha:
            raise RuntimeError('NATIVE_SOURCE_CHANGED: '+name)
    # Keep the native DDS discovery settings, including its UDP RPC workaround.
    uri = os.environ.get('CYCLONEDDS_URI', 'file:///etc/cyclonedds/cyclonedds_shm.xml')
    os.environ['CYCLONEDDS_URI'] = udp_only_uri(uri)
    bridge = scripts/'dds_heft_external_control.py'
    found = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            argv = (p/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
        except (OSError, UnicodeError):
            continue
        if str(bridge) in argv and '--execute' in argv:
            found.append((int(p.name), argv[argv.index(str(bridge))+1:]))
    if len(found) != 1:
        raise RuntimeError('EXACTLY_ONE_NATIVE_BRIDGE_REQUIRED')
    pid, argv = found[0]
    native = parse_args(argv)
    config['arm_speed_deg_s'] = min(config['arm_speed_deg_s'], native.max_speed_deg_s)
    config['waist_speed_deg_s'] = min(config['waist_speed_deg_s'], native.max_speed_deg_s)
    sessions = list((root/'validation/heft_external_21').glob('*_'+str(pid)))
    if '--output-dir' in argv:
        sessions = [native.output_dir]
    if len(sessions) != 1:
        raise RuntimeError('NATIVE_SESSION_NOT_UNIQUE')
    log = sessions[0]/'commands.jsonl'
    from dds_heft_pose_test import TestIo
    from dds_heft_controller_observer import ControllerStateObserver
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.annotations import final
    from cyclonedds.idl.types import int32, uint32, uint8, sequence
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.qos import Qos, Policy

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
    class Float64Array(IdlStruct, typename='leju::msgs::Float64Array'):
        header_sec: int32 = 0
        header_nanosec: uint32 = 0
        data: sequence[float] = field(default_factory=list)

    io = TestIo('/rt/pico/heft_reference', feedback_s)
    observer = ControllerStateObserver(io.participant, poll_interval=.1)
    qos = Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(1))
    reader = DataReader(io.participant, Topic(io.participant, '/rt/hand_state', HandState), qos=qos)
    ownership = PublicationGuard(io.participant, '/rt/hand_cmd')
    command_owner = HandCommandOwner()
    hand_command_reader = DataReader(io.participant, Topic(io.participant, '/rt/hand_cmd', Float64Array),
        qos=Qos(Policy.Reliability.BestEffort,Policy.Durability.Volatile,Policy.History.KeepLast(100)))
    body_ownership = PublicationGuard(io.participant, '/rt/pico/heft_reference')
    observer.start()
    body_guid = None
    from cyclonedds.core import InstanceState
    last_hand, hand_stamp, hand_received = None, 0., 0.
    can, last_can = [], 0.
    can_watch = CanWatch()
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    hand_writer = sock = None
    sent_body = sent_hand = 0
    previous = init = None
    udp_sequence = time.time_ns()

    def observe(warmup=False):
        nonlocal last_hand, hand_stamp, hand_received, can, last_can
        now = time.monotonic()
        io.read()
        for msg in reader.take(N=100):
            if not isinstance(msg, HandState):
                continue
            stamp = msg.header_sec + msg.header_nanosec * 1e-9
            if stamp > hand_stamp:
                last_hand, hand_stamp, hand_received = msg, stamp, now
        if now-last_can >= .2:
            can, last_can = can_status(), now
            can_watch.update(can, now)
            queues = json.loads(subprocess.check_output(['tc','-j','-s','qdisc','show'], text=True, timeout=.15))
            blocked_queues = [r for r in queues if r.get('dev','').startswith('bcan') and r.get('qlen',0)>32]
            if blocked_queues:
                emit(dict(event='can_queue_rejected', at=time.time(), queues=blocked_queues))
                raise RuntimeError('CAN_SEND_QUEUE_GROWING: '+','.join(
                    str(r['dev'])+' qlen='+str(r['qlen']) for r in blocked_queues))
        for endpoint in body_ownership.reader.take(N=1000):
            if endpoint.sample_info.instance_state != InstanceState.Alive:
                body_ownership.endpoints.pop(endpoint.key,None)
            elif endpoint.topic_name == body_ownership.topic:
                body_ownership.endpoints[endpoint.key] = endpoint
        for msg in hand_command_reader.take(N=1000):
            if isinstance(msg,Float64Array) and msg.sample_info.valid_data:
                command_owner.observe(msg.sample_info.publication_handle,
                    msg.header_sec*10**9+msg.header_nanosec,list(msg.data))
        state = observer.snapshot() if warmup else fresh_controller(observer, .5)
        reference = last_line(log)
        return state, reference

    def healthy(state, reference):
        io.require_fresh()
        io.require_imu()
        # TestIo shares an age window for joints and IMU; only joint/hand
        # feedback is changing. Retain the independent 250 ms IMU requirement.
        if (io.last_imu is None or time.monotonic()-io.last_imu.received > .25
                or not -.05 <= time.time()-io.last_imu.stamp <= .25):
            raise RuntimeError('IMU_FEEDBACK_STALE')
        if state is None:
            raise RuntimeError('CONTROLLER_STATE_UNAVAILABLE: '+str(observer.error))
        if not active(state):
            raise RuntimeError('CONTROLLER_NOT_STABLE_HEFT')
        nonlocal body_guid
        if len(body_ownership.endpoints) != 1:
            raise RuntimeError('EXACTLY_ONE_HEFT_WRITER_REQUIRED')
        current_guid = str(next(iter(body_ownership.endpoints)))
        if body_guid is None:
            body_guid = current_guid
        elif current_guid != body_guid:
            raise RuntimeError('HEFT_WRITER_CHANGED')
        # Idle SDK publication endpoints are allowed; another actual hand
        # command is rejected above before the next outgoing target.
        if last_hand is None or time.monotonic()-hand_received > feedback_s:
            raise RuntimeError('HAND_FEEDBACK_STALE')
        hand_feedback(list(last_hand.position))
        require_hand_samples(asdict(last_hand), time.time()-hand_stamp, feedback_ms)
        stamp = reference['header_sec'] + reference['header_nanosec']*1e-9
        if not -.05 <= time.time()-stamp <= .25 or reference['phase'] != 'external':
            raise RuntimeError('NATIVE_REFERENCE_STALE')
        if not Path('/proc', str(pid)).exists():
            raise RuntimeError('NATIVE_BRIDGE_EXITED')

    def udp(q):
        nonlocal udp_sequence, sent_body
        udp_sequence += 1
        sock.sendto(json.dumps({'q':q, 'units':'rad', 'coordinates':'hardware',
                               'seq':udp_sequence, 'sent_at_ns':time.time_ns()}, allow_nan=False).encode(),
                    ('127.0.0.1', config['udp_port']))
        sent_body += 1

    try:
        warmup_start = time.monotonic()
        while True:
            state, reference = observe(True)
            elapsed = time.monotonic()-warmup_start
            # DDS discovery/RPC can take longer than two seconds. Wait only
            # for first discovery, before any target; known bad state fails.
            if elapsed >= 8. or (elapsed >= 2. and state is not None):
                break
            time.sleep(.02)
        errors = []
        try:
            state = fresh_controller(observer, .5)
            healthy(state, reference)
        except Exception as exc:
            errors.append(str(exc))
        errors.extend(native_bridge_preflight_errors(native))
        report = dict(event='probe', boot_id=boot, bridge_pid=pid, bridge_argv=argv,
                      bridge_speed_deg_s=native.max_speed_deg_s, native_console_enabled=not native.no_console,
                      relay_ack_timeout_s=ack_timeout_s, relay_input_timeout_s=input_timeout_s,
                      joint_and_hand_age_limit_ms=feedback_ms,
                      command_packet_age_limit_ms=command_age_ms,
                      body_feedback_monitor_only_joints=config.get('body_feedback_monitor_only_joints', []),
                      tracking_abort_deg=config['tracking_abort_deg'],
                      controller=state, can=can,
                      controller_observer_error=observer.error,
                      controller_recent_requests=observer.requests()[-3:],
                      discovery_seconds=time.monotonic()-warmup_start,
                      reference=reference, errors=errors, commands_sent=0,
                      hand_command_messages_seen=command_owner.received,
                      foreign_hand_commands_seen=command_owner.foreign,
                      hand_position=list(last_hand.position) if last_hand else None)
        emit(report)
        if not (args.execute or args.transport_check):
            return 0
        if errors:
            raise RuntimeError(';'.join(errors))
        lock = open('/tmp/roban_act_heft21_relay.lock', 'a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Clock samples have no hardware side effect. The workstation measures
        # RTT at receipt, before any disk logging or event-queue delay.
        for attempt in range(4):
            if not select.select([sys.stdin], [], [], 5.)[0]:
                raise RuntimeError('INITIAL_CONTEXT_TIMEOUT')
            init = json.loads(sys.stdin.readline(65536))
            if init.get('op') != 'clock_ping':
                break
            emit(dict(event='clock_pong', id=init.get('id'), at=time.time()))
        if init.get('op') != 'initialize' or init.get('boot_id') != boot or init.get('side') != 'right':
            raise RuntimeError('INITIAL_CONTEXT_MISMATCH')
        home, hand_home = finite(init['home'], 21), hands(init['hand_home'])
        measured_hand_home = hand_feedback(init.get('measured_hand_home', hand_home))
        if hand_command_from_feedback(measured_hand_home) != hand_home:
            raise RuntimeError('INITIAL_HAND_COMMAND_SEED_MISMATCH')
        state, reference = observe()
        healthy(state, reference)
        motor = [io.last.q[12]] + list(io.last.q[:12]) + list(io.last.q[13:21])
        measured_home = finite(init['measured_home'],21)
        if (not within(motor, measured_home, range(21), 1.) or
                not within(reference['hardware_q_rad'], home, range(21), 1.) or
                max(abs(a-b) for a,b in zip(last_hand.position, measured_hand_home)) > 3):
            raise RuntimeError('START_CHANGED_REPLAN_REQUIRED')
        previous = {'q':home, 'hand':hand_home}
        if args.execute:
            # DDS discovery may take time. Finish creating endpoints before
            # announcing readiness; no command is sent until the first target.
            from cyclonedds.pub import DataWriter
            hand_writer = DataWriter(io.participant, Topic(io.participant, '/rt/hand_cmd', Float64Array), qos=qos)
            ownership.own_guid = hand_writer.get_guid()
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1',28722))
        emit({'event':'ready', 'commands_sent':0, 'at':time.time(), 'diagnostic':args.transport_check})
        last_seq, last_input = -1, time.monotonic()
        buffer = b''
        next_tick = time.monotonic()
        while True:
            observe_start = time.monotonic()
            state, reference = observe()
            healthy(state, reference)
            observe_ms = (time.monotonic()-observe_start)*1000
            # Keep monitoring R3; only the configured required joints can abort
            # for tracking error, including while waiting for the next command.
            tracking = require_reference_tracking(reference, config)
            if not within(reference['hardware_q_rad'], home, list(range(1,17)), .02):
                raise RuntimeError('INACTIVE_NATIVE_REFERENCE_CHANGED')
            if not within(reference['hardware_q_rad'], previous['q'], [0,17,18,19,20], max(1.,native.max_speed_deg_s*.08)):
                raise RuntimeError('NATIVE_REFERENCE_NOT_FOLLOWING_OR_OTHER_INPUT')
            if select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 65536)
                if not data:
                    break
                buffer += data
            if len(buffer) > 65536:
                raise RuntimeError('INPUT_BACKLOG')
            if b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                packet = json.loads(raw)
                if packet.get('op') == 'stop':
                    break
                received_at = time.time()
                try:
                    q, h = validate_packet(packet, previous, home, hand_home, config, init['side'], received_at, last_seq)
                except ValueError as exc:
                    stamp = packet.get('sent_at')
                    age_ms = ((received_at-stamp)*1000
                              if type(stamp) in (int, float) and math.isfinite(stamp) else None)
                    emit(dict(event='rejected_packet', reason=str(exc), seq=packet.get('seq'),
                              sent_at=stamp, received_at=received_at, buffered_bytes=len(buffer),
                              packet_age_ms=age_ms, command_packet_age_limit_ms=command_age_ms))
                    raise
                if last_seq == -1:
                    motor = [io.last.q[12]] + list(io.last.q[:12]) + list(io.last.q[13:21])
                    if (not within(motor, measured_home, range(21), 1.) or
                            max(abs(a-b) for a,b in zip(last_hand.position,measured_hand_home)) > 3):
                        raise RuntimeError('START_CHANGED_REPLAN_REQUIRED')
                if args.transport_check:
                    # Diagnostic packets must be exactly the starting hold.
                    # This branch never instantiates a publisher or socket.
                    if q != home or h != hand_home:
                        raise RuntimeError('DIAGNOSTIC_TARGET_CHANGED')
                else:
                    udp(q)
                    ns = time.time_ns()
                    command_owner.register(ns,h)
                    hand_writer.write(Float64Array(ns//10**9, ns%10**9, h))
                    sent_hand += 1
                previous = {'q':q, 'hand':h}
                last_seq, last_input = packet['seq'], time.monotonic()
                emit({'event':'ack', 'seq':last_seq, 'at':time.time(), 'requested':previous,
                      'packet_received_at':received_at, 'packet_age_ms':(received_at-packet['sent_at'])*1000,
                      'observe_ms':observe_ms, 'buffered_bytes':len(buffer),
                      'native_reference':reference, 'body_tracking':tracking, 'hand_feedback':list(last_hand.position),
                      'joint_stamp':io.last.stamp, 'hand_stamp':hand_stamp})
            # A delayed return ACK pauses the upper sender. Keep observing local
            # feedback while waiting; never synthesize or queue future targets.
            if time.monotonic()-last_input > (5. if last_seq == -1 else input_timeout_s):
                emit(dict(event='input_timeout', at=time.time(), last_seq=last_seq,
                          last_input_age_ms=(time.monotonic()-last_input)*1000,
                          observe_ms=observe_ms, buffered_bytes=len(buffer)))
                raise RuntimeError('ACTIVATION_TIMEOUT' if last_seq == -1 else 'INPUT_TIMEOUT')
            next_tick += PERIOD
            if next_tick < time.monotonic():
                next_tick = time.monotonic()
            time.sleep(max(0., next_tick-time.monotonic()))
        return 0
    finally:
        # Cancel the outstanding native target at its last actually published
        # reference, not at a future model target or a guessed measured pose.
        if sock is not None:
            try:
                ref = last_line(log)
                stamp = ref['header_sec']+ref['header_nanosec']*1e-9
                if sent_body and -.05 <= time.time()-stamp <= .25:
                    udp(ref['hardware_q_rad'])
            finally:
                sock.close()
        observer.stop()
        emit({'event':'closed', 'body_datagrams_sent':sent_body, 'hand_commands_sent':sent_hand,
              'mode_switches_sent':0, 'native_bridge_stopped':False, 'foreign_hand_commands_seen':command_owner.foreign,
              'diagnostic':args.transport_check})


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        import traceback
        traceback.print_exc(file=sys.stderr)
        emit({'event':'error', 'error':str(exc), 'at':time.time()})
        raise SystemExit(2)
