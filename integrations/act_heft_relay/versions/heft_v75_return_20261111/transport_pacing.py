"""Bounded 50 Hz FIFO transport and local feedback diagnostics; no robot SDK."""
from collections import deque
import json
import math
import time

from execution_core import PERIOD, CLOSE, step_reference, command_packet_age_limit_ms


def pipeline_window(config):
    value = config.get('transport_pipeline_window', 1)
    if type(value) is not int or not 1 <= value <= 4:
        raise ValueError('INVALID_TRANSPORT_PIPELINE_WINDOW')
    return value


def pop_packet(buffer, window):
    """Bound all queued lines and give an already received STOP priority."""
    if len(buffer) > 65536:
        raise RuntimeError('INPUT_BACKLOG')
    lines = buffer.split(b'\n')
    complete = lines[:-1]
    packets = [json.loads(line) for line in complete]
    if any(packet.get('op') == 'stop' for packet in packets):
        return {'op': 'stop'}, b''
    if len(complete) + bool(lines[-1]) > window:
        raise RuntimeError('INPUT_PIPELINE_OVERFLOW')
    if not complete:
        return None, buffer
    return packets[0], b'\n'.join(lines[1:])


def feedback_diagnostics(io, state, reference, observe_ms=None):
    now, wall = time.monotonic(), time.time()
    def age(sample):
        if sample is None:
            return dict(source_age_ms=None, receive_age_ms=None)
        return dict(source_age_ms=(wall-sample.stamp)*1000,
                    receive_age_ms=(now-sample.received)*1000)
    return dict(imu=age(io.last_imu), joint=age(io.last),
                imu_age_limit_ms=250., observe_ms=observe_ms,
                controller=state, native_reference_age_ms=(wall-reference['header_sec']-
                    reference['header_nanosec']*1e-9)*1000)


def require_fresh_imu(io):
    sample = io.last_imu
    if (sample is None or not math.isfinite(sample.stamp) or not math.isfinite(sample.received)
            or time.monotonic()-sample.received > .25
            or not -.05 <= time.time()-sample.stamp <= .25):
        raise RuntimeError('IMU_FEEDBACK_STALE')
    io.require_imu()


def target_admitted(transport, q):
    """Only body row progression may use admission; phase barriers use ACKs."""
    method = getattr(transport, 'admitted', transport.reached)
    return method(q)


def paced_transport(base):
    class PacedTransport(base):
        """At most four immutable targets, including unsent and unacknowledged.

        The main executor checks feedback while backpressured. Every intermediate
        target is transmitted once, in order, with at least one period between
        sends. A late ACK never triggers a catch-up burst. All hand/query/return
        completion barriers still require a lower-host applied ACK.
        """
        def __init__(self, config, folder, initialize, diagnostic=False):
            self.window = pipeline_window(config)
            self.pending = deque()
            self.admitted_seq = 0
            self.last_admitted = None
            self.max_outstanding = 0
            self.backpressure_ticks = 0
            self.wire_targets = {}
            super().__init__(config, folder, initialize, diagnostic)

        def wait_event(self, timeout):
            event = super().wait_event(timeout)
            if event.get('event') == 'probe' and (
                    event.get('transport_pacing_protocol') != 'bounded_fifo_50hz_v1'
                    or event.get('transport_pipeline_window') != self.window):
                self.close()
                raise RuntimeError('REMOTE_PACING_CONFIG_MISMATCH: run prepare.py')
            return event

        def outstanding(self):
            return self.admitted_seq-(self.last_ack['seq'] if self.last_ack else 0)

        def has_capacity(self):
            with self.lock:
                return self.outstanding() < self.window

        def heartbeat(self):
            # Called only after the main loop has rechecked stop and feedback.
            with self.lock:
                self.updated = time.monotonic()
                self.backpressure_ticks += 1
            self.check()

        def update(self, q, hand):
            target = dict(q=list(q), hand=list(hand))
            with self.lock:
                self.updated = time.monotonic()
                if target == self.last_admitted and self.outstanding():
                    return
                if self.outstanding() >= self.window:
                    raise RuntimeError('UPPER_PIPELINE_OVERFLOW')
                if self.last_admitted is not None:
                    bounded = step_reference(self.last_admitted['q'], target['q'], self.caps)
                    if max(abs(a-b) for a,b in zip(bounded, target['q'])) > 1e-9:
                        raise RuntimeError('PIPELINE_BODY_RATE')
                    if any(abs(a-b) > CLOSE[i % 6]*1.875*PERIOD+1e-7
                           for i,(a,b) in enumerate(zip(self.last_admitted['hand'],hand))):
                        raise RuntimeError('PIPELINE_HAND_RATE')
                self.admitted_seq += 1
                self.pending.append((self.admitted_seq, target, self.updated))
                self.last_admitted = target
                self.max_outstanding = max(self.max_outstanding, self.outstanding())

        def admitted(self, q):
            with self.lock:
                return self.last_admitted is not None and max(
                    abs(a-b) for a,b in zip(q,self.last_admitted['q'])) < 1e-7

        def validate_ack(self, event):
            with self.lock:
                seq = event.get('seq')
                expected = (self.last_ack['seq'] if self.last_ack else 0)+1
                if type(seq) is not int or seq != expected or seq not in self.wire_targets:
                    raise RuntimeError('PIPELINE_ACK_SEQUENCE')
                if event.get('requested') != self.wire_targets[seq]:
                    raise RuntimeError('PIPELINE_ACK_TARGET')
                del self.wire_targets[seq]

        def send_loop(self):
            next_send = time.monotonic()
            try:
                while not self.stop_event.is_set():
                    self.check()
                    with self.lock:
                        if time.monotonic()-self.updated > .25:
                            raise RuntimeError('EXECUTOR_HEARTBEAT_LOST')
                        item = self.pending.popleft() if self.pending and time.monotonic() >= next_send else None
                    if item is None:
                        self.stop_event.wait(.002)
                        continue
                    seq, target, created = item
                    begin = time.monotonic()
                    # LAN tolerance does not extend the lifetime of an unsent
                    # local target; keep the previous one-second local budget.
                    if (begin-created)*1000 > min(1000.,command_packet_age_limit_ms(self.config)):
                        raise RuntimeError('UNSENT_TARGET_STALE')
                    stamp = self.clock_anchor_remote+begin-self.clock_anchor_mono
                    with self.lock:
                        self.wire_targets[seq] = target
                    self.process.stdin.write(json.dumps(dict(target,op='target',seq=seq,sent_at=stamp))+'\n')
                    self.process.stdin.flush()
                    self.last_sent = dict(event='sent',seq=seq,at=time.time(),remote_sent_at=stamp,
                                          write_seconds=time.monotonic()-begin, pipeline=True)
                    self.last_wire = target
                    self.log.write(self.last_sent)
                    next_send = time.monotonic()+PERIOD
            except Exception as exc:
                self.fail('sender',exc)

        def pacing_report(self):
            return dict(protocol='bounded_fifo_50hz_v1',window=self.window,
                        maximum_outstanding=self.max_outstanding,
                        backpressure_feedback_checks=self.backpressure_ticks,
                        body_row_completion='admitted_to_bounded_fifo',
                        boundary_completion='lower_applied_ack_and_measured_feedback')

    return PacedTransport
