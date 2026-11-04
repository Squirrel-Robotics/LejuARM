#!/usr/bin/env python3
"""Observe the active controller using only GetControllerState DDS RPC.

All DDS creation, writes and reads run on a daemon thread.  The motion loop
only copies snapshots; missing state is represented by None, never inferred
from an operator confirmation or from the reference topic being published.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
import secrets
import threading
import time
from types import SimpleNamespace


SERVICE = "ControllerService/GetControllerState"
REQUEST_TOPIC = "rq/" + SERVICE + "/Request"
REPLY_TOPIC = "rr/" + SERVICE + "/Reply"


def _wire_types():
    """Lazy mirrors of ControllerService.idl and rpc_types.idl, in field order."""
    from cyclonedds.idl import IdlEnum, IdlStruct
    from cyclonedds.idl.types import array, int32, int64, sequence, uint8

    @dataclass
    class SampleIdentity(IdlStruct, typename="dds_rpc::SampleIdentity"):
        writer_guid: array[uint8, 16]
        sequence_number: int64

    @dataclass
    class RequestHeader(IdlStruct, typename="dds_rpc::RequestHeader"):
        request_id: SampleIdentity
        instance_name: str

    class RemoteExceptionCode(IdlEnum, typename="dds_rpc::RemoteExceptionCode"):
        REMOTE_EX_OK = 0
        REMOTE_EX_UNSUPPORTED = 1
        REMOTE_EX_INVALID_ARGUMENT = 2
        REMOTE_EX_OUT_OF_RESOURCES = 3
        REMOTE_EX_UNKNOWN_OPERATION = 4
        REMOTE_EX_UNKNOWN_EXCEPTION = 5

    @dataclass
    class ReplyHeader(IdlStruct, typename="dds_rpc::ReplyHeader"):
        related_request_id: SampleIdentity
        remote_ex: RemoteExceptionCode

    @dataclass
    class GetControllerStateRequest(IdlStruct,
                                    typename="leju::srv::GetControllerStateRequest"):
        header: RequestHeader

    @dataclass
    class GetControllerStateReply(IdlStruct,
                                  typename="leju::srv::GetControllerStateReply"):
        header: ReplyHeader
        current_controller: str
        arm_mode: int32
        head_mode: int32
        waist_mode: int32
        controller_transitioning: bool
        available_controllers: sequence[str]

    return SimpleNamespace(
        SampleIdentity=SampleIdentity, RequestHeader=RequestHeader,
        ReplyHeader=ReplyHeader, RemoteExceptionCode=RemoteExceptionCode,
        Request=GetControllerStateRequest, Reply=GetControllerStateReply,
    )


def _rpc_qos():
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.util import duration

    # The server's request reader requests TRANSIENT_LOCAL.  The reply reader
    # is VOLATILE, matching the SDK Requester and avoiding historical replies.
    # A short write bound also lets the observer stop promptly under DDS loss.
    common = (Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=20)),
              Policy.History.KeepLast(100))
    return (Qos(*common, Policy.Durability.TransientLocal),
            Qos(*common, Policy.Durability.Volatile))


class _DdsTransport:
    def __init__(self, participant):
        from cyclonedds.pub import DataWriter
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic

        self.types = _wire_types()
        writer_qos, reader_qos = _rpc_qos()
        self.writer = DataWriter(participant,
                                 Topic(participant, REQUEST_TOPIC, self.types.Request),
                                 qos=writer_qos)
        self.reader = DataReader(participant,
                                 Topic(participant, REPLY_TOPIC, self.types.Reply),
                                 qos=reader_qos)

    def write_request(self, guid, seq):
        identity = self.types.SampleIdentity(list(guid), seq)
        header = self.types.RequestHeader(identity, SERVICE)
        self.writer.write(self.types.Request(header))

    def take(self):
        return self.reader.take(N=100)


class ControllerStateObserver:
    """Threaded, read-only RPC observer; no controller or motion mutations."""

    def __init__(self, participant, poll_interval=1.0):
        if not math.isfinite(poll_interval) or poll_interval <= 0:
            raise ValueError("poll_interval must be finite and positive")
        self.participant = participant
        self.poll_interval = float(poll_interval)
        self._guid = secrets.token_bytes(16)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest = None
        self._observations = []
        self._requests = []
        self._error = None
        self._seq = 0
        self._pending = None
        self._last_reply_seq = 0

    @property
    def error(self):
        with self._lock:
            return self._error

    def _set_error(self, error):
        with self._lock:
            self._error = str(error)

    def start(self):
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._run, name="heft-controller-observer", daemon=True)
                self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.1)

    def snapshot(self):
        with self._lock:
            return deepcopy(self._latest)

    def observations(self, start_wall, end_wall):
        with self._lock:
            return deepcopy([item for item in self._observations
                             if start_wall <= item["received_at"] <= end_wall])

    def requests(self):
        """Copies of query times and outcomes for report provenance."""
        with self._lock:
            return deepcopy(self._requests)

    def _send(self, transport):
        self._seq += 1
        request = {"request_seq": self._seq, "requested_at": time.time(),
                   "requested_monotonic": time.monotonic(), "write_ok": False}
        self._pending = request
        with self._lock:
            self._requests.append(request)
        try:
            transport.write_request(self._guid, self._seq)
        except Exception as exc:
            self._pending = None
            with self._lock:
                request["error"] = str(exc)
            self._set_error("GetControllerState write: " + str(exc))
        else:
            with self._lock:
                request["write_ok"] = True

    def _accept(self, sample):
        # A reader can also return invalid/dispose samples; they have no header.
        if self._pending is None or not hasattr(sample, "header"):
            return
        identity = sample.header.related_request_id
        seq = identity.sequence_number
        if (bytes(identity.writer_guid) != self._guid or
                seq != self._pending["request_seq"] or seq <= self._last_reply_seq):
            return
        remote_ex = sample.header.remote_ex
        code = getattr(remote_ex, "value", remote_ex)
        if code != 0:
            self._pending = None
            self._set_error("GetControllerState remote_ex=" + str(code))
            return
        pending = self._pending
        observation = {
            "received_at": time.time(), "received_monotonic": time.monotonic(),
            "request_seq": seq, "requested_at": pending["requested_at"],
            "requested_monotonic": pending["requested_monotonic"],
            "current_controller": sample.current_controller,
            "controller_transitioning": sample.controller_transitioning,
            "arm_mode": sample.arm_mode, "head_mode": sample.head_mode,
            "waist_mode": sample.waist_mode,
            "available_controllers": list(sample.available_controllers),
        }
        self._pending = None
        self._last_reply_seq = seq
        with self._lock:
            self._latest = observation
            self._observations.append(observation)
            self._error = None

    def _run(self):
        try:
            transport = _DdsTransport(self.participant)
        except Exception as exc:
            self._set_error("GetControllerState setup: " + str(exc))
            return
        next_request = time.monotonic()
        while not self._stop.is_set():
            try:
                for sample in transport.take():
                    if self._stop.is_set():
                        break
                    self._accept(sample)
            except Exception as exc:
                self._set_error("GetControllerState read: " + str(exc))
            now = time.monotonic()
            if now >= next_request and not self._stop.is_set():
                if self._pending is not None:
                    self._set_error("GetControllerState: no reply to request " +
                                    str(self._pending["request_seq"]))
                self._send(transport)
                next_request = time.monotonic() + self.poll_interval
            self._stop.wait(min(0.02, max(0.0, next_request - time.monotonic())))
