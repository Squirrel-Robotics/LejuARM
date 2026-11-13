#!/usr/bin/env python3
"""
通用 DDS-RPC 客户端 (Python). 复刻 lejulab 的 dds_rpc::Requester 协议.

Topic 命名约定 (lejulab/src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc/include/dds_rpc/rpc_types.hpp):
    request  topic: "rq/<service_name>/Request"
    reply    topic: "rr/<service_name>/Reply"

关联方式: Request.header.request_id (writer_guid + sequence_number)
       Reply.header.related_request_id 必须等于之上 → 客户端按此匹配回包.
"""
import os
import time
import threading
import uuid
from typing import Type, Optional, Any
from dataclasses import dataclass, field

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.idl import IdlEnum, IdlStruct
from cyclonedds.idl.annotations import final
from cyclonedds.idl.types import sequence, array, uint8, uint32, int32, int64


# =============================================================================
# RPC 基础类型 (跟 lejulab rpc_types.idl 严格一致)
# =============================================================================

@final
@dataclass
class SampleIdentity(IdlStruct, typename="dds_rpc::SampleIdentity"):
    writer_guid: array[uint8, 16] = field(default_factory=lambda: [0] * 16)
    sequence_number: int64 = 0


@final
@dataclass
class RequestHeader(IdlStruct, typename="dds_rpc::RequestHeader"):
    request_id: SampleIdentity = field(default_factory=SampleIdentity)
    instance_name: str = ""


class RemoteExceptionCode(
    IdlEnum, typename="dds_rpc::RemoteExceptionCode"
):
    REMOTE_EX_OK = 0
    REMOTE_EX_UNSUPPORTED = 1
    REMOTE_EX_INVALID_ARGUMENT = 2
    REMOTE_EX_OUT_OF_RESOURCES = 3
    REMOTE_EX_UNKNOWN_OPERATION = 4
    REMOTE_EX_UNKNOWN_EXCEPTION = 5


@final
@dataclass
class ReplyHeader(IdlStruct, typename="dds_rpc::ReplyHeader"):
    related_request_id: SampleIdentity = field(default_factory=SampleIdentity)
    remote_ex: RemoteExceptionCode = RemoteExceptionCode.REMOTE_EX_OK


# =============================================================================
# Requester (RPC 客户端)
# =============================================================================

def _request_topic_name(service_name: str) -> str:
    return f"rq/{service_name}/Request"


def _reply_topic_name(service_name: str) -> str:
    return f"rr/{service_name}/Reply"


class RpcRequester:
    """
    跟一个 DDS-RPC 服务通信的客户端.

    用法:
        req = RpcRequester(
            participant, service_name="ControllerService/SetArmMode",
            request_type=SetInt32Request, reply_type=SetInt32Reply,
        )
        request = SetInt32Request(data=2)   # mode=External
        reply = req.call(request, timeout_sec=3.0)
        if reply: print(reply.success, reply.message)
    """

    def __init__(
        self,
        participant: DomainParticipant,
        service_name: str,
        request_type: Type,
        reply_type: Type,
    ):
        self.participant = participant
        self.service_name = service_name
        self.request_type = request_type
        self.reply_type = reply_type

        # 16-byte 随机 GUID, 标识本客户端
        self._writer_guid = list(uuid.uuid4().bytes)  # 16 bytes
        self._next_seq = 1
        self._seq_lock = threading.Lock()

        # 待响应字典: {(guid_tuple, seq) → (Event, holder_for_reply)}
        self._pending = {}
        self._pending_lock = threading.Lock()

        # lejulab DDS-RPC 的 request reader 使用 TRANSIENT_LOCAL；request
        # writer 必须提供相同或更高的 durability，否则 DDS discovery 虽能
        # 看到 topic，但 reader/writer 不会匹配，最终表现为 RPC 超时。
        request_qos = Qos(
            Policy.Reliability.Reliable(0),
            Policy.Durability.TransientLocal,
            Policy.History.KeepLast(100),
        )
        # Reply reader 与 lejulab C++ Requester 保持一致，使用 VOLATILE，
        # 避免 bridge 重启后读取旧的历史 reply。
        reply_qos = Qos(
            Policy.Reliability.Reliable(0),
            Policy.Durability.Volatile,
            Policy.History.KeepLast(100),
        )

        # Request publisher
        req_topic = Topic(participant, _request_topic_name(service_name),
                          request_type, qos=request_qos)
        self._writer = DataWriter(Publisher(participant), req_topic,
                                  qos=request_qos)

        # Reply subscriber
        rep_topic = Topic(participant, _reply_topic_name(service_name),
                          reply_type, qos=reply_qos)
        self._reader = DataReader(Subscriber(participant), rep_topic,
                                  qos=reply_qos)

        # 后台轮询线程接收 reply
        self._stop = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_loop,
                                             name=f"rpc-poll-{service_name}",
                                             daemon=True)
        self._poll_thread.start()

    def _make_request_id(self) -> SampleIdentity:
        with self._seq_lock:
            seq = self._next_seq
            self._next_seq += 1
        return SampleIdentity(writer_guid=list(self._writer_guid),
                              sequence_number=seq)

    def _pending_key(self, sid: SampleIdentity) -> tuple:
        return (tuple(sid.writer_guid), sid.sequence_number)

    def _poll_loop(self):
        while not self._stop.is_set():
            try:
                samples = self._reader.take(N=10)
                for s in samples:
                    if not isinstance(s, self.reply_type):
                        continue
                    key = self._pending_key(s.header.related_request_id)
                    with self._pending_lock:
                        entry = self._pending.pop(key, None)
                    if entry:
                        event, holder = entry
                        holder['reply'] = s
                        event.set()
            except Exception:
                pass
            time.sleep(0.01)

    def call(self, request, timeout_sec: float = 3.0) -> Optional[Any]:
        """同步调用. 返回 reply (匹配类型) 或 None (超时/异常)."""
        # 1. 给 request 注入 header.request_id
        if not hasattr(request, 'header'):
            raise ValueError("Request 必须有 header 字段")
        rid = self._make_request_id()
        request.header = RequestHeader(request_id=rid, instance_name="")

        # 2. 注册 pending
        key = self._pending_key(rid)
        event = threading.Event()
        holder = {'reply': None}
        with self._pending_lock:
            self._pending[key] = (event, holder)

        # 3. 写 request
        self._writer.write(request)

        # 4. 等 reply
        if event.wait(timeout=timeout_sec):
            return holder['reply']
        # 超时, 清理 pending
        with self._pending_lock:
            self._pending.pop(key, None)
        return None

    def close(self):
        self._stop.set()
        self._poll_thread.join(timeout=1.0)


# =============================================================================
# 跟 lejulab 用的标准类型 (StdSrvs / ControllerService)
# 直接在这里定义, 给 controller_rpc_bridge.py import
# =============================================================================

@final
@dataclass
class SetInt32Request(IdlStruct, typename="std_srvs::SetInt32Request"):
    header: RequestHeader = field(default_factory=RequestHeader)
    data: int32 = 0


@final
@dataclass
class SetInt32Reply(IdlStruct, typename="std_srvs::SetInt32Reply"):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    success: bool = False
    message: str = ""


@final
@dataclass
class TriggerRequest(IdlStruct, typename="std_srvs::TriggerRequest"):
    header: RequestHeader = field(default_factory=RequestHeader)


@final
@dataclass
class TriggerReply(IdlStruct, typename="std_srvs::TriggerReply"):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    success: bool = False
    message: str = ""


@final
@dataclass
class SwitchControllerRequest(IdlStruct, typename="leju::srv::SwitchControllerRequest"):
    header: RequestHeader = field(default_factory=RequestHeader)
    controller_name: str = ""


@final
@dataclass
class SwitchControllerReply(IdlStruct, typename="leju::srv::SwitchControllerReply"):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    success: bool = False
    message: str = ""
    current_controller: str = ""


@final
@dataclass
class GetControllerStateRequest(
    IdlStruct, typename="leju::srv::GetControllerStateRequest"
):
    header: RequestHeader = field(default_factory=RequestHeader)


@final
@dataclass
class GetControllerStateReply(
    IdlStruct, typename="leju::srv::GetControllerStateReply"
):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    current_controller: str = ""
    arm_mode: int32 = 0
    head_mode: int32 = 0
    waist_mode: int32 = 0
    controller_transitioning: bool = False
    available_controllers: sequence[str] = field(default_factory=list)


@final
@dataclass
class GetRuntimeStateRequest(
    IdlStruct, typename="leju::srv::GetRuntimeStateRequest"
):
    header: RequestHeader = field(default_factory=RequestHeader)


@final
@dataclass
class GetRuntimeStateReply(
    IdlStruct, typename="leju::srv::GetRuntimeStateReply"
):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    data_ready: bool = False
    hardware_state: int32 = 0
    lifecycle_state: int32 = 0


# =============================================================================
# ControllerService: GetMotionState (dance/motion playback state)
# 定期由 controller_rpc_bridge.py 调用并发布到 /dance/motion_state topic
# =============================================================================

@final
@dataclass
class GetMotionStateRequest(
    IdlStruct, typename="leju::srv::GetMotionStateRequest"
):
    header: RequestHeader = field(default_factory=RequestHeader)


@final
@dataclass
class GetMotionStateReply(
    IdlStruct, typename="leju::srv::GetMotionStateReply"
):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    supported: bool = False
    motion_playing: bool = False
    current_motion_name: str = ""
    available_motion_names: sequence[str] = field(default_factory=list)


# =============================================================================
# SetString RPC 类型 (对应 std_srvs::SetString)
# 用于 TactPlayerService/PlayTact (传 tact 动作名) 等字符串型 RPC
# =============================================================================

@final
@dataclass
class SetStringRequest(IdlStruct, typename="std_srvs::SetStringRequest"):
    header: RequestHeader = field(default_factory=RequestHeader)
    data: str = ""


@final
@dataclass
class SetStringReply(IdlStruct, typename="std_srvs::SetStringReply"):
    header: ReplyHeader = field(default_factory=ReplyHeader)
    success: bool = False
    message: str = ""


# =============================================================================
# DDS topic 类型 (非 RPC)
# =============================================================================

@final
@dataclass
class StringData(IdlStruct, typename="leju::msgs::StringData"):
    header_sec: int32 = 0
    header_nanosec: uint32 = 0
    data: str = ""
