import ctypes
import ctypes.util
import json
import math
import os
import threading
from .data_types import EndEffectorSide, EndEffectorState


class NativeTransport:
    """通过 C ABI 复用平台 C++ DDS SDK，不依赖 ROS 或 Python DDS 生成器。"""
    def __init__(self, library=None):
        library = library or os.environ.get("LEJUSDK_CLAW_LIBRARY") or ctypes.util.find_library("lejusdk_claw")
        if not library:
            raise RuntimeError("请设置 LEJUSDK_CLAW_LIBRARY 为 liblejusdk_claw.so 的完整路径")
        self._lib = ctypes.CDLL(library)
        self._lib.leju_claw_create.argtypes = []
        self._lib.leju_claw_create.restype = ctypes.c_void_p
        self._lib.leju_claw_destroy.argtypes = [ctypes.c_void_p]
        self._lib.leju_claw_destroy.restype = None
        self._lib.leju_claw_request.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.leju_claw_request.restype = ctypes.c_char_p
        self._lock = threading.Lock()
        self._handle = self._lib.leju_claw_create()
        if not self._handle:
            raise RuntimeError("夹爪 DDS SDK 初始化失败，请检查 ROBOT_VERSION 和 DDS 配置")

    def request(self, payload):
        with self._lock:
            if not self._handle:
                raise RuntimeError("客户端已断开")
            response = json.loads(self._lib.leju_claw_request(
                self._handle, json.dumps(payload, allow_nan=False).encode()))
        if not response["ok"]:
            raise RuntimeError(response.get("error", "DDS 命令发布失败"))
        # 新硬件节点会把 controller 的接受结果随 data 返回；旧节点仍返回 true。
        # 只有明确 rejected/error 才报失败，不能让公布成功被误当成执行成功。
        data = response.get("data", True)
        if payload.get("op") == "control" and isinstance(data, dict):
            status = data.get("status")
            if status in ("rejected", "error") or data.get("success") is False:
                raise RuntimeError(data.get("message", "夹爪控制命令未被执行"))
            return status in ("accepted", "executed")
        return data

    def disconnect(self):
        with self._lock:
            if self._handle:
                self._lib.leju_claw_destroy(self._handle)
                self._handle = None


class WebSocketTransport:
    """与本地封装共用 JSON 操作，服务端仍走同一套 DDS SDK。"""
    def __init__(self, url="ws://127.0.0.1:8765"):
        from websockets.sync.client import connect
        self._connection = connect(url, open_timeout=10, max_size=65536)
        self._lock = threading.Lock()

    def request(self, payload):
        with self._lock:
            self._connection.send(json.dumps(payload, allow_nan=False))
            response = json.loads(self._connection.recv(timeout=12))
        if not response["ok"]:
            raise RuntimeError(response.get("error", "WebSocket 请求失败"))
        data = response.get("data", True)
        if payload.get("op") == "control" and isinstance(data, dict):
            status = data.get("status")
            if status in ("rejected", "error") or data.get("success") is False:
                raise RuntimeError(data.get("message", "夹爪控制命令未被执行"))
            return status in ("accepted", "executed")
        return data

    def disconnect(self):
        self._connection.close()


CLAMP_OPEN_POSITION = 5.0


class LejuClawControl:
    """保留源 SDK 接口，单侧缓存由已有平台控制器统一维护。"""
    def __init__(self, transport=None):
        self.transport = transport if transport is not None else NativeTransport()

    def request(self, payload):
        return self.transport.request(payload)

    def control(self, position, velocity, torque, side, is_high_freq=False):
        side = EndEffectorSide(side.value)
        names = ["left_claw", "right_claw"] if side == EndEffectorSide.BOTH else [side.value + "_claw"]
        if not all(len(values) == len(names) for values in (position, velocity, torque)):
            raise ValueError("夹爪名称、位置、速度和电流长度必须一致")
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in (*position, *velocity, *torque)):
            raise ValueError("夹爪位置、速度和电流必须是有限数值")
        return self.request({"op": "control", "name": names, "position": position,
            "velocity": velocity, "effort": torque, "is_high_freq": is_high_freq})

    def release(self, side):
        size = 2 if side == EndEffectorSide.BOTH else 1
        return self.control([CLAMP_OPEN_POSITION] * size, [90] * size, [1.0] * size, side)

    @property
    def eef_state(self):
        state = self.request({"op": "state"})
        fields = ("name", "position", "velocity", "effort", "state")
        if not isinstance(state, dict) or any(not isinstance(state.get(key), list) for key in fields):
            raise RuntimeError("尚未收到夹爪反馈，不能将缺失反馈视为到位")
        size = len(state["name"])
        if size == 0 or any(len(state[key]) != size for key in fields[1:]):
            raise RuntimeError("夹爪反馈字段缺失或长度不一致")
        available = [name for name in ("left_claw", "right_claw") if name in state["name"]]
        unknown = [name for name in state["name"] if name not in ("left_claw", "right_claw")]
        if unknown or any(state["name"].count(name) != 1 for name in state["name"]):
            raise RuntimeError("夹爪反馈名称未知或重复")
        # 单侧量程失败时允许恰好一个健康侧；故障侧必须是 UNAVAILABLE，不能伪造 0 或当作到位。
        if not available or (len(available) == 1 and state["name"] != available):
            raise RuntimeError("未收到完整且唯一的双侧夹爪反馈")
        if len(available) == 1:
            i = state["name"].index(available[0])
            try:
                reported = EndEffectorState.GraspingState(state["state"][i])
            except ValueError as error:
                raise RuntimeError("夹爪反馈状态值非法") from error
            if reported != EndEffectorState.GraspingState.UNAVAILABLE:
                raise RuntimeError("单侧夹爪反馈必须标记为不可用")
        result = []
        for name in ("left_claw", "right_claw"):
            if name not in state["name"]:
                result.append(EndEffectorState([], [], [], EndEffectorState.GraspingState.UNAVAILABLE))
                continue
            i = state["name"].index(name)
            result.append(EndEffectorState([state["position"][i]], [state["velocity"][i]],
                [state["effort"][i]], EndEffectorState.GraspingState(state["state"][i])))
        return tuple(result)

    def disconnect(self):
        self.transport.disconnect()
