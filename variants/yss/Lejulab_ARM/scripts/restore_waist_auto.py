#!/usr/bin/env python3
"""现场执行：查询状态后将腰部恢复 Auto；Auto 可能使腰部回到默认 0°。"""
import argparse
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspect", action="store_true", help="只查询，不发送模式切换")
    args = parser.parse_args()
    rpc_dir = Path("/home/lab/roban2-gamepad")
    config = rpc_dir / "dds-network.xml"
    if not config.is_file():
        raise RuntimeError(f"缺少已验证的网络 DDS 配置: {config}")
    os.environ["CYCLONEDDS_URI"] = f"file://{config}"
    os.environ["CYCLONEDDS_HOME"] = str(Path(__file__).resolve().parents[1] / "installed")
    sys.path.insert(0, str(rpc_dir))
    from cyclonedds.domain import DomainParticipant
    from _rpc_lib import (RpcRequester, GetRuntimeStateRequest, GetRuntimeStateReply,
                          GetControllerStateRequest, GetControllerStateReply,
                          GetMotionStateRequest, GetMotionStateReply,
                          SetInt32Request, SetInt32Reply)
    participant = DomainParticipant(0)
    clients = []
    try:
        def client(name, request_type, reply_type):
            result = RpcRequester(participant, f"ControllerService/{name}", request_type, reply_type)
            clients.append(result)
            return result
        runtime = client("GetRuntimeState", GetRuntimeStateRequest, GetRuntimeStateReply)
        controller = client("GetControllerState", GetControllerStateRequest, GetControllerStateReply)
        motion = client("GetMotionState", GetMotionStateRequest, GetMotionStateReply)
        mode = client("SetWaistMode", SetInt32Request, SetInt32Reply)
        time.sleep(0.6)
        state = runtime.call(GetRuntimeStateRequest(), timeout_sec=3)
        control = controller.call(GetControllerStateRequest(), timeout_sec=3)
        if state is None or control is None:
            raise RuntimeError("状态 RPC 超时，未发送模式切换")
        print(f"data_ready={state.data_ready}, hardware={state.hardware_state}, "
              f"lifecycle={state.lifecycle_state}, controller={control.current_controller}, "
              f"transitioning={control.controller_transitioning}, waist_mode={control.waist_mode}", flush=True)
        if args.inspect:
            print("只读检查结束；未发送模式切换。")
            return
        if not (state.data_ready and state.hardware_state == 2 and state.lifecycle_state == 2
                and control.current_controller == "amp" and not control.controller_transitioning):
            raise RuntimeError("需要 AMP、ReadyOk / Running 且未切换控制器，未发送模式切换")
        if control.waist_mode == 1:
            print("腰部已经是 Auto，无需切换。")
            return
        if control.waist_mode != 2:
            raise RuntimeError("腰部不在 External，未发送模式切换")
        active_motion = motion.call(GetMotionStateRequest(), timeout_sec=3)
        if active_motion is None or active_motion.motion_playing:
            raise RuntimeError("动作查询超时或正在播放动作，未发送模式切换")
        print("请求腰部 Auto；控制器可能让腰部回到默认 0°。", flush=True)
        reply = mode.call(SetInt32Request(data=1), timeout_sec=3)
        if reply is None or not reply.success:
            raise RuntimeError(f"SetWaistMode(Auto) 超时或被拒绝: {reply}")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            after = controller.call(GetControllerStateRequest(), timeout_sec=0.5)
            if (after is not None and after.current_controller == "amp"
                    and not after.controller_transitioning and after.waist_mode == 1):
                print("已确认 waist_mode=1（Auto）。")
                return
            time.sleep(0.05)
        raise RuntimeError("Auto 请求已发送，但实际模式未确认，请现场检查")
    finally:
        for requester in reversed(clients):
            requester.close()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as error:
        print(f"恢复失败: {error}", file=sys.stderr)
        sys.exit(1)
