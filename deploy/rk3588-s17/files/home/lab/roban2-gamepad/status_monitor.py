#!/usr/bin/env python3
"""Read-only status stream. Never sends start/stop/mode/motion requests."""
import os
import signal
import time

os.environ.setdefault("CYCLONEDDS_HOME", "/home/lab/Lejulab_ARM/installed")
from cyclonedds.domain import DomainParticipant
from _rpc_lib import (
    RpcRequester, RemoteExceptionCode, GetRuntimeStateRequest, GetRuntimeStateReply,
    GetControllerStateRequest, GetControllerStateReply,
)


def main():
    # Exit when the owning C++ process closes its read pipe.
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    dp = DomainParticipant(0)
    runtime = RpcRequester(dp, "ControllerService/GetRuntimeState",
                           GetRuntimeStateRequest, GetRuntimeStateReply)
    controller = RpcRequester(dp, "ControllerService/GetControllerState",
                              GetControllerStateRequest, GetControllerStateReply)
    try:
        while True:
            ready, amp, transitioning, mode = 0, 0, 1, 0
            started = time.time()
            r = runtime.call(GetRuntimeStateRequest(), timeout_sec=0.1)
            c = controller.call(GetControllerStateRequest(), timeout_sec=0.1)
            if (r is not None and c is not None
                    and r.header.remote_ex == RemoteExceptionCode.REMOTE_EX_OK
                    and c.header.remote_ex == RemoteExceptionCode.REMOTE_EX_OK
                    and c.arm_mode in (0, 1, 2)):
                ready = int(r.data_ready and r.hardware_state == 2 and r.lifecycle_state == 2)
                amp = int(c.current_controller == "amp")
                transitioning, mode = int(c.controller_transitioning), c.arm_mode
            print(started, ready, amp, transitioning, mode, flush=True)
            time.sleep(0.05)
    finally:
        runtime.close()
        controller.close()


if __name__ == "__main__":
    main()
