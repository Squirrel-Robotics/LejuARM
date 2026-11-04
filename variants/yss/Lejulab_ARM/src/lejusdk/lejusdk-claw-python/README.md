# 夹爪 Python / WebSocket SDK

公开 `LejuClaw` 接口从 kuavo-ros-control 的 Python SDK 迁移，保留
`control`、`control_left`、`control_right`、`open`、`close`、状态读取、等待完成接口。
平台已有控制器负责单侧目标保留，不再复制 ROS SDK 的队列与另一份目标缓存。
两种传输共用同一个 Python 类，原生桥接只调用平台 C++ DDS SDK，不初始化硬件。

## 构建与本地使用

在仓库根目录构建 `lejusdk_claw` 目标（纯 CMake 或 catkin 均可）。
以 `build_claw_arm` 为例：

```bash
cmake --build build_claw_arm --target lejusdk_claw -j2
export ROBOT_VERSION=17
export PYTHONPATH="$PWD/src/lejusdk/lejusdk-claw-python/python:${PYTHONPATH:-}"
export LEJUSDK_CLAW_LIBRARY="$PWD/build_claw_arm/src/lejusdk/lejusdk-claw-python/liblejusdk_claw.so"
```

```python
from lejusdk_claw import LejuClaw, EndEffectorSide
claw = LejuClaw()
print(claw.get_state())  # 未收到状态时抛出异常，不伪造到位状态
claw.control_left([30])
claw.control_right([70])
claw.close(EndEffectorSide.BOTH)
claw.open(EndEffectorSide.BOTH)
print(claw.get_target())  # 按名称合并的真实下发目标
claw.disconnect()        # 释放连接，与 close（闭爪）不同
```

`control*` 可传 `is_high_freq=True`；连续流由调用方按周期发送，保持原显式模式语义。
发送停止后由驱动租约处理，不在 Python 中自动重发旧目标。
速度默认 90%，电流默认 1A，沿用源 Python SDK。
`control/open/close` 返回发布结果，不是硬件接受或执行确认。
`wait_for_finish` 仅根据状态判定完成，不保证机械目标位置误差。
单侧 `open/close` 只下发该侧，另一侧交由硬件控制器保留。

人工恢复由 `claw.recover("toward_open", current=1.5, duration_ms=100, repeat=1)`
调用，等待最多 10 秒并返回 `success/message/request_id`。
这会驱动双爪，确认现场与方向后才调用；结果超时不表示动作一定未执行，不能盲目重试。

## WebSocket

可选依赖 `websockets>=12,<14`。Python 包可通过本目录的 pyproject.toml 安装，
或使用上面的 PYTHONPATH 加载；安装 Python 包不会替代 C++ 桥接库构建。

在运行 DDS SDK 的机器配置上述环境后启动：

```bash
python3 -m lejusdk_claw.websocket_server --host 127.0.0.1 --port 8765
```

客户端替换传输即可，公开控制接口不变：

```python
from lejusdk_claw import LejuClaw, WebSocketTransport
claw = LejuClaw(WebSocketTransport("ws://127.0.0.1:8765"))
print(claw.get_state())
claw.disconnect()
```

远程客户端只需要 Python 包和 WebSocket 依赖，不加载本机 C++ 库。
默认仅绑定回环地址；跨机器使用建议 SSH 转发该端口。
本服务不包含认证，不应直接开放到不受控网络。

协议：每次发送一个 JSON 对象，每次收到一个 JSON 结果。
操作为 `control/state/target/recover`；成功信封为 `{"ok": true, "data": ...}`，
失败信封为 `{"ok": false, "error": "..."}`。
`control` 数据字段：`name/position/velocity/effort/is_high_freq`，名称为
`left_claw/right_claw`；`recover` 字段：`direction/current/duration_ms/repeat`。
本模块只迁移夹爪 API，不承诺与整个 kuavo_humanoid_websocket_sdk 的全机器人协议兼容。

## 开发验证

已做无硬件 C++ 编译及隔离回环网络的 Python 原生、WebSocket→DDS 往返检查，
覆盖单侧命令、高频标志、开合、双侧分帧目标合并与恢复请求编号匹配。
测试没有初始化 CAN 设备，不代表新增功能已完成真机验收。
