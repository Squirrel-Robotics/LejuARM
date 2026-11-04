# 夹爪硬件与通信链路接入

基线：platform bf65ce66977b0ee48a06b9c2b21642a324842cc0；控制器来源 roscontrol 9ca0b7b880188faa1b686b48e906c032c36124d7。

## 本次接入

- HardwarePlant::HWPlantInit 按末端类型创建夹爪，标定模式不触发夹爪初始化。
- controlLejuClaw(req, res, is_high_freq) 转交原请求响应类型；getLejuClawState(state) 返回原 ClawState。
- 后端选择跟随平台手臂：已知 single_bus/dual_bus 使用统一 CAN，UNKNOWN 使用旧 Ruiwo 后端。
- 控制器迁入原单侧目标缓存、显式高频参数，限位处理沿用平台实现；高频任务覆盖的占位判断保留。
- 忙碌判断、任务锁范围、工作线程、初始化和关闭顺序与 roscontrol 对应实现保持一致。
- 驱动保留平台已有运动控制参数，迁入源驱动断流看门狗、人工恢复、真实下发目标回调及 CAN 非高频保持模式。
- 硬件层检查夹爪实例是否可用，并向调用方返回初始化结果。

## SDK 与 DDS 接入

命令链路：上层 SDK → /rt/claw_cmd → HardwareNode → HardwareInterface::SetClawCommand → HardwarePlant::controlLejuClaw → 原控制器。

状态链路：原 ClawState → HardwareInterface::GetClawState → HardwareNode → /rt/claw_state → 上层 SDK 回调。

- 原夹爪类型统一定义在 lejusdk_hw/claw_types.h，控制器原路径保留转接头；硬件 SDK 不增加另一套夹爪结构。
- 上层通信命令沿用 name、position、velocity、effort，并传递 is_high_freq；状态对应原字段和有符号动作状态。
- DDS 使用 ClawCommand.idl、ClawState.idl 及配套工具生成的 C++ 文件。
- HardwareNode 仅做字段转换和调用，状态由独立 20Hz 非实时循环发布；原灵巧手十二维话题和接口不变。
- 未传入模式参数时默认使用低频模式；VR 输入适配在上层完成。

上层调用示例：

```cpp
leju::GlobalRobot::init_env(leju::RobotVersion(1, 7));
auto& robot = leju::GlobalRobot::getInstance();
robot.subscribeClawState([](const leju::ClawStateConstPtr& state) {
    // 读取 state->position、state->state 等原夹爪状态字段。
});
leju::ClawCmd command;
command.name = {"left_claw"};
command.position = {70.0};
command.velocity = {50.0};
command.effort = {1.0};
command.is_high_freq = false;
robot.publishClawCmd(command);
```

发布成功表示 SDK 通信层成功发送，不代表动作完成；最终动作由状态反馈观察。速度和 effort 留空时，由原控制器使用原默认值。

## 高频断流看门狗

- 控制器只在请求通过检查并成功入队后刷新高频看门狗，接受低频命令时解除武装；初始化后和关闭驱动前解除武装。
- CAN、旧 Ruiwo 后端沿用 roscontrol 的看门狗状态、计时及周期发送处理，默认超时 500 ms。配置文件 `~/.config/lejuconfig/lejuclaw_config.yaml` 的 `lejuclaw.stream_watchdog_timeout_ms` 可覆盖；小于等于零时使用默认值。
- 断流时周期线程持续发送安全 PTM：保留最后目标位置及 kd，速度、力矩、kp 清零；不是继续保持原位置增益。
- 运动循环遇到断流返回内部 `STREAM_EXPIRED`；控制器保留当前动作状态，不把断流报告为到位。此返回值不新增为 DDS 动作状态码。
- 后续有效高频命令刷新看门狗并恢复正常发送；DDS 和 HardwareNode 不另设计时器。
- 保留平台原有限幅、忙碌判断和高频占位分支；扩展功能及接口详见 [夹爪扩展功能](claw_additional_features.md)，不整体覆盖无关控制参数。

## 验证范围

Roban2.2-1 已完成原夹爪链路的 ARM 构建、双爪初始化、控制器高低频、SDK/DDS 完整链路和 LB+X 手柄开合验证。
本次新增的人工恢复、保持电流、目标观测、Python/WebSocket 与 VR 扳机适配通过本地开发检查，仍需重新进行 ARM 构建及对应实机验收。
旧 Ruiwo 后端没有完成本次新增能力的实机验收。

## 开发阶段检查

- x86 容器中定向编译并链接 leju-hardware 通过；两个独立 SDK 进程在回环网络上的命令与状态回环通过，原十二维手部通道回环通过。

本地开发检查不启动真实 HardwareNode，不访问 CAN，不能替代真实寻零、限流或机械动作验收。

- 新增恢复工具、硬件节点、Python 原生桥接、两个 VR 节点，以及正常启动使用的 run_rl_controller、leju-joystick 已在 x86 容器编译链接。
- Python 本地及 WebSocket→DDS 回环覆盖单侧目标、模式、开合、目标分帧合并和恢复结果匹配。
- 保持配置解析及切入高频、恢复时解除保持通过无设备检查；VR 扳机映射及相同输入持续发布通过回环检查。
- 本次公开 C++ SDK 增加了虚函数，更新时须重新编译同一部署目录的使用方，不可只替换共享库并继续使用旧二进制。

## 6.x 兼容性说明

- 控制器已删除 `update_vr_detection`：旧调用方按调用间隔自动进入 VR/高频模式的行为不再存在。原依赖该行为的调用方必须显式传 `is_high_freq=true`；默认低频命令不会武装断流看门狗，也会被在途低频任务拒绝。
- 夹爪百分比目标两端统一限制在 5~95；旧调用方传 0/100 会分别钳到 5/95，开合命令应使用 `open`(5) / `close`(95)。
