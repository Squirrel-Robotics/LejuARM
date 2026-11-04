# 夹爪扩展功能

源实现：kuavo-ros-control `9ca0b7b880188faa1b686b48e906c032c36124d7`。
平台保留已有运动参数，仅迁移功能逻辑和必要依赖。

## 人工卡死恢复

C++ SDK 发布 `ClawRecovery` 到 `/rt/claw_recovery`，订阅
`ClawRecoveryResult`（`/rt/claw_recovery_result`）按 `request_id` 匹配执行结果。
发布成功不表示恢复成功。方向支持 `toward_open`、`toward_close`、
`stuck_open`、`stuck_close`、`stop`，沿用源仓库语义。
`stuck_open` 向关爪方向冲击，`stuck_close` 向开爪方向冲击。
恢复接管双爪，取消普通任务、解除流租约、清零电流；电流上限 3A，
单次最长 500ms，最多重复三次。仅 CAN 后端支持。

硬件 SDK 提供 `RecoverClaw(direction, current, duration_ms, repeat, message)`。
初始化无法通过时，可在停止所有其他夹爪控制进程后使用独立工具
`lejuclaw_can_recovery_pulse`。工具恢复初始化不寻限位，可能使双爪运动，
使用前确认运动方向和现场条件；不可与 HardwareNode 同时控制总线设备。
工具参数及确认流程沿用源仓库，恢复后重新启动正常初始化。

构建目标：`leju-hardware`、`lejuclaw_can_recovery_pulse`。

## 真实下发目标观测

驱动真实发送点通过 `set_tx_callback` 上报最终参数，控制器沿用源仓库
`setTargetCallback` 统一左右爪命名；HardwareNode 异步发布 `/rt/claw_target`。
C++ SDK 使用 `subscribeClawTarget` 读取，字段为 name/position/velocity/effort/kp/kd/zero_kp。
它是原 `/leju_claw_target` 的 DDS 对应通道，不替换 `/rt/claw_state` 实际位置反馈。
CAN 回调逐电机发送，串口回调按索引数组发送；消费者按 name 更新，不假设每帧双侧齐全。
`zero_kp` 表示安全 PTM，串口消费端按此标志将 kp 视为零。
这是驱动发送前的观察口，不表示电机确认接收或执行成功。

## 非高频保持电流

沿用源 CAN 后端配置 `enable_non_vr_hold`（默认 false）、
`non_vr_hold_current`（默认 0.4A）、`non_vr_hold_trigger_percents`（默认字符串 "100.0"）。
运行时读取 `~/.config/lejuconfig/lejuclaw_config.yaml`；存在旧文件时不会自动覆盖，需按需添加配置。
低频目标命中触发集合后持续保持，目标退出集合则解除；保持行为不依赖持续发送命令。
切入高频和人工恢复时解除低频保持，避免保持输出覆盖新模式或恢复零电流。
LB+X 闭合为 95%，默认触发值 100% 不会触发；需要保持时应明确配置触发值，不能只打开开关。
旧 Ruiwo 后端未实现此项，与源仓库一致。

## Python / WebSocket

新增 `src/lejusdk/lejusdk-claw-python`，迁移源 `LejuClaw` 公共接口，
本地传输经 C ABI 复用 C++ DDS SDK，WebSocket 共用同一类和 JSON 操作，
没有另写驱动、控制器或维护另一套单侧目标缓存。
构建 `lejusdk_claw` 并按该包 README 配置路径；WebSocket 为可选依赖。
可读取状态与真实目标，也可调用高低频控制及等待人工恢复结果。

## VR 扳机映射

绝对式 `quest_vr_abs_control_node` 与增量/统一节点 `quest_vr_control_node`
均订阅双侧夹爪状态。收到有效双爪状态后，末端从 HandCmd 切换到 ClawCmd；
未配置双夹爪的机器保留原灵巧手路径，不引入另一份末端配置。
左右扳机对应左右夹爪；沿用源 JoyStickHandler 映射：乘 100、取整并限幅到 0~100，
速度 90%，effort=0 由原控制器使用默认 1A；显式设置 is_high_freq=true。
每帧新扳机输入均发送，数值相同时也发送；不在主循环用旧数据刷新租约。
输入停止后由已迁入的断流看门狗停止输出；绝对式原锁手逻辑也会停止夹爪命令刷新。
本项只接扳机，不包含手部骨骼追踪到夹爪的映射。

构建时启用 `BUILD_VR_CONTROL=ON`，构建两个对应节点。
本地验证不初始化真实硬件；实机仍需验收 VR 连接、同侧映射、扳机保持、锁手与断流停止。

## VR 扳机控制夹爪：构建与参数

- 构建开关：`BUILD_VR_CONTROL=ON`（默认 OFF 时 `leju-vr-control` 不参与构建；源码仓库开启是为了让 `scripts/cmake_configure.sh` 默认配置 VR 链路）。
  `scripts/build_env.sh` 的 `leju_cmake_configure_extra_args` 已默认带上该开关，
  可通过环境变量 `LEJU_BUILD_VR_CONTROL=OFF` 临时关闭。
- 启动：`roslaunch leju_launch vr_teleop.launch robot_version:=17`，
  节点 `quest_vr_control_node`（增量 IK）或 `quest_vr_abs_control_node`（绝对 IK，默认）。
- 扳机映射：`int(100 * trigger)` 限幅 0~100，速度 90，力矩 0（控制器按默认 1A）。
- 死区：夹爪扳机不设死区，按扳机原始值直接映射，`0` 即全开目标（下发前由控制器限幅到 5%）。
  早期版本的 `VR_CLAW_TRIGGER_DEADZONE` / launch 参数 `claw_trigger_deadzone`
  已随 `2a60beed` 回退，本版代码和 launch 均不再读取这两个名字。
