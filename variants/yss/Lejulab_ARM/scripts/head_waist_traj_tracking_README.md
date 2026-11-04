# ROBAN v17 头部、腰部轨迹跟踪测试

源码和 YAML 已放入下位机 `/home/lab/yss/Lejulab_ARM/scripts/`，程序分别为 `/home/lab/yss/Lejulab_ARM/build/head_traj_tracking_test` 和 `/home/lab/yss/Lejulab_ARM/build/waist_traj_tracking_test`。两个测试单独执行。此次完成了构建、离线验证和只读检查，未运行真实运动；本文不提供未经运动测试验证的实测误差。

已核对的可动关节如下。SDK 反馈下标从 0 开始，模型限位作为程序的硬限制。

| 部位 | 关节 | 自由度 | SDK 反馈下标 | 模型角度范围 |
| --- | --- | --- | --- | --- |
| 头部 | `zhead_1_joint` | yaw，左右转头 | 21 | −90° 至 +90° |
| 头部 | `zhead_2_joint` | pitch，俯仰；正值低头 | 22 | −20° 至 +45° |
| 腰部 | `waist_yaw_joint` | yaw，绕竖直轴转动 | 12 | −150° 至 +150° |

默认头部目标为 `target_deg: [40, 25]`，顺序是 yaw、pitch，表示到达绝对 40°、25°。默认腰部目标为 `target_deg: [60]`，使用 External/policy 坐标。当前 AMP 的腰部 `joint_direction=-1`，因此输入 policy +60° 对应 SDK −60°；程序从 SDK 反馈读取起点，乘当前配置中的 direction 转成 policy 坐标，再自动换算发布与验收目标。

1. 在下位机工作目录加载环境，使用已通过只读通信检查的 DDS 网络配置：

   ```bash
   cd /home/lab/yss/Lejulab_ARM
   export ROBOT_VERSION=17
   source installed/setup.bash
   export CYCLONEDDS_URI=file:///home/lab/roban2-gamepad/dds-network.xml
   ```

2. 如需重新编译，只执行独立构建脚本：

   ```bash
   bash scripts/build_head_waist_traj_tracking_tests.sh
   ```

   脚本优先使用 GCC 11，以 C++17 编译两个程序；两个程序均编译成功后才更新 `build/` 输出。它不启动运动，也不修改根 CMake 或已有手臂测试。

3. 先做离线配置校验，再做只读运行状态检查：

   ```bash
   build/head_traj_tracking_test --dry-run
   build/waist_traj_tracking_test --dry-run
   build/head_traj_tracking_test --inspect
   build/waist_traj_tracking_test --inspect
   build/waist_traj_tracking_test --preflight
   ```

   `--dry-run` 校验 YAML，不初始化 DDS。`--inspect` 查询 runtime、控制器、模式并订阅指令/反馈。`--preflight` 还检查配置修改时间、运行前提、动作播放冲突及起止角度，在模式切换前退出。这两种只读模式均不切换模式、不发布运动轨迹。此前只读检查已取得 `hardware=2`（ReadyOk）、`lifecycle=2`（Running）和当前控制器 `amp`；这些是检查时的状态，正式测试会再次确认状态和反馈。

4. 头部可以按以下方式单独启动：

   ```bash
   build/head_traj_tracking_test
   ```

   当前官方头部接口直接发布 `/rt/head_trajectory`，无需切换 head mode。测试依次保持启动姿态 1 秒、平滑到绝对目标、保持 2 秒、平滑返回启动姿态，再保持 1 秒确认回程。头部 YAML 的请求移动时间为 5 秒。

5. 腰部测试前先启用运行控制器的腰部对象。该项初始为 `false`，用户已在 2026-10-13 18:59:58 改为 `true`。若文件中仍为 `false`，修改下位机现有文件 `/home/lab/yss/Lejulab_ARM/src/leju-controllers/leju-rl-controller/config/17/controllers/config_amp.yaml` 中的既有键：

   ```yaml
   HumanoidRobotCfg:
     env:
       robot:
         enable_waist_controller: true
   ```

   这里只展示键的层级，应修改原键并保留文件其余内容。然后按已有机器人初始化、START 流程重新初始化控制器，使该值加载到新实例中；不要在运行中使用 reload，本测试工具也不会重启机器人。现有流程见 [初始化与 START 说明](/media/yulinxie/EXTERNAL_USB/ROBAN/reference/Start_init.md)。

   启用该项也会影响非测试时 AMP 的 Auto 行为：Auto 站立会给腰部默认 0° 的 CSP 目标。它不只是本测试的一项参数。

   程序会比较 AMP 文件修改时间与匹配本项目启动命令的控制器进程启动时间。如果文件更晚，正常测试和 `--preflight` 会在切模式前拒绝运行，`--inspect` 则显示警告。普通账号可能无法读取 root 进程的可执行文件或网络命名空间；这时程序明确显示加载状态未验证，仍会拒绝已发现的配置修改后未重启的候选进程。文件时间检查不能代替后续 External 实际模式确认。

   确认初始化完成且只读检查正常后，单独执行：

   ```bash
   build/waist_traj_tracking_test --preflight
   build/waist_traj_tracking_test
   ```

   腰测试要求起始模式为 Auto，切换到 External 并查询确认实际生效后才发布 `/rt/waist_trajectory`。腰部 YAML 的请求移动时间为 7 秒，保持和回程流程与头部相同。正常结束先平滑回到启动姿态，再恢复原 Auto 模式；若启动腰姿态不是 0°，恢复 Auto 后仍可能继续回到 Auto 默认 0°。

运动测试时让机器人站稳，现场急停可用，并避免 VR、手柄或其他控制器同时向同一部位发送指令。程序要求 runtime ReadyOk、Running、数据就绪且控制器未切换，并检查是否有动作正在播放。腰部当前仅支持已核验的 AMP 配置。

参数分别在下位机 `/home/lab/yss/Lejulab_ARM/scripts/head_traj_tracking.yaml` 和 `/home/lab/yss/Lejulab_ARM/scripts/waist_traj_tracking.yaml`。每次启动重新读取 YAML，修改后下次启动即生效，无需重新编译；已经开始的一次测试沿用启动时读取的值。也可用 `--config PATH` 指定其他测试 YAML。`target_deg` 为绝对目标，`delta_deg` 为相对启动姿态的步进，两者必须且只能填写一项。

默认 `rate_hz=100`，指令峰值速度上限为 `max_velocity_deg_s=15`。`move_sec` 是请求的移动时间；程序根据实际起点与目标计算平滑轨迹所需时间，若请求时间无法满足 15°/s 上限会自动延长，所以 5 秒、7 秒不保证是实际完成时间。反馈超过 500 毫秒未更新，或实际指令与反馈误差超过 15°时，程序停止本测试发布并以失败退出。

目标保持段共 2 秒，跳过前 700 毫秒，对其余新样本统计：实际 `cmd` 和 `feedback` 角度均值、有符号 `cmd−feedback` 均值、绝对误差均值、最大绝对误差、`kp`、`kd`、执行模式 `mode` 和样本数。优先使用 `/rt/motor_cmd`、`/rt/motor_state`；电机话题不齐时使用关节话题。`kp/kd` 只观察，不修改；无有效字段时打印 `-`，执行模式 `mode=2` 表示 CSP。

程序另外打印 SDK 坐标下的预期轨迹目标，以及目标与 `/rt/joint_cmd`、`/rt/joint_state` 的平均绝对差，用来确认测试目标被实际控制链消费。执行层 `cmd−feedback` 很小并不单独代表已经到达测试目标。

只有目标保持段中的实际关节命令、关节反馈都通过目标容差，并且回程反馈确认到达启动姿态，程序才返回退出码 0。默认 `target_tolerance_deg=2`；对非零位移，目标验收容差取 `min(2°, |位移|/2)`，避免很小的运动请求被“完全没动”误判成功。回程使用配置中的起点容差。

按 Ctrl+C 会停止本测试轨迹发布；中断或异常时不会强行执行回程。腰部会尝试恢复启动前的 Auto 模式，控制器随后仍可能自行回到默认腰姿态。停止发布、停止本程序和硬件急停是不同操作，Ctrl+C 不等于现场急停。

若再次出现 `Segmentation fault`，使用已部署的调试启动脚本：

```bash
cd /home/lab/yss/Lejulab_ARM
bash scripts/debug_waist_traj_tracking_test.sh --preflight
# 完成初始化、通过只读预检后，以下命令会执行正常的腰部运动测试：
bash scripts/debug_waist_traj_tracking_test.sh
```

脚本用 GDB 保存所有线程的崩溃调用栈，日志路径会在启动时显示，目录为 `/home/lab/yss/Lejulab_ARM/build/tracking_debug/`。构建脚本现保留调试符号和栈帧；程序还打印动作查询和模式请求阶段。SDK 通信在正常及异常出口都显式关闭。段错误的具体原因尚未由调用栈证实，详见 [排查记录](/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_头腰轨迹测试/故障排查/段错误排查记录.md)。

资料与本地对照源：

- [官方 ROBAN2 产品介绍](https://roban.lejugym.com/roban2_manual/1产品介绍/Roban2%20灵动版产品介绍/)：标准本体自由度组成及关节范围。
- [官方头部、手臂、腰部及手部控制接口](https://roban.lejugym.com/roban2_manual/4开发接口/roban2-humanoid-sdk接口/头部手臂腰部及手部控制接口/)：部位轨迹发布与控制模式接口。
- 本地测试源码根目录：`/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_头腰轨迹测试/scripts/`，共享逻辑见 [traj_tracking_common.hpp](/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_头腰轨迹测试/scripts/traj_tracking_common.hpp)。
- 下位机原始手臂测试、YAML、URDF 和 AMP 配置归档：[remote_reference.tar.gz](/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_头腰轨迹测试/原始参考/remote_reference.tar.gz)。
- 本地已核验的下位机 VR/SDK 对照源根目录：`/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_16-09_VR头部跟随核验/`；链路说明见 [实机核验说明](/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_16-09_VR头部跟随核验/实机核验说明.md)。
- [验证记录](/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-13_头腰轨迹测试/validation/验证记录.md)：最初 46 项、更新后 92 项 C++ 纯逻辑检查以及 10 组 YAML 检查通过。更新后 20:43 新控制器实例的实机 `--preflight` 已通过；此前服务未启动时也正常失败并清理通信。隔离共享内存模式失败路径验证通过。AMP 已由用户修改，历史哈希记录不代表当前 AMP 内容。未向真实机器人发送运动或模式切换 RPC。
