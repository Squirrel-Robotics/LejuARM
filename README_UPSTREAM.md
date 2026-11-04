# LejuLab-Platform

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Platform](https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-orange.svg)](https://ubuntu.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/)
![Roban 2.1](https://img.shields.io/badge/Robot-Roban%202.1-green.svg)
![Roban 2.2](https://img.shields.io/badge/Robot-Roban%202.2-green.svg)
![Kuavo 4 Pro](https://img.shields.io/badge/Robot-Kuavo%204%20Pro-green.svg)
![Kuavo 5](https://img.shields.io/badge/Robot-Kuavo%205-green.svg)

乐聚人形机器人控制与强化学习部署平台，支持 Roban、Kuavo 4、Kuavo 5 的仿真与实机运行。
仓库同时覆盖 x86 研扬平台、x86 本地仿真和 ARM64/RK3588 降本平台。

## 功能

- MuJoCo 仿真与实机控制
- RL、Mimic、Dummy、键盘等控制器
- 手柄与 Quest3 VR 输入
- DDS 通信、数据录制和系统监控
- CAN、电机标定和诊断工具
- LejuSDK 控制接口与示例

```text
手柄 / VR → ControllerManager → LejuSDK + DDS → 实机硬件 / MuJoCo
```

## 支持平台与机型

| 使用场景 | 架构 | 构建方式 | 启动方式 |
|---|---|---|---|
| Roban 2.1/2.2、Kuavo 4/5 原有 ROS1 实机 | x86_64 研扬平台 | catkin | `roslaunch` |
| Roban 2.2 本地仿真 | x86_64 开发机 | 纯 CMake | Shell 启动脚本 |
| Roban 2.2 降本实机 | ARM64 / RK3588 | 纯 CMake | Shell 启动脚本 |

| 机型 | `ROBOT_VERSION` 示例 | 编译与启动 |
|---|---:|---|
| Roban 2.1 | `14` | x86 ROS1：catkin + `roslaunch` |
| Roban 2.2 | `17` | x86 ROS1，或 x86/ARM64 降本 |
| Kuavo 4 Pro/UAE | `46` | x86 研扬：catkin + `roslaunch` |
| Kuavo 5 系列 | `52`、`53`、`54` | x86 研扬：catkin + `roslaunch` |

`ROBOT_VERSION` 不可省略。临时设置可执行 `export ROBOT_VERSION=17`（按实际机型修改）；
如果已将同一命令写入当前运行用户的 `~/.bashrc` 并重新加载，后续编译和启动命令无需重复设置。

## 目录结构

```text
src/
├── leju_assets/            # URDF、MJCF、场景和机器人资源
├── leju-controllers/       # RL、Dummy、键盘控制器
├── leju-hardware/          # 硬件节点、CAN、IMU 和设备配置
├── leju-joystick/          # 手柄输入与自启动服务
├── leju_launch/            # ROS launch、DDS 配置和 Shell 启动脚本
├── leju-mujoco-sim/        # MuJoCo 仿真器
├── leju-audio/             # 音频采集与播放
├── leju-monitor/           # 系统状态监控
└── lejusdk/                # SDK、DDS 通信、录制工具和示例
```

## 编译

根据机器类型只选择下面一种方式。

推荐安装可选构建工具：

```bash
sudo apt install -y ccache ninja-build
```

### x86 研扬平台（Roban、Kuavo 4/5，ROS1/catkin）

适用于原有 ROS Noetic 实机和 `roslaunch` 流程：

```bash
export ROBOT_VERSION=46       # Roban 2.2 使用 17；Kuavo 5 使用 52/53/54
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

`/opt/ros/noetic/setup.bash` 只用于首次编译环境准备。编译完成后，每个新终端运行 ROS 节点前
只需执行 `source devel/setup.bash`。

### x86 Roban 2.2 降本仿真（纯 CMake）

本节只用于在 x86 开发机仿真 Roban 降本运行方式。原有 ROS1 版本使用上一节的 catkin
流程，不需要设置 `FORCE_PURE_CMAKE=ON`。

```bash
export ROBOT_VERSION=17
mkdir -p build
cd build
cmake .. -DFORCE_PURE_CMAKE=ON
make -j$(nproc)
```

`FORCE_PURE_CMAKE=ON` 用于在安装了 ROS/catkin 的 x86 开发机上强制走纯 CMake，模拟
降本版构建方式。`BUILD_MUJOCO_SIM` 默认开启，不需要重复指定。

### ARM64/RK3588 Roban 2.2（纯 CMake）

完整源码仓库直接使用纯 CMake 编译：

```bash
export ROBOT_VERSION=17       # 按实际机型修改

mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

常用 CMake 开关如下，快速使用保持默认值即可：

| 选项 | 默认值 | 说明 |
|---|---:|---|
| `FORCE_PURE_CMAKE` | `OFF` | x86 上强制忽略 catkin |
| `BUILD_RL_CONTROLLER` | `ON` | 构建 RL 控制器 |
| `BUILD_JOYSTICK` | `ON` | 构建手柄模块 |
| `BUILD_MUJOCO_SIM` | `ON` | 完整源码中构建 MuJoCo |
| `BUILD_EXAMPLES` | `ON` | 构建示例 |
| `BUILD_TESTING` | `ON` | 构建测试 |

### 主要编译产物

以上命令默认输出到 `build/`；如果配置时使用 `build_cmake/`，目录结构相同：

```text
<build-dir>/
├── src/leju-hardware/leju-hardware
├── src/leju-controllers/leju-dummy-controller/run_dummy_controller
├── src/lejusdk/lejusdk-recorder/recorder/lejusdk_recorder
└── src/.../*.so
```

启动脚本会优先查找 `build/`，不存在时再查找 `build_cmake/`，通常不需要手动运行这些二进制。

## ARM64/RK3588 手柄自启动部署（重要）

实机首次使用时，在仓库根目录部署手柄自启动服务：

```bash
# 已在当前用户 ~/.bashrc 中设置 ROBOT_VERSION 时，直接执行：
sudo bash src/leju-joystick/services/deploy_joy_autostart.sh

# 仅临时设置时：
export ROBOT_VERSION=17
sudo bash src/leju-joystick/services/deploy_joy_autostart.sh
```

部署脚本会编译工作区并安装手柄自启动服务；已有现场手柄配置时，按终端提示选择是否覆盖。

部署完成后检查服务状态：

```bash
systemctl status lejulab_joy_monitor.service
```

## 运行

权限规则：现有真机部署使用 root 用户运行；进入 root shell 后需要重新加载运行环境。
MuJoCo 仿真、VR 和 Controller CLI 不需要 `sudo`，其中 VR、Controller CLI 应在主程序启动后另开终端运行。

MuJoCo 仿真命令会同时启动控制器、仿真器和手柄节点。启动后：

- 根据终端提示按手柄 `start`。
- 点击 MuJoCo 窗口中的 `Run`。
- 如果初始状态下机器人倒地，依次点击 `Pause`、`Reset`，再点击 `Run`。

### x86 纯 CMake 仿真

编译完成后回到仓库根目录：

```bash
cd ..
export ROBOT_VERSION=17
export BUILD_DIR=build
bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm
```

`BUILD_DIR=build` 指定使用刚编译的 `build/` 产物，避免自动选择旧的 `build_cmake/`；此相对路径需在仓库根目录下使用。

`--no-shm` 适合没有部署 iceoryx RouDi 的本地开发机；RouDi 已运行时可以省略。

### x86 研扬平台（Roban、Kuavo 4/5，ROS1）

```bash
source devel/setup.bash
export ROBOT_VERSION=46       # Roban 2.2 使用 17；Kuavo 5 使用 52/53/54
roslaunch leju_launch load_mujoco_sim.launch
```

实机在仓库根目录进入 root shell 后启动：

```bash
sudo su
source devel/setup.bash
export ROBOT_VERSION=46       # Roban 2.2 使用 17；Kuavo 5 使用 52/53/54
roslaunch leju_launch load_real.launch
```

Kuavo 4/5 的 RL demo 仿真：

```bash
roslaunch leju_launch load_mujoco_sim_rl_demo.launch
```

RL demo 实机同样进入 root shell，并重新加载环境：

```bash
sudo su
source devel/setup.bash
export ROBOT_VERSION=46       # Kuavo 5 使用 52/53/54
roslaunch leju_launch load_real_rl_demo.launch
```

### ARM64/RK3588 Roban 2.2 实机

```bash
sudo su
export ROBOT_VERSION=17
bash src/leju_launch/scripts/launch_real.sh
```

可用 Shell 入口：

| 脚本 | 用途 |
|---|---|
| `launch_real.sh` | 实机：硬件、手柄、控制器和录制 |
| `launch_real_rl_demo.sh` | 实机 RL demo |
| `launch_real_mimic_demo.sh` | 实机 Mimic demo |
| `launch_mujoco_sim.sh` | MuJoCo 仿真 |
| `launch_mujoco_sim_rl_demo.sh` | MuJoCo RL demo |
| `launch_mujoco_sim_mimic_demo.sh` | MuJoCo Mimic demo |
| `launch_keyboard_ctrl.sh` | 键盘电机控制 |

不要依赖 `sudo -E` 传递 `LD_LIBRARY_PATH`；进入 root shell 后重新执行对应的 `source`。
真机启动前必须确认急停、机器人版本、CAN 接线、零点和配置文件均正确。

### 实机首次运行检查

- 确认 `ROBOT_VERSION`、硬件配置和实物机型一致，急停可随时触发，机器人周围无人。
- 出厂机器人通常已完成零点标定；出现以下情况时重新标定：
  - 准备站立时，发现关节角度与零点位置存在偏差。
  - 硬件维修并更换过电机。
- 标定方法见[电机零点标定工具](./docs/howto-use-motor-cali-tool.md)。
- 固定机器人或使用移位架，确认急停有效后拉起机器人背后的急停按钮。
- 将移位架升起，等待机器人进入膝盖微曲状态。
- 降低移位架，让机器人脚掌刚好接触地面。
- 使用手柄 `start` 按键使机器人站立。
- **结束使用机器人时按手柄 `back` 按键安全停机。**

## 控制器与输入设备

### 切换控制器

控制器配置位于：

```text
src/leju-controllers/leju-rl-controller/config/<ROBOT_VERSION>/controller_manager.yaml
```

常用控制器包括 AMP 行走、Mimic 动作和保持姿态。修改 `default_controller` 后重新启动。

### 手柄

![手柄按键](./docs/images/joystick.png)

- `start`：从待机进入站立/运行
- `back`：安全停机并使关节松弛
- 左摇杆：平移；右摇杆：转向
- 其他按键和摇杆根据当前控制器（如 RL demo、Mimic）实现行走、转向或舞蹈等功能，
  具体映射由对应版本的 `teleop_bindings.yaml` 定义

详细说明：[手柄使用说明](./docs/howto-use-joystick.md)。

### Quest3 VR

先按上面的实机或仿真章节启动主程序。下面的命令只启动 VR 控制节点，不会启动机器人硬件，
不需要 `sudo`，直接以当前用户运行。

当前 `vr_teleop.launch` 的启动路由如下：

| 机型 | 控制方式 | 启动参数 |
|---|---|---|
| Roban 2.1/2.2（14/17） | Roban 专用绝对式节点 | `use_incremental_ik:=false`（默认） |
| Kuavo 4/5（46、52/53/54） | Kuavo 通用节点的增量式模式 | `use_incremental_ik:=true` |

Kuavo 通用节点内部同时实现了绝对式和增量式 IK，但当前 launch 在参数为 `false` 时启动的是
Roban 专用绝对式节点；Roban 不支持增量式节点。因此快速使用按上表选择。

x86 ROS1 环境中按机型启动：

```bash
source devel/setup.bash

# Roban 2.2（Roban 2.1 改为 14）
export ROBOT_VERSION=17
roslaunch leju_launch vr_teleop.launch use_incremental_ik:=false

# Kuavo 4（Kuavo 5 改为 52/53/54）
export ROBOT_VERSION=46
roslaunch leju_launch vr_teleop.launch use_incremental_ik:=true
```

指定 Quest3 IP 时，保留对应机型的控制方式：

```bash
# Roban 14/17
roslaunch leju_launch vr_teleop.launch use_incremental_ik:=false quest_ip:=192.168.1.100

# Kuavo 4/5
roslaunch leju_launch vr_teleop.launch use_incremental_ik:=true quest_ip:=192.168.1.100
```

#### Kuavo 4/5 增量式操作方式

- `X+A`：在 `Auto(1)` 和 `External(2)` 之间切换
- `X+B`：在 `KeepPose(0)` 和 `External(2)` 之间切换
- `X+Y`：进入安全停机/关节松弛状态
- 左/右 `grip`：对应手臂进入增量控制
- 头部：由头显姿态驱动
- 左摇杆：控制前后左右平移，原始范围 `[-1, 1]`
- 右摇杆：控制左右转向，原始范围 `[-1, 1]`

三个手臂模式的效果如下：

| 模式 | 值 | 效果 |
|---|---:|---|
| `KeepPose` | `0` | 保持当前手臂姿态，适合临时停在当前位置 |
| `Auto` | `1` | 交还给默认自动行为（行走时摆臂），手臂不再接受当前 VR 增量控制 |
| `External` | `2` | 进入 VR 外部控制模式，按住对应 `grip` 后可进行增量控制 |

Roban 绝对式 VR 已实现手臂、头部、速度和灵巧手控制。ARM64 构建完整 VR 链路时，
需要目标架构的 Drake 等依赖。详细操作见 [VR 控制说明](src/leju-vr-control/README.md)。

### 键盘电机控制

```bash
sudo su
export ROBOT_VERSION=17
bash src/leju_launch/scripts/launch_keyboard_ctrl.sh
```

`launch_keyboard_ctrl.sh` 会启动硬件节点、录制器，并在前台运行 `motor_keyboard_ctrl`；
下面的按键由这个前台键盘控制器处理。该工具用于单电机调试，实机操作前必须确保机器人
已固定并可随时急停。

**按键说明：**

| 按键 | 功能 | 按键 | 功能 |
|---|---|---|---|
| `0-9`/`a-m` | 选择电机 | `W`/`S` | 位置 +/- |
| `[`/`]` | 步长减半/翻倍 | `Space` | 全部回零 |
| `T` | 正弦测试开关 | `E`/`R` | 正弦幅度 -/+ |
| `F`/`G` | 正弦频率 -/+ | `P` | 打印详细位置 |
| `Q` | 退出（自动回零并失能） | | |

## LejuSDK、录制与可视化

- LejuSDK 入口：[src/lejusdk/README.md](src/lejusdk/README.md)
- MCAP 录制配置：`src/leju_launch/config/recorder.yaml`
- 运行数据：`~/.ros/lejulab/{stdout,mcap,coredumps}`
- Foxglove 布局：`config/foxglove-*-layout.json`
- MCAP 分析工具：[scripts/mcap/README.md](scripts/mcap/README.md)
- Controller CLI、SDK 示例位于 `src/lejusdk/examples/`

## 系统诊断

```bash
# 本机全链路检查
sudo bash scripts/diagnostics/check_all.sh

# 从开发机检查板端
bash scripts/diagnostics/check_all.sh --remote <RK3588_IP>
```

专项检查位于 `scripts/diagnostics/`，覆盖 CAN、DDS、硬件节点、实时环境和电机通信。

## 测试

```bash
# 纯 CMake 构建已默认开启测试
cd build
make -j$(nproc)
ctest --output-on-failure
```

## 常用文档

- [x86 Docker 仿真](docs/howto-sim-docker.md)
- [ARM64 交叉编译](docs/howto-cross-compile-docker.md)
- [跨机器 RL/MuJoCo 联调](docs/cross-machine-rl-mujoco-guide.md)
- [手柄使用说明](docs/howto-use-joystick.md)
- [电机零点标定](docs/howto-use-motor-cali-tool.md)
- [CAN 参数工具](docs/howto-use-can-param-tool.md)
- [模型部署](docs/lejulab-model-deployment.md)

## tact 播放结束后保持手臂姿态

在启动使用的 `controller_manager.yaml` 顶层设置，重启控制器生效：

```yaml
tact_player:
  hold_after_tact: true  # true：播完保持；false：播完回正（缺省值）
```

动作最后一帧应为希望保持的姿态，文件内自带的回正动作仍会执行。
启动上位机 ROS-DDS bridge 和下位机控制器后，播放下位机已有的动作：

```bash
rosservice call /play_tact_action "{motion_name: '实际动作名', target_execution_time_ns: 0}"
rostopic echo /tact_state  # 查看完成状态；服务成功仅表示已启动
```

## 常见问题

### iceoryx RouDi 未运行

本地仿真可直接禁用共享内存：

```bash
bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm
```

实机需要共享内存时，部署并检查服务：

```bash
sudo bash src/leju_launch/scripts/setup_cyclonedds_config.sh
systemctl status leju-roudi
```
