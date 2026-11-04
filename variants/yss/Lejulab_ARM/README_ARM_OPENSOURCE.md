# LejuLab-Platform 开源版

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Platform](https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-orange.svg)](https://ubuntu.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/)
![Roban 2.1](https://img.shields.io/badge/Robot-Roban%202.1-green.svg)
![Roban 2.2](https://img.shields.io/badge/Robot-Roban%202.2-green.svg)
![Kuavo 4 Pro](https://img.shields.io/badge/Robot-Kuavo%204%20Pro-green.svg)
![Kuavo 5](https://img.shields.io/badge/Robot-Kuavo%205-green.svg)

乐聚人形机器人控制与强化学习部署平台开源发布包，支持 Roban、Kuavo 4、Kuavo 5 的
仿真与实机运行。开放的业务源码与 CI 预编译闭源程序、SDK 和第三方运行库组合发布，
预编译内容统一位于 `installed/`。

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
| Roban 2.2 降本实机 | ARM64 / RK3588 | 开放源码 + CI 预编译库 | Shell 启动脚本 |

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
installed/                   # CI 预编译程序、SDK、头文件和动态库
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

`installed/` 已提供闭源程序、LejuSDK、DDS、iceoryx、ONNX Runtime、SDL3 和 MuJoCo 等
预编译内容，不需要用户重新编译这些第三方库。ARM64 的 CMake 会直接使用其中的头文件和
库，编译前不需要加载额外环境。

推荐安装可选构建工具：

```bash
sudo apt install -y ccache ninja-build
```

### x86 研扬平台（Roban、Kuavo 4/5，ROS1/catkin）

适用于原有 ROS Noetic 实机和 `roslaunch` 流程：

```bash
export ROBOT_VERSION=46       # Roban 2.2 使用 17；Kuavo 5 使用 52/53/54
source /opt/ros/noetic/setup.bash
source installed/setup.bash
catkin build
source devel/setup.bash
```

`/opt/ros/noetic/setup.bash` 和 `installed/setup.bash` 用于首次编译环境准备。编译完成后，
每个新终端运行 ROS 节点前只需执行 `source devel/setup.bash`。

### ARM64/RK3588 Roban 2.2（开放源码 + CI 预编译库）

直接编译发布包中的开放源码：

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
| `BUILD_TESTS` | `ON` | 构建开放源码附带的测试 |
| `BUILD_EXAMPLES` | `ON` | 构建 LejuSDK 示例 |
| `BUILD_JOYSTICK` | `ON` | 构建手柄模块 |
| `BUILD_VR_CONTROL` | `ON` | 构建开放的完整 VR 链路，需要目标架构 Drake |

RL 控制器、硬件节点、MuJoCo 和 recorder 直接使用 `installed/bin/` 中的预编译程序，
不参与开放源码构建。

`installed/` 不是完整的可运行工作区。首次使用仍须执行上面的 `cmake ..` 和
`make -j$(nproc)`，完成开放源码包及手柄自启动管理程序等组件的编译，不能跳过编译直接
启动机器人。

### 主要编译产物

CI 预编译产物位于 `installed/`，开放源码的增量编译产物位于 `build/`：

```text
installed/
├── bin/                       # 闭源程序和预编译业务程序
├── lib/                       # LejuSDK、DDS 与第三方动态库
├── include/                   # 对外公开头文件
└── setup.bash                 # 运行环境入口

build/
└── src/...                    # 开放源码的编译产物
```

完成编译后，启动脚本会依次从 `build/`、`installed/` 和 `PATH` 查找程序，通常不需要
手动运行二进制。

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

完成编译和手柄部署后，仓库内的 Shell 启动脚本会自动查找 `build/` 与 `installed/bin/`。
ARM64 标准启动流程不需要执行 `source installed/setup.bash`。只有希望直接输入
`controller_cli` 等预编译命令时，才可在当前终端加载该脚本，将 `installed/bin/` 和
`installed/lib/` 加入环境。

权限规则：真机主程序使用 root 权限运行；x86 进入 root shell 后需要重新加载 ROS/catkin 环境，
ARM64 直接使用下面的 `sudo bash` 启动。
MuJoCo 仿真、VR 和 Controller CLI 不需要 `sudo`，其中 VR、Controller CLI 应在主程序启动后另开终端运行。

MuJoCo 仿真命令会同时启动控制器、仿真器和手柄节点。启动后：

- 根据终端提示按手柄 `start`。
- 点击 MuJoCo 窗口中的 `Run`。
- 如果初始状态下机器人倒地，依次点击 `Pause`、`Reset`，再点击 `Run`。

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
export ROBOT_VERSION=17
sudo bash src/leju_launch/scripts/launch_real.sh
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

ARM64 Shell 启动脚本会自行解析 `installed/`，不依赖 `sudo -E` 传递环境变量。
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

Roban 绝对式 VR 已实现手臂、头部、速度和灵巧手控制。ARM64 开源发布包含完整 VR 链路，
构建时需要目标架构的 Drake 等依赖。详细操作见 [VR 控制说明](src/leju-vr-control/README.md)。

### 键盘电机控制

```bash
export ROBOT_VERSION=17
sudo bash src/leju_launch/scripts/launch_keyboard_ctrl.sh
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
- [手柄使用说明](docs/howto-use-joystick.md)
- [电机零点标定](docs/howto-use-motor-cali-tool.md)
- [CAN 参数工具](docs/howto-use-can-param-tool.md)
- [模型部署](docs/lejulab-model-deployment.md)

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
