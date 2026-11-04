# lejulab-platform aarch64 release

源码 + 预编译核心库和业务产物（stub/binary）混合发布。可开源的控制器以源码形式提供，
RL 控制器因依赖闭源 kuavo_solver 以预编译 binary 提供；
`lejusdk-lowlevel` / `lejusdk-vr` 以预编译 `.so` + 头文件提供（藏实现，对齐上游）。

## 完整 Quest3 VR 链路

方案一发布包含完整源码链路，而不是只有 UDP 桥接层：

```text
Quest3 --UDP--> quest_udp_to_dds_node --DDS--> leju-vr-control
                    /rt/quest/bone_poses, /rt/quest/joysticks
                                      --> leju-ik/Drake --> /rt/arm_trajectory
```

完整 VR 构建要求目标架构的 Drake、Eigen3、yaml-cpp、Protobuf 和 GCC 11。Drake 不随本发布树复制：如果安装在标准位置（例如 `/opt/drake`），配置时会自动发现；安装在其他位置时才需要通过 `DRAKE_PREFIX` 指定：

```bash
# 标准安装位置：无需手动指定 Drake
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 自定义安装位置：使用目标架构的 Drake
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
  -DDRAKE_PREFIX=/path/to/aarch64/drake
cmake --build build -j$(nproc)
```

`DRAKE_PREFIX` 也可以作为环境变量提供。Drake 必须是目标架构版本；编译期自动发现不会替代运行时动态库配置，运行程序时仍需确保 Drake 的 `lib` 在系统动态链接器路径或 `LD_LIBRARY_PATH` 中。方案一应生成 `quest_udp_to_dds_node`、`quest_vr_control_node`、`quest_vr_abs_control_node` 和 `libleju-ik.so`。绝对式节点需要 `src/leju_assets/models/biped_s17/urdf/drake/biped_v3_arm.urdf`；联合启动可使用 `src/leju_launch/launch/vr_teleop.launch`。没有 Drake 时不得将构建标记为完整 VR。

## 依赖
```bash
sudo apt-get install -y g++-11 libacl1-dev libeigen3-dev libyaml-cpp-dev libusb-1.0-0-dev
# 手柄：SDL3（随包在 src/leju-joystick/3rd_party，无需另装）
# RL 控制器：已随包提供预编译运行时，无需编译 leju-rl-controller 源码
```

## 构建

```bash
git clone <this-repo-url>
cd <repo-name>
CC=gcc-11 CXX=g++-11 cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
CC=gcc-11 CXX=g++-11 cmake --build build -j$(nproc)
```

预编译的 cyclonedds 静态库内嵌 GCC 11 LTO，必须用 GCC 11 链接。

## 运行 / 自启动

```bash
export ROBOT_VERSION=46    # 14=Roban2 base, 46=Kuavo 4 UAE, 52=Kuavo 5 base
sudo -E bash src/leju_launch/scripts/launch_real.sh
# 或部署手柄上电自启服务：
sudo bash src/leju-joystick/services/deploy_joy_autostart.sh
```

`sudo -E` 透传环境变量（`LD_LIBRARY_PATH` / `PATH` / `ROBOT_VERSION`）。
`installed/setup.bash` 设好 `LD_LIBRARY_PATH=installed/lib`，预编译 `.so` 的运行期依赖（cyclonedds / iceoryx / onnxruntime / SDL3 / MuJoCo）在其中。

## 目录说明

| 路径 | 内容 |
|---|---|
| `CMakeLists.txt` | 开源树根（include stubs + add_subdirectory 源码包） |
| `lejusdk-stubs.cmake` | 核心库和 DDS SDK 的 IMPORTED stub 定义 |
| `opensource_packages.txt` | stub/binary 例外清单；未声明的根 CMake source 包默认整包发布 |
| `installed/bin/` | 预编译产物（leju-hardware / RL 控制器 / recorder 等） |
| `installed/lib/` | 预编译 .so（lejusdk-lowlevel/vr / kuavo_solver + cyclonedds/iceoryx/onnxruntime/SDL3） |
| `installed/include/` | 核心库头和 DDS SDK 公共头（实现不发布） |
| `src/lejusdk/lejusdk-utils/` | header-only 源码（lowlevel/vr 公开头依赖） |
| `src/leju-controllers/*/` | 可开源控制器源码（dummy / keyboard）；RL 控制器在 installed/bin |
| `src/leju-joystick/` | 手柄源码 + services/ 自启动脚本 + SDL3 3rd_party |
| `src/leju_assets/` `src/leju_launch/` | 资产 / launch / 启动脚本 |
| `src/lejusdk/examples/` | SDK 示例源码 |
| `src/leju-hardware/config/` | 硬件配置（leju-hardware 本体是预编译产物） |
| `scripts/` `docs/` | 工具与文档 |
| `resources/` | 舞蹈配乐 / 手势 tact 文件（部署自启动脚本会同步到 ~/.config/lejuconfig） |
| `VERSION` | 版本信息 |
