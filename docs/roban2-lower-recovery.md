# 鲁班 S17 换下位机恢复

2026-11-13 归档针对 **Roban 2.2 / S17、RK3588 ARM64、Ubuntu 22.04**。旧下位机的系统 Python 为 3.10，主控使用 ROS Humble 提供的 Pinocchio 3.9.0 和 urdfdom 3.0 ABI。上位机相机脚本使用原来的 ROS Noetic 环境，8877 数采服务仍运行在工作站。

## 1. 恢复工程和系统环境

- 用对应机型的乐聚下位机系统镜像初始化新机，并使用 `lab` 账户。将本仓库检出到 **`/home/lab/Lejulab_ARM`**；这个路径也用于 ACT 原样保存的哈希校验和数采接收脚本。版本以 [SOURCE_SNAPSHOT.json](../SOURCE_SNAPSHOT.json) 为准。
- 使用归档的 `installed/`。头部单发功能的脚本、测试和说明必须与本仓库中的 `installed/bin/run_rl_controller` 配套。这个 AArch64 主控以二进制归档，现有开源树无法重新构建它；不要被旧 `build/`、`devel/` 或另一个安装目录中的主控覆盖。
- 按 [ARM64 构建说明](../README_ARM_OPENSOURCE.md#arm64rk3588-roban-22开放源码--ci-预编译库) 编译开放组件。机器人型号设为 `ROBOT_VERSION=17`，原机编译工具链为 GCC/G++ 11；如需完整 Quest VR，另准备 ARM64 Drake。
- 在下位机装齐 ROS Humble 的 `ros-humble-pinocchio`（须提供 `libpinocchio_{parsers,default}.so.3.9.0`）、`ros-humble-urdfdom`，以及 Ubuntu 的 `libyaml-cpp0.7`、`python3-lz4` 等系统依赖。旧机的主控在加入 `/opt/ros/humble/lib` 和 `/opt/ros/humble/lib/aarch64-linux-gnu` 后可解析全部依赖；用后文预检确认新机的实际链接结果。
- DDS 和 MCAP 记录使用 `lab` 的系统 Python，即使启动器由 root 调用。参考原机依赖见 [lower-python-requirements.txt](lower-python-requirements.txt)。安装 CycloneDDS Python 包时使用随仓库提供的 C DDS；Pico/GMR 使用独立环境，在**实际运行前端的账户**下用 `src/leju_launch/scripts/setup_pico_gmr_env.sh` 重建，不复制原机 venv。

例如，在下位机以 `lab` 身份、从仓库根目录安装 Python 依赖：

```bash
export CYCLONEDDS_HOME="$PWD/src/lejusdk/3rd_party/aarch64/cyclonedds-0.10.2"
export CYCLONEDDS_LIB_DIR="$CYCLONEDDS_HOME/lib"
export LD_LIBRARY_PATH="$CYCLONEDDS_LIB_DIR:$PWD/installed/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
/usr/bin/python3 -m pip install --user --no-binary=cyclonedds -r docs/lower-python-requirements.txt
```

## 2. 核对现场配置和连接

同一台机器人换 RK3588 板卡时，先看 [换板补充文件](../deploy/rk3588-s17/README.md)：包含手柄服务、8765 专用 DDS，以及网口、CAN、串口和蓝牙配置。用新镜像的板卡路径逐项比对后再采用。

- 以新机厂商镜像为起点，核对 `bcan0` 到 `bcan3`、串口设备、IMU 类型、CAN 映射、手柄映射与关节零位。换同一台机器人的主板时，从旧机受控迁移有效的 `~/.config/lejuconfig/`；换机器人时按新设备重新校准。若主控由 root 启动，还要核对 **root 实际读取的配置**。
- 按新机网卡、CAN 设备和上位机链路核对 udev、CAN 启动服务、`/etc/cyclonedds/` 和 RouDi。旧机现场 DDS 绑定机器人内部链路，不能直接用仓库中的回环测试 XML 替代。使用本地保存的现场配置作比对，不直接整包覆盖系统。确认各机时间及录制时钟偏差满足采集要求。
- 按 [8877 恢复说明](../platforms/data-workbench/README.md) 检查工作站 SSH 的 `roban-lower`、接收配置中覆盖 SSH 的 `HostName`、`remote_helper` 和 `remote_root`；如果上位机也换了，再改 `upper_preview_host` 及下位机的 `src/leju_launch/config/vr_vision_config.json`。核对新机主机指纹，给新机单独配置授权，保持主机密钥校验。验证上位机 RGB 服务和两条预览路径、工作站空目录的接收权限，以及两台机器的缓存目录归属。

## 3. 恢复 ACT 下位机端

[ACT 版本清单](../integrations/act_heft_relay/README.md) 中每套是完整的下位机中继目录。将需要的目录按同名恢复到 `/home/lab/roban2_act_integration/`，不混用各目录中的脚本或配置。例如，在新下位机以 `lab` 身份执行：

```bash
mkdir -p /home/lab/roban2_act_integration
cp -a -n integrations/act_heft_relay/versions/. /home/lab/roban2_act_integration/
```

原 v75 基线仍在 `integrations/act_heft_relay/` 根目录。若要恢复基线，按该目录说明单独复制 6 个文件。ACT 的 `native_root`、原生文件 SHA256、两侧顺序配置均保留原值；工作站连接新机时应同步检查其使用的 SSH 地址或别名，归档配置作为可核验的原件保留。

完整推理还需要同事工作站的推理端、启动脚本、冻结运行时和模型权重。按每套 `config.json` 的 `entrypoint`、`frozen_runtime_directory`、`checkpoint_sha256`、`buffered_capture_entry` 等字段取得并核验对应版本。顺序版需要同一版本的工作站顺序客户端，不能用单臂客户端代替。恢复后由现场人员按原操作流程验收。

## 4. 先离线验收，再验收实机

从仓库根目录运行：

```bash
python3 -B scripts/check_roban2_recovery.py
# 仅在新 ARM64 下位机、以 lab 身份运行；额外检查链接和 Python 依赖：
python3 -B scripts/check_roban2_recovery.py --lower
```

预检校验同步文件、ACT 原生依赖、CLI 导入，以及可选的 ARM64 动态链接和 Python 依赖；不启动硬件、主控、相机或执行入口。完成后现场确认 CAN 和 23 关节反馈、RGB/深度预览、试录与接收，再分别验收头部和 ACT。库能加载、哈希一致和离线测试通过不能代替新机的实际验收。
