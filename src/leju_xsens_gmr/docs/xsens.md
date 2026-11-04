# Xsens/MVN → Roban S17 使用说明

本包将原 `kuavo_xsense_gmr` 的设备接入迁入独立 Xsens 前端，使用平台 S17 模型与公共 GMR 核心。Python 入口不依赖 ROS 或 Pico；平台集成继续使用既有控制器。包边界和迁移清单见 [README](../README.md)。以下命令均从仓库根目录执行。

## 安装与独立运行

```bash
bash src/leju_launch/scripts/setup_xsens_gmr_env.sh
export CYCLONEDDS_URI="file://$PWD/src/leju_launch/config/cyclonedds.xml"
bash src/leju_launch/scripts/run_xsens_gmr.sh --domain-id 0
```

setup 仅 editable 安装 common + Xsens 及其第三方依赖。`XSENS_GMR_VENV` 默认 `~/.cache/leju_xsens_gmr/venv`；不读取或回退到 `PICO_GMR_VENV`。前端与控制器须使用相同 DDS 域及兼容的 CycloneDDS 配置。

`XSENS_GMR_PYTHON` 仅用于 `setup_xsens_gmr_env.sh` 创建虚拟环境时选择基础解释器。`run_xsens_gmr.sh` 始终使用 `XSENS_GMR_VENV/bin/python3`，即使仍导出 `XSENS_GMR_PYTHON` 也不会绕过该环境。完整启动同样不支持 `--xsens-python` 运行时覆盖参数。

此命令只启动 Xsens，不启动 Pico。默认通过 common 的 `JoystickSubscriber` 轮询 `/rt/pico/joysticks`；没有按键发布者也能通过 BVH 自动标定。完全禁用前端按键订阅：

```bash
bash src/leju_launch/scripts/run_xsens_gmr.sh --control-topic ""
```

若需要 Pico 按键，在另一终端使用相同 DDS 配置运行：

```bash
bash src/leju_launch/scripts/setup_pico_gmr_env.sh
export CYCLONEDDS_URI="file://$PWD/src/leju_launch/config/cyclonedds.xml"
bash src/leju_launch/scripts/run_pico_joy.sh --domain-id 0
```

该独立进程运行 `leju_pico_gmr.joy_main`，使用 `PICO_GMR_VENV`（默认 `~/.cache/leju_pico_gmr/venv`），接收 UDP `12345` 并发布 `/rt/pico/joysticks`，不发布人体 HEFT。自定义 topic 时，按键进程的 `--joy-topic` 与 Xsens 的 `--control-topic` 应一致；C++ 控制器的订阅配置也需相应匹配。

## 完整平台启动

```bash
# 仿真 / 实机，默认不启动 Pico
bash src/leju_launch/scripts/launch_xsens_sim.sh --no-shm
bash src/leju_launch/scripts/launch_xsens_real.sh
# 可选按键进程，先安装 Pico 环境
bash src/leju_launch/scripts/launch_xsens_sim.sh --no-shm --with-pico-joy
```

ROS launch 的对应入口：

```bash
roslaunch leju_launch xsens_teleop.launch simulation:=true enable_iceoryx_shm:=false
# 可选 Pico 按键；实机使用 simulation:=false
roslaunch leju_launch xsens_teleop.launch simulation:=true enable_iceoryx_shm:=false enable_pico_joy:=true
```

可选 Pico 始终是独立进程、独立环境。`--with-pico-joy` 属于完整 shell 启动入口，不是 `run_xsens_gmr.sh` 的参数；旧 `--no-pico-joy` 已移除。S17 的 controller_manager、HEFT 策略和 teleop 配置继续使用；前端不会自动切换到 `heft_pico`。同一 HEFT topic 只运行一个人体参考发布者。

## MVN 与 BVH

1. MVN Network Streamer 的目标 IP 设为前端机器，UDP 默认 `9763`，同时启用 Quaternion（02）与 Scale（13）。`--character-id` 默认 `0`，一个前端对应一个演员。
2. 导出同一演员的 BVH，骨骼名称使用 `Hips/Chest/Neck/Head`、左右 `Collar/Shoulder/Elbow/Wrist`、左右 `Hip/Knee/Ankle/Toe`；支持单个演员名称前缀（如 `actor_Hips`）。不支持的命名应在发送端明确转换。
3. 向默认 TCP `9764` 发送骨架：

```bash
python src/leju_xsens_gmr/tools/send_xsens_bvh.py actor.bvh --host ROBOT_IP --port 9764 --offset-scale 0.01
```

发送工具仅依赖 Python 标准库，可单独复制到 Windows。`--offset-scale 0.01` 将厘米换成米；offset 已为米时使用 `1`。只发送 HIERARCHY，不发送 MOTION。

协议为 4 字节大端 JSON 长度 + UTF-8 JSON，标识 `xsens_bvh_desc_v1`，单位 `m`；`joints` 包含 `id`、`name`、`parent_id`、`offset`。ACK 同样 framing，含 `ok` 与 `message`。接收器校验完整骨架、有限坐标与合理身高；无效数据可修正后重发。成功接收后锁定骨架，ACK 不代表 IK 已标定完成；更换演员/骨架需重启并重发。

## 标定、输出与控制

MVN 接收器负责分片重组、Scale 缓存、肘部修正及 Xsens → BVH → canonical 的 20 骨骼映射。完整左右肘部 Scale 到达前不产生姿态帧。默认姿态 FIFO 为 120 Hz，预填充 10 帧、最大 15 帧；公共消费循环默认 100 Hz 取最新帧。

BVH 经 FK 构造默认 90° 手臂参考并估计身高，在消费线程自动标定，随后完成初始朝向对齐才发布有效参考。设备配置为 `ik_configs/xsens_to_roban_s17.json`，采用 S17 的 `robot_hierarchy_v1` 目标重建，不使用 Kuavo S54 关节配置，也不承诺与旧 Kuavo 版本逐帧相同。

输出仍为 `/rt/pico/heft_reference`（`leju::msgs::HeftReference`），下游仍为 `heft_pico`，均未改名：这是既有控制器的线协议兼容，**不是 Pico Python 依赖**。21 关节顺序为腰、左腿 6、右腿 6、左臂 4、右臂 4，保留既有腰符号转换。头部单独使用 `/rt/head_trajectory`；不把 Xsens 的 20 骨骼发布到 Pico 的 24 骨骼 `QuestBonePoses` 接口。

缺帧默认最多保持 0.2 秒，保留原本机接收时间；超时停止实时输出，由下游 HEFT 的 fresh/hold/fallback 处理。接收时间不是设备硬件采样时间。

可选 Pico 按键的行为：

- `RT+B` / `LT+B`：重新应用已接收 BVH 的完整标定，重置朝向与速度历史，不采集当前动作作为标定姿态。
- `RT+Y` / `RT+X`：冻结 / 恢复，保持 yaw 连续。
- `RG+A` / `RG+B`：按平台配置切到 `heft_pico` / 返回 AMP；前者要求参考有效且缓冲就绪。
- `LT+Y` / `LT+X`：由 C++ HEFT policy 开始 / 停止 CSV 录制。

## 验证

在已激活的 Xsens 环境执行：

```bash
python -m pip install -e 'src/leju_xsens_gmr[test]'
python -m pytest src/leju_xsens_gmr/tests
```

测试涵盖 MVN/Scale、BVH 及发送工具、网络收发和 S17 集成；合成测试不替代穿戴设备与实机动作验收。
