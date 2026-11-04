# leju_pico_gmr

Pico VR → Roban S17 GMR → DDS HEFT 前端，无 ROS Python 依赖。包内负责 UDP/protobuf 接收、Unity 与骨骼映射、实时采样标定、Pico IK 配置和入口；设备无关求解、canonical 标定处理、消费循环、S17 HEFT 与 DDS 类型由 [leju_gmr_common](../leju_gmr_common/README.md) 提供。

## 安装与运行

从仓库根目录执行：

```bash
bash src/leju_launch/scripts/setup_pico_gmr_env.sh
bash src/leju_launch/scripts/run_pico_gmr.sh --robot roban_s17
```

setup 使用 `PICO_GMR_VENV`（默认 `~/.cache/leju_pico_gmr/venv`），editable 安装 common + Pico，不安装 Xsens。安装现在需要 common，setup 已处理；手动安装时需在目标环境同时执行 `python -m pip install -e src/leju_gmr_common -e src/leju_pico_gmr`。

`PICO_GMR_PYTHON` 仅供 setup 选择创建虚拟环境的基础解释器；运行脚本使用 `PICO_GMR_VENV/bin/python3`。Xsens 的 `XSENS_GMR_PYTHON` 同样仅用于 setup。

Pico 公开 CLI 不变；激活环境后仍可使用：

```bash
leju-pico-gmr --robot roban_s17
# robans17 也可用；常用选项：--port 12345 --fps 50 --viewer --no-broadcast -v
```

Pico 与 Xsens 环境互不回退；Xsens 使用自己的 `XSENS_GMR_VENV`，旧 Pico 环境变量不再影响它。

## 数据与控制

UDP `12345` 的 24 骨骼经映射、实时标定与朝向对齐后，由Pico消费循环默认以 50 Hz 求解（原始骨架仍按100Hz接收）。输出包括 `/rt/pico/bone_poses`、`/rt/pico/heft_reference`、`/rt/pico/joysticks` 和 `/rt/head_trajectory`。

HEFT 保留 S17 的 21 关节顺序：腰、左腿 6、右腿 6、左臂 4、右臂 4，以及既有腰符号转换。标定和朝向初始化完成后参考才有效；下游 `heft_pico` 仍要求有效、新鲜且缓冲就绪的数据。

- `RT+B` / `LT+B`：上半身 / 全身实时采样标定。
- `RT+Y` / `RT+X`：冻结 / 恢复参考，保持 yaw 连续。
- `LT+Y` / `LT+X`：由 C++ HEFT policy 开始 / 停止 CSV 录制，保存到 `motion_data/pico_recording/`。

控制器切换由 C++ `PicoTeleopAdapter` 按 `config/17/teleop_bindings.yaml` 的 `pico_bindings` 处理。

## 仅用 Pico 按键

```bash
bash src/leju_launch/scripts/run_pico_joy.sh
```

该脚本在 Pico 环境运行独立模块 `leju_pico_gmr.joy_main`，仅接收按键并默认发布 `/rt/pico/joysticks`，不做人形重定向。它可为独立 Xsens 前端提供控制；不要与占用同一 UDP 端口的 Pico 人体入口同时运行。Xsens 的安装、启动及 BVH 工具均位于 [leju_xsens_gmr](../leju_xsens_gmr/README.md)，不再由 Pico 包承载，也不提供旧入口转发。

## 测试

在已激活的 Pico 环境、仓库根目录执行：

```bash
python -m pip install -e 'src/leju_pico_gmr[test]'
python -m pytest src/leju_pico_gmr/tests
```

## Python 与 Docker 环境

GMR 使用 **Python 3.10–3.12（优先 3.10）**。MuJoCo 3.11.0 不支持 Python 3.8；当前 CycloneDDS 0.10.2 在 Python 3.14 上会出现 `_Py_IsFinalizing` 导入错误。安装脚本会选择兼容的解释器，必要时在用户缓存中安装 uv 并下载独立 Python 3.10；缺少 ensurepip 时通过 uv 创建环境，不替换系统或 ROS Python。

请在**实际运行前端的环境内**执行对应 setup。宿主机 `/home/hsw/.cache/...` 的环境不会自动成为容器 `/root/.cache/...` 的环境；`catkin build` 也不会安装这些 Python 包。Docker 内可以直接重新运行 setup，无需先安装 `python3.8-venv`。

不兼容或不完整的旧环境会保存在原路径旁的 `.backup-*` 目录。setup 只有在 CycloneDDS 导入和 S17 模型加载验证通过后才报告成功。安装及运行脚本隔离 ROS 的 `PYTHONPATH`，避免不同 Python ABI 混用。

参考更新说明：骨架旋转采用批量转换，Pico进程默认Python线程交接间隔为1ms。可用`--fps`显式覆盖求解频率；实际性能以参考时间戳和录制健康检查为准。
