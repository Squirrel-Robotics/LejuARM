# leju_xsens_gmr

独立的 Xsens/MVN → Roban S17 GMR → DDS HEFT 前端。包内负责 MVN Quaternion/Scale、坐标映射、BVH 骨架标定、设备 IK 配置和 `leju_xsens_gmr.main`。唯一直接 Python 包依赖为 `leju_gmr_common`；不导入 Pico，也不要求其 protobuf、接收器或虚拟环境。

## 快速启动

以下命令从仓库根目录执行：

```bash
bash src/leju_launch/scripts/setup_xsens_gmr_env.sh
bash src/leju_launch/scripts/run_xsens_gmr.sh
```

setup editable 安装 common + Xsens，使用 `XSENS_GMR_VENV`，默认 `~/.cache/leju_xsens_gmr/venv`，不回退到 Pico 环境。激活该环境后也可运行 `leju-xsens-gmr` 或 `python -m leju_xsens_gmr.main`。

默认监听 MVN UDP `9763`、BVH TCP `9764`；同一演员的 BVH、Scale 和姿态就绪后自动标定。单独运行不会启动 Pico；可选按键由独立 `run_pico_joy.sh` 进程发布，Xsens 通过 common 的 `JoystickSubscriber` 轮询 `--control-topic`（默认 `/rt/pico/joysticks`，传空字符串禁用）。

完整平台入口：

```bash
bash src/leju_launch/scripts/launch_xsens_sim.sh --no-shm
bash src/leju_launch/scripts/launch_xsens_real.sh
# 可选：另启 Pico 按键进程，需先运行 setup_pico_gmr_env.sh
bash src/leju_launch/scripts/launch_xsens_sim.sh --no-shm --with-pico-joy
```

ROS 入口为 `xsens_teleop.launch`，可选 `enable_pico_joy:=true`；Pico 按键进程独立使用 `PICO_GMR_VENV`。具体设备设置、DDS 配置和命令见 [Xsens 使用说明](docs/xsens.md)。

## 迁移

- 旧 `leju_pico_gmr.xsens_main` 改为 `leju_xsens_gmr.main`，不保留跨包别名或 stub。
- 旧 `PICO_GMR_VENV` 不再影响 Xsens；重新运行 Xsens setup。
- `XSENS_GMR_PYTHON` 仅供 `setup_xsens_gmr_env.sh` 选择创建环境的基础解释器，与 `PICO_GMR_PYTHON` 用途一致。运行固定使用 `XSENS_GMR_VENV/bin/python3`；`--xsens-python` 运行时覆盖参数已移除。
- `--no-pico-joy` 已移除；独立启动直接用 `run_xsens_gmr.sh`，完整启动默认也不启动 Pico。
- BVH 工具移至 `src/leju_xsens_gmr/tools/send_xsens_bvh.py`。
- DDS 输出 `/rt/pico/heft_reference` 与下游 `heft_pico` **保留原名**，属于控制器线协议兼容，不是 Pico Python 依赖。

共享核心与设备独立性的边界见 [common 说明](../leju_gmr_common/README.md)。

## Python 与 Docker 环境

GMR 使用 **Python 3.10–3.12（优先 3.10）**。MuJoCo 3.11.0 不支持 Python 3.8；当前 CycloneDDS 0.10.2 在 Python 3.14 上会出现 `_Py_IsFinalizing` 导入错误。安装脚本会选择兼容的解释器，必要时在用户缓存中安装 uv 并下载独立 Python 3.10；缺少 ensurepip 时通过 uv 创建环境，不替换系统或 ROS Python。

请在**实际运行前端的环境内**执行对应 setup。宿主机 `/home/hsw/.cache/...` 的环境不会自动成为容器 `/root/.cache/...` 的环境；`catkin build` 也不会安装这些 Python 包。Docker 内可以直接重新运行 setup，无需先安装 `python3.8-venv`。

不兼容或不完整的旧环境会保存在原路径旁的 `.backup-*` 目录。setup 只有在 CycloneDDS 导入和 S17 模型加载验证通过后才报告成功。安装及运行脚本隔离 ROS 的 `PYTHONPATH`，避免不同 Python ABI 混用。
