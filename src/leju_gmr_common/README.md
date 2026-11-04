# leju_gmr_common

Roban S17 的设备无关 GMR 与 DDS 核心，无 ROS Python 依赖，也不导入 Pico 或 Xsens 设备包。

## 包边界

- **common**：Mink IK 求解器、canonical 人体帧的标定与朝向处理、公共消费循环、冻结/恢复与速度计算、S17 HEFT 关节提取、DDS 类型及发布/订阅（含 `JoystickSubscriber`）。
- **[Pico](../leju_pico_gmr/README.md)**：UDP 接收、protobuf、Unity/骨骼到 canonical 的映射、实时采样标定、Pico IK 配置、人体入口与独立按键入口。
- **[Xsens](../leju_xsens_gmr/README.md)**：MVN Quaternion/Scale 接收与映射、BVH 骨架和标定来源、Xsens IK 配置及主入口。

依赖方向为 `leju_pico_gmr → leju_gmr_common ← leju_xsens_gmr`。设备配置由设备包提供；common 不选择或加载设备实现。Python 不提供跨设备源码别名、旧入口转发或兼容 stub。

这里的独立性按设备接入、配置、入口和运行环境划分。与完整复制两套 `kuavo_*` 包不同，共享核心只维护一份 S17 求解与输出契约，避免标定、关节顺序和 DDS 修复在两份副本中分叉；设备协议和标定来源仍各自维护。共享代码不等于共用虚拟环境，也不表示逐帧复刻原 Kuavo 机型行为。

## 安装与环境

从仓库根目录按所用设备执行：

```bash
bash src/leju_launch/scripts/setup_pico_gmr_env.sh
# 或仅安装 Xsens：
bash src/leju_launch/scripts/setup_xsens_gmr_env.sh
```

每个脚本 editable 安装 common + 对应设备包，不安装另一设备包（第三方依赖照常安装）。默认环境分别为：

- `PICO_GMR_VENV`：`~/.cache/leju_pico_gmr/venv`。
- `XSENS_GMR_VENV`：`~/.cache/leju_xsens_gmr/venv`。

两者互不回退；缺少目标环境时应运行对应 setup。`PICO_GMR_PYTHON` / `XSENS_GMR_PYTHON` 仅为对应 setup 创建虚拟环境时选择基础解释器；运行脚本固定使用对应 `*_GMR_VENV/bin/python3`，不受已导出的 setup 解释器变量影响。旧 `PICO_GMR_VENV` 不再影响 Xsens。setup 使用仓库随附的 CycloneDDS 库构建 Python 绑定；S17 模型从同级 `leju_assets/models/biped_s17/xml/biped_s17.xml` 读取，部署需保留这些资源。

## 保留的控制器契约

HEFT 的 21 关节顺序为 **腰、左腿 6、右腿 6、左臂 4、右臂 4**，按关节名提取，保留既有腰关节符号转换。

输出仍为 `/rt/pico/heft_reference`（`leju::msgs::HeftReference`），下游控制器仍为 `heft_pico`。这些名称是既有控制器的线协议兼容约定，没有重命名，也不是对 Pico Python 包的依赖。公共 DDS 中的 `QuestJoysticks` 类型与 `/rt/pico/joysticks` 同理；Xsens 可仅通过 DDS 接收独立进程的按键。

## Python 与 Docker 环境

GMR 使用 **Python 3.10–3.12（优先 3.10）**。MuJoCo 3.11.0 不支持 Python 3.8；当前 CycloneDDS 0.10.2 在 Python 3.14 上会出现 `_Py_IsFinalizing` 导入错误。安装脚本会选择兼容的解释器，必要时在用户缓存中安装 uv 并下载独立 Python 3.10；缺少 ensurepip 时通过 uv 创建环境，不替换系统或 ROS Python。

请在**实际运行前端的环境内**执行对应 setup。宿主机 `/home/hsw/.cache/...` 的环境不会自动成为容器 `/root/.cache/...` 的环境；`catkin build` 也不会安装这些 Python 包。Docker 内可以直接重新运行 setup，无需先安装 `python3.8-venv`。

不兼容或不完整的旧环境会保存在原路径旁的 `.backup-*` 目录。setup 只有在 CycloneDDS 导入和 S17 模型加载验证通过后才报告成功。安装及运行脚本隔离 ROS 的 `PYTHONPATH`，避免不同 Python ABI 混用。
