# Roban-2 现场观察台

本机及局域网浏览器使用的可视化网站。服务运行在当前电脑，复用 SSH 只读订阅下位机 DDS 和上位机 ROS1。浏览器无需安装 ROS，网页和模型资源均从本地服务加载。

## 打开网站

- 本机：[http://127.0.0.1:8765](http://127.0.0.1:8765)
- 当前局域网：[http://192.168.110.41:8765](http://192.168.110.41:8765)

第二个地址取决于运行服务电脑的实际 IP。换 Wi-Fi 或电脑后执行 `run.sh status` 查看地址。浏览器电脑应与服务电脑处于能够相互访问的局域网；网站不依赖公网托管。

```bash
cd /media/lsj/EXTERNAL_USB/Roban-Ubuntu/projects/roban2-visual-console
./run.sh start
./run.sh status
./run.sh stop
```

`start` 会把服务放到后台，重复启动会复用现有进程。退出浏览器不停止服务；电脑重启后需要再次执行 `start`。此版本没有安装开机自启服务。

## 已提供的功能

| 功能 | 使用方法与数据含义 |
| --- | --- |
| 头部彩色画面 | 上位机 Gemini 335 RGB，支持截图、全屏；显示图像时间戳、分辨率和本机接收间隔 |
| 深度伪彩 | 点击“深度”；动态归一化显示，原始深度尺度未在本次标定，不能据此读取毫米距离 |
| 身体与双 Revo-2 URDF | 鼠标旋转、缩放；正/侧/顶视角、坐标轴、线框、复位；“左手 / 右手”按钮放大手部 |
| 身体关节 | 23 个原生反馈通道，角度/弧度切换，速度、力矩反馈、URDF 范围、分组、30 秒曲线；表格可单独暂停 |
| 灵巧手 | 查看左右手有效标志、样本年龄、各六个通道的原始位置、速度、电流、状态码 |
| 状态详情 | 查看下位机硬件状态、上位机控制器消息的原始内容 |
| 连接诊断 | 查看来源话题、接收频率、接收间隔、源数据年龄、SSH 错误、相机 CameraInfo、ROS 话题列表；支持重新连接采集 |
| 导出状态 | 下载当前状态 JSON，包含数据有效性与来源 |
| 记录反馈 | 手动开始/结束；在服务电脑保存 JSONL，记录反馈和图像元数据、时间戳；在“反馈记录”中下载 |

反馈记录**不包含视频像素**，不能用作完整视频录像或同步训练数据集。单次自动限制为 30 分钟或 200 MB；剩余空间低于阈值时停止记录。截图文件由浏览器下载。

## 运行结构与配置

```text
浏览器（本机 / 局域网其他电脑）
                 │ HTTP + WebSocket + MJPEG :8765
                 ▼
当前电脑 server.py（共享采集、状态缓存、记录、URDF 网格）
  ├─ SSH → lab@roban-lower.local
  │         └─ 原生 DDS：/rt/joint_state、/rt/hand_state、/rt/hardware/state
  └─ SSH → 下位机 → leju_kuavo@192.168.26.12
            └─ 上位机 ROS1：RGB、depth、CameraInfo、/controller_state
```

上下位机地址和话题在 `config.json` 中。当前上位机直接 SSH 免密登录未通过，复用下位机到上位机已有的连接通道。所有 SSH 连接都使用已有密钥、`BatchMode=yes`、`StrictHostKeyChecking=yes`，没有存储密码。修改连接配置后重启本地服务。

多个浏览器共享每台机器人电脑的一条采集进程，不为每个浏览器重开相机或创建新 DDS 读者。断线按 2～30 秒退避重连。

本次启动的是相机的独立采集 launch：

```text
/home/leju_kuavo/roban2-stage0-test/launch/roban2_gemini_test.launch
```

该文件仅包含 Gemini 相机节点，640×480、60 fps，关闭 TF 和点云发布。网页帧率上限由 `camera_fps` 控制（当前 60），实际接收帧率受编码与网络影响。相机进程与网站进程分别管理：

```bash
./run.sh camera-status
./run.sh camera-start
./run.sh camera-stop
```

相机管理器只停止自己记录并核对过命令行的独立相机进程组；已有相机流会直接复用。上位机相机日志与 PID：

```text
/home/leju_kuavo/roban2-visual-console-camera/camera.log
/home/leju_kuavo/roban2-visual-console-camera/owned.pid
```

## URDF 与 Revo-2

当前网站模型位于 `assets/model/robot.urdf`，包括身体 23 轴和左右手各 11 个运动学关节，总计 45 个可动关节。每手保留 6 个主动通道、5 个 mimic，依赖的 99 个网格文件已复制到网站自己的目录。

模型来源为现有工程的 `outputs/model_alignment_20261021/lower_s17_revo2_v1/roban2_lower_revo2.urdf`。双手接在 `zarm_l7_end_effector` 和 `zarm_r7_end_effector`，相对前臂法兰位置为 `[0,0,-0.2175] m`，方向沿用该工程已有安装变换。来源、每个网格的哈希及安装信息见 `assets/model/manifest.json`。

本次只读核对了下位机身体 URDF，SHA256 为：

```text
7cb1820cd6a8717b01a353e05501f22225d7ddc097fa9501116ad54ebad2d3a4
```

它与合并模型的身体来源一致。网站合并 URDF 的 SHA256 为：

```text
f10780b45ad8ba525da9e091c9157ba939f5d5ed32f182828331eeb010e33a30
```

身体姿态仅由名称和长度均通过检查的原生 23 维关节反馈驱动，不使用上位机历史 `/joint_states` 的 23 维索引猜测映射。原生头部通道按 S17 合约映射为 `zhead_1_joint`、`zhead_2_joint`。双手的基座随腕部运动。

**手指当前保持张开参考姿态。** 原始手通道到实际关节弧度的标定尚未具备，因此不把 0～100 原始位置线性换算成 URDF 角度。安装方向继承现有可视化工程，尚未在本次独立测量物理安装方向。手部原始反馈可在“灵巧手”面板查看。身体底座固定显示，不显示世界坐标中的行走位移。

需要重建同一模型时：

```bash
.venv/bin/python tools/build_model.py \
  --source-project /media/lsj/EXTERNAL_USB/Roban-Ubuntu/projects/桌面/roban2-mujoco-sorting
```

## 数据有效性

- 身体反馈接收间隔或采集端源数据年龄超过 250 ms 时标记过期；源时间明显位于未来时标记时钟异常。
- 图像超过 1500 ms 标记过期并覆盖提示；过期帧不能作为实时截图保存。
- 缺失数据保持 `--`；没有收到关节时，模型标记为参考姿态。保留最后姿态不代表机器人已经停止运动。
- 本机接收间隔表示最近一次本机收到该通道之后经过的时间。源数据年龄使用采集端时钟计算。这两项不等于经同步验证的端到端摄像头延迟，也不证明不同通道已经时间同步。
- 网页不发布关节动作、不初始化机器人控制 SDK、不调用控制 RPC、不切换控制器。相机节点会发布图像。页面不是急停装置。

## 本次验证状态（2026-10-22）

- 已实测上位机真实 RGB 和 depth 画面，分辨率 640×480；网页约 9～12 fps，瞬时值随网络波动。
- 2026-10-23：`camera_fps` 调整为 60，相机源 60 fps；受上位机编码负载影响，网页实测约 44～49 fps，瞬时值随网络波动。
- 从下位机访问 `http://192.168.110.41:8765/api/state` 返回 HTTP 200，验证跨电脑局域网访问。
- 浏览器已显示完整 S17 身体与左右 Revo-2 手模型，双手放大、RGB/深度切换、记录开关均已核对。
- 11 项自动检查通过：缺失/异常/重复/过期关节、时钟异常、双手结构与网格、截图有效性、记录边界、多观察端、跨源保护及不存在运动/外部注入接口。
- 下位机订阅进程已建立，但本轮没有收到关节、灵巧手、硬件状态，控制器状态也未收到。**本次没有完成机器人实际运动与模型随动的现场验收。** 正常操作流程启动反馈后，网页会自动订阅；网站没有代为启动机器人控制器。

## 文件与维护

```text
server.py                  服务、共享缓存、数据有效性与 HTTP 接口
bridges/lower_reader.py     DDS 订阅，只读
bridges/upper_reader.py     ROS 图像与状态订阅
web/                       界面与 Three.js URDF 显示
assets/model/              自包含模型、网格和来源清单
state/server.log           本地运行日志
state/records/*.jsonl       手动保存的反馈记录
evidence/                  本次验证记录与页面截图
tests/test_console.py      自动验证
```

```bash
./run.sh test
tail -n 60 state/server.log
```

依赖：本地隔离 Python 环境中的 aiohttp，以及本地 npm 包 Three.js；锁定文件为 `requirements.lock.txt` 和 `package-lock.json`。浏览器加载不访问 CDN。aiohttp 的流式响应实现参考[官方文档](https://docs.aiohttp.org/en/stable/web_reference.html)，3D 视角控制参考 [Three.js OrbitControls 文档](https://threejs.org/docs/pages/OrbitControls)。
