# HEFT 头、手臂、腰部关节角测试

完整扫描使用 `dds_heft_pose_test.sh`，指定单关节使用 `dds_heft_joint_test.sh`。两者共用同一套采集、控制和报告逻辑，无需另跑报告命令。

## 1. 启动与执行顺序

机器人使用吊架，现场有人操作手柄和急停。已有控制器运行时直接使用，机器人先处于 AMP 站立状态、摇杆回中。

如果控制器尚未启动，在机器人终端运行：

```bash
cd Lejulab_ARM
sudo su
bash src/leju_launch/scripts/launch_teleop_real.sh
```

按手柄 START 在 AMP 站稳、处于想作为初始姿态的位置后，在另一个 lab 交互终端先记录，再启动测试，不加 sudo：

```bash
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.sh
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.sh --execute
```

1. 第一条只读采集 3 秒并更新 `validation/heft_pose_test/current_snapshot.json`。第二条读取这个文件作为初始参考；启动时再次检查反馈，但不覆盖已记录的角度。没有记录时会提示先采集，不回退到旧 AMP 文件。
2. 核对后按终端 **Enter**，开始以 50 Hz 发布基准参考。
3. 出现“参考流已预热”后按手柄 **B**，从控制器日志确认已进入 `heft_pico`，再按终端 **Enter** 开始扫描。
4. 自动依次测头部 yaw、pitch，左臂 1～4、右臂 1～4、腰部，共 **11 轴×7 点＝77 点**。每次只改变一个关节参考，其他参考保持基准；同一关节从较近一端测到另一端，然后回一次基准。
5. 完成后程序继续发布基准参考。先按手柄 **X**，确认已回 AMP，再按终端 **Enter** 停止发布并退出。

默认峰值参考速度 10°/s，每段至少 1.5 秒，到点保持 2 秒，用末尾 1 秒反馈统计误差。测点覆盖 URDF 与硬件限位交集内、距两端各 2° 的范围。角度误差大只记录，不因此中止；模型产生的其他关节运动也会记录。Ctrl+C 会停止发布，若仍处于 HEFT，需现场切回 AMP。

头部 yaw 的测试有效限位为 **−60°～+60°**，同时用于全项计划和单关节手动输入。默认范围扫描保留两端各 2° 余量，所以实际端点为 **−58°、+58°**；手动输入允许到 ±60°，超出范围会拒绝执行。

头部使用 `/rt/head_trajectory` 独立通道；手臂和腰部使用 `/rt/pico/heft_reference`；实际关节角来自 `/rt/joint_state`。头部测试时全身 HEFT 基准仍持续发布。脚本不会代按 B/X，后台只读查询的控制器状态写入报告，查询缺失会标为未知。

## 2. 报告与默认路径

以后每次测试的输出统一放在机器人目录：

```text
/home/lab/Lejulab_ARM/validation/heft_pose_test/
  report.md              逐点发布角、实际角、误差及汇总
  result.json            完整统计结果与执行条件
  samples.csv            逐帧全身关节反馈、发布参考与误差
  commands.jsonl         实际成功发布的 DDS 原始消息
  current_snapshot.json  先记录、后由 --execute 读取的初始姿态
```

重复运行会更新这个目录中的同名输出。报告自动生成，**误差＝实际角度−实际成功发布的参考角度**，展示单位为度。腰部硬件角和 HEFT 线值符号相反，报告分别列出。默认 ±1° 只作精度判定。统计窗口中的控制器观测缺失时，不能据此证明该点已处于 HEFT。

默认基准文件统一为 `/home/lab/Lejulab_ARM/validation/heft_pose_test/current_snapshot.json`，全项和单关节入口共用：先运行任一入口且不带 `--execute` 记录，再加 `--execute` 启动，无需复制或改路径。运行时当前姿态检查保存在结果 JSON 内，不会更新初始姿态文件。原 `/home/lab/Lejulab_ARM/validation/heft_amp_stand_snapshot_20261017.json` 仅作为备份；只有显式传入 `--baseline` 才读取它。根姿态由 IMU 和腰角推导，根高度由 URDF 估算，不是外部位姿测量。

需要用新角度覆盖时，在目标姿态下重新运行一次不带 `--execute` 的入口。采集成功后会替换共用的 `current_snapshot.json`，下次执行自动读取新值；采集失败保留上一次记录。若旧测试仍在运行，先按 X 回到 AMP，并按终端提示结束旧测试，再重新记录。无需保留旧值时不用手动删除或复制文件；需要备份时，在重新记录前自行复制这份 JSON。

可选的无运动检查：

```bash
# 离线生成计划，不连接 DDS；输出到同目录 plan.json
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.sh --dry-run

# 只读采集当前角度，不发布；输出到同目录 capture.json、current_snapshot.json
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.sh
```

仅单独选择头部时（`--groups head`），默认不发布 HEFT；需要同时保持 HEFT 时再加 `--heft-for-head`。其他参数见 `--help`。

## 3. 保留脚本

全部位于 `/home/lab/Lejulab_ARM/src/leju_launch/scripts/`；本地对应目录为 `/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/`。

| 文件 | 用途 |
|---|---|
| [dds_heft_pose_test.sh](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.sh) | 完整扫描入口，配置 DDS 与动态库环境 |
| [dds_heft_joint_test.sh](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_joint_test.sh) | 连续单关节入口，从已记录姿态开始，测后保持目标并保留此前参考角 |
| [dds_heft_pose_test.py](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_test.py) | 执行测试、记录数据、自动生成报告；报告函数为 `write_markdown_report()` |
| [dds_heft_pose_guard.py](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_guard.py) | 读取机械限位、生成测点、检查有效数据和实际限位 |
| [dds_heft_pose_kinematics.py](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_pose_kinematics.py) | 读取 URDF、计算根参考与关节坐标转换 |
| [dds_heft_controller_observer.py](/home/cyh/roban_dev/Lejulab_ARM/src/leju_launch/scripts/dds_heft_controller_observer.py) | 后台只读查询活动控制器，为报告记录状态 |

这 6 个文件是同一套测试的两个入口和依赖，4 个 Python 模块不需要单独执行。另依赖项目原有的 `dds_leju_types.py`、控制器配置、URDF、硬件配置、CycloneDDS 和 PyYAML。

接口定义保留在 [HeftReference.idl](/home/cyh/roban_dev/Lejulab_ARM/docs/interfaces/HeftReference.idl)，机器人路径为 `/home/lab/Lejulab_ARM/docs/interfaces/HeftReference.idl`。这是阅读用副本，测试程序不读取它，路径可移动；实际 DDS 类型由 Python 定义，控制器使用已编译的类型。自行实现发布器时必须保持 DDS 类型名、字段及序列化布局兼容。

## 4. 连续单关节测试：从已记录姿态开始，保留上次参考角

在 lab 交互终端先记录，再启动；如果刚用全项入口记录过，直接执行第二条即可：

```bash
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_joint_test.sh
bash /home/lab/Lejulab_ARM/src/leju_launch/scripts/dds_heft_joint_test.sh --execute
```

程序读取共用的 `validation/heft_pose_test/current_snapshot.json` 作为初始参考，不需要复制或编辑文件。按提示 **Enter 开始参考流 → 预热后按 B → 确认 heft_pico 后 Enter**，随后在同一终端逐次输入关节编号和绝对角。输入值单位为度，与硬件关节反馈同坐标。每次输入前都会打印编号表：

| 测试编号 | 关节 |
|---|---|
| 1、2 | 头 yaw、pitch |
| 3、4、5、6 | 左臂 l1、l2、l3、l4 |
| 7、8、9、10 | 右臂 r1、r2、r3、r4 |
| 11 | 腰 waist |

这些是测试菜单编号，不是 DDS 数组下标。

下面仅展示输入语法，数值不是已验证无干涉的运动建议：

```text
4 30
5 -5,0,5
show
```

第一条把 l2 参考设为 30°，测量误差后保持此参考；第二条只修改 l3，依次测 -5°、0°、5°，l2 仍保持 30°，其他参考也不变。第二条结束后，l3 继续保持 5°。后续可以继续输入任何支持的关节，不重启脚本、不自动回位。多个目标可用逗号或空格分隔；所有目标先检查机械限位，非法输入不执行，继续保持当前参考。

终端每个测点的开始和结果都有分隔线；发布角、实际角分行显示，误差与 PASS/FAIL 独立加粗高亮：通过为绿色，不通过为红色。重定向到文件时输出不含颜色控制字符。

`show` 打印当前整套参考角，并标出可测试关节的编号。等待输入期间仍以 50 Hz 发布最后参考。这里保留的是你设置的参考角，不是把 HEFT 的实测偏差重新写成目标；HEFT 仍可能调节其他实际关节。机械限位检查不包含全身碰撞检查。

完成全部操作后输入 `quit`，程序继续保持最后参考；先按手柄 **X** 并确认回到 AMP，再按终端 **Enter** 停止发布。重新启动一个新会话会从最近一次记录的初始姿态开始。

这一会话的所有测点累计写入 `/home/lab/Lejulab_ARM/validation/heft_joint_test/`：`report.md`、`result.json`、`samples.csv`、`commands.jsonl`。初始姿态记录在共用的 `/home/lab/Lejulab_ARM/validation/heft_pose_test/current_snapshot.json`。JSON 逐点记录修改前后的整套参考角；CSV 记录全身发布角、实测角及误差。再次启动会更新同名结果文件，不影响初始姿态或历史 77 点报告。

单关节报告衡量“HEFT 参考角 → 编码器实测角”的端到端跟踪误差。头部仍走独立头部通道，同时保持全身 HEFT 参考。仅检查已记录的初始参考而不连接 DDS，可运行同一入口并加 `--dry-run`。

## 5. 切换和参考断流

S17 配置中，B 请求 `heft_pico` 实时参考模式；X 回 AMP；A 为 `heft` CSV 模式并自动起播，不能把它当作静止保持入口。单独按 B 不会生成参考数据。

实时 HEFT 要求有效、新鲜且已有缓冲的参考。本脚本先以 50 Hz 发布 1 秒基准参考，等待现场操作期间也持续发布；固定姿态时重复同一 `q`、`qd=0`，持续更新时间戳和 `seq`，同一段流的 `calibration_epoch` 保持不变。消息还需正确的 21 维关节顺序、根位姿、版本字段及 `valid/calibrated` 状态。

`config_heft.yaml` 当前配置 `cursor_delay=0.16`、`fresh_timeout=0.15`、`hold_timeout=0.5`、`fallback_controller=amp`。参考未就绪会拒绝切入，参考断流存在回 AMP 的处理路径；这些参数并非电机使能开关，不能据此承诺从最后一帧起精确多少毫秒执行哪一步。控制器 C++ 实现未包含在本地仓库中。

用户先前提供的 `/home/lab/.ros/lejulab/stdout/2026-10-17_14-35-46/stdout.log` 中已核到三次拒绝原因：`HEFT live reference not ready`，且 `frames=0`。这三次切换时控制器没有可用参考帧；日志本身不能证明其他时间的回退也全是同一原因。

## 6. 本次已完成的 77 点结果

本地目录：`/home/cyh/roban_dev/Lejulab_ARM/validation/heft_final_20261017/`。机器人对应目录：`/home/lab/Lejulab_ARM/validation/heft_final_20261017/`。仅保留以下 4 个文件：

| 文件 | 内容 |
|---|---|
| [final_report.md](/home/cyh/roban_dev/Lejulab_ARM/validation/heft_final_20261017/final_report.md) | 条件说明、11 轴汇总与全部 77 点角度和误差 |
| [final_points.csv](/home/cyh/roban_dev/Lejulab_ARM/validation/heft_final_20261017/final_points.csv) | 77 点发布角、DDS 线值、实际角、误差与控制器观测 |
| [final_published_vs_actual.png](/home/cyh/roban_dev/Lejulab_ARM/validation/heft_final_20261017/final_published_vs_actual.png) | 发布角与实际角对照图 |
| [final_error_vs_published.png](/home/cyh/roban_dev/Lejulab_ARM/validation/heft_final_20261017/final_error_vs_published.png) | 各测点误差图 |

本次历史实测的头部 14 点采用 5°/s、保持 3 秒，手臂和腰部 63 点采用 10°/s、保持 2 秒。以后使用上述默认命令时，77 点统一采用当前快速参数；历史报告保留当时实际执行条件。

现场补充 l3、l4、r3、r4 测试时曾碰到本体，报告已标明这 4 轴受接触影响。因缺少具体接触测点标记，保留全部原始数值，不将这些轴的汇总解释为无接触跟踪精度或电机本体精度；需要在无干涉姿态下单独复测。
