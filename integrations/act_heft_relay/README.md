# ACT / HEFT 下位机中继与执行模块

归档来源：`/home/lab/roban2_act_integration/heft_full_inference_20261030_v75`。
这是下位机组件，配套的 ACT 模型、策略调用端和冻结推理工程由外部项目提供。

| 文件 | 用途 |
| --- | --- |
| `robot_relay.py` | 中继、反馈与参考检查、控制来源检查及执行/传输检查入口 |
| `execution_core.py` | 引用限幅、限速、手部过渡及反馈判定 |
| `can_guards.py` | 四路 CAN 状态、收发进展与新增错误检查 |
| `command_ownership.py` | 手部指令来源辨别 |
| `relay.sh` | 按 `config.json` 设置现有 DDS 库和工程路径 |
| `config.json` | 当前部署配置及原生文件哈希 |

当前配置指向 `/home/lab/Lejulab_ARM`，使用 `/rt/pico/heft_reference`。
依赖主工程中的 `dds_heft_external_control.py`、S17 URDF、硬件配置和 HEFT 参数；归档核查时四个依赖哈希均匹配。
CycloneDDS Python 运行环境及 ARM DDS 动态库仍需由原部署环境提供。
`buffered_capture_entry`、`incident_lock_path` 等路径属于同事电脑上的配套推理工程；模型目录和这些外部文件未包含在本组件中。
`entrypoint=run.sh` 与 `frozen_runtime_directory=model` 保留自该配套工程的元数据；本目录的启动脚本为 `relay.sh`。

## 11 月 5 日 v75 基线配置

- 腰和手臂速度上限：60 度/秒。
- 关节/手部反馈年龄上限和命令包年龄上限：1000 ms。
- ACK 等待：1.0 秒；输入超时：1.25 秒。
- `body_feedback_monitor_only_joints` 包含 `zarm_r3_joint`：继续记录该关节误差，但将它从需反馈达标的跟踪中止和到位判定中排除；指令范围与速率限制仍保留。
- `hardware_write_enabled`、`hardware_write_enabled_by_default`、`physical_execution_qualified` 字段当前均为 `false`。本次归档未启动执行入口或改变现场配置。

CAN 的内核告警保留在观测中；判定代码同时检查当前错误计数、收发进展和新增错误事件。
本次仅完成文件哈希、语法、内部导入和依赖核对，未进行模型推理或真机动作验收。


## 11 月 11–13 日下位机版本

各目录保留原机同名版本的完整中继代码和原始 `config.json`。需要回退到 11 月 5 日基线时，只复制本说明开头列出的 6 个文件到 `/home/lab/roban2_act_integration/heft_full_inference_20261030_v75/`。

| 版本 | 部署目录 |
| --- | --- |
| 回位修订 | [heft_v75_return_20261111](versions/heft_v75_return_20261111/) |
| 速度修订 | [heft_v75_speed_20261112](versions/heft_v75_speed_20261112/) |
| 搬运修订 | [heft_v75_carry_20261112](versions/heft_v75_carry_20261112/) |
| 左手启动 | [heft_v75_left_startup_20261112](versions/heft_v75_left_startup_20261112/) |
| 右手启动 | [heft_v75_right_startup_20261112](versions/heft_v75_right_startup_20261112/) |
| 左手入箱 | [heft_v75_left_bin_transfer_20261112](versions/heft_v75_left_bin_transfer_20261112/) |
| 左手提速 | [heft_v75_left_speedup_20261113](versions/heft_v75_left_speedup_20261113/) |
| 右手提速 | [heft_v75_right_speedup_20261113](versions/heft_v75_right_speedup_20261113/) |
| 左手后接右手 | [heft_v75_left_right_sequence_20261113](versions/heft_v75_left_right_sequence_20261113/) |

新版增加 `transport_pacing.py`，顺序版另有 `sequence_protocol.py`，需配合工作站对应的顺序客户端。
相较基线，九套新配置的命令年龄上限、ACK、输入超时分别为 5 秒、5 秒、5.25 秒；反馈年龄上限为 1 秒。跟踪超限采用记录模式，其他现场参数和各版本差异以原始配置为准；配置未获本次真机验收。核验结果可通过 [换机预检](../../docs/roban2-lower-recovery.md) 重现。

旧部署副本、检查点目录、备份、录制和日志不随这批版本归档。完整 ACT 推理所需的模型和工作站组件由原推理工程另行提供。
