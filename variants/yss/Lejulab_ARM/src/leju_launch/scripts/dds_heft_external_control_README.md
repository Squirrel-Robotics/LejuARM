# Roban2 BFM：21关节实时参考控制

2026-10-20，按下位机 `/home/lab/yss/Lejulab_ARM` 的当前文件实现。新入口已放在下位机：

```text
/home/lab/yss/Lejulab_ARM/src/leju_launch/scripts/dds_heft_external_control.sh
/home/lab/yss/Lejulab_ARM/src/leju_launch/scripts/dds_heft_external_control.py
```

## 实际流程

1. 操作者启动硬件和主控制，并按手柄 START 进入正常站立控制，此时应为 `amp`。从零启动21关节外控时使用不带Pico前端的 `launch_real.sh`；`launch_teleop_real.sh` 会额外创建Pico发布器，需先处理发布冲突。机器人已经运行时不要重复启动主控。
2. 运行新入口加 `--execute`。它先只读采集2秒当前的23关节反馈和IMU，抽取21关节，保存本次站姿。
3. 脚本先以100Hz持续发送当前站姿的 HeftReference，预热1秒。看到“现在可单按手柄 B”后再按 B。
4. B 对应 `heft_pico`。脚本通过 GetControllerState 只读RPC观测切换，稳定0.5秒后才接受新目标。提前输入或发送的目标被丢弃。
5. 输入新的21关节绝对角，或让外部模型持续发送，脚本按默认10°/s的参考速度上限推进目标。
6. 正常退出时先单按手柄 X 切回 `amp`。脚本观测到离开 `heft_pico` 后停止发布。

不要同时运行 Pico/GMR 或另一套 HEFT 参考发布器。脚本会检查同一话题的其他DDS发布者；发现冲突就退出。

如果出现“发现另一个HEFT参考发布器”，先定位GUID对应的进程并核实当前AMP状态。Pico即使尚未校准、没有有效帧，也可能已创建DDS发布器。现场本版 `_pico_teleop_common.sh` 将Pico作为可选进程；在核实AMP状态和具体PID后可单独停止它。其他动捕分支可能将前端列为必需进程，不能照搬停止方式。

## 在下位机终端运行

已经 SSH 登录下位机时：

```bash
cd /home/lab/yss/Lejulab_ARM
./src/leju_launch/scripts/dds_heft_external_control.sh --execute
```

从当前电脑另开终端直接运行：

```bash
ssh -t roban-lower '/home/lab/yss/Lejulab_ARM/src/leju_launch/scripts/dds_heft_external_control.sh --execute'
```

不加 `--execute` 只采集并保存姿态，不创建关节参考发布器。每次采集都保存到独立目录：

```text
/home/lab/yss/Lejulab_ARM/validation/heft_external_21/时间_进程号/
  snapshot.json   当前实测角、HEFT角、根姿态估计、控制器观测
  target.json     用当前站姿填好的21角度输入模板
  commands.jsonl  每次实际成功发布的完整帧、硬件参考角及最近反馈
```

本次目录的完整路径会打印到终端。运行时不读取之前的站姿作为起点。

## 21个角的顺序和单位

终端 `deg`、`rad` 与JSON输入，均使用下列顺序。数组下标从0开始：

| q下标 | 关节 | /rt/joint_state.q下标 |
|---|---|---|
| 0 | waist_yaw_joint | 12 |
| 1..6 | leg_l1_joint .. leg_l6_joint | 0..5 |
| 7..12 | leg_r1_joint .. leg_r6_joint | 6..11 |
| 13..16 | zarm_l1_joint .. zarm_l4_joint | 13..16 |
| 17..20 | zarm_r1_joint .. zarm_r4_joint | 17..20 |

21维不含头部。本脚本不发布 `/rt/head_trajectory`。

默认输入是硬件反馈坐标的绝对角。`deg` 是度，`rad` 是弧度。转换到HEFT线帧时，只有腰角取反。JSON若指定 `coordinates="heft"`，则直接使用训练坐标，脚本不会重复改变最终线帧的腰符号。

## 终端控制

确认切入 `heft_pico` 后支持：

```text
status
hold
set zarm_l1_joint 5
```

`status` 显示控制器观测和最近已发送的21个硬件参考角。`hold` 取消未完成的目标并保持最近已发参考。`set` 使用硬件绝对角度，只修改指定关节，其余从最近已发参考保持；上例的5是绝对5°，需要实际运动时再输入。

一次输入21个角：输入 `deg` 或 `rad`，后接恰好21个空格分隔的绝对角。请从本次 `target.json` 或 `status` 的实际站姿开始填写，腿部不能用全零代替当前站姿。

也可以直接输入一行JSON，格式与下节相同。终端目标会持续推进到达后保持，直到新目标、退出或反馈故障。

## 模型实时输入

接收端只绑定下位机 `127.0.0.1:28721`，UDP每帧为一个JSON对象：

```json
{
  "q": ["这里必须填写21个数值，按上表排序"],
  "units": "rad",
  "coordinates": "heft",
  "seq": 1,
  "sent_at_ns": 1790000000000000000
}
```

上面是字段示意，不是可发送的目标。`q` 必须是21个有限数值，`units` 只能为 `deg` / `rad`，`coordinates` 只能为 `hardware` / `heft`。直接UDP发送时，`seq` 和 `sent_at_ns` 都必须递增，时间戳使用下位机当前 Unix 纳秒时间。建议50–100Hz更新。

如果模型在外部电脑或上位机运行，推荐使用SSH标准输入转发，由下位机补充序号和时间戳：

```bash
python3 -u your_model.py | ssh -T roban-lower \
  '/home/lab/yss/Lejulab_ARM/src/leju_launch/scripts/dds_heft_external_control.sh --send-stdin'
```

`your_model.py` 的标准输出每帧只打印一行JSON，例如使用：

```python
print(json.dumps({"q": q21.tolist(), "units": "rad", "coordinates": "heft"}), flush=True)
```

模型日志写到 stderr。只允许一个发送端，避免输入来源互相覆盖；如果转发器退出后要重新建立一个UDP发送端，需要重新启动外控会话。本机UDP端点没有网络身份验证，因此只监听下位机回环地址。

另一种方式是在下位机第二个终端持续转发目标文件：

```bash
cd /home/lab/yss/Lejulab_ARM
./src/leju_launch/scripts/dds_heft_external_control.sh \
  --send --file /本次打印的完整目录/target.json
```

目标文件初始是当前站姿。更改文件时先写临时文件，再用原子替换提交，避免发送器读到半个JSON。

UDP目标以发送时间计，默认0.5秒内有效。过期、乱序、重复、无效或越限的包不续期。失流后保持最近已发参考并取消剩余运动目标；有效输入恢复后可继续。同一会话尽量使用一种输入方式，持续UDP会覆盖终端目标。

## HEFT线帧与行为边界

- DDS domain=0，话题 `/rt/pico/heft_reference`，类型 `leju::msgs::HeftReference`。
- shell入口沿用所选DDS配置的网卡和发现设置，但仅为本进程将共享内存关闭，使用UDP。实机排查发现Python状态RPC走共享内存无回复，UDP可正常查询。系统 `/etc/cyclonedds/cyclonedds_shm.xml` 不会被修改。
- `schema_version=1`，`robot_version="roban_s17"`，`valid=true`，`calibrated=true`。这里不依赖Pico校准；关节角来自本次硬件反馈，根姿态来自IMU及几何估计。
- `seq` 每次发布递增；每次会话生成一个固定的 `calibration_epoch`。发布时间和采集时间随每帧更新。
- `q` 为21维弧度HEFT坐标，`qd` 为实际参考推进速度，腰角和腰速度均在DDS边界换向。
- 根四元数为wxyz，由torso IMU去除腰角得到；根高度由URDF双足接触几何估计，位置固定为 `[0,0,height]`。本脚本只输入关节姿态，不生成行走所需的完整根轨迹。
- 限位使用实机URDF和硬件JSON的交集；检查目标、参考及反馈。位置限位检查不代表任意21关节组合都具备平衡或避碰条件。
- BFM使用约50Hz策略，实机配置参考游标延迟为0.16秒；Python100Hz参考流不属于硬实时执行，也不保证编码器或末端精度。
- 反馈过期、实测角越限、DDS冲突或控制器实际回退时停止发送。单次本地调度间隔超过150ms不会单独永久退出，也不会按延误时间追赶目标。状态RPC暂时缺失时冻结目标，持续缺失时退出。
- Ctrl+C或终端断开会停止发送，不追加回位。HEFT失流回退由原控制器处理；配置为0.15秒新鲜度、0.5秒保持窗口、回退 `amp`，这不是本脚本的急停保证。

## 验证记录

2026-10-20已完成：

1. 本机和下位机各通过11项离线测试：坐标转换、限位、坏包、预热门控、UDP时效及乱序、多发送端、限速、失流、RPC失联、SSH输入转发及UDP配置继承。
2. 下位机通过独立 DDS domain187、`/codex_test/heft_external_21` 话题的实际收发、IDL序列化往返和发布者冲突检测。未向真实机器人参考话题发帧。
3. 新入口 shell 语法、帮助及依赖加载通过。默认只读试运行因主控制未启动，没有收到新鲜 `/rt/joint_state`，按预期退出。
4. 随后的现场排查已完成真实站姿只读采集：默认入口收到21关节反馈、根高度估计约0.7754m。手柄B切换和运动跟踪尚未验证。
5. 本次冲突GUID对应自动启动的Pico前端PID7795。再次用SDK查询确认AMP后，已仅停止该前端，启动器、硬件和主控制保留。该PID仅适用于本次记录，之后不得直接复用。
6. 修复了本外控进程的Python/C++共享内存RPC互通问题，采用UDP；系统DDS配置修改前后SHA256均为 `b98212481e4ce67c7632ea6e9487f536049bbcde51063b6719abaceb7f0f4f1e`。

依据已保存到本目录 `reference/`：HeftReference.idl、原dds_heft_pose_test及其kinematics/guard/observer/types依赖。原下位机参考脚本未被修改。
