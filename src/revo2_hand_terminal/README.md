# Roban2 左右灵巧手键盘控制

程序在下位机运行，Mac 通过 SSH 传入按键。沿用 Leju SDK 的 `HandCmd`：前六路左手，后六路右手，位置范围 0–100。沿用现有 VR 程序的约定：0 张开，100 闭合。

启动不发送运动指令，不默认选择手指通道。先选择左右手和通道，再按键小步操作；无需回车。六路仅编号 1–6，具体手指顺序以实机 SDK 为准。

| 按键 | 功能 |
| --- | --- |
| `a` / `d` / `b` | 左手 / 右手 / 双手 |
| `1`–`6` | 选择单路通道 |
| `0` | 选择全部六路 |
| `j` / `k` | 张开 / 闭合一小步 |
| `-` / `+`（或 `=`） | 调小 / 调大步长，默认 2，范围 0.5–10 |
| 空格 | 发送双手当前实测位置作为保持目标 |
| `q` 或 `Ctrl+C` | 退出并恢复终端输入 |

每次调节从最新实测位置开始，另一侧及未选通道采用本次实测位置。增量指令至少间隔 250 毫秒。屏幕显示实测位置和锁定原因。

## 复制与编译

在 **Mac 本地终端**执行，将程序目录复制到下位机。`roban-lower` 是之前设置的 SSH 别名；若不存在，用当前可连接的 `lab@下位机地址` 替换。成功后回到 Mac 提示符。

```bash
scp -r "/Users/jasonlee/Desktop/鲁班二号/revo2_hand_terminal" roban-lower:/home/lab/Lejulab_ARM/src/
```

在 **已连接下位机的 SSH 窗口**执行，编译本程序。脚本检查工程、备份根 CMakeLists.txt 并按需加入构建项。成功显示 `Built .../revo2_hand_terminal`。

```bash
bash /home/lab/Lejulab_ARM/src/revo2_hand_terminal/build_on_lower.sh
```

依赖下位机的 `lejusdk-lowlevel`、`lejusdk-utils`，以及带 `left_valid`、`right_valid`、`left_sample_age_ms`、`right_sample_age_ms` 字段的 `HandState`。若编译报字段不存在，保留错误信息以核对 SDK 版本，不能直接删掉反馈检查。

## 运行

机器人版本 17 的硬件服务须按你们现有流程运行，并识别两只手。不要同时运行其他给同一双手发目标的 VR 或键盘程序。

在 **下位机 SSH 窗口**执行，进入中文按键界面。下方使用已有工程记录中的共享内存 DDS 配置；应与硬件服务的实际配置一致。程序不会启动硬件服务或更改 CAN 设置。

```bash
ROBOT_VERSION=17 \
CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml \
/home/lab/Lejulab_ARM/build/src/revo2_hand_terminal/revo2_hand_terminal
```

例如：`a` → `1` → `k` 调节左手第一路闭合；`d` → `1` → `j` 调节右手第一路张开；`b` → `0` → `k` 调节双手全部六路闭合。

程序要求硬件为 `READY_OK` 且两只手均有有效反馈。手部反馈超过 1 秒未更新、设备采样过期或无效、数值越界时拒绝发送指令。首次进入先选择单路，小步检查实测读数。

空格保持不是急停。退出只停止本程序发送目标，不会撤销设备已有目标；异常运动使用实体急停。

仅在左手物理断开时使用 `--right-only`；该模式仍要求右手有效反馈，将 12 路命令的左侧六路填 0。正常双手连接时不用该模式。可加 `--commands` 使用逐行输入，`--help` 查看帮助（不初始化 SDK）。

## 验证范围

本次在 Mac 上使用模拟 SDK 检查控制逻辑，包括未就绪拒发、左右通道隔离、范围限制、过期反馈拒发和保持目标。模拟检查不验证 SDK 二进制兼容性或实机运动，仍需在下位机编译并核对反馈。
