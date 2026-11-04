# 右侧 Revo 2 灵巧手终端控制（传统 CAN / CAN FD）

这个程序只访问右手。它直接调用下位机已经安装的 BrainCo SDK，不依赖
`leju-hardware` 发布的手部状态，也不控制左手或手臂。程序不会修改 CAN
接口设置；接口模式与所选协议不符、读不到右手身份或读不到六路位置时，
运动按键会被锁住。

程序使用厂商 SDK 的 0–1000 位置范围：0 为张开，1000 为闭合。
启动后不会发运动命令，也不会默认选中全部关节。先按 `1`–`6` 选中一路，
再用 `j`/`k` 小步操作；默认步长为 10，运动命令之间至少间隔 250 毫秒。

## 放到下位机并编译

先确认 Mac 能用 `ssh roban-lower` 登录下位机。下面的 `192.168.110.60`
来自目前提供的 Mac SSH 配置；若下位机 IP 已变，请替换成实际地址。

以下第一条命令在 **Mac 终端**运行，把整个目录复制到下位机。
正常完成时会回到 Mac 命令提示符：

```bash
scp -r "/Users/jasonlee/Desktop/鲁班二号/right_hand_canfd_terminal" lab@192.168.110.60:/home/lab/Lejulab_ARM/src/
```

然后在 **Mac 终端**连接下位机。登录后提示符应包含 `lab@TER30R-ubuntu`：

```bash
ssh lab@192.168.110.60
```

出现 `lab@TER30R-ubuntu` 后，以下命令在 **下位机 SSH 窗口**运行，
只编译这个小程序，不修改整机工程配置：

```bash
bash /home/lab/Lejulab_ARM/src/right_hand_canfd_terminal/build_on_lower.sh
```

成功时输出 `Built /home/lab/Lejulab_ARM/src/right_hand_canfd_terminal/right_hand_canfd_terminal`。

## 启动

仍在 **下位机 SSH 窗口**。下面的 `bcan3` 是保存的 Leju 配置所写的
右手接口，配置为传统 CAN 模式。程序只会先读取设备信息，读到右侧
Revo 2 灵巧手及六路位置后才允许按键运动。

```bash
/home/lab/Lejulab_ARM/src/right_hand_canfd_terminal/right_hand_canfd_terminal --iface bcan3 --protocol can
```

程序先尝试厂商右手默认 ID `0x7f`，再尝试保存的 Leju 配置中的 `0x02`。
若已明确当前 ID，可指定 `--id 0x7f` 或 `--id 0x02`。
只有读到设备身份为右侧 Revo 2 灵巧手并得到六路位置，才进入按键界面。
厂商 SDK 支持 Revo 2 通过传统 CAN 通信，但这不证明这只手当前会在该模式下应答。
若之后有专用 CAN FD 接口，可用 `--protocol canfd`，但程序不会自行切换接口模式。

| 按键 | 作用 |
| --- | --- |
| `1`–`6` | 选一路：拇指、拇指辅助关节、食指、中指、无名指、小指 |
| `0` | 选全部六路 |
| `j` | 选中通道张开一小步 |
| `k` | 选中通道闭合一小步 |
| `-` / `+` | 减小 / 增大步长 |
| 空格 | 将六路目标设为当前读到的位置（保持）；不是急停 |
| `r` | 重新读取位置 |
| `q` | 退出 |

传统 CAN 模式要求接口 MTU 16；CAN FD 模式要求 MTU 72。
如果接口模式不匹配，程序不会发送任何运动命令。
如果显示 `No right Revo 2 hand answered`，则需要检查实际接口、ID、
通信速率和接线。程序不提供自动更改波特率或重配总线的操作。

不要同时运行其他会给同一只右手发指令的程序。遇到意外运动时使用实体急停；
退出程序只停止它继续发送指令。

这份程序尚未在换回的新右手上编译和实际操作。首次运行先按 `1` 选择
拇指，确认屏幕读数随小步操作变化，再尝试其他通道。若硬件服务也在向右手
持续发送目标，应先停掉那个控制源，避免指令互相覆盖。

API 依据：[BrainCo Revo 2 C++ SDK](https://brainco.tech/docs/revolimb-hand/en/revo2/c_sdk.html)。
