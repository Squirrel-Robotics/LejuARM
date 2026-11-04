# VR摇操数据（本机处理）

后续LT+Y开始、LT+X结束的VR录制会由本机后台服务自动接收。下位机不再裁剪或保留已确认传输的VR原始数据；只在传输未确认时使用临时缓存。本机接收后逐文件核对SHA256和大小，确认原始数据已完整保存，再通知下位机删除对应临时会话。

数据根目录：`/media/yulinxie/EXTERNAL_USB/ROBAN/vr_Data`。

```text
vr_Data/
  YYYY_MM_DD_HH_MM/             # 同一分钟重复录制会追加序号，避免覆盖
    raw/
      Action_ObservationState/ # 未裁剪原始MCAP、原始说明JSON与录制日志
      vision/                  # 原始60FPS RGB视频、逐帧时间戳、相机参数和时钟信息
      processing_config.json   # 本段采集时的处理参数
      transfer_manifest.json   # 下位机生成的逐文件SHA256清单
    processed/
      Action_ObservationState/ # 首尾各裁5秒后的MCAP与说明JSON
      vision/                  # 裁剪视频、时间戳、alignment.csv、aligned_50hz.jsonl和独立检查报告
    receipt.json               # 接收校验及下位机缓存删除回执
    status.json                # ready表示裁剪与对齐检查通过
```

本机自动对MCAP和图像使用相同的下位机源时间窗口，首尾各删5秒，再按20ms连续网格做因果50Hz对齐。实际源参考、状态、图像的最大年龄仍分别为50ms、50ms、40ms；保留原始60FPS视频和逐帧真实时间戳。内部不匹配会标记processing_failed，原始数据仍在raw中，不会输出假合格结果。

后台服务：`roban-vr-workstation.service`，已启用登录自动启动。本机运行、外置盘挂载且能通过SSH连接下位机时会自动传输，通常在结束录制后几秒开始接收。下位机临时目录为`/home/lab/.cache/lejulab/vr_transfer`；本机离线或校验失败时暂留缓存，恢复连接后继续传输。旧数据不迁移。

查看接收服务：

```bash
systemctl --user status roban-vr-workstation.service
journalctl --user -u roban-vr-workstation.service -f
```

处理状态与详细结果见对应会话的status.json和processed/vision/audit_report.json。原始数据在本机始终保留。若需重试某段本机处理，先暂停接收服务避免两个工作进程同时操作：

```bash
systemctl --user stop roban-vr-workstation.service
python3 /media/yulinxie/EXTERNAL_USB/ROBAN/vr_Data/.pipeline/scripts/vr_workstation_receiver.py --retry YYYY_MM_DD_HH_MM
systemctl --user start roban-vr-workstation.service
```

`.pipeline`保存后台程序、独立Python依赖、传输进度和隔离联调证据；其中validation内的数据用于测试，不属于正式采集会话。

## Roban2 数据采集处理网页

本机打开：http://127.0.0.1:8877/ 。自动检查服务、显示手柄采集状态，并分别提示可以开始下一段和可以关闭下位机。还能查看视频与对齐报告。网页服务为roban-vr-dashboard.service，已设置登录自动启动。关闭网页不影响原有接收与裁剪服务。
