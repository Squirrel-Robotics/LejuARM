# 通信探测模块（2026-11-05）

接收配置指向 `/home/lab/Lejulab_ARM` 后，该目录需要同时包含：

- `src/leju_launch/scripts/vr_communication_probe.py`：读取 CAN 接口状态、收发计数、串口设备及占用进程。
- `src/leju_launch/scripts/vr_dashboard_probe.py`：调用通信探测模块，在返回的状态快照中提供 `communication` 字段；探测异常时返回明确的不可用状态。

本次更新补齐这两个文件，文件内容与已部署修复的 SHA256 一致，记录在根目录 `SOURCE_SNAPSHOT.json`。
部署时将两个文件一同放入下位机主目录的对应位置；平台每轮 SSH 调用会启动独立探测进程，后续轮询即可读取更新。

下位机依赖 Python 3、PyYAML、iproute2 的 `ip` 和 psmisc 的 `fuser`。
串口占用查询使用已有读取权限；无法确认时返回未知状态。模块读取现有状态，不打开串口或发送 CAN 帧。

内核的 `ERROR-WARNING` 等状态原样保留。网页分别显示当前收发、累计计数和新增错误，不将持续收发视为可以隐藏内核告警。
