# Roban S17 RK3588 换板补充文件

这份归档用于同一台 Roban 2.2 / S17 更换下位机板卡。文件取自旧 TER30R（RK3588 EVB7 LP4 V10、内核 6.1.118），与旧机逐字一致。新板的厂商镜像和预装依赖按 [S17 恢复说明](../../docs/roban2-lower-recovery.md)核对。

`files/` 下的路径对应原机系统根目录。`manifest.json` 记录原权限（Git 只保留执行位）及 SHA256。在本目录可运行：

```bash
sha256sum -c SHA256SUMS
```

## 手柄服务与 8765 DDS

- `files/home/lab/roban2-gamepad/` 含实际部署的 ARM64 程序、重编所需源码和离线核心测试、DDS 配置以及状态查询的 Python 依赖。
- `files/etc/systemd/system/roban2-arm-jog.service` 是原机 unit；`files/home/lab/.config/lejuconfig/teleop_bindings.yaml` 是原机生效的按键映射，原机的 lab 与 root 两份内容一致。新机核对启动器实际读取哪份配置，保留厂商镜像中的关节标定与设备参数。
- 8765 读取器固定读取 `/home/lab/roban2-gamepad/dds-network.xml`，也使用 `/home/lab/Lejulab_ARM` 下的 CycloneDDS 库。即使不启用手柄扩展，仍须恢复此 XML，并核对绑定的机器人内网地址。
- 如需重编，在新下位机把程序目录放到原路径、确认主工程与 SDK 就绪后，运行 `bash build.sh`。它先跑离线核心测试，然后编译程序；不会启动手柄服务。
- unit 的入口是 `--execute`，可发动作。先保持未启用，核对 DDS 反馈、手柄映射及停止功能，再由现场人员决定何时启用；不要把旧机的 enabled 链接直接复制到新机。

## 联网与设备命名

| 文件 | 新板恢复时核对 |
| --- | --- |
| `etc/udev/rules.d/{60-persistent-net,61-persistent-tty,hipnuc_imu_serial}.rules` | 比对新板的 PCIe、SPI、USB 路径，确认 `lan1`、`bcan0` 到 `bcan3` 及串口别名都对应正确设备。 |
| `etc/NetworkManager/system-connections/lan1-con.nmconnection` | 有线机器人内网为 `192.168.26.1/24`，该连接不设默认路由。若采用此配置，文件须为 root 所有、权限 `0600`；避免让新旧板在同一内网同时使用同一地址。 |
| `etc/NetworkManager/conf.d/*.conf`、`etc/avahi/avahi-daemon.conf` | 与新镜像合并网络管理选项；保留需要的 `roban-lower.local` 别名，工作站按新板地址及已核验的 SSH 主机指纹连接。 |
| `etc/init.d/can_add_server.sh` 及同名 systemd 服务 | 这是旧机实际用的 CAN 配置，和主工程内的同名脚本有差异。比对新镜像已有的设置与接口映射，避免两套启动服务重复配置 CAN。 |
| `etc/modprobe.d/90-roban2-igb-recovery-20261010.conf`、蓝牙服务与脚本 | 旧板的故障恢复参考。网卡黑名单和蓝牙驱动重绑只在新板存在相同问题、且驱动和设备路径匹配时考虑采用。 |

逐项比对或安装，不要将 `files/` 整棵覆盖到系统。Wi-Fi 配置与密码、SSH 密钥和授权列表留在现场，新板单独联网和授权。系统镜像、预装依赖、运行日志和历史部署不在本目录内。
