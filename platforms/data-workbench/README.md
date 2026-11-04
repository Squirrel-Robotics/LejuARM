# 鲁班数采、处理、增强平台

归档来源：本机 `/media/yulinxie/EXTERNAL_USB/ROBAN/vr_Data/.pipeline`。
当前入口：http://192.168.110.88:8877/#services 。功能和数据结构详见
`vr_Data/README.md` 与 `vr_Data/.pipeline/dashboard/README.md`。

## 文件组成

- `vr_Data/.pipeline/dashboard/`：Python HTTP 服务、前端及本地 Three.js 依赖。
- `vr_Data/.pipeline/scripts/`：接收、裁剪、50 Hz 对齐、手部处理、归档与清理程序。
- `vr_Data/.pipeline/urdf/`：审核页所需 URDF 和网格。
- `vr_Data/.pipeline/requirements.txt`：MCAP、LZ4、Zstandard 版本。
- `vr_Data/.pipeline/receiver_config.json`：当前接收端路径与 SSH 参数；不含私钥。
- `deploy/current/`：当前 systemd 服务定义。
- `upper/vr_vision_recorder/`：从上位机导出的相机配套代码。

## 在新电脑恢复

1. 准备 Python 3.10+、ffmpeg、OpenSSH 客户端及 rsync。
2. 选择数据目录，将这里 `vr_Data/.pipeline/` 的代码复制到该目录的 `.pipeline/` 下。
3. 将依赖装入代码已经使用的 `.pipeline/deps`：

   ```bash
   python3 -m pip install --target /实际数据目录/.pipeline/deps -r /实际数据目录/.pipeline/requirements.txt
   ```

4. 修改 `receiver_config.json` 的 `destination`、SSH 主机/地址、下位机 `remote_helper` 和缓存目录。
   当前 `remote_helper` 使用 `/home/lab/yss/Lejulab_ARM/...`，对应本仓库 `variants/yss/Lejulab_ARM/`。
5. 用自己的 SSH 连接配置和已核验主机密钥连接机器人；私钥单独配置。
6. 根据实际路径修改两个服务文件的 `WorkingDirectory`、`ExecStart`、PATH 及网页 `--allow-host`。
   复制到 `~/.config/systemd/user/` 后，可按部署需要执行 `systemctl --user daemon-reload` 与启用服务。
7. 上位机相机脚本依赖原机器上的 ROS Noetic、相机驱动和工作空间，按原服务文件调整路径。

仓库不包含采集会话、处理结果、任务/回收站状态、缓存或现场事件日志。
本次检查只验证归档完整性与代码语法，未在新电脑上执行机器人控制或恢复部署。
