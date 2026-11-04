# 遥控器自启服务

机器上电后通过手柄 START 键拉起机器人 runtime，BACK+START 退出。

## 部署

```bash
sudo su
bash /path/to/lejulab_platform/src/leju-joystick/services/deploy_joy_autostart.sh
```

部署过程会同步安装当前机器人版本的 `teleop_bindings.yaml`。目标配置已存在时，
默认交互询问是否覆盖；自动化部署可显式选择策略：

```bash
# 保留现场按键配置
bash deploy_joy_autostart.sh --keep-teleop-config

# 用标准配置覆盖，并将原文件备份为 teleop_bindings.yaml.bak
bash deploy_joy_autostart.sh --overwrite-teleop-config
```

指定机器人版本：

```bash
ROBOT_VERSION=14 bash /path/to/lejulab_platform/src/leju-joystick/services/deploy_joy_autostart.sh
```

卸载：

```bash
bash deploy_joy_autostart.sh --remove
```

## 按键操作

- **START**：第一次拉起 tmux runtime，第二次触发机器人运动
- **BACK + START 同时按**：停止并退出 runtime

## 常用命令

```bash
systemctl status lejulab_joy_monitor.service   # 查看服务状态
journalctl -u lejulab_joy_monitor.service -f    # 查看日志
tmux attach -t lejulab_runtime                  # 进入 runtime 终端
```
