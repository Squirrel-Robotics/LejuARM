# 同事电脑的两个平台

两个平台均已通过实际服务进程定位并导出源码。当前服务继续在原机器运行。

| 平台 | 局域网入口 | 仓库目录 |
| --- | --- | --- |
| 鲁班状态监测平台 | http://192.168.110.41:8765/ | `roban2-visual-console/` |
| 鲁班仿真与真机推理结果对比平台 | http://192.168.110.41:8766/ | `roban2-inference-compare/` |

来源路径及关键文件哈希见根目录 `SOURCE_SNAPSHOT.json`。原项目 README 与 `config.json`
保留实际部署记录；迁移时调整绝对路径和机器人 SSH 配置。

## 状态监测平台恢复

在 `platforms/roban2-visual-console/` 中执行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt
npm ci
bash run.sh start
```

`run.sh status` 查看地址。Python 锁文件和 `package-lock.json` 均来自原部署。
`assets/model/` 的身体、双手模型及网格已经包含；虚拟环境、node_modules 和现场记录不入库。
机器人连接使用本机已有 SSH 密钥及已核验主机密钥，需要在迁移电脑单独配置。
其相机启动工具依赖上位机既有 ROS 工作空间与独立相机 launch，具体路径见原项目 README。

## 仿真与真机推理结果对比平台恢复

该平台使用 Python 标准库，可复用相邻监测平台的 `.venv` 和本地 Three.js。
恢复监测平台依赖后，进入 `platforms/roban2-inference-compare/`：

1. 将 `config.example.json` 复制为 `config.json`，将 `runs_root` 改为实际 ACT 推理记录目录。
2. 确认 `console_root` 指向已恢复的 `roban2-visual-console`。
3. 在本目录执行 `bash run.sh start`，用 `bash run.sh status` 查看地址。

`config.example.json` 中的场景 XML 指向随本次归档保存的 `assets/scene_full_body.xml`。
平台只解析其中桌面、料箱等静态盒体参数；该 XML 并不代表完整独立的 MuJoCo 仿真资产包。
原 ACT 运行记录、照片、推理模型和完整仿真工程需要在使用时按实际数据位置提供，本次没有上传现场数据。
当前 `config.json` 仍保存原机器配置，方便追溯。

GitHub 保存源码，三个网页仍由原电脑提供服务。
