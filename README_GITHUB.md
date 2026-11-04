# LejuARM 与鲁班平台源码归档

本仓库是 2026-11-04 从实际部署目录导出的当前文件快照，包含尚未在原仓库提交的修改。
原项目说明仍见 [README.md](README.md)、[README_DEPLOY.md](README_DEPLOY.md) 和
[README_HEFT.md](README_HEFT.md)。快照来源与版本见 [SOURCE_SNAPSHOT.json](SOURCE_SNAPSHOT.json)。

## 目录与实际部署

| 仓库路径 | 来源 / 用途 |
| --- | --- |
| 根目录 `src/`、`installed/`、`config/`、`scripts/` 等 | 下位机 `/home/lab/Lejulab_ARM` 主目录 |
| `variants/yss/Lejulab_ARM/` | 下位机 `/home/lab/yss/Lejulab_ARM`，当前数采平台连接的版本 |
| `platforms/data-workbench/vr_Data/.pipeline/` | 本机 8877 数采、处理、增强平台的当前代码、模型、依赖和接收配置 |
| `platforms/roban2-visual-console/` | 同事电脑 8765 状态监测平台，含后端、前端、模型和依赖锁文件 |
| `platforms/roban2-inference-compare/` | 同事电脑 8766 对比平台，含后端、前端、启动脚本和场景 XML |
| `platforms/data-workbench/deploy/current/` | 本机当前 systemd 用户服务文件 |
| `platforms/data-workbench/upper/vr_vision_recorder/` | 上位机 RGB 采集、预览和深度查看所需的配套脚本 |

## 三个平台

| 平台 | 当前局域网入口 | 源码归档状态 |
| --- | --- | --- |
| 鲁班状态监测平台 | http://192.168.110.41:8765/ | 已包含源码，详见 `platforms/colleague-platforms.md` |
| 鲁班数采、处理、增强平台（待完善） | http://192.168.110.88:8877/#services | 已包含本机部署源码及配套脚本 |
| 鲁班仿真与真机推理结果对比平台 | http://192.168.110.41:8766/ | 已包含源码，详见 `platforms/colleague-platforms.md` |

GitHub 保存代码和版本。以上服务仍在原电脑运行；上传不会把服务托管到 GitHub。
同事电脑的平台应保存后端、前端、依赖、模型配置和启动方式，网页文件本身不足以恢复服务。

## 恢复与迁移

下位机主版本按原项目 README 构建；`installed/` 中存在上游提供的 ARM 预编译核心库，
因此保留这些运行依赖。当前数采实际使用 `variants/yss/Lejulab_ARM/`，不要将两个版本混合覆盖。

数采平台的恢复说明见 [platforms/data-workbench/README.md](platforms/data-workbench/README.md)。
`deploy/current/` 和 `receiver_config.json` 记录当前部署路径及局域网地址；迁移电脑时需按实际环境调整。

## 归档范围

保留必要策略模型、URDF/网格、上游预编译 SDK 和部署资源。
录制 CSV、MCAP、视频、运行日志、验证输出、临时备份、构建缓存、Python 依赖安装目录和密钥不入库。
这些数据仍留在原机器，原机器的服务与 Git 配置不受本次归档影响。

此仓库从当前文件创建独立 Git 历史，不导入上游历史。上游地址与基线提交保存在快照说明中。
各文件原有许可与上游使用条款继续适用；本次归档不重新授予第三方代码或二进制的许可。
