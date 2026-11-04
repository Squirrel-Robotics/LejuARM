# beta二次开发迁移（2026-10-29）

当前工程根目录：`/home/lab/yss/Lejulab_ARM`。

已迁入分段MCAP、60FPS RGB同步采集、本机校验传输、首尾5秒裁剪与固定50Hz对齐、网页采集接入、23维反馈启动检查、Pico参考更新优化、独立头部保持、21关节外控、HEFT精度扫描、头腰轨迹测试及两套灵巧手终端。

保留beta的SDK、主控/硬件二进制、控制器参数、按键绑定、根CMake和build_cmake。原有文件仅合入7个文件的二次开发补丁。迁移入口从自身路径定位工程，避免沿用旧LEJULAB_PROJECT_ROOT写回旧目录。独立灵巧手构建不修改根CMake。

## 正常启动

下次按原有停止/启动流程，在本目录使用原入口 `src/leju_launch/scripts/launch_teleop_real.sh`。硬件启动后执行最多12秒的23维反馈检查，然后继续原主控/Pico流程，自动启动相机和独立录制监听器。LT+Y开始、LT+X结束录制，或使用本机网页。

迁移未重启正在运行的硬件、主控和Pico。新启动逻辑与Pico默认50Hz在下次正常启动时加载。显式`--fps`仍可覆盖默认值。

本机工作台：`http://127.0.0.1:8877/`，局域网绑定设置沿用原配置。网页和接收服务已改用本工程工具；历史数据、归档、回收站、审核及3D资源保留。

## 独立工具

- 头部保持：`scripts/head_sdk_pose/head_sdk_pose.sh`，默认只读；实际发送需按原使用说明显式指定`--execute`。
- 21关节外控：`src/leju_launch/scripts/dds_heft_external_control.sh`。
- HEFT扫描/单关节：`dds_heft_pose_test.sh`、`dds_heft_joint_test.sh`，位于同一scripts目录。
- 头腰测试：`build/head_traj_tracking_test`、`build/waist_traj_tracking_test`；`--dry-run`不连接DDS。
- 双手终端：`build/src/revo2_hand_terminal/revo2_hand_terminal`。
- 右手直接SDK终端：`src/right_hand_canfd_terminal/right_hand_canfd_terminal`。

旧实录、旧测量快照、旧进程状态及旧二进制未迁入。首次运行HEFT精度测试应按说明只读采集当前姿态；默认基准文件不存在时会拒绝执行。`validation/migration_20261029/synthetic_baseline.json`仅用于离线计划验证，不能作为实机运动基准。

历史搬运低头二进制补丁和未接通的Y键拇指源码不在此次功能迁移范围；没有替换beta的控制器或VR库。

## 备份与验证

下位机原有文件备份及部署清单：`/home/lab/yss/Lejulab_ARM_migration_backup_20261029_features/`。本机完整备份（含.git、installed、build_cmake）与本机接入文件备份：`/media/yulinxie/EXTERNAL_USB/ROBAN/修改/2026-10-29_beta二次开发迁移/backup/`。

本次5个独立C++产物编译通过，下位机77项、本机105项及Pico数值6项测试通过。DDS213隔离验证采用模拟21维参考、23维反馈和真实RGB，不发布电机指令；完成原生MCAP录制、网页开始/手柄停止、传回本机、SHA256校验、缓存确认清理、首尾裁剪和50Hz对齐，612条样本全部匹配。真实机器人运动精度未在本次复测。

完整记录与逐文件哈希在本机迁移目录的evidence中。
