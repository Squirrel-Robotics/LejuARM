# LejuSDK

Leju 机器人（Kuavo 和 Roban 系列）底层控制 SDK，提供电机控制、传感器数据订阅和实时通信接口。

## 目录结构

```
lejusdk/
├── 3rd_party/                 # 预编译 CycloneDDS 库
├── lejusdk-utils/             # 工具类（版本、日志、时间）
├── lejusdk-comm-dds/          # DDS 通信层
│   ├── lejusdk-dds-idl/       # IDL 定义及生成代码
│   └── lejusdk-topic-pubsub/  # DDS 发布/订阅封装
├── lejusdk-lowlevel/          # 主 SDK 库
│   ├── include/               # 公开 API
│   └── src/
│       ├── communication/     # DDS 通信实现
│       ├── robot_api/         # 机器人实现
│       └── robot_transmission/ # 数据传输层
├── docs/                      # 文档
├── gen_docs.sh                # 文档生成脚本
├── DEVELOPMENT.md             # 开发规范
└── README.md                  # 本文件
```

## 构建

从工作空间根目录 (`lejulab_platform/`)：

```bash
catkin build
```

## 文档

### 生成文档

```bash
./gen_docs.sh        # 生成 HTML + Markdown
./gen_docs.sh --clean # 清理
```

### 输出位置

- HTML：`docs/output/html/index.html`
- Markdown：`docs/output/markdown/`

### 开发规范

详细编码规范请参考 [DEVELOPMENT.md](DEVELOPMENT.md)

---

Copyright © 2024-2025 Leju Robotics
