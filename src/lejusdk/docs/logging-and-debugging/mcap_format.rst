MCAP 数据格式
==============

lejusdk-recorder 录制的数据以 **MCAP** （Message Capture）格式存储，这是一种行业标准的消息录制格式，兼容 Foxglove Studio 等可视化工具。

文件结构
--------

MCAP 文件由以下部分组成：

.. code-block:: text

   MCAP File
   ├── Header (profile: "cyclone-dds-recorder")
   ├── Schemas (OMG IDL 定义)
   │   ├── Schema 1: leju::msgs::JointState
   │   ├── Schema 2: leju::msgs::ImuData
   │   └── ...
   ├── Channels (话题)
   │   ├── Channel 1: /rt/joint_state -> Schema 1
   │   ├── Channel 2: /rt/imu_state -> Schema 2
   │   └── ...
   ├── Messages (CDR 编码的消息数据)
   │   ├── Message 1: channel=1, timestamp=..., data=[CDR bytes]
   │   ├── Message 2: channel=2, timestamp=..., data=[CDR bytes]
   │   └── ...
   └── Footer (索引信息，用于快速定位)

编码格式
--------

- **Schema encoding**: ``omgidl`` （OMG IDL 格式）
  - 每个 DDS 类型的 IDL 定义被转换为 OMG IDL 格式的 Schema
  - 支持通过 XTypes 动态发现的任意 DDS 类型

- **Message encoding**: ``cdr`` （Common Data Representation，DDS 标准序列化）
  - 消息数据使用 CDR 格式序列化，与 DDS 网络传输格式一致
  - 保留完整的类型信息和数据精度

时间戳
------

每条消息包含时间戳信息：

- **接收时间戳** （receiveTime）：recorder 接收到消息的时间
- **发布时间戳** （publishTime）：消息发布时的时间（若 DDS 提供）

Foxglove Studio 默认使用接收时间戳进行时间轴显示。

文件大小与压缩
--------------

- 默认启用压缩（zstd 或 lz4），可通过配置文件 ``output.compression`` 调整
- 支持文件分割：当达到 ``max_file_size`` 或 ``max_duration`` 时自动分割
- 分割后的文件命名： ``recording_YYYY-MM-DD-HH-MM-SS.mcap`` 等

兼容性
------

录制的 MCAP 文件可在以下工具中打开：

- **Foxglove Studio** （推荐）：https://foxglove.dev/studio
- 其他支持 MCAP 格式的工具

附: 官方 ``mcap`` CLI 安装
-----------------------------

除了 Foxglove Studio，Foxglove 还提供命令行工具 ``mcap``，用于查看文件元信息、切片、合并 bag 等。**常见坑**：

- **``npm i -g mcap-cli`` 装的不是这个** —— ``mcap-cli@0.4.0`` 是 mwaylabs 做的 app 脚手架，只是名字撞车。真的 MCAP 工具不在 npm 上，是 Foxglove 用 Go 写的。
- **``@mcap/cli`` 这个包不存在**，不要 ``npm i``安装。

推荐如下方式安装：

.. code-block:: bash

   mkdir -p ~/.local/bin
   # 版本号到 https://github.com/foxglove/mcap/releases 找最新 "releases/mcap-cli/vX.Y.Z"
   curl -fL "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.62/mcap-linux-amd64" \
     -o ~/.local/bin/mcap
   chmod +x ~/.local/bin/mcap
   mcap version   # -> v0.0.62

.. note::

   对于 ``cyclone-dds-recorder`` profile 的 MCAP （即 lejusdk-recorder 的录制产物），
   官方 ``mcap`` CLI 只能给出 header / statistics / 每 channel 的 message count；
   想解出 ``leju::msgs::*`` 消息内容，需要使用仓库内的 ``scripts/mcap/`` Python 脚本。

参考
----

- `MCAP 格式规范 <https://mcap.dev/>`_
- `Foxglove MCAP 文档 <https://foxglove.dev/docs/studio/connection/mcap>`_
- `mcap CLI releases <https://github.com/foxglove/mcap/releases?q=mcap-cli>`_
