# DDS Topic 冲突问题排查与修复报告

## 问题描述

当同时使用 `lejusdk-lowlevel` 和 `lejusdk-vr` 两个库时，VR 初始化失败并报错：

```
[VRCommunication] Failed to initialize: Error Precondition Not Met - Could not create topic.
Context: dds::topic::detail::Topic<T>::Topic
```

## 问题现象

1. **VR 单独运行**：正常工作
2. **lowlevel 先初始化，VR 后初始化**：VR 失败
3. **两个库链接在一起，即使只用 VR**：VR 也失败

## 排查过程

### 1. 初步分析

首先检查了两个库使用的 Topic 名称：

- **lowlevel**: `/rt/joint_cmd`, `/rt/joint_state`, `/rt/imu_state` 等
- **VR**: `/rt/arm_trajectory`, `/rt/head_trajectory`, `/rt/cmd_vel` 等

Topic 名称完全不同，不存在命名冲突。

### 2. 类型分析

检查两个库使用的 DDS 消息类型：

- **lowlevel**: `JointCmd`, `JointState`, `ImuData`, `Joy`, `StringData`
- **VR**: `JointTrajectoryPoint`, `VelocityCmd`

消息类型也不同，不存在类型冲突。

### 3. 尝试方案

#### 方案 1: 升级 CycloneDDS 版本 (0.10.2 → 0.10.5)

**结果**：失败，问题依旧存在

#### 方案 2: 使用不同的 Domain ID

将 VR 的 `kDefaultDomainId` 从 0 改为 1。

**结果**：失败，问题依旧存在

日志显示两个库确实使用了不同的 domain：
```
[0] test_dds_c: ...  (lowlevel 使用 domain 0)
[1] test_dds_c: ...  (VR 使用 domain 1)
```

但仍然报 `PRECONDITION_NOT_MET` 错误。

### 4. 深入分析：CycloneDDS 源码

通过分析 CycloneDDS 和 CycloneDDS-CXX 源码，发现了根本原因。

## 根本原因

### 静态链接导致的符号隔离问题

CycloneDDS 被编译为**静态库** (`libddsc.a`, `libddscxx.a`)，导致：

```
test_dds_conflict (可执行文件)
├── lejusdk-lowlevel.so
│   └── 包含完整的 CycloneDDS 静态副本 (~10MB)
└── lejusdk-vr.so
    └── 包含另一个完整的 CycloneDDS 静态副本 (~10MB)
```

### 技术细节

1. **两个独立的 CycloneDDS 运行时**：
   - 每个 `.so` 文件都有自己的全局域管理器 (`ddsi_domaingv`)
   - 各自独立的 sertype 注册表
   - 各自独立的内部状态变量

2. **共享内存冲突**：
   - CycloneDDS + iceoryx 使用共享内存进行跨进程通信
   - lowlevel 的 CycloneDDS 创建 topic 时更新共享内存元数据
   - VR 的 CycloneDDS 读取共享内存时发现已存在的类型信息
   - 但由于是不同的库实例，sertype 的 `ops` 指针地址不同

3. **Sertype 等价性检查失败**：

   CycloneDDS 使用指针比较来判断 sertype 是否相等：
   ```c
   // ddsi_sertype.c
   bool ddsi_sertype_equal(const ddsi_sertype *a, const ddsi_sertype *b) {
       if (a->ops != b->ops)           // 指针地址比较！
           return false;
       if (a->serdata_ops != b->serdata_ops)
           return false;
       // ...
   }
   ```

4. **C++ 模板静态成员问题**：

   `ddscxx_sertype<T>` 的静态成员 `sertype_ops` 和 `serdata_ops` 在每个翻译单元中独立实例化：
   ```cpp
   template <typename T, class S>
   class ddscxx_sertype : public ddsi_sertype {
   public:
       static const ddscxx_sertype_ops<T,S> sertype_ops;  // 每个 .so 有独立副本
       static const ddscxx_serdata_ops<T,S> serdata_ops;  // 每个 .so 有独立副本
   };
   ```

   即使是相同的类型定义，两个 `.so` 中的静态指针地址也不同，导致被判定为"不兼容的类型"。

## 解决方案

### 将 CycloneDDS 从静态库改为共享库

修改 `3rd_party/scripts/build_cyclonedds.sh`：

```diff
- -DBUILD_SHARED_LIBS=OFF \
+ -DBUILD_SHARED_LIBS=ON \
```

### 原理

使用共享库后：

```
test_dds_conflict (可执行文件)
├── lejusdk-lowlevel.so ─┬─→ libddsc.so (进程中只有一份)
└── lejusdk-vr.so ───────┘   libddscxx.so (进程中只有一份)
```

所有 sertype 的 `ops` 指针地址一致，不再产生冲突。

### 修改的文件

1. **`3rd_party/scripts/build_cyclonedds.sh`**：
   - CycloneDDS C 库：`BUILD_SHARED_LIBS=ON`
   - CycloneDDS C++ 库：`BUILD_SHARED_LIBS=ON`

2. **`lejusdk-comm-dds/lejusdk-dds-idl/CMakeLists.txt`**：
   - 添加 `find_package(iceoryx_binding_c REQUIRED)`（共享库模式需要此依赖）

## 验证结果

```
=== DDS Conflict Test ===
Testing simultaneous use of lejusdk-lowlevel and lejusdk-vr

[Step 1] Initializing lowlevel SDK...
TopicPublisher initialized successfully for topic: /rt/joint_cmd
...
[Step 1] lowlevel SDK initialized successfully

[Step 2] Initializing VR API...
TopicPublisher initialized successfully for topic: /rt/arm_trajectory
TopicPublisher initialized successfully for topic: /rt/head_trajectory
...
[Step 2] VR API initialized successfully

[Step 3] Testing functionality...
  - lowlevel publishRobotCmd: OK
  - VR publishArmJointCmd: OK

=== Test Complete ===
Both APIs work correctly together!
```

## 注意事项

1. **部署时需要确保 CycloneDDS 共享库可被找到**：
   - 设置 `LD_LIBRARY_PATH` 或
   - 将库安装到系统路径

2. **如果需要重新编译 CycloneDDS**：
   ```bash
   cd src/lejusdk/3rd_party/scripts
   rm -rf ../cyclonedds-0.10.2 ../cyclonedds-cxx-0.10.2
   ./build_cyclonedds.sh
   ```

3. **库文件对比**：

   | 库 | 静态版本 | 共享版本 |
   |---|---------|---------|
   | libddsc | 10.7 MB (.a) | 2.3 MB (.so) |
   | libddscxx | 2.7 MB (.a) | 1.4 MB (.so) |

## 总结

这是一个由**静态链接导致的符号隔离问题**，而非 CycloneDDS 本身的 bug。当多个共享库各自静态链接同一个库时，会产生多个独立的运行时实例，导致内部状态不一致。解决方案是将底层依赖改为共享库，确保进程中只有一份实例。
