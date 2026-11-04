# 开发规范

本文档说明 LejuSDK 项目中应遵循的编码风格、注释规范和开发流程。

## 目录

- [命名规范](#命名规范)
- [代码格式](#代码格式)
- [注释规范](#注释规范)
- [头文件规范](#头文件规范)
- [Doxygen 标签参考](#doxygen-标签参考)
- [新增模块指南](#新增模块指南)

---

## 命名规范

### 类名（PascalCase）

```cpp
class RobotBaseAPI { };
class GlobalRobot { };
class KuavoHumanoid { };
```

### 函数/方法（camelCase）

```cpp
void subscribeImuData(const ImuDataCallback& callback);
bool publishRobotCmd(const RobotCmd& cmd);
uint8_t getMotorNumber() const;
```

### 变量（snake_case）

- 普通变量：全小写，下划线分隔
- 成员变量：尾部加下划线

```cpp
std::vector<double> q;           // 普通变量
RobotVersion robot_version_;     // 成员变量
double timestamp;                // 普通变量
```

### 常量/宏（UPPER_SNAKE_CASE）

```cpp
#define LEJU_SDK_VERSION_MAJOR 1
#define MAX_MOTOR_COUNT 50

enum class MotorControlMode : uint8_t {
    CST = 0,  // 力矩控制
    CSV = 1,  // 速度控制
    CSP = 2,  // 位置控制
};
```

### 类型别名（PascalCase，Ptr/ConstPtr 后缀）

```cpp
using RobotStatePtr = std::shared_ptr<leju::RobotState>;
using RobotStateConstPtr = std::shared_ptr<const leju::RobotState>;
using ImuDataCallback = std::function<void(const ImuDataConstPtr&)>;
```

### 枚举类（PascalCase，枚举值 UPPER_SNAKE_CASE）

```cpp
enum class HardwareState {
    UNKNOWN = 0,
    STOPPED,
    ERROR,
    INITIALIZING,
    READY_OK,
};
```

---

## 代码格式

### 访问修饰符

类/结构的访问修饰符使用 2 空格缩进，与类体对齐：

```cpp
class RobotBaseAPI {
public:
  virtual void subscribeImuData(const ImuDataCallback& callback);

protected:
  RobotVersion robot_version_;

private:
  std::mutex mutex_;
};
```

### 函数定义

左大括号不换行：

```cpp
bool publishRobotCmd(const RobotCmd& cmd) {
    // 实现
}
```

### 头文件保护宏

使用全大写，前后各加一个下划线：

```cpp
#ifndef _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_
#define _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_

// 内容...

#endif  // _LEJU_SDK_LOW_LEVEL_DATAT_YPES_H_
```

### 命名空间

使用 `namespace leju` 包裹所有公开 API，结尾添加注释：

```cpp
namespace leju {

// 类、函数、类型定义...

}  // namespace leju
```

---

## 注释规范

LejuSDK 使用 **Doxygen** 风格的注释，用于自动生成 API 文档。

### 文件头注释

每个头文件顶部应包含文件说明：

```cpp
/**
 * @file data_types.h
 * @brief 机器人底层SDK数据类型定义
 * @author Leju Robotics
 * @version 1.0.0
 *
 * @details 本文件定义了机器人底层控制所需的核心数据结构，包括：
 * - 电机控制模式枚举
 * - 机器人状态数据结构
 * - 机器人控制指令数据结构
 *
 * 所有数据结构均提供智能指针类型别名，便于在回调函数中使用。
 */
```

### 类注释

```cpp
/**
 * @brief 全局机器人管理器
 *
 * 提供统一的机器人创建和管理接口，确保全局只有一个机器人实例。
 * 支持类型安全的访问和版本管理。
 */
class GlobalRobot {
    // ...
};
```

### 结构体注释

```cpp
/**
 * @struct RobotState
 * @brief 机器人状态结构体
 *
 * 用于封装机器人电机的状态信息，包括位置、速度、加速度和力矩等反馈数据。
 * 该结构体通常由底层驱动填充，通过回调函数传递给上层应用。
 *
 * @note 所有向量的维度应与机器人电机数量一致，可通过 resize() 方法调整。
 *
 * @see RobotCmd 机器人控制指令结构体
 * @see RobotStatePtr 共享指针类型别名
 */
struct RobotState {
    // ...
};
```

### 枚举注释

```cpp
/**
 * @enum MotorControlMode
 * @brief 电机控制模式枚举
 *
 * 定义了机器人电机支持的三种控制模式，用于指定电机的运行方式。
 *
 * @note 控制模式的选择取决于具体的应用场景：
 * - 力矩控制适用于力控场景和柔顺控制
 * - 速度控制适用于匀速运动场景
 * - 位置控制适用于精确定位场景
 */
enum class MotorControlMode : uint8_t {
    CST = 0,  ///< 力矩控制模式 (Cyclic Synchronous Torque)
    CSV = 1,  ///< 速度控制模式 (Cyclic Synchronous Velocity)
    CSP = 2,  ///< 位置控制模式 (Cyclic Synchronous Position)
};
```

### 函数注释

```cpp
/**
 * @brief 初始化机器人运行环境
 *
 * 创建并初始化指定类型和版本的机器人实例。此方法只能调用一次，
 * 重复调用将抛出异常。
 *
 * @param version 机器人版本信息
 * @return bool 初始化是否成功
 * @throws std::runtime_error 如果机器人已经初始化或者类型不匹配
 */
static bool init_env(const RobotVersion& version);
```

### 成员变量注释

使用行尾注释 `///<`：

```cpp
struct RobotState {
    std::vector<double> q;        ///< 电机位置数组，单位: rad
    std::vector<double> v;        ///< 电机速度数组，单位: rad/s
    std::vector<double> tau;      ///< 电机力矩数组，单位: N·m
    double timestamp;             ///< 时间戳，单位: s
};
```

### 类型别名注释

```cpp
/**
 * @typedef RobotStateConstPtr
 * @brief RobotState 的常量共享指针类型
 *
 * 用于回调函数参数，避免不必要的数据拷贝。
 */
using RobotStateConstPtr = std::shared_ptr<const leju::RobotState>;
```

### 代码示例注释

使用 `@code ... @endcode` 包裹示例代码：

```cpp
/**
 * @brief 获取已初始化的机器人实例
 *
 * 必须在调用 init_env() 之后使用，否则将抛出异常。
 *
 * @code{.cpp}
 * // 从环境变量中获取机器人版本信息
 * auto robot_version = RobotVersion::from_env();
 * if (GlobalRobot::init_env(robot_version)) {
 *     auto& robot = GlobalRobot::getInstance();
 *     robot.publishRobotCmd(cmd);
 * }
 * @endcode
 *
 * @return RobotBaseAPI& 机器人实例的引用
 * @throws std::runtime_error 如果机器人尚未初始化
 */
static RobotBaseAPI& getInstance();
```

### 内部实现标记

对于不公开的内部实现，使用 `@cond INTERNAL ... @endcond`：

```cpp
/** @cond INTERNAL */
private:
    static std::unique_ptr<RobotBaseAPI> createRobot(const RobotVersion& version);
    static std::unique_ptr<RobotBaseAPI> instance_;
    static std::mutex mutex_;
/** @endcond */
```

### TODO 注释

使用 `TODO(作者):` 格式标记待办事项：

```cpp
// TODO(ohh): add get ee_type.
// struct EndEffectorState { };
```

---

## 头文件规范

### 包含顺序

1. 对应的头文件（如果是 .cpp 文件）
2. 同项目内的头文件
3. 第三方库头文件
4. 标准库头文件

```cpp
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-utils/robot_version.hpp"

#include <memory>
#include <string>
#include <vector>
```

### 前向声明

优先使用前向声明减少编译依赖：

```cpp
namespace leju {

// Forward declarations
class RobotBaseAPI;
struct RobotState;

}  // namespace leju
```

---

## Doxygen 标签参考

| 标签 | 用途 | 示例 |
|------|------|------|
| `@file` | 文件名 | `@file data_types.h` |
| `@brief` | 简短描述 | `@brief 机器人状态结构体` |
| `@details` | 详细描述 | `@details 本文件定义了...` |
| `@author` | 作者 | `@author Leju Robotics` |
| `@version` | 版本 | `@version 1.0.0` |
| `@param` | 参数说明 | `@param version 机器人版本信息` |
| `@return` | 返回值说明 | `@return bool 初始化是否成功` |
| `@throws` | 异常说明 | `@throws std::runtime_error 如果...` |
| `@note` | 注意事项 | `@note 所有向量维度应一致` |
| `@warning` | 警告 | `@warning 此函数非线程安全` |
| `@see` | 参照 | `@see RobotCmd 控制指令结构体` |
| `@code` | 代码示例开始 | `@code{.cpp}` |
| `@endcode` | 代码示例结束 | `@endcode` |
| `@cond` | 条件编译开始 | `@cond INTERNAL` |
| `@endcond` | 条件编译结束 | `@endcond` |
| `@struct` | 结构体 | `@struct RobotState` |
| `@class` | 类 | `@class GlobalRobot` |
| `@enum` | 枚举 | `@enum MotorControlMode` |
| `@typedef` | 类型别名 | `@typedef RobotStatePtr` |
| `///<` | 行尾注释 | `double timestamp; ///< 时间戳` |

---

## 新增模块指南

新增模块时需要修改以下文件：

### 1. docs/Doxyfile

添加模块头文件路径：

```bash
INPUT = ../lejusdk-lowlevel/include \
        ../lejusdk-utils/include \
        ../lejusdk-highlevel/include   # 新增模块
```

### 2. docs/modules/index.rst

在 toctree 中添加模块：

```rst
.. toctree::
   :maxdepth: 2

   lejusdk-lowlevel/index
   lejusdk-highlevel/index   # 新增模块
```

### 3. docs/modules/新模块/index.rst

创建模块文档和示例链接：

```rst
lejusdk-highlevel
=================

模块简介...

API 参考
--------

.. doxygenclass:: leju::SomeClass
   :project: LejuSDK
   :members:

示例
----

.. toctree::
   :maxdepth: 1

   examples
```

### 4. docs/modules/新模块/examples.rst

创建示例文件。
