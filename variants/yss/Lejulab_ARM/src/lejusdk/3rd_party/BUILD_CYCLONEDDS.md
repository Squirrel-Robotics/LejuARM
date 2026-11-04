# CycloneDDS + iceoryx 共享内存编译指南

本文档记录如何编译带有 iceoryx 共享内存支持的 CycloneDDS 静态库。

## 版本信息

- iceoryx: 2.0.5
- CycloneDDS: 0.10.2
- CycloneDDS-CXX: 0.10.2

## 快速编译

使用提供的脚本一键编译：

```bash
# 1. 先编译 iceoryx
./src/lejusdk/3rd_party/scripts/build_iceoryx.sh

# 2. 再编译 CycloneDDS (启用 SHM)
./src/lejusdk/3rd_party/scripts/build_cyclonedds.sh
```

编译完成后目录结构：

```
src/lejusdk/3rd_party/
├── iceoryx/                    # iceoryx 库
├── cyclonedds-0.10.2/          # CycloneDDS C 库 (启用 SHM)
├── cyclonedds-cxx-0.10.2/      # CycloneDDS C++ 库
├── build/                      # 临时编译目录
└── scripts/
    ├── build_iceoryx.sh
    └── build_cyclonedds.sh
```

## 手动编译步骤

### 1. 编译 iceoryx

```bash
# 安装依赖
sudo apt-get install -y cmake libacl1-dev libncurses5-dev build-essential wget

# 下载源码
cd /tmp
wget https://github.com/eclipse-iceoryx/iceoryx/archive/refs/tags/v2.0.5.tar.gz
tar -xzf v2.0.5.tar.gz
cd iceoryx-2.0.5

# 编译
cmake -B build -S iceoryx_meta \
    -DCMAKE_INSTALL_PREFIX=<安装路径>/iceoryx \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

cmake --build build -j$(nproc)
cmake --install build
```

### 2. 编译 CycloneDDS (C 核心库，启用 SHM)

```bash
cd /tmp
wget https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.10.2.tar.gz
tar -xzf 0.10.2.tar.gz
cd cyclonedds-0.10.2

cmake -B build -S . \
    -DCMAKE_INSTALL_PREFIX=<安装路径>/cyclonedds-0.10.2 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DENABLE_TOPIC_DISCOVERY=ON \
    -DENABLE_TYPE_DISCOVERY=ON \
    -DENABLE_SECURITY=NO \
    -DENABLE_SSL=NO \
    -DENABLE_SHM=ON \
    -DBUILD_DOCS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_PREFIX_PATH=<安装路径>/iceoryx

cmake --build build -j$(nproc)
cmake --install build
```

### 3. 编译 CycloneDDS-CXX (C++ 绑定库)

```bash
cd /tmp
wget https://github.com/eclipse-cyclonedds/cyclonedds-cxx/archive/refs/tags/0.10.2.tar.gz -O cyclonedds-cxx-0.10.2.tar.gz
tar -xzf cyclonedds-cxx-0.10.2.tar.gz
cd cyclonedds-cxx-0.10.2

cmake -B build -S . \
    -DCMAKE_INSTALL_PREFIX=<安装路径>/cyclonedds-cxx-0.10.2 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_DOCS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_PREFIX_PATH="<安装路径>/cyclonedds-0.10.2;<安装路径>/iceoryx"

cmake --build build -j$(nproc)
cmake --install build
```

## CMake 参数说明

| 参数 | 说明 |
|------|------|
| `-DBUILD_SHARED_LIBS=OFF` | 编译静态库 |
| `-DENABLE_SHM=ON` | **启用共享内存/iceoryx 支持** |
| `-DENABLE_TOPIC_DISCOVERY=ON` | 启用话题发现 |
| `-DENABLE_TYPE_DISCOVERY=ON` | 启用类型发现/XTypes |
| `-DCMAKE_PREFIX_PATH` | 指定 iceoryx 安装路径 |
| `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` | 生成位置无关代码 (PIC) |

## 在项目中使用

### CMakeLists.txt 配置

```cmake
# iceoryx + CycloneDDS (SHM enabled)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)
set(ICEORYX_DIR ${THIRD_PARTY_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)
```

## 运行时配置

### 1. 启动 RouDi 守护进程

使用 iceoryx 共享内存前，必须先启动 RouDi：

```bash
./scripts/iceoryx/start_roudi.sh
```

### 2. 配置 CycloneDDS 使用共享内存

设置环境变量指向 SHM 配置文件：

```bash
export CYCLONEDDS_URI="file://<项目路径>/config/cyclonedds_shm.xml"
```

或使用环境脚本：

```bash
source ./scripts/iceoryx/env.sh
```

### 3. 检查安装状态

```bash
./scripts/iceoryx/check.sh
```

## 验证共享内存是否生效

```bash
./scripts/iceoryx/verify_shm.sh
```

如果共享内存生效，`sendmsg` 系统调用应该接近 0。

## 注意事项

1. **RouDi 必须先启动**：在运行任何使用 DDS 的程序前，必须先启动 RouDi 守护进程
2. **静态链接**：iceoryx 和 CycloneDDS 都是静态链接，不需要安装到系统路径
3. **配置文件**：确保 `CYCLONEDDS_URI` 指向正确的 SHM 配置文件
4. **权限**：共享内存需要适当的系统权限，可能需要配置 `/dev/shm` 的权限

## 清理

```bash
# 清理临时编译目录
rm -rf src/lejusdk/3rd_party/build

# 完全清理（包括已编译的库）
rm -rf src/lejusdk/3rd_party/iceoryx
rm -rf src/lejusdk/3rd_party/cyclonedds-0.10.2
rm -rf src/lejusdk/3rd_party/cyclonedds-cxx-0.10.2
```

---
