#!/bin/bash
#
# 编译 CycloneDDS (启用SHM)
# 注意：需要先运行 build_iceoryx.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRD_PARTY_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_DIR="$(dirname "$(dirname "$(dirname "${THIRD_PARTY_DIR}")")")"

# 配置
BUILD_DIR="${THIRD_PARTY_DIR}/build"
ICEORYX_DIR="${THIRD_PARTY_DIR}/iceoryx"
VERSION="0.10.2"
CYCLONE_INSTALL_DIR="${THIRD_PARTY_DIR}/cyclonedds-${VERSION}"
CYCLONE_CXX_INSTALL_DIR="${THIRD_PARTY_DIR}/cyclonedds-cxx-${VERSION}"

# shellcheck source=../../../../scripts/build_env.sh
source "${PROJECT_DIR}/scripts/build_env.sh"
leju_detect_build_cores
JOBS=$LEJU_BUILD_JOBS

# 下载地址
CYCLONE_URL="https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/${VERSION}.tar.gz"
CYCLONE_CXX_URL="https://github.com/eclipse-cyclonedds/cyclonedds-cxx/archive/refs/tags/${VERSION}.tar.gz"

# 检查iceoryx
if [ ! -d "${ICEORYX_DIR}/lib" ]; then
    echo "错误: iceoryx 未安装，请先运行 build_iceoryx.sh"
    exit 1
fi

echo "============================================================"
echo "编译 CycloneDDS ${VERSION} (启用SHM)"
echo "============================================================"
echo "ICEORYX_DIR:            ${ICEORYX_DIR}"
echo "CYCLONE_INSTALL_DIR:    ${CYCLONE_INSTALL_DIR}"
echo "CYCLONE_CXX_INSTALL_DIR: ${CYCLONE_CXX_INSTALL_DIR}"
echo ""

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# === CycloneDDS C 库 ===
echo ">>> 编译 CycloneDDS C 库..."

if [ -d "cyclonedds-${VERSION}" ]; then
    echo "cyclonedds-${VERSION} 目录已存在，跳过下载"
else
    echo "下载 cyclonedds ${VERSION}..."
    wget -q --show-progress -O cyclonedds-${VERSION}.tar.gz ${CYCLONE_URL}
    tar -xzf cyclonedds-${VERSION}.tar.gz
    rm cyclonedds-${VERSION}.tar.gz
fi

cd "cyclonedds-${VERSION}"

# 必须编译为共享库（BUILD_SHARED_LIBS=ON）：静态库会被以 hidden visibility
# 分别静态链接进每个 lejusdk .so（lowlevel/vr/dds-rpc），符号不导出，导致
# 同一进程内出现多份相互隔离的 CycloneDDS 运行时（GlobalRobot 与 VR 各建一份），
# VR 建 topic 时报 PRECONDITION_NOT_MET / "Could not create topic"。共享库使
# 全进程共用一份运行时，修复该问题。
cmake -B build -S . \
    -DCMAKE_INSTALL_PREFIX=${CYCLONE_INSTALL_DIR} \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DENABLE_TOPIC_DISCOVERY=ON \
    -DENABLE_TYPE_DISCOVERY=ON \
    -DENABLE_SECURITY=NO \
    -DENABLE_SSL=NO \
    -DENABLE_SHM=ON \
    -DBUILD_DOCS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_PREFIX_PATH=${ICEORYX_DIR}

leju_cmake_build_dir build
cmake --install build

cd "${BUILD_DIR}"

# === CycloneDDS C++ 库 ===
echo ""
echo ">>> 编译 CycloneDDS C++ 库..."

if [ -d "cyclonedds-cxx-${VERSION}" ]; then
    echo "cyclonedds-cxx-${VERSION} 目录已存在，跳过下载"
else
    echo "下载 cyclonedds-cxx ${VERSION}..."
    wget -q --show-progress -O cyclonedds-cxx-${VERSION}.tar.gz ${CYCLONE_CXX_URL}
    tar -xzf cyclonedds-cxx-${VERSION}.tar.gz
    rm cyclonedds-cxx-${VERSION}.tar.gz
fi

cd "cyclonedds-cxx-${VERSION}"

cmake -B build -S . \
    -DCMAKE_INSTALL_PREFIX=${CYCLONE_CXX_INSTALL_DIR} \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_DOCS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_PREFIX_PATH="${CYCLONE_INSTALL_DIR};${ICEORYX_DIR}"

leju_cmake_build_dir build
cmake --install build

echo ""
echo "============================================================"
echo "CycloneDDS (SHM) 编译完成"
echo "============================================================"
echo "CycloneDDS:     ${CYCLONE_INSTALL_DIR}"
echo "CycloneDDS-CXX: ${CYCLONE_CXX_INSTALL_DIR}"

# 清理源码目录 (避免 catkin 把源码当作 catkin 包)
echo ""
echo "清理编译目录..."
rm -rf "${BUILD_DIR}/cyclonedds-${VERSION}"
rm -rf "${BUILD_DIR}/cyclonedds-cxx-${VERSION}"
echo "已删除编译目录"

# 如果 build 目录为空则删除
rmdir "${BUILD_DIR}" 2>/dev/null && echo "已删除: ${BUILD_DIR}" || true
