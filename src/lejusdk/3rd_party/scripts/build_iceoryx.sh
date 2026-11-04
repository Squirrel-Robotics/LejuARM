#!/bin/bash
#
# 编译 iceoryx
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRD_PARTY_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_DIR="$(dirname "$(dirname "$(dirname "${THIRD_PARTY_DIR}")")")"

# 配置
BUILD_DIR="${THIRD_PARTY_DIR}/build"
INSTALL_DIR="${THIRD_PARTY_DIR}/iceoryx"
VERSION="2.0.5"
TARBALL_URL="https://github.com/eclipse-iceoryx/iceoryx/archive/refs/tags/v${VERSION}.tar.gz"

# shellcheck source=../../../../scripts/build_env.sh
source "${PROJECT_DIR}/scripts/build_env.sh"
leju_detect_build_cores
JOBS=$LEJU_BUILD_JOBS

echo "============================================================"
echo "编译 iceoryx v${VERSION}"
echo "============================================================"
echo "BUILD_DIR:   ${BUILD_DIR}"
echo "INSTALL_DIR: ${INSTALL_DIR}"
echo ""

# 安装依赖
sudo apt-get update
sudo apt-get install -y cmake libacl1-dev libncurses5-dev build-essential wget

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# 下载源码
if [ -d "iceoryx-${VERSION}" ]; then
    echo "iceoryx-${VERSION} 目录已存在，跳过下载"
else
    echo "下载 iceoryx v${VERSION}..."
    wget -q --show-progress -O iceoryx-${VERSION}.tar.gz ${TARBALL_URL}
    tar -xzf iceoryx-${VERSION}.tar.gz
    rm iceoryx-${VERSION}.tar.gz
fi

cd "iceoryx-${VERSION}"

# 编译
cmake -B build -S iceoryx_meta \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

leju_cmake_build_dir build
cmake --install build

echo ""
echo "============================================================"
echo "iceoryx 编译完成"
echo "============================================================"
echo "安装路径: ${INSTALL_DIR}"
ls -la ${INSTALL_DIR}/lib/libiceoryx*.a 2>/dev/null || echo "警告: 未找到静态库"

# 清理源码目录 (避免 catkin 把源码当作 catkin 包)
echo ""
echo "清理编译目录..."
rm -rf "${BUILD_DIR}/iceoryx-${VERSION}"
echo "已删除: ${BUILD_DIR}/iceoryx-${VERSION}"
