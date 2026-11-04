cmake_minimum_required(VERSION 3.16)

# ────────────────────────────────────────────────────────────────────
# 开源发布树根 CMakeLists（由 release.sh 拷贝到 staging/ 根）
#
# 与闭源开发树根 CMakeLists 的区别：
#   - lejusdk-lowlevel / lejusdk-vr 不从源码编译，而是 link 预编译 .so
#     （installed/lib/liblejusdk-lowlevel.so 等），由 lejusdk-stubs.cmake
#     定义为 IMPORTED target。
#   - dds-idl / topic-pubsub / dds-rpc 以 installed/ 中的预编译 SDK target 提供，源码不发布。
#   - leju-hardware / leju-rl-controller / recorder 是预编译产物（installed/bin），不参与源码编译；
#     csv-recorder、audio、monitor 只依赖 DDS SDK，按源码发布。
#   - CycloneDDS/iceoryx 的 aarch64 3rd_party 构建目录随源码依赖发布；运行时 .so
#     仍由 installed/lib（build-tree RPATH 自动指向）+ setup.bash 解析。
#
# 发布契约：release.sh 从闭源根 CMakeLists.txt 发现 source 包，生成
# source_packages.cmake；opensource_packages.txt 只声明 stub/binary 例外。
# 未声明为例外的上游 src 包默认整包发布。
#
# 部署用法（用户侧）：
#   cmake -B build -S . && cmake --build build -j
#   sudo bash src/leju-joystick/services/deploy_joy_autostart.sh
# ────────────────────────────────────────────────────────────────────

# 预编译 cyclonedds 静态库内嵌 GCC 11 LTO；系统默认 GCC 12+ 链接会失败。
if(NOT CMAKE_C_COMPILER AND NOT CMAKE_CXX_COMPILER)
    find_program(_GCC11 gcc-11)
    find_program(_GXX11 g++-11)
    if(_GCC11 AND _GXX11)
        set(CMAKE_C_COMPILER "${_GCC11}" CACHE FILEPATH "C compiler" FORCE)
        set(CMAKE_CXX_COMPILER "${_GXX11}" CACHE FILEPATH "C++ compiler" FORCE)
        message(STATUS "Auto-selected GCC 11: ${_GCC11}, ${_GXX11}")
    endif()
endif()

project(lejulab_platform_opensource VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# build-tree 二进制的 RPATH 指向 installed/lib（预编译 .so 所在），
# 这样 cmake --build 产出的可执行无需手动 LD_LIBRARY_PATH 即可运行。
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/installed/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_JOYSTICK "Build joystick (requires SDL3)" ON)
option(BUILD_VR_CONTROL
       "Build complete Quest3 VR chain (leju-ik + leju-remote + leju-vr-control; requires Drake)"
       ON)

# ── stub root = 本树 installed/ ──
set(_STUB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/installed")

# 统一初始化 DDS SDK stub 的第三方 imported targets。各功能包的局部
# find_package 不能可靠补齐 CycloneDDS-CXX 配置中引用的 iceoryx target。
set(_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/lejusdk/3rd_party")
set(THIRD_PARTY_DIR "${_THIRD_PARTY_DIR}")
include("${_THIRD_PARTY_DIR}/arch_select.cmake")
list(APPEND CMAKE_PREFIX_PATH
    "${THIRD_PARTY_ARCH_DIR}/iceoryx"
    "${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2"
    "${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2"
)
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

# 核心库作为 IMPORTED stub（预编译 .so + include/）。
# source_packages.cmake 由发布脚本根据闭源根 CMakeLists.txt 自动生成，
# manifest 只负责排除 stub/binary 包。
include(${CMAKE_CURRENT_SOURCE_DIR}/lejusdk-stubs.cmake)

include(${CMAKE_CURRENT_SOURCE_DIR}/source_packages.cmake)

# Layer 6: complete Quest3 VR chain. Drake is discovered from the target
# environment by default; DRAKE_PREFIX remains an explicit override.
if(BUILD_VR_CONTROL)
    set(_DRAKE_CONFIG "lib/cmake/drake/drake-config.cmake")
    set(_DRAKE_PREFIX_EXPLICIT FALSE)
    if(DRAKE_PREFIX)
        set(_DRAKE_PREFIX_EXPLICIT TRUE)
    elseif(DEFINED ENV{DRAKE_PREFIX} AND NOT "$ENV{DRAKE_PREFIX}" STREQUAL "")
        set(DRAKE_PREFIX "$ENV{DRAKE_PREFIX}")
        set(_DRAKE_PREFIX_EXPLICIT TRUE)
    endif()

    if(NOT DRAKE_PREFIX)
        if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
            set(_DRAKE_CANDIDATES "${CMAKE_SYSROOT}/opt/drake")
        elseif(NOT CMAKE_CROSSCOMPILING)
            set(_DRAKE_CANDIDATES /opt/drake /home/test/drake_install_prefix /usr/local /usr)
        endif()
        foreach(_candidate IN LISTS _DRAKE_CANDIDATES)
            if(EXISTS "${_candidate}/${_DRAKE_CONFIG}")
                set(DRAKE_PREFIX "${_candidate}")
                break()
            endif()
        endforeach()
    endif()

    if(NOT DRAKE_PREFIX OR NOT EXISTS "${DRAKE_PREFIX}/${_DRAKE_CONFIG}")
        if(_DRAKE_PREFIX_EXPLICIT)
            message(FATAL_ERROR
                    "BUILD_VR_CONTROL=ON but Drake was not found under DRAKE_PREFIX=${DRAKE_PREFIX}")
        else()
            message(FATAL_ERROR
                    "BUILD_VR_CONTROL=ON but Drake was not found in the standard install locations; "
                    "set -DDRAKE_PREFIX=<Drake install prefix> for a custom location")
        endif()
    endif()

    set(DRAKE_PREFIX "${DRAKE_PREFIX}" CACHE PATH "Drake install prefix" FORCE)
    list(APPEND CMAKE_PREFIX_PATH "${DRAKE_PREFIX}")
    message(STATUS "Found drake at: ${DRAKE_PREFIX}")
    find_package(drake CONFIG REQUIRED)
    add_subdirectory(src/leju-ik)
    add_subdirectory(src/leju-remote)
    add_subdirectory(src/leju-vr-control)
    unset(_DRAKE_CANDIDATES)
    unset(_DRAKE_CONFIG)
else()
    message(STATUS "BUILD_VR_CONTROL=OFF: skipping complete Quest3 VR chain")
endif()
