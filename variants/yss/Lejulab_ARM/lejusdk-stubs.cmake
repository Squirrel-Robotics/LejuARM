# lejusdk 核心库的开源 IMPORTED stub
# 由开源根 CMakeLists.txt 在 add_subdirectory(lejusdk-utils) 之后 include()
#
# 对齐上游 .lejulab stub 的语义：给 include/ + 预编译库，藏实现。
# 不声明 cyclonedds/iceoryx/eigen —— liblejusdk-lowlevel.so 的公开头不暴露 DDS
# （DDS 是 private 实现细节，已静态进 .so）；运行期 NEEDED 由 setup.bash 的
# LD_LIBRARY_PATH=installed/lib 解析（libddscxx.so/libddsc.so 已由 release.sh 第 3 步拷入）。
#
# 前置：调用方须已定义 _STUB_ROOT（=installed/ 根），并已 add lejusdk-utils target
#       （lowlevel/vr 公开头 #include "lejusdk-utils/robot_version.hpp"）。
# DDS SDK target 的公共头位于 installed/include，静态库位于 installed/lib。

set(_STUB_LIBDIR  "${_STUB_ROOT}/lib")
set(_STUB_INCDIR  "${_STUB_ROOT}/include")

add_library(lejusdk-lowlevel SHARED IMPORTED GLOBAL)
set_target_properties(lejusdk-lowlevel PROPERTIES
    IMPORTED_LOCATION             "${_STUB_LIBDIR}/liblejusdk-lowlevel.so"
    INTERFACE_INCLUDE_DIRECTORIES "${_STUB_INCDIR}"
    INTERFACE_LINK_LIBRARIES       lejusdk-utils
)

add_library(lejusdk-vr SHARED IMPORTED GLOBAL)
set_target_properties(lejusdk-vr PROPERTIES
    IMPORTED_LOCATION             "${_STUB_LIBDIR}/liblejusdk-vr.so"
    INTERFACE_INCLUDE_DIRECTORIES "${_STUB_INCDIR}"
    INTERFACE_LINK_LIBRARIES       lejusdk-utils
)

# DDS SDK 的实现保持闭源，但其 ABI/API 以 IMPORTED targets 提供给开源功能包。
# IDL/RPC 当前是静态库，topic-pubsub 是 header-only 接口。
add_library(lejusdk-dds-idl STATIC IMPORTED GLOBAL)
set_target_properties(lejusdk-dds-idl PROPERTIES
    IMPORTED_LOCATION             "${_STUB_LIBDIR}/liblejusdk-dds-idl.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_STUB_INCDIR}"
    INTERFACE_LINK_LIBRARIES       "CycloneDDS-CXX::ddscxx;pthread"
)

add_library(lejusdk-topic-pubsub INTERFACE IMPORTED GLOBAL)
set_target_properties(lejusdk-topic-pubsub PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_STUB_INCDIR}"
)

add_library(lejusdk-dds-rpc STATIC IMPORTED GLOBAL)
set_target_properties(lejusdk-dds-rpc PROPERTIES
    IMPORTED_LOCATION             "${_STUB_LIBDIR}/liblejusdk-dds-rpc.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_STUB_INCDIR}"
    INTERFACE_LINK_LIBRARIES       "CycloneDDS-CXX::ddscxx;pthread"
)
