cmake_minimum_required(VERSION 3.16)
project(lejusdk-claw-python)
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils)
if(DEFINED CATKIN_DEVEL_PREFIX AND NOT catkin_FOUND)
  # QUIET 会吞掉缺组件的原因，会让链接阶段才暴露成 ld: cannot find -llejusdk-lowlevel。
  message(WARNING "lejusdk-claw-python: 未找到 catkin 组件 lejusdk-lowlevel/lejusdk-utils，"
                  "将改为按工作区 devel/lib 下的绝对路径链接。")
endif()
if(catkin_FOUND)
  catkin_package(LIBRARIES lejusdk_claw CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils)
endif()
add_library(lejusdk_claw SHARED src/claw_client.cpp)
target_compile_features(lejusdk_claw PRIVATE cxx_std_17)
target_include_directories(lejusdk_claw PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk-recorder/thirdparty/nlohmann-json/include)
# catkin 构建（含 CI 的 x86 工作区）下 claw_client.cpp 直接 include lejusdk-lowlevel/leju_sdk.h，
# 需显式带上 catkin 导出的依赖 include（与同仓库 lejusdk-hw 写法一致）。
if(catkin_FOUND)
  target_include_directories(lejusdk_claw PRIVATE ${catkin_INCLUDE_DIRS})
endif()

# 链接 lejusdk-lowlevel。三种构建方式依次覆盖：
#   1) 与本包同工程（根 CMakeLists 的 add_subdirectory）→ 直接链 target；
#   2) catkin 工作区里该包已构建完成 → 用工作区 devel/lib 下的绝对路径；
#   3) 兜底：按库名链接（此时 catkin 保证 lejusdk-lowlevel 已先构建）。
# 裸名 -llejusdk-lowlevel 缺 -L devel/lib 正是 CI job 712287 的失败原因。
set(_leju_workspace_devel "${CMAKE_CURRENT_SOURCE_DIR}/../../devel")
set(_lowlevel_lib_names
  "${CMAKE_SHARED_LIBRARY_PREFIX}lejusdk-lowlevel${CMAKE_SHARED_LIBRARY_SUFFIX}"
  "${CMAKE_STATIC_LIBRARY_PREFIX}lejusdk-lowlevel${CMAKE_STATIC_LIBRARY_SUFFIX}")
set(_lowlevel_lib "")
foreach(_dir "${CATKIN_DEVEL_PREFIX}/lib" "${_leju_workspace_devel}/lib" "${CATKIN_DEVEL_PREFIX}" "${_leju_workspace_devel}")
  foreach(_name ${_lowlevel_lib_names})
    if(_lowlevel_lib STREQUAL "" AND EXISTS "${_dir}/${_name}")
      set(_lowlevel_lib "${_dir}/${_name}")
    endif()
  endforeach()
endforeach()

if(TARGET lejusdk-lowlevel)
  target_link_libraries(lejusdk_claw PRIVATE lejusdk-lowlevel pthread)
elseif(NOT _lowlevel_lib STREQUAL "")
  target_link_libraries(lejusdk_claw PRIVATE "${_lowlevel_lib}" pthread)
else()
  # 工作区里还没有 lejusdk-lowlevel 产物：catkin 保证依赖先构建，稍后按库名解析。
  target_link_libraries(lejusdk_claw PRIVATE lejusdk-lowlevel pthread)
endif()

install(TARGETS lejusdk_claw LIBRARY DESTINATION lib)
install(DIRECTORY python/lejusdk_claw DESTINATION share/lejusdk-claw-python/python
  PATTERN "__pycache__" EXCLUDE)
