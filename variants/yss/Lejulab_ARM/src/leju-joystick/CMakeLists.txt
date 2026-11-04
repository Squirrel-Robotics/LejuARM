cmake_minimum_required(VERSION 3.0.2)
project(leju-joystick)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils lejusdk-vr)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package()
endif()

# ── SDL3 ──
include(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/arch_select.cmake)
find_package(SDL3 REQUIRED HINTS ${SDL_3RD_PARTY_DIR}/SDL)

include_directories(${SDL_3RD_PARTY_DIR}/SDL/include/SDL3)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# ── Targets ──
add_executable(gamepad-polling src/gamepad-polling.cpp)
target_link_libraries(gamepad-polling SDL3::SDL3)

add_executable(${PROJECT_NAME} src/leju_joystick.cpp)
target_link_libraries(${PROJECT_NAME} SDL3::SDL3)
if(_USING_CATKIN)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} lejusdk-lowlevel lejusdk-utils lejusdk-vr)
endif()

add_executable(leju_joy_autostart_manager src/leju_joy_autostart_manager.cpp)
if(_USING_CATKIN)
  target_link_libraries(leju_joy_autostart_manager ${catkin_LIBRARIES})
else()
  target_link_libraries(leju_joy_autostart_manager lejusdk-lowlevel lejusdk-utils lejusdk-vr)
endif()

# ── Install ──
if(_USING_CATKIN)
  install(TARGETS ${PROJECT_NAME} leju_joy_autostart_manager
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES ${SDL_3RD_PARTY_DIR}/SDL/lib/libSDL3.so.0.3.3
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
  install(CODE "execute_process(COMMAND ln -sf libSDL3.so.0.3.3 libSDL3.so
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})")
  install(CODE "execute_process(COMMAND ln -sf libSDL3.so.0.3.3 libSDL3.so.0
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})")
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  install(DIRECTORY doc/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc)
  install(PROGRAMS
    services/monitor_lejulab_joy.sh
    services/deploy_joy_autostart.sh
    services/set_active_profile.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/services
  )
  install(FILES
    services/lejulab_joy_monitor.service.template
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/services
  )
else()
  install(TARGETS ${PROJECT_NAME} leju_joy_autostart_manager
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  # 纯 CMake 分支同样安装 SDL3（对齐 catkin 分支）：aarch64 发布走纯 CMake，
  # 缺它则预编译 leju-joystick 运行时找不到 libSDL3.so.0。
  # 用 glob 匹配实际版本（aarch64=libSDL3.so.0.3.2, x86_64=libSDL3.so.0.3.3），
  # 避免硬编码版本号导致 install 找不到文件。
  file(GLOB _sdl3_ver "${SDL_3RD_PARTY_DIR}/SDL/lib/libSDL3.so.0.*")
  get_filename_component(_sdl3_name "${_sdl3_ver}" NAME)
  install(FILES ${_sdl3_ver}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
  install(CODE "execute_process(COMMAND ln -sf \"${_sdl3_name}\" libSDL3.so
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})")
  install(CODE "execute_process(COMMAND ln -sf \"${_sdl3_name}\" libSDL3.so.0
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})")
endif()
