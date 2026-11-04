cmake_minimum_required(VERSION 3.16)

# This package is added by the Lejulab_ARM root CMakeLists after
# lejusdk-stubs.cmake defines the imported SDK targets.
if(NOT TARGET lejusdk-lowlevel OR NOT TARGET lejusdk-utils)
  message(FATAL_ERROR
    "Configure from the Lejulab_ARM root after loading lejusdk-stubs.cmake")
endif()

add_executable(revo2_hand_terminal revo2_hand_terminal.cpp)
target_compile_features(revo2_hand_terminal PRIVATE cxx_std_17)
target_link_libraries(revo2_hand_terminal PRIVATE lejusdk-lowlevel lejusdk-utils)
