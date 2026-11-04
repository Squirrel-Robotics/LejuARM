cmake_minimum_required(VERSION 3.16)
project(leju-tools)

add_executable(joy_trigger joy_trigger.cpp)
target_link_libraries(joy_trigger lejusdk-lowlevel lejusdk-utils)
target_include_directories(joy_trigger PRIVATE
    ${CMAKE_SOURCE_DIR}/src/lejusdk/lejusdk-lowlevel/include
    ${CMAKE_SOURCE_DIR}/src/lejusdk/lejusdk-utils/include)
