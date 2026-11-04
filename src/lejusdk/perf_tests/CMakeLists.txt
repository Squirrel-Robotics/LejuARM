cmake_minimum_required(VERSION 3.16)
project(lejusdk-perf-tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 catkin 依赖
find_package(catkin REQUIRED COMPONENTS lejusdk-dds-idl lejusdk-topic-pubsub)
find_package(Threads REQUIRED)

# iceoryx + CycloneDDS
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

# catkin package 配置
catkin_package(
    CATKIN_DEPENDS lejusdk-dds-idl lejusdk-topic-pubsub
)

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Latency Publisher
add_executable(latency_publisher src/latency_publisher.cpp)
target_link_libraries(latency_publisher
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Latency Subscriber
add_executable(latency_subscriber src/latency_subscriber.cpp)
target_link_libraries(latency_subscriber
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Roundtrip Test (single process ping-pong)
add_executable(roundtrip_test src/roundtrip_test.cpp)
target_link_libraries(roundtrip_test
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Multi-topic Publisher (cross-process)
add_executable(multi_topic_publisher src/multi_topic_publisher.cpp)
target_link_libraries(multi_topic_publisher
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Multi-topic Subscriber (cross-process)
add_executable(multi_topic_subscriber src/multi_topic_subscriber.cpp)
target_link_libraries(multi_topic_subscriber
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Multi-topic Test (single process)
add_executable(multi_topic_test src/multi_topic_test.cpp)
target_link_libraries(multi_topic_test
    ${catkin_LIBRARIES}
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

