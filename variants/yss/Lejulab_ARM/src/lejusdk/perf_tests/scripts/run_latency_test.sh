#!/bin/bash
#
# CycloneDDS 延迟性能测试脚本
#
# 用法:
#   ./run_latency_test.sh [mode] [options]
#
# 模式:
#   latency     - 运行发布者/订阅者延迟测试 (默认)
#   roundtrip   - 运行单进程往返测试
#   multi_topic - 运行多 topic 测试 (JointState + ImuData)
#   all         - 运行所有测试
#
# 示例:
#   ./run_latency_test.sh latency --rate 1000 --size 512 --count 100000
#   ./run_latency_test.sh roundtrip --rate 500 --size 1024 --count 100000
#   ./run_latency_test.sh multi_topic --rate 1000 --count 100000
#   ./run_latency_test.sh all

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts -> perf_tests -> lejusdk -> src -> workspace_root
WORKSPACE_ROOT="${SCRIPT_DIR}/../../../.."

# Default settings
MODE="latency"
TOPIC="/perf/test"
RATE=1000
SIZE=64
COUNT=100000
WARMUP=100
USE_SHM=0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        latency|roundtrip|multi_topic|all)
            MODE="$1"
            shift
            ;;
        --topic|-t)
            TOPIC="$2"
            shift 2
            ;;
        --rate|-r)
            RATE="$2"
            shift 2
            ;;
        --size|-s)
            SIZE="$2"
            shift 2
            ;;
        --count|-c)
            COUNT="$2"
            shift 2
            ;;
        --warmup|-w)
            WARMUP="$2"
            shift 2
            ;;
        --shm)
            USE_SHM=1
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [mode] [options]"
            echo ""
            echo "Modes:"
            echo "  latency     - Run publisher/subscriber latency test (default)"
            echo "  roundtrip   - Run single-process round-trip test"
            echo "  multi_topic - Run multi-topic test (JointState + ImuData)"
            echo "  all         - Run all tests"
            echo ""
            echo "Options:"
            echo "  -t, --topic <name>   Topic name (default: /perf/test)"
            echo "  -r, --rate <hz>      Publish rate in Hz (default: 1000)"
            echo "  -s, --size <bytes>   Message payload size (default: 64)"
            echo "  -c, --count <n>      Number of messages (default: 100000)"
            echo "  -w, --warmup <n>     Warmup messages (default: 100)"
            echo "  --shm                Use shared memory transport (iceoryx)"
            echo "  -h, --help           Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Source ROS environment
if [ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/devel/setup.bash"
else
    echo "Warning: Could not find devel/setup.bash"
fi

# Set CycloneDDS configuration
CONFIG_DIR="${WORKSPACE_ROOT}/src/leju_launch/config"
if [ "$USE_SHM" -eq 1 ]; then
    export CYCLONEDDS_URI="file://${CONFIG_DIR}/cyclonedds_shm.xml"
    echo "Using shared memory transport (iceoryx)"
else
    export CYCLONEDDS_URI="file://${CONFIG_DIR}/cyclonedds.xml"
    echo "Using network transport"
fi

# Find executables
BIN_DIR="${WORKSPACE_ROOT}/devel/lib/lejusdk-lowlevel"
PUBLISHER="${BIN_DIR}/latency_publisher"
SUBSCRIBER="${BIN_DIR}/latency_subscriber"
ROUNDTRIP="${BIN_DIR}/roundtrip_test"
MULTI_PUB="${BIN_DIR}/multi_topic_publisher"
MULTI_SUB="${BIN_DIR}/multi_topic_subscriber"

# Check executables exist
check_executable() {
    if [ ! -x "$1" ]; then
        echo "Error: $1 not found or not executable"
        echo "Please build the project first: catkin build lejusdk-lowlevel"
        exit 1
    fi
}

echo ""
echo "============================================"
echo "CycloneDDS Latency Performance Test"
echo "============================================"
echo "Mode:    ${MODE}"
echo "Topic:   ${TOPIC}"
echo "Rate:    ${RATE} Hz"
echo "Size:    ${SIZE} bytes"
echo "Count:   ${COUNT}"
echo "Warmup:  ${WARMUP}"
echo "============================================"
echo ""

run_latency_test() {
    check_executable "$PUBLISHER"
    check_executable "$SUBSCRIBER"

    echo "Starting subscriber in background..."
    "$SUBSCRIBER" --topic "$TOPIC" --rate "$RATE" --count "$COUNT" --warmup "$WARMUP" &
    SUB_PID=$!

    # Wait for subscriber to start
    sleep 1

    echo "Starting publisher..."
    "$PUBLISHER" --topic "$TOPIC" --rate "$RATE" --size "$SIZE" --count "$COUNT" --warmup "$WARMUP"

    # Wait for subscriber to finish
    wait $SUB_PID 2>/dev/null || true
}

run_roundtrip_test() {
    check_executable "$ROUNDTRIP"

    echo "Running round-trip test..."
    "$ROUNDTRIP" --topic "$TOPIC" --rate "$RATE" --size "$SIZE" --count "$COUNT" --warmup "$WARMUP"
}

run_multi_topic_test() {
    check_executable "$MULTI_PUB"
    check_executable "$MULTI_SUB"

    echo "Starting multi-topic subscriber in background..."
    "$MULTI_SUB" --rate "$RATE" --count "$COUNT" --warmup "$WARMUP" &
    SUB_PID=$!

    # Wait for subscriber to start
    sleep 1

    echo "Starting multi-topic publisher..."
    "$MULTI_PUB" --rate "$RATE" --count "$COUNT" --warmup "$WARMUP"

    # Wait for subscriber to finish
    wait $SUB_PID 2>/dev/null || true
}

case $MODE in
    latency)
        run_latency_test
        ;;
    roundtrip)
        run_roundtrip_test
        ;;
    multi_topic)
        run_multi_topic_test
        ;;
    all)
        echo "=== Latency Test ==="
        run_latency_test
        echo ""
        echo "=== Round-Trip Test ==="
        run_roundtrip_test
        echo ""
        echo "=== Multi-Topic Test ==="
        run_multi_topic_test
        ;;
esac

echo ""
echo "Test complete."
