#!/bin/bash
#
# CycloneDDS 传输方式对比测试脚本
#
# 比较网络传输和共享内存 (iceoryx) 传输的性能差异。
# 使用两种传输方式运行相同的测试，并计算性能提升百分比。
#
# 用法:
#   ./run_comparison_test.sh [options]
#
# 示例:
#   ./run_comparison_test.sh --mode latency --rate 1000 --count 100000
#   ./run_comparison_test.sh --mode roundtrip --rate 2000 --size 1024 --count 100000
#   ./run_comparison_test.sh --mode multi_topic --rate 1000 --count 100000
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts -> perf_tests -> lejusdk -> src -> workspace_root
WORKSPACE_ROOT="${SCRIPT_DIR}/../../../.."

# Default settings
MODE="latency"
RATE=1000
SIZE=64
COUNT=100000
WARMUP=100
ROUDI_TIMEOUT=5
AUTO_START_ROUDI=1
OUTPUT_DIR="${SCRIPT_DIR}/../dds_perf_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode|-m)
            MODE="$2"
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
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --no-auto-roudi)
            AUTO_START_ROUDI=0
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -m, --mode <mode>      Test mode: latency, roundtrip, multi_topic (default: latency)"
            echo "  -r, --rate <hz>        Publish rate in Hz (default: 1000)"
            echo "  -s, --size <bytes>     Message payload size (default: 64)"
            echo "  -c, --count <n>        Number of messages (default: 100000)"
            echo "  -w, --warmup <n>       Warmup messages (default: 100)"
            echo "  --output-dir <dir>     Output directory (default: ../dds_perf_results)"
            echo "  --no-auto-roudi        Don't auto-start RouDi daemon"
            echo "  -h, --help             Show this help"
            echo ""
            echo "Prerequisites:"
            echo "  - For SHM tests, RouDi (iceoryx daemon) must be running"
            echo "  - If RouDi is not running, this script will attempt to start it"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Check if RouDi is running
check_roudi() {
    if pgrep -x "iox-roudi" > /dev/null 2>&1 || pgrep -x "RouDi" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Start RouDi daemon
start_roudi() {
    echo "Starting RouDi daemon..."

    # Try different possible RouDi locations
    local roudi_cmd=""
    if command -v iox-roudi &> /dev/null; then
        roudi_cmd="iox-roudi"
    elif [ -x "/usr/local/bin/iox-roudi" ]; then
        roudi_cmd="/usr/local/bin/iox-roudi"
    elif [ -x "${WORKSPACE_ROOT}/devel/lib/iceoryx_posh/iox-roudi" ]; then
        roudi_cmd="${WORKSPACE_ROOT}/devel/lib/iceoryx_posh/iox-roudi"
    fi

    if [ -z "$roudi_cmd" ]; then
        echo "Error: Could not find RouDi executable (iox-roudi)"
        echo "Please install iceoryx or ensure it's in your PATH"
        return 1
    fi

    # Start RouDi in background
    $roudi_cmd &
    ROUDI_PID=$!
    echo "Started RouDi with PID: $ROUDI_PID"

    # Wait for RouDi to be ready
    local waited=0
    while ! check_roudi && [ $waited -lt $ROUDI_TIMEOUT ]; do
        sleep 1
        waited=$((waited + 1))
    done

    if ! check_roudi; then
        echo "Error: RouDi failed to start within ${ROUDI_TIMEOUT} seconds"
        return 1
    fi

    echo "RouDi is ready"
    return 0
}

# Stop RouDi daemon (only if we started it)
stop_roudi() {
    if [ -n "$ROUDI_PID" ]; then
        echo "Stopping RouDi (PID: $ROUDI_PID)..."
        kill $ROUDI_PID 2>/dev/null || true
        wait $ROUDI_PID 2>/dev/null || true
        ROUDI_PID=""
    fi
}

# Cleanup on exit
cleanup() {
    stop_roudi
}
trap cleanup EXIT

# Extract mean latency from log file
extract_mean_us() {
    local log_file=$1
    if [ -x "${SCRIPT_DIR}/parse_results.py" ]; then
        "${SCRIPT_DIR}/parse_results.py" "$log_file" --format json 2>/dev/null | \
            python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('mean_us', 0) if d else 0)" 2>/dev/null || echo "0"
    else
        # Fallback: grep for Mean value
        grep -oP 'Mean:\s+\K[\d.]+' "$log_file" 2>/dev/null | head -1 || echo "0"
    fi
}

# Extract P99 latency from log file
extract_p99_us() {
    local log_file=$1
    if [ -x "${SCRIPT_DIR}/parse_results.py" ]; then
        "${SCRIPT_DIR}/parse_results.py" "$log_file" --format json 2>/dev/null | \
            python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('p99_us', 0) if d else 0)" 2>/dev/null || echo "0"
    else
        grep -oP 'P99:\s+\K[\d.]+' "$log_file" 2>/dev/null | head -1 || echo "0"
    fi
}

echo ""
echo "============================================"
echo "CycloneDDS Transport Comparison Test"
echo "============================================"
echo "Mode:    ${MODE}"
echo "Rate:    ${RATE} Hz"
echo "Size:    ${SIZE} bytes"
echo "Count:   ${COUNT}"
echo "Warmup:  ${WARMUP}"
echo "============================================"
echo ""

# Create output directory
RESULT_DIR="${OUTPUT_DIR}/comparison_${TIMESTAMP}"
mkdir -p "$RESULT_DIR"

NETWORK_LOG="${RESULT_DIR}/network_${MODE}.log"
SHM_LOG="${RESULT_DIR}/shm_${MODE}.log"

# Run Network Transport Test
echo "=== Phase 1: Network Transport Test ==="
echo ""

"${SCRIPT_DIR}/run_latency_test.sh" "$MODE" \
    --rate "$RATE" --size "$SIZE" --count "$COUNT" --warmup "$WARMUP" \
    > "$NETWORK_LOG" 2>&1

NETWORK_MEAN=$(extract_mean_us "$NETWORK_LOG")
NETWORK_P99=$(extract_p99_us "$NETWORK_LOG")

echo "Network Results:"
echo "  Mean Latency: ${NETWORK_MEAN} us"
echo "  P99 Latency:  ${NETWORK_P99} us"
echo "  Log: ${NETWORK_LOG}"
echo ""

# Check/Start RouDi for SHM test
echo "=== Phase 2: Shared Memory Transport Test ==="
echo ""

ROUDI_PID=""
ROUDI_WAS_RUNNING=0

if check_roudi; then
    echo "RouDi is already running"
    ROUDI_WAS_RUNNING=1
elif [ "$AUTO_START_ROUDI" -eq 1 ]; then
    if ! start_roudi; then
        echo "Error: Failed to start RouDi. SHM test skipped."
        echo ""
        echo "To run SHM tests, please start RouDi manually:"
        echo "  iox-roudi &"
        exit 1
    fi
else
    echo "Error: RouDi is not running and auto-start is disabled"
    echo "Please start RouDi manually: iox-roudi &"
    exit 1
fi

# Run SHM Transport Test
"${SCRIPT_DIR}/run_latency_test.sh" "$MODE" \
    --rate "$RATE" --size "$SIZE" --count "$COUNT" --warmup "$WARMUP" --shm \
    > "$SHM_LOG" 2>&1

SHM_MEAN=$(extract_mean_us "$SHM_LOG")
SHM_P99=$(extract_p99_us "$SHM_LOG")

echo "Shared Memory Results:"
echo "  Mean Latency: ${SHM_MEAN} us"
echo "  P99 Latency:  ${SHM_P99} us"
echo "  Log: ${SHM_LOG}"
echo ""

# Calculate improvement percentages
calc_improvement() {
    local network=$1
    local shm=$2
    if [ "$network" = "0" ] || [ -z "$network" ]; then
        echo "N/A"
        return
    fi
    python3 -c "
network = float('$network')
shm = float('$shm')
if network > 0 and shm > 0:
    improvement = (network - shm) / network * 100
    print(f'{improvement:.1f}%')
else:
    print('N/A')
" 2>/dev/null || echo "N/A"
}

MEAN_IMPROVEMENT=$(calc_improvement "$NETWORK_MEAN" "$SHM_MEAN")
P99_IMPROVEMENT=$(calc_improvement "$NETWORK_P99" "$SHM_P99")

# Generate comparison report
REPORT_FILE="${RESULT_DIR}/comparison_report.md"
cat > "$REPORT_FILE" << EOF
# CycloneDDS Transport Comparison Report

**Date:** $(date '+%Y-%m-%d %H:%M:%S')

## Test Configuration

| Parameter | Value |
|-----------|-------|
| Mode | ${MODE} |
| Rate | ${RATE} Hz |
| Message Size | ${SIZE} bytes |
| Message Count | ${COUNT} |
| Warmup Count | ${WARMUP} |

## Results Comparison

| Metric | Network (UDP) | Shared Memory | Improvement |
|--------|---------------|---------------|-------------|
| Mean Latency (us) | ${NETWORK_MEAN} | ${SHM_MEAN} | ${MEAN_IMPROVEMENT} |
| P99 Latency (us) | ${NETWORK_P99} | ${SHM_P99} | ${P99_IMPROVEMENT} |

## Analysis

EOF

# Add analysis based on results
if [ "$MEAN_IMPROVEMENT" != "N/A" ]; then
    cat >> "$REPORT_FILE" << EOF
### Performance Improvement

Shared memory transport provides significant latency reduction compared to network transport:

- **Mean Latency Reduction:** ${MEAN_IMPROVEMENT}
- **P99 Latency Reduction:** ${P99_IMPROVEMENT}

The improvement is achieved by eliminating:
- Network stack overhead (UDP/IP processing)
- Kernel-user space context switches
- Data serialization/deserialization overhead

### Recommendations

EOF

    # Parse improvement percentage for recommendations
    improvement_val=$(echo "$MEAN_IMPROVEMENT" | tr -d '%' | cut -d'.' -f1)
    if [ "$improvement_val" -gt 50 ] 2>/dev/null; then
        echo "- **Strong recommendation** to use shared memory for this workload" >> "$REPORT_FILE"
    elif [ "$improvement_val" -gt 20 ] 2>/dev/null; then
        echo "- Consider using shared memory for latency-sensitive applications" >> "$REPORT_FILE"
    else
        echo "- Improvement is modest; network transport may be sufficient for most use cases" >> "$REPORT_FILE"
    fi
fi

cat >> "$REPORT_FILE" << EOF

## Raw Logs

- Network: \`$(basename "$NETWORK_LOG")\`
- Shared Memory: \`$(basename "$SHM_LOG")\`
EOF

# Print summary
echo "============================================"
echo "Comparison Results"
echo "============================================"
echo ""
echo "                    Network    SHM    Improvement"
echo "  Mean Latency:     ${NETWORK_MEAN} us    ${SHM_MEAN} us    ${MEAN_IMPROVEMENT}"
echo "  P99 Latency:      ${NETWORK_P99} us    ${SHM_P99} us    ${P99_IMPROVEMENT}"
echo ""
echo "============================================"
echo ""
echo "Output Files:"
echo "  Report:       ${REPORT_FILE}"
echo "  Network Log:  ${NETWORK_LOG}"
echo "  SHM Log:      ${SHM_LOG}"
echo ""
echo "Comparison test complete."
