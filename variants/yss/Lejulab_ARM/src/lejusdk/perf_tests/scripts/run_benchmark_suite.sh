#!/bin/bash
#
# CycloneDDS 基准测试套件脚本
#
# 在多种参数组合下运行自动化性能测试，并生成综合报告。
#
# 用法:
#   ./run_benchmark_suite.sh [options]
#
# 示例:
#   ./run_benchmark_suite.sh --dry-run
#   ./run_benchmark_suite.sh --rates "500,1000" --sizes "64,256" --count 100000
#   ./run_benchmark_suite.sh --test-modes "latency,roundtrip" --shm
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts -> perf_tests -> lejusdk -> src -> workspace_root
WORKSPACE_ROOT="${SCRIPT_DIR}/../../../.."

# Default settings
OUTPUT_DIR="${SCRIPT_DIR}/../dds_perf_results"
TEST_MODES="latency,roundtrip,multi_topic"
RATES="100,500,1000,2000"
SIZES="64,256,1024,4096"
COUNT=100000
WARMUP=100
USE_SHM=0
DRY_RUN=0
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --test-modes)
            TEST_MODES="$2"
            shift 2
            ;;
        --rates)
            RATES="$2"
            shift 2
            ;;
        --sizes)
            SIZES="$2"
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
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --output-dir <dir>    Output directory (default: ../dds_perf_results)"
            echo "  --test-modes <modes>  Comma-separated test modes (default: latency,roundtrip,multi_topic)"
            echo "  --rates <list>        Comma-separated rates in Hz (default: 100,500,1000,2000)"
            echo "  --sizes <list>        Comma-separated message sizes (default: 64,256,1024,4096)"
            echo "  -c, --count <n>       Number of messages (default: 100000)"
            echo "  -w, --warmup <n>      Warmup messages (default: 100)"
            echo "  --shm                 Use shared memory transport (iceoryx)"
            echo "  --dry-run             Preview test plan without executing"
            echo "  -h, --help            Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Convert comma-separated lists to arrays
IFS=',' read -ra MODE_ARRAY <<< "$TEST_MODES"
IFS=',' read -ra RATE_ARRAY <<< "$RATES"
IFS=',' read -ra SIZE_ARRAY <<< "$SIZES"

# Calculate total tests
TOTAL_TESTS=0
for mode in "${MODE_ARRAY[@]}"; do
    if [[ "$mode" == "multi_topic" ]]; then
        # multi_topic doesn't use size parameter
        TOTAL_TESTS=$((TOTAL_TESTS + ${#RATE_ARRAY[@]}))
    else
        TOTAL_TESTS=$((TOTAL_TESTS + ${#RATE_ARRAY[@]} * ${#SIZE_ARRAY[@]}))
    fi
done

# Print test plan
echo ""
echo "============================================"
echo "CycloneDDS Benchmark Suite"
echo "============================================"
echo "Test Modes:  ${TEST_MODES}"
echo "Rates:       ${RATES} Hz"
echo "Sizes:       ${SIZES} bytes"
echo "Count:       ${COUNT}"
echo "Warmup:      ${WARMUP}"
echo "Transport:   $([ $USE_SHM -eq 1 ] && echo 'Shared Memory' || echo 'Network')"
echo "Output Dir:  ${OUTPUT_DIR}"
echo "Total Tests: ${TOTAL_TESTS}"
echo "============================================"
echo ""

if [ "$DRY_RUN" -eq 1 ]; then
    echo "Test Plan (dry-run mode):"
    echo ""
    test_num=1
    for mode in "${MODE_ARRAY[@]}"; do
        if [[ "$mode" == "multi_topic" ]]; then
            for rate in "${RATE_ARRAY[@]}"; do
                echo "  [$test_num] $mode: rate=${rate}Hz"
                test_num=$((test_num + 1))
            done
        else
            for rate in "${RATE_ARRAY[@]}"; do
                for size in "${SIZE_ARRAY[@]}"; do
                    echo "  [$test_num] $mode: rate=${rate}Hz, size=${size}bytes"
                    test_num=$((test_num + 1))
                done
            done
        fi
    done
    echo ""
    echo "Total: $((test_num - 1)) tests"
    echo ""
    echo "Run without --dry-run to execute tests."
    exit 0
fi

# Create output directory
RESULT_DIR="${OUTPUT_DIR}/benchmark_${TIMESTAMP}"
mkdir -p "$RESULT_DIR"

# Initialize CSV file
CSV_FILE="${RESULT_DIR}/benchmark_results.csv"
echo "mode,rate_hz,size_bytes,min_us,max_us,mean_us,median_us,p95_us,p99_us,stddev_us,jitter_us,transport" > "$CSV_FILE"

# Source ROS environment
if [ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/devel/setup.bash"
else
    echo "Warning: Could not find devel/setup.bash"
fi

# Set transport
SHM_OPT=""
TRANSPORT="network"
if [ "$USE_SHM" -eq 1 ]; then
    SHM_OPT="--shm"
    TRANSPORT="shm"
fi

# Run tests
test_num=1
failed_tests=0

run_single_test() {
    local mode=$1
    local rate=$2
    local size=$3
    local log_file=$4

    echo ""
    echo "Running: $mode (rate=${rate}Hz, size=${size}bytes)..."

    if [[ "$mode" == "multi_topic" ]]; then
        "${SCRIPT_DIR}/run_latency_test.sh" "$mode" \
            --rate "$rate" --count "$COUNT" --warmup "$WARMUP" $SHM_OPT \
            > "$log_file" 2>&1
    else
        "${SCRIPT_DIR}/run_latency_test.sh" "$mode" \
            --rate "$rate" --size "$size" --count "$COUNT" --warmup "$WARMUP" $SHM_OPT \
            > "$log_file" 2>&1
    fi
}

parse_and_append_csv() {
    local log_file=$1
    local mode=$2
    local rate=$3
    local size=$4

    # Parse log file using Python parser
    if [ -x "${SCRIPT_DIR}/parse_results.py" ]; then
        results=$("${SCRIPT_DIR}/parse_results.py" "$log_file" --format json 2>/dev/null || echo "[]")

        # Extract values from JSON (first result)
        if [ "$results" != "[]" ]; then
            min_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('min_us','') if d else '')" 2>/dev/null || echo "")
            max_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('max_us','') if d else '')" 2>/dev/null || echo "")
            mean_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('mean_us','') if d else '')" 2>/dev/null || echo "")
            median_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('median_us','') if d else '')" 2>/dev/null || echo "")
            p95_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('p95_us','') if d else '')" 2>/dev/null || echo "")
            p99_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('p99_us','') if d else '')" 2>/dev/null || echo "")
            stddev_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('stddev_us','') if d else '')" 2>/dev/null || echo "")
            jitter_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('jitter_us','') if d else '')" 2>/dev/null || echo "")

            echo "$mode,$rate,$size,$min_us,$max_us,$mean_us,$median_us,$p95_us,$p99_us,$stddev_us,$jitter_us,$TRANSPORT" >> "$CSV_FILE"
        fi
    fi
}

for mode in "${MODE_ARRAY[@]}"; do
    if [[ "$mode" == "multi_topic" ]]; then
        for rate in "${RATE_ARRAY[@]}"; do
            echo ""
            echo "[Test $test_num/$TOTAL_TESTS] $mode: rate=${rate}Hz"

            log_file="${RESULT_DIR}/${mode}_r${rate}.log"
            size=0  # multi_topic doesn't use size

            if run_single_test "$mode" "$rate" "$size" "$log_file"; then
                parse_and_append_csv "$log_file" "$mode" "$rate" "$size"
                echo "  -> Completed. Log: $log_file"
            else
                echo "  -> FAILED!"
                failed_tests=$((failed_tests + 1))
            fi

            test_num=$((test_num + 1))
        done
    else
        for rate in "${RATE_ARRAY[@]}"; do
            for size in "${SIZE_ARRAY[@]}"; do
                echo ""
                echo "[Test $test_num/$TOTAL_TESTS] $mode: rate=${rate}Hz, size=${size}bytes"

                log_file="${RESULT_DIR}/${mode}_r${rate}_s${size}.log"

                if run_single_test "$mode" "$rate" "$size" "$log_file"; then
                    parse_and_append_csv "$log_file" "$mode" "$rate" "$size"
                    echo "  -> Completed. Log: $log_file"
                else
                    echo "  -> FAILED!"
                    failed_tests=$((failed_tests + 1))
                fi

                test_num=$((test_num + 1))
            done
        done
    fi
done

# Generate Markdown report
REPORT_FILE="${RESULT_DIR}/REPORT.md"
cat > "$REPORT_FILE" << EOF
# CycloneDDS Benchmark Report

**Date:** $(date '+%Y-%m-%d %H:%M:%S')
**Transport:** $([ $USE_SHM -eq 1 ] && echo 'Shared Memory (iceoryx)' || echo 'Network (UDP)')

## Test Parameters

| Parameter | Value |
|-----------|-------|
| Test Modes | ${TEST_MODES} |
| Rates | ${RATES} Hz |
| Message Sizes | ${SIZES} bytes |
| Message Count | ${COUNT} |
| Warmup Count | ${WARMUP} |

## Results Summary

EOF

# Append CSV as markdown table
if [ -f "$CSV_FILE" ]; then
    echo "| Mode | Rate (Hz) | Size (bytes) | Mean (us) | P95 (us) | P99 (us) | Max (us) |" >> "$REPORT_FILE"
    echo "|------|-----------|--------------|-----------|----------|----------|----------|" >> "$REPORT_FILE"

    # Skip header line and format as markdown
    tail -n +2 "$CSV_FILE" | while IFS=',' read -r mode rate size min max mean median p95 p99 stddev jitter transport; do
        if [ -n "$mean" ]; then
            echo "| $mode | $rate | $size | $mean | $p95 | $p99 | $max |" >> "$REPORT_FILE"
        fi
    done
fi

cat >> "$REPORT_FILE" << EOF

## Test Status

- **Total Tests:** ${TOTAL_TESTS}
- **Passed:** $((TOTAL_TESTS - failed_tests))
- **Failed:** ${failed_tests}

## Files

- Raw logs: \`${RESULT_DIR}/*.log\`
- CSV data: \`benchmark_results.csv\`
EOF

echo ""
echo "============================================"
echo "Benchmark Suite Complete"
echo "============================================"
echo "Total Tests: ${TOTAL_TESTS}"
echo "Passed:      $((TOTAL_TESTS - failed_tests))"
echo "Failed:      ${failed_tests}"
echo ""
echo "Results:"
echo "  Directory:   ${RESULT_DIR}"
echo "  CSV:         ${CSV_FILE}"
echo "  Report:      ${REPORT_FILE}"
echo "============================================"
