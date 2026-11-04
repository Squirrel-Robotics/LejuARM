#!/bin/bash
#
# CycloneDDS 性能测试 + 图表生成脚本
#
# 一键运行性能测试并生成 PNG 图表。
#
# 用法:
#   ./run_and_plot.sh [options]
#
# 示例:
#   ./run_and_plot.sh                                        # 运行默认测试
#   ./run_and_plot.sh --quick                                # 快速测试
#   ./run_and_plot.sh --rates "500,1000,2000" --count 100000
#   ./run_and_plot.sh --mode latency                         # 延迟测试模式
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}/../../../.."

# Default settings
OUTPUT_DIR="${SCRIPT_DIR}/../dds_perf_results"
RATES="500,1000,2000"
SIZES="64,256,1024"
COUNT=100000
WARMUP=1000
QUICK_MODE=0
TEST_MODE="roundtrip"  # roundtrip, latency, multi_topic
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --output-dir|-o)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --rates|-r)
            RATES="$2"
            shift 2
            ;;
        --sizes|-s)
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
        --mode|-m)
            TEST_MODE="$2"
            shift 2
            ;;
        --quick)
            QUICK_MODE=1
            RATES="500,1000"
            SIZES="64,256"
            COUNT=50000
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -o, --output-dir <dir>  Output directory (default: ../dds_perf_results)"
            echo "  -r, --rates <list>      Comma-separated rates in Hz (default: 500,1000,2000)"
            echo "  -s, --sizes <list>      Comma-separated message sizes (default: 64,256,1024)"
            echo "  -c, --count <n>         Number of messages (default: 100000)"
            echo "  -w, --warmup <n>        Warmup messages (default: 1000)"
            echo "  -m, --mode <mode>       Test mode: roundtrip, latency, multi_topic (default: roundtrip)"
            echo "  --quick                 Quick test with fewer parameters"
            echo "  -h, --help              Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Create output directory
RESULT_DIR="${OUTPUT_DIR}/perf_${TIMESTAMP}"
mkdir -p "$RESULT_DIR"

echo ""
echo "============================================"
echo "CycloneDDS Performance Test + Plot"
echo "============================================"
echo "Mode:       ${TEST_MODE}"
echo "Rates:      ${RATES} Hz"
echo "Sizes:      ${SIZES} bytes"
echo "Count:      ${COUNT}"
echo "Warmup:     ${WARMUP}"
echo "Output:     ${RESULT_DIR}"
echo "============================================"
echo ""

# Source ROS environment
if [ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/devel/setup.bash"
else
    echo "Warning: Could not find devel/setup.bash"
fi

# Convert comma-separated lists to arrays
IFS=',' read -ra RATE_ARRAY <<< "$RATES"
IFS=',' read -ra SIZE_ARRAY <<< "$SIZES"

# CSV file for results
CSV_FILE="${RESULT_DIR}/results.csv"
echo "mode,rate_hz,size_bytes,min_us,max_us,mean_us,median_us,p95_us,p99_us,stddev_us,jitter_us" > "$CSV_FILE"

# Function to run a single test and parse results
run_test() {
    local mode=$1
    local rate=$2
    local size=$3
    local log_file=$4

    echo "  Running: ${mode} rate=${rate}Hz size=${size}bytes..."

    "${SCRIPT_DIR}/run_latency_test.sh" "$mode" \
        --rate "$rate" --size "$size" --count "$COUNT" --warmup "$WARMUP" \
        > "$log_file" 2>&1 || true

    # Parse results and append to CSV
    if [ -f "$log_file" ]; then
        local results=$("${SCRIPT_DIR}/parse_results.py" "$log_file" --format json 2>/dev/null || echo "[]")
        if [ "$results" != "[]" ]; then
            local min_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('min_us','') if d else '')" 2>/dev/null || echo "")
            local max_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('max_us','') if d else '')" 2>/dev/null || echo "")
            local mean_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('mean_us','') if d else '')" 2>/dev/null || echo "")
            local median_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('median_us','') if d else '')" 2>/dev/null || echo "")
            local p95_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('p95_us','') if d else '')" 2>/dev/null || echo "")
            local p99_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('p99_us','') if d else '')" 2>/dev/null || echo "")
            local stddev_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('stddev_us','') if d else '')" 2>/dev/null || echo "")
            local jitter_us=$(echo "$results" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d[0].get('jitter_us','') if d else '')" 2>/dev/null || echo "")

            if [ -n "$mean_us" ]; then
                echo "${mode},${rate},${size},${min_us},${max_us},${mean_us},${median_us},${p95_us},${p99_us},${stddev_us},${jitter_us}" >> "$CSV_FILE"
                echo "    Mean: ${mean_us} us, P99: ${p99_us} us"
            fi
        fi
    fi
}

# Run tests
echo "Running performance tests..."
echo ""

for rate in "${RATE_ARRAY[@]}"; do
    for size in "${SIZE_ARRAY[@]}"; do
        log_file="${RESULT_DIR}/${TEST_MODE}_r${rate}_s${size}.log"
        run_test "$TEST_MODE" "$rate" "$size" "$log_file"
    done
done

echo ""
echo "Tests complete. Generating plots..."
echo ""

# Generate plots using Python
python3 << PYTHON_SCRIPT
#!/usr/bin/env python3
"""Generate performance charts from CSV data."""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

csv_file = "${CSV_FILE}"
output_dir = "${RESULT_DIR}"

# Read CSV
df = pd.read_csv(csv_file)

if df.empty:
    print("No data to plot")
    exit(1)

# Set style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'ggplot')

rates = sorted(df['rate_hz'].unique())
sizes = sorted(df['size_bytes'].unique())

# ========================================
# Chart 1: Mean Latency vs Rate (by size)
# ========================================
fig, ax = plt.subplots(figsize=(10, 6))

markers = ['o', 's', '^', 'D', 'v']
colors = ['#3498db', '#e74c3c', '#2ecc71', '#9b59b6', '#f39c12']

for i, size in enumerate(sizes):
    data = df[df['size_bytes'] == size].sort_values('rate_hz')
    if not data.empty:
        ax.plot(data['rate_hz'], data['mean_us'],
               label=f'{size} bytes',
               marker=markers[i % len(markers)],
               color=colors[i % len(colors)],
               markersize=8, linewidth=2)

ax.set_xlabel('Publish Rate (Hz)', fontsize=12)
ax.set_ylabel('Mean Latency (μs)', fontsize=12)
ax.set_title('CycloneDDS Latency vs Publish Rate', fontsize=14, fontweight='bold')
ax.legend(title='Message Size', loc='best')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f"{output_dir}/latency_vs_rate.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  Generated: latency_vs_rate.png")

# ========================================
# Chart 2: Mean Latency vs Size (by rate)
# ========================================
fig, ax = plt.subplots(figsize=(10, 6))

for i, rate in enumerate(rates):
    data = df[df['rate_hz'] == rate].sort_values('size_bytes')
    if not data.empty:
        ax.plot(data['size_bytes'], data['mean_us'],
               label=f'{rate} Hz',
               marker=markers[i % len(markers)],
               color=colors[i % len(colors)],
               markersize=8, linewidth=2)

ax.set_xlabel('Message Size (bytes)', fontsize=12)
ax.set_ylabel('Mean Latency (μs)', fontsize=12)
ax.set_title('CycloneDDS Latency vs Message Size', fontsize=14, fontweight='bold')
ax.legend(title='Publish Rate', loc='best')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f"{output_dir}/latency_vs_size.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  Generated: latency_vs_size.png")

# ========================================
# Chart 3: Latency Distribution (Bar chart)
# ========================================
fig, ax = plt.subplots(figsize=(12, 6))

# Use highest rate for comparison
target_rate = max(rates)

x_labels = []
mean_vals = []
p95_vals = []
p99_vals = []

for size in sizes:
    data = df[(df['rate_hz'] == target_rate) & (df['size_bytes'] == size)]
    if not data.empty:
        x_labels.append(f'{size}B')
        mean_vals.append(data['mean_us'].values[0])
        p95_vals.append(data['p95_us'].values[0])
        p99_vals.append(data['p99_us'].values[0])

if x_labels:
    x = range(len(x_labels))
    width = 0.25

    bars1 = ax.bar([i - width for i in x], mean_vals, width, label='Mean', color='#3498db', alpha=0.8)
    bars2 = ax.bar(x, p95_vals, width, label='P95', color='#f39c12', alpha=0.8)
    bars3 = ax.bar([i + width for i in x], p99_vals, width, label='P99', color='#e74c3c', alpha=0.8)

    ax.set_xlabel('Message Size', fontsize=12)
    ax.set_ylabel('Latency (μs)', fontsize=12)
    ax.set_title(f'Latency Distribution @ {target_rate} Hz', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Add value labels
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.1f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3), textcoords="offset points",
                       ha='center', va='bottom', fontsize=9)

plt.tight_layout()
plt.savefig(f"{output_dir}/latency_distribution.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  Generated: latency_distribution.png")

# ========================================
# Chart 4: Jitter Analysis
# ========================================
fig, ax = plt.subplots(figsize=(10, 6))

for i, size in enumerate(sizes):
    data = df[df['size_bytes'] == size].sort_values('rate_hz')
    if not data.empty:
        ax.plot(data['rate_hz'], data['jitter_us'],
               label=f'{size} bytes',
               marker=markers[i % len(markers)],
               color=colors[i % len(colors)],
               markersize=8, linewidth=2)

ax.set_xlabel('Publish Rate (Hz)', fontsize=12)
ax.set_ylabel('Jitter (μs)', fontsize=12)
ax.set_title('CycloneDDS Jitter vs Publish Rate', fontsize=14, fontweight='bold')
ax.legend(title='Message Size', loc='best')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f"{output_dir}/jitter_vs_rate.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  Generated: jitter_vs_rate.png")

# ========================================
# Chart 5: Heatmap
# ========================================
fig, ax = plt.subplots(figsize=(10, 6))

pivot = df.pivot_table(values='mean_us', index='size_bytes', columns='rate_hz', aggfunc='mean')

im = ax.imshow(pivot.values, cmap='YlOrRd', aspect='auto')
cbar = plt.colorbar(im, ax=ax)
cbar.set_label('Mean Latency (μs)', fontsize=11)

ax.set_xticks(range(len(pivot.columns)))
ax.set_xticklabels([f'{int(r)} Hz' for r in pivot.columns])
ax.set_yticks(range(len(pivot.index)))
ax.set_yticklabels([f'{int(s)} B' for s in pivot.index])

ax.set_xlabel('Publish Rate', fontsize=12)
ax.set_ylabel('Message Size', fontsize=12)
ax.set_title('Latency Heatmap', fontsize=14, fontweight='bold')

# Add text annotations
for i in range(len(pivot.index)):
    for j in range(len(pivot.columns)):
        val = pivot.values[i, j]
        color = 'white' if val > pivot.values.max() * 0.6 else 'black'
        ax.text(j, i, f'{val:.1f}', ha='center', va='center', color=color, fontsize=10)

plt.tight_layout()
plt.savefig(f"{output_dir}/latency_heatmap.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  Generated: latency_heatmap.png")

# ========================================
# Print Summary
# ========================================
print("")
print("=" * 50)
print("Summary Statistics")
print("=" * 50)
print(f"  Mean Latency:  {df['mean_us'].mean():.2f} μs (avg)")
print(f"  P99 Latency:   {df['p99_us'].mean():.2f} μs (avg)")
print(f"  Jitter:        {df['jitter_us'].mean():.2f} μs (avg)")
print(f"  Best Mean:     {df['mean_us'].min():.2f} μs")
print(f"  Worst P99:     {df['p99_us'].max():.2f} μs")
print("=" * 50)
PYTHON_SCRIPT

# Generate summary report
REPORT_FILE="${RESULT_DIR}/REPORT.md"
cat > "$REPORT_FILE" << EOF
# CycloneDDS Performance Test Report

**Date:** $(date '+%Y-%m-%d %H:%M:%S')
**Test Mode:** ${TEST_MODE}
**Message Count:** ${COUNT}
**Warmup:** ${WARMUP}

## Test Configuration

| Parameter | Value |
|-----------|-------|
| Rates | ${RATES} Hz |
| Sizes | ${SIZES} bytes |

## Generated Charts

- \`latency_vs_rate.png\` - Latency vs Publish Rate
- \`latency_vs_size.png\` - Latency vs Message Size
- \`latency_distribution.png\` - Mean/P95/P99 Distribution
- \`jitter_vs_rate.png\` - Jitter Analysis
- \`latency_heatmap.png\` - Latency Heatmap

## Raw Data

See \`results.csv\` for detailed data.
EOF

echo ""
echo "============================================"
echo "Complete!"
echo "============================================"
echo ""
echo "Output Directory: ${RESULT_DIR}"
echo ""
echo "Generated Files:"
ls -1 "${RESULT_DIR}"/*.png 2>/dev/null | while read f; do echo "  $(basename $f)"; done
echo ""
echo "  results.csv"
echo "  REPORT.md"
echo ""
echo "View charts:"
echo "  xdg-open ${RESULT_DIR}/latency_vs_rate.png"
echo ""
