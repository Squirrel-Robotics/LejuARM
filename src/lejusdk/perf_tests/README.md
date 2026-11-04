# CycloneDDS 性能测试工具

CycloneDDS + iceoryx 的延迟性能测试套件，用于测量 DDS 通信的端到端延迟和往返时间 (RTT)。

## 目录结构

```
perf_tests/
├── include/
│   └── perf_utils.hpp       # 性能测试工具类 (LatencyStats, PerfTestArgs)
├── src/
│   ├── roundtrip_test.cpp       # 单进程往返延迟测试
│   ├── latency_publisher.cpp    # 跨进程延迟测试 - 发布者
│   ├── latency_subscriber.cpp   # 跨进程延迟测试 - 订阅者
│   ├── multi_topic_test.cpp     # 多 topic 延迟测试 (单进程)
│   ├── multi_topic_publisher.cpp # 多 topic 延迟测试 - 发布者
│   └── multi_topic_subscriber.cpp # 多 topic 延迟测试 - 订阅者
├── scripts/
│   ├── run_latency_test.sh      # 延迟测试脚本
│   ├── run_benchmark_suite.sh   # 基准测试套件
│   ├── run_comparison_test.sh   # 传输方式对比测试
│   ├── run_and_plot.sh          # 一键测试 + 生成 PNG 图表
│   └── parse_results.py         # 结果解析器
└── dds_perf_results/            # 测试结果输出目录 (自动生成)
```

## 测试程序

### roundtrip_test - 单进程往返延迟测试

在单进程内使用两个 topic 测量完整的往返时间 (RTT):
- `/perf/ping`: 发布者发送消息
- `/perf/pong`: Pong 订阅者收到后立即回传

```bash
rosrun lejusdk-perf-tests roundtrip_test --rate 1000 --size 64 --count 100000
```

### latency_publisher / latency_subscriber - 跨进程延迟测试

测量两个独立进程之间的单向延迟:

```bash
# 终端 1: 启动订阅者
rosrun lejusdk-perf-tests latency_subscriber --topic /perf/test --count 100000

# 终端 2: 启动发布者
rosrun lejusdk-perf-tests latency_publisher --topic /perf/test --rate 1000 --size 64 --count 100000
```

### multi_topic_test - 多 topic 延迟测试 (单进程)

模拟机器人控制循环，同时发布 JointState 和 ImuData 两个 topic:

```bash
rosrun lejusdk-perf-tests multi_topic_test --rate 1000 --count 100000
```

### multi_topic_publisher / multi_topic_subscriber - 跨进程多 topic 测试

跨进程测试多 topic 的延迟性能:

```bash
# 终端 1: 启动订阅者
rosrun lejusdk-perf-tests multi_topic_subscriber --rate 1000 --count 100000

# 终端 2: 启动发布者
rosrun lejusdk-perf-tests multi_topic_publisher --rate 1000 --count 100000
```

### 通用命令行参数

| 参数 | 缩写 | 说明 | 默认值 |
|------|------|------|--------|
| `--topic` | `-t` | Topic 名称 | `/perf/test` |
| `--rate` | `-r` | 发布频率 (Hz) | 1000 |
| `--size` | `-s` | 消息大小 (bytes) | 64 |
| `--count` | `-c` | 消息数量 | 100000 |
| `--warmup` | `-w` | 预热消息数量 | 100 |

## 测试脚本

### run_latency_test.sh - 延迟测试脚本

一键运行各类延迟测试:

```bash
# 运行发布者/订阅者延迟测试
./scripts/run_latency_test.sh latency --rate 1000 --size 512 --count 100000

# 运行单进程往返测试
./scripts/run_latency_test.sh roundtrip --rate 500 --size 1024

# 运行多 topic 测试
./scripts/run_latency_test.sh multi_topic --rate 1000 --count 100000

# 运行所有测试
./scripts/run_latency_test.sh all

# 使用共享内存传输
./scripts/run_latency_test.sh latency --shm
```

### run_benchmark_suite.sh - 基准测试套件

自动化运行多种参数组合的性能测试:

```bash
# 预览测试计划
./scripts/run_benchmark_suite.sh --dry-run

# 运行完整测试套件
./scripts/run_benchmark_suite.sh

# 自定义参数
./scripts/run_benchmark_suite.sh --rates "500,1000,2000" --sizes "64,256,1024" --count 100000

# 仅测试特定模式
./scripts/run_benchmark_suite.sh --test-modes "latency,roundtrip"

# 使用共享内存
./scripts/run_benchmark_suite.sh --shm
```

输出:
- `dds_perf_results/benchmark_YYYYMMDD_HHMMSS/` - 测试结果目录
- `benchmark_results.csv` - CSV 格式汇总
- `REPORT.md` - Markdown 格式报告

### run_comparison_test.sh - 传输方式对比测试

比较网络传输 (UDP) 和共享内存 (iceoryx) 的性能差异:

```bash
# 对比延迟测试
./scripts/run_comparison_test.sh --mode latency --rate 1000

# 对比往返测试
./scripts/run_comparison_test.sh --mode roundtrip --rate 2000 --size 1024

# 对比多 topic 测试
./scripts/run_comparison_test.sh --mode multi_topic --rate 1000 --count 100000
```

**注意**: 共享内存测试需要运行 RouDi 守护进程。脚本会自动尝试启动，或者手动运行:
```bash
iox-roudi &
```

### run_and_plot.sh - 一键测试并生成图表

运行性能测试并自动生成 PNG 图表:

```bash
# 默认测试 (roundtrip, 多种频率和大小)
./scripts/run_and_plot.sh

# 快速测试 (较少参数组合)
./scripts/run_and_plot.sh --quick

# 自定义参数
./scripts/run_and_plot.sh --rates "500,1000,2000" --sizes "64,256,1024" --count 100000

# 指定测试模式
./scripts/run_and_plot.sh --mode latency
./scripts/run_and_plot.sh --mode multi_topic
```

生成的图表:
- `latency_vs_rate.png` - 延迟 vs 发布频率
- `latency_vs_size.png` - 延迟 vs 消息大小
- `latency_distribution.png` - Mean/P95/P99 分布
- `jitter_vs_rate.png` - 抖动分析
- `latency_heatmap.png` - 延迟热力图

### parse_results.py - 结果解析器

解析测试日志并导出为多种格式:

```bash
# 显示表格格式
./scripts/parse_results.py test_output.log --format table

# 导出 CSV
./scripts/parse_results.py test_output.log --format csv --output results.csv

# 导出 JSON
./scripts/parse_results.py test_output.log --format json

# 批量解析目录
./scripts/parse_results.py results_dir/ --batch --format markdown
```

## 编译和运行

### 编译

```bash
# 编译 perf_tests 包
catkin build lejusdk-perf-tests

# 或编译整个 workspace
catkin build
```

### 环境配置

```bash
# Source workspace
source devel/setup.bash

# 设置 CycloneDDS 配置 (可选)
export CYCLONEDDS_URI="file://$(pwd)/src/leju_launch/config/cyclonedds.xml"

# 使用共享内存时
export CYCLONEDDS_URI="file://$(pwd)/src/leju_launch/config/cyclonedds_shm.xml"
```

### 快速测试

```bash
# 运行基本往返测试
rosrun lejusdk-perf-tests roundtrip_test

# 使用脚本运行完整测试
./src/lejusdk/perf_tests/scripts/run_latency_test.sh roundtrip
```

## 测试结果

### 输出格式

测试程序输出延迟统计信息，包括:

- **Min/Max**: 最小/最大延迟
- **Mean**: 平均延迟
- **Median**: 中位数延迟
- **P95/P99**: 95/99 百分位延迟
- **Std Dev**: 标准差
- **Jitter**: 抖动 (相邻采样差的平均值)

示例输出:
```
============ Round-Trip Latency Results ============
Topic: /perf/ping -> /perf/pong
Message Size: 64 bytes
Publish Rate: 1000 Hz
Messages: 100000
----------------------------------------------
Latency Statistics (microseconds):
  Min:          12.3 us
  Max:         156.7 us
  Mean:         18.5 us
  Median:       17.2 us
  P95:          28.4 us
  P99:          45.6 us
  Std Dev:       8.2 us
  Jitter:        3.1 us
==============================================

Histogram:
  [    12-    26 us]: ######################################## 85420 (85.4%)
  [    26-    41 us]: #####                                     11230 (11.2%)
  ...
```

### 结果文件

使用 `run_benchmark_suite.sh` 时，结果保存在:

```
dds_perf_results/
└── benchmark_YYYYMMDD_HHMMSS/
    ├── benchmark_results.csv    # CSV 汇总数据
    ├── REPORT.md                # Markdown 报告
    ├── latency_r1000_s64.log    # 各测试原始日志
    ├── roundtrip_r1000_s256.log
    └── ...
```

## 依赖

### 编译依赖
- CycloneDDS 0.10.2+
- CycloneDDS-CXX 0.10.2+
- iceoryx (用于共享内存传输)
- lejusdk-dds-idl
- lejusdk-topic-pubsub

### 图表生成依赖 (run_and_plot.sh)
```bash
pip3 install matplotlib pandas
```
