#!/usr/bin/env bash
# 传入 --preflight 时只读；不带参数会执行正常的腰部运动测试。
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
test_binary="$project_dir/build/waist_traj_tracking_test"
if ! command -v gdb >/dev/null 2>&1; then
  echo "缺少 gdb，无法记录段错误调用栈。" >&2
  exit 1
fi
if [[ ! -x "$test_binary" ]]; then
  echo "找不到测试程序：$test_binary；先执行 scripts/build_head_waist_traj_tracking_tests.sh。" >&2
  exit 1
fi

mkdir -p -- "$project_dir/build/tracking_debug"
log_file="$(mktemp "$project_dir/build/tracking_debug/waist_$(date +%Y%m%d_%H%M%S).XXXXXX.log")"
echo "调试日志：$log_file"
# 正常退出时保留测试返回码；崩溃时打印所有线程调用栈。
# Ctrl+C 交给测试程序，停止腰轨迹并尝试腰 KeepPose；保留其他关节固定。
gdb -q -batch --return-child-result \
  -ex 'set confirm off' \
  -ex 'set disable-randomization off' \
  -ex 'set pagination off' \
  -ex 'set print elements 32' \
  -ex 'handle SIGINT nostop noprint pass' \
  -ex run \
  -ex 'thread apply all bt 20' \
  --args "$test_binary" "$@" 2>&1 | tee "$log_file"
