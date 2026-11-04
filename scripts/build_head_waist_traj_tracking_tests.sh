#!/usr/bin/env bash
# 在下位机 Lejulab_ARM 目录执行：
#   bash scripts/build_head_waist_traj_tracking_tests.sh
# 只编译测试程序，不启动机器人运动，不修改已有 arm_traj_tracking_test。
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
installed_dir="$project_dir/installed"
third_party_dir="$project_dir/src/lejusdk/3rd_party/aarch64"
dds_cxx_include="$third_party_dir/cyclonedds-cxx-0.10.2/include/ddscxx"
dds_c_include="$third_party_dir/cyclonedds-0.10.2/include"
library_dir="$installed_dir/lib"

if [[ -n "${CXX:-}" ]]; then
  compiler="$CXX"
elif command -v g++-11 >/dev/null 2>&1; then
  compiler=g++-11
else
  compiler=g++
fi
if ! command -v "$compiler" >/dev/null 2>&1; then
  echo "找不到 C++ 编译器：$compiler（建议安装 g++-11，或设置 CXX）。" >&2
  exit 1
fi

for required_dir in "$installed_dir/include" "$library_dir" "$dds_cxx_include" "$dds_c_include"; do
  if [[ ! -d "$required_dir" ]]; then
    echo "缺少依赖目录：$required_dir" >&2
    echo "请把 scripts 文件放入下位机 /home/lab/Lejulab_ARM，并确认 SDK 已安装。" >&2
    exit 1
  fi
done
for required_file in traj_tracking_common.hpp tracking_rpc.hpp waist_only_tracking.hpp head_traj_tracking_test.cpp waist_traj_tracking_test.cpp; do
  if [[ ! -f "$script_dir/$required_file" ]]; then
    echo "缺少测试源码：$script_dir/$required_file" >&2
    exit 1
  fi
done

include_flags=(
  -isystem "$installed_dir/include"
  "-I$project_dir/src/lejusdk/lejusdk-utils/include"
  -isystem "$dds_cxx_include"
  -isystem "$dds_c_include"
)
# CycloneDDS 的共享内存头文件引用 iceoryx_hoofs/...；SDK 常见布局为该目录。
if [[ -d "$third_party_dir/iceoryx/include/iceoryx/v" ]]; then
  include_flags+=("-I$third_party_dir/iceoryx/include/iceoryx/v")
elif [[ -d "$third_party_dir/iceoryx/include" ]]; then
  while IFS= read -r iceoryx_hoofs_dir; do
    include_flags+=("-I$(dirname -- "$iceoryx_hoofs_dir")")
  done < <(find "$third_party_dir/iceoryx/include" -type d -name iceoryx_hoofs -print)
fi

mkdir -p -- "$project_dir/build"
temporary_dir="$(mktemp -d "$project_dir/build/.head-waist-tests.XXXXXX")"
trap 'rm -rf -- "$temporary_dir"' EXIT

compile_flags=(-std=c++17 -O2 -g -fno-omit-frame-pointer -Wall -Wextra -pthread)
link_flags=(
  "-L$library_dir"
  "-Wl,-rpath,$library_dir"
  -llejusdk-vr -llejusdk-lowlevel -llejusdk-dds-idl -llejusdk-dds-rpc -lddscxx -lddsc -lyaml-cpp -lpthread
)

echo "使用编译器：$compiler"
for test_name in head_traj_tracking_test waist_traj_tracking_test; do
  echo "编译 $test_name ..."
  "$compiler" "${compile_flags[@]}" "${include_flags[@]}" \
    "$script_dir/$test_name.cpp" -o "$temporary_dir/$test_name" "${link_flags[@]}"
  chmod 755 -- "$temporary_dir/$test_name"
done

# 两个程序都编译成功后才替换 build 中的输出，避免失败时留下半成品。
for test_name in head_traj_tracking_test waist_traj_tracking_test; do
  mv -f -- "$temporary_dir/$test_name" "$project_dir/build/$test_name"
done
echo "完成：$project_dir/build/head_traj_tracking_test"
echo "完成：$project_dir/build/waist_traj_tracking_test"
