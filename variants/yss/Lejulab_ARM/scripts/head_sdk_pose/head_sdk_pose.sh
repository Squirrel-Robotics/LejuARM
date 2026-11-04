#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/../.." && pwd)"
arch="$(uname -m)"
bridge="$script_dir/build/$arch/head_legacy_sdk_cli"
export HEAD_SDK_BRIDGE="$bridge"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$project_dir/src/lejusdk/3rd_party/$arch/cyclonedds-0.10.2}"
export CYCLONEDDS_LIB_DIR="${CYCLONEDDS_LIB_DIR:-$CYCLONEDDS_HOME/lib}"
export LD_LIBRARY_PATH="$CYCLONEDDS_LIB_DIR:$project_dir/installed/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/cyclonedds_shm.xml}"

build_bridge() {
  local compiler="${CXX:-g++}" temporary
  command -v "$compiler" >/dev/null || { echo "缺少编译器：$compiler" >&2; return 1; }
  [[ -f "$project_dir/installed/include/lejusdk-vr/lejusdk_vr.h" ]] || {
    echo "缺少下位机旧SDK；请确认 LEJULAB_PROJECT_ROOT=$project_dir" >&2; return 1;
  }
  mkdir -p -- "$(dirname -- "$bridge")"
  temporary="$(mktemp "${bridge}.XXXXXX")"
  if "$compiler" -std=c++17 -O2 -Wall -Wextra \
      "$script_dir/head_legacy_sdk.cpp" -o "$temporary" \
      -I"$project_dir/installed/include" \
      -I"$project_dir/src/lejusdk/lejusdk-utils/include" \
      -L"$project_dir/installed/lib" \
      "-Wl,-rpath,$project_dir/installed/lib" -llejusdk-vr; then
    chmod 755 -- "$temporary"
    mv -f -- "$temporary" "$bridge"
  else
    rm -f -- "$temporary"
    return 1
  fi
  echo "头部SDK适配器已编译：$bridge" >&2
}

if [[ "${1:-}" == "--build" ]]; then
  [[ $# -eq 1 ]] || { echo "--build 单独使用，只编译，不发送指令。" >&2; exit 2; }
  build_bridge
  exit
fi
for option in "$@"; do
  if [[ "$option" == "--execute" ]]; then
    if [[ ! -x "$bridge" || "$script_dir/head_legacy_sdk.cpp" -nt "$bridge" ]]; then
      build_bridge
    fi
    break
  fi
done
exec "${HEAD_SDK_PYTHON:-python3}" "$script_dir/head_sdk_pose.py" "$@"
