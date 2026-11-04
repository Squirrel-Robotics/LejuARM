#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project="$(cd -- "$package_dir/../.." && pwd)"
source_file="$project/src/right_hand_canfd_terminal/right_hand_canfd_terminal.cpp"
binary="$project/src/right_hand_canfd_terminal/right_hand_canfd_terminal"
sdk="$project/installed/lib/libbc_stark_sdk.so"

if [[ ! -f "$source_file" || ! -f "$sdk" ]]; then
  echo "Source or BrainCo SDK library is missing on the lower computer." >&2
  exit 1
fi

g++ -std=c++17 -O2 -Wall -Wextra -pthread \
  "$source_file" "$sdk" \
  -Wl,-rpath,"$project/installed/lib" \
  -o "$binary"

echo "Built $binary"
