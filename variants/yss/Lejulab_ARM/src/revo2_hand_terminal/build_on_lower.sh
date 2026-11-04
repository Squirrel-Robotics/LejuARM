#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$package_dir/../.." && pwd)"
arch="$(uname -m)"
third_party="$root/src/lejusdk/3rd_party/$arch"
if [[ -n "${CXX:-}" ]]; then compiler="$CXX";
elif command -v g++-11 >/dev/null 2>&1; then compiler=g++-11;
else compiler=g++; fi
output="$root/build/src/revo2_hand_terminal/revo2_hand_terminal"
mkdir -p -- "$(dirname -- "$output")"
temporary="$(mktemp "${output}.XXXXXX")"
trap 'rm -f -- "$temporary"' EXIT
"$compiler" -std=c++17 -O2 -Wall -Wextra -pthread \
    -I"$root/installed/include" \
    -I"$root/src/lejusdk/lejusdk-utils/include" \
    -I"$third_party/cyclonedds-cxx-0.10.2/include/ddscxx" \
    -I"$third_party/cyclonedds-0.10.2/include" \
    "$package_dir/revo2_hand_terminal.cpp" \
    -L"$root/installed/lib" "-Wl,-rpath,$root/installed/lib" \
    -llejusdk-lowlevel -o "$temporary"
chmod 755 -- "$temporary"
mv -f -- "$temporary" "$output"
echo "Built $output"
