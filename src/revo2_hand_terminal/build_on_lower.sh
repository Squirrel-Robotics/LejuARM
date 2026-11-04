#!/usr/bin/env bash
set -euo pipefail

root=/home/lab/Lejulab_ARM
package_dir="$root/src/revo2_hand_terminal"
root_cmake="$root/CMakeLists.txt"
package_line='add_subdirectory(src/revo2_hand_terminal)'
anchor='include(${CMAKE_CURRENT_SOURCE_DIR}/source_packages.cmake)'

if [[ ! -f "$package_dir/CMakeLists.txt" || ! -f "$root_cmake" ]]; then
  echo "Expected Lejulab_ARM package or root CMakeLists.txt is missing." >&2
  exit 1
fi

if ! grep -Fq "$package_line" "$root_cmake" &&
   ! grep -Fq "$package_line" "$root/source_packages.cmake"; then
  if ! grep -Fqx "$anchor" "$root_cmake"; then
    echo "Could not find the SDK package anchor in root CMakeLists.txt." >&2
    exit 1
  fi

  backup="$(mktemp "$root/CMakeLists.txt.before-revo2.XXXXXX")"
  cp -p "$root_cmake" "$backup"
  sed -i "/^include(\${CMAKE_CURRENT_SOURCE_DIR}\/source_packages.cmake)$/a $package_line" "$root_cmake"
  if ! grep -Fqx "$package_line" "$root_cmake"; then
    cp -p "$backup" "$root_cmake"
    echo "Could not add package to the root build; restored CMakeLists.txt." >&2
    exit 1
  fi
  echo "Added hand terminal to root build. Original saved at $backup"
fi

cmake -S "$root" -B "$root/build"
cmake --build "$root/build" --target revo2_hand_terminal -j2
echo "Built $root/build/src/revo2_hand_terminal/revo2_hand_terminal"
