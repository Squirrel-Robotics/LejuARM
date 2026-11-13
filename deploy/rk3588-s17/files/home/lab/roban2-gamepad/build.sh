#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
sdk=/home/lab/Lejulab_ARM
g++ -std=c++17 -O2 -Wall -Wextra -Werror test_jog_core.cpp -o test_jog_core
./test_jog_core
g++ -std=c++17 -O2 -Wall -Wextra -pthread \
  -I"$sdk/src/lejusdk/lejusdk-utils/include" \
  -isystem "$sdk/installed/include" \
  roban_arm_jog.cpp -o roban_arm_jog \
  -L"$sdk/installed/lib" -Wl,-rpath,"$sdk/installed/lib" \
  -llejusdk-lowlevel -llejusdk-vr
