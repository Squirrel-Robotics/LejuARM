#!/usr/bin/env bash
set -euo pipefail
act_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
act_root="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["native_root"])' "$act_dir/config.json")"
export LEJULAB_PROJECT_ROOT="$act_root"
export CYCLONEDDS_HOME="$act_root/src/lejusdk/3rd_party/aarch64/cyclonedds-0.10.2"
export CYCLONEDDS_LIB_DIR="$CYCLONEDDS_HOME/lib"
export LD_LIBRARY_PATH="$CYCLONEDDS_LIB_DIR:${LD_LIBRARY_PATH:-}"
exec python3 "$act_dir/robot_relay.py" --config "$act_dir/config.json" "$@"
