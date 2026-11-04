#!/bin/bash
_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="$_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$_DIR/bin${PATH:+:$PATH}"
export LEJULAB_INSTALL_ROOT="$_DIR"
export LEJULAB_PROJECT_ROOT="$(cd "$_DIR/.." && pwd)"
if [ -z "${ROBOT_VERSION:-}" ]; then
    echo "[lejulab] 请 export ROBOT_VERSION=<14|46|52> 后再 launch" >&2
fi
