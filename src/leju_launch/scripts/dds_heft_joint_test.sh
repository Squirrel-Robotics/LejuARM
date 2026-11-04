#!/usr/bin/env bash
# Record a pose first, then keep prior joint references during one HEFT session.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/dds_heft_pose_test.sh" --single-joint --heft-for-head "$@"
