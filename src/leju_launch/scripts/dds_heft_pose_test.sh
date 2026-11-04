#!/usr/bin/env bash
# S17 measured-pose capture and relative test planning.
# Default is READ ONLY; --execute tests seven in-limit targets per upper joint.
# All asset/result paths are resolved from this repository, with no CLI paths needed.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
ARCH="$(uname -m)"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-${PROJECT_DIR}/src/lejusdk/3rd_party/${ARCH}/cyclonedds-0.10.2}"
export CYCLONEDDS_LIB_DIR="${CYCLONEDDS_LIB_DIR:-${CYCLONEDDS_HOME}/lib}"
export LD_LIBRARY_PATH="${CYCLONEDDS_LIB_DIR}:${LD_LIBRARY_PATH:-}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f /etc/cyclonedds/cyclonedds_shm.xml ]]; then
        export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds_shm.xml"
    else
        export CYCLONEDDS_URI="file://${PROJECT_DIR}/src/leju_launch/config/cyclonedds.xml"
    fi
fi
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
exec "${HEFT_TEST_PYTHON:-python3}" "${SCRIPT_DIR}/dds_heft_pose_test.py" "$@"
