#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
export LEJULAB_PROJECT_ROOT="${PROJECT_DIR}"
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
# The Python/C++ RPC pair on this robot does not receive replies over SHM.
# Keep the selected interfaces/discovery and change only this process's URI.
CYCLONEDDS_URI="$("${HEFT_EXTERNAL_PYTHON:-python3}" -c 'import os; from dds_heft_external_control import udp_only_uri; print(udp_only_uri(os.environ["CYCLONEDDS_URI"]))')"
export CYCLONEDDS_URI
# Default for the requested ACT test; explicit CLI speed arguments take precedence.
exec "${HEFT_EXTERNAL_PYTHON:-python3}" "${SCRIPT_DIR}/dds_heft_external_control.py" --max-speed-deg-s 40 "$@"
