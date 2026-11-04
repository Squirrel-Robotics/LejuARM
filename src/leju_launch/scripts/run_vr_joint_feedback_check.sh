#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f /etc/cyclonedds/cyclonedds_shm.xml ]]; then
        export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml
    else
        export CYCLONEDDS_URI="file://${PROJECT_DIR}/src/leju_launch/config/cyclonedds_shm.xml"
    fi
fi
if [[ "${EUID}" == 0 ]]; then
    exec runuser -u lab -- env "CYCLONEDDS_URI=${CYCLONEDDS_URI}" \
        "LD_LIBRARY_PATH=${PROJECT_DIR}/installed/lib:${LD_LIBRARY_PATH:-}" \
        "CYCLONEDDS_LIB_DIR=${PROJECT_DIR}/installed/lib" \
        bash "${SCRIPT_DIR}/run_vr_joint_feedback_check.sh" "$@"
fi
unset PYTHONHOME PYTHONPATH PYTHONNOUSERSITE
export CYCLONEDDS_LIB_DIR="${PROJECT_DIR}/installed/lib"
export LD_LIBRARY_PATH="${CYCLONEDDS_LIB_DIR}:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 "${SCRIPT_DIR}/vr_joint_feedback_check.py" "$@"
