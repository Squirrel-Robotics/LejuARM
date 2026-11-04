#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
cd "${PROJECT_DIR}"

# Recording belongs to lab even when the hardware launcher runs as root.
# Use lab's existing system-Python DDS/MCAP dependencies, not the GMR venv.
if [[ "${EUID}" == 0 ]]; then
    exec runuser -u lab -- env \
        "LEJULAB_PROJECT_ROOT=${PROJECT_DIR}" \
        "CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://${PROJECT_DIR}/src/leju_launch/config/cyclonedds.xml}" \
        "VR_MCAP_PYTHON=${VR_MCAP_PYTHON:-python3}" \
        "LD_LIBRARY_PATH=${PROJECT_DIR}/installed/lib:${LD_LIBRARY_PATH:-}" \
        "CYCLONEDDS_LIB_DIR=${PROJECT_DIR}/installed/lib" \
        bash "${SCRIPT_DIR}/run_vr_mcap_recorder.sh" "$@"
fi
unset PYTHONHOME PYTHONPATH
export CYCLONEDDS_LIB_DIR="${PROJECT_DIR}/installed/lib"
export LD_LIBRARY_PATH="${CYCLONEDDS_LIB_DIR}:${LD_LIBRARY_PATH:-}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${PROJECT_DIR}/src/leju_launch/config/cyclonedds.xml}"
export PYTHONUNBUFFERED=1
exec "${VR_MCAP_PYTHON:-python3}" "${SCRIPT_DIR}/vr_mcap_recorder.py" \
    --recorder "${PROJECT_DIR}/installed/bin/lejusdk_recorder" \
    --config "${PROJECT_DIR}/src/leju_launch/config/vr_mcap_recorder.yaml" "$@"
