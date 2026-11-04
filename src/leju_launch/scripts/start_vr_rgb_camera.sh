#!/usr/bin/env bash
# Start read-only upper-computer acquisition before the VR recorder connects.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"

# The hardware launcher is root; SSH credentials and vision config belong to lab.
if [[ "${EUID}" == 0 ]]; then
    exec runuser -u lab -- env \
        "LEJULAB_PROJECT_ROOT=${PROJECT_DIR}" \
        "VR_MCAP_PYTHON=${VR_MCAP_PYTHON:-python3}" \
        bash "${SCRIPT_DIR}/start_vr_rgb_camera.sh"
fi
unset PYTHONHOME PYTHONPATH
SSH_HOST="$("${VR_MCAP_PYTHON:-python3}" -c \
    'import json,sys; print(json.load(open(sys.argv[1]))["ssh_host"])' \
    "${PROJECT_DIR}/src/leju_launch/config/vr_vision_config.json")"
echo "[vr_camera] Starting upper RGB camera service via ${SSH_HOST}"
# systemctl start is idempotent: an active camera is never restarted here.
timeout 20s ssh -T -o BatchMode=yes -o ConnectTimeout=5 \
    -o ServerAliveInterval=5 -o ServerAliveCountMax=2 "${SSH_HOST}" \
    'sudo -n systemctl start vr-rgb-camera.service && systemctl is-active --quiet vr-rgb-camera.service'
echo "[vr_camera] Camera service active; wait for the RGB+MCAP Ready message"
