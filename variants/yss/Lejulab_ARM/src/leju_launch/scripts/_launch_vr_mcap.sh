#!/usr/bin/env bash
# Sourced by real-hardware launchers; this node only subscribes to DDS.
launch_vr_mcap_recorder() {
    [[ "${VR_MCAP_ENABLED:-true}" == true ]] || return 0
    "${SCRIPT_DIR}/start_node.sh" "${SCRIPT_DIR}/run_vr_mcap_recorder.sh" \
        --domain-id "${VR_MCAP_DOMAIN_ID:-${PICO_DOMAIN_ID:-0}}" \
        --output-dir "${VR_MCAP_OUTPUT_DIR:-/home/lab/.cache/lejulab/vr_transfer}" &
    local pid=$!
    ALL_PIDS+=("${pid}")
    echo "[launcher] Started VR MCAP listener (PID ${pid}); LT+Y start, LT+X stop"
}
