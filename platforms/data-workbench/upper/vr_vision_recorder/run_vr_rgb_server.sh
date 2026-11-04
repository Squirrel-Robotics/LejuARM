#!/usr/bin/env bash
set -euo pipefail
set +u  # ROS/catkin setup scripts reference optional unset environment variables.
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/ros_ws/devel/setup.bash
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONUNBUFFERED=1
exec /usr/bin/python3 "${SCRIPT_DIR}/vr_rgb_server.py" "$@"
