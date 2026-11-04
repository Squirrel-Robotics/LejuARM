#!/usr/bin/env bash
set -eo pipefail
# ROS setup files reference optional unset variables.
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/ros_ws/devel/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Wait for the existing master; do not start another ROS master.
# A camera-node exit ends roslaunch so systemd can restart this acquisition unit.
exec /opt/ros/noetic/bin/roslaunch --wait --required "${SCRIPT_DIR}/vr_rgb_camera.launch"
