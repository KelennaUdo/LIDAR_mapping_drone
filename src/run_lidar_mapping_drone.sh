#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
