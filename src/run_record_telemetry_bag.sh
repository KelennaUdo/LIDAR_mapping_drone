#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
BAG_DIR="$WORKSPACE_DIR/bags"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
DEFAULT_OUTPUT="$BAG_DIR/telemetry_sensors_${TIMESTAMP}"

source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

set -u

mkdir -p "$BAG_DIR"

has_output_arg=false
for arg in "$@"; do
  if [[ "$arg" == output:=* ]]; then
    has_output_arg=true
    break
  fi
done

if [[ "$has_output_arg" == false ]]; then
  set -- "output:=$DEFAULT_OUTPUT" "$@"
fi

ros2 launch lidar_mapping_drone_bringup record_telemetry_bag.launch.py "$@"
