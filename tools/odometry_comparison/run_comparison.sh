#!/usr/bin/env bash

# Runs the position-only KISS-ICP and PX4 odometry comparison collector.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-/mnt/px4-workspace/px4_ros2_ws/install/setup.bash}"
KISS_ICP_SETUP="${KISS_ICP_SETUP:-/mnt/px4-workspace/kiss_icp_ws/install/setup.bash}"

if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
  echo "ROS 2 Lyrical setup was not found at /opt/ros/lyrical/setup.bash" >&2
  exit 1
fi

if [[ ! -f "$PX4_MSGS_SETUP" ]]; then
  echo "px4_msgs setup was not found: $PX4_MSGS_SETUP" >&2
  echo "Connect the PX4 workspace before running this comparison." >&2
  exit 1
fi

if [[ ! -f "$KISS_ICP_SETUP" ]]; then
  echo "KISS-ICP setup was not found: $KISS_ICP_SETUP" >&2
  echo "Connect the PX4 workspace and build KISS-ICP first." >&2
  exit 1
fi

# ROS-generated setup files may read unset variables, so nounset is paused here.
set +u
source /opt/ros/lyrical/setup.bash
source "$PX4_MSGS_SETUP"
source "$KISS_ICP_SETUP"
set -u

echo "Collecting KISS-ICP and PX4 position estimates"
echo "Start the KISS-ICP bag playback in a second terminal."
echo "After playback finishes, return here and press Ctrl+C to save the results."

exec python3 "$SCRIPT_DIR/compare_kiss_px4_odometry.py" "$@"
