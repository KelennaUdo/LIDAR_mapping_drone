#!/usr/bin/env bash
set -eo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
project_root="$(cd "$(dirname "$script_path")/../../.." && pwd)"
px4_ros_setup="${PX4_ROS_SETUP:-/mnt/px4-workspace/px4_ros2_ws/install/setup.bash}"
project_setup="$project_root/install/setup.bash"

if [[ ! -t 0 ]]; then
  echo "Offboard teleop must run in an interactive terminal." >&2
  exit 2
fi

if [[ ! -f "$px4_ros_setup" ]]; then
  echo "PX4 ROS 2 interfaces not found: $px4_ros_setup" >&2
  echo "Connect the external workspace before running this script." >&2
  exit 2
fi

if [[ ! -f "$project_setup" ]]; then
  echo "Project install setup not found: $project_setup" >&2
  echo "Build px4_offboard_control with colcon first." >&2
  exit 2
fi

source /opt/ros/lyrical/setup.bash
source "$px4_ros_setup"
source "$project_setup"

# Run the executable directly so it owns this terminal's keyboard input.
exec ros2 run px4_offboard_control offboard_teleop --ros-args "$@"
