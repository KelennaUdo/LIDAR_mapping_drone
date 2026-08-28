#!/usr/bin/env bash
set -euo pipefail

# Resolve project paths whether this script is called through source or install space.
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
project_root="$(cd "$(dirname "$script_path")/../../.." && pwd)"
project_setup="$project_root/install/setup.bash"

workspace_mount="${PX4_WORKSPACE_MOUNT:-/mnt/px4-workspace}"
kiss_install="${KISS_ICP_INSTALL_DIR:-$workspace_mount/kiss_icp_ws/install}"
kiss_setup="${KISS_ICP_SETUP:-$kiss_install/setup.bash}"
database_path="${RTABMAP_DATABASE_PATH:-$workspace_mount/rtabmap_maps/x500_mapping.db}"

# System ROS provides ros2 itself and the apt-installed RTAB-Map packages.
set +u
source /opt/ros/lyrical/setup.bash
set -u

# Validate the removable workspace before using KISS-ICP or writing a map to it.
if ! mountpoint --quiet "$workspace_mount"; then
  echo "PX4 workspace is not mounted at $workspace_mount." >&2
  echo "Run: $project_root/scripts/px4_workspace.sh connect" >&2
  exit 2
fi

if [[ ! -f "$kiss_setup" ]]; then
  echo "KISS-ICP setup file not found: $kiss_setup" >&2
  echo "Build the KISS-ICP workspace before starting SLAM." >&2
  exit 2
fi

if [[ ! -f "$project_setup" ]]; then
  echo "Project setup file not found: $project_setup" >&2
  echo "Build the project with colcon before starting SLAM." >&2
  exit 2
fi

# The database survives container and ROS process shutdown on the external drive.
mkdir -p "$(dirname "$database_path")"

set +u
source "$kiss_setup"
source "$project_setup"
set -u

echo "Starting offline X500 LiDAR SLAM"
echo "Point cloud:  /x500/lidar/points"
echo "Clock:        /clock from the recorded Gazebo session"
echo "Odometry:     /kiss/odometry"
echo "Map database: $database_path"
echo "Replay the recorded clock and LiDAR topics in another terminal:"
printf '%s\n' \
  "  ros2 bag play <bag> --topics /clock /x500/lidar/points"

exec ros2 launch px4_sitl_bringup rtabmap_slam.launch.py \
  "database_path:=$database_path" \
  "$@"
