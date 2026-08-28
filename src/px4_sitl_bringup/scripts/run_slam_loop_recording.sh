#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Project and workspace paths
# -----------------------------------------------------------------------------

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
project_root="$(cd "$(dirname "$script_path")/../../.." && pwd)"
project_setup="$project_root/install/setup.bash"

workspace_mount="${PX4_WORKSPACE_MOUNT:-/mnt/px4-workspace}"
px4_ros_setup="${PX4_ROS_SETUP:-$workspace_mount/px4_ros2_ws/install/setup.bash}"
bag_root="${SLAM_BAG_ROOT:-$workspace_mount/bags}"
bag_output="${SLAM_BAG_OUTPUT:-$bag_root/x500_slam_loop_$(date +%Y%m%d_%H%M%S)}"

# -----------------------------------------------------------------------------
# Storage and ROS environment checks
# -----------------------------------------------------------------------------

# Check the external workspace before sourcing or writing anything stored on it.
if ! mountpoint --quiet "$workspace_mount"; then
  echo "PX4 workspace is not mounted at $workspace_mount." >&2
  echo "Run: $project_root/scripts/px4_workspace.sh connect" >&2
  exit 2
fi

if [[ ! -f "$px4_ros_setup" ]]; then
  echo "PX4 ROS 2 interfaces not found: $px4_ros_setup" >&2
  echo "Build the PX4 ROS 2 workspace before recording PX4 topics." >&2
  exit 2
fi

if [[ ! -f "$project_setup" ]]; then
  echo "Project setup file not found: $project_setup" >&2
  echo "Build px4_sitl_bringup with colcon before recording." >&2
  exit 2
fi

if [[ -e "$bag_output" ]]; then
  echo "Recording destination already exists: $bag_output" >&2
  echo "Set SLAM_BAG_OUTPUT to a new path, or use the timestamped default." >&2
  exit 2
fi

mkdir -p "$(dirname "$bag_output")"

set +u
source /opt/ros/lyrical/setup.bash
source "$px4_ros_setup"
source "$project_setup"
set -u

# -----------------------------------------------------------------------------
# Live pipeline checks
# -----------------------------------------------------------------------------

required_topics=(
  /clock
  /x500/lidar/points
  /fmu/out/vehicle_odometry
  /fmu/out/vehicle_status_v1
  /tf
  /tf_static
)

echo "Waiting up to 15 seconds for the complete live SLAM pipeline..."

for _ in {1..15}; do
  available_topics="$(ros2 topic list)"
  missing_topics=()

  for topic in "${required_topics[@]}"; do
    if ! grep -Fxq "$topic" <<< "$available_topics"; then
      missing_topics+=("$topic")
    fi
  done

  if (( ${#missing_topics[@]} == 0 )); then
    break
  fi

  sleep 1
done

if (( ${#missing_topics[@]} != 0 )); then
  echo "Recording did not start because these topics are missing:" >&2
  printf '  %s\n' "${missing_topics[@]}" >&2
  echo "Start PX4 with run_px4.sh, then try the recorder again." >&2
  exit 2
fi

# -----------------------------------------------------------------------------
# Recording
# -----------------------------------------------------------------------------

echo "Starting a clock-complete X500 SLAM recording"
echo "Output: $bag_output"
echo "Fly the controlled loop, land, then press Ctrl+C here to finish the bag."

exec ros2 launch px4_sitl_bringup record_slam_loop.launch.py \
  "output:=$bag_output"
