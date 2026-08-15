#!/usr/bin/env bash
set -euo pipefail

# Find the workspace so ros2 can locate this package's compiled TF adapter.
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace_root="${LIDAR_WORKSPACE:-$(cd "$(dirname "$script_path")/../../.." && pwd)}"
GZ_PARTITION="${GZ_PARTITION:-px4_sitl}"
export GZ_PARTITION

if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "Workspace setup not found: $workspace_root/install/setup.bash" >&2
  echo "Build px4_sitl_bringup before starting the TF adapter." >&2
  exit 2
fi

# ROS setup scripts may read unset variables, so nounset is paused while they run.
set +u
source /opt/ros/lyrical/setup.bash
source "$workspace_root/install/setup.bash"
set -u

echo "Publishing TF chain world -> base_link -> lidar_link"
exec ros2 run px4_sitl_bringup x500_tf_bridge
