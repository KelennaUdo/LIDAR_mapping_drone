#!/usr/bin/env bash
set -euo pipefail

# Find this package's config directory from either the source or install tree.
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
package_share="$(cd "$(dirname "$script_path")/.." && pwd)"
bridge_config="${LIDAR_BRIDGE_CONFIG:-$package_share/config/lidar_bridge.yaml}"
GZ_PARTITION="${GZ_PARTITION:-px4_sitl}"
export GZ_PARTITION

if [[ ! -f "$bridge_config" ]]; then
  echo "LiDAR bridge configuration not found: $bridge_config" >&2
  exit 2
fi

# ROS setup scripts may read unset variables, so nounset is paused while they run.
set +u
source /opt/ros/lyrical/setup.bash
set -u

echo "Gazebo Transport partition: $GZ_PARTITION"
echo "Bridging Gazebo /x500/lidar/points to ROS 2 /x500/lidar/points"
exec ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p "config_file:=$bridge_config"
