#!/usr/bin/env bash
set -euo pipefail

# Locate the project-owned RViz configuration in source or install space.
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
package_share="$(cd "$(dirname "$script_path")/.." && pwd)"
rviz_config="${LIDAR_RVIZ_CONFIG:-$package_share/rviz/x500_lidar.rviz}"

if [[ ! -f "$rviz_config" ]]; then
  echo "X500 LiDAR RViz configuration not found: $rviz_config" >&2
  exit 2
fi

# ROS setup scripts may read unset variables, so nounset is paused while they run.
set +u
source /opt/ros/lyrical/setup.bash
set -u

echo "Opening RViz with /x500/lidar/points in frame world"
exec rviz2 -d "$rviz_config"
