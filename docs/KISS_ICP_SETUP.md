# KISS-ICP LiDAR Odometry

KISS-ICP estimates the X500's movement by aligning consecutive 3D LiDAR scans.
It runs on the Ubuntu host and does not command PX4 or individual motors.

## Runtime Architecture

```text
Gazebo X500 3D LiDAR
        |
        v
/x500/lidar/points     sensor_msgs/msg/PointCloud2
        |
        v
KISS-ICP
        |--- /kiss/odometry
        |--- /kiss/frame
        |--- /kiss/keypoints
        |--- /kiss/local_map
        `--- lidar_link -> odom_lidar TF
```

The inverted TF direction is intentional. The Gazebo TF adapter already
publishes `base_link -> lidar_link`, so KISS publishes
`lidar_link -> odom_lidar` to avoid giving `lidar_link` two TF parents.

## External Workspace

KISS-ICP is pinned to release `v1.3.0` under the external ext4 workspace:

```text
/mnt/px4-workspace/kiss_icp_ws/
```

The source checkout is:

```text
/mnt/px4-workspace/kiss_icp_ws/src/kiss-icp
```

The generated ROS installation is:

```text
/mnt/px4-workspace/kiss_icp_ws/install
```

## Recreate The Workspace

Connect the external workspace before running these commands:

```bash
mkdir -p /mnt/px4-workspace/kiss_icp_ws/src

git clone \
  --branch v1.3.0 \
  --depth 1 \
  https://github.com/PRBonn/kiss-icp.git \
  /mnt/px4-workspace/kiss_icp_ws/src/kiss-icp
```

Ubuntu 26.04 currently provides CMake 4.2. KISS-ICP's ROS project must enable
both C and C++ before ROS message-support targets are loaded. In this file:

```text
/mnt/px4-workspace/kiss_icp_ws/src/kiss-icp/ros/CMakeLists.txt
```

change:

```cmake
project(kiss_icp VERSION 1.3.0 LANGUAGES CXX)
```

to:

```cmake
project(kiss_icp VERSION 1.3.0 LANGUAGES C CXX)
```

Build the ROS package:

```bash
source /opt/ros/lyrical/setup.bash
cd /mnt/px4-workspace/kiss_icp_ws

colcon build \
  --packages-select kiss_icp \
  --symlink-install \
  --cmake-clean-cache \
  --event-handlers console_direct+
```

A successful build ends with:

```text
Finished <<< kiss_icp
Summary: 1 package finished
```

## Normal Startup

The existing PX4 launcher starts KISS-ICP by default:

```bash
cd /home/kelenna-udo/LIDAR_mapping_drone
./scripts/px4_workspace.sh connect
./src/px4_sitl_bringup/scripts/run_px4.sh
```

The same workflow is available through ROS launch:

```bash
source /opt/ros/lyrical/setup.bash
source /home/kelenna-udo/LIDAR_mapping_drone/install/setup.bash

ros2 launch px4_sitl_bringup px4.launch.py
```

Disable KISS-ICP for a flight-only test with either entry point:

```bash
START_KISS_ICP=0 \
  ./src/px4_sitl_bringup/scripts/run_px4.sh
```

```bash
ros2 launch px4_sitl_bringup px4.launch.py enable_kiss_icp:=false
```

## Inspection

```bash
ros2 topic list -t | grep -E '^/kiss|/x500/lidar/points'
ros2 topic echo /kiss/odometry --once
ros2 topic hz /kiss/odometry
```

The KISS process log is written to:

```text
/tmp/px4-kiss-icp.log
```

## Storage Behavior

KISS-ICP does not save a permanent map by default. Its local voxel map lives
in RAM, is published on `/kiss/local_map`, and disappears when KISS stops.

The current source, build, install, and build logs occupy about `20 MB` on the
external workspace. Ordinary ROS logs are small. Recording `/kiss/odometry`
is modest, but repeatedly recording `/kiss/local_map` can consume substantial
space because each message contains the current local map.

Generated PX4/KISS comparison plots are stored separately under:

```text
tools/odometry_comparison/generated/
```

## Verified Recording Result

The first 178.6-second mapping flight produced 1,787 KISS odometry samples.
After coordinate and starting-heading alignment, KISS agreed with PX4 at:

```text
3D position RMSE: 0.126 m
Final difference: 0.060 m
```

PX4 odometry is another estimator rather than Gazebo ground truth, so these
numbers measure agreement between estimators rather than absolute accuracy.
