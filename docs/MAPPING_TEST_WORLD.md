# Mapping Test World

`mapping_test.sdf` is a compact Gazebo environment for the future 3D LiDAR and
SLAM pipeline. It does not modify PX4 or the X500 vehicle.

```text
PX4 and X500
     |
     v
mapping_test world
     |
     v
future 3D LiDAR observations
```

## Geometry

The 30 m by 30 m environment contains:

- a clear takeoff and landing area at the origin;
- perimeter walls;
- an interior room with a doorway;
- an open alcove;
- asymmetric pillars, boxes, and a low barrier;
- a clear loop route for future mapping flights.

Every physical obstacle has matching collision and visual geometry. The landing
marker is visual only and does not affect the vehicle.

## Start Through the Shell Launcher

The mapping world is the project default, so no world argument is needed:

```bash
./src/px4_sitl_bringup/scripts/run_px4.sh
```

## Start Through ROS 2 Launch

Build the bringup package after adding or changing installed world files:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select px4_sitl_bringup --symlink-install
source install/setup.bash

ros2 launch px4_sitl_bringup px4.launch.py
```

The launcher mounts the selected project world read-only at the location PX4
expects inside its Docker container. The external PX4 checkout is not changed.

Press `Ctrl+C` in the launcher terminal to stop PX4, Gazebo, the DDS Agent, and
QGroundControl when QGroundControl was started by that launcher. Closing only
the Gazebo window does not stop the complete simulation session.

Use the original empty PX4 world by explicitly setting:

```bash
PX4_GZ_WORLD=default \
  ./src/px4_sitl_bringup/scripts/run_px4.sh
```

## Current Scope

This checkpoint supplies static geometry only. It does not add LiDAR, SLAM,
teleoperation, moving objects, or changes to PX4 flight control.
