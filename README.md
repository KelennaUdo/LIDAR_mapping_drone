# LIDAR_mapping_drone

ROS 2 Lyrical and Gazebo simulation for a robot-mounted planar LiDAR. The current pipeline launches Gazebo, bridges the simulated scan into ROS 2, publishes the required static transform, and visualizes the scan in RViz.

## Packages

- `lidar_mapping_drone_sim`: Gazebo model, world, and RViz assets.
- `lidar_mapping_drone_bringup`: Runtime configuration and the combined launch file.

## Build

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

Alternatively:

```bash
./src/run_lidar_mapping_drone.sh
```
