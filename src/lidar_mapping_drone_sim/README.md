# LIDAR Mapping Drone Sim

First milestone: simulate a robot-mounted LiDAR in Gazebo, bridge the scan into ROS 2, and view `/laser_scan` in RViz 2.

## Build

Install the ROS/Gazebo runtime packages first:

```bash
sudo apt update
sudo apt install ros-lyrical-ros-gz ros-lyrical-rviz2 ros-lyrical-teleop-twist-keyboard
```

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch the LiDAR pipeline

```bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

Expected ROS 2 topic:

```bash
ros2 topic list
ros2 topic echo /laser_scan --once
```

## Record a scan bag

```bash
cd ~/LIDAR_mapping_drone
ros2 bag record -o bags/lidar_test_01 /laser_scan
```
