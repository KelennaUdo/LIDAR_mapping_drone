# LIDAR Mapping Drone Sim

This package provides an X3 quadcopter with a body-mounted planar LiDAR, the Gazebo test world, and an RViz configuration for `/laser_scan`.

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

The X3 motor plugins subscribe to the Gazebo topic below. Raw motor speeds do
not provide flight stabilization or position control.

```bash
gz topic -t /X3/gazebo/command/motor_speed \
  --msgtype gz.msgs.Actuators \
  -p 'velocity:[700, 700, 700, 700]'
```

## Record a scan bag

```bash
cd ~/LIDAR_mapping_drone
ros2 bag record -o bags/lidar_test_01 /laser_scan
```
