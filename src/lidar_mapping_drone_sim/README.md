# LIDAR Mapping Drone Sim

This package provides an X3 quadcopter with a body-mounted planar LiDAR, simulated IMU telemetry, a downward range sensor, the Gazebo test world, and an RViz configuration for the drone model and sensor topics.

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

## Launch the simulation pipeline

```bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

Expected ROS 2 topic:

```bash
ros2 topic list
ros2 topic echo /laser_scan --once
ros2 topic echo /x3_lidar/imu --once
ros2 topic echo /x3_lidar/range/down --once
```

Telemetry topics:

| Sensor | Gazebo topic | ROS 2 topic | ROS 2 type | Rate |
| --- | --- | --- | --- | --- |
| Planar LiDAR | `/lidar2` | `/laser_scan` | `sensor_msgs/msg/LaserScan` | 10 Hz |
| IMU | `/x3_lidar/imu` | `/x3_lidar/imu` | `sensor_msgs/msg/Imu` | 100 Hz |
| Downward range | `/x3_lidar/range/down` | `/x3_lidar/range/down` | `sensor_msgs/msg/Range` | 30 Hz |

The downward range value is measured along the sensor beam. It is not
automatically vertical altitude when the drone tilts.

The X3 motor plugins subscribe to the Gazebo topic below. Raw motor speeds do
not provide flight stabilization or position control.

```bash
gz topic -t /X3/gazebo/command/motor_speed \
  --msgtype gz.msgs.Actuators \
  -p 'velocity:[700, 700, 700, 700]'
```

## Record a telemetry bag

```bash
cd ~/LIDAR_mapping_drone
ros2 bag record \
  -o bags/telemetry_sensors_test_01 \
  --topics \
  /tf \
  /tf_static \
  /laser_scan \
  /x3_lidar/imu \
  /x3_lidar/range/down \
  /flight_controller/estimated_state \
  /flight_controller/estimator_status
```

See `TELEMETRY_SENSORS.md` in the repository root for the full telemetry map.
