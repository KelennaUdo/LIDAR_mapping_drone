# PX4 Offboard Control

This package is a minimal ROS 2 position-control example for PX4 SITL.

```text
ROS 2 Offboard node
        |
        v
Micro XRCE-DDS Agent
        |
        v
PX4 position controller
        |
        v
Gazebo X500
```

## Behavior

The node publishes at 10 Hz. When `auto_start` is enabled, it:

1. streams an Offboard heartbeat and position setpoint for one second;
2. requests Offboard mode and arming;
3. targets `x=0`, `y=0`, and the configured altitude;
4. requests landing after `flight_duration_s`.

PX4 uses NED coordinates, where Z points down. A target altitude of 2 m is
therefore published as `z=-2`.

## Topics

| Topic | Message |
| --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |

## Build

```bash
source /opt/ros/lyrical/setup.bash
source /mnt/px4-workspace/px4_ros2_ws/install/setup.bash

cd /home/kelenna-udo/LIDAR_mapping_drone
colcon build --packages-select px4_offboard_control --symlink-install
```

## Default SITL Flight

The default enables automatic flight, targets 2 m, and requests landing after
15 seconds:

```bash
./src/px4_offboard_control/scripts/run_offboard_control.sh
```

## Monitor-Only Start

To start the node without sending flight commands, explicitly disable automatic
flight:

```bash
./src/px4_offboard_control/scripts/run_offboard_control.sh auto_start:=false
```

Pressing `Ctrl+C` stops the Offboard heartbeat. PX4 then applies its configured
Offboard-loss failsafe. This example is for simulation learning and does not
implement command acknowledgements, retries, or custom failure handling.
