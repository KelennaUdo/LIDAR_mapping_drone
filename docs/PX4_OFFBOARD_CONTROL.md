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

## Keyboard Teleoperation

The separate `offboard_teleop` node keeps a position target and changes that
target one step at a time. When no key is pressed, the target does not change,
so PX4 continues holding the last commanded point.

```text
keyboard -> position target -> PX4 position controller -> Gazebo X500
```

Build the package after adding or changing either executable:

```bash
source /opt/ros/lyrical/setup.bash
source /mnt/px4-workspace/px4_ros2_ws/install/setup.bash

cd /home/kelenna-udo/LIDAR_mapping_drone
colcon build --packages-select px4_offboard_control --symlink-install
```

Start the complete PX4 simulation and wait for QGroundControl to show `Ready`.
In a second terminal, run:

```bash
./src/px4_offboard_control/scripts/run_offboard_teleop.sh
```

The keys are:

| Key | Result |
| --- | --- |
| `T` | Request Offboard mode, arm, and take off |
| `W` / `S` | Move the target forward / backward |
| `A` / `D` | Move the target left / right |
| `R` / `F` | Move the target up / down |
| `Q` / `E` | Turn the target yaw left / right |
| `L` | Ask PX4 to land |
| `Shift+X` | Force-disarm immediately; simulation emergency only |
| `H` | Print the controls again |

Forward, backward, left, and right are relative to the commanded yaw. The
default horizontal step is 0.5 m, so this is deliberate position teleoperation
rather than continuous velocity control.

Optional ROS parameters use standard `ros2 run` syntax:

```bash
./src/px4_offboard_control/scripts/run_offboard_teleop.sh \
  -p takeoff_altitude_m:=2.0 \
  -p movement_step_m:=0.25 \
  -p altitude_step_m:=0.25 \
  -p yaw_step_deg:=10.0
```

A companion ROS launch file is also installed as
`offboard_teleop.launch.py`. The shell runner is preferred for keyboard use
because it gives the executable direct ownership of the active terminal.
