# LIDAR_mapping_drone

ROS 2 Lyrical and Gazebo simulation for an X3 quadcopter with a body-mounted planar LiDAR. The pipeline launches Gazebo, bridges the simulated scan and moving poses into ROS 2, and visualizes the scan in RViz.

## Packages

- `lidar_mapping_drone_sim`: Gazebo model, world, and RViz assets.
- `lidar_mapping_drone_bringup`: Runtime configuration and the combined launch file.
- `lidar_mapping_drone_control`: Experimental block-structured flight controller for the X3 LiDAR drone.

## Build

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Base LiDAR simulation only:

```bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

Alternatively:

```bash
./src/run_lidar_mapping_drone.sh
```

Controller and keyboard, in a second terminal after the base simulation is running:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
./src/run_lidar_mapping_drone_control.sh
```

The controller script defaults to `manual_keyboard`. It starts the controller
launch in the background and runs the keyboard node in the foreground so stdin
works correctly.

Launch file equivalent for non-interactive controller testing:

```bash
ros2 launch lidar_mapping_drone_control flight_controller.launch.py
```

Non-keyboard modes:

```bash
./src/run_lidar_mapping_drone_control.sh mode:=altitude_only enable_keyboard:=false
```

Keyboard commands:

```text
r/f  altitude reference up/down
w/s  pitch reference forward/back
a/d  roll reference left/right
q/e  yaw reference left/right
x    emergency stop
c    clear emergency stop
h    show help
```

Available controller modes:

- `altitude_only`: altitude loop active, roll/pitch/yaw references held level.
- `attitude_hold`: altitude plus roll/pitch/yaw loops active.
- `position_hold`: full cascaded XY position -> pitch/roll -> motor mixer architecture.
- `manual_keyboard`: terminal keys modify altitude, pitch, roll, and yaw references while the same controller blocks remain in the command path.

## Flight Control Architecture

The controller follows the block-level hover architecture described by Brian Douglas in MathWorks, "How Do You Get a Drone to Hover? | Drone Simulation and Control, Part 2" (published 12 Oct 2018): https://www.mathworks.com/videos/drone-simulation-and-control-part-2-how-do-you-get-a-drone-to-hover--1539323448303.html

Implemented signal flow:

```text
position_reference
  -> OuterLoopXYPositionController
  -> reference_pitch_roll_angle

reference_pitch_roll_angle + estimated_states
  -> InnerLoopPitchRollController
  -> pitch_torque_command, roll_torque_command

yaw_reference + estimated_states
  -> YawController
  -> yaw_torque_command

altitude_reference + estimated_states
  -> AltitudeController
  -> thrust_command

thrust_command + roll_torque_command + pitch_torque_command + yaw_torque_command
  -> MotorMixer
  -> four rotor velocity commands
  -> Gazebo /X3/gazebo/command/motor_speed
```

This is an educational controller scaffold, not a tuned autopilot. Start with
`altitude_only`, keep the configured safety limits conservative, and tune gains
through `src/lidar_mapping_drone_control/config/flight_controller.yaml`.
