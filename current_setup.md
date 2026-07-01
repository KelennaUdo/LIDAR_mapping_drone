## 1. Big-Picture Architecture

Mental model: this sandbox is a small autopilot loop wrapped around the Gazebo X3 drone. Gazebo is the plant, TF is the feedback sensor, `FlightControllerNode` is the coordinator, individual Python classes are the control blocks, `MotorMixer` converts controller intent into rotor speeds, and `SafetyLimiter` is the final gate before commands leave ROS.

```text
reference commands
  YAML defaults / ros2 params / keyboard deltas
        |
        v
+--------------------------+
| /flight_controller node  |
| FlightControllerNode     |
+--------------------------+
        |
        | uses latest EstimatedState from /tf
        v
+--------------------------+
| Controller blocks        |
| - XY position -> tilt    |
| - roll/pitch -> torque   |
| - yaw -> torque          |
| - altitude -> thrust     |
+--------------------------+
        |
        v
+--------------------------+
| MotorMixer               |
| thrust + torques         |
| -> 4 rotor rad/s values  |
+--------------------------+
        |
        v
+--------------------------+
| SafetyLimiter            |
| clamp or zero motors     |
+--------------------------+
        |
        v
ROS /X3/gazebo/command/motor_speed
actuator_msgs/msg/Actuators
        |
        v
ros_gz_bridge ROS_TO_GZ
        |
        v
Gazebo /X3/gazebo/command/motor_speed
gz.msgs.Actuators
        |
        v
Gazebo X3 MulticopterMotorModel plugins
        |
        v
Drone motion / pose changes
        |
        v
Gazebo /model/x3_lidar/pose
        |
        v
ros_gz_bridge GZ_TO_ROS
        |
        v
ROS /tf
        |
        v
StateEstimator
        |
        +---- feedback to next controller cycle
```

Core composition lives in `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:64`. The Gazebo motor plugins are in `src/lidar_mapping_drone_sim/models/x3_lidar/model.sdf:70`. The motor bridge is `src/lidar_mapping_drone_control/config/motor_bridge.yaml:1`.

## 2. File-By-File Map

Generated `__pycache__` files are not source files, so I am excluding them.

| File | Purpose | Main inputs | Main outputs | Called by / communicates with |
| --- | --- | --- | --- | --- |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:1` | Core ROS node wiring all control blocks | YAML/params, `/tf`, manual deltas, e-stop | `/X3/gazebo/command/motor_speed` | Launch file starts it; calls all controller blocks |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/state_estimator.py:1` | Converts TF into `EstimatedState` | `tf2_msgs/TFMessage` from `/tf` | position, attitude, numerical velocities/rates | Constructed by `FlightControllerNode` |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/altitude_controller.py:1` | Altitude PID around hover thrust | target altitude, `EstimatedState`, `dt` | total thrust in newtons | Called by `_control_step()` |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/outer_loop_xy_position_controller.py:1` | Converts XY position error into pitch/roll references | target x/y, `EstimatedState` | `PitchRollReference` | Used in `position_hold` |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/inner_loop_pitch_roll_controller.py:1` | Tracks pitch/roll references | desired roll/pitch, state rates, `dt` | roll/pitch torque commands | Called every control cycle once state exists |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/yaw_controller.py:1` | Tracks yaw angle | target yaw, current yaw/rate, `dt` | yaw torque command | Called every control cycle once state exists |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/motor_mixer.py:1` | Allocates thrust/torques to 4 rotors | thrust, roll torque, pitch torque, yaw torque | `MotorCommand` rotor speeds | Called by `FlightControllerNode` |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/safety_limiter.py:1` | Final clamp/stop gate | motor command, state, state age, e-stop | safe or zeroed `MotorCommand` | Called before publishing |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/common.py:1` | Shared math and PID helper | scalar values, PID gains | clamped values, wrapped angles, PID output | Used by all control blocks |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/keyboard_control_node.py:1` | Manual terminal reference input | keypresses | `Twist` deltas, `Bool` e-stop | Publishes to controller topics |
| `src/lidar_mapping_drone_control/lidar_mapping_drone_control/__init__.py:1` | Python package marker | none | none | Import support |
| `src/lidar_mapping_drone_control/config/flight_controller.yaml:1` | Main controller configuration | static YAML values | ROS parameters | Loaded by launch file |
| `src/lidar_mapping_drone_control/config/motor_bridge.yaml:1` | ROS to Gazebo motor bridge config | ROS actuator topic | Gazebo actuator topic | Loaded by `ros_gz_bridge` |
| `src/lidar_mapping_drone_control/launch/flight_controller.launch.py:1` | Starts bridge, controller, optional keyboard | launch args mode, `enable_keyboard` | running ROS nodes | Called by `ros2 launch` or script |
| `src/lidar_mapping_drone_control/scripts/flight_controller_node:1` | Installed executable wrapper | none | runs Python `main()` | ROS launch executable |
| `src/lidar_mapping_drone_control/scripts/keyboard_control_node:1` | Installed keyboard wrapper | none | runs keyboard `main()` | ROS launch executable |
| `src/lidar_mapping_drone_control/CMakeLists.txt:1` | Package install rules | source files | installed package, scripts, config, launch | Used by `colcon build` |
| `src/lidar_mapping_drone_control/package.xml:1` | ROS package metadata/dependencies | dependency declarations | build/runtime metadata | Used by ROS tooling |

## 3. Runtime Data Flow

Mental model: each update cycle turns "where the drone is and where I want it to be" into "four rotor speeds," then safety decides whether those speeds may leave the node.

The actual loop is `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:300`.

1. State estimation

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/state_estimator.py:66` subscribes to `/tf`, filters for `tracked_child_frame`, currently `x3_lidar`, converts quaternion to roll/pitch/yaw, and numerically differentiates previous pose into velocities/rates.

2. References and mode

   `FlightControllerNode` stores references in `ReferenceState`: `x`, `y`, `altitude`, `roll`, `pitch`, `yaw`. It reads `controller_mode` each cycle.

3. Altitude controller

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/altitude_controller.py:35` computes altitude error, applies PID, adds gravity compensation as `mass * (gravity + acceleration_command)`, and clamps thrust.

4. XY position controller

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/outer_loop_xy_position_controller.py:40` only contributes in `position_hold`. It computes world-frame XY acceleration, rotates it into the drone body frame using yaw, then converts small accelerations into pitch/roll angle references.

5. Pitch/roll controller

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/inner_loop_pitch_roll_controller.py:41` compares reference roll/pitch against measured roll/pitch and outputs bounded torque commands.

6. Yaw controller

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/yaw_controller.py:31` compares yaw reference against measured yaw and outputs bounded yaw torque.

7. Motor mixer

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/motor_mixer.py:66` solves a 4x4 allocation system:

   ```text
   total thrust
   roll torque
   pitch torque
   yaw torque
      -> rotor_0, rotor_1, rotor_2, rotor_3 rad/s
   ```

8. Safety limiter

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/safety_limiter.py:42` either zeros motors for a hard stop condition or clamps each rotor speed to min/max.

9. Publish command

   `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:433` publishes `actuator_msgs/msg/Actuators.velocity[]`.

## 4. ROS 2 Communication Map

| Topic / endpoint | Message type | Publisher | Subscriber | ROS-native or bridged | Control role |
| --- | --- | --- | --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | `ros_gz_bridge` from Gazebo `/model/x3_lidar/pose` | `StateEstimator` inside `/flight_controller` | Bridged GZ_TO_ROS | Main pose feedback |
| `/X3/gazebo/command/motor_speed` | `actuator_msgs/msg/Actuators` | `/flight_controller` | `ros_gz_bridge` | ROS side of ROS_TO_GZ bridge | Motor command output |
| Gazebo `/X3/gazebo/command/motor_speed` | `gz.msgs.Actuators` | `ros_gz_bridge` | Gazebo X3 motor plugins | Gazebo native | Drives rotor velocity commands |
| `/flight_controller/manual_reference_delta` | `geometry_msgs/msg/Twist` | `/keyboard_control` or manual publisher | `/flight_controller` | ROS-native | Manual reference adjustments |
| `/flight_controller/emergency_stop` | `std_msgs/msg/Bool` | `/keyboard_control` or manual publisher | `/flight_controller` | ROS-native | E-stop set/clear |
| `/laser_scan` | `sensor_msgs/msg/LaserScan` | `ros_gz_bridge` from Gazebo `/lidar2` | RViz/user tools | Bridged GZ_TO_ROS | Mapping/LiDAR visualization, not used by controller |

Main ROS nodes/processes:

| Node/process | Purpose |
| --- | --- |
| `/flight_controller` | Main controller node |
| `/keyboard_control` | Optional terminal helper |
| `ros_gz_bridge parameter_bridge` from control launch | Bridges motor command ROS_TO_GZ |
| `ros_gz_bridge parameter_bridge` from bringup launch | Bridges LiDAR and pose/TF GZ_TO_ROS |
| `tf2_ros static_transform_publisher` | Publishes world -> lidar_robot_world alias |
| Gazebo `gz sim` process | Runs plant/drone physics, not a ROS node |

The bridge configs are `src/lidar_mapping_drone_bringup/config/bridge_lidar.yaml:1` and `src/lidar_mapping_drone_control/config/motor_bridge.yaml:1`.

## 5. Controller Modes

| Mode | Active blocks | Bypassed / held constant | References used | Expected Gazebo behavior |
| --- | --- | --- | --- | --- |
| `altitude_only` | state estimator, inner pitch/roll, yaw, altitude, mixer, safety | XY outer loop bypassed; pitch/roll reference forced to 0 | target altitude, target yaw default 0, level pitch/roll | Attempts to climb/hold altitude and stay level; XY drift is normal |
| `attitude_hold` | state estimator, pitch/roll, yaw, altitude, mixer, safety | XY outer loop bypassed | target altitude, target roll/pitch/yaw | Holds commanded attitude and altitude; no position correction |
| `position_hold` | all controller blocks | none conceptually | target x/y, altitude, yaw | Attempts cascaded position hold; oscillation/drift likely until tuned |
| `manual_keyboard` | state estimator, pitch/roll, yaw, altitude, mixer, safety | XY outer loop bypassed | keyboard-adjusted altitude, pitch, roll, yaw | Keys change references; drift is normal because no XY correction |

Important nuance: once state exists, altitude, yaw, pitch/roll, mixer, and safety all run every cycle. The mode mainly changes how the pitch/roll reference is chosen in `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:352`.

## 6. YAML / Configuration Map

Most parameters are declared in `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:216`, overridden by `src/lidar_mapping_drone_control/config/flight_controller.yaml:1`, and loaded by `src/lidar_mapping_drone_control/launch/flight_controller.launch.py:42`.

Runtime warning: `_handle_parameter_update()` only actively updates mode, references, and e-stop. Gains/mixer/safety objects are built at startup, so tune those through YAML and restart.

| Group | Parameters | Effect of increasing | Effect of decreasing | Casual tuning? |
| --- | --- | --- | --- | --- |
| Mode/timing | `controller_mode`, `control_rate_hz` | More Hz can reduce lag but increases CPU/noise sensitivity | Lower Hz increases lag | Mode yes; rate cautiously |
| Physical constants | `mass_kg`, `gravity_mps2` | More hover thrust estimate | Less hover thrust estimate | No, unless matching model |
| Topics/frames | `tf_topic`, `tracked_child_frame`, `motor_command_topic`, manual/e-stop topics | Changes wiring, not control math | Same | No, unless debugging integration |
| Targets | `target_altitude_m`, `target_x_m`, `target_y_m`, `target_yaw_deg`, `target_roll_deg`, `target_pitch_deg` | Commands higher/farther/larger attitude | Commands lower/closer/smaller attitude | Yes, within safety limits |
| XY gains | `xy_position_controller.kp`, `kd` | Stronger position correction/damping; too high can oscillate | Slower correction; more drift | Cautiously |
| XY limits | `max_accel_mps2`, `max_tilt_deg` | Allows stronger horizontal correction | Softer, safer correction | Cautiously |
| Pitch/roll gains | roll/pitch `kp`, `ki`, `kd` | Faster/stronger attitude response; too high can oscillate | Slower, softer attitude response | Cautiously |
| Pitch/roll limits | `integral_limit`, `torque_limit_nm` | More accumulated correction / torque authority | Less correction / authority | Torque cautiously; integral carefully |
| Yaw gains | `yaw_controller.kp`, `ki`, `kd` | Stronger yaw hold; can spin/oscillate if wrong sign | Weaker yaw hold | Cautiously |
| Yaw limit | `torque_limit_nm` | More yaw authority | Less yaw authority | Cautiously |
| Altitude gains | `altitude_controller.kp`, `ki`, `kd` | Stronger climb/hold; can bounce or saturate | Sluggish altitude response | Cautiously |
| Thrust limits | `min_thrust_n`, `max_thrust_n` | Wider thrust range | Narrower thrust range | No, safety-critical |
| Mixer constants | `motor_constant_n_per_radps2`, `moment_constant_m`, rotor geometry, spin signs | Changes physical allocation | Same | No, unless matching SDF |
| Motor limits | `min_motor_speed_rad_s`, `max_motor_speed_rad_s` | Higher max permits stronger commands | Lower max clamps earlier | No, safety-critical |
| Safety | `max_altitude_m`, `max_tilt_deg`, `state_timeout_s` | Less restrictive safety | More restrictive safety | Decrease for tests; increase cautiously |
| Keyboard | `altitude_step_m`, `attitude_step_deg`, `yaw_step_deg` | Bigger keypress changes | Finer keypress changes | Yes |

## 7. Brian Douglas Architecture Mapping

| Brian Douglas block | Implemented file/class | Current status | Notes |
| --- | --- | --- | --- |
| Altitude controller | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/altitude_controller.py:25` | Implemented, simplified | PID acceleration correction around hover thrust |
| Yaw controller | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/yaw_controller.py:21` | Implemented, simplified | Direct yaw angle to yaw torque |
| Outer-loop XY position controller | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/outer_loop_xy_position_controller.py:34` | Implemented for `position_hold` | PD position to small-angle pitch/roll |
| Inner-loop roll/pitch controller | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/inner_loop_pitch_roll_controller.py:29` | Implemented | PID angle to torque |
| Motor mixing algorithm | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/motor_mixer.py:54` | Implemented | 4x4 allocation, simple clamp, no priority-aware saturation |
| Plant/drone | `src/lidar_mapping_drone_sim/models/x3_lidar/model.sdf:70` | Gazebo simulation | X3 UAV motor plugins consume velocity commands |
| State estimation / feedback | `src/lidar_mapping_drone_control/lidar_mapping_drone_control/state_estimator.py:66` | Simplified | Uses Gazebo pose/TF and finite differencing, not IMU/EKF |

The architecture match is clear at the block level. The simplification is in sensor realism, filtering, allocation saturation, and tuning maturity.

## 8. Safety Behavior

Safety is implemented in `src/lidar_mapping_drone_control/lidar_mapping_drone_control/safety_limiter.py:67`.

| Condition | Behavior | How to test in sim |
| --- | --- | --- |
| Missing state | Publishes zero motors | Start controller before base sim/TF bridge |
| Stale state older than `state_timeout_s` | Publishes zero motors | Pause/stop Gazebo or TF bridge and watch warning |
| Emergency stop true | Publishes zero motors | Press `x` in keyboard mode or publish Bool true |
| Altitude above `safety.max_altitude_m` | Publishes zero motors | In sim, set target altitude above safety limit and watch cutoff |
| Tilt above `safety.max_tilt_deg` | Publishes zero motors | In sim, command large roll/pitch target in attitude mode |
| Motor speed outside min/max | Clamps each rotor speed | Echo motor topic and confirm values stay within configured bounds |

Safe e-stop commands:

```bash
ros2 topic pub --once /flight_controller/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /flight_controller/emergency_stop std_msgs/msg/Bool "{data: false}"
```

## 9. Why The Drone Drifts

Mental model: drift means the closed feedback loop is not yet strong, accurate, or complete enough to reject horizontal motion.

Expected first-controller limitations:

- `altitude_only` and `manual_keyboard` do not correct XY position.
- `attitude_hold` holds attitude, not world position.
- `position_hold` exists, but it is a simple PD outer loop with conservative tilt limits.
- Mixer saturation is simple clamping, so one axis can steal authority from another.

Possible sign convention issues:

- Outer loop assumes pitch moves body x and roll moves body y with current X3 signs.
- Mixer uses `rotor_y_m` for roll, `-rotor_x_m` for pitch, and `rotor_spin_sign` for yaw.
- If a command makes the error grow instead of shrink, suspect sign convention first.

Tuning/gain issues:

- Low XY gains allow drift before correction.
- High gains can oscillate or saturate motors.
- `ki` is currently zero for altitude, attitude, and yaw, so steady-state bias may remain.
- Incorrect `mass_kg` or motor constants shifts hover thrust.

State-estimation issues:

- Feedback is based on Gazebo pose/TF, not a filtered odometry source.
- Velocities and angular rates are finite differences, so they can be noisy or delayed.
- `PosePublisher` is 30 Hz while the controller runs at 50 Hz.
- The tracked frame is `x3_lidar`; if that frame is not the exact control body frame you expect, attitude/position interpretation can be offset.

Evidence to collect before changing code:

```bash
ros2 topic hz /tf
ros2 topic hz /X3/gazebo/command/motor_speed
ros2 topic echo /tf --once
ros2 topic echo /X3/gazebo/command/motor_speed --once
ros2 param get /flight_controller controller_mode
ros2 param dump /flight_controller
```

## 10. Suggested Inspection Commands

Nodes:

```bash
ros2 node list
ros2 node info /flight_controller
ros2 node info /keyboard_control
```

Topics:

```bash
ros2 topic list -t
ros2 topic info /tf --verbose
ros2 topic info /X3/gazebo/command/motor_speed --verbose
ros2 topic info /flight_controller/manual_reference_delta --verbose
ros2 topic info /flight_controller/emergency_stop --verbose
```

Live data:

```bash
ros2 topic echo /tf --once
ros2 topic echo /X3/gazebo/command/motor_speed --once
ros2 topic hz /tf
ros2 topic hz /X3/gazebo/command/motor_speed
```

Params:

```bash
ros2 param list /flight_controller
ros2 param get /flight_controller controller_mode
ros2 param get /flight_controller target_altitude_m
ros2 param dump /flight_controller
```

Logs / debug:

```bash
ros2 topic echo /rosout
```

TF:

```bash
ros2 run tf2_ros tf2_echo lidar_robot_world x3_lidar
ros2 run tf2_tools view_frames
```

Gazebo topics:

```bash
gz topic -l
gz topic -i -t /X3/gazebo/command/motor_speed
gz topic -e -t /X3/gazebo/command/motor_speed
gz topic -e -t /model/x3_lidar/pose
```

Rosbag:

```bash
ros2 bag record /tf /X3/gazebo/command/motor_speed /flight_controller/manual_reference_delta /flight_controller/emergency_stop /rosout
```

There is no dedicated controller debug topic right now. The available debug surface is terminal logs, `/rosout`, parameters, `/tf`, and motor command echoing.

## 11. Minimal Learning Path

1. `README.md:76`

   Question: what block architecture is this trying to implement?

2. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/flight_controller_node.py:64`

   Question: how are all blocks wired together each cycle?

3. `src/lidar_mapping_drone_control/config/flight_controller.yaml:1`

   Question: what values define the controller's behavior?

4. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/state_estimator.py:66`

   Question: what does the controller believe the drone state is?

5. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/altitude_controller.py:25`, `src/lidar_mapping_drone_control/lidar_mapping_drone_control/inner_loop_pitch_roll_controller.py:29`, `src/lidar_mapping_drone_control/lidar_mapping_drone_control/yaw_controller.py:21`

   Question: how do references become thrust/torques?

6. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/outer_loop_xy_position_controller.py:34`

   Question: how does position error become tilt?

7. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/motor_mixer.py:54`

   Question: how do thrust/torques become rotor speeds?

8. `src/lidar_mapping_drone_control/lidar_mapping_drone_control/safety_limiter.py:36`

   Question: when are commands blocked?

9. `src/lidar_mapping_drone_control/launch/flight_controller.launch.py:9` and `src/lidar_mapping_drone_control/config/motor_bridge.yaml:1`

   Question: how does ROS connect to Gazebo?

## 12. Optional CONTROLLER_MAP.md Outline

I would not create this unless you explicitly approve it. A concise outline could be:

```markdown
# Controller Map

## Mental Model

- Plant, feedback, controller, mixer, safety gate

## Runtime Diagram

- ASCII signal flow

## ROS/Gazebo Interfaces

- Nodes
- Topics
- Bridges
- Message types

## Controller Blocks

- StateEstimator
- AltitudeController
- OuterLoopXYPositionController
- InnerLoopPitchRollController
- YawController
- MotorMixer
- SafetyLimiter

## Controller Modes

- altitude_only
- attitude_hold
- position_hold
- manual_keyboard

## Configuration Guide

- References
- Gains
- Mixer constants
- Safety limits
- Runtime vs restart-required params

## Safety Testing Checklist

## Drift Diagnosis Checklist

## Reading Path
```
