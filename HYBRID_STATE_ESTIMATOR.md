# Hybrid State Estimator

The flight controller uses one `EstimatedState`, but that state is assembled
from three measurement paths. The mental model is a temporary hybrid estimator:

```text
Gazebo pose /tf ----------------------> x, y, vx, vy ---------+
Gazebo IMU -> /x3_lidar/imu ----------> attitude, body rates -+-> EstimatedState
downward beam -> /x3_lidar/range/down -> z, filtered vz ------+      -> controller blocks

Gazebo pose /tf ----------------------> ground-truth comparison only for z/attitude
```

This is direct controller feedback, not a side-channel visualization. The
altitude, attitude, yaw, and position controller blocks consume the assembled
state without changes to their gains or interfaces.

## State Source Map

| Controller state | Active source | Processing |
| --- | --- | --- |
| `x_m`, `y_m` | `/tf` model pose | copied from `world -> x3_lidar` |
| `vx_mps`, `vy_mps` | `/tf` model pose | finite difference of consecutive positions |
| `roll_rad`, `pitch_rad`, `yaw_rad` | `/x3_lidar/imu` | IMU quaternion transformed into the model frame |
| roll, pitch, yaw rates | `/x3_lidar/imu` | IMU angular velocity transformed into the model frame |
| `z_m` | `/x3_lidar/range/down` | beam geometry corrected using IMU attitude and sensor mount TF |
| `vz_mps` | `/x3_lidar/range/down` | finite difference of corrected altitude, then low-pass filtered |

The implementation is in
`src/lidar_mapping_drone_control/lidar_mapping_drone_control/state_estimator.py`.
`FlightControllerNode` reads its output through the same `EstimatedState`
interface the controller blocks already used.

## Range Geometry

The Gazebo range sensor emits a beam along its local `+X` axis. Its fixed mount
rotates that beam downward. The estimator combines:

1. the IMU-derived drone orientation,
2. the fixed `x3_lidar -> x3_lidar/downward_range_link` transform,
3. the measured distance along the beam, and
4. the configured flat-ground height.

It reconstructs the model origin's world `z` rather than treating raw range as
altitude. This corrects for drone tilt and for the sensor's offset from the
model origin. It assumes the beam hits a flat horizontal surface at
`state_estimator.ground_z_m`; an obstacle under the drone is interpreted as the
current ground surface.

## Startup Phases

The estimator exposes one of three phases:

| Phase | Meaning | Altitude behavior |
| --- | --- | --- |
| `uninitialized` | Waiting for pose, IMU, range, or sensor mount TF | no controller state |
| `landed` | All sources exist, but range is inside the landed blind zone | `z = ground_z`, `vz = 0` |
| `airborne` | At least one valid range has been accepted | corrected range altitude |

The range sensor starts only about `0.013 m` above the ground but has a
`0.05 m` minimum range. The initial invalid reading is therefore expected. Once
a valid range moves the estimator to `airborne`, that phase remains latched;
later invalid range data does not reset altitude to zero.

## Vertical Velocity Filter

Raw vertical velocity is the difference between consecutive corrected altitude
samples divided by their sensor timestamps. A first-order low-pass filter then
reduces derivative noise:

```text
filtered_vz = alpha * raw_vz + (1 - alpha) * previous_filtered_vz
```

Larger `alpha` follows changes faster but passes more noise. Smaller `alpha` is
smoother but adds lag. The default is `0.25`; this milestone does not change the
existing altitude-controller gains.

## Freshness And Safety

The existing safety limiter receives the age of the oldest required source:

- TF pose receipt time,
- IMU receipt time,
- current range receipt while landed, or
- last valid range receipt while airborne.

If any required source becomes stale longer than the configured
`safety.state_timeout_s`, the existing safety limiter sends zero motor commands.
An invalid airborne range therefore becomes stale instead of silently reusing a
measurement forever.

## ROS Topics

Inputs:

| Topic | Type | Role |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | model x/y and fixed sensor mounts |
| `/x3_lidar/imu` | `sensor_msgs/msg/Imu` | orientation and angular velocity |
| `/x3_lidar/range/down` | `sensor_msgs/msg/Range` | downward beam distance |

Observability outputs:

| Topic | Type | Role |
| --- | --- | --- |
| `/flight_controller/estimated_state` | `nav_msgs/msg/Odometry` | exact state consumed by controller blocks |
| `/flight_controller/estimator_status` | `std_msgs/msg/String` | phase, range validity, and source age |

The Odometry topic is for inspection and recording. It does not feed back into
the controller.

## Parameters

Parameters are in
`src/lidar_mapping_drone_control/config/flight_controller.yaml`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `imu_topic` | `/x3_lidar/imu` | IMU input topic |
| `range_topic` | `/x3_lidar/range/down` | downward range input topic |
| `imu_link_frame` | `x3_lidar/imu_link` | fixed IMU frame |
| `range_link_frame` | `x3_lidar/downward_range_link` | fixed range frame |
| `state_estimator.ground_z_m` | `0.0` | assumed horizontal ground-plane height |
| `state_estimator.vertical_velocity_filter_alpha` | `0.25` | vertical derivative filter weight |

Topic and frame parameters should match the model and bridge configuration.
Change `ground_z_m` only when the simulated ground surface is at another world
height. Treat the filter alpha as controller tuning, not a casual display
setting.

## Run And Inspect

Start simulation and controller in separate terminals:

```bash
./src/run_lidar_mapping_drone.sh
```

```bash
./src/run_lidar_mapping_drone_control.sh \
  mode:=altitude_only \
  enable_keyboard:=false
```

Inspect estimator state and status:

```bash
ros2 topic echo /flight_controller/estimated_state --once
ros2 topic hz /flight_controller/estimated_state
ros2 topic echo /flight_controller/estimator_status
```

Record the telemetry and controller state:

```bash
./src/run_record_telemetry_bag.sh
```

Then compare telemetry, hybrid state, and TF ground truth:

```bash
python3 tools/plot_telemetry_vs_tf.py bags/TELEMETRY_BAG_DIRECTORY
```

## Current Boundaries

- This is not an EKF and does not integrate IMU acceleration into position.
- TF remains the temporary horizontal position source and ground truth.
- Horizontal velocity is an unfiltered TF finite difference.
- The vertical range model assumes a flat ground plane.
- Gazebo IMU orientation is available directly; a physical IMU would require
  bias handling, gravity treatment, calibration, and fusion.
- Body angular velocity is supplied directly to the controller rather than
  deriving Euler-angle rates.
