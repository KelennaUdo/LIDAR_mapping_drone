# Telemetry Sensors

This milestone adds simulated telemetry to the X3 LiDAR drone without changing
the flight controller. The controller can keep using `/tf` for state feedback;
the new topics are for inspection, logging, visualization, and future control
work.

## Mental Model

```text
Gazebo sensor plugin
  -> Gazebo Transport topic
  -> ros_gz_bridge
  -> ROS 2 telemetry topic
  -> RViz, rosbag, future estimators/controllers
```

The telemetry layer is additive. It does not command motors, tune gains, change
controller modes, or replace the existing planar LiDAR and TF feedback paths.

## Added Sensors

| Sensor | Gazebo sensor type | Gazebo topic | ROS 2 topic | ROS 2 type | Frame | Rate |
| --- | --- | --- | --- | --- | --- | --- |
| IMU | `imu` | `/x3_lidar/imu` | `/x3_lidar/imu` | `sensor_msgs/msg/Imu` | `x3_lidar/imu_link` | 100 Hz |
| Downward range | `gpu_lidar` | `/x3_lidar/range/down` | `/x3_lidar/range/down` | `sensor_msgs/msg/Range` | `x3_lidar/downward_range_link` | 30 Hz |

The downward range sensor is implemented as a narrow one-beam Gazebo
`gpu_lidar` because the installed `ros_gz_bridge` supports converting
`gz.msgs.LaserScan` into `sensor_msgs/msg/Range`. This keeps the Gazebo side
simple while exposing the ROS side as a range sensor.

### Landed Blind Zone

The sensor has a `0.05 m` minimum range. When the drone is resting on the
ground, the sensor origin is only about `0.013 m` above the ground plane, so
the ground is inside that minimum-range blind zone. In this landed condition,
the ROS bridge currently reports `range: 11.0` while `max_range` is `10.0`.
That value means there is no valid return; it does not mean the ground is
11 metres away.

Future estimator code must validate each reading against `min_range` and
`max_range`. During controller startup, an invalid close-range reading can be
handled as the expected landed state. After the first valid airborne reading,
an invalid or stale range must be treated as a sensor fault instead of being
interpreted as zero altitude.

## Why These Sensors

The IMU is the standard telemetry source for body angular velocity, linear
acceleration, and orientation-like inertial data in simulation. It is mounted
near the drone body frame because it represents body motion, not LiDAR-only
motion.

The downward range sensor is a simple height-to-ground telemetry source. It was
chosen instead of a barometer or altimeter because the project needs a concrete
ground-distance signal that is easy to inspect in Gazebo, RViz, and ROS bags.

Important limitation: the range value is measured along the sensor beam. When
the drone tilts, that raw range is not automatically vertical altitude. Future
control or mapping code must combine the range reading with TF or IMU
orientation to compute vertical height or a world-frame ground-hit point.

## Existing Paths Preserved

These paths should still work:

| Path | Gazebo topic | ROS 2 topic | Type |
| --- | --- | --- | --- |
| Planar LiDAR | `/lidar2` | `/laser_scan` | `sensor_msgs/msg/LaserScan` |
| Pose feedback | `/model/x3_lidar/pose` | `/tf` | `tf2_msgs/msg/TFMessage` |
| Motor command | `/X3/gazebo/command/motor_speed` | `/X3/gazebo/command/motor_speed` | `actuator_msgs/msg/Actuators` |

## RViz Visualization

RViz now opens with:

- `RobotModel` display using `/robot_description`
- `TF Frames` display
- planar `/laser_scan`
- downward `/x3_lidar/range/down`
- IMU display for `/x3_lidar/imu`

The RViz robot model uses vendored X3 mesh assets from the Open Robotics
Gazebo Fuel X3 UAV model and is rooted at the `x3_lidar` TF frame. Gazebo
remains the source of the real simulated drone motion. TF frame axes are
available in RViz, but frame-name labels are hidden by default to keep the
drone view readable.

## Inspect Gazebo

```bash
gz topic -l | grep -i -E "imu|range|scan|lidar"
gz topic -i -t /x3_lidar/imu
gz topic -i -t /x3_lidar/range/down
gz topic -i -t /lidar2
```

Expected Gazebo message types:

```text
/x3_lidar/imu          gz.msgs.IMU
/x3_lidar/range/down   gz.msgs.LaserScan
/lidar2                gz.msgs.LaserScan
```

## Inspect ROS 2

```bash
ros2 topic list -t | grep -i -E "imu|range|scan"
ros2 topic echo /x3_lidar/imu --once
ros2 topic hz /x3_lidar/imu
ros2 topic echo /x3_lidar/range/down --once
ros2 topic hz /x3_lidar/range/down
ros2 topic echo /laser_scan --once
```

Expected ROS 2 message types:

```text
/x3_lidar/imu          sensor_msgs/msg/Imu
/x3_lidar/range/down   sensor_msgs/msg/Range
/laser_scan            sensor_msgs/msg/LaserScan
```

Check TF and robot description:

```bash
ros2 topic echo /tf --once
ros2 topic echo /robot_description --once
ros2 run tf2_tools view_frames
```

## ROS Bag Workflow

Check that rosbag support is installed:

```bash
ros2 bag --help
```

If the command is missing on ROS 2 Lyrical, install:

```bash
sudo apt install ros-lyrical-ros2bag ros-lyrical-rosbag2 ros-lyrical-rosbag2-storage-default-plugins
```

Record a telemetry test bag:

```bash
ros2 bag record \
  -o bags/telemetry_sensors_test_01 \
  --topics \
  /tf \
  /tf_static \
  /laser_scan \
  /x3_lidar/imu \
  /x3_lidar/range/down
```

Inspect and play it back:

```bash
ros2 bag info bags/telemetry_sensors_test_01
ros2 bag play bags/telemetry_sensors_test_01
```

Recorded bag data is ignored by Git through `.gitignore`; `bags/.gitkeep`
keeps the directory in the repository.

## Runtime Graph

The system graph tool includes the telemetry bridge mappings:

```bash
python3 tools/generate_system_graph.py --view presentation
python3 tools/generate_system_graph.py --view debug
```

The presentation view should show:

```text
Gazebo /x3_lidar/imu
  -> ROS/Gazebo Bridge
  -> ROS /x3_lidar/imu

Gazebo /x3_lidar/range/down
  -> ROS/Gazebo Bridge
  -> ROS /x3_lidar/range/down
```

## Limitations

- The controller does not consume `/x3_lidar/imu` or `/x3_lidar/range/down`
  yet.
- The IMU uses Gazebo's native simulated IMU output; no estimator has been
  added yet.
- The downward range reading is along the beam direction, not vertical altitude
  when the drone tilts.
- The landed drone starts inside the range sensor's `0.05 m` blind zone, so the
  initial reading is intentionally invalid until the drone rises far enough.
- The RViz robot model is a visual aid based on the Gazebo Fuel X3 meshes. It
  is still not a physics model.
- The future 3D LiDAR mapping sensor is intentionally not part of this
  milestone.
