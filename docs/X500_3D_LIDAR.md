# X500 3D LiDAR

This checkpoint adds a simulated 3D LiDAR to the PX4 X500 without modifying the
external PX4 checkout.

```text
PX4 X500 base model
        |
        v
fixed lidar_link
        |
        v
Gazebo gpu_lidar
        |
        +--> /x500/lidar
        +--> /x500/lidar/points (Gazebo)
                    |
                    v
              ros_gz_bridge
                    |
                    v
        /x500/lidar/points (ROS 2 PointCloud2)
```

## Sensor Configuration

| Setting | Value |
| --- | --- |
| Gazebo sensor type | `gpu_lidar` |
| Frame | `lidar_link` |
| Mount | Above `base_link` at `z=0.085 m` |
| Puck dimensions | 0.09 m diameter, 0.05 m height |
| Horizontal field of view | 360 degrees |
| Horizontal samples | 360 |
| Vertical field of view | 30 degrees |
| Vertical channels | 16 |
| Update rate | 10 Hz |
| Range | 0.3 m to 30 m |
| Gaussian noise standard deviation | 0.01 m |

These settings deliberately keep the first point cloud modest. They provide
useful 3D structure without asking Gazebo and the laptop to process an
unnecessarily dense cloud before the pipeline has been validated.

## Runtime Overlay

The project installs its X500 model under:

```text
src/px4_sitl_bringup/models/x500/model.sdf
```

At startup, `run_px4.sh` mounts this one file read-only over PX4's X500
`model.sdf` inside the container. The original file on the external workspace
is not edited or replaced.

## Gazebo-Side Test

Start the normal PX4 session:

```bash
./src/px4_sitl_bringup/scripts/run_px4.sh
```

In another terminal, inspect the sensor topics:

```bash
GZ_PARTITION=px4_sitl gz topic -l \
  | grep -E '^/x500/lidar($|/points$)'

GZ_PARTITION=px4_sitl gz topic -i -t /x500/lidar
GZ_PARTITION=px4_sitl gz topic -i -t /x500/lidar/points
```

Expected Gazebo message types are `gz.msgs.LaserScan` for `/x500/lidar` and
`gz.msgs.PointCloudPacked` for `/x500/lidar/points`.

## ROS 2 Point-Cloud Test

The normal PX4 launcher also starts a read-only `ros_gz_bridge` process. It
converts the Gazebo point cloud into ROS 2's standard 3D point-cloud message:

```text
Gazebo gz.msgs.PointCloudPacked
              |
              v
ROS 2 sensor_msgs/msg/PointCloud2
```

The launcher sets `GZ_PARTITION=px4_sitl` for both Gazebo and the bridge. A
Gazebo Transport partition is a discovery channel: processes must use the same
partition to exchange Gazebo topics across the Docker/host boundary.

In another terminal, source ROS 2 and this workspace:

```bash
source /opt/ros/lyrical/setup.bash
source /home/kelenna-udo/LIDAR_mapping_drone/install/setup.bash
```

Then inspect the ROS side:

```bash
ros2 topic list -t | grep lidar
ros2 topic info /x500/lidar/points --verbose
ros2 topic echo /x500/lidar/points --once --field header
ros2 topic hz /x500/lidar/points
```

Expected results are the type `sensor_msgs/msg/PointCloud2`, the frame
`lidar_link`, and an update rate near 10 Hz.

## RViz Sensor View

The normal PX4 launcher opens RViz with:

```text
Fixed Frame: world
Display:     PointCloud2
Topic:       /x500/lidar/points
QoS:         Best Effort
```

The project-owned `x500_tf_bridge` adapter reads Gazebo's dynamic pose message,
selects the `x500_0`, `base_link`, and `lidar_link` entities, and publishes:

```text
world -> base_link       dynamic transform on /tf
base_link -> lidar_link  fixed transform on /tf_static
```

The adapter calculates the fixed LiDAR mount from Gazebo's model poses. The
mount offset is therefore not duplicated in ROS configuration.

With `world` selected as RViz's Fixed Frame, the environment remains stationary
while the drone and LiDAR move through it. This is the geometry required before
testing LiDAR odometry.

Inspect the frame relationships while the simulation is running:

```bash
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

To launch RViz by itself while the simulation and bridge are already running:

```bash
./src/px4_sitl_bringup/scripts/run_lidar_rviz.sh
```

To run the complete simulation without automatically opening RViz:

```bash
START_RVIZ=0 ./src/px4_sitl_bringup/scripts/run_px4.sh
```

To temporarily disable the TF adapter for troubleshooting:

```bash
START_TF=0 ./src/px4_sitl_bringup/scripts/run_px4.sh
```
