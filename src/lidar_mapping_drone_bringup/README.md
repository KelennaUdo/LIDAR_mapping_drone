# LIDAR Mapping Drone Bringup

This package owns launch files for the simulation.

The simulation assets live in `lidar_mapping_drone_sim`; this package starts the X3 LiDAR drone in Gazebo, bridges its scan, telemetry, and moving poses into ROS 2, publishes a lightweight RViz robot description, and opens RViz.

## Launch

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

Expected ROS 2 topics after launch:

```bash
ros2 topic list -t | grep -i -E "scan|imu|range|tf|robot_description"
```

The launch keeps the controller separate. The added IMU and downward range
topics are telemetry only; the controller still uses `/tf` for state feedback.
