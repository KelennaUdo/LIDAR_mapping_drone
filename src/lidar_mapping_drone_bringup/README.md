# LIDAR Mapping Drone Bringup

This package owns launch files for the simulation.

The simulation assets live in `lidar_mapping_drone_sim`; this package starts those assets with Gazebo, the ROS/Gazebo bridge, TF, and RViz.

## Launch

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```
