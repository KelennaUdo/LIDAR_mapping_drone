# KISS-ICP and PX4 Odometry Comparison

This developer tool compares the position trajectory estimated by KISS-ICP
with the position trajectory estimated by PX4. It does not command the drone,
change either estimator, or tune KISS-ICP.

## Mental Model

```text
3D LiDAR scans -> KISS-ICP -> /kiss/odometry ---------+
                                                      +-> plots and error summary
PX4 estimator  -> DDS Agent -> /fmu/out/vehicle_odometry +
```

PX4 normally reports positions in North-East-Down (NED). The tool converts
them to East-North-Up (ENU), moves both trajectories to a common starting
origin, and rotates the KISS-ICP XY trajectory to match PX4 without changing
its scale.

## Run The Comparison

Connect the external PX4 workspace first:

```bash
cd /home/kelenna-udo/LIDAR_mapping_drone
./scripts/px4_workspace.sh connect
```

In terminal 1, start the comparison collector:

```bash
cd /home/kelenna-udo/LIDAR_mapping_drone
./tools/odometry_comparison/run_comparison.sh
```

In terminal 2, launch KISS-ICP and replay the recorded mapping flight:

```bash
source /opt/ros/lyrical/setup.bash
source /mnt/px4-workspace/px4_ros2_ws/install/setup.bash
source /mnt/px4-workspace/kiss_icp_ws/install/setup.bash

ros2 launch kiss_icp odometry.launch.py \
  topic:=/x500/lidar/points \
  bagfile:=/home/kelenna-udo/LIDAR_mapping_drone/bags/x500_mapping_loop_20260818_152906 \
  visualize:=true
```

After playback finishes, return to terminal 1 and press `Ctrl+C`. The collector
will save its results and print the output directory.

## Generated Results

Each test creates a timestamped directory under `generated/` containing:

```text
trajectory_xy.png
position_vs_time.png
position_error.png
trajectory_data.csv
summary.txt
```

Generated results are ignored by Git.

## Limitations

- PX4 odometry is another estimate, not simulation ground truth.
- This first tool compares position only, not attitude or velocity.
- KISS-ICP estimates the LiDAR pose while PX4 estimates the vehicle body pose.
- Samples are synchronized using shared receipt timing during bag playback.
- Horizontal alignment removes the arbitrary starting-heading difference but
  does not change trajectory scale.
