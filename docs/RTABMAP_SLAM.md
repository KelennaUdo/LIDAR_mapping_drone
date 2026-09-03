# RTAB-Map LiDAR SLAM

The `slam` bringup mode runs the complete live mapping pipeline with one public
launch file and one public runner.

## Mental Model

KISS-ICP and RTAB-Map both hold point-cloud data, but their maps have different
jobs:

```text
live 3D LiDAR scans
          |
          v
      KISS-ICP                 local geometry for motion estimation
          |
          v
   /kiss/odometry
          |
          v
      RTAB-Map                 global graph, persistent map, and database
          |
          v
 map -> odom_lidar -> lidar_link
```

KISS-ICP's local map is temporary working memory used to align the next scan.
RTAB-Map stores observations from the whole session, relates them through a pose
graph, and can publish a 3D occupancy map for planning.

A **loop closure** is a graph constraint created when SLAM recognizes a
previously observed place. It allows RTAB-Map to correct drift accumulated
along the trajectory.

## Bringup Modes

The public runner accepts one mode:

```bash
./src/px4_sitl_bringup/scripts/run_px4.sh simulation
./src/px4_sitl_bringup/scripts/run_px4.sh odometry
./src/px4_sitl_bringup/scripts/run_px4.sh slam
```

| Mode | Additional perception components |
| --- | --- |
| `simulation` | Raw LiDAR visualization and Gazebo ground-truth TF |
| `odometry` | KISS-ICP local map and LiDAR odometry |
| `slam` | KISS-ICP, RTAB-Map, SLAM RViz, and a persistent database |

The equivalent ROS launch entry point is:

```bash
ros2 launch px4_sitl_bringup px4.launch.py mode:=slam
```

## Files

| File | Purpose |
| --- | --- |
| `config/rtabmap.yaml` | LiDAR-only RTAB-Map parameters |
| `launch/px4.launch.py` | Declares paths, options, and the bringup mode |
| `rviz/x500_slam.rviz` | Displays the map cloud, graph, odometry, and TF |
| `scripts/run_px4.sh` | Starts and supervises the selected mode |

All paths above are inside `src/px4_sitl_bringup/`.

## Clock Requirement

KISS-ICP, RTAB-Map, TF, and RViz must agree about simulation time. The
Gazebo-to-ROS bridge publishes Gazebo's clock as ROS 2 `/clock`, and all SLAM
nodes use that clock.

```text
Gazebo /clock + Gazebo LiDAR
              |
              v
       ROS 2 clock + PointCloud2
              |
              v
       KISS-ICP + RTAB-Map
```

## Build

From the repository root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select px4_sitl_bringup --symlink-install
```

Successful output ends with:

```text
Summary: 1 package finished
```

## Run Live SLAM

Connect the external workspace, then start `slam` mode:

```bash
./scripts/px4_workspace.sh connect
./src/px4_sitl_bringup/scripts/run_px4.sh slam
```

The launcher creates a timestamped database under:

```text
/mnt/px4-workspace/rtabmap_maps/
```

Override that path when a named experiment is useful:

```bash
RTABMAP_DATABASE_PATH=/mnt/px4-workspace/rtabmap_maps/x500_live_test.db \
  ./src/px4_sitl_bringup/scripts/run_px4.sh slam
```

In a second terminal, start keyboard control:

```bash
./src/px4_offboard_control/scripts/run_offboard_teleop.sh
```

Wait briefly for the LiDAR pipeline to initialize, fly slowly around the arena,
return near the starting area, and land. The RViz map should grow while the
flight is happening.

Stop the teleop program first. Then press `Ctrl+C` in the bringup terminal.
The supervisor stops RTAB-Map first so it can save the database before Gazebo
and the LiDAR bridge stop.

## Runtime Outputs

| Topic or transform | Meaning |
| --- | --- |
| `/x500/lidar/points` | Live 3D LiDAR scans |
| `/kiss/odometry` | KISS-ICP motion estimate |
| `/kiss/local_map` | Temporary local geometry used by KISS-ICP |
| `/rtabmap/mapData` | RTAB-Map graph and map observations |
| `/rtabmap/octomap_binary` | 3D free/occupied/unknown map for planning |
| `map -> odom_lidar -> lidar_link` | Corrected SLAM pose chain |

## Inspect a Saved Result

Open a saved database:

```bash
rtabmap-databaseViewer /mnt/px4-workspace/rtabmap_maps/<database>.db
```

When asked whether to use the database parameters, choose **Yes**. To assemble
the LiDAR observations:

1. Select **Edit -> View 3D map**.
2. Clear **From RGB-D images**, because this project uses LiDAR scans.
3. Keep **Assemble clouds** and **Regenerate clouds** enabled.
4. Leave **Meshing** disabled for the first inspection.
5. Select **OK**.

The viewer title `Clouds (1 nodes)` means the selected scans were assembled
into one displayed cloud object. It does not mean that the database contains
only one graph node.

## Current Status

Offline bag playback has already produced recognizable 3D reconstructions of
the mapping arena. The next verification is a complete live flight in `slam`
mode. SLAM tuning should only continue if a problem blocks live mapping or the
future navigation map.
