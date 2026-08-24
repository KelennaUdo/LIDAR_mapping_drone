# RTAB-Map LiDAR SLAM

This checkpoint adds offline graph-based SLAM to the X500 mapping pipeline.
It replays a recorded 3D LiDAR flight, estimates motion with KISS-ICP, and
stores RTAB-Map's graph and point-cloud observations in a persistent database.

## Mental Model

The two mapping tools have different jobs:

```text
recorded 3D LiDAR scans
          |
          v
      KISS-ICP                 relative motion between scans
          |
          v
   /kiss/odometry
          |
          v
      RTAB-Map                 graph, constraints, optimization, database
          |
          v
 map -> odom_lidar -> lidar_link
```

KISS-ICP answers, "How did the LiDAR move since the previous scans?"
RTAB-Map answers, "How do all observed places and motion constraints fit
together, and have we returned to a place seen before?"

A **loop closure** is a constraint created when SLAM recognizes that the
vehicle has returned to a previously observed place. It lets the graph
correct drift accumulated while travelling around the loop.

## Files

| File | Purpose |
| --- | --- |
| `config/rtabmap_slam.yaml` | LiDAR-only RTAB-Map parameters |
| `launch/rtabmap_slam.launch.py` | Starts the clock adapter, KISS-ICP, RTAB-Map, and RViz |
| `rviz/x500_slam.rviz` | Displays the map cloud, graph, odometry, and TF |
| `scripts/run_rtabmap_slam.sh` | Checks paths, sources workspaces, and starts the launch file |
| `src/pointcloud_clock_adapter.cpp` | Reconstructs simulation time from old point-cloud stamps |

All paths above are inside `src/px4_sitl_bringup/`.

## Why the Clock Adapter Exists

The first X500 mapping bag contains Gazebo timestamps in the point-cloud
headers, but it does not contain a `/clock` topic. Normal rosbag playback uses
the bag's much larger wall-clock timestamps for its generated clock. TF then
sees the cloud and transforms as if they came from different timelines and
reports `TF_OLD_DATA` or extrapolation errors.

The adapter keeps every point cloud unchanged while doing this in order:

1. receive `/x500/lidar/points_recorded`;
2. publish its header timestamp on `/clock`;
3. republish the cloud as `/x500/lidar/points`.

KISS-ICP, RTAB-Map, and RViz all use simulation time, so the complete offline
pipeline now agrees about what time it is. This adapter is a compatibility
tool for recordings without `/clock`; future bags should record `/clock`
directly.

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

## Run an Offline Mapping Test

Connect the external workspace first:

```bash
./scripts/px4_workspace.sh connect
```

Start the SLAM pipeline in terminal 1. Choose a new database name when the
recording is a new experiment:

```bash
RTABMAP_DATABASE_PATH=/mnt/px4-workspace/rtabmap_maps/x500_test_02.db \
  ./src/px4_sitl_bringup/scripts/run_rtabmap_slam.sh
```

In terminal 2, replay only the recorded cloud and remap it to the adapter's
input. Do not pass `--clock` for this older bag:

```bash
source /opt/ros/lyrical/setup.bash

ros2 bag play \
  bags/x500_mapping_loop_20260818_152906 \
  --topics /x500/lidar/points \
  --remap /x500/lidar/points:=/x500/lidar/points_recorded
```

After playback finishes, press `Ctrl+C` in terminal 1. RTAB-Map saves the
database before exiting.

## Inspect the Result

Open the saved database:

```bash
rtabmap-databaseViewer \
  /mnt/px4-workspace/rtabmap_maps/x500_test_02.db
```

When asked whether to use the database parameters, choose **Yes**. To assemble
the recorded LiDAR observations:

1. select **Edit -> View 3D map**;
2. clear **From RGB-D images**, because this project uses LiDAR scans;
3. keep **Assemble clouds** and **Regenerate clouds** enabled;
4. leave **Meshing** disabled for the first inspection;
5. select **OK**.

The viewer title `Clouds (1 nodes)` means the selected scans were assembled
into one displayed cloud object. It does not mean that the RTAB-Map database
contains only one graph node.

## Verified First Result

The `x500_test_02.db` run produced a recognizable 3D reconstruction of the
mapping arena and saved a database of about 13 MB.

| Measurement | Result |
| --- | ---: |
| Stored positive node IDs | 179 |
| Connected graph nodes | 60 |
| Sequential neighbor constraints | 59 |
| Local-space closure constraints | 1 |
| Global loop closures | 0 |
| Connected graph components | 1 |
| Timing errors after adapter fix | 0 |

The one local-space closure connects nodes 134 and 149, whose scan timestamps
are 296 s and 311 s. This is useful evidence that geometric proximity matching
works, but the 15-second separation makes it a short local revisit rather than
a deliberate return-to-start loop closure.

## Current Limitations

- This first bag did not record `/clock`, so offline replay needs the adapter.
- The flight was not designed specifically to test a long loop closure.
- LiDAR-only operation does not use camera appearance for place recognition.
- Most graph structure still follows KISS-ICP odometry and sequential links.
- RTAB-Map settings have not been tuned against a controlled loop flight yet.

The next experiment should improve the input evidence before changing the
SLAM parameters: record `/clock`, fly a slow loop, return to the same pose and
yaw, hover there, and then inspect whether RTAB-Map creates a temporally distant
closure.
