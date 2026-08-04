# PX4 SITL

This branch is a focused learning environment for running PX4 SITL with the
PX4-supported Gazebo X500 vehicle. PX4 and Gazebo run inside an Ubuntu 24.04
Docker container while the computer continues to use Ubuntu 26.04.

The custom X3 controller sandbox is preserved on the
`feature/telemetry-sensors` branch. It is not part of this branch's runtime.

## Architecture

```text
ROS 2 telemetry                     Ubuntu 26.04 host
        |
        v
Micro XRCE-DDS Agent                Ubuntu 24.04 Docker container
        |
        v
PX4 SITL                            Ubuntu 24.04 Docker container
        |
        v
Gazebo X500                         Ubuntu 24.04 Docker container

QGroundControl                      Ubuntu 26.04 host, MAVLink supervision
```

## Repository Contents

| Path | Purpose |
| --- | --- |
| `docker/px4/` | Builds the Ubuntu 24.04 PX4 dependency image |
| `src/px4_sitl_bringup/` | ROS launch package and Docker runner |
| `PX4_SETUP.md` | Architecture, storage, startup, and cleanup guide |

PX4 source and build output are intentionally stored outside this repository:

```text
/mnt/px4-workspace/PX4-Autopilot
```

## Current Checkpoint

- Docker Engine is installed and verified.
- NVIDIA Container Toolkit `1.19.1` is installed and registered with Docker.
- Docker containers can access the NVIDIA RTX 4050.
- The Ubuntu 24.04 base image is available.
- A 30 GB ext4 workspace filesystem exists on the external drive.
- The workspace is mounted read-write at `/mnt/px4-workspace` and write-tested.
- PX4 `v1.17.0` and all 39 recursive submodules are checked out.
- Docker image `px4-sitl:v1.17.0` is built.
- PX4 SITL and the Gazebo X500 launch and fly under QGroundControl.
- Micro XRCE-DDS Agent `v2.4.3` is built.
- `px4_msgs release/1.17` is built for ROS 2 Lyrical.
- Read-only `/fmu/out/...` telemetry is verified in ROS 2.

See [PX4_SETUP.md](PX4_SETUP.md) before continuing. The setup proceeds through
small approval checkpoints so every installation and runtime step can be
inspected and understood.

## Run Commands

Connect the external workspace, then start the complete PX4 session:

```bash
./scripts/px4_workspace.sh connect
./src/px4_sitl_bringup/scripts/run_px4.sh
```

The launcher starts the DDS Agent, QGroundControl, PX4 SITL, and Gazebo. The
equivalent ROS 2 launch command is:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash

ros2 launch px4_sitl_bringup px4.launch.py
```

Both commands use the same shell supervisor and request the NVIDIA GPU for
Gazebo. Press `Ctrl+C` in the launch terminal to stop the complete session.

## Comparing With the X3 Sandbox

Commit or stash work before changing branches, then use:

```bash
git switch feature/telemetry-sensors
```

Return to the PX4 branch with:

```bash
git switch feature/px4-sitl
```
