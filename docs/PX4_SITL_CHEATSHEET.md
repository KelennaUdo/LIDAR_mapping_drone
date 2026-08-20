# PX4 SITL Command Cheat Sheet

## Mental Model

SITL means **Software In The Loop**. PX4 runs as a normal computer process
instead of on a physical flight controller.

```text
Project runner
    ↓
├── Micro XRCE-DDS Agent: PX4-to-ROS 2 communication
├── QGroundControl: host supervision application
└── Ubuntu 24.04 Docker container
    ├── PX4 SITL: flight-control software
    └── Gazebo X500: simulated vehicle and physical world
```

PX4 and Gazebo use the external source checkout:

```text
/mnt/px4-workspace/PX4-Autopilot
```

The current PX4 release is `v1.17.0`.

## Preflight Checks

Connect the workspace and display its state:

```bash
# Mount both storage layers and verify PX4.
./scripts/px4_workspace.sh connect

# Repeat the checks later without changing any mounts.
./scripts/px4_workspace.sh status
```

Successful output includes:

```text
Seagate:                 mounted read-write
Virtual PX4 drive:       mounted read-write
PX4 checkout:            found
PX4 container:           not running
DDS Agent container:     not running
Ready to launch PX4:     yes
```

Additional Docker checks:

```bash

# Confirm that Docker is running.
sudo systemctl is-active docker

# Confirm that the PX4 dependency image exists.
sudo docker image ls px4-sitl:v1.17.0

# Confirm that Docker has an NVIDIA runtime.
sudo docker info --format '{{json .Runtimes}}'

# Confirm that an old PX4 container is not still running.
sudo docker ps --filter name=px4-sitl
```

## Direct Startup

Run from the repository root:

```bash
# Start the Agent, QGroundControl, PX4 SITL, and the Gazebo X500.
./src/px4_sitl_bringup/scripts/run_px4.sh
```

The runner defaults to:

```text
PX4 source: /mnt/px4-workspace/PX4-Autopilot
Docker image: px4-sitl:v1.17.0
PX4 target: px4_sitl
Gazebo model: gz_x500
Gazebo world: default
GPU: NVIDIA runtime
```

The underlying PX4 command is:

```bash
make px4_sitl gz_x500
```

Do not run that host command directly from Ubuntu 26.04. The project runner
executes it inside the Ubuntu 24.04 container.

## Headless Startup

Headless mode runs without the Gazebo graphical window:

```bash
# Useful for automated tests or systems without a display.
HEADLESS=1 ./src/px4_sitl_bringup/scripts/run_px4.sh
```

The simulation server can still use graphics hardware for simulated rendering
sensors even when no window is shown.

## ROS Launch Startup

The ROS launch file calls the same Docker runner:

```bash
# Load ROS 2 and the built project workspace.
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# Start the same PX4/Gazebo workflow through ROS launch.
ros2 launch px4_sitl_bringup px4.launch.py
```

```bash
# Inspect available launch arguments without starting PX4.
ros2 launch px4_sitl_bringup px4.launch.py --show-args

# Start without a Gazebo GUI.
ros2 launch px4_sitl_bringup px4.launch.py headless:=1
```

## Optional Overrides

Defaults should be used for the current learning checkpoint:

```bash
# Example: select a different Gazebo world for one launch.
PX4_GZ_WORLD=example_world \
  ./src/px4_sitl_bringup/scripts/run_px4.sh

# Example: use a PX4 checkout stored somewhere else.
PX4_SOURCE_DIR=/another/path/PX4-Autopilot \
  ./src/px4_sitl_bringup/scripts/run_px4.sh
```

An override changes only that command's environment. It does not edit the
runner.

## Runtime Inspection

Use a second terminal while PX4 is running:

```bash
# Confirm that the PX4 container exists.
sudo docker ps --filter name=px4-sitl

# Confirm that the DDS Agent container exists.
sudo docker ps --filter name=px4-dds-agent

# Follow DDS Agent connection logs. Press Ctrl+C to stop watching.
sudo docker logs --follow px4-dds-agent

# Show CPU and memory usage.
sudo docker stats px4-sitl

# Confirm that the container can see the RTX 4050.
sudo docker exec px4-sitl nvidia-smi

# Watch host GPU activity. Press Ctrl+C to stop watching.
watch -n 1 nvidia-smi

# Open an additional shell inside the running PX4 container.
sudo docker exec -it px4-sitl bash
```

## Safe Shutdown

```bash
# Preferred: press Ctrl+C in the original launcher terminal.

# Stop both containers if necessary, unmount both filesystems, and eject.
./scripts/px4_workspace.sh disconnect
```

If the PX4 container remains, the helper asks before stopping it. It then
unmounts the virtual ext4 drive before ejecting the physical Seagate drive.
Unplug only after it prints that removal is safe.

## Rebuilding the Dependency Image

This is not part of normal startup:

```bash
# Rebuild only after changing docker/px4/Dockerfile or PX4 dependencies.
./docker/px4/build_image.sh
```

PX4 source and build output stay on the external workspace. Docker dependency
layers stay on the internal system drive.

## Current Milestone Boundary

Available now:

```text
PX4 v1.17.0 SITL
Gazebo X500
NVIDIA GPU access
Micro XRCE-DDS Agent v2.4.3
px4_msgs release/1.17
Read-only ROS 2 telemetry
Single shell and ROS launch workflow
```

Later checkpoints:

```text
ROS 2 Offboard control example
```

The video-based ARK `px4_offboard` package is not currently installed. Its
older Ubuntu, ROS, and PX4 assumptions should not be copied directly into this
environment.
