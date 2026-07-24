# PX4 SITL

## Purpose

This branch provides a focused PX4 SITL environment using the supported Gazebo
X500. The custom-controller X3 sandbox is preserved separately on the
`feature/telemetry-sensors` branch.

## Architecture

```text
ROS 2 Offboard node                 later checkpoint
        |
        v
Micro XRCE-DDS Agent                later checkpoint
        |
        v UDP 8888
PX4 SITL                            Docker, Ubuntu 24.04
        |
        v Gazebo Transport
Gazebo Harmonic X500                Docker, Ubuntu 24.04
```

Docker is the isolation boundary. The container uses Ubuntu 24.04 packages,
but it shares the Ubuntu 26.04 host kernel. `--network host` lets PX4 use its
normal UDP ports without a large port-mapping list.

## Current Status

| Item | Status |
| --- | --- |
| Docker Engine on Ubuntu 26.04 | Verified, version 29.6.2 |
| NVIDIA Container Toolkit | Verified, version 1.19.1 |
| NVIDIA GPU access from Docker | Verified, RTX 4050 visible |
| Ubuntu 24.04 base image | Verified |
| PX4 branch | `feature/px4-sitl` |
| PX4 release selected | Stable `v1.17.0` |
| External ext4 workspace | Mounted read-write at `/mnt/px4-workspace` and write-tested |
| PX4 source checkout | `v1.17.0`, 39 recursive submodules verified |
| PX4 dependency image | `px4-sitl:v1.17.0` built |
| X500 SITL flight | Not tested yet |
| ROS 2 communication | Later checkpoint |
| ROS 2 Offboard example | Later checkpoint |

## Why This Uses a Source Checkout

PX4 documents a prebuilt `px4io/px4-sitl-gazebo` container, but the currently
published versioned tags begin with PX4 1.18 prereleases. A stable `v1.17.0`
Gazebo container or `px4-gazebo` package was not available when this scaffold
was created.

This environment therefore pins PX4 source to stable `v1.17.0`. The Docker image
uses PX4's own `Tools/setup/ubuntu.sh --no-nuttx` dependency installer:

- common PX4 build tools are installed;
- Gazebo simulation dependencies are installed;
- NuttX hardware cross-compilers are omitted because SITL does not use them;
- PX4 source is not copied into the image.

## Storage Model

```text
Internal system drive
  /var/lib/docker/
    Ubuntu 24.04 layers
    PX4 SITL dependency image

External drive
  PX4_LINUX_WORKSPACE.img
    mounted at /mnt/px4-workspace/
      PX4-Autopilot/
        source
        submodules
        build/px4_sitl_default/
```

The dependency image will consume several gigabytes internally. The larger,
frequently changing source and build output stays on the external drive. The
drive must be mounted before building or running PX4 and must remain connected
while the container is running.

## Files

| File | Role |
| --- | --- |
| `docker/px4/Dockerfile` | Builds the Ubuntu 24.04 SITL dependency image |
| `docker/px4/build_image.sh` | Validates the PX4 tag and builds the image |
| `src/px4_sitl_bringup/scripts/run_px4_sitl.sh` | Direct interactive startup command |
| `src/px4_sitl_bringup/` | ROS package containing launch support |
| `px4_sitl.launch.py` | Equivalent ROS 2 launch entry point |

## External-Drive Workspace

The 30 GB ext4 filesystem image exists at:

```text
/run/media/kelenna-udo/Seagate Backup Plus drive/PX4_LINUX_WORKSPACE.img
```

It is currently mounted at:

```text
/mnt/px4-workspace
```

This loop mount does not return automatically after a restart. First confirm
that the Seagate NTFS volume is mounted read-write, then recreate the workspace
mount:

```bash
IMAGE="/run/media/kelenna-udo/Seagate Backup Plus drive/PX4_LINUX_WORKSPACE.img"

sudo mkdir -p /mnt/px4-workspace
sudo mount -o loop,rw "$IMAGE" /mnt/px4-workspace
sudo chown "$USER:$USER" /mnt/px4-workspace
```

The external drive must remain connected while the workspace is mounted.
After the workspace is available, the planned source checkout command is:

```bash
export PX4_SOURCE_DIR=/mnt/px4-workspace/PX4-Autopilot

git clone --branch v1.17.0 --recursive \
  https://github.com/PX4/PX4-Autopilot.git \
  "$PX4_SOURCE_DIR"
```

The exact tag check should print `v1.17.0`:

```bash
git -C "$PX4_SOURCE_DIR" describe --tags --exact-match
git -C "$PX4_SOURCE_DIR" submodule status --recursive
```

## Build the Dependency Image

```bash
./docker/px4/build_image.sh
```

The build script refuses to continue when the checkout is not exactly
`v1.17.0` or its submodules do not match. It exposes only `Tools/setup` to the
Docker build, so the PX4 source is not stored in the final image.

Expected image name:

```text
px4-sitl:v1.17.0
```

## Start PX4 SITL and Gazebo

The shell script is preferred for the first flight because it preserves the
interactive PX4 console:

```bash
./src/px4_sitl_bringup/scripts/run_px4_sitl.sh
```

Headless operation omits the Gazebo window:

```bash
HEADLESS=1 ./src/px4_sitl_bringup/scripts/run_px4_sitl.sh
```

ROS launch equivalent:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch px4_sitl_bringup px4_sitl.launch.py
```

These commands default to `/mnt/px4-workspace/PX4-Autopilot`. Override
`PX4_SOURCE_DIR` only when using a different checkout location.

The expected underlying PX4 build target is:

```bash
make px4_sitl gz_x500
```

## Networking and Display

- Host networking exposes PX4's normal QGroundControl, MAVSDK, and uXRCE-DDS
  UDP traffic directly on the host.
- The Gazebo GUI uses the host's X11/XWayland socket and display authorization.
- The runner requests Docker's NVIDIA runtime and all available NVIDIA GPUs.
- NVIDIA graphics, display, utility, and compute driver capabilities are
  exposed to the container.
- PRIME render-offload variables select the NVIDIA GPU on this hybrid
  Intel/NVIDIA laptop.
- Available `/dev/dri` graphics devices are passed through to the container.
- Set `HEADLESS=1` if GUI forwarding is unavailable.

Docker access to the RTX 4050 has been verified with:

```bash
sudo docker run --rm \
  --runtime=nvidia \
  --gpus all \
  ubuntu:24.04 \
  nvidia-smi
```

The first X500 checkpoint will confirm that the running Gazebo process is
actively using the GPU before attempting flight.

## Comparing With the X3 Workflow

The X3 implementation is stored on a separate Git branch rather than inside
this PX4-focused branch:

```bash
git switch feature/telemetry-sensors
```

Return with:

```bash
git switch feature/px4-sitl
```

Commit or stash local work before switching branches.

## Cleanup

Containers started by the runner use `--rm`, so the container is deleted after
it stops. Source and build output remain on the external drive.

To remove only the project dependency image:

```bash
sudo docker image rm px4-sitl:v1.17.0
```

Do not remove the image while the PX4 container is running.

## References

- [PX4 Ubuntu development environment](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu)
- [PX4 Gazebo simulation](https://docs.px4.io/v1.17/en/sim_gazebo_gz/)
- [PX4 prebuilt SITL packages](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages)
- [PX4 uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds)
