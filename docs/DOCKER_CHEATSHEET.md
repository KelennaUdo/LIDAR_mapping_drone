# Docker Command Cheat Sheet

## Mental Model

```text
Dockerfile
    ↓ docker build
Image: reusable read-only template
    ↓ docker run
Container: running process environment
```

For this project:

```text
ubuntu:24.04
    ↓ PX4 dependencies are added
px4-sitl:v1.17.0
    ↓ temporary container is created
PX4 SITL + Gazebo X500
```

The PX4 runner uses `--rm`, so its temporary container is deleted after it
stops. The `px4-sitl:v1.17.0` image and external PX4 source remain.

This computer currently requires `sudo` for Docker commands.

## Inspect Docker

```bash
# Confirm that the Docker service is running.
sudo systemctl is-active docker

# Show Docker client, server, containerd, and runtime versions.
sudo docker version

# Show Docker's registered runtimes. The output should include "nvidia".
sudo docker info --format '{{json .Runtimes}}'

# Show Docker's image, container, volume, and build-cache disk usage.
sudo docker system df

# Show a more detailed storage breakdown.
sudo docker system df -v
```

## Images

```bash
# List locally stored images.
sudo docker image ls

# Show only the project's PX4 image.
sudo docker image ls px4-sitl:v1.17.0

# Show detailed metadata for an image.
sudo docker image inspect px4-sitl:v1.17.0

# Show the layers used to build an image.
sudo docker image history px4-sitl:v1.17.0
```

`ubuntu:24.04` is the base template used to build the larger
`px4-sitl:v1.17.0` image. Docker shares common layers instead of storing
identical copies.

## Containers

```bash
# List running containers.
sudo docker ps

# List running and stopped containers.
sudo docker ps -a

# Look specifically for the PX4 container.
sudo docker ps --filter name=px4-sitl

# Show the complete configuration of the PX4 container.
sudo docker inspect px4-sitl

# Show live CPU, memory, network, and process usage.
sudo docker stats px4-sitl

# Show processes running inside the PX4 container.
sudo docker top px4-sitl
```

Closing the Gazebo window does not prove that the container stopped. Use
`docker ps` to check.

## Logs and Container Shells

These commands require the container to be running:

```bash
# Print output previously written by the container.
sudo docker logs px4-sitl

# Follow new log output until Ctrl+C is pressed.
sudo docker logs --follow px4-sitl

# Open an additional Bash shell inside the running container.
sudo docker exec -it px4-sitl bash

# Run one command inside the container without opening a shell.
sudo docker exec px4-sitl nvidia-smi
```

`docker exec` does not create a second PX4 simulation. It runs an additional
command inside the existing container.

## Stopping PX4

```bash
# Preferred method: press Ctrl+C in the terminal that launched PX4.

# If that terminal is unavailable, request a graceful stop.
# Docker waits up to 30 seconds before forcing the remaining processes to stop.
sudo docker stop --timeout 30 px4-sitl

# Confirm that no PX4 container remains.
sudo docker ps --filter name=px4-sitl
```

Avoid treating `docker kill` as a normal shutdown command. It does not give PX4
and Gazebo time to exit cleanly.

## NVIDIA GPU Checks

```bash
# Show the host GPU, memory usage, utilization, and active processes.
nvidia-smi

# Refresh the host GPU display every second. Press Ctrl+C to stop watching.
watch -n 1 nvidia-smi

# Verify that a temporary Ubuntu container can access the NVIDIA GPU.
# --rm deletes this test container after nvidia-smi exits.
sudo docker run --rm \
  --runtime=nvidia \
  --gpus all \
  ubuntu:24.04 \
  nvidia-smi

# Check the GPU from inside a running PX4 container.
sudo docker exec px4-sitl nvidia-smi
```

## Building the PX4 Image

Run from the repository root:

```bash
# Build px4-sitl:v1.17.0 using the project's validation wrapper.
./docker/px4/build_image.sh
```

This is not required before every launch. Rebuild only when the Dockerfile or
dependency setup changes.

## Cleanup Warnings

Do not use the following as routine commands:

```text
docker system prune
docker image prune -a
docker volume prune
```

They can remove caches, images, volumes, or other Docker resources unrelated
to PX4. Inspect with `docker system df` first and review any cleanup operation
separately.
