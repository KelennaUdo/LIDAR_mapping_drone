#!/usr/bin/env bash
set -euo pipefail

PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_IMAGE="${PX4_IMAGE:-px4-sitl:${PX4_VERSION}}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-/mnt/px4-workspace/PX4-Autopilot}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
HEADLESS="${HEADLESS:-0}"
CONTAINER_NAME="${PX4_CONTAINER_NAME:-px4-sitl}"

if [[ ! -d "$PX4_SOURCE_DIR/.git" || ! -f "$PX4_SOURCE_DIR/Makefile" ]]; then
  echo "PX4 checkout not found: $PX4_SOURCE_DIR" >&2
  echo "Mount the external workspace or override PX4_SOURCE_DIR." >&2
  exit 2
fi

docker_command=(docker)
if ! docker info >/dev/null 2>&1; then
  echo "Docker access requires sudo; you may be prompted for your password." >&2
  docker_command=(sudo docker)
fi

if ! "${docker_command[@]}" image inspect "$PX4_IMAGE" >/dev/null; then
  echo "Docker image not found: $PX4_IMAGE" >&2
  echo "Build it first with docker/px4/build_image.sh." >&2
  exit 2
fi

if ! "${docker_command[@]}" info --format '{{json .Runtimes}}' \
  | grep -q '"nvidia"'; then
  echo "Docker's NVIDIA runtime is not available." >&2
  echo "Install and configure the NVIDIA Container Toolkit first." >&2
  exit 2
fi

docker_args=(
  run
  --rm
  --name "$CONTAINER_NAME"
  --runtime nvidia
  --gpus all
  --network host
  --user "$(id -u):$(id -g)"
  --workdir /workspace/PX4-Autopilot
  --env HOME=/home/px4
  --env NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
  --env __NV_PRIME_RENDER_OFFLOAD=1
  --env __GLX_VENDOR_LIBRARY_NAME=nvidia
  --env __VK_LAYER_NV_optimus=NVIDIA_only
  --env "PX4_SIM_MODEL=$PX4_SIM_MODEL"
  --env "PX4_GZ_WORLD=$PX4_GZ_WORLD"
  --volume "$PX4_SOURCE_DIR:/workspace/PX4-Autopilot:rw"
)

if [[ -t 0 && -t 1 ]]; then
  docker_args+=(-it)
else
  docker_args+=(-i)
fi

if [[ "$HEADLESS" == "1" || "$HEADLESS" == "true" ]]; then
  docker_args+=(--env HEADLESS=1)
else
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
    echo "Gazebo GUI needs DISPLAY and /tmp/.X11-unix." >&2
    echo "Set HEADLESS=1 to run without the Gazebo window." >&2
    exit 2
  fi

  docker_args+=(
    --env "DISPLAY=$DISPLAY"
    --env QT_X11_NO_MITSHM=1
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  )

  xauthority_file="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -f "$xauthority_file" ]]; then
    docker_args+=(
      --env "XAUTHORITY=$xauthority_file"
      --volume "$xauthority_file:$xauthority_file:ro"
    )
  fi

  declare -A added_device_groups=()
  shopt -s nullglob
  for device in /dev/dri/renderD* /dev/dri/card*; do
    docker_args+=(--device "$device:$device")
    device_group="$(stat -c '%g' "$device")"
    if [[ -z "${added_device_groups[$device_group]:-}" ]]; then
      docker_args+=(--group-add "$device_group")
      added_device_groups[$device_group]=1
    fi
  done
  shopt -u nullglob
fi

echo "Starting PX4 $PX4_VERSION with $PX4_SIM_MODEL in $PX4_GZ_WORLD"
echo "Source and build output: $PX4_SOURCE_DIR"
echo "Docker image: $PX4_IMAGE"
echo "Graphics: NVIDIA GPU requested through the Docker NVIDIA runtime"

exec "${docker_command[@]}" "${docker_args[@]}" "$PX4_IMAGE" \
  bash -lc 'make px4_sitl "$PX4_SIM_MODEL"'
