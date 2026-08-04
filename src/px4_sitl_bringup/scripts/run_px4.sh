#!/usr/bin/env bash
set -euo pipefail

PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_IMAGE="${PX4_IMAGE:-px4-sitl:${PX4_VERSION}}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-/mnt/px4-workspace/PX4-Autopilot}"
PX4_AGENT_DIR="${PX4_AGENT_DIR:-/mnt/px4-workspace/Micro-XRCE-DDS-Agent}"
QGC_APPIMAGE="${QGC_APPIMAGE:-$HOME/Applications/QGroundControl/QGroundControl.AppImage}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
HEADLESS="${HEADLESS:-0}"
START_QGC="${START_QGC:-1}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
DDS_AGENT_VERBOSE="${DDS_AGENT_VERBOSE:-4}"
PX4_CONTAINER="${PX4_CONTAINER_NAME:-px4-sitl}"
DDS_AGENT_CONTAINER="${DDS_AGENT_CONTAINER_NAME:-px4-dds-agent}"
QGC_LOG="${QGC_LOG:-/tmp/px4-qgroundcontrol.log}"

docker_command=(docker)
qgc_pid=""
qgc_started="no"
cleanup_started="no"

container_id() {
  local container_name="$1"
  "${docker_command[@]}" ps -q --filter "name=^/${container_name}$"
}

stop_container() {
  local container_name="$1"
  local timeout="$2"

  if [[ -n "$(container_id "$container_name")" ]]; then
    echo "Stopping $container_name"
    "${docker_command[@]}" stop --timeout "$timeout" "$container_name" \
      >/dev/null
  fi
}

cleanup() {
  local exit_code=$?

  if [[ "$cleanup_started" == "yes" ]]; then
    return
  fi
  cleanup_started="yes"

  trap - EXIT INT TERM
  set +e
  stop_container "$DDS_AGENT_CONTAINER" 10
  stop_container "$PX4_CONTAINER" 30

  if [[ "$qgc_started" == "yes" && -n "$qgc_pid" ]] \
    && kill -0 "$qgc_pid" 2>/dev/null; then
    echo "Stopping QGroundControl"
    kill "$qgc_pid" 2>/dev/null
    wait "$qgc_pid" 2>/dev/null
  fi

  exit "$exit_code"
}

if [[ ! -d "$PX4_SOURCE_DIR/.git" || ! -f "$PX4_SOURCE_DIR/Makefile" ]]; then
  echo "PX4 checkout not found: $PX4_SOURCE_DIR" >&2
  echo "Mount the external workspace or override PX4_SOURCE_DIR." >&2
  exit 2
fi

if [[ ! -x "$PX4_AGENT_DIR/build/MicroXRCEAgent" ]]; then
  echo "Micro XRCE-DDS Agent executable not found:" >&2
  echo "  $PX4_AGENT_DIR/build/MicroXRCEAgent" >&2
  exit 2
fi

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

for container_name in "$PX4_CONTAINER" "$DDS_AGENT_CONTAINER"; do
  if [[ -n "$("${docker_command[@]}" ps -aq \
    --filter "name=^/${container_name}$")" ]]; then
    echo "Docker container name is already in use: $container_name" >&2
    echo "Run scripts/px4_workspace.sh disconnect, then try again." >&2
    exit 2
  fi
done

graphical="yes"
if [[ "$HEADLESS" == "1" || "$HEADLESS" == "true" ]]; then
  graphical="no"
fi

if [[ "$graphical" == "yes" ]]; then
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
    echo "Gazebo and QGroundControl need DISPLAY and /tmp/.X11-unix." >&2
    echo "Set HEADLESS=1 to run without graphical applications." >&2
    exit 2
  fi

  if [[ "$START_QGC" == "1" || "$START_QGC" == "true" ]]; then
    if [[ ! -x "$QGC_APPIMAGE" ]]; then
      echo "QGroundControl AppImage is not executable: $QGC_APPIMAGE" >&2
      echo "Set QGC_APPIMAGE when it is installed elsewhere." >&2
      exit 2
    fi
  fi
fi

trap cleanup EXIT
trap 'exit 130' INT TERM

agent_args=(
  run
  --detach
  --rm
  --name "$DDS_AGENT_CONTAINER"
  --network host
  --user "$(id -u):$(id -g)"
  --env HOME=/tmp
  --env "DDS_AGENT_PORT=$DDS_AGENT_PORT"
  --env "DDS_AGENT_VERBOSE=$DDS_AGENT_VERBOSE"
  --volume "$PX4_AGENT_DIR:/workspace/Micro-XRCE-DDS-Agent:ro"
  --workdir /workspace/Micro-XRCE-DDS-Agent
)

echo "Starting Micro XRCE-DDS Agent on UDP port $DDS_AGENT_PORT"
"${docker_command[@]}" "${agent_args[@]}" "$PX4_IMAGE" \
  bash -lc \
  './build/MicroXRCEAgent udp4 -p "$DDS_AGENT_PORT" -v "$DDS_AGENT_VERBOSE"' \
  >/dev/null

sleep 1
if [[ -z "$(container_id "$DDS_AGENT_CONTAINER")" ]]; then
  echo "Micro XRCE-DDS Agent stopped during startup." >&2
  "${docker_command[@]}" logs "$DDS_AGENT_CONTAINER" 2>/dev/null || true
  exit 2
fi

if [[ "$graphical" == "yes" \
  && ( "$START_QGC" == "1" || "$START_QGC" == "true" ) ]]; then
  if pgrep -u "$(id -u)" -f 'QGroundControl' >/dev/null 2>&1; then
    echo "QGroundControl is already running; using the existing process."
  else
    echo "Starting QGroundControl"
    "$QGC_APPIMAGE" >"$QGC_LOG" 2>&1 &
    qgc_pid=$!
    qgc_started="yes"
    sleep 1
    if ! kill -0 "$qgc_pid" 2>/dev/null; then
      echo "QGroundControl stopped during startup." >&2
      echo "Log: $QGC_LOG" >&2
      exit 2
    fi
  fi
elif [[ "$graphical" == "no" ]]; then
  echo "HEADLESS mode: skipping QGroundControl and the Gazebo window."
else
  echo "START_QGC is disabled; QGroundControl will not be started."
fi

px4_args=(
  run
  --rm
  --name "$PX4_CONTAINER"
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
  px4_args+=(-it)
else
  px4_args+=(-i)
fi

if [[ "$graphical" == "no" ]]; then
  px4_args+=(--env HEADLESS=1)
else
  px4_args+=(
    --env "DISPLAY=$DISPLAY"
    --env QT_X11_NO_MITSHM=1
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  )

  xauthority_file="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -f "$xauthority_file" ]]; then
    px4_args+=(
      --env "XAUTHORITY=$xauthority_file"
      --volume "$xauthority_file:$xauthority_file:ro"
    )
  fi

  declare -A added_device_groups=()
  shopt -s nullglob
  for device in /dev/dri/renderD* /dev/dri/card*; do
    px4_args+=(--device "$device:$device")
    device_group="$(stat -c '%g' "$device")"
    if [[ -z "${added_device_groups[$device_group]:-}" ]]; then
      px4_args+=(--group-add "$device_group")
      added_device_groups[$device_group]=1
    fi
  done
  shopt -u nullglob
fi

echo "Starting PX4 $PX4_VERSION with $PX4_SIM_MODEL in $PX4_GZ_WORLD"
echo "Source and build output: $PX4_SOURCE_DIR"
echo "Docker image: $PX4_IMAGE"
echo "Graphics: NVIDIA GPU requested through the Docker NVIDIA runtime"
echo "DDS Agent container: $DDS_AGENT_CONTAINER"
if [[ "$qgc_started" == "yes" ]]; then
  echo "QGroundControl log: $QGC_LOG"
fi
echo "Press Ctrl+C to stop the complete PX4 session."

"${docker_command[@]}" "${px4_args[@]}" "$PX4_IMAGE" \
  bash -lc 'make px4_sitl "$PX4_SIM_MODEL"'
