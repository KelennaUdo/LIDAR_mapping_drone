#!/usr/bin/env bash
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
package_share="$(cd "$(dirname "$script_path")/.." && pwd)"

PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_IMAGE="${PX4_IMAGE:-px4-sitl:${PX4_VERSION}}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-/mnt/px4-workspace/PX4-Autopilot}"
PX4_AGENT_DIR="${PX4_AGENT_DIR:-/mnt/px4-workspace/Micro-XRCE-DDS-Agent}"
KISS_ICP_INSTALL_DIR="${KISS_ICP_INSTALL_DIR:-/mnt/px4-workspace/kiss_icp_ws/install}"
KISS_ICP_SETUP="${KISS_ICP_SETUP:-$KISS_ICP_INSTALL_DIR/setup.bash}"
KISS_ICP_CONFIG="${KISS_ICP_CONFIG:-$KISS_ICP_INSTALL_DIR/kiss_icp/share/kiss_icp/config/config.yaml}"
KISS_ICP_POINTCLOUD_TOPIC="${KISS_ICP_POINTCLOUD_TOPIC:-/x500/lidar/points}"
PX4_PROJECT_MODELS_DIR="${PX4_PROJECT_MODELS_DIR:-$package_share/models}"
PX4_PROJECT_WORLDS_DIR="${PX4_PROJECT_WORLDS_DIR:-$package_share/worlds}"
LIDAR_BRIDGE_SCRIPT="${LIDAR_BRIDGE_SCRIPT:-$package_share/scripts/run_lidar_bridge.sh}"
LIDAR_RVIZ_SCRIPT="${LIDAR_RVIZ_SCRIPT:-$package_share/scripts/run_lidar_rviz.sh}"
X500_TF_SCRIPT="${X500_TF_SCRIPT:-$package_share/scripts/run_x500_tf.sh}"
QGC_APPIMAGE="${QGC_APPIMAGE:-$HOME/Applications/QGroundControl/QGroundControl.AppImage}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-mapping_test}"
GZ_PARTITION="${GZ_PARTITION:-px4_sitl}"
HEADLESS="${HEADLESS:-0}"
START_QGC="${START_QGC:-1}"
START_RVIZ="${START_RVIZ:-1}"
START_TF="${START_TF:-1}"
START_KISS_ICP="${START_KISS_ICP:-1}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
DDS_AGENT_VERBOSE="${DDS_AGENT_VERBOSE:-4}"
PX4_CONTAINER="${PX4_CONTAINER_NAME:-px4-sitl}"
DDS_AGENT_CONTAINER="${DDS_AGENT_CONTAINER_NAME:-px4-dds-agent}"
QGC_LOG="${QGC_LOG:-/tmp/px4-qgroundcontrol.log}"
RVIZ_LOG="${RVIZ_LOG:-/tmp/px4-lidar-rviz.log}"
KISS_ICP_LOG="${KISS_ICP_LOG:-/tmp/px4-kiss-icp.log}"

docker_command=(docker)
qgc_pid=""
qgc_started="no"
lidar_bridge_pid=""
tf_bridge_pid=""
kiss_icp_pid=""
rviz_pid=""
rviz_started="no"
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

kiss_icp_process_ids() {
  # Match both the `ros2 run` wrapper and its native KISS-ICP child process.
  pgrep -u "$(id -u)" -f \
    '(^|[/[:space:]])kiss_icp_node([[:space:]]|$)' || true
}

process_is_running() {
  local process_state

  process_state="$(ps -o stat= -p "$1" 2>/dev/null)" || return 1
  process_state="${process_state//[[:space:]]/}"

  [[ -n "$process_state" && "${process_state:0:1}" != "Z" ]]
}

wait_for_processes_to_stop() {
  local timeout_seconds="$1"
  shift

  local checks=$((timeout_seconds * 10))
  local check
  local pid
  local found_running_process

  for ((check = 0; check < checks; ++check)); do
    found_running_process="no"

    for pid in "$@"; do
      if process_is_running "$pid"; then
        found_running_process="yes"
        break
      fi
    done

    if [[ "$found_running_process" == "no" ]]; then
      return 0
    fi

    sleep 0.1
  done

  return 1
}

stop_kiss_icp() {
  local -a process_ids=()
  local pid

  mapfile -t process_ids < <(kiss_icp_process_ids)
  if [[ "${#process_ids[@]}" -eq 0 ]]; then
    return
  fi

  echo "Stopping KISS-ICP odometry"
  kill -TERM "${process_ids[@]}" 2>/dev/null || true

  if ! wait_for_processes_to_stop 5 "${process_ids[@]}"; then
    echo "KISS-ICP did not stop after 5 seconds; forcing shutdown" >&2

    for pid in "${process_ids[@]}"; do
      if process_is_running "$pid"; then
        kill -KILL "$pid" 2>/dev/null || true
      fi
    done
  fi

  if [[ -n "$kiss_icp_pid" ]]; then
    wait "$kiss_icp_pid" 2>/dev/null || true
  fi
}

start_kiss_icp() (
  # ROS setup files may read unset variables, so nounset is paused while sourcing.
  set +u
  source /opt/ros/lyrical/setup.bash
  source "$KISS_ICP_SETUP"
  set -u

  exec ros2 run kiss_icp kiss_icp_node --ros-args \
    --remap "pointcloud_topic:=$KISS_ICP_POINTCLOUD_TOPIC" \
    --params-file "$KISS_ICP_CONFIG" \
    -p base_frame:=lidar_link \
    -p lidar_odom_frame:=odom_lidar \
    -p publish_odom_tf:=true \
    -p invert_odom_tf:=true \
    -p publish_debug_clouds:=true \
    -p use_sim_time:=true \
    -p position_covariance:=0.1 \
    -p orientation_covariance:=0.1
)

cleanup() {
  local exit_code=$?

  if [[ "$cleanup_started" == "yes" ]]; then
    return
  fi
  cleanup_started="yes"

  trap - EXIT INT TERM
  set +e

  if [[ "$rviz_started" == "yes" && -n "$rviz_pid" ]] \
    && kill -0 "$rviz_pid" 2>/dev/null; then
    echo "Stopping RViz"
    kill "$rviz_pid" 2>/dev/null
    wait "$rviz_pid" 2>/dev/null
  fi

  stop_kiss_icp

  if [[ -n "$tf_bridge_pid" ]] \
    && kill -0 "$tf_bridge_pid" 2>/dev/null; then
    echo "Stopping X500 TF adapter"
    kill "$tf_bridge_pid" 2>/dev/null
    wait "$tf_bridge_pid" 2>/dev/null
  fi

  if [[ -n "$lidar_bridge_pid" ]] \
    && kill -0 "$lidar_bridge_pid" 2>/dev/null; then
    echo "Stopping ROS 2 LiDAR bridge"
    kill "$lidar_bridge_pid" 2>/dev/null
    wait "$lidar_bridge_pid" 2>/dev/null
  fi

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

if [[ ! -x "$LIDAR_BRIDGE_SCRIPT" ]]; then
  echo "LiDAR bridge runner is not executable: $LIDAR_BRIDGE_SCRIPT" >&2
  exit 2
fi

if [[ ! -x "$LIDAR_RVIZ_SCRIPT" ]]; then
  echo "LiDAR RViz runner is not executable: $LIDAR_RVIZ_SCRIPT" >&2
  exit 2
fi

if [[ ! -x "$X500_TF_SCRIPT" ]]; then
  echo "X500 TF runner is not executable: $X500_TF_SCRIPT" >&2
  exit 2
fi

if [[ "$START_KISS_ICP" == "1" || "$START_KISS_ICP" == "true" ]]; then
  if [[ ! -f "$KISS_ICP_SETUP" ]]; then
    echo "KISS-ICP workspace setup not found: $KISS_ICP_SETUP" >&2
    echo "Connect the PX4 workspace and build KISS-ICP first." >&2
    echo "Set START_KISS_ICP=0 to launch without LiDAR odometry." >&2
    exit 2
  fi

  if [[ ! -f "$KISS_ICP_CONFIG" ]]; then
    echo "KISS-ICP configuration not found: $KISS_ICP_CONFIG" >&2
    echo "Rebuild the external KISS-ICP workspace before launching." >&2
    exit 2
  fi
fi

mapfile -t stale_kiss_icp_pids < <(kiss_icp_process_ids)
if [[ "${#stale_kiss_icp_pids[@]}" -gt 0 ]]; then
  stale_kiss_icp_pid_list="$(
    IFS=,
    echo "${stale_kiss_icp_pids[*]}"
  )"

  echo "KISS-ICP is already running from an earlier session:" >&2
  ps -o pid=,ppid=,stat=,args= -p "$stale_kiss_icp_pid_list" >&2
  echo >&2
  echo "Stop the stale session, then run this launcher again:" >&2
  echo "  pkill -TERM -u $(id -u) -f 'kiss_icp_node'" >&2
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
trap 'exit 129' HUP

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

echo "Starting ROS 2 LiDAR point-cloud bridge"
GZ_PARTITION="$GZ_PARTITION" "$LIDAR_BRIDGE_SCRIPT" &
lidar_bridge_pid=$!

sleep 1
if ! kill -0 "$lidar_bridge_pid" 2>/dev/null; then
  echo "ROS 2 LiDAR bridge stopped during startup." >&2
  wait "$lidar_bridge_pid" 2>/dev/null || true
  exit 2
fi

if [[ "$START_TF" == "1" || "$START_TF" == "true" ]]; then
  echo "Starting X500 world-frame TF adapter"
  GZ_PARTITION="$GZ_PARTITION" "$X500_TF_SCRIPT" &
  tf_bridge_pid=$!

  sleep 1
  if ! kill -0 "$tf_bridge_pid" 2>/dev/null; then
    echo "X500 TF adapter stopped during startup." >&2
    wait "$tf_bridge_pid" 2>/dev/null || true
    exit 2
  fi
else
  echo "START_TF is disabled; world-frame RViz data may be unavailable."
fi

if [[ "$START_KISS_ICP" == "1" || "$START_KISS_ICP" == "true" ]]; then
  echo "Starting KISS-ICP LiDAR odometry"
  start_kiss_icp >"$KISS_ICP_LOG" 2>&1 &
  kiss_icp_pid=$!

  sleep 1
  if ! kill -0 "$kiss_icp_pid" 2>/dev/null; then
    echo "KISS-ICP stopped during startup." >&2
    echo "Log: $KISS_ICP_LOG" >&2
    tail -n 40 "$KISS_ICP_LOG" >&2 || true
    wait "$kiss_icp_pid" 2>/dev/null || true
    exit 2
  fi
else
  echo "START_KISS_ICP is disabled; LiDAR odometry will not be started."
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

if [[ "$graphical" == "yes" \
  && ( "$START_RVIZ" == "1" || "$START_RVIZ" == "true" ) ]]; then
  echo "Starting RViz for the X500 3D LiDAR"
  "$LIDAR_RVIZ_SCRIPT" >"$RVIZ_LOG" 2>&1 &
  rviz_pid=$!
  rviz_started="yes"
  sleep 1
  if ! kill -0 "$rviz_pid" 2>/dev/null; then
    echo "RViz stopped during startup." >&2
    echo "Log: $RVIZ_LOG" >&2
    exit 2
  fi
elif [[ "$graphical" == "no" ]]; then
  echo "HEADLESS mode: skipping RViz."
else
  echo "START_RVIZ is disabled; RViz will not be started."
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
  --env "GZ_PARTITION=$GZ_PARTITION"
  --volume "$PX4_SOURCE_DIR:/workspace/PX4-Autopilot:rw"
)

# Overlay a project-owned vehicle model at PX4's expected model path. This
# changes only the container's view; the external PX4 checkout stays untouched.
px4_model_name="${PX4_SIM_MODEL#gz_}"
project_model_file="$PX4_PROJECT_MODELS_DIR/$px4_model_name/model.sdf"
if [[ -f "$project_model_file" ]]; then
  container_model_file="/workspace/PX4-Autopilot/Tools/simulation/gz/models/$px4_model_name/model.sdf"
  px4_args+=(
    --volume "$project_model_file:$container_model_file:ro"
  )
fi

# A project-owned world is overlaid at PX4's expected location without
# modifying the external PX4 checkout. Built-in PX4 worlds need no overlay.
project_world_file="$PX4_PROJECT_WORLDS_DIR/$PX4_GZ_WORLD.sdf"
if [[ -f "$project_world_file" ]]; then
  container_world_file="/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf"
  px4_args+=(
    --volume "$project_world_file:$container_world_file:ro"
  )
fi

interactive_terminal="no"
if [[ -t 0 && -t 1 ]]; then
  interactive_terminal="yes"
  px4_args+=(--detach --interactive --tty)
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
if [[ -f "$project_model_file" ]]; then
  echo "Model source: $project_model_file (project-owned, read-only)"
else
  echo "Model source: PX4 built-in model"
fi
if [[ -f "$project_world_file" ]]; then
  echo "World source: $project_world_file (project-owned, read-only)"
else
  echo "World source: PX4 built-in world"
fi
echo "Docker image: $PX4_IMAGE"
echo "Graphics: NVIDIA GPU requested through the Docker NVIDIA runtime"
echo "Gazebo Transport partition: $GZ_PARTITION"
echo "DDS Agent container: $DDS_AGENT_CONTAINER"
echo "ROS 2 point cloud: /x500/lidar/points"
if [[ -n "$kiss_icp_pid" ]]; then
  echo "KISS-ICP odometry: /kiss/odometry"
  echo "KISS-ICP local map: /kiss/local_map"
  echo "KISS-ICP log: $KISS_ICP_LOG"
fi
if [[ -n "$tf_bridge_pid" ]]; then
  echo "ROS 2 TF chain: world -> base_link -> lidar_link"
fi
if [[ "$qgc_started" == "yes" ]]; then
  echo "QGroundControl log: $QGC_LOG"
fi
if [[ "$rviz_started" == "yes" ]]; then
  echo "RViz log: $RVIZ_LOG"
fi
echo "Press Ctrl+C to stop the complete PX4 session."

if [[ "$interactive_terminal" == "yes" ]]; then
  # Start in the background, then attach with Ctrl+C reserved for returning
  # control to this supervisor so cleanup() can stop the complete session.
  "${docker_command[@]}" "${px4_args[@]}" "$PX4_IMAGE" \
    bash -lc 'make px4_sitl "$PX4_SIM_MODEL"' \
    >/dev/null

  "${docker_command[@]}" attach \
    --detach-keys=ctrl-c \
    --sig-proxy=false \
    "$PX4_CONTAINER"
else
  "${docker_command[@]}" "${px4_args[@]}" "$PX4_IMAGE" \
    bash -lc 'make px4_sitl "$PX4_SIM_MODEL"'
fi
