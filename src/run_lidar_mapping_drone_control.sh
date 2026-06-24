#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

MODE="manual_keyboard"
RUN_KEYBOARD="true"
SHOW_ARGS="false"
LAUNCH_ARGS=()

for arg in "$@"; do
  case "$arg" in
    mode:=*)
      MODE="${arg#mode:=}"
      ;;
    enable_keyboard:=*)
      RUN_KEYBOARD="${arg#enable_keyboard:=}"
      ;;
    --show-args)
      SHOW_ARGS="true"
      LAUNCH_ARGS+=("$arg")
      ;;
    *)
      LAUNCH_ARGS+=("$arg")
      ;;
  esac
done

if [[ "$SHOW_ARGS" == "true" || "$MODE" != "manual_keyboard" || "$RUN_KEYBOARD" != "true" ]]; then
  ros2 launch lidar_mapping_drone_control flight_controller.launch.py \
    "mode:=$MODE" \
    enable_keyboard:=false \
    "${LAUNCH_ARGS[@]}"
  exit $?
fi

ros2 launch lidar_mapping_drone_control flight_controller.launch.py \
  mode:=manual_keyboard \
  enable_keyboard:=false \
  "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 -m lidar_mapping_drone_control.keyboard_control_node
