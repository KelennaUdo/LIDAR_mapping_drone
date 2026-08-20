#!/usr/bin/env bash
set -euo pipefail

# This script manages the real Seagate drive and the virtual ext4 drive stored
# inside PX4_LINUX_WORKSPACE.img. Run it as your normal user; it asks for sudo
# only when Linux needs administrator permission to mount or unmount storage.

SEAGATE_LABEL="Seagate Backup Plus drive"
IMAGE_NAME="PX4_LINUX_WORKSPACE.img"
WORKSPACE_MOUNT="/mnt/px4-workspace"
AUTO_WORKSPACE_MOUNT="/run/media/$USER/PX4_WORKSPACE"
PX4_CHECKOUT="$WORKSPACE_MOUNT/PX4-Autopilot"
PX4_CONTAINER="px4-sitl"
DDS_AGENT_CONTAINER="px4-dds-agent"

usage() {
  cat <<EOF
Usage: $0 <connect|status|disconnect>

  connect     Mount the Seagate drive and virtual PX4 workspace safely.
  status      Inspect the drives, PX4 checkout, and containers without changing them.
  disconnect  Stop PX4 services with confirmation, unmount both filesystems, and eject the drive.
EOF
}

is_mounted_at() {
  findmnt -rn --mountpoint "$1" >/dev/null 2>&1
}

mount_options() {
  findmnt -rn --mountpoint "$1" -o OPTIONS 2>/dev/null || true
}

is_read_write() {
  local options
  options="$(mount_options "$1")"
  [[ ",$options," == *,rw,* ]]
}

physical_device() {
  local path=""
  local label=""

  while read -r path label; do
    if [[ "$label" == "$SEAGATE_LABEL" ]]; then
      printf '%s\n' "$path"
      return
    fi
  done < <(lsblk -pno PATH,LABEL)
}

physical_mount() {
  local device="$1"
  findmnt -n -S "$device" -o TARGET 2>/dev/null | sed -n '1p' || true
}

detach_unused_image_loops() {
  local image_path="$1"
  local loop_device=""
  local loop_details=""

  while IFS=: read -r loop_device loop_details; do
    if [[ -z "$loop_device" ]]; then
      continue
    fi

    if findmnt -rn -S "$loop_device" >/dev/null 2>&1; then
      echo "Virtual-drive adapter is still mounted: $loop_device" >&2
      echo "Refusing to detach an active filesystem." >&2
      return 1
    fi

    if sudo fuser -s "$loop_device" 2>/dev/null; then
      echo "Virtual-drive adapter is still in use: $loop_device" >&2
      echo "Close programs using it, then try again." >&2
      return 1
    fi

    echo "Detaching stale virtual-drive adapter $loop_device"
    sudo losetup -d "$loop_device"
  done < <(sudo losetup -j "$image_path")
}

release_workspace_loop() {
  local loop_device="$1"

  if is_mounted_at "$WORKSPACE_MOUNT"; then
    sudo umount "$WORKSPACE_MOUNT"
  fi

  # An autoclear loop may already disappear when the mount is released.
  sudo losetup -d "$loop_device" 2>/dev/null || true
}

container_id() {
  local container_name="$1"

  if docker ps >/dev/null 2>&1; then
    docker ps -q --filter "name=^/${container_name}$"
  elif sudo -n docker ps >/dev/null 2>&1; then
    sudo -n docker ps -q --filter "name=^/${container_name}$"
  else
    return 2
  fi
}

container_id_with_sudo() {
  local container_name="$1"

  sudo docker ps -q --filter "name=^/${container_name}$"
}

print_status() {
  local device=""
  local seagate_mount=""
  local seagate_state="not connected"
  local workspace_state="not mounted"
  local checkout_state="not available"
  local px4_container_state="unknown (run with cached sudo credentials)"
  local agent_container_state="unknown (run with cached sudo credentials)"
  local ready_state="no"
  local unplug_state="no"
  local px4_id=""
  local agent_id=""

  device="$(physical_device)"
  if [[ -n "$device" ]]; then
    seagate_mount="$(physical_mount "$device")"
    if [[ -n "$seagate_mount" ]]; then
      if is_read_write "$seagate_mount"; then
        seagate_state="mounted read-write at $seagate_mount"
      else
        seagate_state="mounted READ-ONLY at $seagate_mount"
      fi
    else
      seagate_state="connected but not mounted"
    fi
  fi

  if is_mounted_at "$WORKSPACE_MOUNT"; then
    if is_read_write "$WORKSPACE_MOUNT"; then
      workspace_state="mounted read-write at $WORKSPACE_MOUNT"
    else
      workspace_state="mounted READ-ONLY at $WORKSPACE_MOUNT"
    fi
  elif is_mounted_at "$AUTO_WORKSPACE_MOUNT"; then
    workspace_state="mounted at unexpected path $AUTO_WORKSPACE_MOUNT"
  fi

  if [[ -d "$PX4_CHECKOUT/.git" && -f "$PX4_CHECKOUT/Makefile" ]]; then
    checkout_state="found at $PX4_CHECKOUT"
  fi

  if px4_id="$(container_id "$PX4_CONTAINER" 2>/dev/null)"; then
    if [[ -n "$px4_id" ]]; then
      px4_container_state="running ($px4_id)"
    else
      px4_container_state="not running"
    fi
  fi

  if agent_id="$(container_id "$DDS_AGENT_CONTAINER" 2>/dev/null)"; then
    if [[ -n "$agent_id" ]]; then
      agent_container_state="running ($agent_id)"
    else
      agent_container_state="not running"
    fi
  fi

  if [[ "$seagate_state" == mounted\ read-write* \
    && "$workspace_state" == mounted\ read-write* \
    && "$checkout_state" == found* \
    && "$px4_container_state" == "not running" \
    && "$agent_container_state" == "not running" ]]; then
    ready_state="yes"
  fi

  if [[ -z "$seagate_mount" \
    && "$workspace_state" == "not mounted" \
    && "$px4_container_state" == "not running" \
    && "$agent_container_state" == "not running" ]]; then
    unplug_state="yes"
  fi

  printf '%-24s %s\n' "Seagate:" "$seagate_state"
  printf '%-24s %s\n' "Virtual PX4 drive:" "$workspace_state"
  printf '%-24s %s\n' "PX4 checkout:" "$checkout_state"
  printf '%-24s %s\n' "PX4 container:" "$px4_container_state"
  printf '%-24s %s\n' "DDS Agent container:" "$agent_container_state"
  printf '%-24s %s\n' "Ready to launch PX4:" "$ready_state"
  printf '%-24s %s\n' "Safe to unplug:" "$unplug_state"
}

unmount_workspace_locations() {
  local target

  for target in "$WORKSPACE_MOUNT" "$AUTO_WORKSPACE_MOUNT"; do
    if is_mounted_at "$target"; then
      echo "Unmounting virtual PX4 drive from $target"
      sudo umount "$target"
    fi
  done
}

connect_workspace() {
  local device=""
  local seagate_mount=""
  local image_path=""
  local px4_running_id=""
  local agent_running_id=""
  local loop_device=""
  local loop_read_only=""

  sudo -v

  px4_running_id="$(container_id_with_sudo "$PX4_CONTAINER")"
  agent_running_id="$(container_id_with_sudo "$DDS_AGENT_CONTAINER")"
  if [[ -n "$px4_running_id" || -n "$agent_running_id" ]]; then
    if [[ -n "$px4_running_id" ]]; then
      echo "PX4 container is already running: $px4_running_id" >&2
    fi
    if [[ -n "$agent_running_id" ]]; then
      echo "DDS Agent container is already running: $agent_running_id" >&2
    fi
    echo "Use '$0 status' instead of changing mounts." >&2
    exit 2
  fi

  device="$(physical_device)"
  if [[ -z "$device" || ! -b "$device" ]]; then
    echo "Seagate drive not found by label: $SEAGATE_LABEL" >&2
    echo "Connect the drive, wait for Ubuntu to detect it, and try again." >&2
    exit 2
  fi

  # Remove an old virtual-drive connection before touching the physical drive.
  unmount_workspace_locations

  seagate_mount="$(physical_mount "$device")"
  if [[ -z "$seagate_mount" ]]; then
    echo "Mounting physical Seagate drive"
    udisksctl mount -b "$device"
    seagate_mount="$(physical_mount "$device")"
  elif ! is_read_write "$seagate_mount"; then
    echo "Seagate is read-only; reconnecting it cleanly"
    udisksctl unmount -b "$device"
    udisksctl mount -b "$device"
    seagate_mount="$(physical_mount "$device")"
  fi

  if [[ -z "$seagate_mount" ]] || ! is_mounted_at "$seagate_mount"; then
    echo "Seagate drive did not mount." >&2
    exit 2
  fi

  if ! is_read_write "$seagate_mount"; then
    echo "Seagate drive is still read-only after reconnecting." >&2
    echo "No PX4 mount was attempted. The NTFS filesystem may need inspection." >&2
    exit 2
  fi

  image_path="$seagate_mount/$IMAGE_NAME"
  if [[ ! -f "$image_path" ]]; then
    echo "PX4 workspace image not found: $image_path" >&2
    exit 2
  fi

  if [[ ! -w "$image_path" ]]; then
    echo "PX4 workspace image is not writable: $image_path" >&2
    exit 2
  fi

  echo "Mounting virtual PX4 drive at $WORKSPACE_MOUNT"
  sudo mkdir -p "$WORKSPACE_MOUNT"

  if ! detach_unused_image_loops "$image_path"; then
    exit 2
  fi

  if ! sudo mount --read-write -o loop "$image_path" "$WORKSPACE_MOUNT"; then
    echo "Could not mount virtual PX4 drive read-write." >&2
    detach_unused_image_loops "$image_path" || true
    exit 2
  fi

  if ! loop_device="$(
    findmnt -n -o SOURCE --target "$WORKSPACE_MOUNT"
  )" || [[ -z "$loop_device" ]]; then
    echo "Could not identify the mounted virtual-drive adapter." >&2
    sudo umount "$WORKSPACE_MOUNT"
    exit 2
  fi

  if ! loop_read_only="$(sudo blockdev --getro "$loop_device")"; then
    echo "Could not verify virtual-drive adapter: $loop_device" >&2
    release_workspace_loop "$loop_device"
    exit 2
  fi

  if [[ "$loop_read_only" != "0" ]]; then
    echo "Virtual-drive adapter was created read-only: $loop_device" >&2
    release_workspace_loop "$loop_device"
    exit 2
  fi

  echo "Created read-write virtual-drive adapter $loop_device"
  if ! is_read_write "$WORKSPACE_MOUNT"; then
    echo "Virtual PX4 drive did not mount read-write." >&2
    release_workspace_loop "$loop_device"
    exit 2
  fi

  sudo chown "$(id -u):$(id -g)" "$WORKSPACE_MOUNT"

  if [[ ! -d "$PX4_CHECKOUT/.git" || ! -f "$PX4_CHECKOUT/Makefile" ]]; then
    echo "PX4 checkout not found after mounting: $PX4_CHECKOUT" >&2
    exit 2
  fi

  echo
  echo "PX4 workspace connected successfully."
  print_status
}

disconnect_workspace() {
  local device=""
  local seagate_mount=""
  local px4_running_id=""
  local agent_running_id=""
  local answer=""
  local parent_name=""
  local parent_device=""

  sudo -v

  px4_running_id="$(container_id_with_sudo "$PX4_CONTAINER")"
  agent_running_id="$(container_id_with_sudo "$DDS_AGENT_CONTAINER")"
  if [[ -n "$px4_running_id" || -n "$agent_running_id" ]]; then
    if [[ ! -t 0 ]]; then
      echo "PX4 services are running. Re-run interactively to confirm shutdown." >&2
      exit 2
    fi

    echo "Running PX4 services:"
    if [[ -n "$agent_running_id" ]]; then
      echo "  DDS Agent: $agent_running_id"
    fi
    if [[ -n "$px4_running_id" ]]; then
      echo "  PX4 SITL:   $px4_running_id"
    fi
    printf 'Stop them now? [y/N] '
    read -r answer
    if [[ ! "$answer" =~ ^[Yy]$ ]]; then
      echo "Disconnect cancelled. The drive remains mounted."
      exit 2
    fi

    if [[ -n "$agent_running_id" ]]; then
      echo "Stopping DDS Agent container"
      sudo docker stop --timeout 10 "$DDS_AGENT_CONTAINER"
    fi
    if [[ -n "$px4_running_id" ]]; then
      echo "Stopping PX4 container"
      sudo docker stop --timeout 30 "$PX4_CONTAINER"
    fi
  fi

  unmount_workspace_locations

  device="$(physical_device)"
  if [[ -z "$device" || ! -b "$device" ]]; then
    echo "Seagate drive is already disconnected."
    print_status
    return
  fi

  seagate_mount="$(physical_mount "$device")"
  if [[ -n "$seagate_mount" ]]; then
    echo "Unmounting physical Seagate drive"
    udisksctl unmount -b "$device"
  fi

  parent_name="$(lsblk -nro PKNAME "$device" | sed -n '1p')"
  if [[ -n "$parent_name" && -b "/dev/$parent_name" ]]; then
    parent_device="/dev/$parent_name"
    echo "Powering off $parent_device for safe removal"
    udisksctl power-off -b "$parent_device"
  fi

  echo
  echo "Workspace disconnected. It is safe to unplug the Seagate drive."
}

if [[ "$(id -u)" == "0" ]]; then
  echo "Run this script as your normal user, not with sudo." >&2
  echo "The script requests sudo only for operations that need it." >&2
  exit 2
fi

case "${1:-}" in
  connect)
    connect_workspace
    ;;
  status)
    sudo -v
    print_status
    ;;
  disconnect)
    disconnect_workspace
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
