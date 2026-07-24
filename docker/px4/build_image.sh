#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_IMAGE="${PX4_IMAGE:-px4-sitl:${PX4_VERSION}}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-}"

if [[ -z "$PX4_SOURCE_DIR" ]]; then
  echo "PX4_SOURCE_DIR must point to the PX4-Autopilot checkout." >&2
  echo "Example: PX4_SOURCE_DIR=/media/$USER/px4/PX4-Autopilot $0" >&2
  exit 2
fi

if [[ "$(id -u)" == "0" ]]; then
  echo "Run this script as your normal user, not as root." >&2
  echo "The script invokes sudo for Docker when required." >&2
  exit 2
fi

if [[ ! -d "$PX4_SOURCE_DIR/.git" ]]; then
  echo "Not a Git checkout: $PX4_SOURCE_DIR" >&2
  exit 2
fi

if [[ ! -f "$PX4_SOURCE_DIR/Tools/setup/ubuntu.sh" ]]; then
  echo "PX4 setup script is missing from $PX4_SOURCE_DIR" >&2
  exit 2
fi

actual_tag="$(git -C "$PX4_SOURCE_DIR" describe --tags --exact-match 2>/dev/null || true)"
if [[ "$actual_tag" != "$PX4_VERSION" ]]; then
  echo "Expected PX4 tag $PX4_VERSION, but checkout is '${actual_tag:-not at an exact tag}'." >&2
  echo "Refusing to build an image with mismatched dependencies." >&2
  exit 2
fi

if git -C "$PX4_SOURCE_DIR" submodule status --recursive | grep -Eq '^[-+U]'; then
  echo "PX4 submodules are missing or do not match $PX4_VERSION." >&2
  echo "Run: git -C '$PX4_SOURCE_DIR' submodule update --init --recursive" >&2
  exit 2
fi

docker_command=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_command=(sudo docker)
fi

echo "Building $PX4_IMAGE"
echo "PX4 release: $PX4_VERSION"
echo "Dependency source: $PX4_SOURCE_DIR/Tools/setup"

"${docker_command[@]}" build \
  --build-context "px4_setup=$PX4_SOURCE_DIR/Tools/setup" \
  --build-arg "PX4_VERSION=$PX4_VERSION" \
  --build-arg "HOST_UID=$(id -u)" \
  --build-arg "HOST_GID=$(id -g)" \
  --file "$SCRIPT_DIR/Dockerfile" \
  --tag "$PX4_IMAGE" \
  "$SCRIPT_DIR"

echo "Built Docker image: $PX4_IMAGE"
