#!/bin/bash
# Start UR5e_Env Docker like ./docker-start.sh, then ensure Xiangqi deps are ready.
#
# Run on the lab dev box (host):
#   ~/par_ur5e_xiangqi/tools/docker-start-xiangqi.sh
#
# Optional env:
#   UR5E=~/UR5e_Env          path to UR5e_Env checkout
#   REPO=~/par_ur5e_xiangqi  path to this repository
#   USE_XIANGQI_IMAGE=1      force ur5e_xiangqi:latest compose override
#   USE_XIANGQI_IMAGE=0      force stock ros:humble + pip deps
#   SKIP_DEPS=1              only docker-compose up (no dep install)
#   SKIP_ONNX_EXPORT=1       skip automatic yolo .pt -> .onnx export after deps
#   FORCE_ONNX_EXPORT=1      re-export ONNX even if file exists
#   YOLO_PT=...              weights path inside container (default v4_best.pt)
#
# After start:
#   cd ~/UR5e_Env && ./docker-attach.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO:-$(cd "$SCRIPT_DIR/.." && pwd)}"
UR5E="${UR5E:-$HOME/UR5e_Env}"
CONTAINER="${CONTAINER:-ros2}"
XIANGQI_IMAGE="${XIANGQI_IMAGE:-ur5e_xiangqi:latest}"

if [[ ! -d "$UR5E" ]]; then
  echo "UR5e_Env not found at: $UR5E"
  echo "Set UR5E=... or clone UR5e_Env to ~/UR5e_Env"
  exit 1
fi

if [[ ! -f "$UR5E/docker-compose.yml" ]]; then
  echo "Missing $UR5E/docker-compose.yml"
  exit 1
fi

use_xiangqi_image=""
if [[ "${USE_XIANGQI_IMAGE:-}" == "1" ]]; then
  use_xiangqi_image=1
elif [[ "${USE_XIANGQI_IMAGE:-}" == "0" ]]; then
  use_xiangqi_image=0
elif docker image inspect "$XIANGQI_IMAGE" >/dev/null 2>&1; then
  use_xiangqi_image=1
else
  use_xiangqi_image=0
fi

compose_args=(-f docker-compose.yml)
if [[ "$use_xiangqi_image" == "1" ]]; then
  override="$REPO/tools/docker-compose.xiangqi.override.yml"
  if [[ ! -f "$override" ]]; then
    echo "Missing $override"
    exit 1
  fi
  compose_args+=(-f "$override")
  echo "==> Starting Docker with image $XIANGQI_IMAGE"
else
  echo "==> Starting Docker with stock ros:humble (will install Xiangqi deps after start)"
  echo "    Tip: run $REPO/tools/lab_build_xiangqi_image.sh once for faster starts"
fi

# docker-compose 1.29 + Docker Engine 29 fails recreating with KeyError: ContainerConfig.
# Prefer Compose v2; remove stale ros2 container before up if recreate would break.
compose() {
  if docker compose version >/dev/null 2>&1; then
    docker compose "$@"
  else
    docker-compose "$@"
  fi
}

if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY="${DISPLAY:-:0}"
fi

echo "==> Starting container (UR5E=$UR5E)"
cd "$UR5E"

export USER_UID="$(id -u)" USER_GID="$(id -g)" USERNAME="${USERNAME:-rosuser}"

# Clean stale state (compose v1.29 + Docker 29 → KeyError: ContainerConfig on recreate).
compose "${compose_args[@]}" down --remove-orphans 2>/dev/null || true
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "==> Removing leftover '$CONTAINER'..."
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
fi

compose "${compose_args[@]}" up -d --remove-orphans

echo "==> Waiting for container '$CONTAINER'..."
for _ in $(seq 1 45); do
  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    break
  fi
  sleep 1
done
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' did not start."
  exit 1
fi

if [[ "${SKIP_DEPS:-}" != "1" ]]; then
  echo "==> Ensuring Xiangqi Python / engine dependencies (+ optional ONNX export)..."
  REPO="$REPO" CONTAINER="$CONTAINER" "$REPO/tools/lab_ensure_deps.sh"
elif [[ "${SKIP_ONNX_EXPORT:-}" != "1" && -x "$REPO/tools/lab_export_yolo_onnx.sh" ]]; then
  echo "==> SKIP_DEPS=1 — running ONNX export check only..."
  REPO="$REPO" CONTAINER="$CONTAINER" "$REPO/tools/lab_export_yolo_onnx.sh"
else
  echo "==> SKIP_DEPS=1 — not running lab_ensure_deps.sh"
fi

echo ""
echo "Ready. Attach with:"
echo "  cd $UR5E && ./docker-attach.sh"
echo ""
echo "Inside the container:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/workspace/install/setup.bash"
echo "  build_workspace    # if you changed packages"
