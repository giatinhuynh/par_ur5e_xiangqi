#!/bin/bash
# Build ur5e_xiangqi:latest on the lab PC (deps baked into the image).
#
# One-time (or after UR5e_Env base rebuild):
#   ~/par_ur5e_xiangqi/tools/lab_build_xiangqi_image.sh
#
# Then start with:
#   ~/par_ur5e_xiangqi/tools/docker-start-xiangqi.sh
#
# (Uses ur5e_xiangqi:latest automatically when the image exists.)
set -euo pipefail

REPO="${REPO:-$HOME/par_ur5e_xiangqi}"
UR5E="${UR5E:-$HOME/UR5e_Env}"
BASE_IMAGE="${BASE_IMAGE:-ros:humble}"
TAG="${TAG:-ur5e_xiangqi:latest}"

if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
  echo "Base image '$BASE_IMAGE' not found. Build UR5e_Env first:"
  echo "  cd $UR5E && ./docker-build.sh -y"
  exit 1
fi

echo "Building $TAG from $REPO/Dockerfile (BASE_IMAGE=$BASE_IMAGE)..."
docker build -f "$REPO/Dockerfile" --build-arg "BASE_IMAGE=$BASE_IMAGE" -t "$TAG" "$REPO"

echo ""
echo "Built $TAG"
echo "Start with:"
echo "  $REPO/tools/docker-start-xiangqi.sh"
echo "  cd $UR5E && ./docker-attach.sh"
