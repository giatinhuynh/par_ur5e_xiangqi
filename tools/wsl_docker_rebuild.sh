#!/usr/bin/env bash
# Rebuild ur5e_xiangqi image (aligned pyffish + Fairy-Stockfish) and start sim on port 5000.
set -eu

REPO="${1:-/mnt/c/Users/Welcome/Documents/GitHub/par_ur5e_xiangqi}"
BASE_IMAGE="${2:-ros:humble}"
PORT="${3:-5000}"

cd "${REPO}"
echo "Building ${BASE_IMAGE} -> ur5e_xiangqi:latest (pyffish from /opt/fairy-stockfish)..."
docker build -f Dockerfile --build-arg "BASE_IMAGE=${BASE_IMAGE}" -t ur5e_xiangqi:latest .

exec "${REPO}/tools/wsl_docker_sim_run.sh" \
  "${REPO}/workspace" ur5e_xiangqi:latest xiangqi_sim_ui "${PORT}"
