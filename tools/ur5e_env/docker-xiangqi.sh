#!/bin/bash
# Place this file in ~/UR5e_Env/ on the lab PC (copy or symlink from par_ur5e_xiangqi).
#
#   cd ~/UR5e_Env
#   ./docker-xiangqi.sh
#
# Same as upstream ./docker-start.sh, plus Xiangqi deps (and ur5e_xiangqi image if built).
set -euo pipefail

UR5E="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO:-$HOME/par_ur5e_xiangqi}"
START_SCRIPT="$REPO/tools/docker-start-xiangqi.sh"

if [[ ! -f "$START_SCRIPT" ]]; then
  echo "Missing $START_SCRIPT"
  echo "Clone/sync par_ur5e_xiangqi to $REPO on this machine."
  exit 1
fi

export UR5E REPO
exec "$START_SCRIPT"
