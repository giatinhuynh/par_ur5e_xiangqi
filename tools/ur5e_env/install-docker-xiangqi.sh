#!/bin/bash
# Install docker-xiangqi.sh into ~/UR5e_Env (run on the lab host once).
set -euo pipefail

REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
UR5E="${UR5E:-$HOME/UR5e_Env}"
SRC="$REPO/tools/ur5e_env/docker-xiangqi.sh"
DEST="$UR5E/docker-xiangqi.sh"

if [[ ! -d "$UR5E" ]]; then
  echo "UR5e_Env not found: $UR5E"
  exit 1
fi

cp -f "$SRC" "$DEST"
chmod +x "$DEST"
ln -sf docker-xiangqi.sh "$UR5E/docker.sh"
echo "Installed: $DEST"
echo "Shortcut:  $UR5E/docker.sh -> docker-xiangqi.sh"
echo "Usage:"
echo "  cd $UR5E"
echo "  ./docker.sh          # or ./docker-xiangqi.sh"
echo "  ./docker-attach.sh"
