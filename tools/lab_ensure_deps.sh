#!/bin/bash
# Ensure Xiangqi Python/system deps inside the running UR5e_Env container.
#
# Run on the lab dev box (host), NOT inside Docker:
#   ~/par_ur5e_xiangqi/tools/lab_ensure_deps.sh
#
# Idempotent: skips work if a marker file exists in the container.
# After ./docker-build.sh or a fresh container, run this once (or use
# lab_build_xiangqi_image.sh for a permanent image — see docs/lab_docker_deps.md).
set -euo pipefail

CONTAINER="${CONTAINER:-ros2}"
MARKER="${XIANGQI_DEPS_MARKER:-/opt/xiangqi_deps_installed_v1}"
REPO="${REPO:-$HOME/par_ur5e_xiangqi}"
DEPS_SCRIPT="$REPO/tools/lab_container_deps.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' is not running. Start UR5e_Env first:"
  echo "  cd ~/UR5e_Env && ./docker-start.sh"
  exit 1
fi

_deps_satisfied() {
  docker exec "$CONTAINER" bash -c '
    source /opt/ros/humble/setup.bash
    python3 -c "import ultralytics, py_trees, flask, pyffish" >/dev/null 2>&1 &&
    command -v fairy-stockfish >/dev/null
  ' 2>/dev/null
}

if docker exec "$CONTAINER" test -f "$MARKER" 2>/dev/null && _deps_satisfied; then
  echo "OK: Xiangqi deps already present ($MARKER in $CONTAINER)."
  exit 0
fi

if _deps_satisfied; then
  docker exec -u root "$CONTAINER" touch "$MARKER" 2>/dev/null || true
  echo "OK: Xiangqi deps verified (marker updated)."
  exit 0
fi

if [[ ! -f "$DEPS_SCRIPT" ]]; then
  echo "Missing $DEPS_SCRIPT — sync par_ur5e_xiangqi to the lab host first."
  exit 1
fi

echo "Installing Xiangqi deps into container '$CONTAINER' (may take several minutes)..."
docker cp "$DEPS_SCRIPT" "$CONTAINER:/tmp/lab_container_deps.sh"
docker exec -u root "$CONTAINER" bash /tmp/lab_container_deps.sh

# Fallback if pip block was skipped earlier (e.g. old script exited on git clone).
if ! docker exec "$CONTAINER" python3 -c "import pyffish" 2>/dev/null; then
  echo "==> Installing Fairy-Stockfish + pyffish (fallback)..."
  docker exec -u root "$CONTAINER" bash -lc '
    set -e
    if ! command -v fairy-stockfish >/dev/null 2>&1; then
      rm -rf /opt/fairy-stockfish
      git clone --depth=1 https://github.com/fairy-stockfish/Fairy-Stockfish.git /opt/fairy-stockfish
      make -C /opt/fairy-stockfish/src -j"$(nproc)" ARCH=x86-64-modern build largeboards=yes
      cp /opt/fairy-stockfish/src/stockfish /usr/local/bin/fairy-stockfish
      chmod +x /usr/local/bin/fairy-stockfish
    fi
    pip3 install --no-cache-dir /opt/fairy-stockfish
  '
fi

echo "Building py_trees_ros in workspace (if present)..."
docker exec -u rosuser -w /home/rosuser/workspace "$CONTAINER" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  if [[ -d src/py_trees_ros ]]; then
    colcon build --packages-select py_trees_ros_interfaces py_trees_ros --symlink-install
  fi
'

echo "Verifying imports..."
if ! docker exec "$CONTAINER" bash -c '
  source /opt/ros/humble/setup.bash
  python3 -c "import ultralytics, py_trees, flask, pyffish; print(\"pip OK\")"
  command -v fairy-stockfish >/dev/null && echo "fairy-stockfish OK"
'; then
  echo "ERROR: dependency verification failed (see above)."
  exit 1
fi

docker exec -u rosuser -w /home/rosuser/workspace "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash
  source install/setup.bash 2>/dev/null || true
  python3 -c "import py_trees_ros; print(\"py_trees_ros OK\")"
' || {
  echo "WARN: py_trees_ros not importable — build xiangqi workspace after colcon build."
}

docker exec -u root "$CONTAINER" touch "$MARKER"
echo "Done. Marker written: $MARKER"
echo "Relaunch: source ~/workspace/install/setup.bash && ros2 launch xiangqi_bringup xiangqi_system.launch.py simulation_mode:=false"
