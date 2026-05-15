#!/usr/bin/env bash
# Run from WSL: sim stack in Docker (avoids /mnt/c colcon permission issues by copying to /tmp).
set -eu

WS_HOST="${1:-/mnt/c/Users/Welcome/Documents/GitHub/par_ur5e_xiangqi/workspace}"
IMAGE="${2:-ur5e_xiangqi:latest}"
# Map dashboard so we do not collide with another container using host port 5000.
DASH_PORT="${3:-15001}"

docker run --rm -p "${DASH_PORT}:5000" -v "${WS_HOST}:/ws:ro" "${IMAGE}" bash -lc '
set -e
rm -rf /tmp/xiangqi_ws
cp -a /ws /tmp/xiangqi_ws
cd /tmp/xiangqi_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
# Older images may lack flask-cors and ship NumPy 2.x (breaks Humble cv_bridge).
pip3 install --no-cache-dir flask-cors "numpy>=1.23,<2" >/tmp/pip_fix.log 2>&1 || true
colcon build --packages-up-to xiangqi_bringup
source install/setup.bash
timeout 55 ros2 launch xiangqi_bringup xiangqi_sim.launch.py engine_type:=minimax self_play:=true 2>&1 | tee /tmp/launch.log &
sleep 14
echo "=== curl /api/state ==="
curl -sS --max-time 5 http://127.0.0.1:5000/api/state -o /tmp/state.json
python3 << "PY"
import json
with open("/tmp/state.json") as f:
    d = json.load(f)
g = d.get("board_grid", [])
print("fen_prefix:", (d.get("fen") or "")[:60])
print("nonzero_cells:", sum(1 for x in g if x))
print("system_state:", d.get("system_state"))
print("move_count:", d.get("move_count"))
print("game_status:", d.get("game_status"))
PY
wait || true
echo "=== launch log tail ==="
tail -40 /tmp/launch.log
'
echo "From Windows/WSL browser (optional): http://127.0.0.1:${DASH_PORT}/"
