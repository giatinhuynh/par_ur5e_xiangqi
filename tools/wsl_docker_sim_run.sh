#!/usr/bin/env bash
# Start persistent sim + dashboard (detached). Open http://127.0.0.1:15001/
set -eu

WS_HOST="${1:-/mnt/c/Users/Welcome/Documents/GitHub/par_ur5e_xiangqi/workspace}"
IMAGE="${2:-ur5e_xiangqi:latest}"
NAME="${3:-xiangqi_sim_ui}"
PORT="${4:-15001}"

docker rm -f "${NAME}" 2>/dev/null || true

docker run -d --name "${NAME}" -p "${PORT}:5000" -v "${WS_HOST}:/ws:ro" "${IMAGE}" \
  bash -lc 'set -e
rm -rf /tmp/xiangqi_ws
cp -a /ws /tmp/xiangqi_ws
cd /tmp/xiangqi_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
pip3 install -q flask-cors "numpy>=1.23,<2" 2>/dev/null || true
colcon build --packages-up-to xiangqi_bringup
source install/setup.bash
exec ros2 launch xiangqi_bringup xiangqi_sim.launch.py engine_type:=minimax difficulty:=20'

echo "Container ${NAME} starting (build ~20s, then dashboard)."
echo "Open: http://127.0.0.1:${PORT}/"
echo "Logs: docker logs -f ${NAME}"
echo "Stop: docker rm -f ${NAME}"
