#!/usr/bin/env bash
# Build, launch sim, run dashboard flow checks (mode lock, board updates).
set -eu

WS_HOST="${1:-/mnt/c/Users/Welcome/Documents/GitHub/par_ur5e_xiangqi/workspace}"
REPO="${2:-/mnt/c/Users/Welcome/Documents/GitHub/par_ur5e_xiangqi}"
IMAGE="${3:-ur5e_xiangqi:latest}"
DASH_PORT="${4:-15001}"

docker run --rm -p "${DASH_PORT}:5000" \
  -v "${WS_HOST}:/ws:ro" \
  -v "${REPO}/tools:/tools:ro" \
  "${IMAGE}" bash -lc '
set -e
rm -rf /tmp/xiangqi_ws
cp -a /ws /tmp/xiangqi_ws
cd /tmp/xiangqi_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
pip3 install --no-cache-dir flask-cors "numpy>=1.23,<2" >/tmp/pip_fix.log 2>&1 || true
echo "=== colcon build ==="
colcon build --packages-up-to xiangqi_bringup 2>&1 | tail -5
source install/setup.bash
timeout 80 ros2 launch xiangqi_bringup xiangqi_sim.launch.py engine_type:=minimax 2>&1 | tee /tmp/launch.log &
LAUNCH_PID=$!
sleep 22
echo "=== Basic /api/state ==="
curl -sS --max-time 5 http://127.0.0.1:5000/api/state | python3 -c "
import json,sys
d=json.load(sys.stdin)
g=d.get(\"board_grid\",[])
print(\"pieces:\", sum(1 for x in g if x))
print(\"move_count:\", d.get(\"move_count\"))
print(\"game_status:\", d.get(\"game_status\"))
print(\"game_result:\", d.get(\"game_result\"))
print(\"game_mode:\", d.get(\"game_mode\"))
"
echo "=== Flow test ==="
python3 /tools/check_dashboard_flow.py http://127.0.0.1:5000
FLOW_RC=$?
echo "=== launch errors (if any) ==="
grep -iE "error|fatal|traceback" /tmp/launch.log | tail -15 || true
echo "=== launch tail ==="
tail -25 /tmp/launch.log
kill $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true
exit $FLOW_RC
'
echo "Dashboard URL: http://127.0.0.1:${DASH_PORT}/"
