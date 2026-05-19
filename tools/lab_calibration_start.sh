#!/bin/bash
# Start background calibration stack inside ros2 container (run on dev box host).
set -euo pipefail

UR5E="${UR5E:-$HOME/UR5e_Env}"
LOG="$UR5E/workspace/log/calibration_session"
mkdir -p "$LOG"

if ! docker ps --format '{{.Names}}' | grep -qx ros2; then
  echo "Starting docker..."
  cd "$UR5E"
  USER_UID="$(id -u)" USER_GID="$(id -g)" USERNAME=rosuser DISPLAY="${DISPLAY:-:0}" ./docker-start.sh
fi

RUN='source /opt/ros/humble/setup.bash && source ~/workspace/install/setup.bash && export DISPLAY=:0'

stop_bg() {
  docker exec ros2 bash -lc "pkill -f 'realsense2_camera|rs_launch' 2>/dev/null || true"
  docker exec ros2 bash -lc "pkill -f 'drivers.launch|arm_drivers' 2>/dev/null || true" || true
}

start_bg() {
  local name="$1"
  shift
  echo "Starting $name -> $LOG/${name}.log"
  docker exec -d ros2 bash -lc "$RUN && $* >> $LOG/${name}.log 2>&1"
}

"$UR5E/workspace/tools/lab_calibration_test.sh" copy-config

stop_bg
sleep 2

start_bg camera "ros2 launch realsense2_camera rs_launch.py"
echo "Waiting for camera..."
sleep 8

if docker exec ros2 bash -lc "$RUN && timeout 6 ros2 topic hz /camera/color/image_raw 2>&1" | grep -q 'average rate'; then
  echo "OK: /camera/color/image_raw publishing"
else
  echo "WARN: camera topic not ready yet — see $LOG/camera.log"
fi

echo ""
echo "=== Next (on dev box monitor) ==="
echo "1) Teach pendant: External Control ON, Play"
echo "2) Start arm drivers in another attach:"
echo "   cd $UR5E && ./docker-attach.sh"
echo "   source ~/workspace/.helper_scripts/helper-aliases.sh && export DISPLAY=:0 && arm_drivers"
echo "3) Calibrate:"
echo "   ros2 run xiangqi_vision calibration_tool"
echo ""
echo "Logs: $LOG/"
echo "Tail camera: tail -f $LOG/camera.log"
