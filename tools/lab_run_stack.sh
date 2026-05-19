#!/bin/bash
# Start VXLab drivers via workspace helper scripts (inside ros2 container).
# Run on dev box host: ~/UR5e_Env/workspace/tools/lab_run_stack.sh [stop|status]
set -euo pipefail

CONTAINER="${CONTAINER:-ros2}"
LOG="${LOG:-/home/rosuser/workspace/log/stack}"
HELPER="/home/rosuser/workspace/.helper_scripts"
RUN="source /opt/ros/humble/setup.bash && source /home/rosuser/workspace/install/setup.bash && export DISPLAY=\${DISPLAY:-:0}"

dc() { docker exec "$CONTAINER" bash -lc "$1"; }
dc_bg() { docker exec -d "$CONTAINER" bash -lc "$1"; }

stop_all() {
  dc "pkill -f xiangqi_system.launch 2>/dev/null || true"
  dc "pkill -f task_planner 2>/dev/null || true"
  dc "pkill -f vision_node 2>/dev/null || true"
  dc "pkill -f manipulation_node 2>/dev/null || true"
  dc "pkill -f par_moveit_config.launch 2>/dev/null || true"
  dc "pkill -f move_group 2>/dev/null || true"
  dc "pkill -f drivers.launch 2>/dev/null || true"
  dc "pkill -f ur_robot_driver 2>/dev/null || true"
  dc "pkill -f realsense2_camera 2>/dev/null || true"
  dc "pkill -f rs_launch 2>/dev/null || true"
  sleep 2
  echo "Stopped stack processes."
}

status() {
  echo "=== Processes ==="
  dc "pgrep -af 'realsense|drivers.launch|moveit|xiangqi|vision_node' 2>/dev/null | head -20 || echo none"
  echo "=== Topics ==="
  dc "$RUN && ros2 topic hz /camera/camera/color/image_raw 2>&1" | tail -2 || true
  dc "$RUN && ros2 action list 2>/dev/null | grep -E 'waypoint|pick' || true"
}

start_helpers() {
  dc "mkdir -p $LOG"
  stop_all

  echo "Starting realsense_driver..."
  dc_bg "$RUN && bash $HELPER/driver-realsense-start.sh >> $LOG/camera.log 2>&1"
  sleep 8

  echo "Starting arm_drivers (no rviz; camera via realsense_driver)..."
  dc_bg "$RUN && ros2 launch par_pkg drivers.launch.py robot_ip:=10.234.6.49 ur_type:=ur5e launch_rviz:=false low_poly:=true gripper:=rg2 enable_camera:=false description_file:=/home/rosuser/workspace/src/par_pkg/urdf/ur_assembly.urdf.xacro moveit_config_file:=/home/rosuser/workspace/src/par_pkg/srdf/ur_assembly.srdf.xacro >> $LOG/arm.log 2>&1"
  sleep 12

  echo "Starting moveit_config_driver (--no-rviz)..."
  dc_bg "$RUN && bash $HELPER/driver-moveit-start.sh --no-rviz >> $LOG/moveit.log 2>&1"
  sleep 15

  echo "=== Helper stack status ==="
  status
}

start_xiangqi() {
  echo "Starting xiangqi_system..."
  dc_bg "$RUN && ros2 launch xiangqi_bringup xiangqi_system.launch.py >> $LOG/xiangqi.log 2>&1"
  sleep 10
  dc "$RUN && ros2 node list 2>/dev/null | grep xiangqi || true"
  echo "Dashboard: http://10.234.7.84:5000"
  tail -5 "$(dirname "$LOG")/stack/xiangqi.log" 2>/dev/null || docker exec "$CONTAINER" tail -5 "$LOG/xiangqi.log" 2>/dev/null || true
}

case "${1:-start}" in
  stop) stop_all ;;
  status) status ;;
  helpers) start_helpers ;;
  xiangqi) start_xiangqi ;;
  start)
    start_helpers
    start_xiangqi
    ;;
  *)
    echo "Usage: $0 [start|stop|status|helpers|xiangqi]"
    ;;
esac
