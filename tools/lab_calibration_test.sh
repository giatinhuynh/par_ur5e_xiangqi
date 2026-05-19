#!/bin/bash
# Lab calibration test — run ON the dev box (vxlab@10.234.7.84).
# Usage:
#   ./tools/lab_calibration_test.sh copy-config   # host: copy YAML into workspace/config
#   ./tools/lab_calibration_test.sh attach      # print docker attach + env
#   ./tools/lab_calibration_test.sh t1-camera   # terminal 1 commands
#   ./tools/lab_calibration_test.sh t2-view     # terminal 2 commands
#   ./tools/lab_calibration_test.sh t3-arm      # terminal 3 commands
#   ./tools/lab_calibration_test.sh t4-calibrate  # terminal 4 commands

set -euo pipefail

UR5E="${UR5E:-$HOME/UR5e_Env}"
WS="$UR5E/workspace"

copy_config() {
  mkdir -p "$WS/config"
  cp -f "$WS/src/xiangqi_bringup/config/board_calibration.yaml" "$WS/config/"
  if [ ! -f "$WS/config/manipulation_config.yaml" ]; then
    cp -f "$WS/src/xiangqi_bringup/config/manipulation_config.yaml" "$WS/config/"
    echo "Installed: $WS/config/manipulation_config.yaml (optional mirror for taught poses)"
  fi
  echo "Installed: $WS/config/board_calibration.yaml"
  grep -E 'grid_spacing_mm|piece_diameter' "$WS/config/board_calibration.yaml" || true
}

docker_env() {
  cat <<'EOF'
source /opt/ros/humble/setup.bash
source ~/workspace/install/setup.bash
source ~/workspace/.helper_scripts/helper-aliases.sh
export DISPLAY=:0
EOF
}

case "${1:-help}" in
  copy-config)
    copy_config
    ;;
  attach)
    echo "cd $UR5E && ./docker-attach.sh"
    echo "Then inside container:"
    docker_env
    ;;
  t1-camera)
    docker_env
    echo "realsense_driver"
    ;;
  t2-view)
    docker_env
    echo "ros2 topic hz /camera/color/image_raw"
    echo "# optional GUI:"
    echo "ros2 run rqt_image_view rqt_image_view"
    ;;
  t3-arm)
    docker_env
    echo "# Teach pendant: External Control ON, E-stop released, Play"
    echo "arm_drivers"
    echo "# In another attach (optional): moveit_config_driver"
    ;;
  t4-calibrate)
    docker_env
    echo "build_workspace xiangqi_vision xiangqi_bringup"
  echo "ros2 run tf2_ros tf2_echo base_link tool0   # should stream after arm_drivers + Play"
  echo "ros2 run xiangqi_vision calibration_tool"
  echo "# SPACE = capture ArUco; ENTER at each corner (TCP from TF, not /tool_pose)"
    ;;
  help|*)
    cat <<EOF
Calibration test — open 4 terminals on the dev box (or 4x docker-attach).

  Terminal 1 (camera):  $0 t1-camera
  Terminal 2 (check):   $0 t2-view
  Terminal 3 (arm):     $0 t3-arm
  Terminal 4 (calibrate): $0 t4-calibrate

First on host: $0 copy-config
Docker running: cd $UR5E && ./docker-start.sh

Config: $WS/config/board_calibration.yaml (grid_spacing_mm 61.25 for 4xA3 mat)
EOF
    ;;
esac
