#!/usr/bin/env bash
# Generate official OpenCV ArUco test sheet. Prefer lab container if host lacks cv2.aruco.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

run_local() {
  python3 tools/generate_aruco_test_a4.py --verify --debug "$@"
}

if python3 -c "import cv2; assert hasattr(cv2.aruco, 'generateImageMarker')" 2>/dev/null; then
  run_local "$@"
  exit 0
fi

if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx 'ros2'; then
  echo "Using ros2 container (python3-opencv)..."
  docker exec ros2 python3 "/home/rosuser/workspace/../par_ur5e_xiangqi/tools/generate_aruco_test_a4.py" \
    --verify --debug "$@"
  exit 0
fi

echo "No local OpenCV ArUco and no running 'ros2' container."
echo "  pip install opencv-contrib-python-headless numpy"
echo "  python3 tools/generate_aruco_test_a4.py --verify"
exit 1
