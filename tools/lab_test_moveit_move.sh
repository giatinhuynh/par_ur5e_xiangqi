#!/usr/bin/env bash
# Run MoveIt motion smoke test inside the lab Docker container.
# Usage (on vxlab host, after ./docker-attach.sh):
#   bash /home/rosuser/workspace/../tools/lab_test_moveit_move.sh --check
#   bash /home/rosuser/workspace/../tools/lab_test_moveit_move.sh --ompl
#   bash /home/rosuser/workspace/../tools/lab_test_moveit_move.sh --cartesian
#
# Or from repo root via SSH:
#   ./tools/lab_test_moveit_move.sh --check

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LAB_HOST="${LAB_HOST:-vxlab@10.234.7.84}"
DOCKER_NAME="${DOCKER_NAME:-ros2}"

run_in_container() {
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$DOCKER_NAME"; then
    docker exec "$DOCKER_NAME" bash -lc "$1"
    return
  fi
  ssh -o StrictHostKeyChecking=no "$LAB_HOST" \
    "docker exec $DOCKER_NAME bash -lc $(printf '%q' "$1")"
}

CMD="source /opt/ros/humble/setup.bash && source /home/rosuser/workspace/install/setup.bash && ros2 run xiangqi_manipulation test_moveit_move $*"
echo ">>> $CMD"
run_in_container "$CMD"
