#!/bin/bash
# Install Xiangqi Python deps inside the running UR5e_Env ros:humble container.
# Run on the lab dev box: docker exec -u root ros2 bash /home/rosuser/workspace/../par_ur5e_xiangqi/tools/lab_container_deps.sh
set -euo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as root inside the container (docker exec -u root ros2 bash ...)"
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

# Refresh ROS apt key (lab images often have an expired EXPKEYSIG)
if [[ -f /etc/apt/sources.list.d/ros2.list ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true
  sed -i 's|signed-by=.*|signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]|' \
    /etc/apt/sources.list.d/ros2.list 2>/dev/null || true
fi

apt-get update -qq
apt-get install -y --no-install-recommends \
  build-essential cmake git python3-pip python3-dev libopencv-dev python3-opencv \
  ros-humble-py-trees ros-humble-py-trees-ros ros-humble-py-trees-ros-interfaces \
  || {
    echo "apt py-trees packages failed; falling back to pip for py-trees only"
    apt-get install -y --no-install-recommends build-essential cmake git python3-pip python3-dev \
      libopencv-dev python3-opencv
    pip3 install --no-cache-dir py-trees
  }

# Uninstall shadowing user-site packages (e.g. pre-baked numpy 2.x and opencv 4.13)
# that override global installations and crash ROS Humble cv_bridge.
if id -u rosuser >/dev/null 2>&1; then
  echo "Cleaning up conflicting user-level packages for rosuser..."
  su rosuser -c "pip3 uninstall -y numpy opencv-python opencv-python-headless 2>/dev/null || true"
fi
pip3 uninstall -y numpy opencv-python opencv-python-headless 2>/dev/null || true

pip3 install --no-cache-dir \
  pymodbus==2.5.3 ultralytics flask flask-cors flask-socketio eventlet scipy \
  'opencv-python>=4.6.0,<4.10.0' 'numpy>=1.23,<2'
# ONNX stack last — its deps can upgrade numpy/opencv; re-pin for ROS cv_bridge.
pip3 install --no-cache-dir onnx onnxruntime
pip3 install --no-cache-dir --force-reinstall \
  'opencv-python>=4.6.0,<4.10.0' 'numpy>=1.23,<2'

WS=/home/rosuser/workspace/src
if ! python3 -c "import py_trees_ros" 2>/dev/null; then
  echo "Ensuring py_trees_ros sources in workspace/src (colcon build runs separately)..."
  if [[ ! -d "$WS/py_trees_ros" ]]; then
    git clone --depth=1 -b release/2.0.x https://github.com/splintered-reality/py_trees_ros.git "$WS/py_trees_ros"
  fi
  if [[ ! -d "$WS/py_trees_ros_interfaces" ]]; then
    git clone --depth=1 -b release/2.0.x https://github.com/splintered-reality/py_trees_ros_interfaces.git "$WS/py_trees_ros_interfaces"
  fi
  chown -R rosuser:rosuser "$WS/py_trees_ros" "$WS/py_trees_ros_interfaces" 2>/dev/null || true
fi

install_fairy_stockfish() {
  echo "Building Fairy-Stockfish + pyffish ..."
  rm -rf /opt/fairy-stockfish
  git clone --depth=1 https://github.com/fairy-stockfish/Fairy-Stockfish.git /opt/fairy-stockfish
  make -C /opt/fairy-stockfish/src -j"$(nproc)" ARCH=x86-64-modern build largeboards=yes
  cp /opt/fairy-stockfish/src/stockfish /usr/local/bin/fairy-stockfish
  chmod +x /usr/local/bin/fairy-stockfish
  pip3 install --no-cache-dir /opt/fairy-stockfish
}

if ! command -v fairy-stockfish >/dev/null 2>&1 || ! python3 -c "import pyffish" 2>/dev/null; then
  install_fairy_stockfish
fi

python3 -c "import ultralytics, onnxruntime, py_trees, pyffish; print('deps OK')"
