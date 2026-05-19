#!/bin/bash
# Install Xiangqi Python deps inside the running UR5e_Env ros:humble container.
# Run on the lab dev box: docker exec -u root ros2 bash /home/rosuser/workspace/../par_ur5e_xiangqi/tools/lab_container_deps.sh
set -euo pipefail

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

pip3 install --no-cache-dir \
  pymodbus==2.5.3 ultralytics flask flask-cors flask-socketio eventlet scipy \
  opencv-python-headless 'numpy>=1.23,<2'

if ! python3 -c "import py_trees_ros" 2>/dev/null; then
  echo "Installing py_trees_ros ROS packages into workspace/src ..."
  WS=/home/rosuser/workspace/src
  git clone --depth=1 -b release/2.0.x https://github.com/splintered-reality/py_trees_ros.git "$WS/py_trees_ros"
  git clone --depth=1 -b release/2.0.x https://github.com/splintered-reality/py_trees_ros_interfaces.git "$WS/py_trees_ros_interfaces"
  chown -R rosuser:rosuser /home/rosuser/workspace/src/py_trees_ros*
fi

if ! command -v fairy-stockfish >/dev/null 2>&1; then
  echo "Building Fairy-Stockfish + pyffish ..."
  rm -rf /opt/fairy-stockfish
  git clone --depth=1 https://github.com/fairy-stockfish/Fairy-Stockfish.git /opt/fairy-stockfish
  make -C /opt/fairy-stockfish/src -j"$(nproc)" ARCH=x86-64-modern build largeboards=yes
  cp /opt/fairy-stockfish/src/stockfish /usr/local/bin/fairy-stockfish
  chmod +x /usr/local/bin/fairy-stockfish
  pip3 install --no-cache-dir /opt/fairy-stockfish
fi

python3 -c "import ultralytics, py_trees, py_trees_ros, pyffish; print('deps OK')"
