# Xiangqi UR5e Dockerfile
# Extends the VXLab Kibibibit/UR5e_Env base image with all Xiangqi project dependencies.
#
# Lab hardware:
#   Arm:     UR5e (controlled via UR ROS 2 driver)
#   Gripper: OnRobot RG2 two-finger gripper (110 mm max, 40 N max)
#            — driver already in base image (onrobot_rg2_driver)
#            — communicates via Modbus TCP to EyeBox at 10.234.6.47:502
#   Camera:  Intel RealSense (USB3, driver in base image)
#
# Place this file alongside the original UR5e_Env Dockerfile, or reference it via
# the docker-compose.yml override.

# Must match the image tag produced by ./docker-build.sh in Kibibibit/UR5e_Env.
# Reference tree uses docker-compose.yml `image: ros:humble` (local lab image overwrites the hub name).
ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

USER root

# --- System packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    libopencv-dev \
    python3-opencv \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    && rm -rf /var/lib/apt/lists/*

# --- Python packages ---
# pymodbus: required by onrobot_rg2_driver (Modbus TCP to RG2 via EyeBox)
# ultralytics: YOLOv8n for piece detection
# flask/flask-socketio/eventlet/flask-cors: web dashboard
# numpy<2: matches ROS Humble cv_bridge (NumPy 2 breaks cv_bridge boost bindings)
# pyffish: installed from same Fairy-Stockfish tree as the binary (see below)
RUN pip3 install --no-cache-dir \
    pymodbus==2.5.3 \
    ultralytics \
    flask \
    flask-cors \
    flask-socketio \
    eventlet \
    scipy \
    opencv-python-headless \
    'numpy>=1.23,<2'

# YOLO weights: place xiangqi_kaggle_v1_best.pt under workspace/src/xiangqi_vision/models/
# before colcon, or use tools/fetch_yolo_weights.py / env XIANGQI_YOLO_DOWNLOAD_URL at runtime.

# --- Fairy-Stockfish + pyffish: single source (aligned move notation) ---
WORKDIR /opt
RUN git clone --depth=1 https://github.com/fairy-stockfish/Fairy-Stockfish.git fairy-stockfish
WORKDIR /opt/fairy-stockfish/src
RUN make -j$(nproc) ARCH=x86-64-modern build largeboards=yes \
    && cp stockfish /usr/local/bin/fairy-stockfish \
    && chmod +x /usr/local/bin/fairy-stockfish
WORKDIR /opt/fairy-stockfish
RUN pip3 install --no-cache-dir . \
    && python3 -c "import pyffish as sf; sf.set_option('VariantPath',''); print('pyffish', sf.__file__)"

# Verify binary + pyffish agree on xiangqi UCI notation
COPY tools/docker_verify_fsf_pyffish.py /opt/tools/docker_verify_fsf_pyffish.py
RUN python3 /opt/tools/docker_verify_fsf_pyffish.py

# --- Xiangqi NNUE weights (optional but recommended for strong play) ---
# Downloaded at runtime via the engine wrapper if not present; can be baked in here:
# RUN mkdir -p /opt/nnue && \
#     wget -q -O /opt/nnue/xiangqi.nnue \
#     https://github.com/fairy-stockfish/Fairy-Stockfish/releases/download/fairy-sf_14/xiangqi-9c65e5ba5a.nnue

# --- Add helper alias for xiangqi system ---
RUN echo '\nalias xiangqi_system="ros2 launch xiangqi_bringup xiangqi_system.launch.py"' \
    >> /home/rosuser/.bashrc

USER rosuser
WORKDIR /home/rosuser/workspace
