# Autonomous Xiangqi-Playing UR5e Cobot

A fully autonomous robotic system that plays Chinese Chess (Xiangqi) against a human opponent using a Universal Robots UR5e collaborative arm, overhead RealSense camera, **OnRobot RG2 two-finger parallel gripper** (Modbus TCP via the lab EyeBox), and a Three-Tier hierarchical software architecture in ROS 2 Humble.

## System Architecture

```
Tier 3 – Deliberative:  Game Manager Node  +  AI Engine Node (Fairy-Stockfish / Custom Minimax)
Tier 2 – Sequencing:    Task Planner Node  (Behavior Tree via py_trees_ros)
Tier 1 – Reactive:      Vision Node  +  Manipulation Node  +  Gripper Controller  +  Safety Monitor
Cross-cutting:          Web Dashboard (Flask + SocketIO at http://localhost:5000)
```

## Quick Start (VXLab Docker Environment)

### 1. Build the Docker image

```bash
cd ~/UR5e_Env
./docker-build.sh      # builds base image
# Then rebuild with Xiangqi additions:
docker build -f ~/par_ur5e_xiangqi/Dockerfile -t ur5e_xiangqi .
```

### 2. Copy packages into the workspace

```bash
cp -r ~/par_ur5e_xiangqi/workspace/src/* ~/UR5e_Env/workspace/src/
```

### 3. Build the ROS 2 workspace

```bash
./docker-attach.sh
cd ~/workspace
build_workspace
```

### 4. Calibrate the board (one-time)

```bash
# With camera running:
ros2 run xiangqi_vision calibration_tool
```

Follow the on-screen instructions to capture the ArUco markers and record TCP positions at the 4 board corners.

### 5. Train/place the YOLOv8 model

Place the trained `.pt` file (e.g. `xiangqi_kaggle_v1_best.pt` from Kaggle) at:
```
/home/rosuser/workspace/models/xiangqi_kaggle_v1_best.pt
```
(`vision_config.yaml` points `model_path` here; copy into `workspace/models/` on the host so the bind-mounted workspace exposes it.)

### 6. Start the robot drivers (as usual)

```bash
arm_drivers
# (in new terminal)
moveit_config_driver
```

### 7. Launch the Xiangqi system

```bash
# In a new docker terminal:
ros2 launch xiangqi_bringup xiangqi_system.launch.py

# Or for simulation (no hardware):
ros2 launch xiangqi_bringup xiangqi_sim.launch.py
```

### 8. Open the dashboard

Navigate to **http://10.234.7.84:5000** in any browser.

Click **New Game** to start. The robot (Red) moves first.

---

## Package Overview

| Package | Tier | Description |
|---------|------|-------------|
| `xiangqi_msgs` | – | Custom ROS 2 messages, services, actions |
| `xiangqi_vision` | Reactive | ArUco board detection, YOLOv8n piece recognition, turn detection |
| `xiangqi_ai` | Deliberative | Fairy-Stockfish wrapper + custom minimax engine + game manager |
| `xiangqi_manipulation` | Reactive | MoveIt2 pick-and-place, gripper control, safety monitor |
| `xiangqi_planner` | Sequencing | py_trees_ros Behavior Tree for move orchestration |
| `xiangqi_dashboard` | Cross-cutting | Flask web dashboard with live board, AI analysis, controls |
| `xiangqi_bringup` | – | Launch files and YAML configuration |

## AI Engines

Two engines are available, switchable via the dashboard or ROS parameter:

- **Fairy-Stockfish** (primary): UCI-protocol professional engine with optional Xiangqi NNUE evaluation. Skill level 1-20.
- **Custom Minimax** (second algorithm): Original implementation with iterative-deepening alpha-beta pruning, hand-crafted piece-square tables, and pyffish for legal move generation.

## Key Files

- `Dockerfile` – Docker image extension with all dependencies
- `workspace/src/xiangqi_bringup/config/` – All YAML configuration
- `workspace/src/xiangqi_vision/xiangqi_vision/board_detector.py` – ArUco + homography
- `workspace/src/xiangqi_ai/xiangqi_ai/minimax_engine.py` – Custom AI engine
- `workspace/src/xiangqi_ai/xiangqi_ai/evaluation.py` – Piece-square tables
- `workspace/src/xiangqi_planner/xiangqi_planner/task_planner_node.py` – BT runner

## References

- [Star-Robot/chinese-chess-robot](https://github.com/Star-Robot/chinese-chess-robot) – YOLOv7 dataset and detection approach
- [fairy-stockfish/Fairy-Stockfish](https://github.com/fairy-stockfish/Fairy-Stockfish) – UCI engine
- [Kibibibit/UR5e_Env](https://github.com/Kibibibit/UR5e_Env) – VXLab base Docker environment
