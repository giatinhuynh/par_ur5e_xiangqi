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

Start the container, then attach (VXLab convention; see [UR5e_Env](https://github.com/Kibibibit/UR5e_Env) for script details):

```bash
./docker-start.sh      # if the container is not already running
./docker-attach.sh
```

### 2. Copy packages into the workspace

```bash
cp -r ~/par_ur5e_xiangqi/workspace/src/* ~/UR5e_Env/workspace/src/
```

Optional: copy `tools/` from this repo if you want to run `generate_board_svg.py` or Kaggle helper scripts from the same tree as `~/UR5e_Env` (they are not required inside the container for normal play).

### 3. Build the ROS 2 workspace

From a shell **inside** the container (`./docker-attach.sh` if needed):

```bash
cd ~/workspace
build_workspace
```

### 4. Calibrate the board (one-time)

With the RealSense driver publishing **`/camera/color/image_raw`** (same topic as in `vision_config.yaml`):

```bash
ros2 run xiangqi_vision calibration_tool
```

Follow the on-screen steps: set **`grid_spacing_mm`** to match your printed mat (from `docs/board_geometry_A2.yaml` or `docs/board_geometry_A3.yaml` after running `tools/generate_board_svg.py`), capture ArUco with **SPACE**, then teach in the four board corners on the pendant.

Calibration is written to **`/home/rosuser/workspace/config/board_calibration.yaml`** (i.e. `workspace/config/board_calibration.yaml` on the host). That path must match **`calibration_file`** in `vision_config.yaml`. The directory is created automatically on first save.

### 5. Train / place the YOLOv8 model

Place the trained `.pt` file (e.g. `xiangqi_kaggle_v1_best.pt` from Kaggle) at:

```
/home/rosuser/workspace/models/xiangqi_kaggle_v1_best.pt
```

Copy it into `workspace/models/` on the host so the bind-mounted workspace exposes it inside the container. The path is set in `workspace/src/xiangqi_bringup/config/vision_config.yaml` (`model_path`). For a different filename, either update that YAML or override at launch:

```bash
ros2 run xiangqi_vision vision_node --ros-args -p model_path:=/path/to/your.pt
```

Training options: full lab pipeline in [docs/vision_training_guide.md](docs/vision_training_guide.md); Kaggle / scripted flow via [tools/kaggle_train_xiangqi_yolo.py](tools/kaggle_train_xiangqi_yolo.py) (see [docs/README.md](docs/README.md) for the tools index).

### 5b. Verify vision on hardware (after model + calibration)

With the system or vision stack running and the RealSense publishing:

- Inspect overlaid detections: topic **`/xiangqi/debug_image`** (e.g. `rqt_image_view` / RViz image).
- If boxes flicker or confidence is wrong, tune **`confidence_threshold`** in `vision_config.yaml` (same file as `model_path`).

The node runs YOLO on the **ArUco-warped** board image; if quality is poor on the real mat but good on the training dataset, capture additional **lab** images and fine-tune (see the vision training guide).

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

# Or for simulation (no hardware; default minimax engine in sim launch):
ros2 launch xiangqi_bringup xiangqi_sim.launch.py
```

Optional launch overrides (full system):

```bash
ros2 launch xiangqi_bringup xiangqi_system.launch.py simulation_mode:=true engine_type:=minimax difficulty:=10
```

See `xiangqi_system.launch.py` for declared arguments (`simulation_mode`, `engine_type`, `difficulty`).

### 8. Open the dashboard

The dashboard node listens on **port 5000** inside the container (`0.0.0.0:5000`). From another machine on the lab network, open **`http://<host-ip>:5000`** (the lab PC is often `10.234.7.84`; use `localhost` only when browsing from the same machine with port forwarding).

Click **New Game** to start. The robot (Red) moves first.

---

## Documentation

| Resource | Contents |
|----------|----------|
| [docs/README.md](docs/README.md) | Index of physical setup, vision/AI guides, and `tools/` scripts |
| [docs/vision_training_guide.md](docs/vision_training_guide.md) | YOLOv8 dataset layout, training, validation, ONNX export, class map |
| [docs/board_printing_guide.md](docs/board_printing_guide.md) | Board mat SVG, printing, ArUco layout |
| [docs/pieces_guide.md](docs/pieces_guide.md) | Piece procurement / 3D print notes |

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
- `workspace/src/xiangqi_bringup/config/` – Launch parameters; **`vision_config.yaml`**, **`game_config.yaml`**, **`manipulation_config.yaml`**, **`board_calibration.yaml`** (template only — runtime file is under `workspace/config/` after calibration)
- `workspace/models/` – YOLO weights (`.pt`); not tracked in git (see `.gitignore`)
- `workspace/src/xiangqi_vision/xiangqi_vision/board_detector.py` – ArUco + homography
- `workspace/src/xiangqi_vision/xiangqi_vision/piece_detector.py` – YOLO class order (must match training labels)
- `workspace/src/xiangqi_ai/xiangqi_ai/minimax_engine.py` – Custom AI engine
- `workspace/src/xiangqi_ai/xiangqi_ai/evaluation.py` – Piece-square tables
- `workspace/src/xiangqi_planner/xiangqi_planner/task_planner_node.py` – BT runner

## References

- [Star-Robot/chinese-chess-robot](https://github.com/Star-Robot/chinese-chess-robot) – YOLOv7 dataset and detection approach
- [fairy-stockfish/Fairy-Stockfish](https://github.com/fairy-stockfish/Fairy-Stockfish) – UCI engine
- [Kibibibit/UR5e_Env](https://github.com/Kibibibit/UR5e_Env) – VXLab base Docker environment
