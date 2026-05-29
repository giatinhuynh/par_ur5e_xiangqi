# xiangqi_vision

**Tier 1 (reactive) perception**: ArUco board warp, YOLO piece detection, grid publishing, human turn detection, calibration tooling.

Vision does not move the arm. Scan timing is coordinated by the game manager and planner (`/xiangqi/move_to_scan_pose` before watching and before post-move verify).

## Executables

| Node | Command |
|------|---------|
| `vision_node` | `ros2 run xiangqi_vision vision_node` (normally via bringup) |
| `calibration_tool` | `ros2 run xiangqi_vision calibration_tool` |
| `vision_preprocess_experiment` | Side-by-side preprocessing comparison (does not affect live node) |

## Pipeline (`vision_node`)

1. Subscribe to `camera_topic` (default `/camera/camera/color/image_raw` on lab RealSense).
2. Timer at `poll_rate_hz` (~4 Hz on hardware).
3. **ArUco** (`BoardDetector`): dictionary `DICT_4X4_50`, IDs **0–3** on sheet corners → homography → warp to 800×890 top-down image.
4. Optional **piece preprocessing** on warped image (`piece_preprocess_*` params).
5. **YOLO** (`PieceDetector`) → map centres to `(file, rank)` via `board_layout.pixel_to_grid`.
6. **`GridStabilizer`**: per-cell temporal filter — a cell updates only after `grid_smooth_frames` consecutive identical raw readings (reduces dashboard / FSM jitter).
7. Publish `/xiangqi/board_state`, `/xiangqi/debug_image`; run **`TurnDetector`** when `/xiangqi/start_watching` is active.

Service **`get_board_state`**: latest grid; `force_rescan=true` runs one fresh processing cycle.

## Grid convention

- Index: `rank * 9 + file`, **rank 0 = Red / robot side**.
- Values: `0` empty; `±1…±7` piece type (General…Soldier), sign = side.
- Must match `game_manager_node` FEN↔grid encoding and `move_translator` file/rank order.

## Calibration (`calibration_tool` → `board_calibration.yaml`)

| Data | Used for |
|------|----------|
| Live ArUco each frame | Image warp (primary localization) |
| `board_to_base_tf`, `grid_spacing_mm`, corner TCP | Legacy rigid grid; bilinear corners preferred |
| `calibration_corners_*`, midpoints, graveyard joints | Manipulation joint interpolation (see `xiangqi_manipulation`) |
| `scan_joint_positions` | Scan pose homing |

Runtime path: `/home/rosuser/workspace/config/board_calibration.yaml` (must match `calibration_file` in `vision_config.yaml`).

Printing and marker layout: [docs/board_printing_guide.md](../../../docs/board_printing_guide.md).

## Turn detection

After `/xiangqi/start_watching`:

1. Wait for grid change vs reference.
2. Require **`stability_frames`** unchanged ticks (~`stability_frames / poll_rate_hz` seconds).
3. Publish `/xiangqi/human_move_detected`.

`/xiangqi/human_ready` skips stability (demos). Game manager moves to scan pose before watching when possible.

## Tuning

| Parameter | Effect |
|-----------|--------|
| `confidence_threshold` | YOLO detection cutoff |
| `grid_smooth_frames` | Per-cell temporal hold-off (1 = off) |
| `stability_frames` | Human “move complete” stability |
| `piece_preprocess_enabled` / `piece_preprocess_preset` | Warped-board lighting (see `image_preprocess.py`) |

Compare presets without touching live output:

```bash
ros2 run xiangqi_vision vision_preprocess_experiment --ros-args \
  -p model_path:=/home/rosuser/workspace/models/xiangqi_kaggle_v4_best.pt \
  -p calibration_file:=/home/rosuser/workspace/config/board_calibration.yaml \
  -p compare_presets:="['none','clahe','bright_sharp']"
```

View: `/xiangqi/preprocess_experiment_image`. Training and lab capture: [docs/vision_training_guide.md](../../../docs/vision_training_guide.md).

## Weights

Default lab path is set in `xiangqi_bringup/config/vision_config.yaml` (typically `workspace/models/xiangqi_kaggle_v4_best.pt`). Package ships `models/xiangqi_kaggle_v1_best.pt` for sim/offline use. See [models/README.md](models/README.md) and [workspace/models/README.md](../../../workspace/models/README.md).

## Modules

| Module | Role |
|--------|------|
| `board_detector.py` | ArUco, homography, warp, `BoardCalibration` I/O |
| `piece_detector.py` | YOLO class map → grid cells |
| `board_layout.py` | Pixel ↔ grid on warped image |
| `turn_detector.py` | Human move stability FSM |
| `vision_node.py` | ROS node, `GridStabilizer`, publishers |
| `calibration_tool.py` | Interactive teach-in |
| `fen_util.py` | FEN ↔ grid helpers |
| `weights_util.py` | Model path resolution, optional download |
