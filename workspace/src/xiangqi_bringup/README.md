# xiangqi_bringup

**Bringup only**: launch files and YAML parameters. No runtime algorithms—this package wires the other packages together.

## Launch files

| File | Behaviour |
|------|-----------|
| `launch/xiangqi_system.launch.py` | Starts the full Xiangqi stack: `vision_node`, `manipulation_node`, `gripper_controller_node`, `safety_monitor_node`, `task_planner_node`, `ai_engine_node`, `game_manager_node`, `dashboard_node`. Launch arguments: `simulation_mode`, `engine_type`, `difficulty`. **Does not** start UR5e, MoveIt, or camera drivers—those are started separately via the lab (`arm_drivers`, `moveit_config_driver`). |
| `launch/xiangqi_sim.launch.py` | Includes `xiangqi_system.launch.py` with `simulation_mode:=true` and defaults suited to no Fairy-Stockfish binary (typically `minimax` + lower difficulty). |

## Config (`config/`)

| File | Contents |
|------|----------|
| `vision_config.yaml` | Camera topic, YOLO `model_path`, `calibration_file`, detection thresholds, turn-detector stability and poll rate. |
| `game_config.yaml` | Engine defaults, AI time limits / depth, robot side, etc. (merged with launch args where overlapped). |
| `manipulation_config.yaml` | Gripper widths/force, approach/transit heights, simulation flag passthrough. |
| `board_calibration.yaml` | **Template** in package share; the **real** file is produced under `workspace/config/` by `calibration_tool` and must match `vision_config.yaml`’s `calibration_file`. |

Install rules copy these into the package share directory so `get_package_share_directory('xiangqi_bringup')` resolves them at launch.
