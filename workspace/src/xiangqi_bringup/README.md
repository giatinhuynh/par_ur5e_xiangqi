# xiangqi_bringup

Launch files and YAML parameters. No algorithms — wires the other packages together.

## Launch files

| File | Purpose |
|------|---------|
| `launch/xiangqi_system.launch.py` | Full stack: `vision_node`, `manipulation_node`, `gripper_controller_node`, `safety_monitor_node`, `task_planner_node`, `ai_engine_node`, `game_manager_node`, `dashboard_node`. Args: `simulation_mode`, `engine_type`, `difficulty`, `self_play`, `robot_plays_red`, `vision_config_file`, `move_to_initial_pose_on_startup`. Does **not** start UR5e, MoveIt, or camera — use lab `arm_drivers` and `moveit_config_driver` first. |
| `launch/xiangqi_sim.launch.py` | Same as above with `simulation_mode:=true` and `vision_config_sim.yaml`. Defaults: `engine_type:=minimax`, `difficulty:=20`. See [README — Simulation mode](../../../README.md#simulation-mode). |

## Config (`config/`)

| File | Purpose |
|------|---------|
| `vision_config.yaml` | Hardware: camera topic, YOLO `model_path`, `calibration_file`, `confidence_threshold`, `grid_smooth_frames`, `stability_frames`, `poll_rate_hz`, piece preprocessing toggles. |
| `vision_config_sim.yaml` | Simulation vision defaults (lighter polling / thresholds). |
| `game_config.yaml` | Engine type, AI time limits, `human_move_grid_tolerance`, `trust_robot_move_after_verify_fail`. |
| `robot_side.yaml` | `robot_plays_red` — shared by game manager and planner. |
| `manipulation_config.yaml` | Gripper widths/force, MoveIt scaling, scan/initial pose, `calibration_file`, startup homing. |
| `planner_config.yaml` | `calibration_file`, `verify_grid_tolerance` for post-move board check. |
| `board_calibration.yaml` | **Template** in package share; runtime file is written to `workspace/config/board_calibration.yaml` by `calibration_tool` and must match paths in vision/manipulation YAML. |

Configs install to `share/xiangqi_bringup/config/` for `get_package_share_directory('xiangqi_bringup')`.
