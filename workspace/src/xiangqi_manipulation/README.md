# xiangqi_manipulation

**Tier 1 (reactive)** motion and gripper: Xiangqi goals → VXLab UR5e + OnRobot RG2.

## `manipulation_node`

- **Action server** `/xiangqi/pick_and_place` (`xiangqi_msgs/PickAndPlace`).
- **Services** `std_srvs/Trigger`: `/xiangqi/move_to_scan_pose`, `/xiangqi/move_to_initial_pose`.

**Hardware motion** (when `board_calibration.yaml` has taught joint poses):

| Phase | Method |
|-------|--------|
| Scan / homing | Joint-space `/move_action` from `scan_joint_positions` |
| Board pick/place (preferred) | **4-patch bilinear joint interpolation** from taught corner, e-file, and rank-midpoint joints — no OMPL between taught configs |
| Fallback | OMPL `/move_action` + Cartesian `/par_moveit/waypoint_move` for vertical descend/lift when joint teach-in is incomplete |
| Gripper | `/rg2/set_width` (`GripperSetWidth`) |

**Simulation** (`simulation_mode:=true`): skips real clients; short sleeps per phase so the behaviour tree can run without drivers.

Pick-and-place sequence: open → approach pick → grasp → lift → transit → approach place → place → release → clear.

## `move_translator.py` (library, not a node)

Loaded by the planner (`SetupMoveCoordinates`) from the same `board_calibration.yaml` as vision:

- Parses coordinate moves (`h0g2` → file/rank).
- **`grid_to_world`**: bilinear TCP interpolation from four taught corners when `calibration_corners_base` is present; else rigid `board_to_base_tf` + spacing.
- **`interpolate_*_joints`**: 2×2 patch bilinear over taught joint configs (corners + optional e-file and rank-5 midpoints).
- Graveyard poses from taught `graveyard_*_joints` (Step 3 of `calibration_tool`).

## `gripper_controller_node`

- Service `/xiangqi/gripper_control` (`GripperControl`) — stable API for the stack.
- Forwards to RG2 when not in simulation; publishes `/xiangqi/gripper_active`.

## `safety_monitor_node`

- Inputs: UR `/ur_hardware_interface/safety_mode`, dashboard `/xiangqi/emergency_stop`.
- Outputs: `/xiangqi/estop` (`Bool`), `/xiangqi/safety_status` (`String`). BT and game manager respect e-stop.

## `test_moveit_move` (lab smoke test)

Does not launch the full Xiangqi stack. Requires `arm_drivers`, pendant **Play**, and `moveit_config_driver`.

```bash
source install/setup.bash
ros2 run xiangqi_manipulation test_moveit_move --check
ros2 run xiangqi_manipulation test_moveit_move --ompl
ros2 run xiangqi_manipulation test_moveit_move --cartesian
ros2 run xiangqi_manipulation test_moveit_move --move e5 e7
ros2 run xiangqi_manipulation test_moveit_move --capture e5 e7 --captured-red
```

Graveyard modes need graveyard joints in `board_calibration.yaml`.
