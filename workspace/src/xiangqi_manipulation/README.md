# xiangqi_manipulation

**Tier 1 (reactive) motion and gripper**: bridges Xiangqi-specific goals to the **VXLab** arm and **OnRobot RG2** stacks.

## `manipulation_node`

- **Action server** `/xiangqi/pick_and_place` (`xiangqi_msgs/PickAndPlace`).
- **Hardware path**: All arm moves use OMPL joint-space planning via MoveIt **`/move_action`** (`moveit_msgs/action/MoveGroup`) — same as RViz Plan & Execute (pick/place, scan, homing). No Cartesian `waypoint_move`. Gripper: **`/rg2/set_width`** (`GripperSetWidth`).
- **Simulation path** (`simulation_mode:=true`): skips real clients and sleeps briefly per phase so the BT can be tested without drivers.

**Executed sequence** (8 logical phases): open gripper → approach above pick → descend to pick → close on piece → lift to transit → move above place → descend → open to release → lift clear. Orientation is fixed “gripper down” for round pieces.

## `gripper_controller_node`

- Exposes **`/xiangqi/gripper_control`** (`GripperControl.srv`) as a stable API for the rest of the stack.
- Internally forwards to the same RG2 action servers as above when not in simulation; publishes `/xiangqi/gripper_active` for the dashboard.

## `safety_monitor_node`

- Subscribes to UR safety topics when available (`/ur_hardware_interface/safety_mode`) and `/xiangqi/emergency_stop` from the dashboard.
- Publishes `/xiangqi/estop` and status strings; planners and BT guard on e-stop.

## `move_translator.py` (library)

Pure geometry (no ROS node):

- Parses 4-character coordinate moves (`h0g2` → file/rank indices).
- Uses **`board_to_base_tf`** and **grid spacing** from calibration to map each intersection to **(x, y, z)** in `base_link`.
- Builds five poses: approach pick, grasp, lift, approach place, place (fixed downward quaternion).
- Maintains **graveyard slot** positions (defaults are placeholders in base frame—tune for your table layout).

Used by `xiangqi_planner`’s `SetupMoveCoordinates` when a `MoveTranslator` is supplied on the blackboard.
