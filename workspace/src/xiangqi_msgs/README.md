# xiangqi_msgs

ROS 2 interface definitions shared by vision, AI, planner, manipulation, and dashboard. No runtime logic — only the data contract between nodes.

Authoritative field lists: `msg/`, `srv/`, `action/`. System overview: [repository README](../../../README.md).

## Messages

| Message | Role |
|---------|------|
| `BoardState` | Flat `int8[90]` grid (`rank * 9 + file`; rank 0 = Red / robot side). `0` = empty; `±1…±7` = piece type, sign = side. Optional FEN/turn fields and mean YOLO confidence. |
| `GameStatus` | FSM string (`idle`, `waiting_human`, `detecting_move`, `computing_ai`, `executing_move`, `game_over`, …), `current_fen`, `engine_type`, `game_result` / `game_result_reason`. |
| `MoveHistory` | One log line: coordinate move, side, `thinking_time_sec`, `search_depth`, `evaluation_cp`, `engine_used`. |
| `EngineInfo` | Live AI telemetry for the dashboard (engine, depth, eval, ponder). |
| `PieceDetection` | Single detection record (optional per-piece telemetry). |
| `AiMoveCommand` | Game manager → planner: `dispatch_id`, `move`, `is_capture`, `expected_fen`. |
| `AiCommandAck` | Planner → game manager: `dispatch_id`, `accepted`, `reason`. |
| `AiExecutionResult` | Planner → game manager: `dispatch_id`, `status` (`ROBOT_MOVE_COMPLETE` \| `BOARD_VERIFY_FAILED` \| `AI_MOTION_FAILED`), `message`. |

## Services

| Service | Role |
|---------|------|
| `GetBoardState` | On-demand board snapshot from `vision_node` (`force_rescan` bypasses cache). |
| `GetBoardTransform` | Board pose / transform helper for calibration consumers. |
| `GetBestMove` | FEN + depth/time + optional engine → move + search stats (`ai_engine_node`). |
| `GripperControl` | Target width/force (mm, N) → RG2 via `gripper_controller_node`. |
| `SetEngine` | Runtime switch `fairystockfish` \| `minimax` + difficulty. |

## Actions

| Action | Role |
|--------|------|
| `PickAndPlace` | **Primary motion primitive** — pick/place poses and heights in `base_link`; `manipulation_node` runs the full sequence. |
| `ExecuteMove` | Higher-level move string + FEN + capture hint. Defined for extensibility; the live stack uses `PickAndPlace` + `AiMoveCommand` instead. |

## Standard services (not in this package)

| Service | Server | Role |
|---------|--------|------|
| `std_srvs/Trigger` | `manipulation_node` | `/xiangqi/move_to_scan_pose`, `/xiangqi/move_to_initial_pose` |
