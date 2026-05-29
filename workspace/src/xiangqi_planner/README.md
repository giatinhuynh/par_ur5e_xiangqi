# xiangqi_planner

**Tier 2 (sequencing)**: `py_trees` / `py_trees_ros` behaviour tree (~10 Hz) executing robot moves after the game manager dispatches them.

Human move detection and pyffish validation live in **`game_manager_node`**, not here. This node runs only after `/xiangqi/ai_move_command`.

## Interfaces

| Direction | Topic / type | Notes |
|-----------|----------------|-------|
| In | `/xiangqi/ai_move_command` (`AiMoveCommand`) | `dispatch_id`, `move`, `is_capture`, `expected_fen` |
| Out | `/xiangqi/ai_command_ack` (`AiCommandAck`) | `accepted=false` when busy |
| Out | `/xiangqi/ai_execution_result` (`AiExecutionResult`) | Always reached via finalize behaviours |
| In | `/xiangqi/estop`, `/xiangqi/game_status` | E-stop clears blackboard move |

Action clients: `/xiangqi/pick_and_place`. Service client: `get_board_state` on `vision_node`.

## Tree structure (`task_planner_node.py`)

```
Root (Sequence)
├── EStopGuard (fail if /xiangqi/estop active)
└── Selector MotionOrAbortReport
    ├── Sequence MoveSequence
    │   ├── SetupMoveCoordinates      → move_translator → blackboard poses
    │   ├── CaptureOrSkip             → graveyard PickAndPlace if is_capture
    │   ├── PickAIPiece               → main PickAndPlace
    │   ├── GoToScanPose              → FailureIsSuccess → /xiangqi/move_to_scan_pose
    │   ├── VerifyBestEffort          → Retry VerifyBoardState (grid vs expected_fen)
    │   └── FinalizeRobotMoveAfterVerify → AiExecutionResult
    └── AiMotionFailureFinalizer      → AI_MOTION_FAILED if inner sequence fails
```

`VerifyBoardState` tolerance: `verify_grid_tolerance` in `planner_config.yaml`.

## Modules

| Path | Role |
|------|------|
| `task_planner_node.py` | Builds tree, ticks when `ai_move` set, constructs `MoveTranslator` from `calibration_file` |
| `behaviours/game_behaviours.py` | Setup, capture, verify, scan pose, finalize, e-stop |
| `behaviours/pick_and_place.py` | `py_trees_ros` action clients for `/xiangqi/pick_and_place` |

If calibration is missing or `move_translator` cannot load, `SetupMoveCoordinates` fails and the failure finalizer reports `AI_MOTION_FAILED`.
