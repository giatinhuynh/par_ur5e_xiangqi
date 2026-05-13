# xiangqi_planner

**Tier 2 (sequencing)**: a **py_trees** / **py_trees_ros** behaviour tree ticks at 10 Hz and turns high-level “execute this coordinate move” commands into **PickAndPlace** action goals.

## Data flow

- Subscribes to `/xiangqi/ai_move_command` (`xiangqi_msgs/AiMoveCommand`): atomic **`dispatch_id`**, **`move`**, **`is_capture`**, **`expected_fen`**. Ignores overlapping commands while a move is in flight (different `dispatch_id`).
- Publishes `/xiangqi/ai_command_ack` (`xiangqi_msgs/AiCommandAck`). Sends `accepted=false` when busy so the manager can abort quickly.
- Subscribes to `/xiangqi/estop`, `/xiangqi/game_status`. On e-stop, clears move blackboard so the BT stops ticking a stale goal.
- The tree only ticks when `ai_move` is non-null.

## Tree structure (actual code)

Roughly:

1. **Root `Sequence`**: e-stop guard, then **`Selector` `MotionOrAbortReport`** (outer fallback).
2. **E-stop guard**: `Inverter(IsEstopActive)` — if safety publishes e-stop, the branch fails fast.
3. **First child — `MoveSequence` (`Sequence`, memory=True)**  
   - **`SetupMoveCoordinates`**: reads `ai_move` and **`move_translator`** from the blackboard; calls `MoveTranslator.move_to_poses(move)` to fill `pick_pose`, `place_pose`, approach/transit heights; if **`is_capture`** is true, sets `capture_pick_pose` and `graveyard_pose`. **`is_capture`** and **`expected_board_fen`** come from the same **`/xiangqi/ai_move_command`** message as **`ai_move`**.  
   - **Capture subtree (`Selector`)**: if `IsCapture` succeeds on the blackboard, run `PlaceInGraveyardBehaviour` (PickAndPlace from capture square to graveyard); otherwise skip.  
   - **`PickPieceBehaviour`**: sends one **`PickAndPlace`** goal using blackboard pick/place poses (full pick-and-place sequence inside `manipulation_node`).  
   - **`VerifyBoardState`** (+ **`Retry`** inside **`VerifyBestEffort` / `FailureIsSuccess`**): service call to `get_board_state`; compares grid to **`expected_board_fen`** on the blackboard (from **`ai_move_command`**).  
   - **`FinalizeRobotMoveAfterVerify`**: publishes **`/xiangqi/ai_execution_result`** (`xiangqi_msgs/AiExecutionResult`) with current `dispatch_id` + status enum.  
4. **Second child — `AiMotionFailureFinalizer`**: runs only if the inner sequence fails (bad calibration poses, capture arm failure, aborted PickAndPlace, etc.). Publishes **`/xiangqi/ai_execution_result`** with `status=AI_MOTION_FAILED` and clears move-related blackboard keys so the game manager can abandon the pending AI move without updating FEN.

Behaviours live under `xiangqi_planner/behaviours/`:

- `game_behaviours.py` — conditions (`IsCapture`, `IsEstopActive`, …), `SetupMoveCoordinates`, verification helper, completion publisher.
- `pick_and_place.py` — `py_trees_ros` action clients targeting `/xiangqi/pick_and_place`.

## Calibration

`task_planner_node` builds **`MoveTranslator`** from **`calibration_file`** (see `planner_config.yaml` / launch) using the same YAML as vision; if the file or `board_to_base_tf` is missing, **`move_translator`** stays `None` and **`SetupMoveCoordinates`** fails (triggering **`/xiangqi/ai_motion_failed`**).

## Unused / auxiliary behaviours

Some behaviours (e.g. `WaitForHumanMove`, `IsHumanMovePending`) support alternative tree shapes; the tree built in `task_planner_node.py` is the authoritative one for launch.
