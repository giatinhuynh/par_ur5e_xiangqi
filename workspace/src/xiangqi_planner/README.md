# xiangqi_planner

**Tier 2 (sequencing)**: a **py_trees** / **py_trees_ros** behaviour tree ticks at 10 Hz and turns high-level “execute this coordinate move” commands into **PickAndPlace** action goals.

## Data flow

- Subscribes to `/xiangqi/execute_move` (`String`): AI move in coordinate form (e.g. `h0g2`). Stores it on the **blackboard** as `ai_move`.
- Subscribes to `/xiangqi/estop` and `/xiangqi/game_status` for context flags.
- The tree only ticks when `ai_move` is non-null.

## Tree structure (actual code)

Roughly:

1. **Root `Sequence`**: e-stop guard, then move pipeline.
2. **E-stop guard**: `Inverter(IsEstopActive)` — if safety publishes e-stop, the branch fails fast.
3. **`MoveSequence` (`Sequence`, memory=True)**  
   - **`SetupMoveCoordinates`**: reads `ai_move` and **`move_translator`** from the blackboard; calls `MoveTranslator.move_to_poses(move)` to fill `pick_pose`, `place_pose`, approach/transit heights; if **`is_capture`** is true, sets `capture_pick_pose` and `graveyard_pose`.  
   - **Capture subtree (`Selector`)**: if `IsCapture` succeeds on the blackboard, run `PlaceInGraveyardBehaviour` (PickAndPlace from capture square to graveyard); otherwise skip. **`is_capture` defaults to false in `task_planner_node`**; for correct capture handling it should be set true when the current AI move captures (e.g. derived with pyffish before or when handling `/xiangqi/execute_move`).  
   - **`PickPieceBehaviour`**: sends one **`PickAndPlace`** goal using blackboard pick/place poses (full pick-and-place sequence inside `manipulation_node`).  
   - **`VerifyBoardState`**: service call to `get_board_state` with retry decorator (up to 3 attempts).  
   - **`UpdateGameStateAfterMove`**: publishes `/xiangqi/robot_move_complete` so `game_manager_node` can return to human-watching.

Behaviours live under `xiangqi_planner/behaviours/`:

- `game_behaviours.py` — conditions (`IsCapture`, `IsEstopActive`, …), `SetupMoveCoordinates`, verification helper, completion publisher.
- `pick_and_place.py` — `py_trees_ros` action clients targeting `/xiangqi/pick_and_place`.

## Integration note: `move_translator`

`SetupMoveCoordinates` expects a live **`MoveTranslator`** instance on the blackboard (constructed from the same `board_calibration.yaml` as vision). **`task_planner_node` currently initialises `move_translator` to `None`**—for hardware runs you should construct `MoveTranslator` after loading calibration (e.g. in the planner’s `__init__`) so pose setup succeeds.

## Unused / auxiliary behaviours

Some behaviours (e.g. `WaitForHumanMove`, `IsHumanMovePending`) support alternative tree shapes; the tree built in `task_planner_node.py` is the authoritative one for launch.
