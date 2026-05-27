# xiangqi_msgs

ROS 2 interface definitions shared by vision, AI, planner, manipulation, and dashboard. There is **no executable logic** here-only types that encode the data contract between nodes.

## Messages

| Message | Role |
|---------|------|
| `BoardState` | Flat `int8[90]` grid (9×10, index `rank * 9 + file`; rank 0 = robot/red side). `0` = empty; `±1…±7` = piece type (General…Soldier), sign = side. Includes optional FEN/turn fields and mean YOLO confidence. |
| `GameStatus` | High-level FSM string from game manager (`idle`, `waiting_human`, `computing_ai`, …), current FEN, move count, active engine type. |
| `PieceDetection` | Single detection (type, image/world coords, confidence)-used where per-piece telemetry is needed. |
| `MoveHistory` | One move record for the log: coordinate move, side, engine metadata (depth, centipawns, time). |
| `EngineInfo` | Live AI telemetry (engine name, depth, eval, best/ponder moves) for the dashboard. |

## Services

| Service | Role |
|---------|------|
| `GetBoardState` | On-demand board snapshot from vision (`force_rescan` can bypass cache). |
| `GetBestMove` | Request best move from `ai_engine_node`: FEN, depth or time limit, optional engine override; returns UCI-style coordinate move plus search stats. |
| `GripperControl` | Thin command surface for open/close semantics used by higher-level code (implemented by `gripper_controller_node`). |
| `SetEngine` | Switch engine type (`fairystockfish` / `minimax` / `mcts`) and difficulty/search hints at runtime. |

## Actions

| Action | Role |
|--------|------|
| `PickAndPlace` | **Primary motion primitive**: pick pose, place pose, approach/transit heights in `base_link`. `manipulation_node` expands this into waypoint + gripper steps. Feedback reports named phases. |
| `ExecuteMove` | Higher-level “full move” action (move string + FEN + capture hint). Defined for extensibility; the live stack mainly uses `PickAndPlace` from the behaviour tree. |

See the `.msg`, `.srv`, and `.action` files under `msg/`, `srv/`, and `action/` for exact fields.
