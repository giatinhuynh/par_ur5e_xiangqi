# xiangqi_dashboard

**Cross-cutting HMI**: Flask + Socket.IO in `dashboard_node` (ROS spin + web server thread).

Does not implement rules or vision — reflects and triggers the running stack.

## ROS I/O

**Subscriptions:** `/xiangqi/board_state`, `/xiangqi/game_status`, `/xiangqi/move_history`, `/xiangqi/engine_info`, `/xiangqi/gripper_active`, `/xiangqi/safety_status`.

**Publishers:** `/xiangqi/new_game`, `/xiangqi/emergency_stop`, `/xiangqi/human_ready`, `/xiangqi/game_mode`.

**Services (via HTTP):** `SetEngine` on `ai_engine_node` for engine and Stockfish skill.

## Board display stability

The UI does not mirror every raw `BoardState` message:

- Grid updates require **N consecutive identical grids** or **high** `detection_confidence` (see `dashboard_node.py`).
- Short **`detecting_move`** labels are debounced to reduce flicker when vision jitters.

Tune upstream: `grid_smooth_frames` and `confidence_threshold` in `vision_config.yaml`.

## Web UI

- **URL:** `http://<host>:5000/` (parameter `port`, bind `0.0.0.0`).
- **Modes:** AI vs AI, AI vs Human; per-side engine (Minimax / Stockfish); Stockfish skill 1–20.
- **Hardware:** AI vs Human — move pieces on the mat; **Confirm move** / `/xiangqi/human_ready` if needed.
- **Simulation:** board clicks for human side; robot moves applied in software.

Default mode: simulation → AI vs AI; hardware → AI vs Human (overridable before **Start Game**).

## API (selected)

| Route | Effect |
|-------|--------|
| `GET /api/state` | JSON snapshot |
| `POST /api/new_game` | `/xiangqi/new_game` |
| `POST /api/emergency_stop` | `/xiangqi/emergency_stop` |
| `POST /api/set_mode` | `/xiangqi/game_mode` |

Socket.IO pushes live updates (`async_mode='threading'`).
