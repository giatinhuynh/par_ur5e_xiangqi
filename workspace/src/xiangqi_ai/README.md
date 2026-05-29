# xiangqi_ai

**Tier 3 (deliberative)** game state, human move inference, and move generation.

## `game_manager_node`

FSM over game phases (`IDLE`, `WAITING_HUMAN`, `DETECTING_MOVE`, `COMPUTING_AI`, `EXECUTING_MOVE`, `GAME_OVER`, …).

**Modes** (`/xiangqi/game_mode` from dashboard):

| Mode | Behaviour |
|------|-----------|
| `ai_vs_human` | Human on board (vision) or dashboard clicks (sim). Robot plays the other color. |
| `ai_vs_ai` | Both sides automated — instant moves in sim; full pick-and-place on hardware. |

**Typical flow (AI vs Human, human = Red):**

1. `/xiangqi/new_game` — scan board (hardware) or reset FEN (sim).
2. After robot move → `WAITING_HUMAN`; **`/xiangqi/move_to_scan_pose`** then **`/xiangqi/start_watching`** for vision turn detection.
3. On `/xiangqi/human_move_detected` — infer move by matching vision grid to pyffish legal moves (`human_move_grid_tolerance` in `game_config.yaml`).
4. `GetBestMove` on `ai_engine_node` (async; e-stop can cancel).
5. Publish `/xiangqi/ai_move_command`; wait for `/xiangqi/ai_command_ack` and `/xiangqi/ai_execution_result` matching `dispatch_id`.
6. On `ROBOT_MOVE_COMPLETE` — apply pending FEN; on verify/motion failure — discard pending move (optional `trust_robot_move_after_verify_fail` on hardware).

Publishes `/xiangqi/game_status`, `/xiangqi/move_history`. Subscribes `/xiangqi/board_state`, planner results, e-stop.

## `ai_engine_node`

- Services: `get_best_move`, `set_engine`.
- Backends: **`FairyStockfishEngine`** (UCI, Xiangqi variant, skill 1–20) and **`MinimaxEngine`** (iterative deepening, alpha–beta, custom eval in `evaluation.py`).
- Legal moves and FEN via **pyffish** only (`VARIANT = 'xiangqi'`).
- Publishes `/xiangqi/engine_info` for the dashboard. Minimax can show NNUE eval from a shallow FSF search (`minimax_nnue_display_eval` in `game_config.yaml`).

## Key files

| File | Role |
|------|------|
| `minimax_engine.py` | Custom search (UG second algorithm) |
| `evaluation.py` | Heuristic evaluation |
| `fairy_stockfish_engine.py` | UCI subprocess wrapper |
| `move_resolver.py` | Move parsing / eval perspective helpers |
