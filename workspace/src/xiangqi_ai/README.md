# xiangqi_ai

**Tier 3 (deliberative) game logic and move generation**: authoritative rules/state, human move inference, and pluggable engines.

## `game_manager_node`

Central **finite-state machine** over `GameState` (`IDLE`, `WAITING_HUMAN`, `DETECTING_MOVE`, `COMPUTING_AI`, `EXECUTING_MOVE`, `GAME_OVER`, …).

**Game modes** (dashboard `/xiangqi/game_mode`):

- **`ai_vs_human`**: Human moves on the physical board (vision) or in simulation (board clicks). Robot plays the opposite color.
- **`ai_vs_ai`**: Robot plays both sides on the physical board (or applies moves instantly in simulation).

**Flow (typical, AI vs Human, human plays Red):**

1. `/xiangqi/new_game` scans the physical board (hardware) or resets to start (sim), then either jumps to **AI first move** or waits for human depending on `robot_plays_red` / human color.
2. After the robot finishes a move, state → `WAITING_HUMAN`; publishes `/xiangqi/start_watching` so vision arms the turn detector (not used in AI vs AI).
3. When `/xiangqi/human_move_detected` is true, the manager takes the latest `BoardState` grid and **`_infer_move_from_board`**: enumerate **legal** moves with **pyffish**, apply each to get a candidate FEN, convert that FEN to the same `int8[90]` encoding as vision, and pick the move whose grid **matches** observation. If none match, publish an illegal/ambiguous alert and return to watching.
4. On success, **apply** the human move to internal FEN/history via pyffish, check terminal conditions, then call **`GetBestMove`** on `ai_engine_node`.
5. On AI reply, the manager **does not** apply the AI move to FEN yet: it calls **`get_best_move`** asynchronously (no blocking spin) so **`/xiangqi/estop`** and timers still run. It snapshots `fen_at_request` for that async call and uses it when generating `expected_fen`. It then publishes **`/xiangqi/ai_move_command`** (`xiangqi_msgs/AiMoveCommand`), starts a short **planner-ack timeout**, sets state to `EXECUTING_MOVE`, and stores the move as pending until step 6. Failures (no service, timeout, bad response, FEN error) return to **`WAITING_HUMAN`** with **`/xiangqi/illegal_move_alert`**. E-stop during **`COMPUTING_AI`** cancels the pending RPC and returns to **`WAITING_HUMAN`**.
6. After motion, planner publishes **`/xiangqi/ai_execution_result`** (`xiangqi_msgs/AiExecutionResult`) with `dispatch_id` + status enum. The manager only accepts results matching the active `dispatch_id`: on `ROBOT_MOVE_COMPLETE` it applies pending FEN and continues; on failure statuses (or `/xiangqi/estop` during `EXECUTING_MOVE`) it discards the pending move (FEN unchanged) and emits **`/xiangqi/illegal_move_alert`**.

`GameStatus` and `MoveHistory` stream to the dashboard throughout.

## `ai_engine_node`

- Exposes **`get_best_move`** and **`set_engine`** services.
- Holds two backends: `FairyStockfishEngine` (subprocess UCI, Xiangqi variant, skill level) and `MinimaxEngine` (custom search-see below).
- `GetBestMove` request chooses depth- vs time-bounded search; publishes `EngineInfo` for the UI.

## `minimax_engine.py` + `evaluation.py`

**Custom second algorithm** (UG requirement):

- **Legal moves** and position I/O from **pyffish** (rules are not reimplemented).
- **Search**: iterative deepening, alpha–beta pruning, time cutoff, move ordering (captures/checks emphasised).
- **Evaluation**: material balance + hand-tuned piece–square tables + mobility/king-safety style terms in `evaluation.py`.

## `fairy_stockfish_engine.py`

- Spawns / speaks to the `fairy-stockfish` binary over UCI (`UCI_Variant xiangqi`), parses `bestmove`, maps skill/time options.

All FEN and move legality ground truth for the game manager goes through **pyffish** (`VARIANT = 'xiangqi'`).
