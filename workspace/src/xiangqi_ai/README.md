# xiangqi_ai

**Tier 3 (deliberative) game logic and move generation**: authoritative rules/state, human move inference, and pluggable engines.

## `game_manager_node`

Central **finite-state machine** over `GameState` (`IDLE`, `WAITING_HUMAN`, `DETECTING_MOVE`, `COMPUTING_AI`, `EXECUTING_MOVE`, `GAME_OVER`, …).

**Flow (typical, robot plays Red):**

1. `/xiangqi/new_game` resets FEN to standard Xiangqi start, history, and either jumps to **AI first move** or waits for human depending on `robot_plays_red`.
2. After the robot finishes a move, state → `WAITING_HUMAN`; publishes `/xiangqi/start_watching` so vision arms the turn detector.
3. When `/xiangqi/human_move_detected` is true, the manager takes the latest `BoardState` grid and **`_infer_move_from_board`**: enumerate **legal** moves with **pyffish**, apply each to get a candidate FEN, convert that FEN to the same `int8[90]` encoding as vision, and pick the move whose grid **matches** observation. If none match, publish an illegal/ambiguous alert and return to watching.
4. On success, **apply** the human move to internal FEN/history via pyffish, check terminal conditions, then call **`GetBestMove`** on `ai_engine_node`.
5. On AI reply, **apply** the AI move to FEN (for bookkeeping), set state to `EXECUTING_MOVE`, and publish the coordinate string on `/xiangqi/execute_move` for the behaviour tree.
6. When the planner finishes motion it publishes `/xiangqi/robot_move_complete`; the manager returns to step 2.

`GameStatus` and `MoveHistory` stream to the dashboard throughout.

## `ai_engine_node`

- Exposes **`get_best_move`** and **`set_engine`** services.
- Holds two backends: `FairyStockfishEngine` (subprocess UCI, Xiangqi variant, skill level) and `MinimaxEngine` (custom search—see below).
- `GetBestMove` request chooses depth- vs time-bounded search; publishes `EngineInfo` for the UI.

## `minimax_engine.py` + `evaluation.py`

**Custom second algorithm** (UG requirement):

- **Legal moves** and position I/O from **pyffish** (rules are not reimplemented).
- **Search**: iterative deepening, alpha–beta pruning, time cutoff, move ordering (captures/checks emphasised).
- **Evaluation**: material balance + hand-tuned piece–square tables + mobility/king-safety style terms in `evaluation.py`.

## `fairy_stockfish_engine.py`

- Spawns / speaks to the `fairy-stockfish` binary over UCI (`UCI_Variant xiangqi`), parses `bestmove`, maps skill/time options.

All FEN and move legality ground truth for the game manager goes through **pyffish** (`VARIANT = 'xiangqi'`).
