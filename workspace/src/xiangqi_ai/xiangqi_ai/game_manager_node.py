"""
game_manager_node: Deliberative layer (Tier 3) central orchestrator.

Maintains the authoritative game state (FEN), validates human moves,
detects game-over conditions, and dispatches AI move requests.
Publishes GameStatus and MoveHistory for the dashboard and planner.
"""

from __future__ import annotations
from enum import Enum, auto
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Empty, Header, String
from std_srvs.srv import Trigger

from xiangqi_msgs.action import ExecuteMove
from xiangqi_msgs.msg import BoardState, GameStatus, MoveHistory
from xiangqi_msgs.srv import GetBestMove, GetBoardState, SetEngine

from .move_resolver import (
    resolve_to_legal_move,
    parse_move as _resolver_parse_move,
    move_critical_indices,
    origin_grid_index,
)

try:
    import pyffish as sf
    PYFFISH_OK = True
except ImportError:
    PYFFISH_OK = False

VARIANT = 'xiangqi'
STARTING_FEN = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'


class GameState(Enum):
    IDLE = auto()
    WAITING_HUMAN = auto()
    DETECTING_MOVE = auto()
    VALIDATING_MOVE = auto()
    COMPUTING_AI = auto()
    EXECUTING_MOVE = auto()
    GAME_OVER = auto()


class GameManagerNode(Node):
    def __init__(self):
        super().__init__('game_manager_node')

        if not PYFFISH_OK:
            self.get_logger().error('pyffish not installed -- game manager cannot function')

        self.declare_parameter('engine_type', 'fairystockfish')
        self.declare_parameter('robot_plays_red', True)   # Robot is Red (moves first)
        self.declare_parameter('self_play', False)          # Both sides played by AI
        self.declare_parameter('simulation_mode', False)    # Skip planner/arm in simulation
        self.declare_parameter('ai_time_limit', 5.0)
        self.declare_parameter('sim_ai_time_limit', 3.0)
        self.declare_parameter('ai_depth', 0)
        self.declare_parameter('human_move_grid_tolerance', 8)
        self.declare_parameter('trust_robot_move_after_verify_fail', True)
        # Max times to re-dispatch a failed robot move before giving up. The robot
        # keeps retrying (descending lower each attempt) instead of waiting for a human.
        self.declare_parameter('ai_move_max_retries', 8)

        self._engine_type = self.get_parameter('engine_type').value
        self._red_engine_type = str(self._engine_type)
        self._black_engine_type = str(self._engine_type)
        self._active_ai_engine = self._red_engine_type
        self._ai_fail_streak = 0
        self._robot_is_red = self.get_parameter('robot_plays_red').value
        self._self_play = self.get_parameter('self_play').value
        self._simulation_mode = self.get_parameter('simulation_mode').value
        self._delayed_ai_timers: list = []
        self._dashboard_mode = 'ai_vs_human'
        self._human_color = 'red'   # which color the human plays in ai_vs_human
        if not self._simulation_mode:
            # Hardware: default human vs AI until dashboard publishes game_mode.
            self._apply_dashboard_mode(self._dashboard_mode)
        self._ai_time_limit = float(self.get_parameter('ai_time_limit').value)
        self._ai_depth = self.get_parameter('ai_depth').value
        if self._simulation_mode:
            sim_cap = float(self.get_parameter('sim_ai_time_limit').value)
            self._ai_time_limit = min(self._ai_time_limit, max(sim_cap, 0.5))
            if not self._self_play:
                # Dashboard sim: human plays Red; AI plays Black.
                self._robot_is_red = False
        self._current_fen = STARTING_FEN
        self._move_history: list[str] = []

        self._move_count = 0
        self._game_state = GameState.IDLE

        cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self._board_state_sub = self.create_subscription(
            BoardState, '/xiangqi/board_state', self._board_state_cb, 10,
            callback_group=cb_group
        )
        self._human_move_detected_sub = self.create_subscription(
            Bool, '/xiangqi/human_move_detected', self._human_move_detected_cb, 10,
            callback_group=cb_group
        )
        self._human_ready_sub = self.create_subscription(
            Empty, '/xiangqi/human_ready', self._human_ready_cb, 10,
            callback_group=cb_group,
        )
        self._new_game_sub = self.create_subscription(
            Empty, '/xiangqi/new_game', self._new_game_cb, 10,
            callback_group=cb_group
        )
        self._stop_game_sub = self.create_subscription(
            Empty, '/xiangqi/stop_game', self._stop_game_cb, 10,
            callback_group=cb_group,
        )
        self._reset_game_sub = self.create_subscription(
            Empty, '/xiangqi/reset_game', self._reset_game_cb, 10,
            callback_group=cb_group,
        )
        self._game_mode_sub = self.create_subscription(
            String, '/xiangqi/game_mode', self._game_mode_cb, 10,
            callback_group=cb_group,
        )
        self._ai_engines_sub = self.create_subscription(
            String, '/xiangqi/ai_engines', self._ai_engines_cb, 10,
            callback_group=cb_group,
        )
        self._simulate_human_move_sub = self.create_subscription(
            String, '/xiangqi/simulate_human_move', self._simulate_human_move_cb, 10,
            callback_group=cb_group,
        )
        self._human_color_sub = self.create_subscription(
            String, '/xiangqi/human_color', self._human_color_cb, 10,
            callback_group=cb_group,
        )
        self._resync_sub = self.create_subscription(
            Empty,
            '/xiangqi/resync_from_vision',
            self._resync_from_vision_cb,
            10,
            callback_group=cb_group,
        )
        self._starting_fen_sub = self.create_subscription(
            String, '/xiangqi/starting_fen', self._starting_fen_cb, 10,
            callback_group=cb_group,
        )
        self._estop_sub = self.create_subscription(
            Bool, '/xiangqi/estop', self._estop_cb, 10, callback_group=cb_group
        )

        # Publishers
        self._game_status_pub = self.create_publisher(GameStatus, '/xiangqi/game_status', 10)
        self._board_state_pub = self.create_publisher(BoardState, '/xiangqi/board_state', 10)
        self._move_history_pub = self.create_publisher(MoveHistory, '/xiangqi/move_history', 10)
        self._start_watching_pub = self.create_publisher(Bool, '/xiangqi/start_watching', 10)
        self._illegal_move_pub = self.create_publisher(String, '/xiangqi/illegal_move_alert', 10)

        # Service clients
        self._get_best_move_cli = self.create_client(
            GetBestMove, 'get_best_move', callback_group=cb_group
        )
        self._get_board_state_cli = self.create_client(
            GetBoardState, 'get_board_state', callback_group=cb_group
        )
        self._move_to_scan_pose_cli = self.create_client(
            Trigger, '/xiangqi/move_to_scan_pose', callback_group=cb_group
        )

        # Action client: replaces AiMoveCommand/AiCommandAck/AiExecutionResult topics
        self._execute_move_cli = ActionClient(
            self, ExecuteMove, '/xiangqi/execute_move', callback_group=cb_group
        )

        # Status timer
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self._latest_board_state: BoardState | None = None
        self._human_watch_reference_grid: list | None = None
        self._pending_human_move: str | None = None

        # Topic-based scan collection (avoids redundant YOLO calls)
        self._collecting_ai_scan: bool = False
        self._ai_verify_scan_start: float = 0.0
        self._collecting_human_scan: bool = False
        self._human_scan_start: float = 0.0

        # AI move bookkeeping - apply to FEN only after action server confirms completion
        self._pending_ai_move: str | None = None
        self._pending_ai_eval_cp: int = 0
        self._pending_ai_depth: int = 0
        self._pending_ai_elapsed: float = 0.0
        self._active_goal_handle = None   # rclpy ClientGoalHandle for the in-flight ExecuteMove

        # Non-blocking GetBestMove (executor stays responsive for e-stop, etc.)
        self._ai_move_future = None
        self._ai_rpc_timeout_timer = None
        self._ai_service_retry_timer = None
        self._ai_service_retry_count: int = 0
        self._abort_ai_computation: bool = False
        self._ai_request_token: int = 0
        self._active_ai_request_token: int | None = None
        self._ai_fen_at_request: str | None = None
        self._game_result: str = "ongoing"
        self._game_result_reason: str = ""
        # FEN forwarded by dashboard after a successful Scan Board press.
        # Consumed once in _begin_new_game to skip the startup rescan.
        self._prescan_fen: str | None = None

        # Interference detection: track consecutive frames where board differs
        # from reference FEN during AI's turn (COMPUTING_AI only).
        self._ai_interference_streak: int = 0
        self._AI_INTERFERENCE_THRESHOLD: int = 3

        # Move retry: track retries for the current AI move (reset each new move).
        self._ai_move_retry_count: int = 0

        self.get_logger().info('game_manager_node started -- waiting for /xiangqi/new_game')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _starting_fen_cb(self, msg: String) -> None:
        """Dashboard sends the Scan Board result here just before publishing new_game."""
        fen = (msg.data or '').strip()
        if fen:
            self._prescan_fen = fen
            self.get_logger().info(f'Received pre-scanned FEN from dashboard: {fen}')

    def _board_state_cb(self, msg: BoardState) -> None:
        # Ignore logical snapshots we publish ourselves (confidence 1.0).
        if float(msg.detection_confidence) >= 0.999:
            return
        self._latest_board_state = msg
        now = time.monotonic()

        # AI verify scan: collect frames from the background detection stream so we
        # never trigger a redundant synchronous YOLO inference via force_rescan.
        if self._collecting_ai_scan and now >= self._ai_verify_scan_start:
            if self._game_state == GameState.EXECUTING_MOVE:
                self._ai_verify_scan_grids.append(list(msg.grid))
                if len(self._ai_verify_scan_grids) >= self._AI_VERIFY_SCAN_ROUNDS:
                    self._collecting_ai_scan = False
                    finisher = getattr(self, '_verify_scan_finisher', None) or self._finish_ai_verify_recovery
                    finisher()
                    return

        # Human move scan: same approach — collect from the live topic stream.
        elif self._collecting_human_scan and now >= self._human_scan_start:
            if self._game_state == GameState.DETECTING_MOVE:
                self._human_move_scan_grids.append(list(msg.grid))
                if len(self._human_move_scan_grids) >= self._HUMAN_SCAN_ROUNDS:
                    self._collecting_human_scan = False
                    merged = self._merge_startup_grids(self._human_move_scan_grids)
                    synthetic = BoardState()
                    synthetic.grid = [int(v) for v in merged]
                    synthetic.fen = self._grid_to_fen(merged, self._current_fen)
                    synthetic.detection_confidence = 0.5
                    self._latest_board_state = synthetic
                    self._process_human_move()
                    return

        # Interference check: in ai_vs_human hardware mode, if the human touches
        # any piece during COMPUTING_AI, declare the AI the winner.
        if (self._game_state == GameState.COMPUTING_AI
                and not self._simulation_mode):
            self._check_ai_turn_interference(list(msg.grid))

    def _apply_dashboard_mode(self, mode: str) -> None:
        """Apply sim play style from dashboard mode (ai_vs_ai = both sides AI)."""
        self._dashboard_mode = mode
        self._self_play = mode == 'ai_vs_ai'
        if self._self_play:
            self._robot_is_red = True
        else:
            # AI (robot) plays Red when human chose Black - applies to both sim and hardware.
            self._robot_is_red = (self._human_color == 'black')

    @staticmethod
    def _normalize_engine(name: str, default: str = 'minimax') -> str:
        n = (name or '').strip().lower()
        if n in ('stockfish', 'fairy', 'fairystockfish', 'fsf'):
            return 'fairystockfish'
        if n in ('minimax', 'custom'):
            return 'minimax'
        return default

    @staticmethod
    def _fen_active_is_red(fen: str) -> bool | None:
        """Parse FEN active-color field: w=Red, b=Black, None if missing/unknown."""
        parts = (fen or '').split()
        if len(parts) < 2:
            return None
        active = parts[1].strip().lower()
        if active == 'w':
            return True
        if active == 'b':
            return False
        return None

    @staticmethod
    def _set_fen_active_color(fen: str, red_to_move: bool) -> str:
        """Ensure FEN field 2 is w or b (vision often publishes '-')."""
        parts = (fen or STARTING_FEN).split()
        while len(parts) < 6:
            parts.append('-' if len(parts) == 2 else '0' if len(parts) >= 5 else '1')
        parts[1] = 'w' if red_to_move else 'b'
        return ' '.join(parts)

    def _side_to_move_is_red(self) -> bool:
        parsed = self._fen_active_is_red(self._current_fen)
        if parsed is not None:
            return parsed
        return True  # standard Xiangqi: Red moves first when field is '-'

    def _align_fen_side_to_play(self) -> None:
        """Match FEN active color to who should move next (human or robot)."""
        if self._self_play:
            red_to_move = self._side_to_move_is_red()
        elif self._game_state == GameState.WAITING_HUMAN:
            red_to_move = self._human_side_is_red()
        elif self._game_state == GameState.COMPUTING_AI:
            red_to_move = self._robot_is_red
        else:
            return
        self._current_fen = self._set_fen_active_color(self._current_fen, red_to_move)

    def _engine_for_side(self, red: bool) -> str:
        return self._red_engine_type if red else self._black_engine_type

    def _engine_for_current_turn(self) -> str:
        return self._engine_for_side(self._side_to_move_is_red())

    def _ai_engines_cb(self, msg: String) -> None:
        """Dashboard: per-side engine selection (setup only)."""
        if self._game_state not in (GameState.IDLE, GameState.GAME_OVER):
            self.get_logger().warn('Ignoring engine change during active game')
            return
        try:
            data = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid ai_engines JSON: {msg.data!r}')
            return
        if 'red' in data:
            self._red_engine_type = self._normalize_engine(
                data['red'], self._red_engine_type
            )
        if 'black' in data:
            self._black_engine_type = self._normalize_engine(
                data['black'], self._black_engine_type
            )
        self.get_logger().info(
            f'Engines: Red={self._red_engine_type}, Black={self._black_engine_type}'
        )

    def _game_mode_cb(self, msg: String) -> None:
        """Dashboard: ai_vs_ai = robot plays both sides; ai_vs_human = vision or sim clicks."""
        mode = (msg.data or '').strip()
        if mode not in ('ai_vs_ai', 'ai_vs_human'):
            return
        self._dashboard_mode = mode
        if self._game_state not in (GameState.IDLE, GameState.GAME_OVER):
            self.get_logger().warn(
                f'Ignoring game mode change during active game (state={self._game_state.name})'
            )
            return
        prev = self._self_play
        self._apply_dashboard_mode(mode)
        self.get_logger().info(f'Game mode: {mode} (self_play={self._self_play})')

        if prev and not self._self_play:
            self._clear_delayed_ai_timers()
            self._cancel_ai_rpc_in_flight()
            if self._game_state == GameState.COMPUTING_AI:
                self._game_state = GameState.WAITING_HUMAN
                self._tell_vision_to_watch(True)
                self._publish_status()
        elif not prev and self._self_play:
            self._clear_delayed_ai_timers()
            if self._game_state == GameState.WAITING_HUMAN and self._game_result == 'ongoing':
                self._game_state = GameState.COMPUTING_AI
                self._publish_status()
                self._compute_and_emit_ai_move()

    def _human_color_cb(self, msg: String) -> None:
        """Dashboard: which color the human plays in ai_vs_human (red or black)."""
        color = (msg.data or '').strip().lower()
        if color not in ('red', 'black'):
            return
        self._human_color = color
        # Update robot_is_red: AI (robot) plays Red when human plays Black.
        if not self._self_play:
            self._robot_is_red = color == 'black'
        self.get_logger().info(f'Human color: {color} (robot_is_red={self._robot_is_red})')

    def _simulate_human_move_cb(self, msg: String) -> None:
        """Sim-only: human move from dashboard board clicks (not used on hardware)."""
        if not self._simulation_mode or self._self_play:
            return
        move = (msg.data or '').strip()
        if len(move) != 4:
            return
        if self._game_state != GameState.WAITING_HUMAN:
            self.get_logger().warn(
                f'Ignoring dashboard move {move}: not waiting for human (state={self._game_state.name})'
            )
            return
        if not PYFFISH_OK:
            return
        try:
            legal = sf.legal_moves(VARIANT, self._current_fen, [])
        except Exception as e:
            self.get_logger().error(f'Could not list legal moves: {e}')
            return
        if move not in legal:
            alert = String()
            alert.data = f'Illegal move {move} - try again'
            self._illegal_move_pub.publish(alert)
            return
        self.get_logger().info(f'[SIM] Dashboard human move: {move}')
        self._apply_move(move, is_ai=False)
        self._check_game_over()
        if self._game_state == GameState.GAME_OVER:
            self._publish_status()
            return
        self._game_state = GameState.COMPUTING_AI
        self._publish_status()
        self._compute_and_emit_ai_move()

    def _human_ready_cb(self, _: Empty) -> None:
        """Dashboard confirm - process human move without relying on turn-detector IDLE/WATCHING."""
        if self._self_play:
            return
        if self._game_state != GameState.WAITING_HUMAN:
            self.get_logger().warn(
                f'human_ready ignored (state={self._game_state.name})'
            )
            return
        self.get_logger().info('human_ready - confirming human move')
        self._begin_human_move_detection()

    def _human_move_detected_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._self_play:
            return
        if self._game_state != GameState.WAITING_HUMAN:
            return
        self._begin_human_move_detection()

    @staticmethod
    def _move_origin_matches_human(move: str, grid: list, human_red: bool) -> bool:
        """True if reference grid shows a human piece on the move's from-square."""
        idx = origin_grid_index(move)
        if idx is None or idx < 0 or idx >= len(grid):
            return False
        val = grid[idx]
        if val == 0:
            return False
        return val > 0 if human_red else val < 0

    _HUMAN_SCAN_ROUNDS = 5       # frames to collect from the live topic before inferring
    _HUMAN_VERIFY_SETTLE_DELAY = 1.2  # seconds to wait (settle) before collecting starts

    def _begin_human_move_detection(self) -> None:
        """Transition to DETECTING_MOVE and start collecting board frames from the topic."""
        self._tell_vision_to_watch(False)
        self._game_state = GameState.DETECTING_MOVE
        self._publish_status()
        if self._simulation_mode:
            self._process_human_move()
            return
        self._human_move_scan_grids = []
        self._collecting_human_scan = True
        self._human_scan_start = time.monotonic() + self._HUMAN_VERIFY_SETTLE_DELAY

    # ------------------------------------------------------------------
    # AI verify-failure recovery via multi-scan
    # ------------------------------------------------------------------

    _AI_VERIFY_SCAN_ROUNDS = 5      # frames to collect from the live topic before verifying
    _AI_VERIFY_SETTLE_DELAY = 1.5   # seconds to wait (settle) before collecting starts

    def _begin_ai_verify_scan(self) -> None:
        """Start multi-frame verify scan by collecting from the live board-state topic."""
        self._ai_verify_scan_grids = []
        self._collecting_ai_scan = True
        self._ai_verify_scan_start = time.monotonic() + self._AI_VERIFY_SETTLE_DELAY

    def _finish_robot_move_complete(self) -> None:
        """Planner reported success: check for interference, then commit the move."""
        if self._game_state != GameState.EXECUTING_MOVE or not self._pending_ai_move:
            return
        if self._ai_verify_scan_grids:
            merged = self._merge_startup_grids(self._ai_verify_scan_grids)
            synthetic = BoardState()
            synthetic.grid = [int(v) for v in merged]
            synthetic.fen = self._grid_to_fen(merged, self._current_fen)
            synthetic.detection_confidence = 0.5
            self._latest_board_state = synthetic

        # A piece off the planned squares changed (e.g. a piece was removed during
        # execution) — interference, regardless of which side's piece. Game over.
        if self._execution_interference_detected():
            result = 'red_wins' if self._robot_is_red else 'black_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                "Illegal: a piece was moved or removed during the robot's move — game over."
            )
            self._illegal_move_pub.publish(alert)
            self._pending_ai_move = None
            self._active_goal_handle = None
            self._publish_status()
            return

        # Clean board — commit the robot's move and advance the game.
        self._apply_move(
            self._pending_ai_move,
            is_ai=True,
            eval_cp=self._pending_ai_eval_cp,
            depth=self._pending_ai_depth,
            elapsed=self._pending_ai_elapsed,
        )
        self._pending_ai_move = None
        self._active_goal_handle = None
        self._check_game_over()

        if self._game_state == GameState.GAME_OVER:
            self.get_logger().info('Game over - robot finished last move')
            self._publish_status()
            return
        if self._self_play:
            self.get_logger().info('Self-play: computing next AI move immediately')
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()
            return
        self.get_logger().info('Robot move execution confirmed - waiting for human')
        self._game_state = GameState.WAITING_HUMAN
        self._tell_vision_to_watch(True)
        self._publish_status()

    def _finish_ai_verify_recovery(self) -> None:
        """Use merged multi-scan grid to retry AI move verification after BOARD_VERIFY_FAILED."""
        if self._ai_verify_scan_grids:
            merged = self._merge_startup_grids(self._ai_verify_scan_grids)
            synthetic = BoardState()
            synthetic.grid = [int(v) for v in merged]
            synthetic.fen = self._grid_to_fen(merged, self._current_fen)
            synthetic.detection_confidence = 0.5
            self._latest_board_state = synthetic

        # Before recovery: if non-planned squares changed, a piece was touched/removed
        # during execution — that's interference regardless of which side's piece it was.
        if self._execution_interference_detected():
            result = 'red_wins' if self._robot_is_red else 'black_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                "Illegal: a piece was moved or removed during the robot's execution — game over."
            )
            self._illegal_move_pub.publish(alert)
            self._pending_ai_move = None
            self._active_goal_handle = None
            self._publish_status()
            return

        if self._retry_ai_move_lower_grasp():
            return
        if self._commit_pending_ai_after_robot(
            'Board verify failed - committing pending AI move (trust_robot)'
        ):
            return
        if self._recover_ai_move_after_verify_failure():
            return
        self._discard_pending_ai_after_planner_abort(
            'Board verification failed after robot move (FEN unchanged)',
            'Robot move could not be confirmed by vision - use Sync Board or New Game',
        )


    def _cancel_active_execution(self) -> None:
        """Cancel the in-flight ExecuteMove action goal (e-stop, halt, etc.)."""
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._active_goal_handle = None

    def _dispatch_to_planner(
        self,
        ai_move: str,
        expected_fen: str,
        is_capture: bool,
        retry: int = 0,
    ) -> None:
        """Send an ExecuteMove goal to the task_planner action server."""
        if not self._execute_move_cli.server_is_ready():
            self._discard_pending_ai_after_planner_abort(
                'execute_move action server not ready - move aborted',
                'Planner not available - check task_planner_node',
            )
            return

        goal = ExecuteMove.Goal()
        goal.move = ai_move
        goal.expected_fen = expected_fen
        goal.is_capture = is_capture
        goal.retry_attempt = retry

        future = self._execute_move_cli.send_goal_async(goal)
        future.add_done_callback(self._on_execute_move_goal_response)

    def _on_execute_move_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._discard_pending_ai_after_planner_abort(
                'ExecuteMove goal rejected by planner',
                'Planner rejected move command - check task_planner_node',
            )
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_execute_move_result)
        self.get_logger().info(
            f'ExecuteMove goal accepted for {self._pending_ai_move}'
        )

    def _on_execute_move_result(self, future) -> None:
        if self._game_state != GameState.EXECUTING_MOVE:
            return
        self._active_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as e:
            self._discard_pending_ai_after_planner_abort(
                f'ExecuteMove result error: {e}',
                'Planner result error - move aborted; check logs',
            )
            return

        result = wrapped.result
        status_code = int(result.status_code)

        if status_code == ExecuteMove.Result.ROBOT_MOVE_COMPLETE:
            if not self._pending_ai_move:
                self.get_logger().error(
                    'robot_move_complete but no pending AI move - ignoring'
                )
                return
            self._verify_scan_finisher = self._finish_robot_move_complete
            self._begin_ai_verify_scan()
            return

        if status_code == ExecuteMove.Result.BOARD_VERIFY_FAILED:
            self._verify_scan_finisher = self._finish_ai_verify_recovery
            self._begin_ai_verify_scan()
            return

        if status_code == ExecuteMove.Result.AI_MOTION_FAILED:
            if self._redispatch_ai_move('AI motion subtree failed'):
                return
            self._discard_pending_ai_after_planner_abort(
                'AI motion subtree failed - retries exhausted, pending AI move discarded',
                'Robot could not complete the planned move after retries - check '
                'arm/gripper/board; use New Game if the physical board moved',
            )
            return

        self._discard_pending_ai_after_planner_abort(
            f'Unknown ExecuteMove status_code={status_code} - pending move discarded',
            f'Unexpected planner status ({status_code}) - pending move discarded; check logs',
        )

    def _commit_pending_ai_after_robot(self, log_msg: str) -> bool:
        """Apply pending AI move after robot motion; advance game state."""
        move = self._pending_ai_move
        if not move or self._game_state != GameState.EXECUTING_MOVE:
            return False

        trust = bool(self.get_parameter('trust_robot_move_after_verify_fail').value)
        grid = (
            list(self._latest_board_state.grid)
            if self._latest_board_state is not None
            else None
        )
        if not trust:
            if grid is None or not self._pending_move_matches_grid(
                move, self._current_fen, grid
            ):
                return False
            self.get_logger().warn(log_msg)
        else:
            if grid is not None and not self._pending_move_matches_grid(
                move, self._current_fen, grid, loose=True
            ):
                self.get_logger().warn(
                    f'{log_msg} - weak square match; applying {move} anyway'
                )
            else:
                self.get_logger().warn(log_msg)

        self._apply_move(
            move,
            is_ai=True,
            eval_cp=self._pending_ai_eval_cp,
            depth=self._pending_ai_depth,
            elapsed=self._pending_ai_elapsed,
        )
        self._pending_ai_move = None
        self._active_goal_handle = None
        self._check_game_over()
        if self._game_state == GameState.GAME_OVER:
            self._publish_status()
            return True
        if self._self_play:
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()
        else:
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()
        return True

    def _retry_ai_move_lower_grasp(self) -> bool:
        """Re-issue the same AI move with a lower grasp height if the piece was never picked up.

        Keeps retrying (descending lower each attempt) up to ai_move_max_retries while the
        source square still holds the AI's piece, rather than handing control to the human.
        Returns True if a retry was dispatched (caller should return immediately).
        """
        if self._ai_move_retry_count >= int(self.get_parameter('ai_move_max_retries').value):
            return False
        move = self._pending_ai_move
        if not move or self._game_state != GameState.EXECUTING_MOVE:
            return False
        if self._latest_board_state is None:
            return False

        # Check source square: if piece is still there, the robot never picked it up.
        indices = move_critical_indices(move)
        if not indices:
            return False
        ref_grid = self._fen_to_grid(self._current_fen)
        observed = list(self._latest_board_state.grid)
        # from-square: the critical index that has a piece in the reference FEN AND still
        # shows that same piece in the observed grid (i.e. nothing moved).
        # Sign-only: piece still at source if it has the same colour as in ref
        # (YOLO may misidentify type but colour is reliable).
        from_idx = next(
            (i for i in indices
             if ref_grid[i] != 0
             and (observed[i] > 0) == (ref_grid[i] > 0)
             and (observed[i] < 0) == (ref_grid[i] < 0)),
            None,
        )
        if from_idx is None:
            return False  # piece gone from source — partial move, don't retry

        # Destination square occupied in reference → capture move
        is_capture = any(i != from_idx and ref_grid[i] != 0 for i in indices)

        expected_fen_after = ''
        try:
            expected_fen_after = sf.get_fen(VARIANT, self._current_fen, [move])
        except Exception:
            pass

        self._ai_move_retry_count += 1
        self._dispatch_to_planner(move, expected_fen_after, is_capture,
                                  retry=self._ai_move_retry_count)

        self.get_logger().warn(
            f'Board verify failed: source square still occupied — retrying {move} '
            f'with lower grasp (attempt #{self._ai_move_retry_count})'
        )
        alert = String()
        alert.data = f'Retrying robot move {move} with lower grasp height...'
        self._illegal_move_pub.publish(alert)

        self._publish_status()
        return True

    def _recover_ai_move_after_verify_failure(self) -> bool:
        """Infer pending AI move from vision when verify fails (trust_robot disabled path)."""
        if bool(self.get_parameter('trust_robot_move_after_verify_fail').value):
            return False
        move = self._pending_ai_move
        if not move or self._game_state != GameState.EXECUTING_MOVE:
            return False
        if self._latest_board_state is None:
            return False
        grid = list(self._latest_board_state.grid)
        inferred = self._infer_move_from_board(self._current_fen, grid, tolerance=12)
        if inferred != move and not self._pending_move_matches_grid(
            move, self._current_fen, grid, loose=True
        ):
            return False
        return self._commit_pending_ai_after_robot(
            f'Board verify failed but vision matches pending move {move}'
        )

    def _redispatch_ai_move(self, reason: str) -> bool:
        """Re-issue the current pending AI move to the planner (keep retrying).

        Used when the robot fails to execute/verify a move: rather than giving up
        and waiting for a human, the same move is re-sent (the planner descends a bit
        lower each retry_attempt). Returns True if a retry was dispatched.
        """
        move = self._pending_ai_move
        if not move or self._game_state != GameState.EXECUTING_MOVE:
            return False
        max_retries = int(self.get_parameter('ai_move_max_retries').value)
        if self._ai_move_retry_count >= max_retries:
            self.get_logger().error(
                f'{reason} - exhausted {max_retries} robot retries for {move}'
            )
            return False

        is_capture = self._is_capture_move(self._current_fen, move)
        expected_fen_after = ''
        try:
            expected_fen_after = sf.get_fen(VARIANT, self._current_fen, [move])
        except Exception:
            pass

        self._ai_move_retry_count += 1
        self._dispatch_to_planner(move, expected_fen_after, is_capture,
                                  retry=self._ai_move_retry_count)

        self.get_logger().warn(
            f'{reason} - re-dispatching robot move {move} '
            f'(attempt #{self._ai_move_retry_count}/{max_retries})'
        )
        alert = String()
        alert.data = f'Robot move failed — retrying {move} (attempt {self._ai_move_retry_count})...'
        self._illegal_move_pub.publish(alert)

        self._publish_status()
        return True

    def _discard_pending_ai_after_planner_abort(self, log_msg: str, alert_text: str) -> None:
        """Drop pending AI move without applying FEN; return to human watching."""
        if self._game_state != GameState.EXECUTING_MOVE:
            return
        if not self._pending_ai_move:
            self.get_logger().warn(f'{log_msg} - no pending AI move (ignored)')
            return
        self.get_logger().error(log_msg)
        self._pending_ai_move = None
        self._cancel_active_execution()
        alert = String()
        alert.data = alert_text
        self._illegal_move_pub.publish(alert)
        if self._self_play:
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()
        else:
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()

    def _estop_cb(self, msg: Bool) -> None:
        """E-stop during motion or while waiting on the AI engine."""
        if not msg.data:
            return
        if self._game_state == GameState.EXECUTING_MOVE:
            self._cancel_active_execution()
            if self._self_play:
                self._pending_ai_move = None
                self._game_state = GameState.IDLE
                alert = String()
                alert.data = (
                    'E-stop during AI vs AI move - confirm board matches UI, then Start'
                )
                self._illegal_move_pub.publish(alert)
                self._publish_status()
            else:
                self._discard_pending_ai_after_planner_abort(
                    'E-stop asserted during robot AI move - pending move discarded (FEN unchanged)',
                    'E-stop: robot move cancelled in software - confirm physical board matches UI',
                )
            return
        if self._game_state == GameState.COMPUTING_AI:
            self._abort_ai_computation = True
            self._cancel_ai_rpc_in_flight()
            self._active_ai_request_token = None
            if self._self_play:
                self._game_state = GameState.IDLE
                alert = String()
                alert.data = (
                    'E-stop during AI vs AI - cancelled engine request; press Start to resume'
                )
            else:
                self._game_state = GameState.WAITING_HUMAN
                self._tell_vision_to_watch(True)
                alert = String()
                alert.data = (
                    'E-stop during AI thinking - cancelled engine request; '
                    'make a move or New Game'
                )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

    def _recover_ai_computation_failed(self, detail: str) -> None:
        """AI service or FEN computation failed while in COMPUTING_AI - unblock the game."""
        self._clear_ai_rpc_timeout_timer()
        self._clear_ai_service_retry_timer()
        self._ai_move_future = None
        self._active_ai_request_token = None
        self._ai_fen_at_request = None
        self.get_logger().error(detail)
        alert = String()
        alert.data = (
            f'{detail} - make a move on the board or press New Game.'
        )
        self._illegal_move_pub.publish(alert)
        if self._game_state == GameState.COMPUTING_AI:
            if self._self_play:
                self._ai_fail_streak += 1
                if self._ai_fail_streak >= 15:
                    self._ai_fail_streak = 0
                    self._game_state = GameState.GAME_OVER
                    self._game_result = 'unknown'
                    self._game_result_reason = 'ai_engine_failed'
                    alert.data = (
                        f'{detail} - AI vs AI stopped after repeated failures. '
                        'Try Reset or change engines.'
                    )
                    self._illegal_move_pub.publish(alert)
                else:
                    self._schedule_ai_service_retry()
            else:
                self._game_state = GameState.WAITING_HUMAN
                self._tell_vision_to_watch(True)
        self._publish_status()

    def _halt_game(self) -> None:
        """Cancel all AI work and return to idle with the starting position."""
        self._abort_ai_computation = True
        self._ai_request_token += 1
        self._clear_delayed_ai_timers()
        self._cancel_ai_rpc_in_flight()
        self._cancel_active_execution()
        self._pending_human_move = None
        self._pending_ai_move = None
        self._ai_service_retry_count = 0
        self._ai_fail_streak = 0
        self._ai_interference_streak = 0
        self._ai_move_retry_count = 0
        self._current_fen = self._prescan_fen if self._prescan_fen else STARTING_FEN
        self._move_history = []
        self._move_count = 0
        self._game_result = 'ongoing'
        self._game_result_reason = ''
        self._game_state = GameState.IDLE
        if self._simulation_mode:
            self._publish_logical_board_state()
        self._publish_status()

    def _stop_game_cb(self, _: Empty) -> None:
        """Abort current game and return to idle (setup); board reset to start."""
        self.get_logger().info('Stop game - returning to idle')
        self._halt_game()

    def _reset_game_cb(self, _: Empty) -> None:
        """Abort and immediately start a fresh game in the current dashboard mode."""
        self.get_logger().info('Reset game - restarting')
        self._halt_game()
        self._begin_new_game()

    def _new_game_cb(self, _: Empty) -> None:
        self.get_logger().info('New game started')
        self._halt_game()
        self._begin_new_game()

    def _begin_new_game(self) -> None:
        """Start from the initial position (call after _halt_game or from cold idle)."""
        self._abort_ai_computation = False
        self._ai_fen_at_request = None
        self._ai_fail_streak = 0
        self._game_result = 'ongoing'
        self._game_result_reason = ''

        if self._simulation_mode:
            self._apply_dashboard_mode(self._dashboard_mode)
            self._publish_logical_board_state()
            self._kick_off_game()
            return

        # ── Hardware mode ────────────────────────────────────────────────
        self._apply_dashboard_mode(self._dashboard_mode)
        if self._self_play:
            self._game_state = GameState.COMPUTING_AI
        else:
            self._game_state = GameState.WAITING_HUMAN
        self._publish_status()

        # If the dashboard Scan Board button already established a position, use it.
        # _prescan_fen is intentionally NOT cleared here so that Restart also
        # reuses the same scanned position without triggering another vision scan.
        # It is only overwritten when the user presses Scan Board again.
        if self._prescan_fen:
            repaired = self._sanitize_fen_for_engine(self._prescan_fen)
            if repaired != self._prescan_fen:
                self.get_logger().warn(
                    f'Pre-scanned FEN repaired (missing king). '
                    f'Raw: {self._prescan_fen}  Repaired: {repaired}'
                )
            else:
                self.get_logger().info(
                    f'Using pre-scanned FEN from Scan Board: {repaired}'
                )
            self._current_fen = repaired
            self._move_history = []
            self._move_count = 0
            self._kick_off_game()
            return

        # Fallback: no pre-scan available — scan the physical board now.
        if self._get_board_state_cli.service_is_ready():
            self.get_logger().info(
                'New game (hardware): no pre-scan — scanning physical board for starting position…'
            )
            self._startup_scan_retries = 0
            self._startup_scan_grids: list = []
            self._do_startup_scan()
        else:
            self.get_logger().warn(
                'Vision service not ready - starting from STARTING_FEN (place pieces first)'
            )
            self._kick_off_game()

    _STARTUP_SCAN_ROUNDS = 7      # how many frames to accumulate
    _STARTUP_SCAN_INTERVAL = 0.35  # seconds between scans (> camera period at 4 Hz)

    def _do_startup_scan(self):
        req = GetBoardState.Request()
        req.force_rescan = True
        future = self._get_board_state_cli.call_async(req)
        future.add_done_callback(self._on_startup_board_scan_done)

    def _retry_startup_scan(self):
        if hasattr(self, '_startup_retry_timer') and self._startup_retry_timer:
            self.destroy_timer(self._startup_retry_timer)
            self._startup_retry_timer = None
        self._do_startup_scan()

    @staticmethod
    def _merge_startup_grids(grids: list) -> list:
        """Majority-vote merge across all N scans (zeros count as votes too).

        Used for short multi-scan windows (human-move detection, AI verify) where
        a coherent single frame is not guaranteed.
        """
        merged = [0] * 90
        for i in range(90):
            values = [g[i] for g in grids]
            merged[i] = max(set(values), key=values.count)
        return merged

    @staticmethod
    def _best_frame_grid(grids: list) -> list:
        """Return the single frame that detected the most pieces.

        Produces a coherent real snapshot instead of a cell-by-cell reconstruction
        that can mix detections from different moments.
        """
        return max(grids, key=lambda g: sum(1 for v in g if v != 0))

    def _on_startup_board_scan_done(self, future) -> None:
        """Accumulate vision grids over _STARTUP_SCAN_ROUNDS frames, then pick the best.

        The densest frame (most pieces detected) is used as the starting FEN so
        the board state is a coherent snapshot rather than a majority-vote hybrid.
        """
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'Startup board scan failed ({e}) - using STARTING_FEN')
            self._kick_off_game()
            return

        if resp is None or not resp.success:
            if self._startup_scan_retries < 6:
                self._startup_scan_retries += 1
                self.get_logger().info(
                    f'Vision not ready, retrying ({self._startup_scan_retries}/6) in 0.5s...'
                )
                self._startup_retry_timer = self.create_timer(0.5, self._retry_startup_scan)
            else:
                self.get_logger().warn('Board scan failed after retries - using STARTING_FEN')
                self._kick_off_game()
            return

        frame_pieces = sum(1 for v in resp.board_state.grid if v != 0)
        self._startup_scan_grids.append(list(resp.board_state.grid))
        n = len(self._startup_scan_grids)
        self.get_logger().info(
            f'Startup scan {n}/{self._STARTUP_SCAN_ROUNDS} complete '
            f'(pieces detected: {frame_pieces})'
        )

        if n < self._STARTUP_SCAN_ROUNDS:
            # Schedule next scan after a short delay so camera delivers a fresh frame
            self._startup_retry_timer = self.create_timer(
                self._STARTUP_SCAN_INTERVAL, self._retry_startup_scan
            )
            return

        # All rounds done — pick the densest frame and commit
        best_grid = self._best_frame_grid(self._startup_scan_grids)
        raw_fen = GameManagerNode._grid_to_fen(best_grid, STARTING_FEN)
        repaired_fen = self._sanitize_fen_for_engine(raw_fen)
        piece_count = sum(1 for v in best_grid if v != 0)
        if repaired_fen != raw_fen:
            self.get_logger().warn(
                f'Best-frame FEN was incomplete (missing king(s)) - repaired. '
                f'Pieces found: {piece_count}  Raw: {raw_fen}  Repaired: {repaired_fen}'
            )
        else:
            self.get_logger().info(
                f'Startup scan complete (best frame: {piece_count} pieces '
                f'from {self._STARTUP_SCAN_ROUNDS} candidates). FEN: {repaired_fen}'
            )
        self._current_fen = repaired_fen
        self._move_history = []
        self._move_count = 0
        self._kick_off_game()

    def _kick_off_game(self) -> None:
        """Transition into the first game state and begin play."""
        self._apply_dashboard_mode(self._dashboard_mode)
        if self._robot_is_red or self._self_play:
            self._game_state = GameState.COMPUTING_AI
            self._align_fen_side_to_play()
            self._publish_status()
            self._compute_and_emit_ai_move()
        else:
            self._game_state = GameState.WAITING_HUMAN
            self._align_fen_side_to_play()
            self._tell_vision_to_watch(True)
            self._publish_status()

    def _resync_from_vision_cb(self, _: Empty) -> None:
        """Align game FEN with the camera; keeps move history (sync is not a new game)."""
        self.get_logger().warn('Resync requested - adopting vision FEN (history preserved)')
        self._abort_ai_computation = True
        self._cancel_ai_rpc_in_flight()
        self._cancel_active_execution()
        self._pending_ai_move = None
        self._pending_human_move = None

        if not self._get_board_state_cli.service_is_ready():
            alert = String()
            alert.data = 'Resync failed: get_board_state service not available'
            self._illegal_move_pub.publish(alert)
            return

        req = GetBoardState.Request()
        req.force_rescan = True
        future = self._get_board_state_cli.call_async(req)

        def _done(fut):
            try:
                resp = fut.result()
            except Exception as e:
                self.get_logger().error(f'Resync GetBoardState error: {e}')
                alert = String()
                alert.data = f'Resync failed: {e}'
                self._illegal_move_pub.publish(alert)
                return
            if resp is None or not resp.success:
                message = getattr(resp, "message", "no response")
                self.get_logger().error(f'Resync failed: {message}')
                alert = String()
                alert.data = f'Resync failed: {message}'
                self._illegal_move_pub.publish(alert)
                return
            fen = resp.board_state.fen
            if not fen:
                fen = self._grid_to_fen(
                    list(resp.board_state.grid), self._current_fen or STARTING_FEN
                )
            if not fen or not any(resp.board_state.grid):
                self.get_logger().error('Resync failed: empty FEN from vision')
                alert = String()
                alert.data = 'Resync failed: could not build FEN from camera grid'
                self._illegal_move_pub.publish(alert)
                return

            self._current_fen = self._sanitize_fen_for_engine(fen)
            self._align_fen_side_to_play()
            if self._self_play:
                self._game_state = GameState.COMPUTING_AI
                self._publish_status()
                self._compute_and_emit_ai_move()
            else:
                self._game_state = GameState.WAITING_HUMAN
                self._tell_vision_to_watch(True)
                self._publish_status()
            ok = String()
            ok.data = (
                'Board synced from camera. Move history kept - use Restart if the '
                'physical game does not match the log.'
            )
            self._illegal_move_pub.publish(ok)

        future.add_done_callback(_done)

    # ------------------------------------------------------------------
    # Game flow
    # ------------------------------------------------------------------

    def _human_side_is_red(self) -> bool:
        return self._human_color == 'red'

    def _extra_changes_for_move(
        self,
        move: str,
        ref_grid: list,
        observed_grid: list,
        human_sign: int,
        *,
        sign_only: bool = False,
    ) -> list:
        """Return cell indices that changed beyond what a single legal `move` explains.

        A clean move produces exactly:
          - from-square: human piece vanishes  (ref=human, observed=0)
          - to-square:   human piece appears   (observed=human; ref=0 for non-capture,
                         ref=opponent for a capture)
        Any other differing cell is an unexpected extra change (piece removed, piece
        relocated elsewhere, etc.) and is returned in the list.

        When ``sign_only`` is True, only red/black/empty changes count (YOLO type
        noise on an unchanged square is ignored).
        """
        try:
            parsed = _resolver_parse_move(move)
            if parsed is None:
                return []
            (from_f, from_r1), (to_f, to_r1) = parsed
            from_i = (from_r1 - 1) * 9 + (ord(from_f) - ord('a'))
            to_i   = (to_r1   - 1) * 9 + (ord(to_f)   - ord('a'))
        except Exception:
            return []
        extra = []
        for i in range(90):
            old, new = ref_grid[i], observed_grid[i]
            if sign_only:
                if (old > 0) == (new > 0) and (old < 0) == (new < 0):
                    continue
            elif old == new:
                continue
            if i == from_i and old * human_sign > 0 and new == 0:
                continue  # expected: human piece left source
            if i == to_i and new * human_sign > 0:
                continue  # expected: human piece arrived at dest (capture or empty)
            extra.append(i)
        return extra

    def _execution_interference_detected(self) -> bool:
        """Return True if the board changed beyond the pending robot move.

        Anchors each scan frame to the expected post-move state via
        ``_extra_changes_for_move``: only cells that changed *outside* the
        planned from/to squares count.  This ignores YOLO type noise on the
        move squares and also catches pieces appearing on squares that were
        empty before the move (the old pre-move colour-diff skipped those).

        A cell is flagged only when it is an extra change in a near-unanimous
        majority of scan frames (4/5 when n=5).
        """
        move = self._pending_ai_move
        if not move:
            return False
        ref = self._fen_to_grid(self._current_fen)
        planned = move_critical_indices(move)
        robot_sign = 1 if self._robot_is_red else -1

        raw_grids = getattr(self, '_ai_verify_scan_grids', None)
        if not raw_grids and self._latest_board_state is not None:
            raw_grids = [list(self._latest_board_state.grid)]
        if not raw_grids:
            return False

        n = len(raw_grids)
        # Near-unanimity for irreversible game-over (4/5 when n=5).
        majority = max(n - 1, n // 2 + 1)

        extra_counts = [0] * 90
        for g in raw_grids:
            for i in self._extra_changes_for_move(
                move, ref, g, robot_sign, sign_only=True
            ):
                extra_counts[i] += 1

        for i in range(90):
            if i in planned:
                continue
            count = extra_counts[i]
            if count >= majority:
                self.get_logger().warn(
                    f'Execution interference: cell {i} (ref={ref[i]}) extra change in '
                    f'{count}/{n} scan frames'
                )
                return True
        return False

    def _check_ai_turn_interference(self, grid: list) -> None:
        """Declare AI win if any piece has moved while the AI is computing its move.

        Requires _AI_INTERFERENCE_THRESHOLD consecutive differing frames to avoid
        triggering on YOLO jitter.
        """
        ref = self._fen_to_grid(self._current_fen)
        board_changed = any(grid[i] != ref[i] for i in range(90))
        if board_changed:
            self._ai_interference_streak += 1
            if self._ai_interference_streak >= self._AI_INTERFERENCE_THRESHOLD:
                self._ai_interference_streak = 0
                result = 'red_wins' if self._robot_is_red else 'black_wins'
                self._declare_game_over(result, 'illegal_move')
                alert = String()
                alert.data = (
                    "Illegal: pieces were moved during the AI's turn — AI wins."
                )
                self._illegal_move_pub.publish(alert)
                self._publish_status()
        else:
            self._ai_interference_streak = 0

    def _return_to_human_watch(self, alert_text: str | None = None) -> None:
        if alert_text:
            alert = String()
            alert.data = alert_text
            self._illegal_move_pub.publish(alert)
        self._game_state = GameState.WAITING_HUMAN
        self._align_fen_side_to_play()
        self._tell_vision_to_watch(True)
        self._publish_status()

    def _process_human_move(self) -> None:
        """Detect and validate the human move from the current board state."""
        if self._latest_board_state is None:
            self.get_logger().warn('No board state available for human move detection')
            self._return_to_human_watch(
                'No camera board state - check vision, then move again or press Confirm move'
            )
            return

        human_red = self._human_side_is_red()
        side_red = self._side_to_move_is_red()
        if side_red != human_red:
            self.get_logger().info(
                f'FEN active color was {"red" if side_red else "black"} but human is '
                f'{self._human_color} - aligning for move detection'
            )
            self._current_fen = self._set_fen_active_color(self._current_fen, human_red)

        grid = list(self._latest_board_state.grid)
        base_tol = int(self.get_parameter('human_move_grid_tolerance').value)
        ref_grid = self._human_watch_reference_grid
        ref_for_side = ref_grid if ref_grid is not None else self._fen_to_grid(self._current_fen)
        human_sign = 1 if human_red else -1
        opp_sign   = -human_sign

        # Early guard: if an opponent's piece clearly moved (disappeared from one square,
        # appeared at another), call illegal immediately — BEFORE move inference so a
        # coincidentally low-mismatch legal move cannot mask the wrong-color violation.
        appeared_opp = [
            i for i in range(90)
            if (grid[i] * opp_sign > 0) and not (ref_for_side[i] * opp_sign > 0)
        ]
        disappeared_opp = [
            i for i in range(90)
            if (ref_for_side[i] * opp_sign > 0) and grid[i] == 0
        ]
        if len(appeared_opp) == 1 and len(disappeared_opp) == 1:
            ap_i  = appeared_opp[0]
            ap_sq = f"{chr(ord('a') + ap_i % 9)}{ap_i // 9 + 1}"
            self.get_logger().warn(
                f'Human moved opponent piece to {ap_sq} — illegal, game over'
            )
            result = 'black_wins' if human_red else 'red_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                f"Illegal move: you moved your opponent's piece to {ap_sq} — game over."
            )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

        # Definitive single-piece move: if the human's own piece clearly left exactly one
        # square and appeared at exactly one square, the move is unambiguous. Validate it
        # directly rather than relying on tolerance inference (which can pick a spurious,
        # unrelated legal move — e.g. i10h10 — when the real move is illegal).
        appeared_human_def = [
            i for i in range(90)
            if (grid[i] * human_sign > 0) and not (ref_for_side[i] * human_sign > 0)
        ]
        disappeared_human_def = [
            i for i in range(90)
            if (ref_for_side[i] * human_sign > 0) and (grid[i] * human_sign <= 0)
        ]

        # Net loss of the human's own pieces: a legal move never reduces the mover's
        # piece count (a move relocates one piece; a capture removes an OPPONENT piece).
        # If more human pieces vanished than appeared, the human removed their own
        # piece(s) — possibly alongside a real move. Illegal, game over.
        # Allow 1 extra disappeared piece as potential jitter; 2+ is unambiguous removal.
        if len(disappeared_human_def) > len(appeared_human_def) + 1:
            sqs = ', '.join(
                f"{chr(ord('a') + i % 9)}{i // 9 + 1}" for i in disappeared_human_def
            )
            self.get_logger().warn(
                f'Human pieces vanished from {sqs} but only '
                f'{len(appeared_human_def)} reappeared — own piece removed, game over'
            )
            result = 'black_wins' if human_red else 'red_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                'Illegal: you removed your own piece from the board — game over.'
            )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

        if len(appeared_human_def) == 1 and len(disappeared_human_def) == 1:
            from_i = disappeared_human_def[0]
            to_i   = appeared_human_def[0]
            from_sq = f"{chr(ord('a') + from_i % 9)}{from_i // 9 + 1}"
            to_sq   = f"{chr(ord('a') + to_i % 9)}{to_i // 9 + 1}"
            candidate = f"{from_sq}{to_sq}"
            try:
                legal = sf.legal_moves(VARIANT, self._current_fen, [])
            except Exception:
                legal = []
            if candidate in legal:
                # Confirm no extra pieces were disturbed alongside the valid move.
                # sign_only=True ignores YOLO type-jitter (cannon↔rook same colour);
                # only genuine colour changes (disappearing or opponent piece) count.
                extra = self._extra_changes_for_move(candidate, ref_for_side, grid, human_sign, sign_only=True)
                if extra:
                    sqs = ', '.join(f"{chr(ord('a') + i % 9)}{i // 9 + 1}" for i in extra)
                    self.get_logger().warn(
                        f'Interference alongside move {candidate}: extra changes at {sqs} — game over'
                    )
                    result = 'black_wins' if human_red else 'red_wins'
                    self._declare_game_over(result, 'illegal_move')
                    alert = String()
                    alert.data = (
                        'Illegal: extra pieces were moved or removed alongside your move — game over.'
                    )
                    self._illegal_move_pub.publish(alert)
                    self._publish_status()
                    return
                self.get_logger().info(f'Human move (definitive): {candidate}')
                self._apply_move(candidate, is_ai=False)
                self._check_game_over()
                if self._game_state != GameState.GAME_OVER:
                    self._game_state = GameState.COMPUTING_AI
                    self._publish_status()
                    self._compute_and_emit_ai_move()
                return
            self.get_logger().warn(
                f'Human made illegal move {from_sq}->{to_sq} — game over'
            )
            result = 'black_wins' if human_red else 'red_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                f'Illegal move: {from_sq.upper()}->{to_sq.upper()} is not legal — game over.'
            )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

        detected_move = self._infer_move_from_board(
            self._current_fen,
            grid,
            tolerance=base_tol,
            reference_grid=ref_grid,
            human_red=human_red,
        )
        if detected_move is None and base_tol < 16:
            detected_move = self._infer_move_from_board(
                self._current_fen,
                grid,
                tolerance=16,
                reference_grid=ref_grid,
                human_red=human_red,
            )

        if detected_move is not None and not self._move_origin_matches_human(
            detected_move, ref_for_side, human_red
        ):
            self.get_logger().warn(
                f'Rejecting inferred human move {detected_move}: no '
                f'{self._human_color} piece on from-square in reference grid'
            )
            detected_move = None

        if detected_move is None:
            self.get_logger().warn(
                'Could not infer a valid human move from board state '
                f'(tol={base_tol}, pieces_on_board={sum(1 for x in grid if x != 0)})'
            )
            self._return_to_human_watch(
                'Could not read your move from the camera - adjust pieces, '
                'press Confirm move in the dashboard, or Sync board'
            )
            return

        # Validate: if vision unambiguously shows the human's piece appeared at exactly one
        # square AND that square doesn't match the inferred destination, the human placed the
        # piece illegally (e.g. horse to f2 instead of g3).  Reject the move so the human
        # can redo it rather than silently executing a different legal move.
        appeared_human = [
            i for i in range(90)
            if (grid[i] * human_sign > 0) and not (ref_for_side[i] * human_sign > 0)
        ]
        if len(appeared_human) == 1:
            try:
                parsed = _resolver_parse_move(detected_move)
                if parsed is not None:
                    _, (to_file_ch, to_rank_1based) = parsed
                    inferred_dest_idx = (to_rank_1based - 1) * 9 + (ord(to_file_ch) - ord('a'))
                    if appeared_human[0] != inferred_dest_idx:
                        ap_i  = appeared_human[0]
                        ap_sq = f"{chr(ord('a') + ap_i % 9)}{ap_i // 9 + 1}"
                        self.get_logger().warn(
                            f'Vision shows piece at {ap_sq} but inferred move {detected_move} '
                            f'goes to {to_file_ch}{to_rank_1based} — YOLO position mismatch, '
                            'trusting move inference'
                        )
            except Exception as e:
                self.get_logger().warn(f'Placement check error: {e}')

        # Ghost-move guard: if no human piece appeared anywhere on the board and the
        # inferred destination is empty, the piece was physically removed rather than
        # moved. Tolerance-based inference can still "find" a legal move in this case
        # because the from-square being empty is a valid match. Catch it here.
        if not appeared_human:
            try:
                parsed = _resolver_parse_move(detected_move)
                if parsed is not None:
                    _, (to_file_ch, to_rank_1based) = parsed
                    dest_idx = (to_rank_1based - 1) * 9 + (ord(to_file_ch) - ord('a'))
                    if grid[dest_idx] * human_sign <= 0:
                        self.get_logger().warn(
                            f'Ghost move blocked: {detected_move} — no human piece '
                            f'appeared and destination {to_file_ch}{to_rank_1based} is empty'
                        )
                        result = 'black_wins' if human_red else 'red_wins'
                        self._declare_game_over(result, 'illegal_move')
                        alert = String()
                        alert.data = (
                            'Illegal: your piece was removed from the board — game over.'
                        )
                        self._illegal_move_pub.publish(alert)
                        self._publish_status()
                        return
            except Exception as e:
                self.get_logger().warn(f'Ghost move check error: {e}')

        # Final interference check: confirm no extra pieces were disturbed.
        # sign_only=True ignores YOLO type-jitter (cannon↔rook same colour);
        # only genuine colour changes (disappearing or opponent piece) count.
        extra = self._extra_changes_for_move(detected_move, ref_for_side, grid, human_sign, sign_only=True)
        if extra:
            sqs = ', '.join(f"{chr(ord('a') + i % 9)}{i // 9 + 1}" for i in extra)
            self.get_logger().warn(
                f'Interference alongside inferred move {detected_move}: extra changes at {sqs} — game over'
            )
            result = 'black_wins' if human_red else 'red_wins'
            self._declare_game_over(result, 'illegal_move')
            alert = String()
            alert.data = (
                'Illegal: extra pieces were moved or removed alongside your move — game over.'
            )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

        self.get_logger().info(f'Human move detected: {detected_move}')
        self._apply_move(detected_move, is_ai=False)
        self._check_game_over()

        if self._game_state != GameState.GAME_OVER:
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()

    def _clear_ai_rpc_timeout_timer(self) -> None:
        if self._ai_rpc_timeout_timer is not None:
            self._ai_rpc_timeout_timer.cancel()
            self._ai_rpc_timeout_timer = None

    def _clear_ai_service_retry_timer(self) -> None:
        if self._ai_service_retry_timer is not None:
            self._ai_service_retry_timer.cancel()
            self._ai_service_retry_timer = None

    def _cancel_ai_rpc_in_flight(self) -> None:
        self._clear_ai_rpc_timeout_timer()
        self._clear_ai_service_retry_timer()
        if self._ai_move_future is not None and not self._ai_move_future.done():
            try:
                self._get_best_move_cli.remove_pending_request(self._ai_move_future)
            except (AttributeError, RuntimeError) as e:
                self.get_logger().warn(f'Could not cancel AI request: {e}')
        self._ai_move_future = None
        self._active_ai_request_token = None
        self._ai_fen_at_request = None

    def _ai_service_retry_tick(self) -> None:
        self._clear_ai_service_retry_timer()
        if self._game_state != GameState.COMPUTING_AI:
            self._ai_service_retry_count = 0
            return
        self._compute_and_emit_ai_move()

    def _schedule_ai_service_retry(self) -> None:
        if self._ai_service_retry_timer is not None:
            return
        self._ai_service_retry_count += 1
        if self._ai_service_retry_count > 60:
            self._ai_service_retry_count = 0
            self._recover_ai_computation_failed(
                'AI engine service unavailable (timeout waiting for server)'
            )
            return
        self._ai_service_retry_timer = self.create_timer(0.5, self._ai_service_retry_tick)

    def _on_ai_rpc_timeout(self) -> None:
        self._clear_ai_rpc_timeout_timer()
        self._active_ai_request_token = None
        fut = self._ai_move_future
        self._ai_move_future = None
        if fut is not None and not fut.done():
            try:
                self._get_best_move_cli.remove_pending_request(fut)
            except (AttributeError, RuntimeError) as e:
                self.get_logger().warn(f'AI RPC timeout remove_pending_request: {e}')
        if (
            self._game_state == GameState.COMPUTING_AI
            and not self._abort_ai_computation
        ):
            self._recover_ai_computation_failed('AI engine call timed out')

    def _on_get_best_move_done(self, future, token: int) -> None:
        self._clear_ai_rpc_timeout_timer()
        if self._ai_move_future is future:
            self._ai_move_future = None

        if self._active_ai_request_token is None or token != self._active_ai_request_token:
            return

        if self._abort_ai_computation or self._game_state != GameState.COMPUTING_AI:
            return

        try:
            resp = future.result()
        except Exception as e:
            self._recover_ai_computation_failed(f'AI service error: {e}')
            return

        if resp is None or not resp.success:
            self._recover_ai_computation_failed('AI engine returned no move')
            return

        self._ai_fail_streak = 0
        ai_move = resp.best_move
        self.get_logger().info(
            f'AI move: {ai_move} (depth={resp.depth_reached}, eval={resp.evaluation_cp}cp)'
        )

        fen_before_ai = self._ai_fen_at_request or self._current_fen

        # Validate move is legal - pyffish and Stockfish sometimes disagree on coordinate system
        import threading
        if not hasattr(self, '_pyffish_lock'):
            self._pyffish_lock = threading.Lock()

        try:
            with self._pyffish_lock:
                legal = sf.legal_moves(VARIANT, fen_before_ai, [])
            if not legal:
                self._recover_ai_computation_failed('No legal moves - game over?')
                return
            resolved, exact = resolve_to_legal_move(fen_before_ai, ai_move, legal)
            if not exact and resolved != ai_move:
                self.get_logger().warn(
                    f'Engine move {ai_move!r} not in pyffish legal set '
                    f'({len(legal)} moves) - using {resolved!r}'
                )
            elif not exact:
                self.get_logger().warn(
                    f'Engine move {ai_move!r} matched by square parse → {resolved!r}'
                )
            ai_move = resolved
        except Exception as e:
            self.get_logger().warn(f'Could not validate move legality: {e}')

        is_capture = self._is_capture_move(fen_before_ai, ai_move)

        try:
            with self._pyffish_lock:
                expected_fen = sf.get_fen(VARIANT, fen_before_ai, [ai_move])
        except Exception as e:
            self._recover_ai_computation_failed(
                f'Could not compute post-move FEN for {ai_move}: {e}'
            )
            return

        self._ai_move_retry_count = 0  # fresh move, reset retry counter

        self._pending_ai_move = ai_move
        self._pending_ai_eval_cp = resp.evaluation_cp
        self._pending_ai_depth = resp.depth_reached
        self._pending_ai_elapsed = resp.thinking_time_sec

        # --- Simulation mode: apply move directly, no physical robot needed ---
        if self._simulation_mode:
            if self._abort_ai_computation or self._game_state != GameState.COMPUTING_AI:
                self.get_logger().info(
                    f'[SIM] Dropping stale AI move {ai_move} (game halted or not computing)'
                )
                return
            self.get_logger().info(f'[SIM] Applying AI move directly: {ai_move}')
            # Invalidate current token so any in-flight Stockfish response (2nd bestmove)
            # will be dropped by the token guard at the top of this callback
            self._active_ai_request_token = None
            self._ai_move_future = None

            self._apply_move(
                ai_move,
                is_ai=True,
                eval_cp=resp.evaluation_cp,
                depth=resp.depth_reached,
                elapsed=resp.thinking_time_sec,
            )
            self._pending_ai_move = None
            self._check_game_over()
            if self._game_state == GameState.GAME_OVER:
                self._publish_status()
                return
            if self._self_play:
                self._game_state = GameState.COMPUTING_AI
                self._publish_status()
                # Small delay before next AI request so Stockfish has time to flush
                delay = 0.25 if self._simulation_mode else 1.0
                t = self.create_timer(delay, self._delayed_next_ai_move)
                self._delayed_ai_timers.append(t)
            else:
                self._game_state = GameState.WAITING_HUMAN
                self._tell_vision_to_watch(True)
                self._publish_status()
            return
        # --- Hardware mode: dispatch to task_planner via action server ---
        self._game_state = GameState.EXECUTING_MOVE
        self._publish_status()
        self._dispatch_to_planner(ai_move, expected_fen, bool(is_capture), retry=0)

    def _clear_delayed_ai_timers(self) -> None:
        for t in self._delayed_ai_timers:
            try:
                t.cancel()
            except Exception:
                pass
        self._delayed_ai_timers.clear()

    def _delayed_next_ai_move(self) -> None:
        """One-shot timer callback: fires next AI request after a short settle delay."""
        self._clear_delayed_ai_timers()
        if (
            not self._abort_ai_computation
            and self._game_state == GameState.COMPUTING_AI
            and self._self_play
        ):
            self._compute_and_emit_ai_move()

    def _compute_and_emit_ai_move(self) -> None:
        """Start async GetBestMove so the executor can still process e-stop and other I/O."""
        if self._game_state != GameState.COMPUTING_AI:
            return
        if self._ai_move_future is not None and not self._ai_move_future.done():
            self.get_logger().warn('GetBestMove request already in flight')
            return

        self._clear_ai_service_retry_timer()

        if not self._get_best_move_cli.service_is_ready():
            self._schedule_ai_service_retry()
            return

        self._ai_service_retry_count = 0

        if PYFFISH_OK and self._current_fen:
            try:
                fen_stm = self._current_fen.split()[1]
                is_red_turn = fen_stm == 'w'
                if self._check_no_legal_moves(self._current_fen, is_red_turn):
                    self._publish_status()
                    return
            except Exception as e:
                self.get_logger().warn(f'Pre-move terminal check failed: {e}')

        req = GetBestMove.Request()
        self._ai_fen_at_request = self._current_fen
        req.fen = self._ai_fen_at_request
        req.depth = self._ai_depth
        req.time_limit = self._ai_time_limit
        engine = self._engine_for_current_turn()
        self._active_ai_engine = engine
        req.engine_type = engine

        self._ai_request_token += 1
        token = self._ai_request_token
        self._active_ai_request_token = token
        self._ai_move_future = self._get_best_move_cli.call_async(req)
        self._ai_move_future.add_done_callback(lambda fut: self._on_get_best_move_done(fut, token))

        timeout_sec = float(self._ai_time_limit) + 15.0
        self._ai_rpc_timeout_timer = self.create_timer(timeout_sec, self._on_ai_rpc_timeout)

    def _apply_move(self, move: str, is_ai: bool,
                    eval_cp: int = 0, depth: int = 0, elapsed: float = 0.0) -> None:
        """Apply a validated move to the game state."""
        try:
            if not PYFFISH_OK:
                raise RuntimeError('pyffish unavailable')
            fen_before = self._current_fen
            parsed_red = self._fen_active_is_red(fen_before)
            is_red_move = parsed_red if parsed_red is not None else True

            self._current_fen = sf.get_fen(VARIANT, self._current_fen, [move])
            self._move_history.append(move)
            self._move_count += 1

            hist_msg = MoveHistory()
            hist_msg.header = Header()
            hist_msg.header.stamp = self.get_clock().now().to_msg()
            hist_msg.move = move
            hist_msg.is_red_move = is_red_move
            hist_msg.thinking_time_sec = elapsed
            hist_msg.search_depth = depth
            hist_msg.evaluation_cp = eval_cp
            hist_msg.engine_used = self._active_ai_engine if is_ai else 'human'
            self._move_history_pub.publish(hist_msg)
            self._publish_status()
            self._publish_logical_board_state()
        except Exception as e:
            self.get_logger().error(f'Failed to apply move {move}: {e}')

    # Safety-net move limit (~150 full moves); pyffish handles repetition natively
    _MAX_HALF_MOVES = 300

    @staticmethod
    def _pyffish_score_to_result(value: int, is_red_turn: bool,
                                  is_immediate: bool) -> tuple[str, str]:
        """
        Map a pyffish side-to-move score to (game_result, reason).
          value > 0  → side to move wins
          value < 0  → side to move loses
          value == 0 → draw
        """
        if value > 0:
            result = 'red_wins' if is_red_turn else 'black_wins'
            reason = 'checkmate' if is_immediate else 'perpetual_rule'
        elif value < 0:
            result = 'black_wins' if is_red_turn else 'red_wins'
            reason = 'checkmate' if is_immediate else 'perpetual_rule'
        else:
            result = 'draw'
            reason = 'stalemate' if is_immediate else 'draw_by_repetition'
        return result, reason

    def _declare_game_over(self, result: str, reason: str) -> None:
        self._game_state = GameState.GAME_OVER
        self._game_result = result
        self._game_result_reason = reason
        self.get_logger().info(
            f'Game over: result={result}, reason={reason}, move={self._move_count}'
        )

    def _terminal_from_game_result(self, gr: int, is_red_turn: bool) -> tuple[str, str] | None:
        """Map pyffish game_result() to (result, reason) when side to move has no moves."""
        if gr == -sf.VALUE_MATE:
            winner = 'red_wins' if not is_red_turn else 'black_wins'
            return winner, 'checkmate'
        if gr == sf.VALUE_MATE:
            winner = 'black_wins' if is_red_turn else 'red_wins'
            return winner, 'checkmate'
        if gr in (sf.VALUE_DRAW, 0):
            return 'draw', 'stalemate'
        return None

    def _check_no_legal_moves(self, fen: str, is_red_turn: bool) -> bool:
        """True if game was declared over (checkmate/stalemate with 0 legal moves)."""
        legal = sf.legal_moves(VARIANT, fen, [])
        if legal:
            return False
        gr = sf.game_result(VARIANT, fen, [])
        parsed = self._terminal_from_game_result(gr, is_red_turn)
        if parsed:
            result, reason = parsed
            self._declare_game_over(result, reason)
            return True
        self.get_logger().warn(
            f'No legal moves but game_result={gr}; declaring checkmate for side to move'
        )
        result = 'red_wins' if not is_red_turn else 'black_wins'
        self._declare_game_over(result, 'checkmate')
        return True

    def _check_game_over(self) -> None:
        """
        Classify terminal positions using pyffish native game-end functions:
          1. is_immediate_game_end  – checkmate / stalemate
          2. is_optional_game_end   – repetition / perpetual check / chasing rules
          3. has_insufficient_material – dead-draw material
          4. _MAX_HALF_MOVES         – safety-net draw
        """
        fen = self._current_fen
        if not (PYFFISH_OK and fen):
            return

        is_red_turn = 'w' in fen.split()[1]

        try:
            # ── 1. Immediate: checkmate / stalemate ──────────────────────────
            immediate, imm_val = sf.is_immediate_game_end(VARIANT, fen, [])
            if immediate:
                result, reason = self._pyffish_score_to_result(imm_val, is_red_turn, True)
                self._declare_game_over(result, reason)
                return

            # Some mates: 0 legal moves but is_immediate_game_end is False
            if self._check_no_legal_moves(fen, is_red_turn):
                return

            # ── 2. Optional: repetition / perpetual check / chasing ──────────
            # Pass the full move history from STARTING_FEN so pyffish can
            # reconstruct position counts and apply Xiangqi chasing rules.
            optional, opt_val = sf.is_optional_game_end(
                VARIANT, STARTING_FEN, self._move_history
            )
            if optional:
                result, reason = self._pyffish_score_to_result(opt_val, is_red_turn, False)
                self._declare_game_over(result, reason)
                return

            # ── 3. Insufficient material (dead draw) ─────────────────────────
            try:
                red_insuf, blk_insuf = sf.has_insufficient_material(VARIANT, fen, [])
                if red_insuf and blk_insuf:
                    self._declare_game_over('draw', 'insufficient_material')
                    return
            except AttributeError:
                pass  # older pyffish build without this function

            # ── 4. Safety-net move limit ──────────────────────────────────────
            if self._move_count >= self._MAX_HALF_MOVES:
                self._declare_game_over('draw', 'move_limit')

        except Exception as e:
            self.get_logger().error(f'_check_game_over error: {e}')
            # Fallback: detect checkmate / stalemate via gives_check
            try:
                if not sf.legal_moves(VARIANT, fen, []):
                    in_check = sf.gives_check(VARIANT, fen, [])
                    result = ('black_wins' if is_red_turn else 'red_wins') if in_check else 'draw'
                    reason = 'checkmate' if in_check else 'stalemate'
                    self._declare_game_over(result, reason)
            except Exception:
                pass

    def _grid_mismatch_count(self, expected: list, observed: list) -> int:
        return sum(1 for a, b in zip(expected, observed) if a != b)

    def _pending_move_matches_grid(
        self, move: str, fen: str, grid: list, loose: bool = False
    ) -> bool:
        """True if from/to squares of move match expected post-move grid vs observation."""
        try:
            expected = self._fen_to_grid(sf.get_fen(VARIANT, fen, [move]))
        except Exception:
            return False
        critical = move_critical_indices(move)
        if not critical:
            return self._grids_match(expected, grid, tolerance=12 if loose else 6)
        for idx in critical:
            if expected[idx] != grid[idx]:
                if not loose:
                    return False
                if expected[idx] != 0 and grid[idx] != 0 and expected[idx] != grid[idx]:
                    return False
        return True

    def _infer_move_from_board(
        self,
        fen: str,
        new_grid: list,
        tolerance: int = 0,
        reference_grid: list | None = None,
        human_red: bool | None = None,
    ) -> str | None:
        """
        Pick the legal move whose post-move grid best matches the observation.

        When reference_grid is set (board before the human moved), only consider
        moves that touch squares that actually changed - avoids spurious back-rank
        matches when YOLO is sparse.
        """
        try:
            legal_moves = sf.legal_moves(VARIANT, fen, [])
            ref = reference_grid if reference_grid is not None else self._fen_to_grid(fen)
            changed = {i for i in range(90) if ref[i] != new_grid[i]}

            pool = legal_moves
            if changed:
                touched = [
                    m
                    for m in legal_moves
                    if move_critical_indices(m) & changed
                ]
                if len(changed) >= 2:
                    both_ends = [
                        m
                        for m in touched
                        if move_critical_indices(m).issubset(changed)
                    ]
                    if both_ends:
                        touched = both_ends
                if touched:
                    pool = touched
                else:
                    # The board physically changed but no legal move touches any changed
                    # square — the change has no legal explanation. Do NOT fall back to all
                    # legal moves (that picks a spurious unrelated move). Signal no match.
                    return None

            if human_red is not None:
                pool = [
                    m
                    for m in pool
                    if self._move_origin_matches_human(m, ref, human_red)
                ]
                if not pool:
                    return None

            # Authoritative grid from FEN: pieces that should be on the board.
            # Used to distinguish "vision missed a stable piece" from "piece actually moved".
            auth_grid = self._fen_to_grid(fen)

            best_move: str | None = None
            best_mismatches = 91
            for move in pool:
                candidate_fen = sf.get_fen(VARIANT, fen, [move])
                candidate_grid = self._fen_to_grid(candidate_fen)
                critical = move_critical_indices(move)
                # Robust mismatch: skip cells where vision shows 0 but the authoritative
                # FEN had a piece AND the move doesn't touch that cell.  These are missed
                # detections (YOLO jitter), not real captures — ignoring them prevents a
                # spurious capture from looking like the best match.
                # Sign-only comparison: only check piece colour (red/black/empty),
                # not piece type. YOLO often misidentifies types but colour is reliable.
                mismatches = sum(
                    1 for i in range(90)
                    if (candidate_grid[i] > 0) != (new_grid[i] > 0)
                    or (candidate_grid[i] < 0) != (new_grid[i] < 0)
                    if not (new_grid[i] == 0 and auth_grid[i] != 0 and i not in critical)
                )
                if mismatches < best_mismatches:
                    best_mismatches = mismatches
                    best_move = move
            if best_move is not None and best_mismatches <= tolerance:
                if pool is not legal_moves:
                    self.get_logger().info(
                        f'Move inference narrowed to {len(pool)} candidate(s) '
                        f'from {len(changed)} changed square(s)'
                    )
                return best_move
        except Exception as e:
            self.get_logger().error(f'Move inference error: {e}')
        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _tell_vision_to_watch(self, watch: bool) -> None:
        """Enable/disable vision turn-watching.

        When enabling, first request arm move to scan pose (async) so the
        wrist-mounted camera is pointing straight down at the board.
        start_watching is published once the arm confirms it has arrived
        (or immediately if the service is unavailable, to degrade gracefully).
        """
        if self._self_play:
            if not watch:
                self._human_watch_reference_grid = None
                msg = Bool()
                msg.data = False
                self._start_watching_pub.publish(msg)
            return
        if not watch:
            self._human_watch_reference_grid = None
            msg = Bool()
            msg.data = False
            self._start_watching_pub.publish(msg)
            return

        if not self._move_to_scan_pose_cli.service_is_ready():
            self.get_logger().warn(
                'move_to_scan_pose service not ready - starting watch without repositioning'
            )
            self._publish_start_watching()
            return

        fut = self._move_to_scan_pose_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_scan_pose_ready)

    def _on_scan_pose_ready(self, future) -> None:
        """Called when the arm has reached (or failed to reach) scan pose."""
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn(
                    f'Scan pose move failed ({getattr(result, "message", "no result")}) '
                    '- starting watch anyway'
                )
        except Exception as e:
            self.get_logger().warn(f'Scan pose service error: {e} - starting watch anyway')
        self._publish_start_watching()

    def _publish_start_watching(self) -> None:
        # Prefer the latest camera grid so inference matches vision's turn detector.
        if self._latest_board_state is not None:
            self._human_watch_reference_grid = list(self._latest_board_state.grid)
        elif self._current_fen:
            self._human_watch_reference_grid = self._fen_to_grid(self._current_fen)
        msg = Bool()
        msg.data = True
        self._start_watching_pub.publish(msg)

    @staticmethod
    def _is_capture_move(fen: str, move: str) -> bool:
        """True if destination square holds an opponent piece before this move (coordinate notation)."""
        if not PYFFISH_OK or not move or len(move) < 4:
            return False
        try:
            parsed = _resolver_parse_move(move)
            if parsed is None:
                return False
            _, (to_file_ch, to_rank_1based) = parsed
            to_file = ord(to_file_ch) - ord('a')
            to_rank = to_rank_1based - 1  # UCI rank 1-10 → 0-indexed 0-9
            if not (0 <= to_file < 9 and 0 <= to_rank < 10):
                return False
            idx = to_rank * 9 + to_file
            grid = GameManagerNode._fen_to_grid(fen)
            occupant = grid[idx]
            if occupant == 0:
                return False
            is_red_turn = 'w' in fen.split()[1]
            if is_red_turn:
                return occupant < 0
            return occupant > 0
        except (IndexError, ValueError, AttributeError):
            return False

    def _publish_logical_board_state(self) -> None:
        """Publish authoritative board from game FEN (sim / dashboard display)."""
        grid = self._fen_to_grid(self._current_fen)
        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.grid = [int(x) for x in grid]
        msg.fen = self._current_fen
        msg.last_move = self._move_history[-1] if self._move_history else ''
        msg.is_red_turn = self._side_to_move_is_red()
        msg.detection_confidence = 1.0
        self._board_state_pub.publish(msg)

    def _publish_status(self) -> None:
        msg = GameStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = self._game_state.name.lower()
        msg.is_red_turn = self._side_to_move_is_red()
        msg.move_count = self._move_count
        msg.current_fen = self._current_fen
        if self._game_state == GameState.COMPUTING_AI:
            msg.engine_type = self._engine_for_current_turn()
        elif self._self_play:
            msg.engine_type = f'{self._red_engine_type}|{self._black_engine_type}'
        else:
            msg.engine_type = self._black_engine_type
        msg.system_state = self._game_state.name
        msg.game_result = self._game_result
        msg.game_result_reason = self._game_result_reason
        self._game_status_pub.publish(msg)
        if self._simulation_mode:
            self._publish_logical_board_state()

    @staticmethod
    def _fen_to_grid(fen: str) -> list:
        """Parse FEN to flat int8[90] grid (same as MinimaxEngine._fen_to_grid)."""
        PIECE_CODES = {
            'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,
            'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,
        }
        grid = [0] * 90
        board_part = fen.split()[0]
        for fen_rank_idx, rank_str in enumerate(board_part.split('/')):
            board_rank = 9 - fen_rank_idx
            file_idx = 0
            for ch in rank_str:
                if ch.isdigit():
                    file_idx += int(ch)
                else:
                    code = PIECE_CODES.get(ch, 0)
                    if code:
                        idx = board_rank * 9 + file_idx
                        grid[idx] = code if ch.isupper() else -code
                    file_idx += 1
        return grid

    @staticmethod
    def _grid_to_fen(grid: list, original_fen: str) -> str:
        """Reconstruct a FEN string from a flat int8[90] grid.

        The board part is rebuilt from the grid; the metadata tail (side to
        move, castling, en-passant, move counters) is preserved unchanged from
        `original_fen`.
        """
        PIECE_CHARS = {1: 'K', 2: 'A', 3: 'B', 4: 'N', 5: 'R', 6: 'C', 7: 'P'}
        rows = []
        for rank in range(9, -1, -1):   # FEN: rank 9 (black home) → rank 0 (red home)
            row = ''
            empty = 0
            for file in range(9):
                val = grid[rank * 9 + file]
                if val == 0:
                    empty += 1
                else:
                    if empty:
                        row += str(empty)
                        empty = 0
                    ch = PIECE_CHARS.get(abs(val), '?')
                    row += ch if val > 0 else ch.lower()
            if empty:
                row += str(empty)
            rows.append(row)
        board_part = '/'.join(rows)
        parts = original_fen.split()
        parts[0] = board_part
        return ' '.join(parts)

    @staticmethod
    def _sanitize_fen_for_engine(fen: str) -> str:
        """Ensure a vision-scanned FEN is legally playable.

        The YOLO detector may miss pieces (especially in glitchy conditions).
        If either king is absent, we insert it at its standard starting square
        (e0 for Red = index 4, e9 for Black = index 85) or, if that square is
        already occupied, at the first empty cell in the home row.

        This means the AI always receives a FEN that pyffish can parse, even
        when the physical board is partially set up or detection is imperfect.
        No pieces are *removed* - we only add the missing king(s).
        """
        try:
            grid = list(GameManagerNode._fen_to_grid(fen))
            has_red_king   = any(v ==  1 for v in grid)
            has_black_king = any(v == -1 for v in grid)

            if has_red_king and has_black_king:
                out = fen
            else:
                out = None
            if out is not None:
                if GameManagerNode._fen_active_is_red(out) is None:
                    out = GameManagerNode._set_fen_active_color(out, True)
                return out

            if not has_red_king:
                # Red king default: e0 → rank 0, file 4 → index 4
                preferred = 0 * 9 + 4
                if grid[preferred] == 0:
                    grid[preferred] = 1
                else:
                    for f in range(9):
                        if grid[f] == 0:
                            grid[f] = 1
                            break

            if not has_black_king:
                # Black king default: e9 → rank 9, file 4 → index 85
                preferred = 9 * 9 + 4
                if grid[preferred] == 0:
                    grid[preferred] = -1
                else:
                    for f in range(9):
                        idx = 9 * 9 + f
                        if grid[idx] == 0:
                            grid[idx] = -1
                            break

            out = GameManagerNode._grid_to_fen(grid, fen)
            if GameManagerNode._fen_active_is_red(out) is None:
                out = GameManagerNode._set_fen_active_color(out, True)
            return out
        except Exception:
            return fen  # if anything goes wrong, pass through unchanged

    @staticmethod
    def _grids_match(a: list, b: list, tolerance: int = 0) -> bool:
        diffs = sum(1 for x, y in zip(a, b) if x != y)
        return diffs <= tolerance


def main(args=None):
    rclpy.init(args=args)
    node = GameManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
