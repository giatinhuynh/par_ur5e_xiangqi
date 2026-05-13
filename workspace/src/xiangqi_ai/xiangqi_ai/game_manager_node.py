"""
game_manager_node: Deliberative layer (Tier 3) central orchestrator.

Maintains the authoritative game state (FEN), validates human moves,
detects game-over conditions, and dispatches AI move requests.
Publishes GameStatus and MoveHistory for the dashboard and planner.
"""

from __future__ import annotations
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Empty, Header, String
from std_srvs.srv import Trigger

from xiangqi_msgs.msg import BoardState, GameStatus, MoveHistory
from xiangqi_msgs.msg import AiMoveCommand, AiCommandAck, AiExecutionResult
from xiangqi_msgs.srv import GetBestMove, GetBoardState, SetEngine

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
        self.declare_parameter('ai_time_limit', 5.0)
        self.declare_parameter('ai_depth', 0)

        self._engine_type = self.get_parameter('engine_type').value
        self._robot_is_red = self.get_parameter('robot_plays_red').value
        self._ai_time_limit = self.get_parameter('ai_time_limit').value
        self._ai_depth = self.get_parameter('ai_depth').value

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
        self._new_game_sub = self.create_subscription(
            Empty, '/xiangqi/new_game', self._new_game_cb, 10,
            callback_group=cb_group
        )
        self._resync_sub = self.create_subscription(
            Empty,
            '/xiangqi/resync_from_vision',
            self._resync_from_vision_cb,
            10,
            callback_group=cb_group,
        )
        # Planner execution feedback (ID-scoped JSON: status + dispatch_id)
        self._ai_exec_result_sub = self.create_subscription(
            AiExecutionResult,
            '/xiangqi/ai_execution_result',
            self._ai_execution_result_cb,
            10,
            callback_group=cb_group,
        )
        self._ai_cmd_ack_sub = self.create_subscription(
            AiCommandAck,
            '/xiangqi/ai_command_ack',
            self._ai_command_ack_cb,
            10,
            callback_group=cb_group,
        )
        self._estop_sub = self.create_subscription(
            Bool, '/xiangqi/estop', self._estop_cb, 10, callback_group=cb_group
        )

        # Publishers
        self._game_status_pub = self.create_publisher(GameStatus, '/xiangqi/game_status', 10)
        self._move_history_pub = self.create_publisher(MoveHistory, '/xiangqi/move_history', 10)
        self._start_watching_pub = self.create_publisher(Bool, '/xiangqi/start_watching', 10)
        # Single atomic dispatch (move + capture + expected FEN + id) for the planner
        self._ai_move_command_pub = self.create_publisher(AiMoveCommand, '/xiangqi/ai_move_command', 10)
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

        # Status timer
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self._latest_board_state: BoardState | None = None
        self._pending_human_move: str | None = None

        # AI move bookkeeping — apply to FEN only after ai_execution_result=robot_move_complete
        self._pending_ai_move: str | None = None
        self._pending_ai_eval_cp: int = 0
        self._pending_ai_depth: int = 0
        self._pending_ai_elapsed: float = 0.0
        self._ai_dispatch_id: int = 0
        self._active_dispatch_id: int | None = None
        self._planner_ack_timer = None

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

        self.get_logger().info('game_manager_node started -- waiting for /xiangqi/new_game')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _board_state_cb(self, msg: BoardState) -> None:
        self._latest_board_state = msg

    def _human_move_detected_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._game_state != GameState.WAITING_HUMAN:
            return
        self._game_state = GameState.DETECTING_MOVE
        self._publish_status()
        self._process_human_move()

    def _ai_execution_result_cb(self, msg: AiExecutionResult) -> None:
        """Handle planner feedback with dispatch_id guard against stale callbacks."""
        dispatch_id = int(msg.dispatch_id)
        status = int(msg.status)

        if self._game_state != GameState.EXECUTING_MOVE:
            return
        if self._active_dispatch_id is None:
            self.get_logger().warn(
                f'Ignoring ai_execution_result id={dispatch_id}: no active dispatch'
            )
            return
        if dispatch_id != self._active_dispatch_id:
            self.get_logger().warn(
                f'Ignoring stale ai_execution_result id={dispatch_id} '
                f'(active={self._active_dispatch_id})'
            )
            return

        if status == AiExecutionResult.ROBOT_MOVE_COMPLETE:
            if self._pending_ai_move:
                self._apply_move(
                    self._pending_ai_move,
                    is_ai=True,
                    eval_cp=self._pending_ai_eval_cp,
                    depth=self._pending_ai_depth,
                    elapsed=self._pending_ai_elapsed,
                )
                self._pending_ai_move = None
                self._active_dispatch_id = None
                self._check_game_over()
            else:
                self.get_logger().error(
                    'robot_move_complete but no pending AI move — ignoring spurious signal'
                )
                return

            if self._game_state == GameState.GAME_OVER:
                self.get_logger().info('Game over — robot finished last move')
                self._publish_status()
                return

            self.get_logger().info('Robot move execution confirmed — waiting for human')
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()
            return

        if status == AiExecutionResult.BOARD_VERIFY_FAILED:
            self._discard_pending_ai_after_planner_abort(
                'Board verification failed after robot move (FEN unchanged)',
                'Robot move did not match vision — align pieces with the UI or use New Game',
            )
            return

        if status == AiExecutionResult.AI_MOTION_FAILED:
            self._discard_pending_ai_after_planner_abort(
                'AI motion subtree failed — pending AI move discarded (FEN unchanged)',
                'Robot could not complete the planned move — check arm/gripper/board; '
                'UI position unchanged; use New Game if the physical board moved',
            )
            return

        self._discard_pending_ai_after_planner_abort(
            f'Unknown ai_execution_result status={status} — pending move discarded (FEN unchanged)',
            f'Unexpected planner status ({status}) — pending move discarded; check logs',
        )
        return

    def _clear_planner_ack_timer(self) -> None:
        if self._planner_ack_timer is not None:
            self._planner_ack_timer.cancel()
            self._planner_ack_timer = None

    def _on_planner_ack_timeout(self) -> None:
        self._clear_planner_ack_timer()
        if self._game_state != GameState.EXECUTING_MOVE:
            return
        if self._active_dispatch_id is None:
            return
        # Planner never acked the command - abandon this pending move
        self._discard_pending_ai_after_planner_abort(
            f'Planner did not ack ai_move_command id={self._active_dispatch_id} (timeout)',
            'Planner did not accept AI command — move aborted; check planner logs',
        )

    def _ai_command_ack_cb(self, msg: AiCommandAck) -> None:
        dispatch_id = int(msg.dispatch_id)
        accepted = bool(msg.accepted)
        reason = msg.reason
        if self._active_dispatch_id is None or dispatch_id != self._active_dispatch_id:
            return
        if accepted:
            self._clear_planner_ack_timer()
        else:
            self._discard_pending_ai_after_planner_abort(
                f'Planner NACKed ai_move_command id={dispatch_id}: {reason}',
                f'Planner rejected AI command ({reason}) — move aborted; check planner logs',
            )

    def _discard_pending_ai_after_planner_abort(self, log_msg: str, alert_text: str) -> None:
        """Drop pending AI move without applying FEN; return to human watching."""
        if self._game_state != GameState.EXECUTING_MOVE:
            return
        if not self._pending_ai_move:
            self.get_logger().warn(f'{log_msg} — no pending AI move (ignored)')
            return
        self.get_logger().error(log_msg)
        self._pending_ai_move = None
        self._active_dispatch_id = None
        self._clear_planner_ack_timer()
        alert = String()
        alert.data = alert_text
        self._illegal_move_pub.publish(alert)
        self._game_state = GameState.WAITING_HUMAN
        self._tell_vision_to_watch(True)
        self._publish_status()

    def _estop_cb(self, msg: Bool) -> None:
        """E-stop during motion or while waiting on the AI engine."""
        if not msg.data:
            return
        if self._game_state == GameState.EXECUTING_MOVE:
            self._discard_pending_ai_after_planner_abort(
                'E-stop asserted during robot AI move — pending move discarded (FEN unchanged)',
                'E-stop: robot move cancelled in software — confirm physical board matches UI',
            )
            return
        if self._game_state == GameState.COMPUTING_AI:
            self._abort_ai_computation = True
            self._cancel_ai_rpc_in_flight()
            self._active_ai_request_token = None
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            alert = String()
            alert.data = (
                'E-stop during AI thinking — cancelled engine request; make a move or New Game'
            )
            self._illegal_move_pub.publish(alert)
            self._publish_status()
            return

    def _recover_ai_computation_failed(self, detail: str) -> None:
        """AI service or FEN computation failed while in COMPUTING_AI — unblock the game."""
        self._clear_ai_rpc_timeout_timer()
        self._clear_ai_service_retry_timer()
        self._ai_move_future = None
        self._active_ai_request_token = None
        self._ai_fen_at_request = None
        self.get_logger().error(detail)
        alert = String()
        alert.data = (
            f'{detail} — make a move on the board or press New Game.'
        )
        self._illegal_move_pub.publish(alert)
        if self._game_state == GameState.COMPUTING_AI:
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
        self._publish_status()

    def _new_game_cb(self, _: Empty) -> None:
        self.get_logger().info('New game started')
        self._abort_ai_computation = False
        self._cancel_ai_rpc_in_flight()
        self._active_ai_request_token = None
        self._ai_fen_at_request = None
        self._ai_request_token += 1
        self._current_fen = STARTING_FEN
        self._move_history = []
        self._move_count = 0
        self._pending_human_move = None
        self._pending_ai_move = None
        self._active_dispatch_id = None
        self._clear_planner_ack_timer()
        self._ai_dispatch_id = 0
        self._ai_service_retry_count = 0
        self._game_result = "ongoing"
        self._game_result_reason = ""

        # Robot plays Red and moves first -- start with AI move
        if self._robot_is_red:
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()
        else:
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()

    def _resync_from_vision_cb(self, _: Empty) -> None:
        """Force a GetBoardState scan and adopt the returned FEN as authoritative."""
        self.get_logger().warn('Resync requested — adopting vision FEN as authoritative state')
        self._abort_ai_computation = True
        self._cancel_ai_rpc_in_flight()
        self._active_dispatch_id = None
        self._pending_ai_move = None
        self._pending_human_move = None
        self._clear_planner_ack_timer()

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
                self.get_logger().error('Resync failed: empty FEN from vision')
                alert = String()
                alert.data = 'Resync failed: empty FEN from vision'
                self._illegal_move_pub.publish(alert)
                return

            self._current_fen = fen
            self._move_history = []
            self._move_count = 0
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()
            ok = String()
            ok.data = 'Resync OK: adopted vision FEN as authoritative'
            self._illegal_move_pub.publish(ok)

        future.add_done_callback(_done)

    # ------------------------------------------------------------------
    # Game flow
    # ------------------------------------------------------------------

    def _process_human_move(self) -> None:
        """Detect and validate the human move from the current board state."""
        if self._latest_board_state is None:
            self.get_logger().warn('No board state available for human move detection')
            self._game_state = GameState.WAITING_HUMAN
            return

        detected_move = self._infer_move_from_board(
            self._current_fen, self._latest_board_state.grid
        )

        if detected_move is None:
            self.get_logger().warn('Could not infer a valid human move from board state')
            alert_msg = String()
            alert_msg.data = 'Could not detect move -- please re-place your piece'
            self._illegal_move_pub.publish(alert_msg)
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
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

        if self._abort_ai_computation:
            self._abort_ai_computation = False
            return
        if self._game_state != GameState.COMPUTING_AI:
            return

        try:
            resp = future.result()
        except Exception as e:
            self._recover_ai_computation_failed(f'AI service error: {e}')
            return

        if resp is None or not resp.success:
            self._recover_ai_computation_failed('AI engine returned no move')
            return

        ai_move = resp.best_move
        self.get_logger().info(
            f'AI move: {ai_move} (depth={resp.depth_reached}, eval={resp.evaluation_cp}cp)'
        )

        fen_before_ai = self._ai_fen_at_request or self._current_fen
        is_capture = self._is_capture_move(fen_before_ai, ai_move)

        try:
            expected_fen = sf.get_fen(VARIANT, fen_before_ai, [ai_move])
        except Exception as e:
            self._recover_ai_computation_failed(
                f'Could not compute post-move FEN for {ai_move}: {e}'
            )
            return

        self._ai_dispatch_id += 1
        dispatch_id = self._ai_dispatch_id

        self._pending_ai_move = ai_move
        self._pending_ai_eval_cp = resp.evaluation_cp
        self._pending_ai_depth = resp.depth_reached
        self._pending_ai_elapsed = resp.thinking_time_sec
        self._active_dispatch_id = dispatch_id

        cmd = AiMoveCommand()
        cmd.dispatch_id = dispatch_id
        cmd.move = ai_move
        cmd.is_capture = bool(is_capture)
        cmd.expected_fen = expected_fen
        self._ai_move_command_pub.publish(cmd)

        self._game_state = GameState.EXECUTING_MOVE
        self._publish_status()

        self._active_dispatch_id = dispatch_id
        self._clear_planner_ack_timer()
        # Planner must ack the command quickly, or we abandon to avoid deadlock
        self._planner_ack_timer = self.create_timer(2.0, self._on_planner_ack_timeout)

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

        req = GetBestMove.Request()
        self._ai_fen_at_request = self._current_fen
        req.fen = self._ai_fen_at_request
        req.depth = self._ai_depth
        req.time_limit = self._ai_time_limit
        req.engine_type = self._engine_type

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
            side_before = fen_before.split()[1] if fen_before else 'w'
            is_red_move = 'w' in side_before

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
            hist_msg.engine_used = self._engine_type if is_ai else 'human'
            self._move_history_pub.publish(hist_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to apply move {move}: {e}')

    def _check_game_over(self) -> None:
        """Classify terminal positions using pyffish game_result."""
        fen = self._current_fen
        if not (PYFFISH_OK and fen):
            return
        try:
            legal = sf.legal_moves(VARIANT, fen, [])
            res = sf.game_result(VARIANT, fen, [])
        except Exception as e:
            self.get_logger().error(f'game_result/legal_moves error: {e}')
            # Fallback: preserve legacy behaviour
            try:
                legal = sf.legal_moves(VARIANT, fen, [])
                if not legal:
                    self._game_state = GameState.GAME_OVER
                    self._game_result = "unknown"
                    self._game_result_reason = "no_legal_moves"
                    self.get_logger().info('Game over -- no legal moves (fallback)')
            except Exception:
                pass
            return

        no_legal = len(legal) == 0
        result = "ongoing"
        reason = ""

        if res in ("1-0", "0-1", "1/2-1/2"):
            # Winner / draw from pyffish score
            if res == "1-0":
                result = "red_wins"
            elif res == "0-1":
                result = "black_wins"
            else:
                result = "draw"

            if result in ("red_wins", "black_wins"):
                reason = "checkmate" if no_legal else "win_by_rule_or_resign"
            else:
                reason = "stalemate" if no_legal else "draw_by_rule"
        else:
            # Non-terminal or unknown code from pyffish
            if no_legal:
                result = "unknown"
                reason = "no_legal_moves"

        if result != "ongoing":
            self._game_state = GameState.GAME_OVER
            self._game_result = result
            self._game_result_reason = reason
            self.get_logger().info(
                f'Game over: result={result}, reason={reason}, res_code={res}, no_legal={no_legal}'
            )

    def _infer_move_from_board(self, fen: str, new_grid: list) -> str | None:
        """
        Infer the human move by comparing the new board grid to the legal moves
        in the current position and finding which legal move results in the observed grid.
        """
        try:
            legal_moves = sf.legal_moves(VARIANT, fen, [])
            for move in legal_moves:
                candidate_fen = sf.get_fen(VARIANT, fen, [move])
                candidate_grid = self._fen_to_grid(candidate_fen)
                if self._grids_match(candidate_grid, new_grid):
                    return move
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
        if not watch:
            msg = Bool()
            msg.data = False
            self._start_watching_pub.publish(msg)
            return

        if not self._move_to_scan_pose_cli.service_is_ready():
            self.get_logger().warn(
                'move_to_scan_pose service not ready — starting watch without repositioning'
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
                    '— starting watch anyway'
                )
        except Exception as e:
            self.get_logger().warn(f'Scan pose service error: {e} — starting watch anyway')
        self._publish_start_watching()

    def _publish_start_watching(self) -> None:
        msg = Bool()
        msg.data = True
        self._start_watching_pub.publish(msg)

    @staticmethod
    def _is_capture_move(fen: str, move: str) -> bool:
        """True if destination square holds an opponent piece before this move (coordinate notation)."""
        if not PYFFISH_OK or not move or len(move) < 4:
            return False
        try:
            to_file = ord(move[2].lower()) - ord('a')
            to_rank = int(move[3])
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

    def _publish_status(self) -> None:
        msg = GameStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = self._game_state.name.lower()
        msg.is_red_turn = 'w' in self._current_fen.split()[1] if self._current_fen else True
        msg.move_count = self._move_count
        msg.current_fen = self._current_fen
        msg.engine_type = self._engine_type
        msg.system_state = self._game_state.name
        msg.game_result = self._game_result
        msg.game_result_reason = self._game_result_reason
        self._game_status_pub.publish(msg)

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
    def _grids_match(a: list, b: list, tolerance: int = 0) -> bool:
        diffs = sum(1 for x, y in zip(a, b) if x != y)
        return diffs <= tolerance


def main(args=None):
    rclpy.init(args=args)
    node = GameManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
