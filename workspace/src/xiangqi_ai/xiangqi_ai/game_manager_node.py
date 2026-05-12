"""
game_manager_node: Deliberative layer (Tier 3) central orchestrator.

Maintains the authoritative game state (FEN), validates human moves,
detects game-over conditions, and dispatches AI move requests.
Publishes GameStatus and MoveHistory for the dashboard and planner.
"""

from __future__ import annotations
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Empty, Header, String

from xiangqi_msgs.msg import BoardState, GameStatus, MoveHistory
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
        # Confirmation from task_planner BT that the robot physically finished a move
        self._robot_done_sub = self.create_subscription(
            Bool, '/xiangqi/robot_move_complete', self._robot_move_complete_cb, 10,
            callback_group=cb_group
        )

        # Publishers
        self._game_status_pub = self.create_publisher(GameStatus, '/xiangqi/game_status', 10)
        self._move_history_pub = self.create_publisher(MoveHistory, '/xiangqi/move_history', 10)
        self._start_watching_pub = self.create_publisher(Bool, '/xiangqi/start_watching', 10)
        self._execute_move_pub = self.create_publisher(String, '/xiangqi/execute_move', 10)
        self._illegal_move_pub = self.create_publisher(String, '/xiangqi/illegal_move_alert', 10)

        # Service clients
        self._get_best_move_cli = self.create_client(
            GetBestMove, 'get_best_move', callback_group=cb_group
        )
        self._get_board_state_cli = self.create_client(
            GetBoardState, 'get_board_state', callback_group=cb_group
        )

        # Status timer
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self._latest_board_state: BoardState | None = None
        self._pending_human_move: str | None = None

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

    def _robot_move_complete_cb(self, msg: Bool) -> None:
        """BT publishes True here after the robot physically executes its move."""
        if not msg.data:
            return
        if self._game_state != GameState.EXECUTING_MOVE:
            return
        self.get_logger().info('Robot move execution confirmed — waiting for human')
        self._game_state = GameState.WAITING_HUMAN
        self._tell_vision_to_watch(True)
        self._publish_status()

    def _new_game_cb(self, _: Empty) -> None:
        self.get_logger().info('New game started')
        self._current_fen = STARTING_FEN
        self._move_history = []
        self._move_count = 0
        self._pending_human_move = None

        # Robot plays Red and moves first -- start with AI move
        if self._robot_is_red:
            self._game_state = GameState.COMPUTING_AI
            self._publish_status()
            self._compute_and_emit_ai_move()
        else:
            self._game_state = GameState.WAITING_HUMAN
            self._tell_vision_to_watch(True)
            self._publish_status()

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

    def _compute_and_emit_ai_move(self) -> None:
        """Call AI engine service and emit the move for the planner."""
        if not self._get_best_move_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('AI engine service unavailable')
            return

        req = GetBestMove.Request()
        req.fen = self._current_fen
        req.depth = self._ai_depth
        req.time_limit = self._ai_time_limit
        req.engine_type = self._engine_type

        future = self._get_best_move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._ai_time_limit + 5.0)

        if future.result() is None or not future.result().success:
            self.get_logger().error('AI engine returned no move')
            return

        resp = future.result()
        ai_move = resp.best_move
        self.get_logger().info(f'AI move: {ai_move} (depth={resp.depth_reached}, eval={resp.evaluation_cp}cp)')

        self._apply_move(ai_move, is_ai=True, eval_cp=resp.evaluation_cp,
                         depth=resp.depth_reached, elapsed=resp.thinking_time_sec)
        self._check_game_over()

        if self._game_state != GameState.GAME_OVER:
            # Tell planner to execute this move
            self._game_state = GameState.EXECUTING_MOVE
            self._publish_status()
            msg = String()
            msg.data = ai_move
            self._execute_move_pub.publish(msg)

    def _apply_move(self, move: str, is_ai: bool,
                    eval_cp: int = 0, depth: int = 0, elapsed: float = 0.0) -> None:
        """Apply a validated move to the game state."""
        try:
            self._current_fen = sf.get_fen(VARIANT, self._current_fen, [move])
            self._move_history.append(move)
            self._move_count += 1

            hist_msg = MoveHistory()
            hist_msg.header = Header()
            hist_msg.header.stamp = self.get_clock().now().to_msg()
            hist_msg.move = move
            hist_msg.is_red_move = 'w' not in self._current_fen.split()[1]  # Already applied
            hist_msg.thinking_time_sec = elapsed
            hist_msg.search_depth = depth
            hist_msg.evaluation_cp = eval_cp
            hist_msg.engine_used = self._engine_type if is_ai else 'human'
            self._move_history_pub.publish(hist_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to apply move {move}: {e}')

    def _check_game_over(self) -> None:
        try:
            legal = sf.legal_moves(VARIANT, self._current_fen, [])
            if not legal:
                self._game_state = GameState.GAME_OVER
                self.get_logger().info('Game over -- no legal moves')
        except Exception:
            pass

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
        msg = Bool()
        msg.data = watch
        self._start_watching_pub.publish(msg)

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
