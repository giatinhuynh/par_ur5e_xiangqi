"""
game_behaviours.py: py_trees leaf behaviours for game logic steps.

These are Condition and Action nodes that interface with the game manager
and vision node via ROS topics and services.
"""

from __future__ import annotations
import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from xiangqi_msgs.srv import GetBoardState
from xiangqi_msgs.msg import AiExecutionResult


PIECE_CODES = {
    'K': 1, 'A': 2, 'B': 3, 'N': 4, 'R': 5, 'C': 6, 'P': 7,
    'k': 1, 'a': 2, 'b': 3, 'n': 4, 'r': 5, 'c': 6, 'p': 7,
}


def _bb_get(bb, key: str, default=None):
    """Compatibility wrapper: py_trees Blackboard.get() may not accept a default argument."""
    try:
        val = bb.get(key)
        return val if val is not None else default
    except Exception:
        return default


def _parse_uci_square(move: str, start: int) -> tuple[int, int, int] | None:
    """Return (file, rank_1based, next_index) for UCI square at move[start:]."""
    if start >= len(move):
        return None
    f = ord(move[start]) - 97
    if not (0 <= f < 9):
        return None
    if start + 2 < len(move) and move[start + 1] == '1' and move[start + 2] == '0':
        return f, 10, start + 3
    if start + 1 < len(move) and move[start + 1].isdigit():
        return f, int(move[start + 1]), start + 2
    return None


def uci_move_critical_indices(move: str) -> set[int]:
    """Grid indices for from/to squares of a coordinate move (e.g. d4e4)."""
    if not move:
        return set()
    a = _parse_uci_square(move, 0)
    if not a:
        return set()
    from_file, from_rank, next_i = a
    b = _parse_uci_square(move, next_i)
    if not b:
        return set()
    to_file, to_rank, _ = b
    return {
        (from_rank - 1) * 9 + from_file,
        (to_rank - 1) * 9 + to_file,
    }


def fen_to_grid(fen: str) -> list[int]:
    """Parse Xiangqi FEN board part to int8[90] grid (same indexing as vision / game_manager)."""
    grid = [0] * 90
    if not fen:
        return grid
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


# ------------------------------------------------------------------
# Conditions (return SUCCESS/FAILURE without side-effects)
# ------------------------------------------------------------------

class IsHumanMovePending(py_trees.behaviour.Behaviour):
    """
    Checks the blackboard for a pending human move.
    SUCCESS if 'human_move' key is set and non-empty.
    """
    def __init__(self):
        super().__init__('IsHumanMovePending')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        move = _bb_get(self._bb, 'human_move')
        if move:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsCapture(py_trees.behaviour.Behaviour):
    """
    Checks blackboard 'is_capture' flag.
    SUCCESS if the current AI move captures an opponent piece.
    """
    def __init__(self):
        super().__init__('IsCapture')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        if _bb_get(self._bb, 'is_capture', False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsEstopActive(py_trees.behaviour.Behaviour):
    """Checks blackboard 'estop_active' flag -- FAILURE if e-stop is active."""
    def __init__(self):
        super().__init__('IsEstopActive')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        if _bb_get(self._bb, 'estop_active', False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# ------------------------------------------------------------------
# Actions (RUNNING until complete, then SUCCESS/FAILURE)
# ------------------------------------------------------------------

class WaitForHumanMove(py_trees.behaviour.Behaviour):
    """
    RUNNING until the blackboard receives a 'human_move_detected' flag.
    The vision node sets this via the game manager.
    """
    def __init__(self):
        super().__init__('WaitForHumanMove')
        self._bb = py_trees.blackboard.Blackboard()

    def initialise(self) -> None:
        self._bb.set('human_move_detected', False)

    def update(self) -> py_trees.common.Status:
        if _bb_get(self._bb, 'human_move_detected', False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status) -> None:
        self._bb.set('human_move_detected', False)


class AlertIllegalMove(py_trees.behaviour.Behaviour):
    """
    Publishes an illegal move alert and returns FAILURE so the BT retries.
    """
    def __init__(self, node: Node):
        super().__init__('AlertIllegalMove')
        self._pub = node.create_publisher(String, '/xiangqi/illegal_move_alert', 10)
        self._sent = False

    def initialise(self) -> None:
        self._sent = False

    def update(self) -> py_trees.common.Status:
        if not self._sent:
            msg = String()
            msg.data = 'Illegal move detected -- please re-make your move'
            self._pub.publish(msg)
            self._sent = True
        return py_trees.common.Status.FAILURE


class GoToScanPose(py_trees.behaviour.Behaviour):
    """
    Calls /xiangqi/move_to_scan_pose (std_srvs/Trigger) to move the
    wrist-mounted camera to the configured top-down bird's-eye position
    before any board vision scan.

    SUCCESS when the service returns success=True.
    FAILURE when the service is unavailable or returns success=False.
    Callers should wrap this in FailureIsSuccess so a transient arm
    failure does not abort the entire move sequence.
    """

    def __init__(self, node: Node):
        super().__init__('GoToScanPose')
        self._node = node
        self._cli = node.create_client(Trigger, '/xiangqi/move_to_scan_pose')
        self._future = None

    def initialise(self) -> None:
        self._future = None
        if not self._cli.service_is_ready():
            self._node.get_logger().warn(
                'move_to_scan_pose service not ready - skipping scan pose'
            )
            return
        self._future = self._cli.call_async(Trigger.Request())

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            # Service not available - degrade gracefully
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        result = self._future.result()
        if result is not None and result.success:
            return py_trees.common.Status.SUCCESS
        self._node.get_logger().warn(
            f'GoToScanPose: {getattr(result, "message", "no response")}'
        )
        return py_trees.common.Status.FAILURE


class SetupMoveCoordinates(py_trees.behaviour.Behaviour):
    """
    Reads 'ai_move' and 'move_translator' from the blackboard,
    computes pick/place poses, and writes them back to the blackboard.
    """
    def __init__(self):
        super().__init__('SetupMoveCoordinates')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        move = _bb_get(self._bb, 'ai_move')
        translator = _bb_get(self._bb, 'move_translator')
        if not move or translator is None:
            return py_trees.common.Status.FAILURE

        try:
            approach_pick, grasp, lift, approach_place, place = translator.move_to_poses(move)
            self._bb.set('pick_pose', grasp)
            self._bb.set('place_pose', place)
            self._bb.set('approach_height', translator._approach_height_m())
            self._bb.set('transit_height', translator._transit_height_m())

            # Check if capture and set capture pick pose
            is_capture = _bb_get(self._bb, 'is_capture', False)
            if is_capture:
                self._bb.set('capture_pick_pose', place)  # The destination has the capturable piece
                # Post-move FEN: if Red to move next, Black just captured a Red piece (and vice versa).
                expected_fen = _bb_get(self._bb, 'expected_board_fen', '') or ''
                parts = expected_fen.split()
                captured_is_red = len(parts) > 1 and parts[1].strip().lower() == 'w'
                graveyard = translator.graveyard_pose(is_red_piece=captured_is_red)
                self._bb.set('graveyard_pose', graveyard)

            return py_trees.common.Status.SUCCESS
        except Exception:
            return py_trees.common.Status.FAILURE


class FinalizeRobotMoveAfterVerify(py_trees.behaviour.Behaviour):
    """
    After motion + verify (possibly failed after retries), publish
    ``/xiangqi/ai_execution_result`` with ``dispatch_id`` and a ``status`` of
    ``robot_move_complete`` or ``board_verify_failed``. Always clears
    move-related blackboard keys.
    """
    def __init__(self, node: Node):
        super().__init__('FinalizeRobotMove')
        self._node = node
        self._result_pub = node.create_publisher(AiExecutionResult, '/xiangqi/ai_execution_result', 10)
        self._done = False

    def initialise(self) -> None:
        self._done = False

    def update(self) -> py_trees.common.Status:
        if self._done:
            return py_trees.common.Status.SUCCESS

        bb = py_trees.blackboard.Blackboard()
        dispatch_id = _bb_get(bb, 'current_dispatch_id')
        if dispatch_id is None:
            self._node.get_logger().error(
                'FinalizeRobotMoveAfterVerify missing current_dispatch_id; skipping result publish'
            )
        elif _bb_get(bb, 'verification_passed', False):
            msg = AiExecutionResult()
            msg.dispatch_id = int(dispatch_id)
            msg.status = AiExecutionResult.ROBOT_MOVE_COMPLETE
            msg.message = ''
            self._result_pub.publish(msg)
        else:
            fail = AiExecutionResult()
            fail.dispatch_id = int(dispatch_id)
            fail.status = AiExecutionResult.BOARD_VERIFY_FAILED
            fail.message = 'board mismatch after retries'
            self._result_pub.publish(fail)
            self._node.get_logger().warn(
                'Board verification failed after retries - publishing board_verify_failed'
            )

        bb.set('ai_move', None)
        bb.set('is_capture', False)
        bb.set('expected_board_fen', None)
        bb.set('current_dispatch_id', None)
        bb.set('verification_passed', False)
        self._done = True
        return py_trees.common.Status.SUCCESS


class AiMotionFailureFinalizer(py_trees.behaviour.Behaviour):
    """
    Fallback child of a ``Selector`` wrapping the main move sequence: runs when
    capture / pick-place / setup fails before ``FinalizeRobotMoveAfterVerify``.
    Publishes ``/xiangqi/ai_execution_result`` with ``status=ai_motion_failed``
    and clears move blackboard keys so the game manager can drop the pending AI
    move without committing FEN.
    """
    def __init__(self, node: Node):
        super().__init__('AiMotionFailureFinalizer')
        self._node = node
        self._pub = node.create_publisher(AiExecutionResult, '/xiangqi/ai_execution_result', 10)
        self._done = False

    def initialise(self) -> None:
        self._done = False

    def update(self) -> py_trees.common.Status:
        if self._done:
            return py_trees.common.Status.SUCCESS

        bb = py_trees.blackboard.Blackboard()
        dispatch_id = _bb_get(bb, 'current_dispatch_id')
        if dispatch_id is None:
            self._node.get_logger().error(
                'AiMotionFailureFinalizer missing current_dispatch_id; skipping result publish'
            )
        else:
            msg = AiExecutionResult()
            msg.dispatch_id = int(dispatch_id)
            msg.status = AiExecutionResult.AI_MOTION_FAILED
            msg.message = 'setup/capture/manipulation failed before verify'
            self._pub.publish(msg)
        bb.set('ai_move', None)
        bb.set('is_capture', False)
        bb.set('expected_board_fen', None)
        bb.set('current_dispatch_id', None)
        bb.set('verification_passed', False)
        self._node.get_logger().error(
            'Move subtree failed (setup / capture / manipulation) - publishing ai_motion_failed'
        )
        self._done = True
        return py_trees.common.Status.SUCCESS


class VerifyBoardState(py_trees.behaviour.Behaviour):
    """
    Calls ``get_board_state`` (vision) and compares the observed grid to
    ``expected_board_fen`` on the blackboard.

    Plain rclpy client (lab py_trees_ros 2.0.x has no ``service_clients`` module).
    """

    def __init__(self, node: Node, name: str = 'VerifyBoardState'):
        super().__init__(name)
        self._node = node
        self._bb = py_trees.blackboard.Blackboard()
        self._cli = node.create_client(GetBoardState, 'get_board_state')
        self._future = None

    def initialise(self) -> None:
        self._bb.set('verification_passed', False)
        self._future = None
        if not self._cli.service_is_ready():
            self.feedback_message = 'get_board_state service not ready'
            return
        req = GetBoardState.Request()
        req.force_rescan = True
        self._future = self._cli.call_async(req)

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.feedback_message = f'GetBoardState error: {e}'
            return py_trees.common.Status.FAILURE

        if resp is None or not resp.success:
            self.feedback_message = 'vision scan failed or no board'
            return py_trees.common.Status.FAILURE

        expected_fen = _bb_get(self._bb, 'expected_board_fen')
        if not expected_fen:
            self.feedback_message = 'missing expected_board_fen on blackboard'
            return py_trees.common.Status.FAILURE

        observed = list(resp.board_state.grid)
        expected = fen_to_grid(expected_fen)
        tol = int(_bb_get(self._bb, 'verify_grid_tolerance', 6))
        move = _bb_get(self._bb, 'ai_move') or ''
        critical = uci_move_critical_indices(move)

        if critical:
            bad_critical = [i for i in critical if observed[i] != expected[i]]
            if bad_critical:
                self.feedback_message = (
                    f'move squares mismatch: {len(bad_critical)} of {len(critical)} '
                    f'critical cells (move={move})'
                )
                return py_trees.common.Status.FAILURE

        mismatches = sum(1 for a, b in zip(observed, expected) if a != b)
        if mismatches <= tol:
            self._bb.set('verification_passed', True)
            self.feedback_message = (
                f'board matches expected FEN (diff={mismatches}, tol={tol})'
            )
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f'board mismatch: {mismatches} cells differ (tol={tol})'
        return py_trees.common.Status.FAILURE
