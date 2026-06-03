"""
task_planner_node: Sequencing layer (Tier 2) -- Behavior Tree runner.

Subscribes to game manager commands, builds and ticks a Behavior Tree
that orchestrates pick-and-place moves with capture handling and verification.

Tree structure:
  Root (Sequence)
    ├── EStopGuard
    └── Selector ``MotionOrAbortReport``
          ├── Sequence ``MoveSequence`` (memory)
          │     ├── SetupMoveCoordinates
          │     ├── CaptureSubtree (Selector)
          │     ├── PickPiece (PickAndPlace)
          │     ├── GoToScanPose (FailureIsSuccess - arm to bird's-eye scan position)
          │     ├── VerifyBestEffort (Retry verify; swallow failure → always finalize)
          │     └── FinalizeRobotMove (``/xiangqi/ai_execution_result``)
          └── AiMotionFailureFinalizer → ``/xiangqi/ai_execution_result`` if inner Sequence fails
"""

from __future__ import annotations
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

import py_trees
import py_trees.trees

from xiangqi_msgs.action import ExecuteMove
from xiangqi_msgs.msg import GameStatus

from xiangqi_manipulation.move_translator import BoardCalibration, MoveTranslator
from xiangqi_manipulation.calibration_paths import resolve_manipulation_calibration_path

from .behaviours.game_behaviours import (
    AiMotionFailureFinalizer,
    FinalizeRobotMoveAfterVerify,
    GoToScanPose,
    IsCapture,
    IsEstopActive,
    SetupMoveCoordinates,
    VerifyBoardState,
)
from .behaviours.pick_and_place import PickPieceBehaviour, PlaceInGraveyardBehaviour


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        self.declare_parameter(
            'calibration_file',
            '/home/workspace/config/board_calibration.yaml',
        )
        self.declare_parameter('robot_plays_red', True)
        self.declare_parameter('verify_grid_tolerance', 6)
        self.declare_parameter('graveyard_slot_x', 0.25)
        self.declare_parameter('grasp_z_lower_on_retry_m', 0.005)  # 5 mm lower on retry → 2 mm at retry

        self._bb = py_trees.blackboard.Blackboard()
        self._bb.set('ai_move', None)
        self._bb.set('human_move', None)
        self._bb.set('expected_board_fen', None)
        self._bb.set('current_dispatch_id', None)
        self._bb.set('is_capture', False)
        self._bb.set('estop_active', False)
        self._bb.set('human_move_detected', False)
        self._bb.set('retry_attempt', 0)
        self._bb.set(
            'grasp_z_lower_on_retry_m',
            float(self.get_parameter('grasp_z_lower_on_retry_m').value),
        )
        self._bb.set(
            'verify_grid_tolerance',
            int(self.get_parameter('verify_grid_tolerance').value),
        )
        robot_red = self.get_parameter('robot_plays_red').value
        self._bb.set('robot_is_red', robot_red)

        cal_path = resolve_manipulation_calibration_path(
            self.get_parameter('calibration_file').value,
            self.get_logger(),
        )
        self._load_move_translator(cal_path)

        cb_group = ReentrantCallbackGroup()

        # Action server: replaces the old AiMoveCommand topic + AiCommandAck publisher.
        # execute_move_callback blocks in a thread until the BT completes the goal.
        self._action_server = ActionServer(
            self,
            ExecuteMove,
            '/xiangqi/execute_move',
            self._execute_move_callback,
            callback_group=cb_group,
        )

        # Threading primitives for signalling from BT finaliser → execute callback
        self._action_lock = threading.Lock()
        self._action_done_event: threading.Event | None = None
        self._action_final_status: int = ExecuteMove.Result.ROBOT_MOVE_COMPLETE
        self._action_final_message: str = ''

        self.create_subscription(
            Bool, '/xiangqi/human_move_detected', self._human_move_detected_cb, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Bool, '/xiangqi/estop', self._estop_cb, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            GameStatus, '/xiangqi/game_status', self._game_status_cb, 10,
            callback_group=cb_group,
        )

        self._tree = self._build_tree()
        self._tree.setup(timeout=15.0, node=self)

        self._tick_timer = self.create_timer(0.1, self._tick_tree)

        self.get_logger().info('task_planner_node started (BT at 10 Hz, ExecuteMove action server)')

    def _bb_get(self, key: str, default=None):
        """Compatibility wrapper: py_trees Blackboard.get() may not accept a default argument."""
        try:
            val = self._bb.get(key)
            return val if val is not None else default
        except Exception:
            return default

    def _load_move_translator(self, cal_path: str) -> None:
        """Load board calibration and construct MoveTranslator for pose generation."""
        if not os.path.isfile(cal_path):
            self.get_logger().error(
                f'Calibration file not found: {cal_path} - pick-and-place poses will fail '
                'until board_calibration.yaml exists (run calibration_tool).'
            )
            self._bb.set('move_translator', None)
            return
        try:
            cal = BoardCalibration.load(cal_path)
            if cal.board_to_base_tf is None:
                self.get_logger().error(
                    'Calibration has no board_to_base_tf - complete calibration_tool teach-in.'
                )
                self._bb.set('move_translator', None)
                return
            slot_x = float(self.get_parameter('graveyard_slot_x').value)
            translator = MoveTranslator(cal, graveyard_slot_x=slot_x)
            self._bb.set('move_translator', translator)
            self.get_logger().info(f'MoveTranslator loaded from {cal_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load MoveTranslator: {e}')
            self._bb.set('move_translator', None)

    def _execute_move_callback(self, goal_handle) -> ExecuteMove.Result:
        """Action server execute callback — blocks until the BT completes the goal."""
        req = goal_handle.request

        with self._action_lock:
            if self._bb_get('ai_move') is not None:
                self.get_logger().warn('ExecuteMove rejected: planner already busy')
                result = ExecuteMove.Result()
                result.success = False
                result.status_code = ExecuteMove.Result.AI_MOTION_FAILED
                result.message = 'planner busy'
                goal_handle.abort()
                return result

            done_event = threading.Event()
            self._action_done_event = done_event
            self._action_final_status = ExecuteMove.Result.ROBOT_MOVE_COMPLETE
            self._action_final_message = ''

        # Populate blackboard so the BT can start on the next tick
        self._bb.set('ai_move', req.move)
        self._bb.set('expected_board_fen', req.expected_fen)
        self._bb.set('is_capture', req.is_capture)
        self._bb.set('retry_attempt', int(req.retry_attempt))
        self._bb.set('current_dispatch_id', id(goal_handle))
        self.get_logger().info(
            f'ExecuteMove accepted: move={req.move} capture={req.is_capture} '
            f'retry={req.retry_attempt}'
        )

        # Wait for BT to signal completion (or cancellation from game_manager estop)
        while not done_event.wait(timeout=0.05):
            if goal_handle.is_cancel_requested:
                with self._action_lock:
                    self._action_done_event = None
                self._bb.set('ai_move', None)
                self._bb.set('current_dispatch_id', None)
                self.get_logger().info('ExecuteMove cancelled')
                goal_handle.canceled()
                result = ExecuteMove.Result()
                result.success = False
                result.status_code = ExecuteMove.Result.AI_MOTION_FAILED
                result.message = 'cancelled'
                return result

        with self._action_lock:
            status = self._action_final_status
            message = self._action_final_message
            self._action_done_event = None

        result = ExecuteMove.Result()
        result.status_code = status
        result.success = (status == ExecuteMove.Result.ROBOT_MOVE_COMPLETE)
        result.message = message
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def signal_action_complete(self, status: int, message: str = '') -> None:
        """Called by BT finaliser behaviours when the move sequence ends."""
        with self._action_lock:
            self._action_final_status = status
            self._action_final_message = message
            event = self._action_done_event
        if event is not None:
            event.set()

    def _human_move_detected_cb(self, msg: Bool) -> None:
        self._bb.set('human_move_detected', msg.data)

    def _estop_cb(self, msg: Bool) -> None:
        self._bb.set('estop_active', msg.data)
        if msg.data and self._bb_get('ai_move') is not None:
            self.get_logger().warn('E-stop: clearing planner blackboard')
            self._bb.set('ai_move', None)
            self._bb.set('is_capture', False)
            self._bb.set('expected_board_fen', None)
            self._bb.set('verification_passed', False)
            self._bb.set('current_dispatch_id', None)
            # Unblock any waiting execute_callback so it can return cancelled
            self.signal_action_complete(
                ExecuteMove.Result.AI_MOTION_FAILED, 'e-stop asserted'
            )

    def _game_status_cb(self, msg: GameStatus) -> None:
        # Could extend GameStatus with robot side; keep launch parameter as source of truth
        self._bb.set('robot_is_red', self.get_parameter('robot_plays_red').value)

    def _build_tree(self) -> py_trees.trees.BehaviourTree:
        estop_check = py_trees.decorators.Inverter(
            name='NotEstopped', child=IsEstopActive()
        )

        capture_sequence = py_trees.composites.Sequence(
            name='CaptureSequence', memory=True
        )
        capture_sequence.add_children([
            PlaceInGraveyardBehaviour(name='CaptureToGraveyard'),
        ])

        capture_subtree = py_trees.composites.Selector(
            name='CaptureOrSkip', memory=False
        )
        capture_subtree.add_children([
            py_trees.decorators.Inverter(name='NotACapture', child=IsCapture()),
            capture_sequence,
        ])

        execute_sequence = py_trees.composites.Sequence(
            name='ExecuteMove', memory=True
        )
        execute_sequence.add_children([
            PickPieceBehaviour(name='PickAIPiece'),
        ])

        # Move arm to top-down scan pose before verification; wrapped in
        # FailureIsSuccess so a transient arm failure does not abort the sequence.
        go_to_scan = py_trees.decorators.FailureIsSuccess(
            name='ScanPoseBestEffort', child=GoToScanPose(self)
        )

        verify = VerifyBoardState(self, name='VerifyBoard')
        retry_verify = py_trees.decorators.Retry(
            name='RetryVerify', child=verify, num_failures=4
        )
        # Always reach finalize: on mismatch, FinalizeRobotMoveAfterVerify publishes
        # ``board_verify_failed`` instead of ``robot_move_complete``.
        verify_best_effort = py_trees.decorators.FailureIsSuccess(
            name='VerifyBestEffort', child=retry_verify
        )

        move_sequence = py_trees.composites.Sequence(
            name='MoveSequence', memory=True
        )
        move_sequence.add_children([
            SetupMoveCoordinates(),
            capture_subtree,
            execute_sequence,
            go_to_scan,
            verify_best_effort,
            FinalizeRobotMoveAfterVerify(self),
        ])

        motion_or_abort = py_trees.composites.Selector(
            name='MotionOrAbortReport', memory=False
        )
        motion_or_abort.add_children([
            move_sequence,
            AiMotionFailureFinalizer(self),
        ])

        root = py_trees.composites.Sequence(name='Root', memory=False)
        root.add_children([estop_check, motion_or_abort])

        return py_trees.trees.BehaviourTree(root)

    def _tick_tree(self) -> None:
        if self._bb_get('ai_move') is None:
            return
        try:
            self._tree.tick()
        except Exception as e:
            self.get_logger().error(f'BT tick error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    # MultiThreadedExecutor required: the action execute_callback blocks on a
    # threading.Event while the BT tick timer runs concurrently.
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
