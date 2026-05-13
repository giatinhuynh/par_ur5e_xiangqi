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
          │     ├── VerifyBestEffort (Retry verify; swallow failure → always finalize)
          │     └── FinalizeRobotMove (``/xiangqi/ai_execution_result``)
          └── AiMotionFailureFinalizer → ``/xiangqi/ai_execution_result`` if inner Sequence fails
"""

from __future__ import annotations
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool

import py_trees
import py_trees_ros.trees

from xiangqi_msgs.msg import GameStatus
from xiangqi_msgs.msg import AiMoveCommand, AiCommandAck

from xiangqi_manipulation.move_translator import BoardCalibration, MoveTranslator

from .behaviours.game_behaviours import (
    AiMotionFailureFinalizer,
    FinalizeRobotMoveAfterVerify,
    IsCapture,
    IsEstopActive,
    SetupMoveCoordinates,
    VerifyBoardState,
)
from .behaviours.pick_and_place import PickPieceBehaviour, PlaceInGraveyardBehaviour


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        self._ack_pub = self.create_publisher(AiCommandAck, '/xiangqi/ai_command_ack', 10)

        self.declare_parameter(
            'calibration_file',
            '/home/rosuser/workspace/config/board_calibration.yaml',
        )
        self.declare_parameter('robot_plays_red', True)

        self._bb = py_trees.blackboard.Blackboard()
        self._bb.set('ai_move', None)
        self._bb.set('human_move', None)
        self._bb.set('expected_board_fen', None)
        self._bb.set('current_dispatch_id', None)
        self._bb.set('is_capture', False)
        self._bb.set('estop_active', False)
        self._bb.set('human_move_detected', False)
        self._bb.set('verify_grid_tolerance', 0)
        robot_red = self.get_parameter('robot_plays_red').value
        self._bb.set('robot_is_red', robot_red)

        cal_path = self.get_parameter('calibration_file').value
        self._load_move_translator(cal_path)

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            AiMoveCommand,
            '/xiangqi/ai_move_command',
            self._ai_move_command_cb,
            10,
            callback_group=cb_group,
        )
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
        self._tree.setup(timeout=15.0)

        self._tick_timer = self.create_timer(0.1, self._tick_tree)

        self.get_logger().info('task_planner_node started (BT running at 10 Hz)')

    def _load_move_translator(self, cal_path: str) -> None:
        """Load board calibration and construct MoveTranslator for pose generation."""
        if not os.path.isfile(cal_path):
            self.get_logger().error(
                f'Calibration file not found: {cal_path} — pick-and-place poses will fail '
                'until board_calibration.yaml exists (run calibration_tool).'
            )
            self._bb.set('move_translator', None)
            return
        try:
            cal = BoardCalibration.load(cal_path)
            if cal.board_to_base_tf is None:
                self.get_logger().error(
                    'Calibration has no board_to_base_tf — complete calibration_tool teach-in.'
                )
                self._bb.set('move_translator', None)
                return
            translator = MoveTranslator(cal)
            self._bb.set('move_translator', translator)
            self.get_logger().info(f'MoveTranslator loaded from {cal_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load MoveTranslator: {e}')
            self._bb.set('move_translator', None)

    def _ai_move_command_cb(self, msg: AiMoveCommand) -> None:
        """Atomic AI dispatch: move, capture flag, expected FEN, monotonic dispatch_id."""
        dispatch_id = int(msg.dispatch_id)
        move = msg.move
        is_capture = bool(msg.is_capture)
        expected_fen = msg.expected_fen

        busy_move = self._bb.get('ai_move', None)
        current_id = self._bb.get('current_dispatch_id', None)
        if busy_move is not None:
            if current_id is not None and dispatch_id == current_id:
                return
            nack = AiCommandAck()
            nack.dispatch_id = dispatch_id
            nack.accepted = False
            nack.reason = f'busy (current_dispatch_id={current_id})'
            self._ack_pub.publish(nack)
            self.get_logger().warn(
                f'NACK ai_move_command id={dispatch_id} (busy with id={current_id})'
            )
            return

        self._bb.set('current_dispatch_id', dispatch_id)
        self._bb.set('expected_board_fen', expected_fen)
        self._bb.set('is_capture', is_capture)
        self._bb.set('ai_move', move)
        ack = AiCommandAck()
        ack.dispatch_id = dispatch_id
        ack.accepted = True
        ack.reason = ''
        self._ack_pub.publish(ack)
        self.get_logger().info(
            f'AI dispatch id={dispatch_id} move={move} capture={is_capture}'
        )

    def _human_move_detected_cb(self, msg: Bool) -> None:
        self._bb.set('human_move_detected', msg.data)

    def _estop_cb(self, msg: Bool) -> None:
        self._bb.set('estop_active', msg.data)
        if msg.data and self._bb.get('ai_move', None) is not None:
            self.get_logger().warn('E-stop: clearing planner move blackboard')
            self._bb.set('ai_move', None)
            self._bb.set('is_capture', False)
            self._bb.set('expected_board_fen', None)
            self._bb.set('verification_passed', False)
            self._bb.set('current_dispatch_id', None)

    def _game_status_cb(self, msg: GameStatus) -> None:
        # Could extend GameStatus with robot side; keep launch parameter as source of truth
        self._bb.set('robot_is_red', self.get_parameter('robot_plays_red').value)

    def _build_tree(self) -> py_trees_ros.trees.BehaviourTree:
        estop_check = py_trees.decorators.Inverter(
            IsEstopActive(), name='NotEstopped'
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
            py_trees.decorators.FailureIsSuccess(
                py_trees.decorators.Inverter(IsCapture(), name='NotACapture'),
                name='SkipCaptureIfNone'
            ),
            capture_sequence,
        ])

        execute_sequence = py_trees.composites.Sequence(
            name='ExecuteMove', memory=True
        )
        execute_sequence.add_children([
            PickPieceBehaviour(name='PickAIPiece'),
        ])

        verify = VerifyBoardState(name='VerifyBoard')
        retry_verify = py_trees.decorators.Retry(
            verify, num_failures=3, name='RetryVerify'
        )
        # Always reach finalize: on mismatch, FinalizeRobotMoveAfterVerify publishes
        # ``board_verify_failed`` instead of ``robot_move_complete``.
        verify_best_effort = py_trees.decorators.FailureIsSuccess(
            retry_verify, name='VerifyBestEffort'
        )

        move_sequence = py_trees.composites.Sequence(
            name='MoveSequence', memory=True
        )
        move_sequence.add_children([
            SetupMoveCoordinates(),
            capture_subtree,
            execute_sequence,
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

        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        return tree

    def _tick_tree(self) -> None:
        if self._bb.get('ai_move', None) is None:
            return
        try:
            self._tree.tick()
        except Exception as e:
            self.get_logger().error(f'BT tick error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
