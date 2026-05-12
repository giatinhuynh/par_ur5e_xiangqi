"""
task_planner_node: Sequencing layer (Tier 2) -- Behavior Tree runner.

Subscribes to game manager commands, builds and ticks a Behavior Tree
that orchestrates pick-and-place moves with capture handling and verification.

Tree structure:
  Root (Selector)
    ├── EStopGuard (Sequence: check not e-stopped -> proceed)
    └── GameLoop (Sequence)
          ├── WaitForAIMove          (condition: 'ai_move' on blackboard)
          ├── SetupMoveCoordinates   (compute pick/place poses)
          ├── CaptureSubtree         (Selector)
          │     ├── NOT IsCapture    (skip if no capture)
          │     └── CaptureSequence (Sequence)
          │           ├── PlaceInGraveyard
          │           └── (success)
          ├── ExecuteMoveSequence   (Sequence)
          │     ├── PickPiece
          │     └── PlacePiece
          ├── VerifyOrRetry         (Fallback: verify up to 3x)
          └── UpdateGameState
"""

from __future__ import annotations
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool

import py_trees
import py_trees_ros.trees

from xiangqi_msgs.msg import GameStatus

from .behaviours.game_behaviours import (
    WaitForHumanMove,
    IsCapture,
    IsEstopActive,
    AlertIllegalMove,
    SetupMoveCoordinates,
    UpdateGameStateAfterMove,
    VerifyBoardState,
)
from .behaviours.pick_and_place import PickPieceBehaviour, PlaceInGraveyardBehaviour


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        self._bb = py_trees.blackboard.Blackboard()
        self._bb.set('ai_move', None)
        self._bb.set('human_move', None)
        self._bb.set('is_capture', False)
        self._bb.set('estop_active', False)
        self._bb.set('human_move_detected', False)
        self._bb.set('robot_is_red', True)
        self._bb.set('move_translator', None)

        cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self.create_subscription(
            String, '/xiangqi/execute_move', self._execute_move_cb, 10,
            callback_group=cb_group
        )
        self.create_subscription(
            Bool, '/xiangqi/human_move_detected', self._human_move_detected_cb, 10,
            callback_group=cb_group
        )
        self.create_subscription(
            Bool, '/xiangqi/estop', self._estop_cb, 10,
            callback_group=cb_group
        )
        self.create_subscription(
            GameStatus, '/xiangqi/game_status', self._game_status_cb, 10,
            callback_group=cb_group
        )

        # Build behavior tree
        self._tree = self._build_tree()
        self._tree.setup(timeout=15.0)

        # Tick tree at 10 Hz
        self._tick_timer = self.create_timer(0.1, self._tick_tree)

        self.get_logger().info('task_planner_node started (BT running at 10 Hz)')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _execute_move_cb(self, msg: String) -> None:
        move = msg.data
        self.get_logger().info(f'Task planner received move to execute: {move}')
        self._bb.set('ai_move', move)
        # Determine if this is a capture from game status board
        # (game manager sets 'is_capture' before publishing execute_move)

    def _human_move_detected_cb(self, msg: Bool) -> None:
        self._bb.set('human_move_detected', msg.data)

    def _estop_cb(self, msg: Bool) -> None:
        self._bb.set('estop_active', msg.data)

    def _game_status_cb(self, msg: GameStatus) -> None:
        self._bb.set('robot_is_red', True)  # Configured at launch

    # ------------------------------------------------------------------
    # Behavior Tree construction
    # ------------------------------------------------------------------

    def _build_tree(self) -> py_trees_ros.trees.BehaviourTree:
        # --- E-Stop guard ---
        estop_check = py_trees.decorators.Inverter(
            IsEstopActive(), name='NotEstopped'
        )

        # --- Capture subtree ---
        capture_sequence = py_trees.composites.Sequence(
            name='CaptureSequence', memory=True
        )
        capture_sequence.add_children([
            PlaceInGraveyardBehaviour(name='CaptureToGraveyard'),
        ])

        skip_if_no_capture = py_trees.behaviours.Success(name='SkipCapture')

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

        # --- Execute move sequence ---
        execute_sequence = py_trees.composites.Sequence(
            name='ExecuteMove', memory=True
        )
        execute_sequence.add_children([
            PickPieceBehaviour(name='PickAIPiece'),
        ])

        # --- Verify with retry (Fallback = try multiple times) ---
        verify = VerifyBoardState(name='VerifyBoard')
        retry_verify = py_trees.decorators.Retry(
            verify, num_failures=3, name='RetryVerify'
        )

        # --- Full move sequence ---
        move_sequence = py_trees.composites.Sequence(
            name='MoveSequence', memory=True
        )
        move_sequence.add_children([
            SetupMoveCoordinates(),
            capture_subtree,
            execute_sequence,
            retry_verify,
            UpdateGameStateAfterMove(self),
        ])

        # --- Root: guard + move ---
        root = py_trees.composites.Sequence(name='Root', memory=False)
        root.add_children([estop_check, move_sequence])

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
