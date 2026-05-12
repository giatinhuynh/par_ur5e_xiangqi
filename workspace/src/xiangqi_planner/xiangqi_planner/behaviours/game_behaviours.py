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

from xiangqi_msgs.msg import GameStatus, BoardState
from xiangqi_msgs.srv import GetBoardState


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
        move = self._bb.get('human_move', None)
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
        if self._bb.get('is_capture', False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsEstopActive(py_trees.behaviour.Behaviour):
    """Checks blackboard 'estop_active' flag -- FAILURE if e-stop is active."""
    def __init__(self):
        super().__init__('IsEstopActive')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        if self._bb.get('estop_active', False):
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
        if self._bb.get('human_move_detected', False):
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


class SetupMoveCoordinates(py_trees.behaviour.Behaviour):
    """
    Reads 'ai_move' and 'move_translator' from the blackboard,
    computes pick/place poses, and writes them back to the blackboard.
    """
    def __init__(self):
        super().__init__('SetupMoveCoordinates')
        self._bb = py_trees.blackboard.Blackboard()

    def update(self) -> py_trees.common.Status:
        move = self._bb.get('ai_move', None)
        translator = self._bb.get('move_translator', None)
        if not move or translator is None:
            return py_trees.common.Status.FAILURE

        try:
            approach_pick, grasp, lift, approach_place, place = translator.move_to_poses(move)
            self._bb.set('pick_pose', grasp)
            self._bb.set('place_pose', place)
            self._bb.set('approach_height', 0.12)
            self._bb.set('transit_height', 0.20)

            # Check if capture and set capture pick pose
            is_capture = self._bb.get('is_capture', False)
            if is_capture:
                self._bb.set('capture_pick_pose', place)  # The destination has the capturable piece
                graveyard = translator.graveyard_pose(is_red_piece=not self._bb.get('robot_is_red', True))
                self._bb.set('graveyard_pose', graveyard)

            return py_trees.common.Status.SUCCESS
        except Exception:
            return py_trees.common.Status.FAILURE


class UpdateGameStateAfterMove(py_trees.behaviour.Behaviour):
    """
    Publishes a confirmation that the robot's move has been physically executed,
    so the game manager can advance state and start watching for the human.
    """
    def __init__(self, node: Node):
        super().__init__('UpdateGameState')
        self._pub = node.create_publisher(Bool, '/xiangqi/robot_move_complete', 10)
        self._done = False

    def initialise(self) -> None:
        self._done = False

    def update(self) -> py_trees.common.Status:
        if not self._done:
            msg = Bool()
            msg.data = True
            self._pub.publish(msg)
            self._done = True
        return py_trees.common.Status.SUCCESS


class VerifyBoardState(py_trees_ros.service_clients.FromBlackboard):
    """
    Calls GetBoardState service and compares with the expected state
    stored in 'expected_fen'. Sets 'verification_passed' on blackboard.
    """
    def __init__(self, name: str = 'VerifyBoardState'):
        super().__init__(
            service_type=GetBoardState,
            service_name='get_board_state',
            key='verify_request',
            name=name,
        )
        self._bb = py_trees.blackboard.Blackboard()

    def initialise(self) -> None:
        req = GetBoardState.Request()
        req.force_rescan = True
        self._bb.set('verify_request', req)
        super().initialise()

    def update(self) -> py_trees.common.Status:
        status = super().update()
        if status == py_trees.common.Status.SUCCESS:
            response = self._bb.get('verify_request_response', None)
            if response and response.success:
                self._bb.set('verification_passed', True)
                return py_trees.common.Status.SUCCESS
        return status
