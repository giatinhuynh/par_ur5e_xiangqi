"""
pick_and_place.py: py_trees behaviour wrappers for the PickAndPlace action.
"""

from __future__ import annotations
import py_trees
import py_trees_ros
import rclpy

from geometry_msgs.msg import Pose
from xiangqi_msgs.action import PickAndPlace


def _bb_get(bb, key: str, default=None):
    """Compatibility wrapper: py_trees Blackboard.get() may not accept a default."""
    try:
        val = bb.get(key)
        return val if val is not None else default
    except Exception:
        return default


class PickPieceBehaviour(py_trees_ros.action_clients.FromBlackboard):
    """
    Sends a PickAndPlace action goal using poses from the blackboard.

    Reads from blackboard:
      'pick_pose'   (geometry_msgs/Pose)
      'place_pose'  (geometry_msgs/Pose)
      'approach_height'  (float)
      'transit_height'   (float)
    """

    def __init__(self, name: str = 'PickPiece'):
        super().__init__(
            action_type=PickAndPlace,
            action_name='/xiangqi/pick_and_place',
            key='pick_place_goal',
            name=name,
        )

    def initialise(self) -> None:
        bb = py_trees.blackboard.Blackboard()
        goal = PickAndPlace.Goal()
        goal.pick_pose = _bb_get(bb, 'pick_pose')
        goal.place_pose = _bb_get(bb, 'place_pose')
        goal.approach_height = float(_bb_get(bb, 'approach_height', 0.12))
        goal.transit_height = float(_bb_get(bb, 'transit_height', 0.20))
        bb.set('pick_place_goal', goal)
        super().initialise()


class PlaceInGraveyardBehaviour(py_trees_ros.action_clients.FromBlackboard):
    """
    Moves a captured piece to the graveyard.
    Uses 'capture_pick_pose' and 'graveyard_pose' from the blackboard.
    """

    def __init__(self, name: str = 'PlaceInGraveyard'):
        super().__init__(
            action_type=PickAndPlace,
            action_name='/xiangqi/pick_and_place',
            key='graveyard_goal',
            name=name,
        )

    def initialise(self) -> None:
        bb = py_trees.blackboard.Blackboard()
        goal = PickAndPlace.Goal()
        goal.pick_pose = _bb_get(bb, 'capture_pick_pose')
        goal.place_pose = _bb_get(bb, 'graveyard_pose')
        goal.approach_height = float(_bb_get(bb, 'approach_height', 0.12))
        goal.transit_height = float(_bb_get(bb, 'transit_height', 0.20))
        bb.set('graveyard_goal', goal)
        super().initialise()
