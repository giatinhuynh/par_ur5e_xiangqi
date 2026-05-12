"""
pick_and_place.py: py_trees behaviour wrappers for the PickAndPlace action.
"""

from __future__ import annotations
import py_trees
import py_trees_ros
import rclpy

from geometry_msgs.msg import Pose
from xiangqi_msgs.action import PickAndPlace


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
        goal.pick_pose = bb.get('pick_pose')
        goal.place_pose = bb.get('place_pose')
        goal.approach_height = float(bb.get('approach_height', 0.12))
        goal.transit_height = float(bb.get('transit_height', 0.20))
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
        goal.pick_pose = bb.get('capture_pick_pose')
        goal.place_pose = bb.get('graveyard_pose')
        goal.approach_height = float(bb.get('approach_height', 0.12))
        goal.transit_height = float(bb.get('transit_height', 0.20))
        bb.set('graveyard_goal', goal)
        super().initialise()
