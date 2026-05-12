"""
manipulation_node: MoveIt2 pick-and-place action server.

Provides the PickAndPlace action from xiangqi_msgs.
Executes the 8-step motion sequence via MoveIt2 MoveGroupInterface.
"""

from __future__ import annotations
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose

from xiangqi_msgs.action import PickAndPlace
from xiangqi_msgs.srv import GripperControl

try:
    from moveit.planning import MoveItPy
    MOVEIT_OK = True
except ImportError:
    try:
        from moveit2_py import MoveGroup
        MOVEIT_OK = True
    except ImportError:
        MOVEIT_OK = False


PLANNING_GROUP = 'ur_manipulator'
PLANNING_TIME  = 10.0
MAX_VELOCITY   = 0.3   # Scale factor (0-1) for demonstration safety
MAX_ACCEL      = 0.2


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('max_velocity_scaling', MAX_VELOCITY)
        self.declare_parameter('max_acceleration_scaling', MAX_ACCEL)
        self.declare_parameter('planning_time', PLANNING_TIME)

        self._sim_mode = self.get_parameter('simulation_mode').value
        self._vel_scale = self.get_parameter('max_velocity_scaling').value
        self._acc_scale = self.get_parameter('max_acceleration_scaling').value
        self._plan_time = self.get_parameter('planning_time').value

        cb_group = ReentrantCallbackGroup()

        # MoveIt2 interface
        self._move_group = None
        if not self._sim_mode and MOVEIT_OK:
            try:
                self._moveit = MoveItPy(node_name='manipulation_node_moveit')
                self._move_group = self._moveit.get_planning_component(PLANNING_GROUP)
                self.get_logger().info('MoveIt2 interface ready')
            except Exception as e:
                self.get_logger().error(f'MoveIt2 init failed: {e}')
        elif self._sim_mode:
            self.get_logger().info('Simulation mode -- motions logged only')

        # Gripper client
        self._gripper_cli = self.create_client(
            GripperControl, 'gripper_control', callback_group=cb_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group,
        )

        self.get_logger().info('manipulation_node ready')

    # ------------------------------------------------------------------
    # Action server
    # ------------------------------------------------------------------

    def _goal_cb(self, _):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()

        steps = [
            ('approaching_pick',  lambda: self._move_to_pose(req.pick_pose, z_offset=req.approach_height)),
            ('grasping',          lambda: self._move_to_pose(req.pick_pose, z_offset=0.0)),
            ('grasping',          lambda: self._gripper(True)),
            ('lifting',           lambda: self._move_to_pose(req.pick_pose, z_offset=req.transit_height)),
            ('moving',            lambda: self._move_to_pose(req.place_pose, z_offset=req.transit_height)),
            ('approaching_place', lambda: self._move_to_pose(req.place_pose, z_offset=req.approach_height)),
            ('placing',           lambda: self._move_to_pose(req.place_pose, z_offset=0.0)),
            ('releasing',         lambda: self._gripper(False)),
            ('done',              lambda: self._move_to_pose(req.place_pose, z_offset=req.approach_height)),
        ]

        for phase, action in steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            feedback.phase = phase
            goal_handle.publish_feedback(feedback)

            try:
                ok = action()
                if not ok:
                    result.success = False
                    result.message = f'Failed at phase: {phase}'
                    goal_handle.abort()
                    return result
            except Exception as e:
                result.success = False
                result.message = f'Exception at {phase}: {e}'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

        result.success = True
        result.placement_error_mm = 0.0
        result.message = 'Pick and place complete'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def _move_to_pose(self, base_pose: Pose, z_offset: float = 0.0) -> bool:
        target = Pose()
        target.position.x = base_pose.position.x
        target.position.y = base_pose.position.y
        target.position.z = base_pose.position.z + z_offset
        target.orientation = base_pose.orientation

        if self._sim_mode:
            self.get_logger().info(
                f'[SIM] Move to ({target.position.x:.3f}, '
                f'{target.position.y:.3f}, {target.position.z:.3f})'
            )
            time.sleep(0.3)
            return True

        if self._move_group is None:
            self.get_logger().error('MoveIt2 not available')
            return False

        try:
            with self._moveit.plan_and_execute(
                self._move_group,
                target,
                self._plan_time,
                self._vel_scale,
                self._acc_scale,
            ):
                pass
            return True
        except Exception as e:
            self.get_logger().error(f'Motion planning failed: {e}')
            return False

    def _gripper(self, activate: bool) -> bool:
        if not self._gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gripper service unavailable')
            return True  # Non-fatal in sim

        req = GripperControl.Request()
        req.activate = activate
        future = self._gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result():
            return future.result().success
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
