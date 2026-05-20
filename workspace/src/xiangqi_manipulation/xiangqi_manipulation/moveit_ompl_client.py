"""
OMPL / joint-space arm motions for the UR5e (6-DOF) via move_group.

Uses /move_action (moveit_msgs/action/MoveGroup) — the same pipeline as RViz Plan & Execute.
MoveIt chooses joint trajectories; we only specify the end-effector goal pose.

Requires moveit_config_driver (move_group) to be running.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive


def yaw_to_downward_quaternion(yaw: float) -> Quaternion:
    """Match par_moveit_config pose_from_waypoint_pose (roll=pi, pitch=0, yaw=rotation-pi)."""
    roll = math.pi
    pitch = 0.0
    yaw_adj = yaw - math.pi
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw_adj * 0.5)
    sy = math.sin(yaw_adj * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def build_motion_plan_request(
    *,
    group_name: str,
    link_name: str,
    frame_id: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    velocity_scaling: float,
    acceleration_scaling: float,
    allowed_planning_time: float,
    num_planning_attempts: int,
) -> MotionPlanRequest:
    goal_pose = Pose()
    goal_pose.position = Point(x=float(x), y=float(y), z=float(z))
    goal_pose.orientation = yaw_to_downward_quaternion(yaw)

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.001]

    region = BoundingVolume()
    region.primitives = [sphere]
    region.primitive_poses = [goal_pose]

    pos_c = PositionConstraint()
    pos_c.header.frame_id = frame_id
    pos_c.link_name = link_name
    pos_c.target_point_offset = Point(x=0.0, y=0.0, z=0.0)
    pos_c.constraint_region = region
    pos_c.weight = 1.0

    ori_c = OrientationConstraint()
    ori_c.header.frame_id = frame_id
    ori_c.link_name = link_name
    ori_c.orientation = goal_pose.orientation
    ori_c.absolute_x_axis_tolerance = 0.15
    ori_c.absolute_y_axis_tolerance = 0.15
    ori_c.absolute_z_axis_tolerance = 0.15
    ori_c.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [pos_c]
    constraints.orientation_constraints = [ori_c]

    req = MotionPlanRequest()
    req.group_name = group_name
    req.num_planning_attempts = int(num_planning_attempts)
    req.allowed_planning_time = float(allowed_planning_time)
    req.max_velocity_scaling_factor = float(velocity_scaling)
    req.max_acceleration_scaling_factor = float(acceleration_scaling)
    req.goal_constraints = [constraints]
    return req


class MoveGroupOmplClient:
    """Plans and executes via move_group MoveGroup action (OMPL, joint-space trajectory)."""

    def __init__(
        self,
        node: Node,
        *,
        action_name: str = '/move_action',
        group_name: str = 'ur_manipulator_end_effector',
        end_effector_link: str = 'end_effector_link',
        planning_frame: str = 'base_link',
        velocity_scaling: float = 0.05,
        acceleration_scaling: float = 0.05,
        allowed_planning_time: float = 5.0,
        num_planning_attempts: int = 10,
        action_timeout_sec: float = 90.0,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._group_name = group_name
        self._link_name = end_effector_link
        self._frame_id = planning_frame
        self._velocity_scaling = velocity_scaling
        self._acceleration_scaling = acceleration_scaling
        self._allowed_planning_time = allowed_planning_time
        self._num_planning_attempts = num_planning_attempts
        self._action_timeout_sec = action_timeout_sec
        self._client: Optional[ActionClient] = None
        self._server_checked = False
        self._server_available = False

    @property
    def available(self) -> bool:
        if not self._server_checked:
            self._probe_server()
        return self._server_available

    def wait_for_server(self, timeout_sec: float = 30.0) -> bool:
        if self._client is None:
            self._client = ActionClient(self._node, MoveGroup, self._action_name)
        ok = self._client.wait_for_server(timeout_sec=timeout_sec)
        self._server_checked = True
        self._server_available = ok
        if ok:
            self._node.get_logger().info(
                f'OMPL arm planner: MoveGroup action {self._action_name!r} ready '
                f'(group={self._group_name!r}, tip={self._link_name!r})'
            )
        else:
            self._node.get_logger().warn(
                f'OMPL arm planner: {self._action_name!r} not available '
                f'(start moveit_config_driver before xiangqi arm moves)'
            )
        return ok

    def _probe_server(self, timeout_sec: float = 2.0) -> None:
        self.wait_for_server(timeout_sec=timeout_sec)

    def move_to_pose(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        if not self.available:
            return False
        assert self._client is not None

        motion_req = build_motion_plan_request(
            group_name=self._group_name,
            link_name=self._link_name,
            frame_id=self._frame_id,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            velocity_scaling=self._velocity_scaling,
            acceleration_scaling=self._acceleration_scaling,
            allowed_planning_time=self._allowed_planning_time,
            num_planning_attempts=self._num_planning_attempts,
        )

        goal = MoveGroup.Goal()
        goal.request = motion_req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self._node.get_logger().info(
            f'OMPL move → ({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f}'
        )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error('MoveGroup goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, result_future, timeout_sec=self._action_timeout_sec
        )
        wrapped = result_future.result()
        if wrapped is None:
            self._node.get_logger().error('MoveGroup timed out')
            return False

        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f'MoveGroup failed (MoveItErrorCodes.val={code})'
            )
            return False

        self._node.get_logger().info('OMPL move succeeded')
        return True
