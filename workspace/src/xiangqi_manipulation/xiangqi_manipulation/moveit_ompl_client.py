"""
OMPL / joint-space arm motions for the UR5e (6-DOF) via move_group.

Uses /move_action (moveit_msgs/action/MoveGroup) — the same pipeline as RViz Plan & Execute.
MoveIt chooses joint trajectories; we only specify the end-effector goal pose.

Requires moveit_config_driver (move_group) to be running.
"""

from __future__ import annotations

import math
import threading
from typing import Any, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    BoundingVolume,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


# Goal position tolerance sphere (metres). 1 mm is too tight for OMPL sampling.
GOAL_POSITION_TOLERANCE_M = 0.01


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


def moveit_error_name(code: int) -> str:
    """Human-readable MoveItErrorCodes name (FAILURE=99999 is generic)."""
    for name in dir(MoveItErrorCodes):
        if not name.isupper():
            continue
        if getattr(MoveItErrorCodes, name) == code:
            return name
    return 'UNKNOWN'


def _wait_on_future(future: Any, timeout_sec: float) -> Any:
    """Block without calling spin_until_future_complete (main executor must spin)."""
    if future.done():
        return future.result()
    done = threading.Event()
    holder: dict = {'value': None, 'error': None}

    def _done_cb(fut: Any) -> None:
        try:
            holder['value'] = fut.result()
        except Exception as exc:  # noqa: BLE001 — surface action errors
            holder['error'] = exc
        done.set()

    future.add_done_callback(_done_cb)
    if not done.wait(timeout_sec):
        return None
    if holder['error'] is not None:
        raise holder['error']
    return holder['value']


def build_motion_plan_request(
    *,
    group_name: str,
    link_name: str,
    frame_id: str,
    stamp: Time,
    x: float,
    y: float,
    z: float,
    yaw: float,
    velocity_scaling: float,
    acceleration_scaling: float,
    allowed_planning_time: float,
    num_planning_attempts: int,
    planner_id: str,
    pipeline_id: str,
    position_tolerance_m: float = GOAL_POSITION_TOLERANCE_M,
) -> MotionPlanRequest:
    goal_pose = Pose()
    goal_pose.position = Point(x=float(x), y=float(y), z=float(z))
    goal_pose.orientation = yaw_to_downward_quaternion(yaw)

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tolerance_m)]

    region = BoundingVolume()
    region.primitives = [sphere]
    region.primitive_poses = [goal_pose]

    pos_c = PositionConstraint()
    pos_c.header.frame_id = frame_id
    pos_c.header.stamp = stamp
    pos_c.link_name = link_name
    pos_c.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
    pos_c.constraint_region = region
    pos_c.weight = 1.0

    ori_c = OrientationConstraint()
    ori_c.header.frame_id = frame_id
    ori_c.header.stamp = stamp
    ori_c.link_name = link_name
    ori_c.orientation = goal_pose.orientation
    ori_c.absolute_x_axis_tolerance = 0.2
    ori_c.absolute_y_axis_tolerance = 0.2
    ori_c.absolute_z_axis_tolerance = 0.4
    ori_c.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
    ori_c.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [pos_c]
    constraints.orientation_constraints = [ori_c]

    req = MotionPlanRequest()
    req.group_name = group_name
    req.planner_id = planner_id
    # pipeline_id must be non-empty — rclpy empty string has a CDR null-terminator bug
    # that can corrupt the DDS stream.  When no planning_pipelines param is set, move_group
    # falls back to "move_group" as the pipeline name (see move_group.cpp main()).
    req.pipeline_id = pipeline_id if pipeline_id else 'move_group'
    req.num_planning_attempts = int(num_planning_attempts)
    req.allowed_planning_time = float(allowed_planning_time)
    req.max_velocity_scaling_factor = float(velocity_scaling)
    req.max_acceleration_scaling_factor = float(acceleration_scaling)
    req.goal_constraints = [constraints]
    # Tell move_group to start from the live joint state (same as RViz Plan & Execute)
    req.start_state = RobotState()
    req.start_state.is_diff = True
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
        planner_id: str = 'RRTConnectkConfigDefault',
        pipeline_id: str = 'move_group',
        action_timeout_sec: float = 90.0,
        callback_group: Optional[CallbackGroup] = None,
        spin_node: bool = False,
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
        self._planner_id = planner_id
        self._pipeline_id = pipeline_id
        self._action_timeout_sec = action_timeout_sec
        self._callback_group = callback_group
        self._spin_node = spin_node
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
            self._client = ActionClient(
                self._node,
                MoveGroup,
                self._action_name,
                callback_group=self._callback_group,
            )
        ok = self._client.wait_for_server(timeout_sec=timeout_sec)
        self._server_checked = True
        self._server_available = ok
        if ok:
            self._node.get_logger().info(
                f'OMPL arm planner: MoveGroup action {self._action_name!r} ready '
                f'(group={self._group_name!r}, tip={self._link_name!r}, '
                f'planner={self._planner_id!r})'
            )
        else:
            self._node.get_logger().warn(
                f'OMPL arm planner: {self._action_name!r} not available '
                f'(start moveit_config_driver before xiangqi arm moves)'
            )
        return ok

    def _probe_server(self, timeout_sec: float = 2.0) -> None:
        self.wait_for_server(timeout_sec=timeout_sec)

    def _stamp_now(self) -> Time:
        t = self._node.get_clock().now().to_msg()
        return t

    def _wait_future(self, future: Any, timeout_sec: float) -> Any:
        if self._spin_node:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=timeout_sec
            )
            if not future.done():
                return None
            return future.result()
        return _wait_on_future(future, timeout_sec)

    def move_to_pose(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        if not self.available:
            return False
        assert self._client is not None

        stamp = self._stamp_now()
        motion_req = build_motion_plan_request(
            group_name=self._group_name,
            link_name=self._link_name,
            frame_id=self._frame_id,
            stamp=stamp,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            velocity_scaling=self._velocity_scaling,
            acceleration_scaling=self._acceleration_scaling,
            allowed_planning_time=self._allowed_planning_time,
            num_planning_attempts=self._num_planning_attempts,
            planner_id=self._planner_id,
            pipeline_id=self._pipeline_id,
        )

        goal = MoveGroup.Goal()
        goal.request = motion_req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        # Required: tell move_group the planning scene diff is relative to current state
        # (matches MoveGroupInterface::move() in C++ — see constructGoal / move())
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._node.get_logger().info(
            f'OMPL move → ({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f}'
        )

        send_future = self._client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, timeout_sec=15.0)
        if goal_handle is None:
            self._node.get_logger().error(
                'MoveGroup send_goal timed out (is an executor spinning?)'
            )
            return False
        if not goal_handle.accepted:
            self._node.get_logger().error('MoveGroup goal rejected by server')
            return False

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future, timeout_sec=self._action_timeout_sec)
        if wrapped is None:
            self._node.get_logger().error('MoveGroup timed out')
            return False

        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            name = moveit_error_name(code)
            hints = []
            if code == MoveItErrorCodes.FAILURE:
                hints.append(
                    'generic FAILURE — check pendant Play (External Control), '
                    'arm_drivers, and scaled_joint_trajectory_controller'
                )
            elif code == MoveItErrorCodes.PLANNING_FAILED:
                hints.append('goal may be unreachable or in collision')
            elif code in (
                MoveItErrorCodes.CONTROL_FAILED,
                MoveItErrorCodes.TIMED_OUT,
                MoveItErrorCodes.PREEMPTED,
            ):
                hints.append('trajectory execution failed on hardware')
            hint = f' ({hints[0]})' if hints else ''
            self._node.get_logger().error(
                f'MoveGroup failed: {name} (val={code}){hint}'
            )
            return False

        self._node.get_logger().info('OMPL move succeeded')
        return True
