#!/usr/bin/env python3
"""
Standalone MoveIt motion test (lab / hardware).

Run AFTER arm_drivers + moveit_config_driver, pendant on Play (External Control).
Does NOT require xiangqi_system.launch.py.

  ros2 run xiangqi_manipulation test_moveit_move --check
  ros2 run xiangqi_manipulation test_moveit_move --joints                          # from calibration YAML
  ros2 run xiangqi_manipulation test_moveit_move --joints -1.321 -0.452 -0.046 -1.983 0.302 2.282  # explicit values
  ros2 run xiangqi_manipulation test_moveit_move --ompl
  ros2 run xiangqi_manipulation test_moveit_move --ompl --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --cartesian --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --ompl --nudge-z 0.02   # current Z + 2 cm
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from moveit_msgs.action import MoveGroup

from xiangqi_manipulation.moveit_ompl_client import MoveGroupOmplClient
from xiangqi_manipulation.move_translator import BoardCalibration
from xiangqi_manipulation.calibration_paths import resolve_manipulation_calibration_path

try:
    from par_interfaces.action import WaypointMove
    from par_interfaces.srv import CurrentPose
    PAR_INTERFACES_OK = True
except ImportError:
    PAR_INTERFACES_OK = False


# Taught scan / initial pose (board_calibration.yaml)
DEFAULT_X = -0.04182
DEFAULT_Y = 0.20871
DEFAULT_Z = 0.82941
DEFAULT_YAW = -0.01369


class MoveItTestNode(Node):
    def __init__(self) -> None:
        super().__init__('test_moveit_move')


def _discovered_node_names(node: Node) -> set:
    try:
        return {n for n in node.get_node_names()}
    except AttributeError:
        pass
    try:
        return {n for n in node.get_graph().get_node_names()}
    except Exception:
        return set()


def _check_prerequisites(node: Node) -> bool:
    ok = True

    ompl_ac = ActionClient(node, MoveGroup, '/move_action')
    if ompl_ac.wait_for_server(timeout_sec=5.0):
        node.get_logger().info('OK  action /move_action')
    else:
        node.get_logger().error('Missing /move_action — run moveit_config_driver')
        ok = False
    ompl_ac.destroy()

    if PAR_INTERFACES_OK:
        wp_ac = ActionClient(node, WaypointMove, '/par_moveit/waypoint_move')
        if wp_ac.wait_for_server(timeout_sec=2.0):
            node.get_logger().info('OK  action /par_moveit/waypoint_move')
        else:
            node.get_logger().warn('No /par_moveit/waypoint_move (moveit_action_server not in launch?)')
        wp_ac.destroy()

    nodes = _discovered_node_names(node)
    if '/move_group' in nodes:
        node.get_logger().info('OK  node /move_group')
    elif ok:
        node.get_logger().info('OK  /move_action server (move_group node name not listed)')
    else:
        node.get_logger().error('No /move_group — run moveit_config_driver')
        ok = False

    if '/controller_manager' in nodes or any('controller' in n for n in nodes):
        node.get_logger().info('OK  ros2_control present')
    else:
        node.get_logger().warn('No controller_manager visible (is arm_drivers running?)')

    return ok


def _get_current_pose(node: Node, timeout_sec: float = 8.0):
    if not PAR_INTERFACES_OK:
        node.get_logger().warn('par_interfaces not available — skip current pose')
        return None
    client = node.create_client(CurrentPose, '/par_moveit/get_current_pose')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().warn('/par_moveit/get_current_pose not available')
        return None
    future = client.call_async(CurrentPose.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        node.get_logger().warn('get_current_pose call failed')
        return None
    p = future.result().pose.position
    node.get_logger().info(
        f'Current EE (get_current_pose): x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}'
    )
    return future.result().pose


DEFAULT_CAL_PATH = '/home/rosuser/workspace/config/board_calibration.yaml'


def _joint_names_from_topic(node: Node, timeout_sec: float = 4.0) -> list:
    """Read joint names from /joint_states (executor already spinning in background thread)."""
    from sensor_msgs.msg import JointState
    holder: dict = {'names': None}

    def cb(msg):
        if msg.name and holder['names'] is None:
            holder['names'] = list(msg.name)

    sub = node.create_subscription(JointState, '/joint_states', cb, 1)
    deadline = time.monotonic() + timeout_sec
    while holder['names'] is None and time.monotonic() < deadline and rclpy.ok():
        time.sleep(0.05)
    node.destroy_subscription(sub)
    return holder['names'] or []


def _run_joints(node: Node, cal_path: str, explicit_positions: list) -> bool:
    # Try to load calibration for joint names / saved positions
    joint_names = []
    joint_positions = []

    resolved = resolve_manipulation_calibration_path(cal_path, node.get_logger())
    try:
        cal = BoardCalibration.load(resolved)
        joint_names = cal.scan_joint_names or []
        joint_positions = cal.scan_joint_positions or []
    except Exception:
        cal = None

    if explicit_positions:
        joint_positions = explicit_positions
        if not joint_names:
            node.get_logger().info(
                'No joint names in calibration — reading from /joint_states ...'
            )
            joint_names = _joint_names_from_topic(node)
            if not joint_names:
                node.get_logger().error(
                    'Could not get joint names from /joint_states — is arm_drivers running?'
                )
                return False
        if len(explicit_positions) != len(joint_names):
            node.get_logger().error(
                f'Got {len(explicit_positions)} values but robot has '
                f'{len(joint_names)} joints: {joint_names}'
            )
            return False
        node.get_logger().info('Joint-space move to explicit values:')
    else:
        if not joint_names or not joint_positions:
            node.get_logger().error(
                f'No scan_joint_positions in {resolved} — '
                'run calibration_tool Step 1 first, or pass values directly: --joints v1 v2 ...'
            )
            return False
        node.get_logger().info(
            f'Joint-space move to calibrated scan pose ({len(joint_positions)} joints from {resolved}):'
        )

    for name, pos in zip(joint_names, joint_positions):
        node.get_logger().info(f'  {name}: {pos:.4f} rad')

    client = MoveGroupOmplClient(
        node,
        velocity_scaling=0.05,
        acceleration_scaling=0.05,
        allowed_planning_time=10.0,
        num_planning_attempts=10,
        action_timeout_sec=120.0,
        spin_node=False,
    )
    if not client.wait_for_server(timeout_sec=15.0):
        return False
    return client.move_to_joints(joint_names, joint_positions)


def _run_ompl(node: Node, x: float, y: float, z: float, yaw: float) -> bool:
    client = MoveGroupOmplClient(
        node,
        velocity_scaling=0.05,
        acceleration_scaling=0.05,
        allowed_planning_time=10.0,
        num_planning_attempts=10,
        action_timeout_sec=120.0,
        spin_node=False,   # executor is already spinning in thread
    )
    if not client.wait_for_server(timeout_sec=15.0):
        return False
    node.get_logger().info(
        f'OMPL test move → ({x:.4f}, {y:.4f}, {z:.4f}), yaw={yaw:.4f}'
    )
    return client.move_to_pose(x, y, z, yaw)


def _run_cartesian(node: Node, x: float, y: float, z: float, yaw: float) -> bool:
    if not PAR_INTERFACES_OK:
        node.get_logger().error('par_interfaces required for --cartesian')
        return False
    ac = ActionClient(node, WaypointMove, '/par_moveit/waypoint_move')
    if not ac.wait_for_server(timeout_sec=15.0):
        node.get_logger().error('/par_moveit/waypoint_move not available')
        return False
    goal = WaypointMove.Goal()
    goal.target_pose.position.x = float(x)
    goal.target_pose.position.y = float(y)
    goal.target_pose.position.z = float(z)
    goal.target_pose.rotation = float(yaw)
    node.get_logger().info(
        f'Cartesian test move → ({x:.4f}, {y:.4f}, {z:.4f}), yaw={yaw:.4f}'
    )
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    gh = send.result()
    if gh is None or not gh.accepted:
        node.get_logger().error('waypoint_move goal rejected')
        return False
    result_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=120.0)
    wrapped = result_fut.result()
    if wrapped is None:
        node.get_logger().error('waypoint_move timed out')
        return False
    node.get_logger().info('Cartesian move finished (check arm motion)')
    return True


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Test MoveIt arm motion on lab hardware')
    parser.add_argument('--check', action='store_true', help='Only verify MoveIt / actions')
    parser.add_argument(
        '--cal', default=DEFAULT_CAL_PATH, metavar='PATH',
        help='board_calibration.yaml path (default: %(default)s)',
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--joints', nargs='*', metavar='RAD',
                      help='Joint-space move: no values = read from --cal YAML; '
                           'or pass 6 values directly e.g. --joints -1.32 -0.45 -0.05 -1.98 0.30 2.28')
    mode.add_argument('--ompl', action='store_true', help='Test /move_action (OMPL, default)')
    mode.add_argument('--cartesian', action='store_true', help='Test /par_moveit/waypoint_move')
    parser.add_argument('--x', type=float, default=DEFAULT_X)
    parser.add_argument('--y', type=float, default=DEFAULT_Y)
    parser.add_argument('--z', type=float, default=DEFAULT_Z)
    parser.add_argument('--yaw', type=float, default=DEFAULT_YAW)
    parser.add_argument(
        '--nudge-z',
        type=float,
        default=None,
        metavar='METRES',
        help='Move to current Z + delta instead of --z (safer smoke test)',
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = MoveItTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.5)  # let executor warm up

    try:
        if not _check_prerequisites(node):
            sys.exit(1)
        if args.check:
            node.get_logger().info('Prerequisites OK (--check only)')
            return

        if args.joints is not None:
            explicit = [float(v) for v in args.joints] if args.joints else []
            ok = _run_joints(node, args.cal, explicit)
            if ok:
                _get_current_pose(node)
                node.get_logger().info('TEST PASSED')
                sys.exit(0)
            node.get_logger().error('TEST FAILED')
            sys.exit(1)

        x, y, z, yaw = args.x, args.y, args.z, args.yaw
        if args.nudge_z is not None:
            pose = _get_current_pose(node)
            if pose is None:
                node.get_logger().error('--nudge-z needs /par_moveit/get_current_pose')
                sys.exit(1)
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z + float(args.nudge_z)
            node.get_logger().info(f'Nudge target Z = current + {args.nudge_z:.3f} → z={z:.4f}')

        use_cartesian = args.cartesian
        if not args.ompl and not args.cartesian:
            use_cartesian = False  # default OMPL

        node.get_logger().info(
            'Ensure pendant is on Play (External Control). Move may take 30–90 s at 5% speed.'
        )
        time.sleep(1.0)

        if use_cartesian:
            ok = _run_cartesian(node, x, y, z, yaw)
        else:
            ok = _run_ompl(node, x, y, z, yaw)

        if ok:
            _get_current_pose(node)
            node.get_logger().info('TEST PASSED')
            sys.exit(0)
        node.get_logger().error('TEST FAILED')
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
