#!/usr/bin/env python3
"""
Standalone MoveIt motion test (lab / hardware).

Run AFTER arm_drivers + moveit_config_driver, pendant on Play (External Control).
Does NOT require xiangqi_system.launch.py.

  ros2 run xiangqi_manipulation test_moveit_move --check
  ros2 run xiangqi_manipulation test_moveit_move --joints                          # from calibration YAML
  ros2 run xiangqi_manipulation test_moveit_move --joints -1.321 -0.452 -0.046 -1.983 0.302 2.282
  ros2 run xiangqi_manipulation test_moveit_move --ompl
  ros2 run xiangqi_manipulation test_moveit_move --ompl --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --cartesian --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --ompl --nudge-z 0.02
  ros2 run xiangqi_manipulation test_moveit_move --board e5              # approach only
  ros2 run xiangqi_manipulation test_moveit_move --board e5 --grasp      # approach + grip + lift
  ros2 run xiangqi_manipulation test_moveit_move --move e5 e7            # full pick-and-place e5 → e7
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

from xiangqi_manipulation.moveit_ompl_client import MoveGroupOmplClient, _wait_on_future
from xiangqi_manipulation.move_translator import BoardCalibration
from xiangqi_manipulation.calibration_paths import resolve_manipulation_calibration_path

try:
    from par_interfaces.action import WaypointMove
    from par_interfaces.srv import CurrentPose
    PAR_INTERFACES_OK = True
except ImportError:
    PAR_INTERFACES_OK = False

try:
    from onrobot_rg2_msgs.action import GripperSetWidth
    RG2_OK = True
except ImportError:
    RG2_OK = False

OPEN_WIDTH    = 50.0
GRASP_WIDTH   = 18.0
RELEASE_WIDTH = 34.0
GRASP_FORCE   = 15.0
OPEN_FORCE    = 10.0


def _gripper(node: Node, width: float, force: float, timeout_sec: float = 10.0) -> bool:
    """Send a single gripper width command. Returns True on success."""
    if not RG2_OK:
        node.get_logger().warn(f'[NO RG2] gripper → {width:.0f} mm @ {force:.0f} N (skipped)')
        return True
    ac = ActionClient(node, GripperSetWidth, '/rg2/set_width')
    if not ac.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('/rg2/set_width not available')
        return False
    goal = GripperSetWidth.Goal()
    goal.target_width = float(width)
    goal.target_force = float(force)
    # Use _wait_on_future (polling) - rclpy.spin_until_future_complete is unsafe when a
    # MultiThreadedExecutor is already spinning on another thread and corrupts action state.
    gh = _wait_on_future(ac.send_goal_async(goal), timeout_sec=8.0)
    if gh is None or not gh.accepted:
        node.get_logger().error('Gripper goal rejected')
        return False
    result = _wait_on_future(gh.get_result_async(), timeout_sec=timeout_sec)
    if result is None:
        node.get_logger().error('Gripper timed out')
        return False
    node.get_logger().info(f'Gripper at {result.result.final_width:.1f} mm')
    return True


def _parse_square(node: Node, square: str):
    """Parse algebraic square (e.g. 'e5') to (file_idx, rank). Returns None on error."""
    sq = square.strip().lower()
    if len(sq) < 2 or sq[0] not in 'abcdefghi':
        node.get_logger().error(f'Invalid square {square!r} - use file (a-i) + rank (0-9), e.g. e5')
        return None
    try:
        rank = int(sq[1:])
    except ValueError:
        node.get_logger().error(f'Invalid rank in {square!r}')
        return None
    if rank < 0 or rank > 9:
        node.get_logger().error(f'Rank {rank} out of range (0-9)')
        return None
    return ord(sq[0]) - ord('a'), rank


def _cell_joints(node: Node, cal: BoardCalibration, square: str):
    """Return (approach_j, grasp_j) for a board square, or (None, None) on error."""
    parsed = _parse_square(node, square)
    if parsed is None:
        return None, None
    file_idx, rank = parsed
    approach = cal.interpolate_approach_joints(float(file_idx), float(rank))
    grasp    = cal.interpolate_board_joints(float(file_idx), float(rank))
    if approach is None or grasp is None:
        node.get_logger().error(
            'Board joint configs missing - run calibration_tool Step 2 first'
        )
        return None, None
    return approach, grasp


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
        node.get_logger().error('Missing /move_action - run moveit_config_driver')
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
        node.get_logger().error('No /move_group - run moveit_config_driver')
        ok = False

    if '/controller_manager' in nodes or any('controller' in n for n in nodes):
        node.get_logger().info('OK  ros2_control present')
    else:
        node.get_logger().warn('No controller_manager visible (is arm_drivers running?)')

    return ok


def _get_current_pose(node: Node, timeout_sec: float = 8.0):
    if not PAR_INTERFACES_OK:
        node.get_logger().warn('par_interfaces not available - skip current pose')
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
                'No joint names in calibration - reading from /joint_states ...'
            )
            joint_names = _joint_names_from_topic(node)
            if not joint_names:
                node.get_logger().error(
                    'Could not get joint names from /joint_states - is arm_drivers running?'
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
                f'No scan_joint_positions in {resolved} - '
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


def _make_client(node: Node) -> MoveGroupOmplClient:
    return MoveGroupOmplClient(
        node,
        velocity_scaling=0.05,
        acceleration_scaling=0.05,
        allowed_planning_time=10.0,
        num_planning_attempts=10,
        action_timeout_sec=120.0,
        spin_node=False,
    )


def _run_board_cell(node: Node, cal_path: str, square: str, go_to_grasp: bool) -> bool:
    """Move to a board cell.  Sequence: scan → approach → (open, grasp, grip, lift) → scan."""
    resolved = resolve_manipulation_calibration_path(cal_path, node.get_logger())
    try:
        cal = BoardCalibration.load(resolved)
    except Exception as e:
        node.get_logger().error(f'Cannot load calibration: {e}')
        return False

    approach_j, grasp_j = _cell_joints(node, cal, square)
    if approach_j is None:
        return False

    approach_names, approach_pos = approach_j
    grasp_names,    grasp_pos    = grasp_j
    scan_names = cal.scan_joint_names or []
    scan_pos   = cal.scan_joint_positions or []
    if not scan_names:
        node.get_logger().error('No scan_joint_positions in calibration')
        return False

    sq = square.strip().upper()
    node.get_logger().info(f'--board {sq}: approach joints:')
    for n, p in zip(approach_names, approach_pos):
        node.get_logger().info(f'  {n}: {p:.4f}')
    if go_to_grasp:
        node.get_logger().info(f'--board {sq}: grasp joints:')
        for n, p in zip(grasp_names, grasp_pos):
            node.get_logger().info(f'  {n}: {p:.4f}')

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False

    node.get_logger().info('→ scan pose')
    if not client.move_to_joints(scan_names, scan_pos):
        return False

    node.get_logger().info(f'→ approach {sq}')
    if not client.move_to_joints(approach_names, approach_pos):
        return False

    if go_to_grasp:
        _gripper(node, OPEN_WIDTH, OPEN_FORCE)
        node.get_logger().info(f'→ grasp {sq}')
        if not client.move_to_joints(grasp_names, grasp_pos):
            return False
        _gripper(node, GRASP_WIDTH, GRASP_FORCE)
        _gripper(node, RELEASE_WIDTH, OPEN_FORCE)
        node.get_logger().info(f'→ lift back to approach {sq}')
        client.move_to_joints(approach_names, approach_pos)

    node.get_logger().info('→ scan pose')
    client.move_to_joints(scan_names, scan_pos)
    return True


def _run_board_move(node: Node, cal_path: str, from_sq: str, to_sq: str) -> bool:
    """Full pick-and-place between two board squares using joint-space interpolation.

    Sequence: open → scan → approach_pick → grasp_pick → grip →
              lift_pick → approach_place → place → release → lift_place → scan
    """
    resolved = resolve_manipulation_calibration_path(cal_path, node.get_logger())
    try:
        cal = BoardCalibration.load(resolved)
    except Exception as e:
        node.get_logger().error(f'Cannot load calibration: {e}')
        return False

    pick_approach_j,  pick_grasp_j  = _cell_joints(node, cal, from_sq)
    place_approach_j, place_grasp_j = _cell_joints(node, cal, to_sq)
    if pick_approach_j is None or place_approach_j is None:
        return False

    scan_names = cal.scan_joint_names or []
    scan_pos   = cal.scan_joint_positions or []
    if not scan_names:
        node.get_logger().error('No scan_joint_positions in calibration')
        return False

    pa_n, pa_p = pick_approach_j
    pg_n, pg_p = pick_grasp_j
    da_n, da_p = place_approach_j
    dg_n, dg_p = place_grasp_j

    node.get_logger().info(
        f'Pick-and-place {from_sq.upper()} → {to_sq.upper()} (joint-space, 5% speed)'
    )

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False

    def step(label: str, fn) -> bool:
        node.get_logger().info(f'→ {label}')
        ok = fn()
        if not ok:
            node.get_logger().error(f'FAILED at: {label}')
        return ok

    if not step('open gripper',            lambda: _gripper(node, OPEN_WIDTH, OPEN_FORCE)):
        return False
    if not step('scan pose',               lambda: client.move_to_joints(scan_names, scan_pos)):
        return False
    if not step(f'approach {from_sq.upper()}', lambda: client.move_to_joints(pa_n, pa_p)):
        return False
    if not step(f'grasp {from_sq.upper()}',    lambda: client.move_to_joints(pg_n, pg_p)):
        return False
    if not step('grip',                    lambda: _gripper(node, GRASP_WIDTH, GRASP_FORCE)):
        return False
    if not step(f'lift {from_sq.upper()}',     lambda: client.move_to_joints(pa_n, pa_p)):
        return False
    if not step(f'approach {to_sq.upper()}',   lambda: client.move_to_joints(da_n, da_p)):
        return False
    if not step(f'place {to_sq.upper()}',      lambda: client.move_to_joints(dg_n, dg_p)):
        return False
    if not step('release',                 lambda: _gripper(node, RELEASE_WIDTH, OPEN_FORCE)):
        return False
    if not step(f'lift {to_sq.upper()}',       lambda: client.move_to_joints(da_n, da_p)):
        return False
    if not step('scan pose (done)',        lambda: client.move_to_joints(scan_names, scan_pos)):
        return False

    return True


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
    mode.add_argument('--board', metavar='SQUARE',
                      help='Move to board cell via joint interpolation, e.g. --board e5. '
                           'Add --grasp to open gripper, descend, grip, lift.')
    mode.add_argument('--move', nargs=2, metavar=('FROM', 'TO'),
                      help='Full pick-and-place, e.g. --move e5 e7')
    parser.add_argument('--grasp', action='store_true',
                        help='With --board: open gripper, descend to grasp, grip, lift')
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

        if args.board is not None:
            ok = _run_board_cell(node, args.cal, args.board, args.grasp)
            node.get_logger().info('TEST PASSED' if ok else 'TEST FAILED')
            sys.exit(0 if ok else 1)

        if args.move is not None:
            ok = _run_board_move(node, args.cal, args.move[0], args.move[1])
            node.get_logger().info('TEST PASSED' if ok else 'TEST FAILED')
            sys.exit(0 if ok else 1)

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
