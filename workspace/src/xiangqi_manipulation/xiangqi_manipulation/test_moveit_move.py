#!/usr/bin/env python3
"""
Standalone MoveIt motion test (lab / hardware).

Run AFTER arm_drivers + moveit_config_driver, pendant on Play (External Control).
Does NOT require xiangqi_system.launch.py.

--move / --capture use the same motion stack as manipulation_node: 20% velocity,
joint-space approach/lift (4-patch bilinear), IK descend to grasp/place height.

  ros2 run xiangqi_manipulation test_moveit_move --check
  ros2 run xiangqi_manipulation test_moveit_move --joints                          # from calibration YAML
  ros2 run xiangqi_manipulation test_moveit_move --joints -1.321 -0.452 -0.046 -1.983 0.302 2.282
  ros2 run xiangqi_manipulation test_moveit_move --ompl
  ros2 run xiangqi_manipulation test_moveit_move --ompl --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --cartesian --x -0.042 --y 0.209 --z 0.829 --yaw -0.014
  ros2 run xiangqi_manipulation test_moveit_move --ompl --nudge-z 0.02
  ros2 run xiangqi_manipulation test_moveit_move --board e5              # approach only
  ros2 run xiangqi_manipulation test_moveit_move --board e5 --grasp      # approach + grip + lift
  ros2 run xiangqi_manipulation test_moveit_move --move d4 a0          # full pick-and-place (game motion)
  ros2 run xiangqi_manipulation test_moveit_move --move e5 e7            # full pick-and-place e5 → e7
  ros2 run xiangqi_manipulation test_moveit_move --capture e5 e7 --captured-red
      # capture demo: remove piece on e7 to red graveyard, then move e5 → e7
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

from sensor_msgs.msg import JointState as SensorJointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveItRobotState, MoveItErrorCodes

from xiangqi_manipulation.moveit_ompl_client import (
    MoveGroupOmplClient, _wait_on_future, yaw_to_downward_quaternion,
)
from xiangqi_manipulation.move_translator import BoardCalibration, CARTESIAN_GRASP_DROP
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

# Match manipulation_config.yaml / manipulation_node defaults
GAME_VELOCITY_SCALING = 0.2
GAME_ACCELERATION_SCALING = 0.2
GAME_ALLOWED_PLANNING_TIME = 5.0
GAME_NUM_PLANNING_ATTEMPTS = 10
GAME_PLANNER_ID = 'RRTConnectkConfigDefault'
GAME_PIPELINE_ID = 'move_group'
DEFAULT_SCAN_YAW = -1.23353


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


def _scan_yaw_from_cal(cal: BoardCalibration) -> float:
    if cal.scan_pose and 'yaw' in cal.scan_pose:
        return float(cal.scan_pose['yaw'])
    if cal.initial_pose and 'yaw' in cal.initial_pose:
        return float(cal.initial_pose['yaw'])
    return DEFAULT_SCAN_YAW


def _square_grasp_xyz(cal: BoardCalibration, file_idx: int, rank_idx: int):
    """Return (x, y, grasp_z, board_z) for a board cell (same Z logic as MoveTranslator)."""
    xyz = cal.grid_to_world(file_idx, rank_idx)
    board_z = float(xyz[2])
    grasp_z = board_z + cal.grasp_height_mm / 1000.0 - CARTESIAN_GRASP_DROP
    return float(xyz[0]), float(xyz[1]), grasp_z, board_z


def _ik_seed_at_xy(
    cal: BoardCalibration, x: float, y: float, for_approach: bool
) -> tuple[list, list] | None:
    return cal.ik_seed_joints(x, y, for_approach)


def _compute_ik(
    node: Node,
    x: float, y: float, z: float,
    scan_yaw: float,
    arm_joint_names: list,
    seed_names: list | None = None,
    seed_positions: list | None = None,
) -> tuple[list, list] | None:
    client = node.create_client(GetPositionIK, '/compute_ik')
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn('/compute_ik not available')
        node.destroy_client(client)
        return None

    req = GetPositionIK.Request()
    ik_req = PositionIKRequest()
    ik_req.group_name = 'ur_manipulator_end_effector'
    ik_req.avoid_collisions = True

    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation = yaw_to_downward_quaternion(scan_yaw)
    ik_req.pose_stamped = ps

    rs = MoveItRobotState()
    if seed_names and seed_positions:
        js = SensorJointState()
        js.name = list(seed_names)
        js.position = [float(p) for p in seed_positions]
        rs.joint_state = js
        rs.is_diff = False
    else:
        rs.is_diff = True
    ik_req.robot_state = rs
    ik_req.timeout.sec = 1

    req.ik_request = ik_req
    future = client.call_async(req)
    deadline = time.monotonic() + 4.0
    while not future.done():
        if time.monotonic() > deadline:
            node.get_logger().warn(f'IK timed out for ({x:.3f},{y:.3f},{z:.3f})')
            node.destroy_client(client)
            return None
        time.sleep(0.05)

    try:
        resp = future.result()
    except Exception as exc:
        node.get_logger().error(f'IK service error: {exc}')
        return None
    finally:
        node.destroy_client(client)

    if resp.error_code.val != MoveItErrorCodes.SUCCESS:
        node.get_logger().warn(
            f'IK no solution for ({x:.3f},{y:.3f},{z:.3f}): error_code={resp.error_code.val}'
        )
        return None

    js = resp.solution.joint_state
    if arm_joint_names:
        pairs = [(n, p) for n, p in zip(js.name, js.position) if n in arm_joint_names]
        if len(pairs) >= len(arm_joint_names):
            pairs.sort(key=lambda np_: arm_joint_names.index(np_[0]))
            names, positions = zip(*pairs)
            return list(names), list(positions)
    return list(js.name), list(js.position)


def _move_via_ik(
    node: Node,
    client: MoveGroupOmplClient,
    x: float, y: float, z: float,
    scan_yaw: float,
    arm_joint_names: list,
    seed_names: list | None = None,
    seed_positions: list | None = None,
) -> bool:
    ik = _compute_ik(
        node, x, y, z, scan_yaw, arm_joint_names, seed_names, seed_positions,
    )
    if ik is not None:
        return client.move_to_joints(ik[0], ik[1])
    node.get_logger().warn(f'IK failed for ({x:.3f},{y:.3f},{z:.3f}) — falling back to OMPL')
    return client.move_to_pose(x, y, z, scan_yaw)


def _move_board_approach(
    node: Node,
    client: MoveGroupOmplClient,
    cal: BoardCalibration,
    x: float, y: float, z: float,
    scan_yaw: float,
    arm_joint_names: list,
) -> bool:
    seed = _ik_seed_at_xy(cal, x, y, for_approach=True)
    if seed:
        return client.move_to_joints(seed[0], seed[1])
    node.get_logger().warn(f'No taught approach joints at ({x:.3f},{y:.3f}) — IK fallback')
    return _move_via_ik(node, client, x, y, z, scan_yaw, arm_joint_names)


def _move_grasp_descend(
    node: Node,
    client: MoveGroupOmplClient,
    x: float, y: float, z: float,
    scan_yaw: float,
    arm_joint_names: list,
    seed_names: list | None = None,
    seed_positions: list | None = None,
) -> bool:
    return _move_via_ik(
        node, client, x, y, z, scan_yaw, arm_joint_names, seed_names, seed_positions,
    )


def _graveyard_joints(node: Node, cal: BoardCalibration, is_red: bool):
    """Return (approach_j, grasp_j) for a graveyard zone, or (None, None) on error."""
    result = cal.get_graveyard_joints(is_red)
    if result is None:
        zone = 'red' if is_red else 'black'
        node.get_logger().error(
            f'No graveyard_{zone}_* joints in calibration - '
            'teach graveyard poses in calibration_tool first'
        )
        return None, None
    return result


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

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False
    return client.move_to_joints(joint_names, joint_positions)


def _run_ompl(node: Node, x: float, y: float, z: float, yaw: float) -> bool:
    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False
    node.get_logger().info(
        f'OMPL test move → ({x:.4f}, {y:.4f}, {z:.4f}), yaw={yaw:.4f}'
    )
    return client.move_to_pose(x, y, z, yaw)


def _make_client(node: Node) -> MoveGroupOmplClient:
    return MoveGroupOmplClient(
        node,
        velocity_scaling=GAME_VELOCITY_SCALING,
        acceleration_scaling=GAME_ACCELERATION_SCALING,
        allowed_planning_time=GAME_ALLOWED_PLANNING_TIME,
        num_planning_attempts=GAME_NUM_PLANNING_ATTEMPTS,
        planner_id=GAME_PLANNER_ID,
        pipeline_id=GAME_PIPELINE_ID,
        action_timeout_sec=120.0,
        spin_node=False,
    )


def _run_board_cell(node: Node, cal_path: str, square: str, go_to_grasp: bool) -> bool:
    """Move to a board cell.  Sequence: scan → approach → (open, IK descend, grip, lift) → scan."""
    resolved = resolve_manipulation_calibration_path(cal_path, node.get_logger())
    try:
        cal = BoardCalibration.load(resolved)
    except Exception as e:
        node.get_logger().error(f'Cannot load calibration: {e}')
        return False

    parsed = _parse_square(node, square)
    if parsed is None:
        return False
    file_idx, rank = parsed
    pick_x, pick_y, pick_gr_z, pick_board_z = _square_grasp_xyz(cal, file_idx, rank)
    approach_h = cal.approach_height_mm / 1000.0
    pick_ap_z = pick_board_z + approach_h

    scan_names = cal.scan_joint_names or []
    scan_pos   = cal.scan_joint_positions or []
    if not scan_names:
        node.get_logger().error('No scan_joint_positions in calibration')
        return False

    sq = square.strip().upper()
    scan_yaw = _scan_yaw_from_cal(cal)
    arm_joint_names = scan_names
    pick_gr_seed = _ik_seed_at_xy(cal, pick_x, pick_y, for_approach=False)
    pick_gr_n, pick_gr_p = pick_gr_seed or (None, None)

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False

    node.get_logger().info('→ scan pose')
    if not client.move_to_joints(scan_names, scan_pos):
        return False

    node.get_logger().info(f'→ approach {sq}')
    if not _move_board_approach(
        node, client, cal, pick_x, pick_y, pick_ap_z, scan_yaw, arm_joint_names,
    ):
        return False

    if go_to_grasp:
        if not _gripper(node, OPEN_WIDTH, OPEN_FORCE):
            return False
        node.get_logger().info(f'→ descend {sq} (IK)')
        if not _move_grasp_descend(
            node, client, pick_x, pick_y, pick_gr_z,
            scan_yaw, arm_joint_names, pick_gr_n, pick_gr_p,
        ):
            return False
        _gripper(node, GRASP_WIDTH, GRASP_FORCE)
        _gripper(node, RELEASE_WIDTH, OPEN_FORCE)
        node.get_logger().info(f'→ lift back to approach {sq}')
        _move_board_approach(
            node, client, cal, pick_x, pick_y, pick_ap_z, scan_yaw, arm_joint_names,
        )

    node.get_logger().info('→ scan pose')
    client.move_to_joints(scan_names, scan_pos)
    return True


def _run_game_pick_place_leg(
    node: Node,
    client: MoveGroupOmplClient,
    cal: BoardCalibration,
    scan_yaw: float,
    arm_joint_names: list,
    pick_x: float,
    pick_y: float,
    pick_gr_z: float,
    pick_board_z: float,
    place_x: float,
    place_y: float,
    place_gr_z: float,
    place_board_z: float,
    pick_label: str,
    place_label: str,
    *,
    place_joint_approach: tuple | None = None,
    place_joint_grasp: tuple | None = None,
    return_scan: bool = False,
    scan_names: list | None = None,
    scan_pos: list | None = None,
) -> bool:
    """One pick-and-place leg matching manipulation_node (joint approach/lift, IK descend)."""
    approach_h = cal.approach_height_mm / 1000.0
    pick_ap_z = pick_board_z + approach_h
    place_ap_z = place_board_z + approach_h
    pick_gr_seed = _ik_seed_at_xy(cal, pick_x, pick_y, for_approach=False) or (None, None)
    place_gr_seed = _ik_seed_at_xy(cal, place_x, place_y, for_approach=False) or (None, None)
    pick_gr_n, pick_gr_p = pick_gr_seed
    place_gr_n, place_gr_p = place_gr_seed
    place_is_graveyard = place_joint_approach is not None

    def step(label: str, fn) -> bool:
        node.get_logger().info(f'→ {label}')
        ok = fn()
        if not ok:
            node.get_logger().error(f'FAILED at: {label}')
        return ok

    if not step('open gripper', lambda: _gripper(node, OPEN_WIDTH, OPEN_FORCE)):
        return False
    if return_scan and scan_names and not step(
        'scan pose', lambda: client.move_to_joints(scan_names, scan_pos)
    ):
        return False
    if not step(
        f'approach {pick_label}',
        lambda: _move_board_approach(
            node, client, cal, pick_x, pick_y, pick_ap_z, scan_yaw, arm_joint_names,
        ),
    ):
        return False
    if not step(
        f'descend {pick_label}',
        lambda: _move_grasp_descend(
            node, client, pick_x, pick_y, pick_gr_z,
            scan_yaw, arm_joint_names, pick_gr_n, pick_gr_p,
        ),
    ):
        return False
    if not step('grip', lambda: _gripper(node, GRASP_WIDTH, GRASP_FORCE)):
        return False
    if not step(
        f'lift {pick_label}',
        lambda: _move_board_approach(
            node, client, cal, pick_x, pick_y, pick_ap_z, scan_yaw, arm_joint_names,
        ),
    ):
        return False
    if place_is_graveyard:
        gy_n, gy_p = place_joint_approach
        if not step('transit to graveyard', lambda: client.move_to_joints(gy_n, gy_p)):
            return False
        da_n, da_p = place_joint_approach
        if place_joint_grasp is not None:
            dr_n, dr_p = place_joint_grasp
        else:
            dr_n, dr_p = da_n, da_p
        if not step(f'approach {place_label}', lambda: client.move_to_joints(da_n, da_p)):
            return False
        if not step(f'descend {place_label}', lambda: client.move_to_joints(dr_n, dr_p)):
            return False
    else:
        if not step(
            f'approach {place_label}',
            lambda: _move_board_approach(
                node, client, cal, place_x, place_y, place_ap_z, scan_yaw, arm_joint_names,
            ),
        ):
            return False
        if not step(
            f'descend {place_label}',
            lambda: _move_grasp_descend(
                node, client, place_x, place_y, place_gr_z,
                scan_yaw, arm_joint_names, place_gr_n, place_gr_p,
            ),
        ):
            return False

    if not step('release', lambda: _gripper(node, RELEASE_WIDTH, OPEN_FORCE)):
        return False

    if place_is_graveyard:
        da_n, da_p = place_joint_approach
        if not step(f'lift {place_label}', lambda: client.move_to_joints(da_n, da_p)):
            return False
    elif not step(
        f'lift {place_label}',
        lambda: _move_board_approach(
            node, client, cal, place_x, place_y, place_ap_z, scan_yaw, arm_joint_names,
        ),
    ):
        return False

    if return_scan and scan_names and not step(
        'scan pose', lambda: client.move_to_joints(scan_names, scan_pos)
    ):
        return False
    return True


def _load_calibration(node: Node, cal_path: str) -> BoardCalibration | None:
    resolved = resolve_manipulation_calibration_path(cal_path, node.get_logger())
    try:
        return BoardCalibration.load(resolved)
    except Exception as e:
        node.get_logger().error(f'Cannot load calibration: {e}')
        return None


def _load_cal_and_scan(node: Node, cal_path: str):
    """Return (BoardCalibration, scan_names, scan_pos) or (None, None, None)."""
    cal = _load_calibration(node, cal_path)
    if cal is None:
        return None, None, None
    scan_names = cal.scan_joint_names or []
    scan_pos = cal.scan_joint_positions or []
    if not scan_names:
        node.get_logger().error('No scan_joint_positions in calibration')
        return None, None, None
    return cal, scan_names, scan_pos


def _run_board_move(node: Node, cal_path: str, from_sq: str, to_sq: str) -> bool:
    """Full pick-and-place between two board squares (matches manipulation_node)."""
    cal = _load_calibration(node, cal_path)
    if cal is None:
        return False
    scan_names = cal.scan_joint_names or []

    from_parsed = _parse_square(node, from_sq)
    to_parsed = _parse_square(node, to_sq)
    if from_parsed is None or to_parsed is None:
        return False

    pick_x, pick_y, pick_gr_z, pick_board_z = _square_grasp_xyz(cal, *from_parsed)
    place_x, place_y, place_gr_z, place_board_z = _square_grasp_xyz(cal, *to_parsed)

    node.get_logger().info(
        f'Pick-and-place {from_sq.upper()} → {to_sq.upper()} '
        f'(joint approach/lift, IK descend, {int(GAME_VELOCITY_SCALING * 100)}% speed)'
    )

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False

    scan_yaw = _scan_yaw_from_cal(cal)
    arm_joint_names = scan_names or cal.scan_joint_names or []

    return _run_game_pick_place_leg(
        node, client, cal, scan_yaw, arm_joint_names,
        pick_x, pick_y, pick_gr_z, pick_board_z,
        place_x, place_y, place_gr_z, place_board_z,
        from_sq.upper(), to_sq.upper(),
        return_scan=False,
    )


def _run_capture_move(
    node: Node,
    cal_path: str,
    from_sq: str,
    to_sq: str,
    captured_is_red: bool,
) -> bool:
    """Capture demo: remove opponent on destination square to graveyard, then move piece.

    Mirrors planner order (PlaceInGraveyardBehaviour then PickPieceBehaviour):
      1. Pick captured piece at TO (board; IK on manipulation_node) → graveyard (taught joints)
      2. Pick moving piece at FROM → TO (all board waypoints via IK on manipulation_node)

    This test script uses joint targets for board cells until IK helpers are wired here;
    on-robot behaviour follows manipulation_node (IK board, joint graveyard).

    Place a piece on the destination square before running (the captured victim).
    """
    cal, scan_names, scan_pos = _load_cal_and_scan(node, cal_path)
    if cal is None:
        return False

    gy_approach_j, gy_grasp_j = _graveyard_joints(node, cal, captured_is_red)
    to_parsed = _parse_square(node, to_sq)
    from_parsed = _parse_square(node, from_sq)
    if gy_approach_j is None or to_parsed is None or from_parsed is None:
        return False

    cap_pick_x, cap_pick_y, cap_pick_gr_z, cap_pick_board_z = _square_grasp_xyz(
        cal, *to_parsed,
    )
    pick_x, pick_y, pick_gr_z, pick_board_z = _square_grasp_xyz(cal, *from_parsed)
    place_x, place_y, place_gr_z, place_board_z = _square_grasp_xyz(cal, *to_parsed)

    # Red captured piece → red graveyard joints; black → black graveyard joints.
    gy_zone = 'red graveyard' if captured_is_red else 'black graveyard'
    node.get_logger().info(
        f'Capture move {from_sq.upper()} → {to_sq.upper()}: '
        f'remove {"red" if captured_is_red else "black"} piece on {to_sq.upper()} '
        f'to {gy_zone}, then relocate'
    )

    client = _make_client(node)
    if not client.wait_for_server(timeout_sec=15.0):
        return False

    scan_yaw = _scan_yaw_from_cal(cal)
    arm_joint_names = scan_names or cal.scan_joint_names or []

    node.get_logger().info(
        f'--- Phase 1: capture {to_sq.upper()} → {gy_zone} (board pick, joint graveyard) ---'
    )
    if not _run_game_pick_place_leg(
        node, client, cal, scan_yaw, arm_joint_names,
        cap_pick_x, cap_pick_y, cap_pick_gr_z, cap_pick_board_z,
        0.0, 0.0, 0.0, 0.0,  # unused for graveyard place leg
        to_sq.upper(), gy_zone,
        place_joint_approach=gy_approach_j,
        place_joint_grasp=gy_grasp_j,
        return_scan=False,
    ):
        return False

    node.get_logger().info('--- Phase 2: main pick-and-place ---')
    return _run_game_pick_place_leg(
        node, client, cal, scan_yaw, arm_joint_names,
        pick_x, pick_y, pick_gr_z, pick_board_z,
        place_x, place_y, place_gr_z, place_board_z,
        from_sq.upper(), to_sq.upper(),
        return_scan=False,
    )


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
    mode.add_argument('--capture', nargs=2, metavar=('FROM', 'TO'),
                      help='Capture demo: remove piece on TO to graveyard, then FROM → TO')
    graveyard = parser.add_mutually_exclusive_group()
    graveyard.add_argument(
        '--captured-red', action='store_true',
        help='With --capture: captured piece is red (goes to red graveyard zone)',
    )
    graveyard.add_argument(
        '--captured-black', action='store_true',
        help='With --capture: captured piece is black (goes to black graveyard zone)',
    )
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

        if args.capture is not None:
            if args.captured_red == args.captured_black:
                node.get_logger().error(
                    'Specify exactly one of --captured-red or --captured-black'
                )
                sys.exit(1)
            captured_is_red = args.captured_red
            ok = _run_capture_move(
                node, args.cal,
                args.capture[0], args.capture[1],
                captured_is_red=captured_is_red,
            )
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
            f'Ensure pendant is on Play (External Control). '
            f'Moves run at {int(GAME_VELOCITY_SCALING * 100)}% speed (same as game).'
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
