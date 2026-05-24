"""
calibration_tool: Interactive utility to calibrate the board-to-robot transform.

Usage:
  ros2 run xiangqi_vision calibration_tool

Steps:
  1. Set grid_spacing_mm in board_calibration.yaml (e.g. 61.25 from docs/board_geometry_4xA3.yaml
     for the 4×A3 mat). Place the board under the camera.
     under the camera. ArUco markers are at the **sheet corners** (all four visible). Press SPACE to capture.
  2. Using the teach pendant, move the robot TCP to 4 reference points
     (the 4 board corners), recording the TCP pose after each.
  3. The tool computes the board_to_base_tf and saves to calibration.yaml.
"""

import os
import select
import sys
import time
import json
import math
import threading
from typing import Optional
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from .board_detector import BoardDetector, BoardCalibration, BOARD_FILES, BOARD_RANKS


CALIBRATION_OUTPUT = '/home/rosuser/workspace/config/board_calibration.yaml'
MANIPULATION_CONFIG_OUTPUT = '/home/rosuser/workspace/config/manipulation_config.yaml'

# The 4 corner grid positions we use for the robot teach-in
# (file, rank) -> description
CALIBRATION_CORNERS = [
    (0, 0),   # Bottom-left  (red side, file a, rank 0)
    (8, 0),   # Bottom-right (red side, file i, rank 0)
    (8, 9),   # Top-right    (black side, file i, rank 9)
    (0, 9),   # Top-left     (black side, file a, rank 9)
]

# Reject readings closer than this to the previous corner (teach order spans ~0.5–0.9 m).
MIN_CORNER_SEPARATION_M = 0.15
# Step 2: min change in any joint (rad) vs last recorded corner — detects stale TF after Stop+Freedrive.
MIN_JOINT_DELTA_RAD = 0.02
# Corners must be at least this far from taught scan pose (scan is above the board).
MIN_SCAN_POSE_SEPARATION_M = 0.05


class CalibrationTool(Node):
    def __init__(self):
        super().__init__('calibration_tool')
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('tool_pose_topic', '/tool_pose')
        self.declare_parameter('base_frame', 'base_link')
        # Match MoveIt tip (ur_manipulator_end_effector → end_effector_link).
        self.declare_parameter('tcp_frame', 'end_effector_link')
        self._camera_topic = self.get_parameter('camera_topic').value
        self._tool_pose_topic = self.get_parameter('tool_pose_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._tcp_frame = self.get_parameter('tcp_frame').value
        self._bridge = CvBridge()
        self._detector = BoardDetector()
        self._calibration = BoardCalibration()
        self._latest_image = None
        self._captured_homography = None
        self._tcp_poses = []            # XYZ at each board-level corner
        self._corner_joints = []        # Joint positions at each board-level corner (grasp)
        self._approach_joints = []      # Joint positions at approach height above each corner
        self._latest_tcp = None
        self._latest_tcp_time = None
        self._spin_stop = None
        self._spin_thread = None
        self._latest_joint_positions = None
        self._latest_joint_names = None
        self._joints_at_last_tcp = None
        self._step2_corners = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._img_sub = self.create_subscription(
            Image, self._camera_topic, self._img_cb, 1
        )
        self.get_logger().info(f'Subscribing to {self._camera_topic}')
        self._tcp_sub = self.create_subscription(
            PoseStamped, self._tool_pose_topic, self._tcp_cb, 10
        )
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10
        )
        self.get_logger().info(
            f'TCP: optional topic {self._tool_pose_topic!r}; '
            f'else TF {self._base_frame} -> {self._tcp_frame} '
            f'(lab arm_drivers does not publish /tool_pose by default)'
        )
        # Graveyard teach-in storage: one centre position per zone
        self._graveyard_tcp = {'red': [None], 'black': [None]}
        self._graveyard_grasp_joints = {'red': [None], 'black': [None]}
        self._graveyard_approach_joints = {'red': [None], 'black': [None]}

        if os.path.isfile(CALIBRATION_OUTPUT):
            try:
                loaded = BoardCalibration.load(CALIBRATION_OUTPUT)
                if loaded.grid_spacing_mm and loaded.grid_spacing_mm > 0:
                    self._calibration = loaded
                    self.get_logger().info(
                        f'Loaded existing calibration (grid_spacing_mm={self._calibration.grid_spacing_mm})'
                    )
            except Exception as e:
                self.get_logger().warn(f'Could not load {CALIBRATION_OUTPUT}: {e}')

        self.get_logger().info('Calibration tool started. Press SPACE in the window to capture board.')

    def _img_cb(self, msg):
        self._latest_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _tcp_cb(self, msg: PoseStamped):
        self._latest_tcp = msg.pose
        self._latest_tcp_time = self.get_clock().now()

    def _joint_cb(self, msg: JointState):
        if msg.position:
            self._latest_joint_positions = tuple(float(p) for p in msg.position)
            if msg.name and self._latest_joint_names is None:
                self._latest_joint_names = list(msg.name)

    @staticmethod
    def _max_joint_delta(a: tuple, b: tuple) -> float:
        n = min(len(a), len(b))
        if n == 0:
            return 0.0
        return max(abs(a[i] - b[i]) for i in range(n))

    @staticmethod
    def _prompt_manual_tcp_xyz() -> Optional[np.ndarray]:
        vals = input('  Pendant TCP x y z (metres, space-separated): ').strip().split()
        if len(vals) != 3:
            return None
        return np.array([float(v) for v in vals])

    def _too_close_to_scan_pose(self, tcp: np.ndarray) -> bool:
        sp = self._calibration.scan_pose
        if not sp or not all(k in sp for k in ('x', 'y', 'z')):
            return False
        scan = np.array([sp['x'], sp['y'], sp['z']])
        return float(np.linalg.norm(tcp[:3] - scan)) < MIN_SCAN_POSE_SEPARATION_M

    def _start_background_spin(self):
        """Keep TF/joint_states updating while blocked on input() (single-threaded executor)."""
        self._spin_stop = threading.Event()

        def _spin_loop():
            while not self._spin_stop.is_set() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        self._spin_thread = threading.Thread(target=_spin_loop, daemon=True)
        self._spin_thread.start()

    def _stop_background_spin(self):
        if self._spin_stop is not None:
            self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._spin_stop = None
        self._spin_thread = None

    def _spin_brief(self, seconds: float = 0.5):
        """Extra spin after ENTER so TF buffer is fresh."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _tcp_from_tool_pose_topic(self):
        if self._latest_tcp is None or self._latest_tcp_time is None:
            return None
        age = (self.get_clock().now() - self._latest_tcp_time).nanoseconds / 1e9
        if age > 2.0:
            return None
        p = self._latest_tcp.position
        return np.array([p.x, p.y, p.z])

    @staticmethod
    def _lab_waypoint_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """Extract WaypointMove rotation (RZ) from quaternion — matches CurrentWaypointPose.rotation."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        while yaw > math.pi / 2:
            yaw -= math.pi
        while yaw < -math.pi / 2:
            yaw += math.pi
        return yaw

    @staticmethod
    def _unique_frame_names(primary: str, *fallbacks: str) -> list:
        names = []
        for frame in (primary, *fallbacks):
            if frame and frame not in names:
                names.append(frame)
        return names

    def _tcp_transform_from_tf(self, max_wait_s: float = 5.0):
        """Get TCP pose. Tries CurrentWaypointPose service first, then TF, then CurrentPose."""
        result = self._tcp_from_moveit_service()
        if result is not None:
            return result

        base_frames = self._unique_frame_names(self._base_frame, 'base_link', 'base')
        child_frames = self._unique_frame_names(self._tcp_frame, 'tool0', 'end_effector_link', 'flange')
        deadline = time.monotonic() + max_wait_s
        last_err = None
        while time.monotonic() < deadline and rclpy.ok():
            in_bg = self._spin_thread is not None and self._spin_thread.is_alive()
            if not in_bg:
                rclpy.spin_once(self, timeout_sec=0.05)
            else:
                time.sleep(0.05)
            for base in base_frames:
                for child in child_frames:
                    try:
                        tf = self._tf_buffer.lookup_transform(
                            base,
                            child,
                            Time(),
                            timeout=Duration(seconds=0.3),
                        )
                        t = tf.transform.translation
                        r = tf.transform.rotation
                        yaw = self._lab_waypoint_yaw(r.x, r.y, r.z, r.w)
                        pose = {'x': t.x, 'y': t.y, 'z': t.z, 'yaw': yaw}
                        self._base_frame = base
                        self.get_logger().info(
                            f'TCP from TF {base} -> {child}: '
                            f'[{pose["x"]:.4f}, {pose["y"]:.4f}, {pose["z"]:.4f}], '
                            f'yaw={pose["yaw"]:.4f}'
                        )
                        return pose
                    except (LookupException, ConnectivityException, ExtrapolationException) as e:
                        last_err = e
        if last_err is not None:
            self.get_logger().warn(f'TF lookup failed after {max_wait_s}s: {last_err}')
        return self._tcp_from_current_pose_service()

    def _tcp_from_current_pose_service(self):
        """Query /par_moveit/get_current_pose (CurrentPose); same service as test_moveit_move."""
        try:
            from par_interfaces.srv import CurrentPose
        except ImportError:
            return None
        if not hasattr(self, '_current_pose_cli'):
            self._current_pose_cli = self.create_client(
                CurrentPose, '/par_moveit/get_current_pose'
            )
        if not self._current_pose_cli.service_is_ready():
            return None
        future = self._current_pose_cli.call_async(CurrentPose.Request())
        deadline = time.monotonic() + 3.0
        in_bg = self._spin_thread is not None and self._spin_thread.is_alive()
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            if in_bg:
                time.sleep(0.05)
            else:
                rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        try:
            p = future.result().pose
        except Exception:
            return None
        r = p.orientation
        yaw = self._lab_waypoint_yaw(r.x, r.y, r.z, r.w)
        pose = {
            'x': float(p.position.x),
            'y': float(p.position.y),
            'z': float(p.position.z),
            'yaw': yaw,
        }
        self.get_logger().info(
            f'TCP from /par_moveit/get_current_pose: '
            f'[{pose["x"]:.4f}, {pose["y"]:.4f}, {pose["z"]:.4f}], yaw={pose["yaw"]:.4f}'
        )
        return pose

    def _tcp_from_moveit_service(self):
        """Fallback when TF not ready; requires moveit_config_driver."""
        try:
            from par_interfaces.srv import CurrentWaypointPose
        except ImportError:
            return None
        if not hasattr(self, '_waypoint_cli'):
            self._waypoint_cli = self.create_client(
                CurrentWaypointPose, '/par_moveit/get_current_waypoint_pose'
            )
        if not self._waypoint_cli.service_is_ready():
            return None
        future = self._waypoint_cli.call_async(CurrentWaypointPose.Request())
        deadline = time.monotonic() + 3.0
        in_bg = self._spin_thread is not None and self._spin_thread.is_alive()
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            if in_bg:
                time.sleep(0.05)
            else:
                rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        try:
            wp = future.result().pose
        except Exception:
            return None
        pose = {
            'x': float(wp.position.x),
            'y': float(wp.position.y),
            'z': float(wp.position.z),
            'yaw': float(wp.rotation),
        }
        self.get_logger().info(
            f'TCP from /par_moveit/get_current_waypoint_pose: '
            f'[{pose["x"]:.4f}, {pose["y"]:.4f}, {pose["z"]:.4f}], yaw={pose["yaw"]:.4f}'
        )
        return pose

    def _tcp_from_tf(self):
        """Lookup TCP position in base_frame (metres). Matches pendant Base frame when tcp_frame=tool0."""
        pose = self._tcp_transform_from_tf()
        if pose is None:
            return None
        return np.array([pose['x'], pose['y'], pose['z']])

    def _record_tcp_position(self):
        # After Stop+Freedrive+Play the UR driver needs time to publish real joint_states.
        wait_s = 2.5 if self._step2_corners else 0.5
        if self._spin_thread is not None and self._spin_thread.is_alive():
            time.sleep(wait_s)
        else:
            end = self.get_clock().now() + Duration(seconds=wait_s)
            while rclpy.ok() and self.get_clock().now() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        joints_now = self._latest_joint_positions
        if (
            self._step2_corners
            and self._joints_at_last_tcp is not None
            and joints_now is not None
            and self._max_joint_delta(joints_now, self._joints_at_last_tcp) < MIN_JOINT_DELTA_RAD
        ):
            print(
                '  NOTE: /joint_states unchanged — ROS still has the pre-freedrive pose.\n'
                '  Use MoveIt/pendant jog with Play ON, or enter pendant TCP manually.'
            )
            manual = self._prompt_manual_tcp_xyz()
            if manual is not None:
                print(f'  Recorded (manual): {manual}')
                return manual

        tcp = self._tcp_from_tool_pose_topic()
        if tcp is not None:
            print(f'  Recorded (topic {self._tool_pose_topic}): {tcp}')
            return tcp
        pose = self._tcp_transform_from_tf(max_wait_s=3.0)
        if pose is not None:
            tcp = np.array([pose['x'], pose['y'], pose['z']])
            print(f'  Recorded (TF {self._base_frame}): {tcp}')
            if joints_now is not None:
                self._joints_at_last_tcp = joints_now
            return tcp
        return None

    def _save_scan_and_initial_pose(self, pose: dict) -> None:
        """Store scan + initial pose from Step 1 (same TCP) into calibration YAML."""
        self._calibration.scan_pose = dict(pose)
        self._calibration.initial_pose = dict(pose)
        if self._latest_joint_positions is not None:
            self._calibration.scan_joint_positions = list(self._latest_joint_positions)
            self._calibration.scan_joint_names = self._latest_joint_names or []
            names = self._calibration.scan_joint_names
            positions = self._calibration.scan_joint_positions
            print('  Saved joint positions for joint-space homing:')
            for n, p in zip(names, positions):
                print(f'    {n}: {p:.4f} rad')
        else:
            print('  WARN: No /joint_states received — joint-space homing will not be available.')
        self._write_manipulation_config_poses(pose)
        print(
            f'\n  Saved scan_pose & initial_pose (base_link, m / rad):\n'
            f'    x={pose["x"]:.5f}  y={pose["y"]:.5f}  z={pose["z"]:.5f}  yaw={pose["yaw"]:.5f}'
        )

    def _write_manipulation_config_poses(self, pose: dict) -> None:
        """Mirror taught poses into workspace/config/manipulation_config.yaml if present."""
        path = MANIPULATION_CONFIG_OUTPUT
        if not os.path.isfile(path):
            self.get_logger().info(
                f'No {path} — poses only in {CALIBRATION_OUTPUT} '
                '(manipulation_node loads them from calibration_file).'
            )
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            params = data.setdefault('manipulation_node', {}).setdefault('ros__parameters', {})
            for prefix in ('initial_pose', 'scan_pose'):
                params[f'{prefix}_x'] = float(pose['x'])
                params[f'{prefix}_y'] = float(pose['y'])
                params[f'{prefix}_z'] = float(pose['z'])
                params[f'{prefix}_yaw'] = float(pose['yaw'])
            params['use_manual_scan_pose'] = True
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Updated {path} with taught scan/initial pose.')
        except Exception as e:
            self.get_logger().warn(f'Could not update {path}: {e}')

    def run(self):
        cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)
        self.get_logger().info(
            'Step 1: Move arm to SCAN pose (board in view, Play ON), then SPACE for ArUco.'
        )
        print(
            '\nStep 1: Park the arm at the top-down SCAN pose (full board + markers visible).'
            '\n  arm_drivers + Play must be ON. Press SPACE when ArUco overlay is OK.'
            '\n  (SPACE also saves scan_pose & initial_pose for manipulation_node.)\n'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_image is None:
                continue

            img = self._latest_image.copy()
            ok, H, debug = self._detector.detect(img)
            cv2.imshow('Calibration', debug)

            key = cv2.waitKey(30) & 0xFF
            if key == ord(' ') and ok:
                self._captured_homography = H
                self._calibration.homography = H
                self.get_logger().info('Homography captured!')
                self._spin_brief(1.0)
                taught = self._tcp_transform_from_tf(max_wait_s=5.0)
                if taught:
                    self._save_scan_and_initial_pose(taught)
                else:
                    print(
                        '\n  WARN: Could not read TCP for scan/initial pose '
                        '(Play ON?). Edit poses in board_calibration.yaml later.\n'
                    )
                break
            elif key == ord('q'):
                return

        cv2.destroyAllWindows()

        # Step 2: Record board-level + approach joints at 4 corners
        approach_h_m = getattr(self._calibration, 'approach_height_mm', 120.0) / 1000.0
        have_scan_joints = bool(
            self._calibration.scan_joint_positions
            and self._calibration.scan_joint_names
        )
        print('\nStep 2: Record grasp position + approach height at each of the 4 corners.')
        print('For each corner:')
        print('  A) Jog TCP to PIECE GRASP HEIGHT at the corner (board level + piece equator).')
        print('  B) Jog straight UP to approach height — tool shows live dZ guide.')
        if have_scan_joints:
            print('  The arm will auto-return to scan pose (joint-space) between corners.')
        print('BEST: Keep Play ON — use pendant jog or MoveIt RViz.')
        print('AVOID Stop+Freedrive: joint_states freeze; TF stays at old pose.\n')

        self._step2_corners = True
        self._joints_at_last_tcp = self._latest_joint_positions
        self._start_background_spin()
        try:
            for i, (file_idx, rank_idx) in enumerate(CALIBRATION_CORNERS):
                corner_label = f'{"abcdefghi"[file_idx]}{rank_idx}'
                print(f'\n--- Corner {i+1}/4: {corner_label} '
                      f'(file={file_idx}, rank={rank_idx}) ---')

                # ---- Part A: board-level grasp position ----
                print('  Part A: Jog to PIECE GRASP HEIGHT (board + piece equator).')
                tcp = None
                while True:
                    input('  >>> TCP at grasp position, Play ON — ENTER... ')
                    tcp = self._record_tcp_position()
                    if tcp is None:
                        self.get_logger().warn('Could not read TCP — is Play ON?')
                        tcp = self._prompt_manual_tcp_xyz()
                        if tcp is None:
                            continue
                    if self._too_close_to_scan_pose(tcp):
                        print('  REJECTED: matches scan pose. Move DOWN to piece grasp height.')
                        tcp = None
                        continue
                    if self._tcp_poses:
                        sep_mm = np.linalg.norm(tcp - self._tcp_poses[-1]) * 1000.0
                        if sep_mm < MIN_CORNER_SEPARATION_M * 1000.0:
                            print(
                                f'  REJECTED: too close to previous corner ({sep_mm:.1f} mm). '
                                'ROS pose may not have updated — try entering pendant x y z manually.'
                            )
                            manual = self._prompt_manual_tcp_xyz()
                            if manual is not None:
                                tcp = manual
                            else:
                                continue
                            if self._too_close_to_scan_pose(tcp):
                                print('  REJECTED: manual point matches scan pose.')
                                continue
                            if (np.linalg.norm(tcp - self._tcp_poses[-1]) * 1000.0
                                    < MIN_CORNER_SEPARATION_M * 1000.0):
                                print('  REJECTED: manual point still too close.')
                                continue
                    break

                self._tcp_poses.append(tcp)
                joints_now = self._latest_joint_positions
                names_now  = self._latest_joint_names
                self._joints_at_last_tcp = joints_now
                if joints_now is not None and names_now is not None:
                    self._corner_joints.append(list(joints_now))
                    print(f'  Grasp joints at {corner_label}:')
                    for jn, jv in zip(names_now, joints_now):
                        print(f'    {jn}: {jv:.4f} rad')
                else:
                    self._corner_joints.append(None)
                    print('  WARN: /joint_states not available — grasp joints not recorded.')

                # ---- Part B: approach height ----
                cz  = float(tcp[2])
                tz  = cz + approach_h_m
                print(f'\n  Part B: Jog straight UP to approach height.')
                print(f'    Grasp z:   {cz:.4f} m')
                print(f'    Target z:  {tz:.4f} m  (+{approach_h_m*1000:.0f} mm)')
                print('    Press ENTER when at target (live dZ below):\n')

                last_shown = None
                deadline = time.monotonic() + 180.0
                while time.monotonic() < deadline:
                    pose = self._tcp_transform_from_tf(max_wait_s=0.3)
                    if pose is not None:
                        dz  = pose['z'] - tz
                        line = (f'    z={pose["z"]:.4f}  |  dZ from target: {dz*1000:+.1f} mm   \r')
                        if line != last_shown:
                            print(line, end='', flush=True)
                            last_shown = line
                    r, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if r:
                        sys.stdin.readline()
                        print()
                        break
                else:
                    print('\n  Timed out — approach joints not recorded for this corner.')
                    self._approach_joints.append(None)
                    continue

                time.sleep(1.0)  # let joint_states settle
                joints_now = self._latest_joint_positions
                names_now  = self._latest_joint_names
                if joints_now is not None and names_now is not None:
                    self._approach_joints.append(list(joints_now))
                    print(f'  Approach joints at {corner_label}:')
                    for jn, jv in zip(names_now, joints_now):
                        print(f'    {jn}: {jv:.4f} rad')
                else:
                    self._approach_joints.append(None)
                    print('  WARN: /joint_states not available — approach joints not recorded.')

                # ---- Auto-return to scan pose between corners ----
                if i < 3 and have_scan_joints:
                    print(f'\n  Returning to scan pose (joint-space) before next corner…')
                    ok = self._move_to_joint_config(
                        self._calibration.scan_joint_names,
                        self._calibration.scan_joint_positions,
                    )
                    print('  At scan pose — jog to next corner.' if ok
                          else '  Auto-return failed — jog to scan pose manually.')

        finally:
            self._step2_corners = False
            self._stop_background_spin()

        # Compute board_to_base_tf and save all joint configs
        self._compute_transform()

        # Step 3: Teach graveyard drop-zone reference positions
        self._run_graveyard_step()

        self._calibration.save(CALIBRATION_OUTPUT)
        self.get_logger().info(f'Calibration saved to {CALIBRATION_OUTPUT}')
        print(f'\nCalibration saved to {CALIBRATION_OUTPUT}')

    def _move_to_joint_config(
        self, joint_names: list, joint_positions: list,
        velocity_scaling: float = 0.2,
        timeout_sec: float = 60.0,
    ) -> bool:
        """Move arm to a joint configuration via MoveGroup /move_action (joint-space OMPL).

        Requires moveit_config_driver running. Background spin must be active before calling.
        Returns True on success, False on failure / unavailable.
        """
        try:
            from moveit_msgs.action import MoveGroup
            from moveit_msgs.msg import (
                Constraints, JointConstraint, MoveItErrorCodes,
                MotionPlanRequest, PlanningOptions, RobotState,
            )
        except ImportError:
            print('  moveit_msgs not installed — cannot auto-drive (jog manually).')
            return False

        if not hasattr(self, '_move_action_client'):
            self._move_action_client = ActionClient(self, MoveGroup, '/move_action')

        client = self._move_action_client
        if not client.wait_for_server(timeout_sec=5.0):
            print('  /move_action not available — is moveit_config_driver running?')
            return False

        constraints = Constraints()
        for jname, jpos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jpos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator_end_effector'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.pipeline_id = 'move_group'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = float(velocity_scaling)
        req.max_acceleration_scaling_factor = float(velocity_scaling)
        req.goal_constraints = [constraints]
        req.start_state = RobotState()
        req.start_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        done = threading.Event()
        result_holder: dict = {'ok': False}

        def _result_cb(future):
            try:
                wrapped = future.result()
                result_holder['ok'] = (
                    wrapped.result.error_code.val == MoveItErrorCodes.SUCCESS
                )
            except Exception:
                pass
            done.set()

        def _goal_cb(future):
            try:
                gh = future.result()
            except Exception:
                done.set()
                return
            if not gh or not gh.accepted:
                print('  Joint-space move goal rejected.')
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        client.send_goal_async(goal).add_done_callback(_goal_cb)

        if not done.wait(timeout_sec):
            print('  Joint-space move timed out.')
            return False
        return result_holder['ok']

    def _run_graveyard_step(self) -> None:
        """Step 3: Teach 1 centre position per graveyard zone (red + black)."""
        approach_h_m = getattr(self._calibration, 'approach_height_mm', 120.0) / 1000.0
        have_scan_joints = bool(
            self._calibration.scan_joint_positions
            and self._calibration.scan_joint_names
        )

        print('\nStep 3: Teach graveyard joint configs (captured-piece drop zones).')
        print('Jog to the CENTRE of each graveyard zone where pieces will be dropped.')
        print('  RED zone:   where red pieces are captured (robot drops black captures here).')
        print('  BLACK zone: where black pieces are captured (robot drops red captures here).')
        print('For each zone:')
        print('  A) Jog TCP to PIECE GRASP HEIGHT at the drop centre.')
        print('  B) Jog straight UP to approach height — tool shows live dZ guide.')
        if have_scan_joints:
            print('  The arm will auto-return to scan pose between zones.')
        print()

        self._start_background_spin()
        try:
            for zone_idx, zone in enumerate(('red', 'black')):
                print(f'\n=== Graveyard zone: {zone.upper()} ===')

                # ---- Part A: grasp height ----
                print(f'  Part A: Jog to PIECE GRASP HEIGHT at the {zone} drop centre.')
                tcp = None
                while True:
                    input('  >>> TCP at grasp position, Play ON — ENTER... ')
                    tcp = self._record_tcp_position()
                    if tcp is None:
                        self.get_logger().warn('Could not read TCP — is Play ON?')
                        tcp = self._prompt_manual_tcp_xyz()
                        if tcp is None:
                            continue
                    break

                self._graveyard_tcp[zone][0] = tcp
                joints_now = self._latest_joint_positions
                names_now  = self._latest_joint_names
                if joints_now is not None and names_now is not None:
                    self._graveyard_grasp_joints[zone][0] = list(joints_now)
                    print(f'  Grasp joints at {zone} centre:')
                    for jn, jv in zip(names_now, joints_now):
                        print(f'    {jn}: {jv:.4f} rad')
                else:
                    print('  WARN: /joint_states not available — grasp joints not recorded.')

                # ---- Part B: approach height ----
                cz = float(tcp[2])
                tz = cz + approach_h_m
                print(f'\n  Part B: Jog straight UP to approach height.')
                print(f'    Grasp z:  {cz:.4f} m')
                print(f'    Target z: {tz:.4f} m  (+{approach_h_m*1000:.0f} mm)')
                print('    Press ENTER when at target (live dZ below):\n')

                last_shown = None
                deadline = time.monotonic() + 180.0
                while time.monotonic() < deadline:
                    pose = self._tcp_transform_from_tf(max_wait_s=0.3)
                    if pose is not None:
                        dz  = pose['z'] - tz
                        line = (f'    z={pose["z"]:.4f}  |  dZ from target: {dz*1000:+.1f} mm   \r')
                        if line != last_shown:
                            print(line, end='', flush=True)
                            last_shown = line
                    r, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if r:
                        sys.stdin.readline()
                        print()
                        break
                else:
                    print(f'\n  Timed out — approach joints not recorded for {zone} zone.')
                    continue

                time.sleep(1.0)
                joints_now = self._latest_joint_positions
                if joints_now is not None and names_now is not None:
                    self._graveyard_approach_joints[zone][0] = list(joints_now)
                    print(f'  Approach joints at {zone} centre:')
                    for jn, jv in zip(names_now, joints_now):
                        print(f'    {jn}: {jv:.4f} rad')
                else:
                    print('  WARN: /joint_states not available — approach joints not recorded.')

                # Auto-return to scan pose between zones
                if zone_idx == 0 and have_scan_joints:
                    print(f'\n  Returning to scan pose before black zone…')
                    ok = self._move_to_joint_config(
                        self._calibration.scan_joint_names,
                        self._calibration.scan_joint_positions,
                    )
                    print('  At scan pose.' if ok else '  Auto-return failed — jog manually.')

        finally:
            self._stop_background_spin()

        # Store recorded data in calibration object
        jnames = self._latest_joint_names
        any_saved = False
        for zone in ('red', 'black'):
            tcp  = self._graveyard_tcp[zone][0]
            g    = self._graveyard_grasp_joints[zone][0]
            a    = self._graveyard_approach_joints[zone][0]

            if tcp is None:
                print(f'  WARN: No TCP recorded for {zone} graveyard — skipping.')
                continue
            if g is None or a is None:
                print(f'  WARN: Incomplete joint data for {zone} graveyard — skipping.')
                continue

            if jnames is not None and self._calibration.graveyard_joint_names is None:
                self._calibration.graveyard_joint_names = list(jnames)

            setattr(self._calibration, f'graveyard_{zone}_y',               float(tcp[1]))
            setattr(self._calibration, f'graveyard_{zone}_grasp_joints',    list(g))
            setattr(self._calibration, f'graveyard_{zone}_approach_joints', list(a))
            print(
                f'  {zone.upper()} graveyard: y={tcp[1]:.4f} m — joints saved.'
            )
            any_saved = True

        if not any_saved:
            print('  No graveyard data saved — graveyard moves will use OMPL fallback.')

    def _compute_transform(self):
        """
        Compute the 4x4 board_frame -> robot_base rigid transform.
        board_frame origin = (file=0, rank=0), X = file direction, Y = rank direction.
        """
        spacing = self._calibration.grid_spacing_mm / 1000.0
        print(f'\nUsing grid_spacing_mm={self._calibration.grid_spacing_mm} for board frame (intersection spacing).')

        # Board-frame positions of the 4 calibration corners
        board_pts = np.array([
            [f * spacing, r * spacing, 0.0]
            for (f, r) in CALIBRATION_CORNERS
        ], dtype=float)

        # Robot-base positions recorded at those corners
        robot_pts = np.array(self._tcp_poses, dtype=float)

        # Solve: robot_pt = R @ board_pt + t  (rigid body, 3 DOF rotation + 3 DOF translation)
        # Use least squares on the centred point clouds
        board_centroid = board_pts.mean(axis=0)
        robot_centroid = robot_pts.mean(axis=0)
        B = board_pts - board_centroid
        R_pts = robot_pts - robot_centroid
        H = B.T @ R_pts
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = robot_centroid - R @ board_centroid

        tf = np.eye(4)
        tf[:3, :3] = R
        tf[:3, 3] = t
        self._calibration.board_to_base_tf = tf
        self._calibration.board_origin_mm = (0.0, 0.0)
        # Raw corner TCP positions for bilinear grid_to_world interpolation
        self._calibration.calibration_corners_base = [tcp.tolist() for tcp in self._tcp_poses]

        corner_names = ['a0', 'i0', 'i9', 'a9']
        jnames = self._latest_joint_names

        # Board-level (grasp) joint configs
        valid_grasp = [j for j in self._corner_joints if j is not None]
        if len(valid_grasp) == 4 and jnames is not None:
            self._calibration.calibration_corners_joint_names = list(jnames)
            self._calibration.calibration_corners_joints = [list(j) for j in self._corner_joints]
            print('\nGrasp joints saved for all 4 corners.')
        else:
            print(f'  Only {len(valid_grasp)}/4 grasp joints recorded.')

        # Approach-height joint configs
        valid_approach = [j for j in self._approach_joints if j is not None]
        if len(valid_approach) == 4 and jnames is not None:
            self._calibration.cell_approach_joint_names = list(jnames)
            self._calibration.cell_approach_joints = [list(j) for j in self._approach_joints]
            print('Approach joints saved for all 4 corners.')
        else:
            print(f'  Only {len(valid_approach)}/4 approach joints recorded — '
                  'board moves will use OMPL fallback.')

        print(f'\nboard_to_base_tf:\n{tf}')
        residuals = np.linalg.norm(robot_pts - (R @ board_pts.T).T - t, axis=1)
        print(f'Residuals (mm): {residuals * 1000}')


def main(args=None):
    rclpy.init(args=args)
    tool = CalibrationTool()
    tool.run()
    tool.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
