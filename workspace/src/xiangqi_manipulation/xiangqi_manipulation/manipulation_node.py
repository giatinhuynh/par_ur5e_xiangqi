"""
manipulation_node: Pick-and-place action server for Xiangqi pieces.

Arm motion (hardware):
    Homing:      joint-space via /move_action (scan_joint_positions from YAML).
    Board cells: taught joint-space for approach / lift / transit; Cartesian (or IK) only for
                 the short vertical descend to grasp or release. Graveyard legs stay all joint.
    Board XY/Z from ArUco-derived board frame (FlatBoardLocator / board_to_base_tf).

Board frame: derived at startup by calling the vision node GetBoardTransform service.
             The vision node uses cv2.solvePnP on detected ArUco markers and transforms
             via a static TF (camera_config.yaml) to give 4 marker 3D positions in base_link.

The RG2 gripper is controlled via (from onrobot_rg2_driver):

    /rg2/set_width  (onrobot_rg2_msgs/action/GripperSetWidth)
        goal:   float32 target_width   (mm)
                float32 target_force   (N)
        result: float32 final_width    (mm)

This node accepts xiangqi_msgs/action/PickAndPlace goals (in robot base frame
metres) and executes the full pick-and-place sequence.

Coordinate conventions:
    - pick_pose / place_pose are geometry_msgs/Pose in the robot base_link frame.
    - Only position (x, y, z) is used; orientation is ignored because Xiangqi
      pieces are round and the gripper is always pointing straight down.
    - approach_height and transit_height are z offsets added on top of board_z.

Gripper widths (all in mm):
    OPEN_WIDTH    = 50   Pre-grasp open; fingers clear around a ~20 mm piece
    GRASP_WIDTH   = 18   Closed on piece (~20 mm diameter, 2 mm compression)
    RELEASE_WIDTH = 50   Open enough to lift off a released piece
    GRASP_FORCE   = 15 N Firm but gentle
"""

from __future__ import annotations
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveItRobotState, MoveItErrorCodes

from xiangqi_msgs.action import PickAndPlace
from xiangqi_msgs.srv import GetBoardTransform
from xiangqi_manipulation.move_translator import BoardCalibration, FlatBoardLocator
from xiangqi_manipulation.calibration_paths import resolve_manipulation_calibration_path
from xiangqi_manipulation.moveit_ompl_client import (
    MoveGroupOmplClient, _wait_on_future, yaw_to_downward_quaternion
)

try:
    from onrobot_rg2_msgs.action import GripperSetWidth
    RG2_OK = True
except ImportError:
    RG2_OK = False

try:
    from par_interfaces.action import WaypointMove
    WAYPOINT_MOVE_OK = True
except ImportError:
    WAYPOINT_MOVE_OK = False


# Gripper widths in millimetres
OPEN_WIDTH    = 50.0   # Clearance width before descending onto piece
GRASP_WIDTH   = 18.0   # Grip width for ~20 mm diameter Xiangqi piece
RELEASE_WIDTH = 50.0   # Full open at release so piece drops cleanly (same as OPEN_WIDTH)
GRASP_FORCE   = 15.0   # Newtons - firm grip without crushing
OPEN_FORCE    = 10.0   # Newtons - gentle open


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        self.declare_parameter('simulation_mode',    False)
        self.declare_parameter('open_width',         OPEN_WIDTH)
        self.declare_parameter('grasp_width',        GRASP_WIDTH)
        self.declare_parameter('release_width',      RELEASE_WIDTH)
        self.declare_parameter('grasp_force',        GRASP_FORCE)
        # Rest / scan pose (base_link, metres / radians). Pendant: 41.11, -357.06, 459.54 mm; RZ=-0.339.
        self.declare_parameter('initial_pose_x',     -0.04182)
        self.declare_parameter('initial_pose_y',      0.20871)
        self.declare_parameter('initial_pose_z',      0.82941)
        self.declare_parameter('initial_pose_yaw',   -0.01369)
        self.declare_parameter('scan_pose_x',        -0.04182)
        self.declare_parameter('scan_pose_y',        0.20871)
        self.declare_parameter('scan_pose_z',         0.82941)
        self.declare_parameter('scan_pose_yaw',      -0.01369)
        self.declare_parameter('use_manual_scan_pose', True)
        self.declare_parameter(
            'calibration_file',
            '/home/rosuser/workspace/config/board_calibration.yaml',
        )
        self.declare_parameter('move_to_initial_pose_on_startup', True)
        self.declare_parameter('startup_move_delay_sec', 5.0)
        self.declare_parameter('startup_move_max_retries', 3)
        self.declare_parameter('move_max_retries', 2)
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('move_group_name', 'ur_manipulator_end_effector')
        self.declare_parameter('end_effector_link', 'end_effector_link')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('max_velocity_scaling_factor', 0.2)
        self.declare_parameter('max_acceleration_scaling_factor', 0.2)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')
        self.declare_parameter('pipeline_id', 'move_group')
        # Height of gripper centre above board surface when grasping a piece (mm).
        # Typically piece_height / 2; for 20mm pieces side-gripped ≈ 10mm.
        self.declare_parameter('grasp_height_mm', 2.0)
        # Below this final_width (mm) after closing = no piece grasped (fingers closed through air)
        self.declare_parameter('grasp_check_min_width_mm', 12.0)
        # How much lower (m) to re-descend on a failed grasp before retrying
        self.declare_parameter('grasp_retry_lower_m', 0.005)
        # Graveyard positions: fixed XY in base_link (Z = board_z + grasp_height)
        self.declare_parameter('graveyard_red_x',    0.25)
        self.declare_parameter('graveyard_red_y',   -0.30)
        self.declare_parameter('graveyard_black_x',  0.25)
        self.declare_parameter('graveyard_black_y',  0.30)

        self._sim_mode      = self.get_parameter('simulation_mode').value
        self._move_max_retries = int(self.get_parameter('move_max_retries').value)
        self._startup_timer = None
        self._startup_move_done = False
        self._open_width    = self.get_parameter('open_width').value
        self._grasp_width   = self.get_parameter('grasp_width').value
        self._release_width = self.get_parameter('release_width').value
        self._grasp_force   = self.get_parameter('grasp_force').value
        self._scan_pose_yaw = self.get_parameter('scan_pose_yaw').value
        self._initial_pose_x = float(self.get_parameter('initial_pose_x').value)
        self._initial_pose_y = float(self.get_parameter('initial_pose_y').value)
        self._initial_pose_z = float(self.get_parameter('initial_pose_z').value)
        self._initial_pose_yaw = float(self.get_parameter('initial_pose_yaw').value)
        self._grasp_height_m         = float(self.get_parameter('grasp_height_mm').value) / 1000.0
        self._grasp_check_min_width  = float(self.get_parameter('grasp_check_min_width_mm').value)
        self._grasp_retry_lower_m    = float(self.get_parameter('grasp_retry_lower_m').value)
        self._last_gripper_width: float = self._open_width
        self._graveyard_red_x    = float(self.get_parameter('graveyard_red_x').value)
        self._graveyard_red_y    = float(self.get_parameter('graveyard_red_y').value)
        self._graveyard_black_x  = float(self.get_parameter('graveyard_black_x').value)
        self._graveyard_black_y  = float(self.get_parameter('graveyard_black_y').value)

        # Board frame from ArUco (populated at startup via GetBoardTransform service)
        self._flat_locator: FlatBoardLocator | None = None
        self._board_to_base_tf: np.ndarray | None = None   # 4×4 float64
        self._board_z: float = 0.0

        # IK seeds: a0 corner fallback; per-cell seeds from 4-corner bilinear interpolation.
        self._a0_grasp_seed_names:     list = []
        self._a0_grasp_seed_positions:  list = []
        self._a0_approach_seed_names:  list = []
        self._a0_approach_seed_positions: list = []

        self._taught_scan_pose = False
        self._initial_joint_positions: list = []
        self._initial_joint_names: list = []
        self._board_calibration: BoardCalibration | None = None
        self._apply_taught_poses_from_calibration()

        if not self._taught_scan_pose:
            self._scan_pose_x, self._scan_pose_y, self._scan_pose_z = \
                self._resolve_scan_pose()

        # Callback groups: pick/place server vs arm vs gripper.
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        self._arm_cbg = MutuallyExclusiveCallbackGroup()
        self._ompl_cbg = ReentrantCallbackGroup()
        self._grip_cbg = MutuallyExclusiveCallbackGroup()
        self._move_lock = threading.Lock()

        # --- OMPL arm planner + IK service client + gripper ---
        self._ompl_client = None
        self._ik_client   = None
        self._waypoint_client = None
        self._rg2_client  = None

        if not self._sim_mode:
            self._setup_arm_planner()
            self._ik_client = self.create_client(
                GetPositionIK, '/compute_ik',
                callback_group=self._arm_cbg,
            )
            if WAYPOINT_MOVE_OK:
                self._waypoint_client = ActionClient(
                    self, WaypointMove, '/par_moveit/waypoint_move',
                    callback_group=self._ompl_cbg,
                )
            else:
                self.get_logger().warn(
                    'par_interfaces not found - Cartesian descend/lift will fall back to OMPL'
                )
            if RG2_OK:
                self._rg2_client = ActionClient(
                    self, GripperSetWidth, '/rg2/set_width',
                    callback_group=self._grip_cbg
                )
            else:
                self.get_logger().warn(
                    'onrobot_rg2_msgs not found - gripper motion will be simulated'
                )

        # --- Action server for the rest of the xiangqi stack ---
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            '/xiangqi/pick_and_place',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._server_cbg,
        )

        # --- Services: rest (initial) and top-down scan poses ---
        self._initial_pose_srv = self.create_service(
            Trigger,
            '/xiangqi/move_to_initial_pose',
            self._move_to_initial_pose_cb,
            callback_group=self._arm_cbg,
        )
        self._scan_pose_srv = self.create_service(
            Trigger,
            '/xiangqi/move_to_scan_pose',
            self._move_to_scan_pose_cb,
            callback_group=self._arm_cbg,
        )

        # --- GetBoardTransform service client (vision node) ---
        self._board_transform_client = self.create_client(
            GetBoardTransform, 'get_board_transform',
            callback_group=self._arm_cbg,
        )

        arm_backend = 'sim' if self._sim_mode else ('ompl' if self._ompl_client else 'none')
        has_joints = bool(self._initial_joint_positions)
        self.get_logger().info(
            f'manipulation_node ready '
            f'(sim={self._sim_mode}, arm={arm_backend}, '
            f'joint_homing={has_joints}, aruco_board=pending_startup, rg2={RG2_OK})'
        )

        if (
            not self._sim_mode
            and self.get_parameter('move_to_initial_pose_on_startup').value
        ):
            delay = float(self.get_parameter('startup_move_delay_sec').value)
            self.get_logger().info(
                f'Startup: will move to scan pose + acquire board frame in {delay:.1f}s '
                f'(planner={arm_backend}; pendant Play + moveit_config_driver required)'
            )
            self._startup_timer = self.create_timer(
                delay,
                self._startup_move_to_initial_pose_cb,
                callback_group=self._arm_cbg,
            )

    def _refresh_board_frame(self) -> bool:
        """Call vision node GetBoardTransform service and store the FlatBoardLocator.

        Returns True on success. Safe to call from a daemon thread (blocking).
        """
        if not self._board_transform_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'get_board_transform service not available (vision_node running?)'
            )
            return False

        req = GetBoardTransform.Request()
        future = self._board_transform_client.call_async(req)

        deadline = time.monotonic() + 10.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error('GetBoardTransform request timed out')
                return False
            time.sleep(0.05)

        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'GetBoardTransform call failed: {exc}')
            return False

        if not resp.success:
            self.get_logger().error(f'GetBoardTransform failed: {resp.message}')
            return False

        try:
            self._flat_locator = FlatBoardLocator(list(resp.marker_centres_base))
            self._board_z = self._flat_locator.board_z
            tf_flat = list(resp.board_to_base)
            if len(tf_flat) == 16:
                self._board_to_base_tf = np.array(tf_flat, dtype=np.float64).reshape(4, 4)
            a0 = self._flat_locator.cell_xyz(0, 0)
            self.get_logger().info(
                f'Board frame acquired: board_z={self._board_z:.4f} m  '
                f'a0=({a0[0]:.3f},{a0[1]:.3f})  '
                f'grasp_z={self._board_z + self._grasp_height_m:.4f} m'
            )
        except Exception as exc:
            self.get_logger().error(f'FlatBoardLocator construction failed: {exc}')
            return False

        return True

    def _setup_arm_planner(self) -> None:
        """OMPL via move_group /move_action (joint-space, same as RViz)."""
        self._ompl_client = MoveGroupOmplClient(
            self,
            action_name=str(self.get_parameter('move_group_action').value),
            group_name=str(self.get_parameter('move_group_name').value),
            end_effector_link=str(self.get_parameter('end_effector_link').value),
            planning_frame=str(self.get_parameter('planning_frame').value),
            velocity_scaling=float(
                self.get_parameter('max_velocity_scaling_factor').value
            ),
            acceleration_scaling=float(
                self.get_parameter('max_acceleration_scaling_factor').value
            ),
            allowed_planning_time=float(
                self.get_parameter('allowed_planning_time').value
            ),
            num_planning_attempts=int(
                self.get_parameter('num_planning_attempts').value
            ),
            planner_id=str(self.get_parameter('planner_id').value),
            pipeline_id=str(self.get_parameter('pipeline_id').value),
            callback_group=self._ompl_cbg,
        )

        if not self._ompl_client.available:
            self.get_logger().error(
                'move_group /move_action not available - run moveit_config_driver first'
            )

    # ------------------------------------------------------------------
    # Board-frame pose computation
    # ------------------------------------------------------------------

    def _board_frame_pose(
        self, x_base: float, y_base: float, z_board_m: float
    ) -> tuple[float, float, float]:
        """Transform a planner XY + board-frame Z into a full base_link position.

        The math (user's board-frame approach):
          1. Inverse-transform planner XY from base_link to board frame
             board_xy = R.T @ (base_xy - t)
          2. Re-build a point at the desired board-frame Z
             T_local = [board_xy[0], board_xy[1], z_board_m, 1]
          3. Multiply through board_to_base_tf
             T_base  = board_to_base_tf @ T_local

        This gives the exact position in base_link regardless of board tilt/position,
        with Z measured perpendicularly from the board surface — not from base_link Z-axis.

        Falls back to (x_base, y_base, z_board_m) when no board frame is available.
        """
        tf = self._board_to_base_tf
        if tf is None:
            return x_base, y_base, z_board_m

        R = tf[:3, :3]
        t = tf[:3, 3]

        # Project the planner XY onto the board plane (Z = 0 in board frame)
        base_pt = np.array([x_base, y_base, t[2]])   # put at board-origin height first
        board_pt = R.T @ (base_pt - t)               # inverse rotate + translate

        # Rebuild with the desired board-frame Z, then transform back to base_link
        local = np.array([board_pt[0], board_pt[1], z_board_m, 1.0])
        base  = tf @ local
        return float(base[0]), float(base[1]), float(base[2])

    # ------------------------------------------------------------------
    # IK → joint-space motion  (geometry → smooth deterministic moves)
    # ------------------------------------------------------------------

    def _compute_ik(
        self,
        x: float, y: float, z: float,
        seed_names: list | None = None,
        seed_positions: list | None = None,
    ) -> tuple[list, list] | None:
        """Call MoveIt /compute_ik and return (joint_names, joint_positions).

        When seed_names/seed_positions are provided (e.g. the taught a0 joint state),
        IK is solved starting from that configuration.  This ensures solutions stay in
        the same elbow-up/down region as the board-level work space — critical for
        avoiding wild arm reconfigurations between waypoints.

        Falls back to the live robot state as seed when no seed is given.
        Returns None in simulation or when the service fails.
        """
        if self._sim_mode or self._ik_client is None:
            return None

        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/compute_ik not available')
            return None

        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name        = str(self.get_parameter('move_group_name').value)
        ik_req.avoid_collisions  = True

        ps = PoseStamped()
        ps.header.frame_id      = str(self.get_parameter('planning_frame').value)
        ps.header.stamp         = self.get_clock().now().to_msg()
        ps.pose.position.x      = float(x)
        ps.pose.position.y      = float(y)
        ps.pose.position.z      = float(z)
        ps.pose.orientation     = yaw_to_downward_quaternion(self._scan_pose_yaw)
        ik_req.pose_stamped     = ps

        rs = MoveItRobotState()
        if seed_names and seed_positions:
            js = SensorJointState()
            js.name     = list(seed_names)
            js.position = [float(p) for p in seed_positions]
            rs.joint_state = js
            rs.is_diff     = False   # use provided seed, not live state
        else:
            rs.is_diff = True        # fallback: live state as seed
        ik_req.robot_state      = rs
        ik_req.timeout.sec      = 1
        ik_req.timeout.nanosec  = 0

        req.ik_request = ik_req

        future = self._ik_client.call_async(req)
        deadline = time.monotonic() + 4.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f'IK timed out for ({x:.3f},{y:.3f},{z:.3f})')
                return None
            time.sleep(0.05)

        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'IK service error: {exc}')
            return None

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f'IK no solution for ({x:.3f},{y:.3f},{z:.3f}): '
                f'error_code={resp.error_code.val}'
            )
            return None

        js = resp.solution.joint_state
        arm_names = self._initial_joint_names   # 6 UR5e joints from scan calibration
        if arm_names:
            pairs = [
                (n, p) for n, p in zip(js.name, js.position)
                if n in arm_names
            ]
            if len(pairs) >= len(arm_names):
                pairs.sort(key=lambda np_: arm_names.index(np_[0]))
                names, positions = zip(*pairs)
                return list(names), list(positions)

        # Fallback: return everything in the IK response
        return list(js.name), list(js.position)

    def _move_via_ik(
        self,
        x: float, y: float, z: float,
        seed_names: list | None = None,
        seed_positions: list | None = None,
    ) -> bool:
        """Move to pose via IK → joint-space (smooth, deterministic).

        seed_names/seed_positions: IK starting configuration (a0 taught state recommended).
        Falls back to OMPL if IK is unavailable or fails.
        """
        ik = self._compute_ik(x, y, z, seed_names, seed_positions)
        if ik is not None:
            names, positions = ik
            return self._move_joints(names, positions)
        self.get_logger().warn(
            f'IK failed for ({x:.3f},{y:.3f},{z:.3f}) — falling back to OMPL'
        )
        return self._move(x, y, z, self._scan_pose_yaw)

    def _graveyard_side_from_y(self, place_y: float) -> str:
        """Pick which graveyard ('red'/'black') a place_pose belongs to by nearest Y.

        Only called when the action goal already says place_is_graveyard=True, so this
        just disambiguates red vs black — no risk of misclassifying a board square.
        Prefers the taught calibration Y values; falls back to config-param Y.
        """
        cal = getattr(self, '_board_calibration', None)
        red_y   = getattr(cal, 'graveyard_red_y', None)   if cal else None
        black_y = getattr(cal, 'graveyard_black_y', None) if cal else None
        if red_y is None:
            red_y = self._graveyard_red_y
        if black_y is None:
            black_y = self._graveyard_black_y
        return 'red' if abs(place_y - float(red_y)) <= abs(place_y - float(black_y)) else 'black'

    def _graveyard_joints_for_zone(
        self, zone: str
    ) -> tuple[tuple[list, list] | None, tuple[list, list] | None]:
        """Taught (approach, grasp) joint configs for red or black graveyard."""
        cal = getattr(self, '_board_calibration', None)
        if cal is None:
            return None, None
        result = cal.get_graveyard_joints(zone == 'red')
        if result is None:
            return None, None
        return result[0], result[1]

    def _ik_seed_at_xy(
        self, x: float, y: float, for_approach: bool
    ) -> tuple[list, list] | None:
        """IK seed from 4 taught corners (bilinear), matching the target board XY.

        Uses the same 4-patch interpolation as joint-space cell moves (a0/i0/i9/a9),
        so IK along the i-file (i0→i9) stays in the correct elbow region.
        Falls back to the a0 corner seed when interpolation is unavailable.
        """
        if for_approach:
            fb_n, fb_p = self._a0_approach_seed_names, self._a0_approach_seed_positions
        else:
            fb_n, fb_p = self._a0_grasp_seed_names, self._a0_grasp_seed_positions

        cal = getattr(self, '_board_calibration', None)
        if cal is not None:
            interp = cal.ik_seed_joints(x, y, for_approach)
            if interp is not None:
                return interp

        if fb_n and fb_p:
            return list(fb_n), list(fb_p)
        return None

    def _move_board_approach_joints(
        self, x: float, y: float, pa_x: float, pa_y: float, pa_z: float
    ) -> bool:
        """Move to taught approach-height joints (4-corner bilinear) for board cell (x, y)."""
        seed = self._ik_seed_at_xy(x, y, for_approach=True)
        if seed:
            return self._move_joints(seed[0], seed[1])
        self.get_logger().warn(
            f'No taught approach joints at ({x:.3f},{y:.3f}) — IK fallback'
        )
        return self._move_via_ik(pa_x, pa_y, pa_z, None, None)

    def _move_grasp_descend(
        self,
        x: float, y: float, z: float,
        seed_names: list | None = None,
        seed_positions: list | None = None,
    ) -> bool:
        """Vertical descend at fixed XY via IK → joint-space.

        Cartesian (/par_moveit/waypoint_move) is deliberately NOT used: it fails on
        this hardware with CONTROL_FAILED and, worse, can report success without
        moving — so the arm "descends" by zero and grasps air. IK seeded from the
        cell's taught grasp joints keeps the descend in the correct elbow region.
        """
        return self._move_via_ik(x, y, z, seed_names, seed_positions)

    # ------------------------------------------------------------------
    # Action execution: hybrid pick-and-place
    #   Board cells: joint approach/lift (4-corner bilinear), IK descend.
    #   Graveyard:   taught graveyard joints when available, else IK.
    #   place_is_graveyard comes from the action goal (set by the planner) —
    #   no fragile geometric guessing of whether a square is a graveyard.
    #   NO return-to-scan here: the behaviour tree's GoToScanPose does the
    #   single scan move after BOTH capture and main move complete.
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result   = PickAndPlace.Result()

        pick_x  = req.pick_pose.position.x
        pick_y  = req.pick_pose.position.y
        place_x = req.place_pose.position.x
        place_y = req.place_pose.position.y

        approach_h = req.approach_height   # metres above board surface
        place_is_graveyard = bool(getattr(req, 'place_is_graveyard', False))

        if self._board_to_base_tf is None:
            self.get_logger().warn(
                'No board frame yet — call /xiangqi/move_to_scan_pose first'
            )

        def step(phase, fn):
            if goal_handle.is_cancel_requested:
                return False
            feedback.phase = phase
            goal_handle.publish_feedback(feedback)
            try:
                ok = fn()
                if not ok:
                    result.success = False
                    result.message = f'Failed at: {phase}'
                    goal_handle.abort()
                return ok
            except Exception as e:
                result.success = False
                result.message = f'Exception at {phase}: {e}'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return False

        # ── Board-frame poses (board_to_base_tf × local board-frame Z offset) ──
        # Approach: computed from board-relative height (same for all attempts).
        pa_x, pa_y, pa_z = self._board_frame_pose(pick_x,  pick_y,  approach_h)
        da_x, da_y, da_z = self._board_frame_pose(place_x, place_y, approach_h)
        # Grasp/place: use the planner-provided pose directly — it already encodes
        # board tilt compensation and any retry z offset from SetupMoveCoordinates.
        pg_x = req.pick_pose.position.x
        pg_y = req.pick_pose.position.y
        pg_z = req.pick_pose.position.z
        dg_x = req.place_pose.position.x
        dg_y = req.place_pose.position.y
        dg_z = req.place_pose.position.z

        # IK seeds at the pick board cell (4-corner bilinear taught joints)
        pick_ap_n, pick_ap_p = self._ik_seed_at_xy(pick_x, pick_y, True)  or (None, None)
        pick_gr_n, pick_gr_p = self._ik_seed_at_xy(pick_x, pick_y, False) or (None, None)

        # Place leg: graveyard (taught joints, all joint-space) vs board cell.
        gy_approach_joints = gy_grasp_joints = None
        place_gr_n = place_gr_p = None   # IK seed for the place-cell descend (board only)
        if place_is_graveyard:
            zone = self._graveyard_side_from_y(place_y)
            gy_approach_joints, gy_grasp_joints = self._graveyard_joints_for_zone(zone)
            self.get_logger().info(
                f'Pick→place: pick board ({pick_x:.3f},{pick_y:.3f}); '
                f'place {zone} GRAVEYARD ({place_x:.3f},{place_y:.3f}) '
                f'taught_approach={gy_approach_joints is not None} '
                f'taught_grasp={gy_grasp_joints is not None}'
            )
        else:
            place_gr_n, place_gr_p = self._ik_seed_at_xy(place_x, place_y, False) or (None, None)
            self.get_logger().info(
                f'Pick→place: pick ({pick_x:.3f},{pick_y:.3f}) '
                f'place ({place_x:.3f},{place_y:.3f}) board cells; '
                f'pick_grasp_z={pg_z:.4f} place_grasp_z={dg_z:.4f}'
            )

        # ── Pick leg (always a board cell) ─────────────────────────────
        def _approach_pick():
            return self._move_board_approach_joints(pick_x, pick_y, pa_x, pa_y, pa_z)

        def _descend_pick():
            return self._move_grasp_descend(pg_x, pg_y, pg_z, pick_gr_n, pick_gr_p)

        def _lift_pick():
            return self._move_board_approach_joints(pick_x, pick_y, pa_x, pa_y, pa_z)

        # ── Place leg (board cell or graveyard) ────────────────────────
        def _approach_place():
            if place_is_graveyard:
                if gy_approach_joints:
                    return self._move_joints(*gy_approach_joints)
                # No taught graveyard joints: IK to graveyard approach, seed from pick approach
                return self._move_via_ik(da_x, da_y, da_z, pick_ap_n, pick_ap_p)
            return self._move_board_approach_joints(place_x, place_y, da_x, da_y, da_z)

        def _descend_place():
            if place_is_graveyard:
                # Graveyard descend is pure joint-space (taught grasp joints) — no IK,
                # no Cartesian. Fall back to taught approach joints if grasp not taught.
                if gy_grasp_joints:
                    return self._move_joints(*gy_grasp_joints)
                if gy_approach_joints:
                    return self._move_joints(*gy_approach_joints)
                # Last resort only (no taught graveyard joints at all): IK descend.
                return self._move_via_ik(dg_x, dg_y, dg_z, pick_gr_n, pick_gr_p)
            return self._move_grasp_descend(dg_x, dg_y, dg_z, place_gr_n, place_gr_p)

        def _lift_place():
            if place_is_graveyard:
                if gy_approach_joints:
                    return self._move_joints(*gy_approach_joints)
                return self._move_via_ik(da_x, da_y, da_z, pick_ap_n, pick_ap_p)
            return self._move_board_approach_joints(place_x, place_y, da_x, da_y, da_z)

        if not step('opening_gripper',
                    lambda: self._gripper(self._open_width, OPEN_FORCE)):     return result
        if not step('approaching_pick',    _approach_pick):                   return result
        if not step('descending_to_piece', _descend_pick):                   return result
        if not step('grasping_piece',
                    lambda: self._gripper(self._grasp_width, self._grasp_force)):  return result

        # Grasp confirmation: if final_width is below threshold the fingers closed
        # through air — no piece was grasped. Re-open, descend lower, retry once.
        if (not self._sim_mode
                and self._last_gripper_width < self._grasp_check_min_width):
            self.get_logger().warn(
                f'Grasp check failed: final_width={self._last_gripper_width:.1f} mm '
                f'< {self._grasp_check_min_width:.1f} mm — re-descending '
                f'{self._grasp_retry_lower_m * 1000:.1f} mm lower'
            )
            pg_z_retry = pg_z - self._grasp_retry_lower_m

            def _regrasp_lower():
                if not self._gripper(self._open_width, OPEN_FORCE):
                    return False
                if not self._move_grasp_descend(pg_x, pg_y, pg_z_retry, pick_gr_n, pick_gr_p):
                    return False
                return self._gripper(self._grasp_width, self._grasp_force)

            if not step('regrasp_lower', _regrasp_lower):
                return result

        if not step('lifting',             _lift_pick):                       return result
        if not step('approaching_place',   _approach_place):                  return result
        if not step('descending_to_place', _descend_place):                  return result
        if not step('releasing_piece',
                    lambda: self._gripper(self._release_width, OPEN_FORCE)):  return result
        if not step('lifting_clear',       _lift_place):                      return result

        result.success = True
        result.placement_error_mm = 0.0
        result.message = 'Pick and place complete'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Scan pose helpers
    # ------------------------------------------------------------------

    def _apply_taught_poses_from_calibration(self) -> None:
        """Use scan_pose / initial_pose from calibration_tool Step 1 if present."""
        cal_file = resolve_manipulation_calibration_path(
            self.get_parameter('calibration_file').value,
            self.get_logger(),
        )
        if not os.path.isfile(cal_file):
            return
        try:
            cal = BoardCalibration.load(cal_file)
        except Exception as e:
            self.get_logger().warn(f'Could not load taught poses from {cal_file}: {e}')
            return

        if cal.initial_pose and all(k in cal.initial_pose for k in ('x', 'y', 'z', 'yaw')):
            p = cal.initial_pose
            self._initial_pose_x = float(p['x'])
            self._initial_pose_y = float(p['y'])
            self._initial_pose_z = float(p['z'])
            self._initial_pose_yaw = float(p['yaw'])
            self.get_logger().info(
                f'Initial pose from calibration: '
                f'({self._initial_pose_x:.3f}, {self._initial_pose_y:.3f}, '
                f'{self._initial_pose_z:.3f}), yaw={self._initial_pose_yaw:.3f}'
            )

        if cal.scan_pose and all(k in cal.scan_pose for k in ('x', 'y', 'z', 'yaw')):
            p = cal.scan_pose
            self._scan_pose_x = float(p['x'])
            self._scan_pose_y = float(p['y'])
            self._scan_pose_z = float(p['z'])
            self._scan_pose_yaw = float(p['yaw'])
            self._taught_scan_pose = True
            self.get_logger().info(
                f'Scan pose from calibration: '
                f'({self._scan_pose_x:.3f}, {self._scan_pose_y:.3f}, '
                f'{self._scan_pose_z:.3f}), yaw={self._scan_pose_yaw:.3f}'
            )

        if cal.scan_joint_positions and cal.scan_joint_names:
            self._initial_joint_positions = list(cal.scan_joint_positions)
            self._initial_joint_names = list(cal.scan_joint_names)
            self.get_logger().info(
                f'Joint-space homing from calibration: {len(self._initial_joint_positions)} joints'
            )

        self._board_calibration = cal

        # Extract a0 (corner 0) joint state as IK seed for board-level moves.
        # This is the existing taught configuration at file=0, rank=0 — no re-teaching needed.
        if (cal.calibration_corners_joints and cal.calibration_corners_joint_names
                and len(cal.calibration_corners_joints) >= 1):
            self._a0_grasp_seed_names     = list(cal.calibration_corners_joint_names)
            self._a0_grasp_seed_positions  = list(cal.calibration_corners_joints[0])
            self.get_logger().info(
                'a0 grasp seed loaded (IK fallback); board IK uses 4-corner bilinear seeds'
            )

        if (cal.cell_approach_joints and cal.cell_approach_joint_names
                and len(cal.cell_approach_joints) >= 1):
            self._a0_approach_seed_names     = list(cal.cell_approach_joint_names)
            self._a0_approach_seed_positions  = list(cal.cell_approach_joints[0])
            self.get_logger().info('a0 approach seed loaded (IK fallback)')

        n_corners = (
            len(cal.calibration_corners_joints)
            if cal.calibration_corners_joints else 0
        )
        self.get_logger().info(
            f'Calibration loaded: scan/home poses + IK seeds from {n_corners} taught corners '
            '(a0/i0/i9/a9 bilinear, 4-patch when midpoints present).'
        )

    def _resolve_scan_pose(self):
        """Return (x, y, z) for the scan pose in base_link (metres).

        When use_manual_scan_pose is true, returns scan_pose_x/y/z from params
        (absolute TCP position). Otherwise, if board_to_base_tf exists, x/y are
        derived from the board centre and scan_pose_z is height above the board surface.
        """
        z = float(self.get_parameter('scan_pose_z').value)
        fallback_x = float(self.get_parameter('scan_pose_x').value)
        fallback_y = float(self.get_parameter('scan_pose_y').value)

        if self.get_parameter('use_manual_scan_pose').value:
            self.get_logger().info(
                f'Using manual scan pose: ({fallback_x:.3f}, {fallback_y:.3f}, {z:.3f})'
            )
            return fallback_x, fallback_y, z

        cal_file = resolve_manipulation_calibration_path(
            self.get_parameter('calibration_file').value,
            self.get_logger(),
        )
        if not os.path.exists(cal_file):
            self.get_logger().info(
                f'No calibration file at {cal_file} - using manual scan_pose_x/y params'
            )
            return fallback_x, fallback_y, z

        try:
            cal = BoardCalibration.load(cal_file)
            if cal.board_to_base_tf is None:
                self.get_logger().warn(
                    'Calibration loaded but board_to_base_tf missing - using manual scan_pose params'
                )
                return fallback_x, fallback_y, z

            # Board centre: file 4 (middle of 0-8), rank 4.5 (middle of 0-9).
            # Average of rank-4 and rank-5 world positions at centre file.
            centre_lo = cal.grid_to_world(4, 4)
            centre_hi = cal.grid_to_world(4, 5)
            cx = float((centre_lo[0] + centre_hi[0]) / 2.0)
            cy = float((centre_lo[1] + centre_hi[1]) / 2.0)
            # z from cal is the board surface; add the configured height offset above it
            board_z = float((centre_lo[2] + centre_hi[2]) / 2.0)
            self.get_logger().info(
                f'Scan pose derived from calibration: '
                f'({cx:.3f}, {cy:.3f}, {board_z + z:.3f})'
            )
            return cx, cy, board_z + z

        except Exception as e:
            self.get_logger().warn(
                f'Failed to derive scan pose from calibration ({e}) - using manual params'
            )
            return fallback_x, fallback_y, z

    # ------------------------------------------------------------------
    # Rest / scan pose services
    # ------------------------------------------------------------------

    def _startup_move_to_initial_pose_cb(self) -> None:
        """One-shot homing when the stack starts (hardware only)."""
        if self._startup_move_done:
            return
        self._startup_move_done = True
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

        threading.Thread(
            target=self._startup_move_to_initial_pose_worker,
            name='xiangqi_startup_homing',
            daemon=True,
        ).start()

    def _startup_move_to_initial_pose_worker(self) -> None:
        if self._ompl_client is not None:
            wait_s = max(5.0, float(self.get_parameter('startup_move_delay_sec').value))
            if not self._ompl_client.wait_for_server(timeout_sec=wait_s):
                self.get_logger().warn(
                    f'Startup: {self.get_parameter("move_group_action").value} '
                    'not ready - start moveit_config_driver, then call '
                    '/xiangqi/move_to_initial_pose'
                )
                return

        max_retries = int(self.get_parameter('startup_move_max_retries').value)
        ok = False
        for attempt in range(1, max_retries + 1):
            if attempt > 1:
                self.get_logger().info(f'Startup: initial pose retry {attempt}/{max_retries}')
                time.sleep(2.0)
            if self._initial_joint_positions:
                self.get_logger().info('Startup: moving to initial pose (joint-space)')
                ok = self._move_joints(self._initial_joint_names, self._initial_joint_positions)
            else:
                self.get_logger().info(
                    f'Startup: moving to initial pose (OMPL) '
                    f'({self._initial_pose_x:.3f}, {self._initial_pose_y:.3f}, '
                    f'{self._initial_pose_z:.3f})'
                )
                ok = self._move(
                    self._initial_pose_x,
                    self._initial_pose_y,
                    self._initial_pose_z,
                    self._initial_pose_yaw,
                )
            if ok:
                break
        if ok:
            self.get_logger().info('Startup: reached scan pose - acquiring board frame via ArUco')
            time.sleep(1.0)  # let camera settle
            if not self._refresh_board_frame():
                self.get_logger().warn(
                    'Startup: board frame acquisition failed - moves will use planner Z. '
                    'Ensure vision_node + camera_config.yaml static TF are running, '
                    'then call /xiangqi/move_to_scan_pose to retry.'
                )
        else:
            self.get_logger().warn(
                'Startup: scan pose move failed - check arm_drivers, '
                'moveit_config_driver, pendant Play (External Control)'
            )

    def _move_to_initial_pose_cb(self, _request, response: Trigger.Response) -> Trigger.Response:
        """Move arm to configured rest / initial position (joint-space if calibrated)."""
        if self._initial_joint_positions:
            self.get_logger().info('Moving to initial pose (joint-space)')
            ok = self._move_joints(self._initial_joint_names, self._initial_joint_positions)
        else:
            self.get_logger().info(
                f'Moving to initial pose (OMPL) '
                f'({self._initial_pose_x:.3f}, {self._initial_pose_y:.3f}, {self._initial_pose_z:.3f})'
            )
            ok = self._move(
                self._initial_pose_x,
                self._initial_pose_y,
                self._initial_pose_z,
                self._initial_pose_yaw,
            )
        response.success = ok
        response.message = 'at initial pose' if ok else 'initial pose move failed'
        return response

    def _move_to_scan_pose_cb(self, _request, response: Trigger.Response) -> Trigger.Response:
        """Move arm to top-down scan position, then refresh board frame from ArUco."""
        if self._initial_joint_positions:
            self.get_logger().info('Moving to scan pose (joint-space)')
            ok = self._move_joints(self._initial_joint_names, self._initial_joint_positions)
        else:
            self.get_logger().info(
                f'Moving to scan pose (OMPL) '
                f'({self._scan_pose_x:.3f}, {self._scan_pose_y:.3f}, {self._scan_pose_z:.3f})'
            )
            ok = self._move(
                self._scan_pose_x,
                self._scan_pose_y,
                self._scan_pose_z,
                self._scan_pose_yaw,
            )
        if ok:
            time.sleep(0.5)
            board_ok = self._refresh_board_frame()
            response.success = True
            response.message = (
                'at scan pose; board frame acquired'
                if board_ok else
                'at scan pose; board frame acquisition failed (check vision_node + TF)'
            )
        else:
            response.success = False
            response.message = 'scan pose move failed'
        return response

    # ------------------------------------------------------------------
    # Arm motion: OMPL only (move_group /move_action)
    # ------------------------------------------------------------------

    def _move_joints(self, joint_names: list, joint_positions: list) -> bool:
        """Move to a specific joint configuration (deterministic - no IK ambiguity).

        Calls move_to_joints directly (uses spin_until_future_complete, same as _gripper).
        Safe to call from an executor thread OR from a daemon thread.
        """
        if self._sim_mode:
            self.get_logger().info(f'[SIM] Joint move → {joint_positions}')
            time.sleep(0.3)
            return True

        if self._ompl_client is None:
            self.get_logger().error('OMPL planner not initialized')
            return False

        with self._move_lock:
            try:
                for attempt in range(1, self._move_max_retries + 1):
                    if attempt > 1:
                        self.get_logger().info(
                            f'Joint move retry {attempt}/{self._move_max_retries}'
                        )
                        time.sleep(1.0)
                    if self._ompl_client.move_to_joints(joint_names, joint_positions):
                        return True
                self.get_logger().error(
                    f'Joint move failed after {self._move_max_retries} attempt(s)'
                )
            except Exception as exc:
                self.get_logger().error(f'Joint move exception: {exc}')
        return False

    def _move_cartesian(self, x: float, y: float, z: float, yaw: float) -> bool:
        """Straight-line Cartesian move via WaypointMove (descend/lift steps)."""
        if self._sim_mode:
            self.get_logger().info(f'[SIM] Cartesian → ({x:.3f}, {y:.3f}, {z:.3f})')
            time.sleep(0.3)
            return True

        if self._waypoint_client is None:
            self.get_logger().warn('WaypointMove not available - falling back to OMPL')
            return self._move(x, y, z, yaw)

        with self._move_lock:
            try:
                if not self._waypoint_client.wait_for_server(timeout_sec=10.0):
                    self.get_logger().error('/par_moveit/waypoint_move not ready')
                    return False

                goal = WaypointMove.Goal()
                goal.target_pose.position.x = float(x)
                goal.target_pose.position.y = float(y)
                goal.target_pose.position.z = float(z)
                goal.target_pose.rotation = float(yaw)

                self.get_logger().info(
                    f'Cartesian → ({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f}'
                )

                send_future = self._waypoint_client.send_goal_async(goal)
                # Use _wait_on_future (polling) instead of spin_until_future_complete.
                # spin_until_future_complete deadlocks when called from inside an action
                # server callback because the executor callback group is already active.
                goal_handle = _wait_on_future(send_future, timeout_sec=15.0)
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().error('WaypointMove goal rejected or timed out')
                    return False

                result_future = goal_handle.get_result_async()
                wrapped = _wait_on_future(result_future, timeout_sec=120.0)
                if wrapped is None:
                    self.get_logger().error('WaypointMove timed out waiting for result')
                    return False

                return True
            except Exception as exc:
                self.get_logger().error(f'Cartesian move exception: {exc}')
        return False

    def _move(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        """Plan and execute in joint space (OMPL) to (x,y,z) + downward yaw."""
        if self._sim_mode:
            self.get_logger().info(
                f'[SIM] Move → ({x:.3f}, {y:.3f}, {z:.3f})'
            )
            time.sleep(0.3)
            return True

        if self._ompl_client is None:
            self.get_logger().error('OMPL planner not initialized')
            return False

        with self._move_lock:
            try:
                for attempt in range(1, self._move_max_retries + 1):
                    if attempt > 1:
                        self.get_logger().info(
                            f'OMPL move retry {attempt}/{self._move_max_retries}'
                        )
                        time.sleep(1.0)
                    if self._ompl_client.move_to_pose(x, y, z, yaw):
                        return True
                self.get_logger().error(
                    f'OMPL move failed after {self._move_max_retries} attempt(s) '
                    f'→ ({x:.3f}, {y:.3f}, {z:.3f})'
                )
            except Exception as exc:
                self.get_logger().error(f'OMPL move exception: {exc}')
        return False

    # ------------------------------------------------------------------
    # Gripper primitive  →  /rg2/set_width
    # ------------------------------------------------------------------

    def _gripper(self, target_width: float, target_force: float) -> bool:
        """Set RG2 gripper opening width (mm) with given force (N)."""
        if self._sim_mode or self._rg2_client is None:
            self.get_logger().info(
                f'[{"SIM" if self._sim_mode else "NORG2"}] '
                f'Gripper → {target_width:.1f} mm @ {target_force:.1f} N'
            )
            time.sleep(0.2)
            self._last_gripper_width = target_width
            return True

        if not self._rg2_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('/rg2/set_width not available')
            return False

        goal = GripperSetWidth.Goal()
        goal.target_width = target_width
        goal.target_force = target_force

        send_future = self._rg2_client.send_goal_async(goal)
        goal_handle = _wait_on_future(send_future, timeout_sec=5.0)

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('GripperSetWidth goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        wrapped = _wait_on_future(result_future, timeout_sec=10.0)

        if wrapped is None:
            self.get_logger().error('GripperSetWidth timed out')
            return False

        self._last_gripper_width = wrapped.result.final_width
        self.get_logger().info(
            f'RG2 at {self._last_gripper_width:.1f} mm'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
