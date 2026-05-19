"""
manipulation_node: Pick-and-place action server for Xiangqi pieces.

The lab's UR5e uses a custom C++ MoveIt action server (from par_moveit_config)
that exposes arm motion via:

    /par_moveit/waypoint_move  (par_interfaces/action/WaypointMove)
        goal:   par_interfaces/WaypointPose target_pose
                    geometry_msgs/Point position   (x, y, z in metres, robot base frame)
                    float64 rotation               (end-effector yaw in radians)
        result: par_interfaces/WaypointPose final_pose

The RG2 gripper is controlled via (from onrobot_rg2_driver):

    /rg2/set_width  (onrobot_rg2_msgs/action/GripperSetWidth)
        goal:   float32 target_width   (mm)
                float32 target_force   (N)
        result: float32 final_width    (mm)

This node accepts xiangqi_msgs/action/PickAndPlace goals (in robot base frame
metres) and executes the full 8-step pick-and-place sequence.

Coordinate conventions:
    - pick_pose / place_pose are geometry_msgs/Pose in the robot base_link frame.
    - Only position (x, y, z) is used; orientation is ignored because Xiangqi
      pieces are round and the gripper is always pointing straight down.
    - approach_height and transit_height are z offsets added on top of
      pick/place z values.

Gripper widths (all in mm):
    OPEN_WIDTH    = 50   Pre-grasp open; fingers clear around a ~20 mm piece
    GRASP_WIDTH   = 18   Closed on piece (~20 mm diameter, 2 mm compression)
    RELEASE_WIDTH = 50   Open enough to lift off a released piece
    GRASP_FORCE   = 15 N Firm but gentle
"""

from __future__ import annotations
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from xiangqi_msgs.action import PickAndPlace
from xiangqi_manipulation.move_translator import BoardCalibration
from xiangqi_manipulation.calibration_paths import resolve_manipulation_calibration_path

try:
    from par_interfaces.action import WaypointMove
    from par_interfaces.msg import WaypointPose
    PAR_INTERFACES_OK = True
except ImportError:
    PAR_INTERFACES_OK = False

try:
    from onrobot_rg2_msgs.action import GripperSetWidth
    RG2_OK = True
except ImportError:
    RG2_OK = False


# Gripper widths in millimetres
OPEN_WIDTH    = 50.0   # Clearance width before descending onto piece
GRASP_WIDTH   = 18.0   # Grip width for ~20 mm diameter Xiangqi piece
RELEASE_WIDTH = 34.0   # Width after releasing piece at destination
GRASP_FORCE   = 15.0   # Newtons — firm grip without crushing
OPEN_FORCE    = 10.0   # Newtons — gentle open


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

        self._sim_mode      = self.get_parameter('simulation_mode').value
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

        self._taught_scan_pose = False
        self._apply_taught_poses_from_calibration()

        if not self._taught_scan_pose:
            # Compute scan pose x/y from board calibration (board centre in base_link).
            self._scan_pose_x, self._scan_pose_y, self._scan_pose_z = \
                self._resolve_scan_pose()

        # Separate callback groups to avoid ROS action deadlocks
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        self._arm_cbg    = MutuallyExclusiveCallbackGroup()
        self._grip_cbg   = MutuallyExclusiveCallbackGroup()

        # --- Action clients for lab infrastructure ---
        self._arm_client  = None
        self._rg2_client  = None

        if not self._sim_mode:
            if PAR_INTERFACES_OK:
                self._arm_client = ActionClient(
                    self, WaypointMove, '/par_moveit/waypoint_move',
                    callback_group=self._arm_cbg
                )
            else:
                self.get_logger().warn(
                    'par_interfaces not found — arm motion will be simulated'
                )
            if RG2_OK:
                self._rg2_client = ActionClient(
                    self, GripperSetWidth, '/rg2/set_width',
                    callback_group=self._grip_cbg
                )
            else:
                self.get_logger().warn(
                    'onrobot_rg2_msgs not found — gripper motion will be simulated'
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

        self.get_logger().info(
            f'manipulation_node ready '
            f'(sim={self._sim_mode}, arm={PAR_INTERFACES_OK}, rg2={RG2_OK})'
        )

        if (
            not self._sim_mode
            and self.get_parameter('move_to_initial_pose_on_startup').value
        ):
            delay = float(self.get_parameter('startup_move_delay_sec').value)
            self.get_logger().info(
                f'Startup: will move to initial pose in {delay:.1f}s '
                '(pendant Play + External Control must be ON)'
            )
            self._startup_timer = self.create_timer(
                delay,
                self._startup_move_to_initial_pose_cb,
                callback_group=self._arm_cbg,
            )

    # ------------------------------------------------------------------
    # Action execution: 8-step pick-and-place sequence
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result   = PickAndPlace.Result()

        pick_x = req.pick_pose.position.x
        pick_y = req.pick_pose.position.y
        pick_z = req.pick_pose.position.z

        place_x = req.place_pose.position.x
        place_y = req.place_pose.position.y
        place_z = req.place_pose.position.z

        approach_h = req.approach_height
        transit_h  = req.transit_height

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

        # ---- Sequence -----------------------------------------------
        # 1. Open gripper wide (pre-grasp clearance)
        if not step('opening_gripper',
                    lambda: self._gripper(self._open_width, OPEN_FORCE)):
            return result

        # 2. Move to approach height above pick position
        if not step('approaching_pick',
                    lambda: self._move(pick_x, pick_y, pick_z + approach_h)):
            return result

        # 3. Descend to piece
        if not step('descending_to_piece',
                    lambda: self._move(pick_x, pick_y, pick_z)):
            return result

        # 4. Close gripper to grip piece
        if not step('grasping_piece',
                    lambda: self._gripper(self._grasp_width, self._grasp_force)):
            return result

        # 5. Lift to transit height
        if not step('lifting',
                    lambda: self._move(pick_x, pick_y, pick_z + transit_h)):
            return result

        # 6. Move laterally to destination (at transit height)
        if not step('transiting',
                    lambda: self._move(place_x, place_y, place_z + transit_h)):
            return result

        # 7. Descend to place position
        if not step('descending_to_place',
                    lambda: self._move(place_x, place_y, place_z)):
            return result

        # 8. Release piece
        if not step('releasing_piece',
                    lambda: self._gripper(self._release_width, OPEN_FORCE)):
            return result

        # 9. Lift clear of placed piece
        if not step('lifting_clear',
                    lambda: self._move(place_x, place_y, place_z + approach_h)):
            return result

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
                f'No calibration file at {cal_file} — using manual scan_pose_x/y params'
            )
            return fallback_x, fallback_y, z

        try:
            cal = BoardCalibration.load(cal_file)
            if cal.board_to_base_tf is None:
                self.get_logger().warn(
                    'Calibration loaded but board_to_base_tf missing — using manual scan_pose params'
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
                f'Failed to derive scan pose from calibration ({e}) — using manual params'
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

        self.get_logger().info(
            f'Startup: moving to initial pose '
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
            self.get_logger().info('Startup: reached initial pose')
        else:
            self.get_logger().warn(
                'Startup: initial pose move failed — check pendant Play and MoveIt'
            )

    def _move_to_initial_pose_cb(self, _request, response: Trigger.Response) -> Trigger.Response:
        """Move arm to configured rest / initial position."""
        self.get_logger().info(
            f'Moving to initial pose '
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
        """Move arm to configured top-down scan position for board vision."""
        self.get_logger().info(
            f'Moving to scan pose '
            f'({self._scan_pose_x:.3f}, {self._scan_pose_y:.3f}, {self._scan_pose_z:.3f})'
        )
        ok = self._move(
            self._scan_pose_x,
            self._scan_pose_y,
            self._scan_pose_z,
            self._scan_pose_yaw,
        )
        response.success = ok
        response.message = 'at scan pose' if ok else 'scan pose move failed'
        return response

    # ------------------------------------------------------------------
    # Arm motion primitive  →  /par_moveit/waypoint_move
    # ------------------------------------------------------------------

    def _move(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        """Move end-effector to (x, y, z) in robot base frame (metres)."""
        if self._sim_mode or self._arm_client is None:
            self.get_logger().info(
                f'[{"SIM" if self._sim_mode else "NOARM"}] '
                f'Move → ({x:.3f}, {y:.3f}, {z:.3f})'
            )
            time.sleep(0.3)
            return True

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/par_moveit/waypoint_move not available')
            return False

        wp = WaypointPose()
        wp.position.x = x
        wp.position.y = y
        wp.position.z = z
        wp.rotation = yaw    # End-effector yaw; 0.0 works for all Xiangqi pieces

        goal = WaypointMove.Goal()
        goal.target_pose = wp

        send_future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('WaypointMove goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('WaypointMove timed out')
            return False

        return True

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
            return True

        if not self._rg2_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('/rg2/set_width not available')
            return False

        goal = GripperSetWidth.Goal()
        goal.target_width = target_width
        goal.target_force = target_force

        send_future = self._rg2_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('GripperSetWidth goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('GripperSetWidth timed out')
            return False

        self.get_logger().info(
            f'RG2 at {result.result.final_width:.1f} mm'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    executor = MultiThreadedExecutor()
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
