"""
gripper_controller_node: ROS 2 bridge for the OnRobot RG2 two-finger gripper.

The lab's UR5e is fitted with an OnRobot RG2 gripper (max width 110 mm,
max force 40 N). The existing onrobot_rg2_driver package (from UR5e_Env)
exposes the gripper via:

  Action servers (provided by gripper_control_node from onrobot_rg2_driver):
    /rg2/set_width   (onrobot_rg2_msgs/action/GripperSetWidth)
    /rg2/full_open   (onrobot_rg2_msgs/action/GripperFullOpen)
    /rg2/full_close  (onrobot_rg2_msgs/action/GripperFullClose)

  Topics (published by gripper_state_publisher_node):
    /rg2/state       (onrobot_rg2_msgs/msg/GripperState)  -- width, depth, busy

This node provides the xiangqi_msgs/srv/GripperControl service as a
thin synchronous bridge so the rest of the xiangqi stack does not need
to import onrobot_rg2_msgs directly.

Gripper geometry for round Xiangqi pieces (~20 mm diameter):
  OPEN_WIDTH    = 50 mm   - finger clearance to lower around a piece
  GRASP_WIDTH   = 18 mm   - firm grip on ~20 mm piece
  RELEASE_WIDTH = 34 mm   - enough clearance to lift off a released piece
  GRASP_FORCE   = 15 N    - firm but gentle (pieces are plastic/wood)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool
from xiangqi_msgs.srv import GripperControl

try:
    from onrobot_rg2_msgs.action import GripperSetWidth, GripperFullOpen
    from onrobot_rg2_msgs.msg import GripperState
    RG2_OK = True
except ImportError:
    RG2_OK = False


# Default gripper widths (mm) — tune after physical testing
OPEN_WIDTH    = 50.0   # Opening width to clear piece before descent
GRASP_WIDTH   = 18.0   # Closing width to grip a ~20 mm Xiangqi piece
RELEASE_WIDTH = 34.0   # Width to open when releasing piece at destination
DEFAULT_FORCE = 15.0   # Gripping force in Newtons


class GripperControllerNode(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')

        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('gripper_ip',   '10.234.6.47')
        self.declare_parameter('gripper_port', 502)
        self.declare_parameter('open_width',    OPEN_WIDTH)
        self.declare_parameter('grasp_width',   GRASP_WIDTH)
        self.declare_parameter('release_width', RELEASE_WIDTH)
        self.declare_parameter('default_force', DEFAULT_FORCE)

        self._sim_mode     = self.get_parameter('simulation_mode').value
        self._open_width   = self.get_parameter('open_width').value
        self._grasp_width  = self.get_parameter('grasp_width').value
        self._release_width = self.get_parameter('release_width').value
        self._default_force = self.get_parameter('default_force').value

        self._current_width: float = self._open_width

        cb_group = ReentrantCallbackGroup()

        # Service we expose to the rest of the xiangqi stack
        self._gripper_srv = self.create_service(
            GripperControl, '/xiangqi/gripper_control', self._gripper_cb,
            callback_group=cb_group
        )

        # Bool publisher consumed by the dashboard:
        # True = gripper is closed/gripping (width < open threshold)
        self._active_pub = self.create_publisher(Bool, '/xiangqi/gripper_active', 10)

        # Action client → lab's onrobot_rg2_driver gripper_control_node
        self._rg2_client = None
        if not self._sim_mode and RG2_OK:
            self._rg2_client = ActionClient(
                self, GripperSetWidth, '/rg2/set_width',
                callback_group=cb_group
            )

        # Subscribe to gripper state for current-width tracking
        if not self._sim_mode and RG2_OK:
            self.create_subscription(
                GripperState, '/rg2/state',
                self._state_cb, 10
            )

        self.get_logger().info(
            f'gripper_controller_node started '
            f'(sim={self._sim_mode}, rg2_ok={RG2_OK})'
        )

    # ------------------------------------------------------------------
    # State subscriber
    # ------------------------------------------------------------------

    def _state_cb(self, msg):
        self._current_width = msg.width
        # Publish Bool for dashboard: True = gripper is actively gripping
        active = Bool()
        active.data = self._current_width < (self._open_width - 5.0)
        self._active_pub.publish(active)

    # ------------------------------------------------------------------
    # Service callback (synchronous bridge)
    # ------------------------------------------------------------------

    def _gripper_cb(self, request: GripperControl.Request,
                    response: GripperControl.Response):
        target_width = float(request.target_width)
        target_force = float(request.target_force) if request.target_force > 0 else self._default_force

        try:
            self._set_width(target_width, target_force)
            response.success = True
            response.final_width = self._current_width
            response.message = f'Gripper set to {target_width:.1f} mm'
        except Exception as e:
            response.success = False
            response.final_width = self._current_width
            response.message = str(e)
            self.get_logger().error(f'Gripper control error: {e}')
        return response

    # ------------------------------------------------------------------
    # Gripper actuation
    # ------------------------------------------------------------------

    def _set_width(self, target_width: float, target_force: float) -> None:
        if self._sim_mode:
            self.get_logger().info(
                f'[SIM] Gripper → {target_width:.1f} mm @ {target_force:.1f} N'
            )
            self._current_width = target_width
            return

        if self._rg2_client is None or not RG2_OK:
            self.get_logger().warn(
                'RG2 driver not available — gripper command ignored'
            )
            return

        if not self._rg2_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('/rg2/set_width action server not available')

        goal = GripperSetWidth.Goal()
        goal.target_width = target_width
        goal.target_force = target_force

        future = self._rg2_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('RG2 set_width goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result()
        if result is None:
            raise RuntimeError('RG2 set_width timed out')

        self._current_width = result.result.final_width
        self.get_logger().info(
            f'RG2 gripper at {self._current_width:.1f} mm'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
