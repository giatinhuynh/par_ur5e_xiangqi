"""
gripper_controller_node: Controls the vacuum suction gripper via UR digital I/O.

The VXLab vacuum gripper is wired to the UR5e tool I/O. It is toggled by
setting digital output 0 (DO0) on the UR controller. The UR ROS 2 driver
exposes this via the /io_and_status_controller/set_io service.

For the EyeBox interface (IP 10.234.6.47), an HTTP or TCP command can be
used as an alternative -- configure with the 'use_eyebox' parameter.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool
from xiangqi_msgs.srv import GripperControl

try:
    from ur_msgs.srv import SetIO
    UR_MSGS_OK = True
except ImportError:
    UR_MSGS_OK = False


GRIPPER_IO_PIN = 0   # Digital output pin on UR5e tool I/O


class GripperControllerNode(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')

        self.declare_parameter('use_eyebox', False)
        self.declare_parameter('eyebox_ip', '10.234.6.47')
        self.declare_parameter('eyebox_port', 80)
        self.declare_parameter('simulation_mode', False)

        self._sim_mode = self.get_parameter('simulation_mode').value
        self._use_eyebox = self.get_parameter('use_eyebox').value
        self._is_active = False

        cb_group = ReentrantCallbackGroup()

        # Service we provide to other nodes
        self._gripper_srv = self.create_service(
            GripperControl, 'gripper_control', self._gripper_cb,
            callback_group=cb_group
        )

        # State publisher for dashboard
        self._state_pub = self.create_publisher(Bool, '/xiangqi/gripper_active', 10)

        # UR IO service client (if using direct UR driver)
        self._ur_io_cli = None
        if not self._sim_mode and not self._use_eyebox and UR_MSGS_OK:
            self._ur_io_cli = self.create_client(
                SetIO, '/io_and_status_controller/set_io',
                callback_group=cb_group
            )

        self.get_logger().info(
            f'gripper_controller_node started '
            f'(sim={self._sim_mode}, eyebox={self._use_eyebox})'
        )

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def _gripper_cb(self, request: GripperControl.Request, response: GripperControl.Response):
        try:
            self._set_gripper(request.activate)
            response.success = True
            response.is_active = self._is_active
            response.message = 'Gripper activated' if request.activate else 'Gripper released'
        except Exception as e:
            response.success = False
            response.is_active = self._is_active
            response.message = str(e)
            self.get_logger().error(f'Gripper control error: {e}')
        return response

    # ------------------------------------------------------------------
    # Gripper actuation
    # ------------------------------------------------------------------

    def _set_gripper(self, activate: bool) -> None:
        if self._sim_mode:
            self.get_logger().info(f'[SIM] Gripper {"ON" if activate else "OFF"}')
            self._is_active = activate
            self._publish_state()
            return

        if self._use_eyebox:
            self._set_via_eyebox(activate)
        else:
            self._set_via_ur_io(activate)

        self._is_active = activate
        self._publish_state()

    def _set_via_ur_io(self, activate: bool) -> None:
        """Toggle vacuum via UR digital output pin."""
        if self._ur_io_cli is None or not UR_MSGS_OK:
            self.get_logger().warn('UR IO service not available -- gripper command ignored')
            return

        if not self._ur_io_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('UR IO service not available')

        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = GRIPPER_IO_PIN
        req.state = 1.0 if activate else 0.0

        future = self._ur_io_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            raise RuntimeError('UR IO service call timed out')

    def _set_via_eyebox(self, activate: bool) -> None:
        """Toggle vacuum via EyeBox HTTP API (adjust endpoint per lab configuration)."""
        import urllib.request
        ip = self.get_parameter('eyebox_ip').value
        port = self.get_parameter('eyebox_port').value
        cmd = 'on' if activate else 'off'
        url = f'http://{ip}:{port}/gripper/{cmd}'
        try:
            with urllib.request.urlopen(url, timeout=2) as resp:
                self.get_logger().debug(f'EyeBox response: {resp.read()}')
        except Exception as e:
            self.get_logger().warn(f'EyeBox request failed: {e} -- continuing')

    def _publish_state(self) -> None:
        msg = Bool()
        msg.data = self._is_active
        self._state_pub.publish(msg)


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
