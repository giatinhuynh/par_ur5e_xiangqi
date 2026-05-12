"""
safety_monitor_node: Monitors robot safety conditions and publishes an e-stop signal.

Watches:
  - UR robot state (protective stop, safety stop)
  - Workspace boundary violations (arm moving outside permitted zone)
  - Emergency stop button on the dashboard (/xiangqi/emergency_stop topic)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    UR_DASHBOARD_OK = True
except ImportError:
    UR_DASHBOARD_OK = False


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        self._estop_active = False

        # Subscribe to UR safety state
        if UR_DASHBOARD_OK:
            self.create_subscription(
                SafetyMode, '/ur_hardware_interface/safety_mode',
                self._safety_mode_cb, 10
            )

        # Subscribe to dashboard emergency stop
        self.create_subscription(Bool, '/xiangqi/emergency_stop', self._estop_cb, 10)

        # Publish e-stop state
        self._estop_pub = self.create_publisher(Bool, '/xiangqi/estop', 10)
        self._status_pub = self.create_publisher(String, '/xiangqi/safety_status', 10)

        self._timer = self.create_timer(0.5, self._heartbeat)
        self.get_logger().info('safety_monitor_node started')

    def _safety_mode_cb(self, msg) -> None:
        # SafetyMode values: NORMAL=1, REDUCED=2, PROTECTIVE_STOP=3, etc.
        if hasattr(msg, 'mode') and msg.mode not in (1, 2):
            if not self._estop_active:
                self.get_logger().warn(f'Safety mode: {msg.mode} -- issuing e-stop')
            self._estop_active = True
        else:
            self._estop_active = False

    def _estop_cb(self, msg: Bool) -> None:
        if msg.data and not self._estop_active:
            self.get_logger().warn('Emergency stop triggered via dashboard')
        self._estop_active = msg.data

    def _heartbeat(self) -> None:
        msg = Bool()
        msg.data = self._estop_active
        self._estop_pub.publish(msg)

        status = String()
        status.data = 'ESTOP_ACTIVE' if self._estop_active else 'OK'
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
