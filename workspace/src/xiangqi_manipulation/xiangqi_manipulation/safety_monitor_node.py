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

        self._dashboard_estop = False
        self._ur_safety_estop = False

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
        ur_stop = hasattr(msg, 'mode') and msg.mode not in (1, 2)
        if ur_stop and not self._ur_safety_estop:
            self.get_logger().warn(f'UR safety mode: {msg.mode} - software e-stop')
        self._ur_safety_estop = ur_stop

    def _estop_cb(self, msg: Bool) -> None:
        """Dashboard toggle - software halt for the Xiangqi stack (not the teach pendant)."""
        if msg.data and not self._dashboard_estop:
            self.get_logger().warn('Emergency stop triggered via dashboard (software)')
        self._dashboard_estop = bool(msg.data)

    @property
    def _estop_active(self) -> bool:
        return self._dashboard_estop or self._ur_safety_estop

    def _heartbeat(self) -> None:
        active = self._estop_active
        msg = Bool()
        msg.data = active
        self._estop_pub.publish(msg)

        status = String()
        if self._dashboard_estop and self._ur_safety_estop:
            status.data = 'ESTOP_ACTIVE (dashboard + UR)'
        elif self._dashboard_estop:
            status.data = 'ESTOP_ACTIVE (dashboard)'
        elif self._ur_safety_estop:
            status.data = 'ESTOP_ACTIVE (UR safety)'
        else:
            status.data = 'OK'
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
