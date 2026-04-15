from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class ThrusterAllocator3D(Node):
    """Simple 8-thruster mixer for 3D command tracking."""

    def __init__(self) -> None:
        super().__init__("thruster_allocator_3d")
        self.declare_parameter("cmd_topic", "/control/cmd_vel_3d")
        self.declare_parameter("thruster_topic", "/auv/thruster_cmd")
        self.declare_parameter("pwm_neutral", 1500)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_max", 1900)

        self.declare_parameter("k_surge_pwm", 280.0)
        self.declare_parameter("k_sway_pwm", 260.0)
        self.declare_parameter("k_heave_pwm", 300.0)
        self.declare_parameter("k_roll_pwm", 180.0)
        self.declare_parameter("k_pitch_pwm", 220.0)
        self.declare_parameter("k_yaw_pwm", 220.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)

        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        thruster_topic = str(self.get_parameter("thruster_topic").value)
        period = 1.0 / max(1e-3, float(self.get_parameter("publish_rate_hz").value))

        self._sub = self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self._pub = self.create_publisher(Int32MultiArray, thruster_topic, 10)
        self._timer = self.create_timer(period, self._on_timer)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _neutral(self) -> List[int]:
        return [int(self.get_parameter("pwm_neutral").value)] * 8

    def _on_timer(self) -> None:
        timeout = float(self.get_parameter("command_timeout_sec").value)
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > timeout:
            self._publish(self._neutral())
            return

        n = int(self.get_parameter("pwm_neutral").value)
        lo = int(self.get_parameter("pwm_min").value)
        hi = int(self.get_parameter("pwm_max").value)

        du = float(self.get_parameter("k_surge_pwm").value) * float(self._last_cmd.linear.x)
        dv = float(self.get_parameter("k_sway_pwm").value) * float(self._last_cmd.linear.y)
        dw = float(self.get_parameter("k_heave_pwm").value) * float(self._last_cmd.linear.z)
        dp = float(self.get_parameter("k_roll_pwm").value) * float(self._last_cmd.angular.x)
        dq = float(self.get_parameter("k_pitch_pwm").value) * float(self._last_cmd.angular.y)
        dr = float(self.get_parameter("k_yaw_pwm").value) * float(self._last_cmd.angular.z)

        # Horizontal set (0..3): surge/sway/yaw.
        t0 = int(round(n + du + dv + dr))
        t1 = int(round(n + du - dv - dr))
        t2 = int(round(n + du - dv + dr))
        t3 = int(round(n + du + dv - dr))

        # Vertical set (4..7): heave + roll/pitch moments.
        t4 = int(round(n + dw + dp + dq))
        t5 = int(round(n + dw - dp + dq))
        t6 = int(round(n + dw + dp - dq))
        t7 = int(round(n + dw - dp - dq))

        self._publish([clamp(v, lo, hi) for v in (t0, t1, t2, t3, t4, t5, t6, t7)])

    def _publish(self, values: List[int]) -> None:
        m = Int32MultiArray()
        m.data = values
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterAllocator3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

