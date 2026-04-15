from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class ThrusterAllocator2D(Node):
    """
    Maps planar command (/control/cmd_vel) to 8-thruster PWM command (/auv/thruster_cmd).

    Thruster order matches Arduino `processMessage()`:
      0: thrusterpsfr
      1: thrustersbfr
      2: thrusterpsaf
      3: thrustersbaf
      4: thrusterpsms1
      5: thrustersbms1
      6: thrusterpsms2
      7: thrustersbms2

    Horizontal mixer is inferred from your RF code (X-drive style):
      surge pattern: [+ + + +]
      sway  pattern: [+ - - +]
      yaw   pattern: [+ - + -]
    """

    def __init__(self) -> None:
        super().__init__("thruster_allocator_2d")

        self.declare_parameter("cmd_topic", "/control/cmd_vel")
        self.declare_parameter("thruster_topic", "/auv/thruster_cmd")

        self.declare_parameter("pwm_neutral", 1500)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_max", 1900)

        # Gains from cmd_vel components to PWM delta.
        self.declare_parameter("k_surge_pwm", 300.0)
        self.declare_parameter("k_sway_pwm", 300.0)
        self.declare_parameter("k_yaw_pwm", 220.0)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)

        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        thruster_topic = str(self.get_parameter("thruster_topic").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1e-3, rate_hz)

        self._sub = self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self._pub = self.create_publisher(Int32MultiArray, thruster_topic, 10)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"thruster_allocator_2d started | cmd={cmd_topic} thrusters={thruster_topic}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _neutral_command(self) -> List[int]:
        neutral = int(self.get_parameter("pwm_neutral").value)
        return [neutral] * 8

    def _on_timer(self) -> None:
        timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > timeout_sec:
            self._publish(self._neutral_command())
            return

        neutral = int(self.get_parameter("pwm_neutral").value)
        pwm_min = int(self.get_parameter("pwm_min").value)
        pwm_max = int(self.get_parameter("pwm_max").value)

        k_surge = float(self.get_parameter("k_surge_pwm").value)
        k_sway = float(self.get_parameter("k_sway_pwm").value)
        k_yaw = float(self.get_parameter("k_yaw_pwm").value)

        du = k_surge * float(self._last_cmd.linear.x)
        dv = k_sway * float(self._last_cmd.linear.y)
        dr = k_yaw * float(self._last_cmd.angular.z)

        # Horizontal thrusters: [psfr, sbfr, psaf, sbaf]
        t0 = int(round(neutral + du + dv + dr))  # psfr
        t1 = int(round(neutral + du - dv - dr))  # sbfr
        t2 = int(round(neutral + du - dv + dr))  # psaf
        t3 = int(round(neutral + du + dv - dr))  # sbaf

        # Vertical thrusters untouched in 2D mode.
        t4 = neutral
        t5 = neutral
        t6 = neutral
        t7 = neutral

        cmd = [
            clamp(t0, pwm_min, pwm_max),
            clamp(t1, pwm_min, pwm_max),
            clamp(t2, pwm_min, pwm_max),
            clamp(t3, pwm_min, pwm_max),
            clamp(t4, pwm_min, pwm_max),
            clamp(t5, pwm_min, pwm_max),
            clamp(t6, pwm_min, pwm_max),
            clamp(t7, pwm_min, pwm_max),
        ]
        self._publish(cmd)

    def _publish(self, values: List[int]) -> None:
        m = Int32MultiArray()
        m.data = values
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterAllocator2D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

