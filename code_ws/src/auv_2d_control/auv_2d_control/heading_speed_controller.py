import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingSpeedController(Node):
    """
    Tracks guidance reference:
      /guidance/reference (Float64MultiArray): [u_d, v_d, psi_d]

    and publishes:
      /control/cmd_vel (Twist): linear.x (surge cmd), linear.y (sway cmd), angular.z (yaw-rate cmd)
    """

    def __init__(self) -> None:
        super().__init__("heading_speed_controller")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference")
        self.declare_parameter("cmd_topic", "/control/cmd_vel")

        self.declare_parameter("kp_yaw", 1.5)
        self.declare_parameter("kd_yaw", 0.25)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kp_u", 1.0)
        self.declare_parameter("ki_u", 0.0)
        self.declare_parameter("kd_u", 0.0)
        self.declare_parameter("kp_v", 1.0)
        self.declare_parameter("ki_v", 0.0)
        self.declare_parameter("kd_v", 0.0)
        self.declare_parameter("max_yaw_rate", 0.8)
        self.declare_parameter("max_surge_speed", 0.4)
        self.declare_parameter("max_sway_speed", 0.3)
        self.declare_parameter("yaw_integral_limit", 0.8)
        self.declare_parameter("u_integral_limit", 0.5)
        self.declare_parameter("v_integral_limit", 0.5)

        self._psi: Optional[float] = None
        self._u: float = 0.0
        self._v: float = 0.0
        self._r: float = 0.0
        self._u_d: float = 0.0
        self._v_d: float = 0.0
        self._psi_d: Optional[float] = None
        self._int_yaw: float = 0.0
        self._int_u: float = 0.0
        self._int_v: float = 0.0
        self._prev_heading_error: float = 0.0
        self._prev_u_error: float = 0.0
        self._prev_v_error: float = 0.0
        self._last_t = self.get_clock().now()

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._ref_sub = self.create_subscription(Float64MultiArray, reference_topic, self._on_reference, 10)
        self._timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            f"heading_speed_controller started | odom={odom_topic} ref={reference_topic} cmd={cmd_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._psi = yaw_from_quat(q.x, q.y, q.z, q.w)
        self._u = float(msg.twist.twist.linear.x)
        self._v = float(msg.twist.twist.linear.y)
        self._r = float(msg.twist.twist.angular.z)

    def _on_reference(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn("Guidance reference must be [u_d, v_d, psi_d].")
            return
        self._u_d = float(msg.data[0])
        self._v_d = float(msg.data[1])
        self._psi_d = float(msg.data[2])

    def _on_timer(self) -> None:
        if self._psi is None or self._psi_d is None:
            return

        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        kp = float(self.get_parameter("kp_yaw").value)
        kd = float(self.get_parameter("kd_yaw").value)
        ki_yaw = float(self.get_parameter("ki_yaw").value)
        kp_u = float(self.get_parameter("kp_u").value)
        ki_u = float(self.get_parameter("ki_u").value)
        kd_u = float(self.get_parameter("kd_u").value)
        kp_v = float(self.get_parameter("kp_v").value)
        ki_v = float(self.get_parameter("ki_v").value)
        kd_v = float(self.get_parameter("kd_v").value)
        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        max_surge = float(self.get_parameter("max_surge_speed").value)
        max_sway = float(self.get_parameter("max_sway_speed").value)
        lim_int_yaw = float(self.get_parameter("yaw_integral_limit").value)
        lim_int_u = float(self.get_parameter("u_integral_limit").value)
        lim_int_v = float(self.get_parameter("v_integral_limit").value)

        heading_error = wrap_to_pi(self._psi_d - self._psi)
        u_error = self._u_d - self._u
        v_error = self._v_d - self._v

        self._int_yaw += heading_error * dt
        self._int_u += u_error * dt
        self._int_v += v_error * dt
        self._int_yaw = max(-lim_int_yaw, min(lim_int_yaw, self._int_yaw))
        self._int_u = max(-lim_int_u, min(lim_int_u, self._int_u))
        self._int_v = max(-lim_int_v, min(lim_int_v, self._int_v))

        d_u_error = (u_error - self._prev_u_error) / dt
        d_v_error = (v_error - self._prev_v_error) / dt
        self._prev_heading_error = heading_error
        self._prev_u_error = u_error
        self._prev_v_error = v_error

        yaw_rate_cmd = kp * heading_error + ki_yaw * self._int_yaw - kd * self._r
        yaw_rate_cmd = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate_cmd))

        surge_cmd = kp_u * u_error + ki_u * self._int_u + kd_u * d_u_error
        sway_cmd = kp_v * v_error + ki_v * self._int_v + kd_v * d_v_error
        surge_cmd = max(-max_surge, min(max_surge, surge_cmd))
        sway_cmd = max(-max_sway, min(max_sway, sway_cmd))

        cmd = Twist()
        cmd.linear.x = surge_cmd
        cmd.linear.y = sway_cmd
        cmd.angular.z = yaw_rate_cmd
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingSpeedController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

