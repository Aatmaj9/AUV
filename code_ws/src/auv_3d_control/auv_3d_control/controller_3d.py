import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_from_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Controller3D(Node):
    """Tracks [u_d,v_d,w_d,phi_d,theta_d,psi_d] and publishes /control/cmd_vel_3d."""

    def __init__(self) -> None:
        super().__init__("controller_3d")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference_3d")
        self.declare_parameter("cmd_topic", "/control/cmd_vel_3d")

        for axis, kp, ki, kd in (
            ("u", 1.1, 0.1, 0.08),
            ("v", 1.0, 0.1, 0.08),
            ("w", 1.2, 0.12, 0.09),
            ("roll", 1.2, 0.05, 0.2),
            ("pitch", 1.4, 0.08, 0.25),
            ("yaw", 1.6, 0.06, 0.25),
        ):
            self.declare_parameter(f"kp_{axis}", kp)
            self.declare_parameter(f"ki_{axis}", ki)
            self.declare_parameter(f"kd_{axis}", kd)
            self.declare_parameter(f"{axis}_integral_limit", 0.8)

        self.declare_parameter("max_u_cmd", 0.45)
        self.declare_parameter("max_v_cmd", 0.35)
        self.declare_parameter("max_w_cmd", 0.3)
        self.declare_parameter("max_roll_rate_cmd", 0.6)
        self.declare_parameter("max_pitch_rate_cmd", 0.6)
        self.declare_parameter("max_yaw_rate_cmd", 0.9)

        self._u = self._v = self._w = 0.0
        self._p = self._q = self._r = 0.0
        self._phi: Optional[float] = None
        self._theta: Optional[float] = None
        self._psi: Optional[float] = None

        self._u_d = self._v_d = self._w_d = 0.0
        self._phi_d = self._theta_d = self._psi_d = 0.0
        self._have_ref = False

        self._int = {k: 0.0 for k in ("u", "v", "w", "roll", "pitch", "yaw")}
        self._prev_e = {k: 0.0 for k in ("u", "v", "w", "roll", "pitch", "yaw")}
        self._last_t = self.get_clock().now()

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._ref_sub = self.create_subscription(Float64MultiArray, reference_topic, self._on_reference, 10)
        self._timer = self.create_timer(0.05, self._on_timer)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._phi, self._theta, self._psi = euler_from_quat(q.x, q.y, q.z, q.w)
        self._u = float(msg.twist.twist.linear.x)
        self._v = float(msg.twist.twist.linear.y)
        self._w = float(msg.twist.twist.linear.z)
        self._p = float(msg.twist.twist.angular.x)
        self._q = float(msg.twist.twist.angular.y)
        self._r = float(msg.twist.twist.angular.z)

    def _on_reference(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().warn("3D guidance reference must be [u_d,v_d,w_d,phi_d,theta_d,psi_d].")
            return
        self._u_d, self._v_d, self._w_d, self._phi_d, self._theta_d, self._psi_d = [float(v) for v in msg.data[:6]]
        self._have_ref = True

    def _pid(self, axis: str, e: float, rate_fb: float, dt: float) -> float:
        kp = float(self.get_parameter(f"kp_{axis}").value)
        ki = float(self.get_parameter(f"ki_{axis}").value)
        kd = float(self.get_parameter(f"kd_{axis}").value)
        lim_i = float(self.get_parameter(f"{axis}_integral_limit").value)

        self._int[axis] += e * dt
        self._int[axis] = max(-lim_i, min(lim_i, self._int[axis]))
        de = (e - self._prev_e[axis]) / dt
        self._prev_e[axis] = e
        return kp * e + ki * self._int[axis] + kd * de - kd * rate_fb * 0.2

    def _on_timer(self) -> None:
        if self._phi is None or self._theta is None or self._psi is None or not self._have_ref:
            return

        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        eu = self._u_d - self._u
        ev = self._v_d - self._v
        ew = self._w_d - self._w
        eroll = wrap_to_pi(self._phi_d - self._phi)
        epitch = wrap_to_pi(self._theta_d - self._theta)
        eyaw = wrap_to_pi(self._psi_d - self._psi)

        cmd = Twist()
        cmd.linear.x = self._pid("u", eu, 0.0, dt)
        cmd.linear.y = self._pid("v", ev, 0.0, dt)
        cmd.linear.z = self._pid("w", ew, 0.0, dt)
        cmd.angular.x = self._pid("roll", eroll, self._p, dt)
        cmd.angular.y = self._pid("pitch", epitch, self._q, dt)
        cmd.angular.z = self._pid("yaw", eyaw, self._r, dt)

        cmd.linear.x = max(-float(self.get_parameter("max_u_cmd").value), min(float(self.get_parameter("max_u_cmd").value), cmd.linear.x))
        cmd.linear.y = max(-float(self.get_parameter("max_v_cmd").value), min(float(self.get_parameter("max_v_cmd").value), cmd.linear.y))
        cmd.linear.z = max(-float(self.get_parameter("max_w_cmd").value), min(float(self.get_parameter("max_w_cmd").value), cmd.linear.z))
        cmd.angular.x = max(-float(self.get_parameter("max_roll_rate_cmd").value), min(float(self.get_parameter("max_roll_rate_cmd").value), cmd.angular.x))
        cmd.angular.y = max(-float(self.get_parameter("max_pitch_rate_cmd").value), min(float(self.get_parameter("max_pitch_rate_cmd").value), cmd.angular.y))
        cmd.angular.z = max(-float(self.get_parameter("max_yaw_rate_cmd").value), min(float(self.get_parameter("max_yaw_rate_cmd").value), cmd.angular.z))

        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Controller3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

