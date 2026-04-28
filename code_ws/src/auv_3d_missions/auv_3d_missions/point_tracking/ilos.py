import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def normalize_waypoints(xs: List[float], ys: List[float], zs: List[float]) -> List[Tuple[float, float, float]]:
    n = min(len(xs), len(ys), len(zs))
    return [(float(xs[i]), float(ys[i]), float(zs[i])) for i in range(n)]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PointTrackingMission3dIlos(Node):
    """3D point tracking using LOS + integral correction on lateral and vertical errors."""

    def __init__(self) -> None:
        super().__init__("point_tracking_mission_3d_ilos")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference_3d")
        self.declare_parameter("status_topic", "/mission/status")

        self.declare_parameter("waypoints_x", [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter("waypoints_y", [0.0, 0.0, 1.0, 1.0])
        self.declare_parameter("waypoints_z", [0.0, 0.2, 0.2, 0.0])

        self.declare_parameter("acceptance_radius", 0.45)
        self.declare_parameter("switch_alongtrack_margin", 0.15)
        self.declare_parameter("u_cruise", 0.25)
        self.declare_parameter("u_min", 0.05)
        self.declare_parameter("u_max", 0.40)
        self.declare_parameter("lookahead_xy", 0.9)
        self.declare_parameter("lookahead_z", 0.7)
        self.declare_parameter("ilos_ki_xy", 0.05)
        self.declare_parameter("ilos_ki_z", 0.05)
        self.declare_parameter("ilos_int_limit_xy", 2.0)
        self.declare_parameter("ilos_int_limit_z", 1.5)
        self.declare_parameter("w_gain", 0.7)
        self.declare_parameter("w_max", 0.25)

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._waypoints = normalize_waypoints(
            list(self.get_parameter("waypoints_x").value),
            list(self.get_parameter("waypoints_y").value),
            list(self.get_parameter("waypoints_z").value),
        )
        self._idx = 0
        self._done = len(self._waypoints) < 2
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._z_int_xy = 0.0
        self._z_int_z = 0.0
        self._last_t = self.get_clock().now()

        self._ref_pub = self.create_publisher(Float64MultiArray, reference_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

    def _on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        self._z = float(msg.pose.pose.position.z)

    def _publish_status(self, text: str) -> None:
        m = String()
        m.data = text
        self._status_pub.publish(m)

    def _publish_reference(self, u_d: float, v_d: float, w_d: float, phi_d: float, theta_d: float, psi_d: float) -> None:
        m = Float64MultiArray()
        m.data = [u_d, v_d, w_d, phi_d, theta_d, psi_d]
        self._ref_pub.publish(m)

    def _on_timer(self) -> None:
        if self._x is None or self._y is None or self._z is None:
            return
        if self._done:
            self._publish_reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._publish_status("point_tracking_3d_ilos: completed")
            return

        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        acceptance = float(self.get_parameter("acceptance_radius").value)
        switch_margin = float(self.get_parameter("switch_alongtrack_margin").value)
        u_d = clamp(
            float(self.get_parameter("u_cruise").value),
            float(self.get_parameter("u_min").value),
            float(self.get_parameter("u_max").value),
        )
        look_xy = float(self.get_parameter("lookahead_xy").value)
        look_z = float(self.get_parameter("lookahead_z").value)
        ki_xy = float(self.get_parameter("ilos_ki_xy").value)
        ki_z = float(self.get_parameter("ilos_ki_z").value)
        lim_xy = float(self.get_parameter("ilos_int_limit_xy").value)
        lim_z = float(self.get_parameter("ilos_int_limit_z").value)

        i0 = self._idx
        i1 = i0 + 1
        if i1 >= len(self._waypoints):
            self._done = True
            return

        x0, y0, z0 = self._waypoints[i0]
        x1, y1, z1 = self._waypoints[i1]
        sx, sy, sz = x1 - x0, y1 - y0, z1 - z0
        sl = math.sqrt(sx * sx + sy * sy + sz * sz)
        if sl < 1e-6:
            self._idx += 1
            return

        tx, ty, tz = sx / sl, sy / sl, sz / sl
        rx, ry, rz = self._x - x0, self._y - y0, self._z - z0
        along = tx * rx + ty * ry + tz * rz
        ex, ey, ez = rx - along * tx, ry - along * ty, rz - along * tz

        # Use signed horizontal cross-track so integral yaw correction preserves side-of-path information.
        hlen = math.hypot(sx, sy)
        if hlen > 1e-6:
            hx, hy = sx / hlen, sy / hlen
            e_xy_signed = -hy * rx + hx * ry
        else:
            e_xy_signed = 0.0
        e_xy_abs = math.hypot(ex, ey)

        if along >= sl - switch_margin:
            self._idx += 1
            if self._idx >= len(self._waypoints) - 1:
                xf, yf, zf = self._waypoints[-1]
                if math.sqrt((xf - self._x) ** 2 + (yf - self._y) ** 2 + (zf - self._z) ** 2) <= acceptance:
                    self._done = True
                    self._publish_reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    self._publish_status("point_tracking_3d_ilos: completed")
                return
            self._z_int_xy *= 0.5
            self._z_int_z *= 0.5
            return

        self._z_int_xy = clamp(self._z_int_xy + e_xy_signed * dt, -lim_xy, lim_xy)
        self._z_int_z = clamp(self._z_int_z + ez * dt, -lim_z, lim_z)

        chi = math.atan2(sy, sx)
        psi_d = chi - math.atan2(e_xy_signed + ki_xy * self._z_int_xy, max(1e-3, look_xy))
        theta_d = math.atan2(-(ez + ki_z * self._z_int_z), max(1e-3, look_z))

        w_gain = float(self.get_parameter("w_gain").value)
        w_max = float(self.get_parameter("w_max").value)
        w_d = clamp(-w_gain * (ez + ki_z * self._z_int_z), -w_max, w_max)

        self._publish_reference(u_d, 0.0, w_d, 0.0, theta_d, psi_d)
        self._publish_status(
            f"point_tracking_3d_ilos: leg={i0 + 1}->{i1 + 1}/{len(self._waypoints)} "
            f"cross_xy={e_xy_signed:.2f}m |abs|={e_xy_abs:.2f}m cross_z={ez:.2f}m"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointTrackingMission3dIlos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
