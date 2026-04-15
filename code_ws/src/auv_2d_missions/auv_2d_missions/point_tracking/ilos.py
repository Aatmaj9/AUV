import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def normalize_waypoints(xs: List[float], ys: List[float]) -> List[Tuple[float, float]]:
    n = min(len(xs), len(ys))
    return [(float(xs[i]), float(ys[i])) for i in range(n)]


class PointTrackingMission2dIlos(Node):
    """2D point tracking with Integral LOS guidance."""

    def __init__(self) -> None:
        super().__init__("point_tracking_mission_2d_ilos")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference")
        self.declare_parameter("status_topic", "/mission/status")
        self.declare_parameter("waypoints_x", [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter("waypoints_y", [0.0, 0.0, 1.0, 1.0])
        self.declare_parameter("acceptance_radius", 0.35)
        self.declare_parameter("u_cruise", 0.25)
        self.declare_parameter("u_min", 0.05)
        self.declare_parameter("u_max", 0.35)
        self.declare_parameter("lookahead", 0.8)
        self.declare_parameter("ilos_ki", 0.05)
        self.declare_parameter("ilos_int_limit", 2.0)
        self.declare_parameter("switch_alongtrack_margin", 0.1)

        xs = list(self.get_parameter("waypoints_x").value)
        ys = list(self.get_parameter("waypoints_y").value)
        self._waypoints = normalize_waypoints(xs, ys)
        self._idx = 0
        self._done = len(self._waypoints) < 2
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z_ilos = 0.0
        self._last_t = self.get_clock().now()

        self._ref_pub = self.create_publisher(Float64MultiArray, str(self.get_parameter("reference_topic").value), 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._odom_sub = self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

    def _on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)

    def _publish_status(self, text: str) -> None:
        m = String()
        m.data = text
        self._status_pub.publish(m)

    def _publish_reference(self, u_d: float, v_d: float, psi_d: float) -> None:
        m = Float64MultiArray()
        m.data = [float(u_d), float(v_d), float(psi_d)]
        self._ref_pub.publish(m)

    def _on_timer(self) -> None:
        if self._x is None or self._y is None:
            return
        if self._done:
            self._publish_reference(0.0, 0.0, 0.0)
            self._publish_status("point_tracking_2d_ilos: completed")
            return

        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        acceptance = float(self.get_parameter("acceptance_radius").value)
        u_cruise = float(self.get_parameter("u_cruise").value)
        u_min = float(self.get_parameter("u_min").value)
        u_max = float(self.get_parameter("u_max").value)
        lookahead = float(self.get_parameter("lookahead").value)
        ilos_ki = float(self.get_parameter("ilos_ki").value)
        ilos_int_limit = float(self.get_parameter("ilos_int_limit").value)
        switch_margin = float(self.get_parameter("switch_alongtrack_margin").value)

        i0 = self._idx
        i1 = i0 + 1
        if i1 >= len(self._waypoints):
            self._done = True
            return

        x0, y0 = self._waypoints[i0]
        x1, y1 = self._waypoints[i1]
        seg_dx, seg_dy = x1 - x0, y1 - y0
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            self._idx += 1
            return

        tx, ty = seg_dx / seg_len, seg_dy / seg_len
        rx, ry = self._x - x0, self._y - y0
        e_x = tx * rx + ty * ry
        e_y = -ty * rx + tx * ry

        if e_x >= seg_len - switch_margin:
            self._idx += 1
            if self._idx >= len(self._waypoints) - 1:
                xf, yf = self._waypoints[-1]
                if math.hypot(xf - self._x, yf - self._y) <= acceptance:
                    self._done = True
                    self._publish_reference(0.0, 0.0, 0.0)
                    self._publish_status("point_tracking_2d_ilos: completed")
                return
            self._z_ilos *= 0.5
            return

        self._z_ilos += e_y * dt
        self._z_ilos = max(-ilos_int_limit, min(ilos_int_limit, self._z_ilos))

        chi_path = math.atan2(seg_dy, seg_dx)
        psi_d = chi_path - math.atan2(e_y + ilos_ki * self._z_ilos, max(1e-3, lookahead))
        self._publish_reference(max(u_min, min(u_max, u_cruise)), 0.0, psi_d)
        self._publish_status(f"point_tracking_2d_ilos: leg={i0 + 1}->{i1 + 1}/{len(self._waypoints)} cross={e_y:.2f}m")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointTrackingMission2dIlos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

