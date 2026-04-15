import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def normalize_waypoints(xs: List[float], ys: List[float]) -> List[Tuple[float, float]]:
    n = min(len(xs), len(ys))
    return [(float(xs[i]), float(ys[i])) for i in range(n)]


class PointTrackingMission2D(Node):
    """
    Mission: track XY waypoints in local 2D frame.

    Publishes /guidance/reference as:
      [u_d, v_d, psi_d]
      u_d   : desired surge speed [m/s]
      v_d   : desired sway speed [m/s]
      psi_d : desired yaw heading [rad]
    """

    def __init__(self) -> None:
        super().__init__("point_tracking_mission_2d")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference")
        self.declare_parameter("status_topic", "/mission/status")

        self.declare_parameter("waypoints_x", [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter("waypoints_y", [0.0, 0.0, 1.0, 1.0])

        self.declare_parameter("acceptance_radius", 0.35)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("min_speed", 0.05)
        self.declare_parameter("k_dist", 0.5)

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        xs = list(self.get_parameter("waypoints_x").value)
        ys = list(self.get_parameter("waypoints_y").value)
        self._waypoints = normalize_waypoints(xs, ys)
        self._idx = 0
        self._done = len(self._waypoints) == 0

        self._x: Optional[float] = None
        self._y: Optional[float] = None

        self._ref_pub = self.create_publisher(Float64MultiArray, reference_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            f"point_tracking_mission_2d started | odom={odom_topic} ref={reference_topic} waypoints={self._waypoints}"
        )

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
            self._publish_status("point_tracking_2d: completed")
            return

        acceptance = float(self.get_parameter("acceptance_radius").value)
        max_speed = float(self.get_parameter("max_speed").value)
        min_speed = float(self.get_parameter("min_speed").value)
        k_dist = float(self.get_parameter("k_dist").value)

        tx, ty = self._waypoints[self._idx]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)

        if dist <= acceptance:
            self._idx += 1
            if self._idx >= len(self._waypoints):
                self._done = True
                self._publish_reference(0.0, 0.0, 0.0)
                self._publish_status("point_tracking_2d: completed")
                self.get_logger().info("point_tracking_mission_2d completed.")
                return
            tx, ty = self._waypoints[self._idx]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.hypot(dx, dy)

        psi_d = math.atan2(dy, dx)
        u_d = max(min_speed, min(max_speed, k_dist * dist))
        v_d = 0.0

        self._publish_reference(u_d, v_d, psi_d)
        self._publish_status(
            f"point_tracking_2d: wp={self._idx + 1}/{len(self._waypoints)} "
            f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointTrackingMission2D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

