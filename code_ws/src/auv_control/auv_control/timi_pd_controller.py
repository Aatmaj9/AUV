#!/usr/bin/env python3
# ==========================================================
# timi_pd_controller.py
# Unified PD controller for TIMI (SAUVC AUV)
# Depth + Roll + Yaw
# 8 Thrusters
# Surge thrusters at 45 degrees (COMPENSATED)
# ==========================================================

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
import time

from depth_pd import DepthPD
from roll_pd import RollPD
from yaw_pd import YawPD


# ---------------- UTILS ----------------
def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))


def thrust_to_pwm(thrust, pwm_min, pwm_max):
    thrust = clamp(thrust, -1.0, 1.0)
    mid = 0.5 * (pwm_min + pwm_max)
    amp = 0.5 * (pwm_max - pwm_min)
    return mid + thrust * amp


# ---------------- CONTROLLER ----------------
class TimiPDController(Node):

    def __init__(self):
        super().__init__('timi_pd_controller')

        # ---------- PARAMETERS ----------
        self.declare_parameters('', [
            ('depth.kp', 2.0),
            ('depth.kd', 0.4),
            ('depth.deadband', 0.05),

            ('roll.kp', 1.2),
            ('roll.kd', 0.3),

            ('yaw.kp', 2.5),
            ('yaw.kd', 0.6),

            ('mission.depth', 1.5),
            ('mission.forward_thrust', 0.3),

            ('limits.heave_max', 0.25),
            ('limits.surge_max', 0.5),
            ('limits.yaw_max', 0.3),

            ('pwm.min', 1350.0),
            ('pwm.max', 1650.0),
        ])

        # ---------- LOAD ----------
        self.z_ref = self.get_parameter('mission.depth').value
        self.forward_thrust = self.get_parameter('mission.forward_thrust').value

        self.heave_max = self.get_parameter('limits.heave_max').value
        self.surge_max = self.get_parameter('limits.surge_max').value
        self.yaw_max = self.get_parameter('limits.yaw_max').value

        self.deadband = self.get_parameter('depth.deadband').value

        self.pwm_min = self.get_parameter('pwm.min').value
        self.pwm_max = self.get_parameter('pwm.max').value

        # ---------- PD CONTROLLERS ----------
        self.depth_pd = DepthPD(
            self.get_parameter('depth.kp').value,
            self.get_parameter('depth.kd').value
        )
        self.roll_pd = RollPD(
            self.get_parameter('roll.kp').value,
            self.get_parameter('roll.kd').value
        )
        self.yaw_pd = YawPD(
            self.get_parameter('yaw.kp').value,
            self.get_parameter('yaw.kd').value
        )

        # ---------- STATE ----------
        self.depth = 0.0
        self.depth_rate = 0.0
        self.prev_depth = None
        self.prev_time = None

        self.roll = 0.0
        self.roll_rate = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.yaw_ref = 0.0

        # ---------- ROS ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(Float32, '/depth', self.depth_cb, 10)

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/timi/thruster_cmd', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("TIMI PD Controller started.")

    # ---------- CALLBACKS ----------
    def imu_cb(self, msg: Imu):
        q = msg.orientation

        self.roll = math.atan2(
            2.0 * (q.w*q.x + q.y*q.z),
            1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        )

        self.yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        self.roll_rate = msg.angular_velocity.x
        self.yaw_rate = msg.angular_velocity.z

    def depth_cb(self, msg: Float32):
        now = self.get_clock().now().nanoseconds * 1e-9
        z = msg.data

        if self.prev_depth is not None:
            dt = now - self.prev_time
            if dt > 0.01:
                dz = (z - self.prev_depth) / dt
                self.depth_rate = 0.7 * self.depth_rate + 0.3 * dz

        self.depth = z
        self.prev_depth = z
        self.prev_time = now

    # ---------- CONTROL LOOP ----------
    def control_loop(self):
        # ---- Depth ----
        err = self.z_ref - self.depth
        if abs(err) < self.deadband:
            u_depth = 0.0
        else:
            u_depth = self.depth_pd.compute(
                self.z_ref, self.depth, self.depth_rate)

        # ---- Attitude ----
        u_roll = self.roll_pd.compute(self.roll, self.roll_rate)
        u_yaw = self.yaw_pd.compute(self.yaw_ref, self.yaw, self.yaw_rate)

        # ---- Clamp ----
        u_depth = clamp(u_depth, -self.heave_max, self.heave_max)
        u_roll = clamp(u_roll, -self.heave_max, self.heave_max)
        u_yaw = clamp(u_yaw, -self.yaw_max, self.yaw_max)

        # ---- 45 DEGREE SURGE THRUSTER COMPENSATION ----
        COS_45 = 0.70710678  # cos(45°)

        surge_cmd = clamp(
            self.forward_thrust / COS_45,
            -self.surge_max,
            self.surge_max
        )

        yaw_cmd = clamp(
            u_yaw / COS_45,
            -self.yaw_max,
            self.yaw_max
        )

        # ---- Thrusters (8) ----
        thrust = [0.0] * 8

        # Surge thrusters (T1–T4) at 45°
        thrust[0] = surge_cmd - yaw_cmd
        thrust[1] = surge_cmd + yaw_cmd
        thrust[2] = surge_cmd - yaw_cmd
        thrust[3] = surge_cmd + yaw_cmd

        # Heave thrusters (T5–T8)
        thrust[4] = u_depth + u_roll
        thrust[5] = u_depth - u_roll
        thrust[6] = u_depth + u_roll
        thrust[7] = u_depth - u_roll

        # ---- PWM ----
        pwm = [
            thrust_to_pwm(t, self.pwm_min, self.pwm_max)
            for t in thrust
        ]

        self.cmd_pub.publish(Float32MultiArray(data=pwm))


def main():
    rclpy.init()
    node = TimiPDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
