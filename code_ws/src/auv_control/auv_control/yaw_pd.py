# ==========================================================
# yaw_pd.py
# PD controller for yaw control
# ==========================================================

import math

class YawPD:
    def __init__(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd

    def compute(self, yaw_ref: float, yaw: float, yaw_rate: float) -> float:
        error = yaw_ref - yaw

        # Wrap error to [-pi, pi]
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi

        return self.kp * error - self.kd * yaw_rate

