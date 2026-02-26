# ==========================================================
# depth_pd.py
# PD controller for depth (heave)
# ==========================================================

class DepthPD:
    def __init__(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd

    def compute(self, z_ref: float, z: float, z_dot: float) -> float:
        error = z_ref - z
        return self.kp * error - self.kd * z_dot

