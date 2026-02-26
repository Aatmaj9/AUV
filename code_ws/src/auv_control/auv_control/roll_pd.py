# ==========================================================
# roll_pd.py
# PD controller for roll stabilization
# ==========================================================

class RollPD:
    def __init__(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd

    def compute(self, roll: float, roll_rate: float) -> float:
        return -self.kp * roll - self.kd * roll_rate


