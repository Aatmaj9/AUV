"""Measurement models and Jacobians (compact analytic + central differences where practical)."""

from __future__ import annotations

import numpy as np

from navigation.nav_kinematics import eul_to_rotm, rotm_to_eul, skew


def _deg_to_rad_mount(theta_bs_deg: np.ndarray) -> np.ndarray:
    return np.asarray(theta_bs_deg, dtype=float).reshape(3) * (np.pi / 180.0)


def lever_arm_position(x: np.ndarray, r_bs_b: np.ndarray) -> np.ndarray:
    """Antenna / sensor origin in NED: p + R_bn @ r_bs."""
    x = np.asarray(x, dtype=float).flatten()
    p = x[0:3]
    euler = x[3:6]
    R_bn = eul_to_rotm(euler)
    r_bs_b = np.asarray(r_bs_b, dtype=float).reshape(3)
    return p + R_bn @ r_bs_b


def predict_imu(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray) -> np.ndarray:
    """9-vector: euler_synthetic, gyro_sensor, accel_sensor (matches auv_navigation IMU fusion convention)."""
    x = np.asarray(x, dtype=float).flatten()
    euler = x[3:6]
    v = x[6:9]
    w = x[9:12]
    a = x[12:15]
    R_bn = eul_to_rotm(euler)
    Theta_bs = _deg_to_rad_mount(theta_bs_deg)
    R_bs = eul_to_rotm(Theta_bs)
    R_sb = R_bs.T
    y = np.zeros(9)
    y[0:3] = rotm_to_eul(R_bn @ R_bs, prev_eul=euler)
    y[3:6] = R_sb @ w
    y[6:9] = R_sb @ a + R_sb @ np.cross(w, np.cross(w, r_bs_b))
    return y


def jacobian_imu_numerical(
    x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray, n_state: int, eps: float = 1e-5
) -> np.ndarray:
    x0 = np.asarray(x, dtype=float).flatten()[:n_state].copy()
    H = np.zeros((9, n_state))
    for j in range(n_state):
        h = eps * max(1.0, abs(x0[j]))
        if j >= 3 and j < 6:
            h = max(h, 1e-5)
        xp = x0.copy()
        xm = x0.copy()
        xp[j] += h
        xm[j] -= h
        H[:, j] = (predict_imu(xp, r_bs_b, theta_bs_deg) - predict_imu(xm, r_bs_b, theta_bs_deg)) / (2.0 * h)
    return H


def predict_dvl_body(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float).flatten()
    v = x[6:9]
    w = x[9:12]
    Theta_bs = _deg_to_rad_mount(theta_bs_deg)
    R_bs = eul_to_rotm(Theta_bs)
    R_sb = R_bs.T
    r_bs_b = np.asarray(r_bs_b, dtype=float).reshape(3)
    u_b = v + np.cross(w, r_bs_b)
    return R_sb @ u_b


def jacobian_dvl(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray, n_state: int) -> np.ndarray:
    Theta_bs = _deg_to_rad_mount(theta_bs_deg)
    R_bs = eul_to_rotm(Theta_bs)
    R_sb = R_bs.T
    r_bs_b = np.asarray(r_bs_b, dtype=float).reshape(3)
    H = np.zeros((3, n_state))
    H[:, 6:9] = R_sb
    H[:, 9:12] = -R_sb @ skew(r_bs_b)
    return H


def predict_dr_position(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray) -> np.ndarray:
    return lever_arm_position(x, r_bs_b)


def jacobian_dr_position(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray, n_state: int) -> np.ndarray:
    """Analytic in p; central diff on euler columns only (3 evals per column)."""
    x0 = np.asarray(x, dtype=float).flatten()[:n_state].copy()
    H = np.zeros((3, n_state))
    H[0:3, 0:3] = np.eye(3)
    for j in range(3, 6):
        h = 1e-5
        xp = x0.copy()
        xm = x0.copy()
        xp[j] += h
        xm[j] -= h
        H[0:3, j] = (lever_arm_position(xp, r_bs_b) - lever_arm_position(xm, r_bs_b)) / (
            2.0 * h
        )
    return H


def predict_depth(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray) -> float:
    p = lever_arm_position(x, r_bs_b)
    return float(p[2])


def jacobian_depth(x: np.ndarray, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray, n_state: int) -> np.ndarray:
    H3 = jacobian_dr_position(x, r_bs_b, theta_bs_deg, n_state)
    return H3[2:3, :]


def predict_range(
    x: np.ndarray, z_seabed_ned: float, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray
) -> float:
    p = lever_arm_position(x, r_bs_b)
    return float(z_seabed_ned - p[2])


def jacobian_range(
    x: np.ndarray, z_seabed_ned: float, r_bs_b: np.ndarray, theta_bs_deg: np.ndarray, n_state: int
) -> np.ndarray:
    _ = z_seabed_ned
    return -jacobian_depth(x, r_bs_b, theta_bs_deg, n_state)


def jacobian_encoders(n_state: int, n_act: int) -> np.ndarray:
    if n_act <= 0:
        raise ValueError("encoders require n_act > 0")
    H = np.zeros((n_act, n_state))
    H[:, 15 : 15 + n_act] = np.eye(n_act)
    return H
