"""Continuous-time kinematic plant used for prediction (same physical model, clean implementation)."""

from __future__ import annotations

import numpy as np

from navigation.nav_kinematics import eul_rate_matrix, eul_to_rotm


def plant_rhs(x: np.ndarray, u: np.ndarray | None) -> np.ndarray:
    """
    dx/dt for state x = [p_ned, euler, v_body, omega_body, a_body, actuators...].
    Actuator block: x_dot_act = u - x_act (tracking command with unit time constant).
    """
    x = np.asarray(x, dtype=float).flatten()
    n = len(x)
    euler = x[3:6]
    v = x[6:9]
    w = x[9:12]
    a = x[12:15]
    R_bn = eul_to_rotm(euler)
    J = eul_rate_matrix(euler)
    xd = np.zeros(n)
    xd[0:3] = R_bn @ v
    xd[3:6] = J @ w
    xd[6:9] = a + np.cross(v, w)
    xd[9:12] = 0.0
    xd[12:15] = np.cross(a, w)
    if n > 15:
        uu = np.asarray(u, dtype=float).flatten() if u is not None else np.zeros(n - 15)
        xd[15:] = uu - x[15:]
    return xd


def process_noise_coupling_n(euler: np.ndarray, n_states: int) -> np.ndarray:
    E = np.zeros((n_states, 6))
    R_bn = eul_to_rotm(np.asarray(euler, dtype=float).reshape(3))
    R_nb = R_bn.T
    E[9:12, 0:3] = R_nb
    E[12:15, 3:6] = R_nb
    return E


def jacobian_square(fun, x: np.ndarray, eps_scale: float = 1e-6) -> np.ndarray:
    """Central-difference Jacobian of fun: R^n -> R^n (square)."""
    x = np.asarray(x, dtype=float).flatten()
    n = len(x)
    f0 = np.asarray(fun(x), dtype=float).flatten()
    assert f0.shape[0] == n, "jacobian_square expects f: R^n -> R^n"
    J = np.zeros((n, n))
    for i in range(n):
        h = eps_scale * max(1.0, abs(x[i]))
        if h < 1e-12:
            h = eps_scale
        xp = x.copy()
        xm = x.copy()
        xp[i] += h
        xm[i] -= h
        J[:, i] = (fun(xp).flatten() - fun(xm).flatten()) / (2.0 * h)
    return J
