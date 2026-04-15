"""Minimal NED/body kinematics (ZYX Euler): self-contained, not shared with auv_navigation."""

from __future__ import annotations

import warnings

import numpy as np


def ssa(ang: float, deg: bool = False) -> float:
    a = float(ang)
    if deg:
        a = (a + 180.0) % 360.0 - 180.0
        return a
    return (a + np.pi) % (2 * np.pi) - np.pi


def clip(value: float, threshold: float) -> float:
    if threshold is None or not np.isfinite(threshold) or threshold == np.inf:
        return value
    if value > threshold:
        return threshold
    if value < -threshold:
        return -threshold
    return value


def skew(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


def eul_to_rotm(eul: np.ndarray, deg: bool = False) -> np.ndarray:
    """Body → NED rotation matrix, ZYX intrinsic (yaw ψ, pitch θ, roll φ)."""
    eul = np.asarray(eul, dtype=float).reshape(3)
    if deg:
        phi, theta, psi = eul * (np.pi / 180.0)
    else:
        phi, theta, psi = eul
    c1, s1 = np.cos(phi), np.sin(phi)
    c2, s2 = np.cos(theta), np.sin(theta)
    c3, s3 = np.cos(psi), np.sin(psi)
    return np.array(
        [
            [c2 * c3, -c1 * s3 + s1 * s2 * c3, s1 * s3 + c1 * s2 * c3],
            [c2 * s3, c1 * c3 + s1 * s2 * s3, -s1 * c3 + c1 * s2 * s3],
            [-s2, s1 * c2, c1 * c2],
        ]
    )


def rotm_to_eul(rotm: np.ndarray, prev_eul: np.ndarray | None = None, silent: bool = True) -> np.ndarray:
    """Rotation matrix (body→NED) to ZYX Euler [φ, θ, ψ] radians."""
    rotm = np.asarray(rotm, dtype=float).reshape(3, 3)
    theta1 = np.arcsin(-rotm[2, 0])
    theta2 = np.pi - theta1 if theta1 > 0 else -np.pi - theta1

    if np.isclose(theta1, np.pi / 2):
        phi1 = np.arctan2(rotm[0, 1], rotm[1, 1])
        eul1 = np.array([phi1, theta1, 0.0])
        eul2 = eul1
    elif np.isclose(theta1, -np.pi / 2):
        phi1 = np.arctan2(-rotm[0, 1], rotm[1, 1])
        eul1 = np.array([phi1, theta1, 0.0])
        eul2 = eul1
    else:
        phi1 = np.arctan2(rotm[2, 1], rotm[2, 2])
        phi2 = np.arctan2(-rotm[2, 1], -rotm[2, 2])
        psi1 = np.arctan2(rotm[1, 0], rotm[0, 0])
        psi2 = np.arctan2(-rotm[1, 0], -rotm[0, 0])
        eul1 = np.array([phi1, theta1, psi1])
        eul2 = np.array([phi2, theta2, psi2])

    if prev_eul is not None:
        pe = np.asarray(prev_eul, dtype=float).reshape(3)
        eul = eul1 if np.linalg.norm(eul1 - pe) <= np.linalg.norm(eul2 - pe) else eul2
    else:
        if not silent:
            warnings.warn("rotm_to_eul: ambiguous branch; using primary solution.")
        eul = eul1
    return eul


def eul_rate_matrix(eul: np.ndarray, deg: bool = False) -> np.ndarray:
    """Euler rates from body angular velocity: euler_dot = J @ omega_body."""
    eul = np.asarray(eul, dtype=float).reshape(3)
    if deg:
        phi, theta, psi = eul * (np.pi / 180.0)
    else:
        phi, theta, psi = eul
    J = np.zeros((3, 3))
    J[0, 0] = 1.0
    J[0, 1] = np.sin(phi) * np.tan(theta)
    J[0, 2] = np.cos(phi) * np.tan(theta)
    J[1, 1] = np.cos(phi)
    J[1, 2] = -np.sin(phi)
    J[2, 1] = np.sin(phi) / np.cos(theta)
    J[2, 2] = np.cos(phi) / np.cos(theta)
    return J


def eul_to_quat(eul: np.ndarray, deg: bool = False) -> np.ndarray:
    """Unit quaternion [w, x, y, z] from ZYX Euler."""
    eul = np.asarray(eul, dtype=float).reshape(3)
    if deg:
        phi, theta, psi = eul * (np.pi / 180.0)
    else:
        phi, theta, psi = eul
    quat = np.array(
        [
            np.cos(psi / 2) * np.cos(theta / 2) * np.cos(phi / 2)
            + np.sin(psi / 2) * np.sin(theta / 2) * np.sin(phi / 2),
            np.cos(psi / 2) * np.cos(theta / 2) * np.sin(phi / 2)
            - np.sin(psi / 2) * np.sin(theta / 2) * np.cos(phi / 2),
            np.sin(psi / 2) * np.cos(theta / 2) * np.sin(phi / 2)
            + np.cos(psi / 2) * np.sin(theta / 2) * np.cos(phi / 2),
            np.sin(psi / 2) * np.cos(theta / 2) * np.cos(phi / 2)
            - np.cos(psi / 2) * np.sin(theta / 2) * np.sin(phi / 2),
        ]
    )
    return quat / np.linalg.norm(quat)


def quat_to_eul(quat: np.ndarray, deg: bool = False) -> np.ndarray:
    """ZYX Euler from unit quaternion [w, x, y, z]."""
    quat = np.asarray(quat, dtype=float).reshape(4)
    qw, qx, qy, qz = quat
    theta1 = np.arcsin(2.0 * (qy * qw - qx * qz))
    theta2 = np.pi - theta1 if theta1 > 0 else -np.pi - theta1
    if np.isclose(theta1, np.pi / 2):
        phi1 = np.arctan2(2.0 * (qx * qy - qz * qw), 1.0 - 2.0 * (qx**2 + qz**2))
        eul = np.array([phi1, theta1, 0.0])
    elif np.isclose(theta1, -np.pi / 2):
        phi1 = np.arctan2(-2.0 * (qx * qy - qz * qw), 1.0 - 2.0 * (qx**2 + qz**2))
        eul = np.array([phi1, theta1, 0.0])
    else:
        phi1 = np.arctan2(2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx**2 + qy**2))
        psi1 = np.arctan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy**2 + qz**2))
        eul = np.array([phi1, theta1, psi1])
    if deg:
        eul = eul * (180.0 / np.pi)
    return eul


def quat_to_rotm(quat: np.ndarray) -> np.ndarray:
    """Body → NED from unit quaternion [w,x,y,z]."""
    quat = np.asarray(quat, dtype=float).reshape(4)
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
