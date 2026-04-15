"""Continuous-discrete EKF with stable covariance updates and optional innovation gating."""

from __future__ import annotations

import warnings

import numpy as np
import scipy.linalg

from navigation.nav_kinematics import clip, ssa
from navigation.nav_plant import jacobian_square, plant_rhs, process_noise_coupling_n


class NavEKF:
    def __init__(
        self,
        dt: float,
        n_states: int,
        n_inp: int,
        pro_noise_cov: np.ndarray,
    ):
        self.dt = float(dt)
        self.t = 0.0
        self.n_states = int(n_states)
        self.n_inp = int(n_inp)
        self.n_pro_noise = int(pro_noise_cov.shape[0])
        self.Q = np.asarray(pro_noise_cov, dtype=float)

        self.x = np.zeros((self.n_states, 1))
        self.P0 = 10.0 * np.eye(self.n_states)
        self.P = np.array(self.P0, copy=True)

    def predict(self, u: np.ndarray, threshold: np.ndarray | None = None) -> None:
        xf = self.x.flatten().copy()
        u = np.asarray(u, dtype=float).flatten() if u is not None else np.zeros(max(0, self.n_inp))

        def rhs(z):
            return plant_rhs(z, u).flatten()

        A = jacobian_square(rhs, xf)

        E = process_noise_coupling_n(xf[3:6], self.n_states)

        if np.abs(np.linalg.det(A)) < 1e-8:
            Ad = np.eye(self.n_states) + A * self.dt
            Ed = E * self.dt
        else:
            Ad = scipy.linalg.expm(A * self.dt)
            try:
                Ai = np.linalg.inv(A)
                Ed = Ai @ (Ad - np.eye(self.n_states)) @ E
            except np.linalg.LinAlgError:
                Ed = E * self.dt

        k1 = rhs(xf)
        k2 = rhs(xf + 0.5 * self.dt * k1)
        k3 = rhs(xf + 0.5 * self.dt * k2)
        k4 = rhs(xf + self.dt * k3)
        change = (self.dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        if threshold is not None:
            for i in range(len(change)):
                change[i] = clip(change[i], threshold[i])

        xf_new = xf + change
        self.x = xf_new.reshape(-1, 1)
        for i in range(3, 6):
            self.x[i, 0] = ssa(self.x[i, 0])

        P = self.P
        self.P = Ad @ P @ Ad.T + Ed @ self.Q @ Ed.T

        if np.any(np.isnan(self.P)):
            warnings.warn("NaN in P after predict; resetting covariance.")
            self.P = np.array(self.P0, copy=True)

        self.t += self.dt

    def update(
        self,
        y: np.ndarray,
        H: np.ndarray,
        R: np.ndarray,
        h_at_x: np.ndarray,
        threshold: np.ndarray | None = None,
        imu_ssa: bool = False,
        mahalanobis_gate: float | None = None,
    ) -> bool:
        """
        Joseph-form covariance update. K via solve(S.T, H @ P) for numerical stability.
        Returns False if update was skipped (singular S or failed gating).
        """
        P = self.P
        x = self.x.flatten()
        y = np.asarray(y, dtype=float).reshape(-1, 1)
        z_pred = np.asarray(h_at_x, dtype=float).reshape(-1, 1)
        inn = y - z_pred

        if imu_ssa:
            for i in range(min(3, inn.shape[0])):
                inn[i, 0] = ssa(inn[i, 0])

        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)
        S = H @ P @ H.T + R

        if mahalanobis_gate is not None and np.isfinite(mahalanobis_gate):
            try:
                md2 = float(inn.T @ np.linalg.solve(S, inn))
                if md2 > mahalanobis_gate:
                    return False
            except np.linalg.LinAlgError:
                warnings.warn("Mahalanobis gate skipped (ill-conditioned S).")

        try:
            K = np.linalg.solve(S.T, (P @ H.T).T).T
        except np.linalg.LinAlgError:
            warnings.warn("Singular innovation covariance; skipping update.")
            return False

        dx = (K @ inn).flatten()
        if threshold is not None:
            for i in range(len(dx)):
                dx[i] = clip(dx[i], threshold[i])

        self.x = (x + dx).reshape(-1, 1)
        for i in range(3, 6):
            self.x[i, 0] = ssa(self.x[i, 0])

        I = np.eye(self.n_states)
        KH = K @ H
        self.P = (I - KH) @ P @ (I - KH).T + K @ R @ K.T

        if np.any(np.isnan(self.P)):
            warnings.warn("NaN in P after update; resetting covariance.")
            self.P = np.array(self.P0, copy=True)

        return True
