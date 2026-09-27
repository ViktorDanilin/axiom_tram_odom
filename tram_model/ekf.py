import math
from dataclasses import dataclass

import numpy as np

IX, IY, IYAW, IKAPPA, IV, IA = range(6)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pure_pursuit_curvature(x: float, y: float, yaw: float, tx: float, ty: float) -> float:

    dx, dy = tx - x, ty - y
    l2 = dx * dx + dy * dy
    if l2 < 1e-6:
        return 0.0
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return 2.0 * lateral / l2


# параметры
@dataclass
class EkfParams:

    q_pos: float = 0.01
    q_yaw: float = 1e-4
    q_kappa: float = 1e-6
    q_v: float = 0.05
    q_a: float = 0.5

    tau_kappa: float = 15.0
    tau_a: float = 2.0

    p0_pos: float = 1e4
    p0_yaw: float = 10.0
    p0_kappa: float = 1e-4
    p0_v: float = 1.0
    p0_a: float = 1.0

    gate_pos: float = 13.8


class PoseSpeedEkf:
    def __init__(self, params: EkfParams | None = None):
        self.p = params or EkfParams()
        self.x = np.zeros(6)
        self.P = np.diag([self.p.p0_pos, self.p.p0_pos, self.p.p0_yaw,
                          self.p.p0_kappa, self.p.p0_v, self.p.p0_a])

    @property
    def position(self) -> tuple[float, float]:
        return float(self.x[IX]), float(self.x[IY])

    @property
    def yaw(self) -> float:
        return float(self.x[IYAW])

    @property
    def kappa(self) -> float:
        return float(self.x[IKAPPA])

    @property
    def speed(self) -> float:
        return max(float(self.x[IV]), 0.0)

    @property
    def accel(self) -> float:
        return float(self.x[IA])

    def snap_pose(self, x: float, y: float, yaw: float, kappa: float) -> None:

        self.x[IX], self.x[IY] = x, y
        self.x[IYAW] = wrap_angle(yaw)
        self.x[IKAPPA] = kappa

    def reset_pose(self, x: float, y: float, yaw: float,
                   sigma_pos: float = 1.0, sigma_yaw: float = 0.05) -> None:

        self.x[IX], self.x[IY], self.x[IYAW] = x, y, wrap_angle(yaw)
        self.x[IKAPPA] = 0.0
        for i in (IX, IY, IYAW, IKAPPA):
            self.P[i, :] = 0.0
            self.P[:, i] = 0.0
        self.P[IX, IX] = self.P[IY, IY] = sigma_pos**2
        self.P[IYAW, IYAW] = sigma_yaw**2
        self.P[IKAPPA, IKAPPA] = self.p.p0_kappa

    # прогноз
    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        px, py, yaw, kappa, v, a = self.x
        dk = math.exp(-dt / self.p.tau_kappa)
        da = math.exp(-dt / self.p.tau_a)
        v_mid = max(v + 0.5 * a * dt, 0.0)
        yaw_mid = yaw + 0.5 * v_mid * kappa * dt
        c, s = math.cos(yaw_mid), math.sin(yaw_mid)

        self.x = np.array([
            px + v_mid * c * dt,
            py + v_mid * s * dt,
            wrap_angle(yaw + v_mid * kappa * dt),
            kappa * dk,
            max(v + a * dt, 0.0),
            a * da,
        ])

        F = np.eye(6)
        F[IX, IYAW] = -v_mid * s * dt
        F[IX, IV] = c * dt
        F[IX, IA] = 0.5 * c * dt * dt
        F[IY, IYAW] = v_mid * c * dt
        F[IY, IV] = s * dt
        F[IY, IA] = 0.5 * s * dt * dt
        F[IYAW, IKAPPA] = v_mid * dt
        F[IYAW, IV] = kappa * dt
        F[IKAPPA, IKAPPA] = dk
        F[IV, IA] = dt
        F[IA, IA] = da

        Q = np.diag([self.p.q_pos, self.p.q_pos, self.p.q_yaw,
                     self.p.q_kappa, self.p.q_v, self.p.q_a]) * dt
        self.P = F @ self.P @ F.T + Q

    # коррекция
    def _update(self, innovation: np.ndarray, H: np.ndarray, R: np.ndarray,
                gate: float | None = None) -> bool:
        S = H @ self.P @ H.T + R
        if gate is not None:
            d2 = float(innovation @ np.linalg.solve(S, innovation))
            if d2 > gate:
                return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[IYAW] = wrap_angle(self.x[IYAW])
        self.x[IV] = max(self.x[IV], 0.0)
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True

    def update_speed(self, v: float, sigma: float) -> None:
        H = np.zeros((1, 6))
        H[0, IV] = 1.0
        self._update(np.array([v - self.x[IV]]), H, np.array([[sigma**2]]))

    def update_accel(self, a: float, sigma: float) -> None:
        H = np.zeros((1, 6))
        H[0, IA] = 1.0
        self._update(np.array([a - self.x[IA]]), H, np.array([[sigma**2]]))

    def update_position(self, x: float, y: float, sigma: float, gated: bool = True) -> bool:
        H = np.zeros((2, 6))
        H[0, IX] = H[1, IY] = 1.0
        innovation = np.array([x - self.x[IX], y - self.x[IY]])
        R = np.eye(2) * sigma**2
        return self._update(innovation, H, R, self.p.gate_pos if gated else None)

    def update_position_along_track(self, x: float, y: float, track_yaw: float,
                                    sigma_along: float, sigma_lateral: float,
                                    gated: bool = True) -> bool:

        c, s = math.cos(track_yaw), math.sin(track_yaw)
        rot = np.array([[c, -s], [s, c]])
        R = rot @ np.diag([sigma_along**2, sigma_lateral**2]) @ rot.T
        H = np.zeros((2, 6))
        H[0, IX] = H[1, IY] = 1.0
        innovation = np.array([x - self.x[IX], y - self.x[IY]])
        return self._update(innovation, H, R, self.p.gate_pos if gated else None)

    def update_heading(self, yaw: float, sigma: float) -> None:
        H = np.zeros((1, 6))
        H[0, IYAW] = 1.0
        self._update(np.array([wrap_angle(yaw - self.x[IYAW])]), H, np.array([[sigma**2]]))

    def update_curvature(self, kappa: float, sigma: float) -> None:
        H = np.zeros((1, 6))
        H[0, IKAPPA] = 1.0
        self._update(np.array([kappa - self.x[IKAPPA]]), H, np.array([[sigma**2]]))

    def update_pure_pursuit(self, target_x: float, target_y: float, sigma: float) -> float:

        kappa = pure_pursuit_curvature(self.x[IX], self.x[IY], self.x[IYAW],
                                       target_x, target_y)
        self.update_curvature(kappa, sigma)
        return kappa
