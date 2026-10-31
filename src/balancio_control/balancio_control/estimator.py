"""State estimation for the twsbr model from IMU + wheel encoders.

state = [x, theta, psi, dx, dtheta, dpsi]

  theta, psi        pitch / yaw from the IMU orientation (ZYX Euler, yaw unwrapped)
  dtheta, dpsi      Euler-angle rates from the IMU gyro (body rates)
  x, dx             wheel odometry. Joint angles are measured relative to the
                    body, so the absolute wheel angle is phi + theta and
                    x = r * (mean(phi) + theta).

Also dead-reckons the planar pose (X, Y) by integrating dx along psi.
"""

from __future__ import annotations

import math

import numpy as np


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float]:
    """ZYX Euler angles (roll, pitch, yaw)."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class TwsbrEstimator:
    def __init__(self, wheel_radius: float) -> None:
        self.r = wheel_radius
        self.have_imu = False
        self.have_joints = False
        self._roll = self._pitch = self._yaw = 0.0
        self._gyro = (0.0, 0.0, 0.0)
        self._phi = 0.0
        self._dphi = 0.0
        self._psi = 0.0
        self._yaw_last: float | None = None
        self._x_offset: float | None = None
        self._s_last: float | None = None
        self.X = 0.0
        self.Y = 0.0

    @property
    def ready(self) -> bool:
        return self.have_imu and self.have_joints

    def update_imu(
        self, quat: tuple[float, float, float, float], gyro: tuple[float, float, float]
    ) -> None:
        self._roll, self._pitch, self._yaw = quat_to_euler(*quat)
        if self._yaw_last is None:
            self._psi = self._yaw
        else:
            self._psi += wrap_angle(self._yaw - self._yaw_last)
        self._yaw_last = self._yaw
        self._gyro = gyro
        self.have_imu = True

    def update_wheels(
        self, phi_left: float, phi_right: float, dphi_left: float, dphi_right: float
    ) -> None:
        self._phi = 0.5 * (phi_left + phi_right)
        self._dphi = 0.5 * (dphi_left + dphi_right)
        self.have_joints = True

    @property
    def roll(self) -> float:
        return self._roll

    def state(self) -> np.ndarray:
        theta = self._pitch
        phi_r = self._roll
        _, q, r = self._gyro
        dtheta = q * math.cos(phi_r) - r * math.sin(phi_r)
        cos_t = math.cos(theta)
        dpsi = (q * math.sin(phi_r) + r * math.cos(phi_r)) / max(cos_t, 1e-3)

        s = self.r * (self._phi + theta)
        if self._x_offset is None:
            self._x_offset = s
        x = s - self._x_offset
        dx = self.r * (self._dphi + dtheta)

        # Planar dead reckoning
        if self._s_last is not None:
            ds = s - self._s_last
            self.X += ds * math.cos(self._psi)
            self.Y += ds * math.sin(self._psi)
        self._s_last = s

        return np.array([x, theta, self._psi, dx, dtheta, dpsi])
