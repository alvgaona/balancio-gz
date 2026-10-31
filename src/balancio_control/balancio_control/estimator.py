"""State estimation for the twsbr model from the IMU alone (no wheel encoders).

state = [x, theta, psi, dx, dtheta, dpsi]

An extended Kalman filter runs the same model as the NMPC (robot + DC
motors), driven by the duties the controller sent, and corrects it with the
IMU:

  theta, psi        pitch / yaw from the IMU orientation (ZYX Euler, yaw unwrapped)
  dtheta, dpsi      Euler-angle rates from the IMU gyro (body rates)

dx is not measured. It shows up through the motors: for a given duty, the
wheel torque falls with wheel speed (back-EMF), and that changes how the
body pitches. x is the integral of dx and drifts.

Everything is timestamped: the model is run up to the IMU sample's own time,
applying each duty from the moment it was sent, so an irregular control loop
(a slow solve delays the next step) doesn't turn into a speed error.

Also dead-reckons the planar pose (X, Y) by integrating dx along psi.
"""

from __future__ import annotations

import math

import casadi as ca
import numpy as np

from balancio_control.model import NU, NX, TwsbrParams, make_dynamics_function

# Measured state components: theta, psi, dtheta, dpsi
H = np.zeros((4, NX))
H[[0, 1, 2, 3], [1, 2, 4, 5]] = 1.0


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


def make_rk4_step(p: TwsbrParams) -> tuple[ca.Function, ca.Function]:
    """One RK4 step x(t + h) = F(x, u, h) and its Jacobian dF/dx."""
    f = make_dynamics_function(p)
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    h = ca.SX.sym("h")
    k1 = f(x, u)
    k2 = f(x + h / 2 * k1, u)
    k3 = f(x + h / 2 * k2, u)
    k4 = f(x + h * k3, u)
    xn = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return (
        ca.Function("F", [x, u, h], [xn]),
        ca.Function("A", [x, u, h], [ca.jacobian(xn, x)]),
    )


class ImuEkf:
    MAX_STEP = 0.005  # longest model step [s]

    def __init__(
        self,
        params: TwsbrParams,
        dt: float,
        q: list[float] | None = None,
        r: list[float] | None = None,
    ) -> None:
        """dt: nominal control period. q: process noise std per dt.
        r: IMU noise std [theta, psi, dtheta, dpsi]."""
        self.p = params
        self.dt = dt
        self._F, self._A = make_rk4_step(params)
        q = q or [1e-4, 1e-3, 1e-3, 1e-3, 5e-2, 2e-2]
        r = r or [3e-3, 1e-2, 5e-3, 5e-3]
        self.Q = np.diag(np.square(q))
        self.R = np.diag(np.square(r))

        self.have_imu = False
        self._roll = 0.0
        self._z = np.zeros(4)  # theta, psi, dtheta, dpsi
        self._z_time = 0.0
        self._psi = 0.0
        self._yaw_last: float | None = None
        self.x: np.ndarray | None = None  # filter state at time self.t
        self.t = 0.0
        self.P = np.zeros((NX, NX))
        # Duties sent, (time, u), each applied until the next one.
        self._inputs: list[tuple[float, np.ndarray]] = []
        self._t_out: float | None = None
        self.X = 0.0
        self.Y = 0.0

    @property
    def ready(self) -> bool:
        return self.have_imu

    @property
    def roll(self) -> float:
        return self._roll

    def update_imu(
        self,
        quat: tuple[float, float, float, float],
        gyro: tuple[float, float, float],
        stamp: float,
    ) -> None:
        roll, pitch, yaw = quat_to_euler(*quat)
        if self._yaw_last is None:
            self._psi = yaw
        else:
            self._psi += wrap_angle(yaw - self._yaw_last)
        self._yaw_last = yaw
        _, q, r = gyro
        dtheta = q * math.cos(roll) - r * math.sin(roll)
        dpsi = (q * math.sin(roll) + r * math.cos(roll)) / max(math.cos(pitch), 1e-3)
        self._roll = roll
        self._z = np.array([pitch, self._psi, dtheta, dpsi])
        self._z_time = stamp
        self.have_imu = True

    def set_input(self, stamp: float, u: np.ndarray) -> None:
        """Record a duty sent to the motors at time stamp."""
        self._inputs.append((stamp, np.asarray(u, dtype=float).copy()))

    def reset(self) -> None:
        """Restart from the latest IMU sample with the robot at rest."""
        x0 = self.x[0] if self.x is not None else 0.0
        self.x = np.array([x0, *self._z[:2], 0.0, *self._z[2:]])
        self.t = self._z_time
        self.P = np.diag([0.0, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2])

    def _segments(self, t0: float, t1: float):
        """(u, h) pieces of constant duty covering [t0, t1], h <= MAX_STEP."""
        u = np.zeros(NU)
        for t, ui in self._inputs:
            if t <= t0:
                u = ui
        times = [t for t, _ in self._inputs if t0 < t < t1] + [t1]
        a = t0
        for b in times:
            n = max(1, math.ceil((b - a) / self.MAX_STEP - 1e-9))
            for _ in range(n):
                yield u, (b - a) / n
            a = b
            u = next((ui for t, ui in self._inputs if t == b), u)

    def _propagate(self, x, P, t0: float, t1: float):
        for u, h in self._segments(t0, t1):
            if P is not None:
                A = np.asarray(self._A(x, u, h))
                P = A @ P @ A.T + self.Q * (h / self.dt)
            x = np.asarray(self._F(x, u, h)).ravel()
        return x, P

    def step(self, now: float) -> np.ndarray:
        """Fuse the latest IMU sample and return the state predicted at now."""
        if self.x is None:
            self.reset()
        elif self._z_time > self.t:
            self.x, self.P = self._propagate(self.x, self.P, self.t, self._z_time)
            self.t = self._z_time
            S = H @ self.P @ H.T + self.R
            K = np.linalg.solve(S, H @ self.P).T
            self.x = self.x + K @ (self._z - H @ self.x)
            I_KH = np.eye(NX) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T  # Joseph form
        # Forget duties that ended before self.t (keep the one in force).
        while len(self._inputs) > 1 and self._inputs[1][0] <= self.t:
            self._inputs.pop(0)

        x_now, _ = self._propagate(self.x, None, self.t, max(now, self.t))

        # Planar dead reckoning
        if self._t_out is not None:
            ds = x_now[3] * (now - self._t_out)
            self.X += ds * math.cos(x_now[2])
            self.Y += ds * math.sin(x_now[2])
        self._t_out = now
        return x_now
