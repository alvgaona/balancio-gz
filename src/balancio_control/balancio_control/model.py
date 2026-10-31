"""Two-wheeled self-balancing robot dynamics.

Direct port of alvgaona/twsbr `matlab/twsbr_dynamics.m`:

    M(q) q_ddot + C(q, q_dot) q_dot + D q_dot + G(q) = B u

    q = [x, theta, psi]      forward distance, pitch, yaw
    u = [duty_L, duty_R]     motor PWM duty in [-1, 1]
    state = [x, theta, psi, dx, dtheta, dpsi]

Added to twsbr: the wheels are driven by DC gear motors from a fixed battery
voltage, so the wheel torque is

    tau = max_torque * (duty - w / motor_no_load_speed)

with w the wheel speed relative to the body, and the motor rotor inertia
(reflected to the wheel, motor_inertia) spins with w.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import casadi as ca
import yaml

NX = 6
NU = 2


@dataclass(frozen=True)
class TwsbrParams:
    d: float
    l: float
    r: float
    mb: float
    mw: float
    J: float
    K: float
    I1: float
    I2: float
    I3: float
    calpha: float
    max_torque: float  # stall torque at full duty [N m]
    motor_no_load_speed: float  # [rad/s]
    motor_inertia: float  # rotor inertia at the wheel [kg m^2]
    g: float = 9.81

    @classmethod
    def from_yaml(cls, path: str) -> TwsbrParams:
        with open(path) as f:
            data = yaml.safe_load(f)
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in data.items() if k in names})


def motor_torques(state, u, p: TwsbrParams):
    """Wheel torques [tau_L, tau_R] for duty u (no clipping)."""
    w_l, w_r = wheel_speeds(state, p)
    k = p.max_torque / p.motor_no_load_speed
    return p.max_torque * u[0] - k * w_l, p.max_torque * u[1] - k * w_r


def continuous_dynamics(state: ca.SX, u: ca.SX, p: TwsbrParams) -> ca.SX:
    """State derivative for duty u, with CasADi symbols (works with SX, MX or DM)."""
    theta = state[1]
    dx = state[3]
    dtheta = state[4]
    dpsi = state[5]

    s = ca.sin(theta)
    c = ca.cos(theta)
    k = p.I3 - p.I1 - p.mb * p.l**2

    # Mass matrix. The rotors add Jm (dx / r - dtheta)^2 + Jm (d dpsi / 2 r)^2
    # to the kinetic energy (two motors, relative wheel speeds).
    jm = p.motor_inertia
    a11 = p.mb + 2 * p.mw + 2 * p.J / p.r**2 + 2 * jm / p.r**2
    a12 = p.mb * p.l * c - 2 * jm / p.r
    a22 = p.I2 + p.mb * p.l**2 + 2 * jm
    a33 = (
        p.I3
        + 2 * p.K
        + p.mw * p.d**2 / 2
        + (p.J + jm) * p.d**2 / (2 * p.r**2)
        - k * s**2
    )

    # Coriolis matrix
    c12 = -p.mb * p.l * dtheta * s
    c13 = -p.mb * p.l * dpsi * s
    c23 = k * dpsi * s * c
    c31 = p.mb * p.l * dpsi * s
    c32 = -k * dpsi * s * c
    c33 = -k * dtheta * s * c

    # Damping matrix
    d11 = 2 * p.calpha / p.r**2
    d12 = -2 * p.calpha / p.r
    d22 = 2 * p.calpha
    d33 = p.calpha * p.d**2 / (2 * p.r**2)

    tau_l, tau_r = motor_torques(state, u, p)

    # Right-hand side: B u - C q_dot - D q_dot - G
    f1 = (tau_l + tau_r) / p.r - (c12 * dtheta + c13 * dpsi) - (d11 * dx + d12 * dtheta)
    f2 = (
        -(tau_l + tau_r) - c23 * dpsi - (d12 * dx + d22 * dtheta) + p.mb * p.l * p.g * s
    )
    f3 = (
        p.d / (2 * p.r) * (tau_r - tau_l)
        - (c31 * dx + c32 * dtheta + c33 * dpsi)
        - d33 * dpsi
    )

    # M is block diagonal: 2x2 (x, theta) and scalar (psi)
    det = a11 * a22 - a12 * a12
    ddx = (a22 * f1 - a12 * f2) / det
    ddtheta = (-a12 * f1 + a11 * f2) / det
    ddpsi = f3 / a33

    return ca.vertcat(dx, dtheta, dpsi, ddx, ddtheta, ddpsi)


def wheel_speeds(state, p: TwsbrParams):
    """Wheel speeds relative to the body (= motor shaft speeds) [rad/s].

    phi_dot_L = (dx - d/2 dpsi) / r - dtheta,  phi_dot_R = (dx + d/2 dpsi) / r - dtheta
    """
    dx, dtheta, dpsi = state[3], state[4], state[5]
    w_l = (dx - p.d / 2 * dpsi) / p.r - dtheta
    w_r = (dx + p.d / 2 * dpsi) / p.r - dtheta
    return w_l, w_r


def make_dynamics_function(p: TwsbrParams) -> ca.Function:
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    return ca.Function("f", [x, u], [continuous_dynamics(x, u, p)], ["x", "u"], ["dx"])


def make_rk4_function(p: TwsbrParams, dt: float, substeps: int = 1) -> ca.Function:
    """Discrete-time model x_{k+1} = F(x_k, u_k) with zero-order-hold input."""
    f = make_dynamics_function(p)
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    h = dt / substeps
    xn = x
    for _ in range(substeps):
        k1 = f(xn, u)
        k2 = f(xn + h / 2 * k1, u)
        k3 = f(xn + h / 2 * k2, u)
        k4 = f(xn + h * k3, u)
        xn = xn + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return ca.Function("F", [x, u], [xn], ["x", "u"], ["x_next"])
