"""Two-wheeled self-balancing robot dynamics.

Direct port of alvgaona/twsbr `matlab/twsbr_dynamics.m`:

    M(q) q_ddot + C(q, q_dot) q_dot + D q_dot + G(q) = B u

    q = [x, theta, psi]      forward distance, pitch, yaw
    u = [tau_L, tau_R]       wheel torques
    state = [x, theta, psi, dx, dtheta, dpsi]
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
    g: float = 9.81

    @classmethod
    def from_yaml(cls, path: str) -> TwsbrParams:
        with open(path) as f:
            data = yaml.safe_load(f)
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in data.items() if k in names})


def continuous_dynamics(state: ca.SX, u: ca.SX, p: TwsbrParams) -> ca.SX:
    """State derivative, built with CasADi symbols (works with SX, MX or DM)."""
    theta = state[1]
    dx = state[3]
    dtheta = state[4]
    dpsi = state[5]

    s = ca.sin(theta)
    c = ca.cos(theta)
    k = p.I3 - p.I1 - p.mb * p.l**2

    # Mass matrix
    a11 = p.mb + 2 * p.mw + 2 * p.J / p.r**2
    a12 = p.mb * p.l * c
    a22 = p.I2 + p.mb * p.l**2
    a33 = p.I3 + 2 * p.K + p.mw * p.d**2 / 2 + p.J * p.d**2 / (2 * p.r**2) - k * s**2

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

    tau_l = u[0]
    tau_r = u[1]

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
