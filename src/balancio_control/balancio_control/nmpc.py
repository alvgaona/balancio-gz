"""Nonlinear MPC for the two-wheeled self-balancing robot (CasADi + IPOPT).

Python counterpart of the `nlmpc` setup in alvgaona/twsbr `matlab/twsbr_nmpc.m`:
same states, inputs, stage cost (Q, R, input-rate weight) and constraints
(torque, torque rate, pitch). Differences:

  * Direct multiple shooting with RK4 instead of a single Euler step.
  * Terminal cost from the discrete LQR of the model linearized upright.
  * A per-stage reference over the horizon (reference preview).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from balancio_control.model import NU, NX, TwsbrParams, make_rk4_function, wheel_speeds


@dataclass
class NmpcConfig:
    dt: float = 0.02
    horizon: int = 25
    # State weights: [x, theta, psi, dx, dtheta, dpsi]
    q: list[float] = field(default_factory=lambda: [40.0, 100.0, 10.0, 4.0, 0.5, 0.5])
    r: list[float] = field(default_factory=lambda: [16.0, 16.0])
    r_rate: list[float] = field(default_factory=lambda: [1.0, 1.0])
    max_torque: float = 0.25  # stall torque per wheel [N m]
    # DC motor back-EMF: available torque falls linearly to zero at the no-load
    # speed, tau in [-stall (1 + w / w0), stall (1 - w / w0)]. <= 0 disables.
    motor_no_load_speed: float = 0.0  # [rad/s]
    max_torque_rate: float = 25.0  # [N m / s]
    max_theta: float = 0.6  # [rad]
    max_iter: int = 100
    terminal_lqr: bool = True
    solver: str = "fatrop"  # "fatrop" (fast, structure-exploiting) or "ipopt"


def dlqr_terminal_cost(
    F: ca.Function, Q: np.ndarray, R: np.ndarray, iters: int = 2000
) -> np.ndarray:
    """Solve the discrete Riccati equation at the upright equilibrium."""
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    jac = ca.Function(
        "linearize", [x, u], [ca.jacobian(F(x, u), x), ca.jacobian(F(x, u), u)]
    )
    A_dm, B_dm = jac(np.zeros(NX), np.zeros(NU))
    A, B = np.asarray(A_dm), np.asarray(B_dm)
    P = Q.copy()
    for _ in range(iters):
        BtP = B.T @ P
        P_next = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(R + BtP @ B, BtP @ A)
        if np.max(np.abs(P_next - P)) < 1e-9 * max(1.0, np.max(np.abs(P))):
            P = P_next
            break
        P = P_next
    return 0.5 * (P + P.T)


@dataclass
class NmpcSolution:
    u: np.ndarray  # first control move [tau_L, tau_R]
    x_pred: np.ndarray  # (NX, N+1)
    u_pred: np.ndarray  # (NU, N)
    success: bool
    status: str
    iterations: int


class Nmpc:
    """Multiple-shooting NMPC.

    The previous input is carried as an extra state (z = [x; u_last]) so the
    torque-rate constraint is local to each stage. This keeps the NLP in the
    stage-wise OCP form required by fatrop (and is harmless for IPOPT).
    Decision vector layout: [z_0, u_0, z_1, u_1, ..., z_N].
    """

    NZ = NX + NU

    def __init__(self, params: TwsbrParams, cfg: NmpcConfig | None = None) -> None:
        self.p = params
        self.cfg = cfg or NmpcConfig()
        c = self.cfg
        N = c.horizon
        NZ = self.NZ

        F = make_rk4_function(params, c.dt)
        Q = np.diag(c.q)
        R = np.diag(c.r)
        Rd = np.diag(c.r_rate)
        Qf = dlqr_terminal_cost(F, Q, R) if c.terminal_lqr else Q

        Z = [ca.SX.sym(f"z{k}", NZ) for k in range(N + 1)]
        U = [ca.SX.sym(f"u{k}", NU) for k in range(N)]
        z0 = ca.SX.sym("z0", NZ)  # [x0; u_prev]
        xref = ca.SX.sym("xref", NX, N + 1)

        du_max = c.max_torque_rate * c.dt
        cost = 0
        g = [Z[0] - z0]
        lbg = [np.zeros(NZ)]
        ubg = [np.zeros(NZ)]
        eq = [True] * NZ
        w = []
        lbw: list[np.ndarray] = []
        ubw: list[np.ndarray] = []
        inf = np.inf
        z_lb = np.array([-inf, -c.max_theta, -inf, -inf, -inf, -inf, -inf, -inf])
        u_lb = -c.max_torque * np.ones(NU)
        for k in range(N):
            x_k, ul_k = Z[k][:NX], Z[k][NX:]
            e = x_k - xref[:, k]
            du = U[k] - ul_k
            cost += ca.bilin(Q, e, e) + ca.bilin(R, U[k], U[k]) + ca.bilin(Rd, du, du)
            g += [Z[k + 1] - ca.vertcat(F(x_k, U[k]), U[k]), du]
            lbg += [np.zeros(NZ), -du_max * np.ones(NU)]
            ubg += [np.zeros(NZ), du_max * np.ones(NU)]
            eq += [True] * NZ + [False] * NU
            if c.motor_no_load_speed > 0:
                # Back-EMF: -stall <= tau + stall * w / w0 <= stall, per wheel
                w_l, w_r = wheel_speeds(x_k, params)
                k_emf = c.max_torque / c.motor_no_load_speed
                g += [U[k] + k_emf * ca.vertcat(w_l, w_r)]
                lbg += [-c.max_torque * np.ones(NU)]
                ubg += [c.max_torque * np.ones(NU)]
                eq += [False] * NU
            w += [Z[k], U[k]]
            lbw += [z_lb, u_lb]
            ubw += [-z_lb, -u_lb]
        eN = Z[N][:NX] - xref[:, N]
        cost += ca.bilin(Qf, eN, eN)
        w += [Z[N]]
        lbw += [z_lb]
        ubw += [-z_lb]

        nlp = {
            "x": ca.vertcat(*w),
            "f": cost,
            "g": ca.vertcat(*g),
            "p": ca.vertcat(z0, ca.vec(xref)),
        }
        if c.solver == "fatrop":
            opts: dict = {
                "structure_detection": "auto",
                "equality": eq,
                "expand": True,
                "print_time": False,
                "fatrop.print_level": -1,
                "fatrop.max_iter": c.max_iter,
                "fatrop.tol": 1e-6,
                "fatrop.mu_init": 1e-2,
                "fatrop.warm_start_init_point": True,
            }
        elif c.solver == "ipopt":
            opts = {
                "print_time": False,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "ipopt.max_iter": c.max_iter,
                "ipopt.tol": 1e-6,
                "ipopt.acceptable_tol": 1e-4,
                "ipopt.warm_start_init_point": "yes",
                "ipopt.mu_init": 1e-3,
            }
        else:
            raise ValueError(f"unknown solver '{c.solver}' (use 'fatrop' or 'ipopt')")
        self._solver = ca.nlpsol("nmpc", c.solver, nlp, opts)
        self._lbg = np.concatenate(lbg)
        self._ubg = np.concatenate(ubg)
        self._lbw = np.concatenate(lbw)
        self._ubw = np.concatenate(ubw)

        self.N = N
        self._nw = self._lbw.shape[0]
        self._w0: np.ndarray | None = None
        self._lam_x0 = np.zeros(self._nw)
        self._lam_g0 = np.zeros(self._lbg.shape[0])

        # Index helpers to unpack the stage-interleaved decision vector.
        stride = NZ + NU
        self._ix = np.array(
            [k * stride + np.arange(NX) for k in range(N + 1)]
        ).T  # (NX, N+1)
        self._iu = np.array(
            [k * stride + NZ + np.arange(NU) for k in range(N)]
        ).T  # (NU, N)
        self._iz = np.array([k * stride + np.arange(NZ) for k in range(N + 1)]).T

    def reset(self) -> None:
        self._w0 = None
        self._lam_x0[:] = 0.0
        self._lam_g0[:] = 0.0

    def _initial_guess(self, z0: np.ndarray) -> np.ndarray:
        if self._w0 is None:
            w = np.zeros(self._nw)
            w[self._iz] = z0.reshape(-1, 1)
            return w
        # Shift the previous solution by one stage.
        w_prev = self._w0
        Zp = w_prev[self._iz]
        Up = w_prev[self._iu]
        Z = np.hstack([Zp[:, 1:], Zp[:, -1:]])
        Uq = np.hstack([Up[:, 1:], Up[:, -1:]])
        Z[:, 0] = z0
        w = np.empty(self._nw)
        w[self._iz] = Z
        w[self._iu] = Uq
        return w

    def solve(
        self, x0: np.ndarray, xref: np.ndarray, u_prev: np.ndarray
    ) -> NmpcSolution:
        """x0: (NX,), xref: (NX,) or (NX, N+1), u_prev: (NU,)."""
        x0 = np.asarray(x0, dtype=float).ravel()
        xref = np.asarray(xref, dtype=float)
        if xref.ndim == 1:
            xref = np.tile(xref.reshape(-1, 1), (1, self.N + 1))
        z0 = np.concatenate([x0, np.asarray(u_prev, dtype=float).ravel()])
        p = np.concatenate([z0, xref.ravel(order="F")])

        sol = self._solver(
            x0=self._initial_guess(z0),
            lam_x0=self._lam_x0,
            lam_g0=self._lam_g0,
            lbx=self._lbw,
            ubx=self._ubw,
            lbg=self._lbg,
            ubg=self._ubg,
            p=p,
        )
        stats = self._solver.stats()
        w = np.asarray(sol["x"]).ravel()
        success = bool(stats.get("success", False)) and bool(np.all(np.isfinite(w)))
        if success:
            self._w0 = w
            self._lam_x0 = np.asarray(sol["lam_x"]).ravel()
            self._lam_g0 = np.asarray(sol["lam_g"]).ravel()
        else:
            self.reset()

        X = w[self._ix]
        U = w[self._iu]
        u0 = np.nan_to_num(U[:, 0])
        u0 = np.clip(u0, -self.cfg.max_torque, self.cfg.max_torque)
        return NmpcSolution(
            u=u0,
            x_pred=X,
            u_pred=U,
            success=success,
            status=str(stats.get("return_status", "")),
            iterations=int(stats.get("iter_count", -1)),
        )
