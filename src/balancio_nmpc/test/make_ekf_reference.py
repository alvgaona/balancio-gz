"""Write a replayable run of the Python ImuEkf for test_ekf.cpp.

usage: make_ekf_reference.py <balancio.yaml> <out.csv>

A plant (the model itself, fine RK4 steps) is held upright by an LQR on its
true state plus a wandering drive command. Its IMU is sampled at ~200 Hz and
the controller runs at ~50 Hz, both with jitter, so input changes fall
between IMU samples. The yaw starts near pi to exercise the unwrapping, and
the filter is reset once mid-run.

Each row is an event, padded to 9 numbers after the kind:
  0 imu    t, qx, qy, qz, qw, gx, gy, gz
  1 input  t, uL, uR
  2 step   t, then the expected x (6), X, Y
  3 reset
"""

import math
import sys

import numpy as np
from balancio_control.estimator import ImuEkf, make_rk4_step
from balancio_control.model import NU, NX, TwsbrParams

DT = 0.02
T_END = 6.0
PLANT_STEP = 0.0005

p = TwsbrParams.from_yaml(sys.argv[1])
F, A = make_rk4_step(p)


def f(x, u, h):
    return np.asarray(F(x, u, h)).ravel()


# Discrete LQR at the upright equilibrium, for the plant's controller only.
x0 = np.zeros(NX)
u0 = np.zeros(NU)
Ad = np.asarray(A(x0, u0, DT))
eps = 1e-6
Bd = np.column_stack(
    [
        (f(x0, u0 + eps * e, DT) - f(x0, u0 - eps * e, DT)) / (2 * eps)
        for e in np.eye(NU)
    ]
)
Q = np.diag([40.0, 100.0, 10.0, 4.0, 0.5, 0.5])
R = np.diag([0.1, 0.1])
Pr = Q.copy()
for _ in range(5000):
    K = np.linalg.solve(R + Bd.T @ Pr @ Bd, Bd.T @ Pr @ Ad)
    Pr = Q + Ad.T @ Pr @ (Ad - Bd @ K)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


rng = np.random.default_rng(1)
x = np.array([0.0, 0.05, 3.0, 0.0, 0.0, 0.0])
u = np.zeros(NU)
ekf = ImuEkf(p, DT)
rows = []


def row(kind, *vals):
    rows.append([kind, *vals] + [0.0] * (9 - len(vals)))


t = 0.0
t_imu = 0.0
t_ctrl = 0.003
x_ref = 0.0
reset_done = False
while t < T_END:
    t_next = min(t_imu, t_ctrl)
    while t < t_next - 1e-12:
        h = min(PLANT_STEP, t_next - t)
        x = f(x, u, h)
        t += h
    if t_imu <= t_ctrl:
        roll = 0.02 * math.sin(3.0 * t) + rng.normal(0, 2e-3)
        qx, qy, qz, qw = quat(
            roll, x[1] + rng.normal(0, 3e-3), x[2] + rng.normal(0, 1e-2)
        )
        # Body rates of the true motion (roll rate ignored), plus noise.
        gq = x[4] * math.cos(roll) + x[5] * math.cos(x[1]) * math.sin(roll)
        gr = -x[4] * math.sin(roll) + x[5] * math.cos(x[1]) * math.cos(roll)
        gyro = (rng.normal(0, 5e-3), gq + rng.normal(0, 5e-3), gr + rng.normal(0, 5e-3))
        ekf.update_imu((qx, qy, qz, qw), gyro, t)
        row(0, t, qx, qy, qz, qw, *gyro)
        t_imu += 0.005 + rng.uniform(-1e-3, 1e-3)
    else:
        if not reset_done and t > 3.0:
            ekf.reset()
            row(3)
            reset_done = True
        xe = ekf.step(t)
        row(2, t, *xe, ekf.X, ekf.Y)
        v = 0.3 * math.sin(0.8 * t)
        w = 0.6 * math.cos(0.5 * t)
        x_ref += v * DT
        ref = np.array([x_ref, 0.0, x[2], v, 0.0, w])
        e = x - ref
        e[2] = 0.0
        u = np.clip(-K @ e, -1.0, 1.0)
        ekf.set_input(t, u)
        row(1, t, *u)
        t_ctrl += DT + rng.uniform(-4e-3, 4e-3)

assert abs(x[1]) < 0.3, f"plant fell: theta={x[1]}"
np.savetxt(sys.argv[2], np.array(rows), delimiter=",", fmt="%.17g")
print(f"{len(rows)} events, final yaw {x[2]:.2f}, x {x[0]:.2f}")
