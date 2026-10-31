"""Closed-loop checks of the NMPC against the twsbr model (no ROS/Gazebo)."""

import math
import os

import numpy as np
import pytest
from balancio_control.estimator import ImuEkf
from balancio_control.model import (
    TwsbrParams,
    make_dynamics_function,
    make_rk4_function,
)
from balancio_control.nmpc import Nmpc, NmpcConfig

HERE = os.path.dirname(__file__)
PARAMS = os.path.join(
    HERE, "..", "..", "balancio_description", "config", "balancio.yaml"
)


@pytest.fixture(scope="module")
def params() -> TwsbrParams:
    return TwsbrParams.from_yaml(PARAMS)


def test_upright_is_equilibrium_and_unstable(params):
    f = make_dynamics_function(params)
    assert np.allclose(np.asarray(f(np.zeros(6), np.zeros(2))).ravel(), 0.0)
    # Leaning forward with no torque accelerates the fall.
    dx = np.asarray(f(np.array([0, 0.1, 0, 0, 0, 0]), np.zeros(2))).ravel()
    assert dx[4] > 0.0


def test_duty_signs(params):
    f = make_dynamics_function(params)
    duty = 0.5
    fwd = np.asarray(f(np.zeros(6), np.array([duty, duty]))).ravel()
    assert fwd[3] > 0.0 and fwd[4] < 0.0  # wheels forward, body pitched back
    turn = np.asarray(f(np.zeros(6), np.array([-duty, duty]))).ravel()
    assert turn[5] > 0.0  # right wheel forward -> yaw left (CCW)


def test_back_emf_limits_speed(params):
    """Full duty, held upright: back-EMF and axle friction balance the motor."""
    F = make_rk4_function(params, 0.001)
    x = np.zeros(6)
    for _ in range(3000):
        x = np.asarray(F(x, np.ones(2))).ravel()
        x[1] = x[4] = 0.0  # hold the body upright
    w = x[3] / params.r
    k = params.max_torque / params.motor_no_load_speed
    assert w == pytest.approx(params.max_torque / (k + params.calpha), rel=0.01)


def mismatched(params: TwsbrParams) -> TwsbrParams:
    """+-10 % on every parameter, as in twsbr_nmpc.m"""
    return TwsbrParams(
        **{
            k: v * (1.1 if i % 2 else 0.9)
            for i, (k, v) in enumerate(vars(params).items())
            if k != "g"
        },
        g=params.g,
    )


@pytest.mark.parametrize("solver", ["fatrop", "ipopt"])
def test_closed_loop_recovery_and_tracking(params, solver):
    cfg = NmpcConfig(solver=solver)
    mpc = Nmpc(params, cfg)
    plant = make_rk4_function(mismatched(params), cfg.dt, substeps=20)

    x = np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0])
    u = np.zeros(2)
    x_ref = psi_ref = 0.0
    N = cfg.horizon
    k = np.arange(N + 1) * cfg.dt
    for step in range(int(8.0 / cfg.dt)):
        t = step * cfg.dt
        v, w = (0.3, 0.5) if t > 3.0 else (0.0, 0.0)
        ref = np.zeros((6, N + 1))
        ref[0], ref[2], ref[3], ref[5] = x_ref + v * k, psi_ref + w * k, v, w
        sol = mpc.solve(x, ref, u)
        assert sol.success, sol.status
        assert np.all(np.abs(sol.u) <= cfg.max_duty + 1e-9)
        u = sol.u
        x = np.asarray(plant(x, u)).ravel()
        x_ref += v * cfg.dt
        psi_ref += w * cfg.dt
        assert abs(x[1]) < cfg.max_theta + 0.05
        if abs(t - 3.0) < 1e-9:
            assert abs(x[1]) < 0.02 and abs(x[0]) < 0.05  # balanced in place

    assert abs(x[3] - 0.3) < 0.05
    assert abs(x[5] - 0.5) < 0.05
    assert abs(x[1]) < 0.1


def imu_reading(x, rng):
    """Noisy IMU for state x, upright in roll: quaternion and gyro."""
    th = x[1] + rng.normal(0.0, 0.003)
    psi = x[2] + rng.normal(0.0, 0.01)
    q = (
        -math.sin(th / 2) * math.sin(psi / 2),
        math.sin(th / 2) * math.cos(psi / 2),
        math.cos(th / 2) * math.sin(psi / 2),
        math.cos(th / 2) * math.cos(psi / 2),
    )
    gyro = (
        0.0,
        x[4] + rng.normal(0.0, 0.005),
        x[5] * math.cos(x[1]) + rng.normal(0.0, 0.005),
    )
    return q, gyro


def test_closed_loop_imu_only(params):
    """Balance and track speed from a noisy IMU, with a mismatched plant."""
    cfg = NmpcConfig()
    mpc = Nmpc(params, cfg)
    est = ImuEkf(params, cfg.dt)
    plant = make_rk4_function(mismatched(params), cfg.dt, substeps=20)
    rng = np.random.default_rng(0)

    x = np.array([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
    u = np.zeros(2)
    x_ref = psi_ref = 0.0
    N = cfg.horizon
    k = np.arange(N + 1) * cfg.dt
    dx_err = []
    for step in range(int(10.0 / cfg.dt)):
        t = step * cfg.dt
        v, w = (0.3, 0.5) if t > 4.0 else (0.0, 0.0)
        est.update_imu(*imu_reading(x, rng), t)
        xh = est.step(t)
        if t > 1.0:
            dx_err.append(xh[3] - x[3])
        ref = np.zeros((6, N + 1))
        ref[0], ref[2], ref[3], ref[5] = x_ref + v * k, psi_ref + w * k, v, w
        u = mpc.solve(xh, ref, u).u
        est.set_input(t, u)
        x = np.asarray(plant(x, u)).ravel()
        x_ref += v * cfg.dt
        psi_ref += w * cfg.dt
        assert abs(x[1]) < cfg.max_theta
        if abs(t - 4.0) < 1e-9:
            assert abs(x[1]) < 0.03 and abs(x[3]) < 0.03  # standing still

    assert abs(x[3] - 0.3) < 0.06
    assert abs(x[5] - 0.5) < 0.1
    assert np.sqrt(np.mean(np.square(dx_err))) < 0.05
