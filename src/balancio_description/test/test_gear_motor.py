"""Checks of the gear motor model used by the Gazebo plugin (no Gazebo)."""

import math

import pytest
from balancio_description.gear_motor import GearMotor, GearMotorParams

DT = 0.001  # Gazebo physics step
P = GearMotorParams(stall=0.078, no_load_speed=20.9, inertia=1.4e-4)
J_WHEEL = 2.4e-5


def spin(motor: GearMotor, duty, t: float, inertia: float = J_WHEEL):
    """Drive a free wheel of the given inertia, as Gazebo would. Yields per step."""
    phi = dphi = 0.0
    for _ in range(round(t / DT)):
        tau_c, _ = motor.step(duty, phi, dphi, DT)
        dphi += DT * tau_c / inertia
        phi += DT * dphi
        yield phi, dphi, tau_c


def test_stall_torque():
    m = GearMotor(P)
    for _ in range(500):
        tau_c, _ = m.step(1.0, 0.0, 0.0, DT)
    assert tau_c == pytest.approx(P.stall, rel=0.01)


def test_no_load_speed():
    *_, (_, dphi, _) = spin(GearMotor(P), 1.0, 2.0)
    assert dphi == pytest.approx(P.no_load_speed, rel=0.01)


def test_matches_rigid_gearbox():
    """Without backlash the spring is stiff enough to act like a rigid gear."""
    j = J_WHEEL + P.inertia
    tau_m = P.no_load_speed * j / P.stall  # mechanical time constant
    *_, (_, dphi, _) = spin(GearMotor(P), 1.0, tau_m)
    rigid = P.no_load_speed * (1.0 - math.exp(-1.0))
    assert dphi == pytest.approx(rigid, rel=0.02)


def test_duty_is_clipped():
    a, b = GearMotor(P), GearMotor(P)
    *_, (_, wa, _) = spin(a, 1.0, 0.5)
    *_, (_, wb, _) = spin(b, 3.0, 0.5)
    assert wa == pytest.approx(wb)


def test_backlash_dead_time():
    """Reversing against a held wheel: no torque while crossing the gap."""
    backlash = math.radians(1.0)
    m = GearMotor(GearMotorParams(**{**vars(P), "backlash": backlash}))
    # Light push first, so the gear spring is barely compressed.
    for _ in range(300):
        tau_c, _ = m.step(0.05, 0.0, 0.0, DT)
    assert tau_c == pytest.approx(0.05 * P.stall, rel=0.01)
    dead = 0
    for _ in range(300):
        tau_c, _ = m.step(-1.0, 0.0, 0.0, DT)
        if tau_c == 0.0:
            dead += 1
        elif dead:
            break
    assert tau_c < 0.0
    # Crossing 2 * backlash from rest at stall torque: t = sqrt(4 b J / stall)
    t_cross = math.sqrt(4.0 * backlash * P.inertia / P.stall)
    assert dead * DT == pytest.approx(t_cross, rel=0.15)
