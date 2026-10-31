"""Voltage-driven DC gear motor with rotor inertia and gearbox backlash.

Plain Python, no Gazebo: the Gazebo plugin (gz_motor.py) runs this every
physics step, and the tests run it on its own.

All quantities are at the gearbox output (the wheel shaft), relative to the
body:

  duty      PWM duty cycle in [-1, 1] (fraction of the battery voltage)
  theta_m   gearbox output angle; the motor side of the backlash gap
  w_m       its speed
  phi, dphi wheel joint angle and speed

Motor (electrical time constant neglected):

  tau_m = stall * (duty - w_m / w0)

Gearbox: the motor side drives the wheel through a gap of +-backlash. Inside
the gap nothing is transmitted. In contact the teeth act as a stiff spring
with damping that can only push:

  gap   = theta_m - phi
  pen   = gap - backlash  (gap > backlash),  gap + backlash  (gap < -backlash)
  tau_c = k * pen + c * (w_m - dphi),  same sign as pen, 0 inside the gap

Rotor (inertia J_m, the motor rotor reflected through the gear ratio):

  J_m * dw_m/dt = tau_m - tau_c

tau_c acts on the wheel and -tau_c on the body. With backlash = 0 the
gearbox is just a stiff spring, and this matches the rigid model in
balancio_control/model.py (rotor kinetic energy 1/2 J_m w^2).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GearMotorParams:
    stall: float  # stall torque at full duty [N m]
    no_load_speed: float  # [rad/s] at full duty
    inertia: float  # rotor inertia at the gearbox output [kg m^2]
    backlash: float = 0.0  # half of the total free play [rad]
    stiffness: float = 10.0  # gear mesh stiffness [N m / rad]
    damping: float = 0.01  # gear mesh damping [N m s / rad]

    @classmethod
    def from_dict(cls, p: dict) -> GearMotorParams:
        return cls(
            stall=float(p["max_torque"]),
            no_load_speed=float(p["motor_no_load_speed"]),
            inertia=float(p["motor_inertia"]),
            backlash=float(p.get("backlash", 0.0)),
            stiffness=float(p.get("gear_stiffness", 10.0)),
            damping=float(p.get("gear_damping", 0.01)),
        )


class GearMotor:
    def __init__(self, p: GearMotorParams) -> None:
        self.p = p
        self.theta_m: float | None = None
        self.w_m = 0.0

    def motor_torque(self, duty: float) -> float:
        duty = max(-1.0, min(1.0, duty))
        return self.p.stall * (duty - self.w_m / self.p.no_load_speed)

    def coupling_torque(self, phi: float, dphi: float) -> float:
        p = self.p
        gap = self.theta_m - phi
        if gap > p.backlash:
            pen = gap - p.backlash
        elif gap < -p.backlash:
            pen = gap + p.backlash
        else:
            return 0.0
        tau = p.stiffness * pen + p.damping * (self.w_m - dphi)
        if p.backlash > 0.0:
            # Teeth push, they don't pull.
            tau = max(tau, 0.0) if pen > 0.0 else min(tau, 0.0)
        return tau

    def step(
        self, duty: float, phi: float, dphi: float, dt: float, substeps: int = 10
    ) -> tuple[float, float]:
        """Advance dt. Returns the mean (tau_c on the wheel, tau_m on the rotor)."""
        if self.theta_m is None:
            # Start in the middle of the gap, moving with the wheel.
            self.theta_m = phi
            self.w_m = dphi
        h = dt / substeps
        tc_sum = tm_sum = 0.0
        for i in range(substeps):
            # The wheel is integrated by the physics engine; assume it moves at
            # a constant speed during the step.
            phi_i = phi + dphi * h * i
            tm = self.motor_torque(duty)
            tc = self.coupling_torque(phi_i, dphi)
            # Semi-implicit Euler on the rotor.
            self.w_m += h * (tm - tc) / self.p.inertia
            self.theta_m += h * self.w_m
            tc_sum += tc
            tm_sum += tm
        return tc_sum / substeps, tm_sum / substeps
