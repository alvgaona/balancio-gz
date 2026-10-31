"""Gazebo system: Balancio's two TT gear motors (see gear_motor.py).

Runs every physics step. Load it from the model with the Python system
loader:

  <plugin filename="gz-sim-python-system-loader-system"
          name="gz::sim::systems::PythonSystemLoader">
    <module_name>balancio_description.gz_motor</module_name>
    <params_file>/path/to/balancio.yaml</params_file>
  </plugin>

Subscribes (gz-transport)
  /balancio/{left,right}_wheel/duty   gz.msgs.Double   PWM duty in [-1, 1]

Applies the gearbox torque tau_c to {left,right}_wheel_joint; Gazebo puts
the reaction -tau_c on the body. The body doesn't feel the rest of the stator
reaction, tau_m - tau_c (the torque that spins the rotor up): the gearbox
housing cancels all but 1/48 of it.
"""

from __future__ import annotations

import sys
import threading

import yaml
from gz.msgs10.double_pb2 import Double
from gz.sim8 import Joint, Model
from gz.transport13 import Node

from balancio_description.gear_motor import GearMotor, GearMotorParams

SIDES = ("left", "right")


class BalancioMotors:
    def configure(self, entity, sdf, ecm, event_mgr) -> None:
        path = sdf.get_string("params_file")
        with open(path) as f:
            p = GearMotorParams.from_dict(yaml.safe_load(f))
        self.model = Model(entity)
        self.joints = {}
        self.motors = {}
        self.duty = dict.fromkeys(SIDES, 0.0)
        self.lock = threading.Lock()
        for s in SIDES:
            j = Joint(self.model.joint_by_name(ecm, f"{s}_wheel_joint"))
            j.enable_position_check(ecm, True)
            j.enable_velocity_check(ecm, True)
            self.joints[s] = j
            self.motors[s] = GearMotor(p)
        self.node = Node()
        for s in SIDES:
            self.node.subscribe(
                Double,
                f"/balancio/{s}_wheel/duty",
                lambda msg, s=s: self._on_duty(s, msg),
            )
        print(
            f"[balancio motors] stall {p.stall} N m, no-load {p.no_load_speed} rad/s, "
            f"rotor {p.inertia} kg m^2, backlash +-{p.backlash} rad",
            file=sys.stderr,
            flush=True,
        )

    def _on_duty(self, side: str, msg: Double) -> None:
        with self.lock:
            self.duty[side] = msg.data

    def pre_update(self, info, ecm) -> None:
        if info.paused:
            return
        dt = info.dt.total_seconds()
        if dt <= 0.0:
            return
        with self.lock:
            duty = dict(self.duty)
        for s in SIDES:
            j = self.joints[s]
            pos, vel = j.position(ecm), j.velocity(ecm)
            if not pos or not vel:
                return  # not populated by physics yet
            tau_c, _ = self.motors[s].step(duty[s], pos[0], vel[0], dt)
            j.set_force(ecm, [tau_c])


def get_system() -> BalancioMotors:
    return BalancioMotors()
