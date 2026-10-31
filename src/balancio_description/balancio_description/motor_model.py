"""DC gear motor model between the controller and Gazebo (part of the plant).

Gazebo's ApplyJointForce applies whatever torque it is given. A real motor
driven from a fixed voltage can't: its torque falls linearly with speed
(back-EMF) and reaches zero at the no-load speed:

    tau_applied = clip(tau_cmd, -stall (1 + w / w0), stall (1 - w / w0))

w is the wheel speed relative to the body, i.e. the joint velocity.

Subscribes
  /balancio/{left,right}_wheel/torque   std_msgs/Float64  commanded torque
  /balancio/joint_states                sensor_msgs/JointState
Publishes
  /balancio/{left,right}_wheel/effort   std_msgs/Float64  torque sent to Gazebo
"""

from __future__ import annotations

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

SIDES = ("left", "right")


def motor_torque(
    tau_cmd: float, speed: float, stall: float, no_load_speed: float
) -> float:
    if no_load_speed <= 0.0:
        return max(-stall, min(stall, tau_cmd))
    k = stall / no_load_speed
    return max(-stall - k * speed, min(stall - k * speed, tau_cmd))


class MotorModel(Node):
    def __init__(self) -> None:
        super().__init__("balancio_motor_model")
        default_params = os.path.join(
            get_package_share_directory("balancio_description"),
            "config",
            "balancio.yaml",
        )
        self.declare_parameter("params_file", default_params)
        with open(self.get_parameter("params_file").value) as f:
            p = yaml.safe_load(f)
        self.stall = float(p["max_torque"])
        self.no_load_speed = float(p.get("motor_no_load_speed", 0.0))

        self.cmd = dict.fromkeys(SIDES, 0.0)
        self.pub = {
            s: self.create_publisher(Float64, f"/balancio/{s}_wheel/effort", 10)
            for s in SIDES
        }
        for s in SIDES:
            self.create_subscription(
                Float64,
                f"/balancio/{s}_wheel/torque",
                lambda msg, s=s: self.cmd.__setitem__(s, msg.data),
                10,
            )
        self.create_subscription(
            JointState,
            "/balancio/joint_states",
            self.on_joints,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Motor model: stall {self.stall} N m, no-load speed {self.no_load_speed} rad/s"
        )

    def on_joints(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            return
        for s in SIDES:
            try:
                w = msg.velocity[msg.name.index(f"{s}_wheel_joint")]
            except ValueError:
                continue
            tau = motor_torque(self.cmd[s], w, self.stall, self.no_load_speed)
            self.pub[s].publish(Float64(data=tau))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
