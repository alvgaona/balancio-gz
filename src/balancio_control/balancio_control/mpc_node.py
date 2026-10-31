"""ROS 2 node: NMPC balancing + velocity/heading tracking for Balancio.

Subscribes
  /balancio/imu            sensor_msgs/Imu
  /balancio/joint_states   sensor_msgs/JointState
  /cmd_vel                 geometry_msgs/Twist   (linear.x [m/s], angular.z [rad/s])

Publishes
  /balancio/left_wheel/torque, /balancio/right_wheel/torque   std_msgs/Float64 [N m]
  /balancio/odom             nav_msgs/Odometry (wheel/IMU dead reckoning)
  /balancio/mpc/predicted_path nav_msgs/Path
  /balancio/mpc/state        std_msgs/Float64MultiArray  [x, th, psi, dx, dth, dpsi]
  /balancio/mpc/reference    std_msgs/Float64MultiArray  (same layout)
  /balancio/mpc/input        std_msgs/Float64MultiArray  [tau_L, tau_R, solve_ms, iters]

The twist command is turned into a reference over the horizon (as in twsbr):
x_ref and psi_ref are integrated from (v, w), with dx_ref = v, dpsi_ref = w,
theta_ref = dtheta_ref = 0.
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import TransformBroadcaster

from balancio_control.estimator import TwsbrEstimator
from balancio_control.model import TwsbrParams
from balancio_control.nmpc import Nmpc, NmpcConfig


def yaw_pitch_to_quat(yaw: float, pitch: float = 0.0, roll: float = 0.0):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class BalancioMpcNode(Node):
    def __init__(self) -> None:
        super().__init__("balancio_mpc")
        default_model = os.path.join(
            get_package_share_directory("balancio_description"),
            "config",
            "balancio.yaml",
        )
        d = NmpcConfig()
        self.declare_parameter("model_file", default_model)
        self.declare_parameter("dt", d.dt)
        self.declare_parameter("horizon", d.horizon)
        self.declare_parameter("q", d.q)
        self.declare_parameter("r", d.r)
        self.declare_parameter("r_rate", d.r_rate)
        self.declare_parameter("max_torque", -1.0)  # <= 0: use model_file value
        self.declare_parameter("motor_no_load_speed", -1.0)  # < 0: use model_file value
        self.declare_parameter("max_torque_rate", d.max_torque_rate)
        self.declare_parameter("max_theta", d.max_theta)
        self.declare_parameter("solver", d.solver)
        self.declare_parameter("fall_angle", 1.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("max_speed", 0.6)
        self.declare_parameter("max_yaw_rate", 3.0)
        self.declare_parameter("max_position_lead", 0.2)
        self.declare_parameter("max_heading_lead", 0.6)
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        gp = self.get_parameter
        model_file = gp("model_file").value
        self.params = TwsbrParams.from_yaml(model_file)
        with open(model_file) as f:
            model_yaml = yaml.safe_load(f)
        max_torque = float(gp("max_torque").value)
        if max_torque <= 0.0:
            max_torque = float(model_yaml.get("max_torque", d.max_torque))
        no_load_speed = float(gp("motor_no_load_speed").value)
        if no_load_speed < 0.0:
            no_load_speed = float(model_yaml.get("motor_no_load_speed", 0.0))

        self.cfg = NmpcConfig(
            dt=float(gp("dt").value),
            horizon=int(gp("horizon").value),
            q=list(gp("q").value),
            r=list(gp("r").value),
            r_rate=list(gp("r_rate").value),
            max_torque=max_torque,
            motor_no_load_speed=no_load_speed,
            max_torque_rate=float(gp("max_torque_rate").value),
            max_theta=float(gp("max_theta").value),
            solver=str(gp("solver").value),
        )
        t0 = time.perf_counter()
        self.mpc = Nmpc(self.params, self.cfg)
        self.get_logger().info(
            f"NMPC ready ({self.cfg.solver}, N={self.cfg.horizon}, dt={self.cfg.dt}s, "
            f"|tau|<={self.cfg.max_torque} Nm, no-load {self.cfg.motor_no_load_speed} rad/s) "
            f"built in {time.perf_counter() - t0:.2f}s"
        )

        self.est = TwsbrEstimator(self.params.r)
        self.left_joint = gp("left_wheel_joint").value
        self.right_joint = gp("right_wheel_joint").value
        self.fall_angle = float(gp("fall_angle").value)
        self.cmd_timeout = float(gp("cmd_timeout").value)
        self.max_speed = float(gp("max_speed").value)
        self.max_yaw_rate = float(gp("max_yaw_rate").value)
        self.max_pos_lead = float(gp("max_position_lead").value)
        self.max_head_lead = float(gp("max_heading_lead").value)
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value

        self.u_prev = np.zeros(2)
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_time = None
        self.x_ref: float | None = None
        self.psi_ref: float | None = None
        self.fallen = False
        self.solve_times: list[float] = []

        self.create_subscription(
            Imu, "/balancio/imu", self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/balancio/joint_states",
            self.on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self.pub_left = self.create_publisher(
            Float64, "/balancio/left_wheel/torque", 10
        )
        self.pub_right = self.create_publisher(
            Float64, "/balancio/right_wheel/torque", 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/balancio/odom", 10)
        self.pub_pred = self.create_publisher(Path, "/balancio/mpc/predicted_path", 10)
        self.pub_state = self.create_publisher(
            Float64MultiArray, "/balancio/mpc/state", 10
        )
        self.pub_ref = self.create_publisher(
            Float64MultiArray, "/balancio/mpc/reference", 10
        )
        self.pub_input = self.create_publisher(
            Float64MultiArray, "/balancio/mpc/input", 10
        )
        self.tf = TransformBroadcaster(self) if gp("publish_tf").value else None

        self.create_timer(self.cfg.dt, self.step)
        self.create_timer(5.0, self.report)

    # ---------------------------------------------------------------- inputs
    def on_imu(self, msg: Imu) -> None:
        o, w = msg.orientation, msg.angular_velocity
        self.est.update_imu((o.x, o.y, o.z, o.w), (w.x, w.y, w.z))

    def on_joints(self, msg: JointState) -> None:
        try:
            il = msg.name.index(self.left_joint)
            ir = msg.name.index(self.right_joint)
        except ValueError:
            return
        vel = (
            msg.velocity
            if len(msg.velocity) == len(msg.name)
            else [0.0] * len(msg.name)
        )
        self.est.update_wheels(msg.position[il], msg.position[ir], vel[il], vel[ir])

    def on_cmd_vel(self, msg: Twist) -> None:
        self.v_cmd = float(np.clip(msg.linear.x, -self.max_speed, self.max_speed))
        self.w_cmd = float(
            np.clip(msg.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        )
        self.last_cmd_time = self.get_clock().now()

    # ---------------------------------------------------------------- control
    def publish_torque(self, u: np.ndarray) -> None:
        self.pub_left.publish(Float64(data=float(u[0])))
        self.pub_right.publish(Float64(data=float(u[1])))

    def step(self) -> None:
        if not self.est.ready:
            return
        x = self.est.state()
        now = self.get_clock().now()
        dt = self.cfg.dt

        if (
            self.last_cmd_time is None
            or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout
        ):
            self.v_cmd = self.w_cmd = 0.0

        if self.x_ref is None or self.psi_ref is None:
            self.x_ref, self.psi_ref = float(x[0]), float(x[2])

        if abs(x[1]) > self.fall_angle:
            if not self.fallen:
                self.get_logger().warn(
                    f"Robot fell (pitch {math.degrees(x[1]):.1f} deg). Torques disabled."
                )
            self.fallen = True
            self.u_prev[:] = 0.0
            self.mpc.reset()
            self.x_ref, self.psi_ref = float(x[0]), float(x[2])
            self.publish_torque(self.u_prev)
            return
        if self.fallen and abs(x[1]) < 0.2:
            self.get_logger().info("Robot upright again, NMPC re-enabled.")
            self.fallen = False

        # Advance the reference, never letting it run away from the robot.
        self.x_ref = float(
            np.clip(
                self.x_ref + self.v_cmd * dt,
                x[0] - self.max_pos_lead,
                x[0] + self.max_pos_lead,
            )
        )
        self.psi_ref = float(
            np.clip(
                self.psi_ref + self.w_cmd * dt,
                x[2] - self.max_head_lead,
                x[2] + self.max_head_lead,
            )
        )
        k = np.arange(self.cfg.horizon + 1) * dt
        xref = np.zeros((6, self.cfg.horizon + 1))
        xref[0] = self.x_ref + self.v_cmd * k
        xref[2] = self.psi_ref + self.w_cmd * k
        xref[3] = self.v_cmd
        xref[5] = self.w_cmd

        t0 = time.perf_counter()
        sol = self.mpc.solve(x, xref, self.u_prev)
        solve_ms = 1e3 * (time.perf_counter() - t0)
        self.solve_times.append(solve_ms)
        if not sol.success:
            self.get_logger().warn(
                f"NMPC solver: {sol.status} ({sol.iterations} it)",
                throttle_duration_sec=1.0,
            )
        self.u_prev = sol.u
        self.publish_torque(sol.u)

        self.publish_debug(x, xref[:, 0], sol, solve_ms, now)

    # ---------------------------------------------------------------- output
    def publish_debug(self, x, xref0, sol, solve_ms, now) -> None:
        stamp = now.to_msg()
        self.pub_state.publish(Float64MultiArray(data=[float(v) for v in x]))
        self.pub_ref.publish(Float64MultiArray(data=[float(v) for v in xref0]))
        self.pub_input.publish(
            Float64MultiArray(
                data=[float(sol.u[0]), float(sol.u[1]), solve_ms, float(sol.iterations)]
            )
        )

        X, Y, psi = self.est.X, self.est.Y, float(x[2])
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = X
        odom.pose.pose.position.y = Y
        odom.pose.pose.position.z = self.params.r
        q = yaw_pitch_to_quat(psi, float(x[1]), self.est.roll)
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = q
        odom.twist.twist.linear.x = float(x[3])
        odom.twist.twist.angular.y = float(x[4])
        odom.twist.twist.angular.z = float(x[5])
        self.pub_odom.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = X
            t.transform.translation.y = Y
            t.transform.translation.z = self.params.r
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)

        # Predicted path, projected on the plane from the current pose.
        path = Path()
        path.header = odom.header
        px, py = X, Y
        xs = sol.x_pred
        for i in range(xs.shape[1]):
            if i > 0:
                ds = xs[0, i] - xs[0, i - 1]
                px += ds * math.cos(xs[2, i])
                py += ds * math.sin(xs[2, i])
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            _, _, qz, qw = yaw_pitch_to_quat(float(xs[2, i]))
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.pub_pred.publish(path)

    def report(self) -> None:
        if not self.solve_times:
            return
        st = np.array(self.solve_times)
        self.solve_times.clear()
        self.get_logger().info(
            f"solve time: mean {st.mean():.2f} ms, max {st.max():.2f} ms "
            f"({len(st)} solves) | v_cmd={self.v_cmd:.2f} w_cmd={self.w_cmd:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalancioMpcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_torque(np.zeros(2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
