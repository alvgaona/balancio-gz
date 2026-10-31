"""Global path following for Balancio (twsbr `twsbr_nmpc_global.m` in ROS 2).

Generates a reference path (line, circle, sine, figure8) and publishes the
/cmd_vel (forward speed + yaw rate) that the NMPC tracks. As in twsbr, the
feedforward is the path tangent speed and heading; a Kanayama tracking law adds
feedback on the along-track, cross-track and heading errors so the robot does
not drift off the path.

Trajectories start at the robot's initial pose with the initial tangent along +x.
"""

from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from balancio_control.estimator import quat_to_euler, wrap_angle

Traj = Callable[[float], tuple[float, float]]


def make_trajectory(kind: str, speed: float, size: float) -> Traj:
    """Return P(t) = (X, Y). `speed` is the nominal path speed, `size` its scale."""
    if kind == "line":
        return lambda t: (speed * t, 0.0)
    if kind == "circle":
        w = speed / size
        return lambda t: (size * math.sin(w * t), size * (1.0 - math.cos(w * t)))
    if kind == "sine":
        w = 2.0 * math.pi * speed / (4.0 * size)  # wavelength = 4 * size
        return lambda t: (speed * t, 0.5 * size * math.sin(w * t))
    if kind == "figure8":
        w = speed / (size * 1.25)  # roughly constant mean speed
        return lambda t: (size * math.sin(w * t), 0.5 * size * math.sin(2.0 * w * t))
    raise ValueError(f"unknown trajectory '{kind}'")


def derivatives(traj: Traj, t: float, h: float = 1e-3):
    """Position, velocity and acceleration by central differences."""
    x0, y0 = traj(t)
    xp, yp = traj(t + h)
    xm, ym = traj(t - h)
    vel = ((xp - xm) / (2 * h), (yp - ym) / (2 * h))
    acc = ((xp - 2 * x0 + xm) / h**2, (yp - 2 * y0 + ym) / h**2)
    return (x0, y0), vel, acc


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__("balancio_path_follower")
        self.declare_parameter("trajectory", "figure8")
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("size", 1.0)
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("k_along", 1.5)
        self.declare_parameter("k_cross", 8.0)
        self.declare_parameter("k_heading", 3.0)
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 2.5)
        self.declare_parameter("odom_topic", "/balancio/odom")
        self.declare_parameter("frame_id", "odom")

        gp = self.get_parameter
        self.kind = str(gp("trajectory").value)
        self.traj = make_trajectory(
            self.kind, float(gp("speed").value), float(gp("size").value)
        )
        self.start_delay = float(gp("start_delay").value)
        self.kx = float(gp("k_along").value)
        self.ky = float(gp("k_cross").value)
        self.kpsi = float(gp("k_heading").value)
        self.vmax = float(gp("max_speed").value)
        self.wmax = float(gp("max_yaw_rate").value)
        self.frame_id = str(gp("frame_id").value)

        self.pose: tuple[float, float, float] | None = None  # X, Y, yaw
        self.origin: tuple[float, float, float] | None = None
        self.t_start = None

        self.create_subscription(
            Odometry, str(gp("odom_topic").value), self.on_odom, 10
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_ref_path = self.create_publisher(Path, "/balancio/reference_path", 1)
        self.pub_path = self.create_publisher(Path, "/balancio/path", 1)
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.ref_path: Path | None = None

        self.create_timer(1.0 / float(gp("rate").value), self.step)
        self.create_timer(1.0, self.publish_paths)
        self.get_logger().info(f"Following '{self.kind}' after {self.start_delay:.1f}s")

    def on_odom(self, msg: Odometry) -> None:
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        _, _, yaw = quat_to_euler(o.x, o.y, o.z, o.w)
        self.pose = (p.x, p.y, yaw)

    def to_world(self, X: float, Y: float) -> tuple[float, float]:
        assert self.origin is not None
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * X - s * Y, oy + s * X + c * Y

    def step(self) -> None:
        if self.pose is None:
            return
        now = self.get_clock().now()
        if self.t_start is None:
            self.t_start = now
        t = (now - self.t_start).nanoseconds * 1e-9 - self.start_delay
        if t < 0.0:
            self.pub_cmd.publish(Twist())
            return
        if self.origin is None:
            self.origin = self.pose
            self.build_reference_path()

        (Xr, Yr), (dX, dY), (ddX, ddY) = derivatives(self.traj, t)
        Xr, Yr = self.to_world(Xr, Yr)
        v_ff = math.hypot(dX, dY)
        psi_ff = math.atan2(dY, dX) + self.origin[2]
        w_ff = (dX * ddY - dY * ddX) / max(v_ff**2, 1e-6)

        X, Y, psi = self.pose
        ex, ey = Xr - X, Yr - Y
        e_along = math.cos(psi) * ex + math.sin(psi) * ey
        e_cross = -math.sin(psi) * ex + math.cos(psi) * ey
        e_psi = wrap_angle(psi_ff - psi)

        v = v_ff * math.cos(e_psi) + self.kx * e_along
        w = w_ff + v_ff * (self.ky * e_cross + self.kpsi * math.sin(e_psi))

        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.vmax, self.vmax))
        cmd.angular.z = float(np.clip(w, -self.wmax, self.wmax))
        self.pub_cmd.publish(cmd)

        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x, ps.pose.position.y = X, Y
        self.path.poses.append(ps)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

    def build_reference_path(self) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        for t in np.linspace(0.0, 120.0, 1200):
            X, Y = self.to_world(*self.traj(float(t)))
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x, ps.pose.position.y = X, Y
            path.poses.append(ps)
        self.ref_path = path

    def publish_paths(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self.ref_path is not None:
            self.ref_path.header.stamp = stamp
            self.pub_ref_path.publish(self.ref_path)
        self.path.header.stamp = stamp
        self.pub_path.publish(self.path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
