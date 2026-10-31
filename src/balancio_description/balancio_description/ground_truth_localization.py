"""Simulated localization: publishes map -> odom from Gazebo's ground truth.

On a real robot a localizer (AMCL, SLAM, ...) estimates map -> odom so that
map -> base_footprint is the robot's true pose while odom keeps the drift of the
on-board odometry (REP-105). In simulation the true pose is known, so:

    map -> odom = (map -> base_footprint)_truth * (odom -> base_footprint)_estimate^-1

Subscribes
  /balancio/ground_truth  nav_msgs/Odometry  true pose of base_footprint in map
  /balancio/odom          nav_msgs/Odometry  estimated pose of base_footprint in odom
Publishes
  TF map -> odom
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def pose_to_matrix(p: Pose) -> np.ndarray:
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return float(x), float(y), float(z), float(w)


class GroundTruthLocalization(Node):
    def __init__(self) -> None:
        super().__init__("balancio_ground_truth_localization")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        self.odom_T: np.ndarray | None = None
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/balancio/odom", self.on_odom, 10)
        self.create_subscription(Odometry, "/balancio/ground_truth", self.on_truth, 10)

    def on_odom(self, msg: Odometry) -> None:
        self.odom_T = pose_to_matrix(msg.pose.pose)

    def on_truth(self, msg: Odometry) -> None:
        if self.odom_T is None:
            return
        T = pose_to_matrix(msg.pose.pose) @ np.linalg.inv(self.odom_T)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = matrix_to_quat(T[:3, :3])
        self.tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthLocalization()
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
