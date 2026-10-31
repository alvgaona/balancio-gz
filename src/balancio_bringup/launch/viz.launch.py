"""RViz with the Balancio view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory("balancio_bringup")

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(bringup, "config", "balancio.rviz")],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
