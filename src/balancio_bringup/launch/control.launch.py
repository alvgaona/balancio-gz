"""NMPC controller, plus the path follower when a trajectory is given.

controller:=python runs balancio_control's CasADi NMPC, controller:=cpp the
acados one from balancio_nmpc. Both read mpc.yaml and use the same topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    desc = get_package_share_directory("balancio_description")
    ctrl = get_package_share_directory("balancio_control")
    mpc_config = os.path.join(ctrl, "config", "mpc.yaml")
    model_file = LaunchConfiguration("params_file")
    trajectory = LaunchConfiguration("trajectory")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory",
                default_value="none",
                description="none | line | circle | sine | figure8",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(desc, "config", "balancio.yaml"),
            ),
            DeclareLaunchArgument(
                "controller", default_value="python", description="python | cpp"
            ),
            *[
                Node(
                    package=package,
                    executable="mpc_node",
                    name="balancio_mpc",
                    parameters=[mpc_config, {"model_file": model_file}],
                    output="screen",
                    condition=IfCondition(
                        EqualsSubstitution(
                            LaunchConfiguration("controller"), controller
                        )
                    ),
                )
                for controller, package in [
                    ("python", "balancio_control"),
                    ("cpp", "balancio_nmpc"),
                ]
            ],
            Node(
                package="balancio_control",
                executable="path_follower",
                name="balancio_path_follower",
                parameters=[mpc_config, {"trajectory": trajectory}],
                output="screen",
                condition=IfCondition(NotEqualsSubstitution(trajectory, "none")),
            ),
        ]
    )
