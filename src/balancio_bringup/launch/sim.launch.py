"""Gazebo Harmonic + Balancio bringup (no controller).

The server and the GUI are started as separate processes; this works on both
Linux and macOS (where `gz sim` cannot run server and GUI in one process).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("balancio_description")
    bringup = get_package_share_directory("balancio_bringup")
    world = os.path.join(bringup, "worlds", "balancio_world.sdf")
    xacro_file = os.path.join(pkg, "urdf", "balancio.urdf.xacro")
    bridge_config = os.path.join(bringup, "config", "bridge.yaml")

    headless = LaunchConfiguration("headless")
    params_file = LaunchConfiguration("params_file")
    imu_noise = LaunchConfiguration("imu_noise")
    initial_pitch = LaunchConfiguration("initial_pitch")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " params_file:=",
                params_file,
                " imu_noise:=",
                imu_noise,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            # Lets Gazebo resolve package://balancio_description/... mesh URIs
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg)),
            DeclareLaunchArgument(
                "headless", default_value="false", description="Do not start the GUI"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg, "config", "balancio.yaml"),
                description="Physical parameters YAML",
            ),
            DeclareLaunchArgument("imu_noise", default_value="true"),
            DeclareLaunchArgument(
                "initial_pitch",
                default_value="0.1",
                description="Initial body pitch [rad] (positive = leaning forward)",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.035",
                description="Spawn height of the wheel axle (>= wheel radius) [m]",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", "-v", "2", world],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-g", "-v", "2"],
                output="screen",
                condition=UnlessCondition(headless),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": True}
                ],
                remappings=[("/joint_states", "/balancio/joint_states")],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world",
                    "balancio_world",
                    "-name",
                    "balancio",
                    "-topic",
                    "robot_description",
                    "-z",
                    LaunchConfiguration("spawn_z"),
                    "-P",
                    initial_pitch,
                ],
                output="screen",
            ),
            Node(
                package="balancio_description",
                executable="motor_model",
                parameters=[{"params_file": params_file, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config, "use_sim_time": True}],
                output="screen",
            ),
        ]
    )
