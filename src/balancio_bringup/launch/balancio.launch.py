"""Everything at once: Gazebo and the robot, the NMPC, and optionally path following and RViz.

Each part is also its own launch file (sim, control, viz), to run in separate terminals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    desc = get_package_share_directory("balancio_description")
    bringup = get_package_share_directory("balancio_bringup")

    def include(name, arguments, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", name)),
            launch_arguments={k: LaunchConfiguration(k) for k in arguments}.items(),
            **kwargs,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("initial_pitch", default_value="0.1"),
            DeclareLaunchArgument(
                "trajectory",
                default_value="none",
                description="none | line | circle | sine | figure8",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(desc, "config", "balancio.yaml"),
            ),
            include("sim.launch.py", ["headless", "initial_pitch", "params_file"]),
            DeclareLaunchArgument(
                "controller", default_value="python", description="python | cpp"
            ),
            include("control.launch.py", ["trajectory", "params_file", "controller"]),
            include(
                "viz.launch.py", [], condition=IfCondition(LaunchConfiguration("rviz"))
            ),
        ]
    )
