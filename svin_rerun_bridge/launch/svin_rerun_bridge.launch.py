from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("svin_rerun_bridge"))
    parameter_file = package_share / "config" / "svin_rerun_bridge_pinhole.yaml"
    return LaunchDescription(
        [
            Node(
                package="svin_rerun_bridge",
                executable="svin_rerun_bridge_node",
                name="svin_rerun_bridge",
                output="screen",
                parameters=[str(parameter_file)],
            )
        ]
    )
