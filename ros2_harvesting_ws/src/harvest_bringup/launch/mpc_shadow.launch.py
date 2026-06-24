from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    config = (
        Path(get_package_share_directory("harvest_mpc_controller"))
        / "config"
        / "mpc.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="harvest_mpc_controller",
                executable="mpc_reference",
                name="mpc_reference",
                parameters=[str(config)],
                output="screen",
            )
        ]
    )
