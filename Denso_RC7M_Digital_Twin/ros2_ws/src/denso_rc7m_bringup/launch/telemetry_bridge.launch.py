from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    config = (
        Path(get_package_share_directory("denso_rc7m_gateway"))
        / "config"
        / "gateway.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="denso_rc7m_gateway",
                executable="telemetry_bridge",
                name="telemetry_bridge",
                parameters=[str(config)],
                output="screen",
            )
        ]
    )
