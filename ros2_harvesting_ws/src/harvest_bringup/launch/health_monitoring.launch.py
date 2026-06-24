from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def _config(package: str, filename: str) -> str:
    return str(Path(get_package_share_directory(package)) / "config" / filename)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="harvest_digital_twin",
                executable="twin_monitor",
                name="twin_monitor",
                parameters=[_config("harvest_digital_twin", "twin_monitor.yaml")],
                output="screen",
            ),
            Node(
                package="harvest_condition_monitor",
                executable="condition_monitor",
                name="condition_monitor",
                parameters=[_config("harvest_condition_monitor", "condition_monitor.yaml")],
                output="screen",
            ),
            Node(
                package="harvest_predictive_maintenance",
                executable="maintenance_predictor",
                name="maintenance_predictor",
                parameters=[
                    _config(
                        "harvest_predictive_maintenance",
                        "maintenance_predictor.yaml",
                    )
                ],
                output="screen",
            ),
        ]
    )
