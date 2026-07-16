"""Bring up the full control stack (without the simulator).

Starts mapping, planning, navigation and the mission orchestrator, all
reading parameters from config/youbot_params.yaml. Launch the simulator
separately (youbot_webots/launch/webots.launch.py) so you can also run the
stack against a bag file or a different robot.

    ros2 launch youbot_bringup bringup.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("youbot_bringup")) / "config" / "youbot_params.yaml")

    def node(executable):
        return Node(
            package="youbot_control",
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params],
        )

    return LaunchDescription([
        node("mapping_node"),
        node("planning_node"),
        node("navigation_node"),
        node("mission_node"),
        node("vision_node"),   # real camera detection (Webots). Headless, sim_node emits detections.
    ])
