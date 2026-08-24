"""Compatibility shim: redirects to the unified launch with slam_backend:=toolbox.

Use the unified launch instead:
    ros2 launch youbot_slam gazebo_slam.launch.py slam_backend:=toolbox
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    slam_share = Path(get_package_share_directory("youbot_slam"))
    return LaunchDescription([
        LogInfo(msg="gazebo_slam_toolbox.launch.py is deprecated. "
                    "Use: ros2 launch youbot_slam gazebo_slam.launch.py slam_backend:=toolbox"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(slam_share / "launch" / "gazebo_slam.launch.py")),
            launch_arguments={"slam_backend": "toolbox"}.items()),
    ])
