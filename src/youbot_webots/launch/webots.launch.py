"""Start Webots + the webots_ros2 driver for the YouBot.

    ros2 launch youbot_webots webots.launch.py

This opens the simulator on `worlds/smart_agriculture.wbt`, attaches the
YoubotDriver plugin (declared in resource/youbot.urdf), and starts the
webots_ros2 supervisor. Sensors show up as ROS topics (/scan, camera...),
/cmd_vel drives the base, /odom carries the pose.

Then, in another terminal, bring up the brain:
    ros2 launch youbot_bringup bringup.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description() -> LaunchDescription:
    pkg = Path(get_package_share_directory("youbot_webots"))

    webots = WebotsLauncher(world=str(pkg / "worlds" / "smart_agriculture.wbt"))

    youbot_driver = WebotsController(
        robot_name="youbot",  # must match the YouBot `name` field in the .wbt
        parameters=[{"robot_description": str(pkg / "resource" / "youbot.urdf")}],
    )

    return LaunchDescription([
        webots,
        youbot_driver,
        # Shut everything down cleanly when Webots is closed.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
