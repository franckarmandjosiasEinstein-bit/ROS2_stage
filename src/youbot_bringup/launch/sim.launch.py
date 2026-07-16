"""Run the WHOLE stack without Webots: a headless fake robot + the control
nodes + RViz. The robot is a point that moves exactly as commanded and emits
a synthetic /scan, so you can develop and watch the system entirely in ROS.

    ros2 launch youbot_bringup sim.launch.py            # with RViz
    ros2 launch youbot_bringup sim.launch.py rviz:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("youbot_bringup"))
    params = str(share / "config" / "youbot_params.yaml")
    rviz_cfg = str(share / "config" / "youbot.rviz")
    use_rviz = LaunchConfiguration("rviz")

    def node(executable):
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz to visualise the stack."),
        node("sim_node"),          # fake robot: /cmd_vel -> /odom + /scan + TF
        node("mapping_node"),
        node("planning_node"),
        node("navigation_node"),
        node("mission_node"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_cfg], condition=IfCondition(use_rviz)),
    ])
