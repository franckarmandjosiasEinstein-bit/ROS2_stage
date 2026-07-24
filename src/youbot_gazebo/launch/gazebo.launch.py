"""Phase 3 -- the digital twin in Gazebo Harmonic (realistic 3D render) driven
by the SAME ROS 2 control stack validated headless.

Brings up:
    * Gazebo (gz sim) with the strawberry-greenhouse world;
    * the YouBot spawned from its URDF (gpu_lidar + VelocityControl + odom);
    * ros_gz_bridge  (/clock /cmd_vel /odom /scan /tf);
    * robot_state_publisher  (base_link -> wheels/arm/lidar TF + /robot_description);
    * the control stack (mapping -> planning -> navigation -> mission);
    * RViz (optional).

    ros2 launch youbot_gazebo gazebo.launch.py
    ros2 launch youbot_gazebo gazebo.launch.py rviz:=false gui:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gz_share = Path(get_package_share_directory("youbot_gazebo"))
    bringup_share = Path(get_package_share_directory("youbot_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = str(gz_share / "worlds" / "greenhouse.sdf")
    robot_desc = (gz_share / "urdf" / "youbot_gz.urdf").read_text()
    bridge_cfg = str(gz_share / "config" / "gz_bridge.yaml")
    params = str(bringup_share / "config" / "youbot_params.yaml")
    rviz_cfg = str(bringup_share / "config" / "youbot.rviz")

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    sim_time = {"use_sim_time": True}

    # gz sim: -r run immediately. Full GUI when gui:=true, else -s headless server.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v3 ", world]}.items(),
        condition=IfCondition(use_gui),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v3 ", world]}.items(),
        condition=UnlessCondition(use_gui),
    )

    def control(executable):
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params, sim_time])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz to visualise the stack."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Launch the Gazebo GUI (gui:=false runs a headless server)."),

        gz_sim,
        gz_sim_headless,

        # base_link -> wheels/arm/lidar TF, and /robot_description for the spawn.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_desc}, sim_time]),

        # Spawn the robot into the running world at the sim start pose.
        Node(package="ros_gz_sim", executable="create", name="spawn_youbot",
             output="screen",
             arguments=["-topic", "robot_description", "-name", "youbot",
                        "-x", "-4.6", "-y", "1.9", "-z", "0.0"]),

        # Gazebo <-> ROS bridge.
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             output="screen",
             parameters=[{"config_file": bridge_cfg}, sim_time]),

        # The validated control stack (no sim_node -- Gazebo IS the robot now).
        control("odom_tf"),               # /odom -> TF map->base_link (sim time)
        control("mapping_node"),
        control("planning_node"),
        control("navigation_node"),
        control("mission_node"),
        control("strawberry_detector"),   # camera -> /camera/detections + /ripe_count

        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_cfg], parameters=[sim_time],
             condition=IfCondition(use_rviz)),
    ])
