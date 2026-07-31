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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Lock the Gazebo GUI camera onto the spawned robot: move_to recentres, follow
# keeps it in frame as it drives. Retried for ~60 s since the GUI can be slow.
FOLLOW_ROBOT = (
    'req=\'data: "youbot"\'; '
    'for i in $(seq 1 30); do '
    '  if gz service -s /gui/move_to --reqtype gz.msgs.StringMsg '
    '       --reptype gz.msgs.Boolean --timeout 2000 --req "$req" 2>/dev/null '
    '       | grep -q "data: true"; then '
    '    gz service -s /gui/follow --reqtype gz.msgs.StringMsg '
    '       --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; '
    '    echo "[view] Gazebo camera locked on the robot."; exit 0; '
    '  fi; sleep 2; done; '
    'echo "[view] could not reach the Gazebo GUI -- use the Entity Tree to '
    'right-click youbot > Follow."'
)


def generate_launch_description() -> LaunchDescription:
    gz_share = Path(get_package_share_directory("youbot_gazebo"))
    bringup_share = Path(get_package_share_directory("youbot_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = str(gz_share / "worlds" / "greenhouse.sdf")
    robot_desc = (gz_share / "urdf" / "youbot_gz.urdf").read_text()
    bridge_cfg = str(gz_share / "config" / "gz_bridge.yaml")
    params = str(bringup_share / "config" / "youbot_params.yaml")
    rviz_cfg = str(bringup_share / "config" / "youbot.rviz")

    # Let gz resolve the robot's package:// mesh URIs: adding the share dir that
    # contains youbot_gazebo/ makes "package://youbot_gazebo/meshes/x.stl" resolve.
    res_path = os.pathsep.join(
        [str(gz_share.parent), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]).strip(os.pathsep)

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

    # navigation and mission both drive the base; both go through the guard.
    GUARDED = ("navigation_node", "mission_node")

    def control(executable):
        remaps = [("cmd_vel", "cmd_vel_raw")] if executable in GUARDED else []
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params, sim_time],
                    remappings=remaps)

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),
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
        control("safety_node"),      # /cmd_vel_raw -> /cmd_vel
        control("mapping_node"),
        control("planning_node"),
        control("navigation_node"),
        control("mission_node"),
        control("strawberry_detector"),   # camera -> /camera/detections + /ripe_count
        control("arm_node"),              # /do_pick -> pick sequence + /joint_states


        # The GUI camera: Gazebo REWRITES ~/.gz/sim/8/gui.config on every exit
        # with wherever the camera was left, and reads it back in preference to
        # the world's <gui> pose. Since the robot roams, that saved view is
        # almost never where it respawns -- which is why it "disappears" at
        # random. Lock the camera onto the robot once the GUI is up (retried,
        # because GUI load time varies hugely with the GPU).
        TimerAction(period=6.0, actions=[ExecuteProcess(
            cmd=["bash", "-c", FOLLOW_ROBOT], output="screen",
            condition=IfCondition(use_gui))]),

        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_cfg], parameters=[sim_time],
             condition=IfCondition(use_rviz)),

        # gz sim does not die on Ctrl-C -- kill any leftover on shutdown so the
        # next run starts with a single /clock publisher (no zombie sims).
        RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(
                cmd=["bash", "-c", 'pkill -9 -f "gz sim"; pkill -9 -f "gz-sim"; '
                     'pkill -9 -f "ruby.*gz sim"; pkill -9 -f parameter_bridge; '
                     'pkill -9 -f rviz2; pkill -9 -f rqt_image_view'],
                output="screen"),
        ])),
    ])
