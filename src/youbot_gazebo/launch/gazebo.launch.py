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

# Lock the Gazebo GUI camera onto the spawned robot.
#
# Why this is a confirmation loop and not one call. /gui/move_to answers
# "data: true" as soon as the SERVICE is up, which happens well before the GUI's
# render scene has been populated from /world/<w>/scene/info. The old version
# exited on that first true and the log then showed, a second later:
#
#     [GUI] [Err] [CameraTracking.cc:425] Unable to move to target.
#                                         Target: 'youbot' not found
#
# -- the request was accepted and quietly did nothing, so the camera stayed
# wherever it had been left and the robot was "invisible". Whether it worked
# was a race between the GUI loading and our 6 s timer, which is exactly why it
# came and went between runs.
#
# So: keep re-issuing, and believe it only when the CameraTracking plugin says
# on /gui/currently_tracked that it is tracking the robot.
#
# And keep watching afterwards. Confirming the lock once is not enough: a run
# logged "[view] Gazebo camera is following the robot (confirmed)." at 12:53
# and the robot was off screen again by 12:58. The GUI drops the follow on its
# own -- a scene rebuild, a stray click in the 3D view, the entity being
# re-created -- and nothing said so, which is what made the robot look like it
# had "disappeared" when it was in fact driving normally the whole time. The
# watchdog re-issues /gui/follow (NOT /gui/move_to: that one snaps the camera,
# and doing it every 15 s would be unusable) and logs the loss and the recovery
# so the log tells you which of the two happened.
FOLLOW_ROBOT = (
    ': gui_follow watchdog; '        # marker: lets run.sh pkill this by name
    'req=\'data: "youbot"\'; '
    'moveto() { gz service -s /gui/move_to --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    'follow() { gz service -s /gui/follow --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    'tracked() { timeout 3 gz topic -e -t /gui/currently_tracked -n 1 '
    '  2>/dev/null | grep -q youbot; }; '
    'locked=0; '
    'for i in $(seq 1 40); do '
    '  moveto; follow; '
    '  if tracked; then locked=1; break; fi; sleep 1; done; '
    'if [ "$locked" = 1 ]; then '
    '  echo "[view] Gazebo camera is following the robot (confirmed)."; '
    'else '
    '  echo "[view] the Gazebo camera would not lock on -- right-click youbot '
    'in the Entity Tree > Follow."; fi; '
    'lost=0; '
    'while sleep 15; do '
    '  if tracked; then '
    '    if [ "$lost" != 0 ]; then '
    '      echo "[view] camera lock recovered."; lost=0; fi; '
    '    continue; fi; '
    '  if [ "$lost" = 0 ]; then '
    '    echo "[view] the Gazebo camera stopped following the robot -- '
    'the robot is still driving, the view lost it. Re-following."; fi; '
    '  lost=$((lost + 1)); follow; '
    '  if [ "$lost" -ge 4 ]; then moveto; fi; '
    'done'
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
        # Spawn INSIDE safety_node's fence (|x| < 4.60, |y| < 2.15). The old
        # -4.6 sat exactly on it, so the guard opened every run by shoving the
        # robot back in before the mission had said anything.
        Node(package="ros_gz_sim", executable="create", name="spawn_youbot",
             output="screen",
             arguments=["-topic", "robot_description", "-name", "youbot",
                        "-x", "-4.40", "-y", "1.85", "-z", "0.0"]),

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
