"""Industrial-comparison bringup: same stack, slam_toolbox as the SLAM backend.

Wiring (versus gazebo_slam.launch.py, which uses the homemade slam_node):

    /odom -> noisy_odom (publish_tf) -> /odom_noisy + TF odom->base_link
    /scan + TF ------------------------> slam_toolbox -> TF map->odom
    TF map->base_link -----------------> pose_from_tf -> /pose_slam
    control stack (unchanged) reads /pose_slam.

Requires:  sudo apt install ros-jazzy-slam-toolbox

    ros2 launch youbot_slam gazebo_slam_toolbox.launch.py
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

# Lock the Gazebo GUI camera onto the spawned robot, and KEEP it locked.
#
# Three things had to be learned the hard way here, all visible in field logs.
#
# 1. /gui/move_to answers "data: true" as soon as the SERVICE is up, which is
#    well before the GUI's render scene has been populated. The old one-shot
#    call exited on that first true and a second later the log said
#        [GUI] [Err] Unable to move to target. Target: 'youbot' not found
#    so the camera stayed where it was and the robot looked "invisible".
#    Hence: keep re-issuing until /gui/currently_tracked names the robot.
#
# 2. Confirming the lock once is not enough. One run confirmed it at 12:53 and
#    the robot was off screen by 12:58; another logged the loss at 15:18 and
#    never got it back. The GUI drops the follow on its own.
#
# 3. /gui/follow is DEPRECATED in Harmonic -- the GUI says so at startup, right
#    next to the replacement: "Tracking topic on [/gui/track]". Re-issuing a
#    deprecated service is why the watchdog could detect the loss but never
#    repair it. The track TOPIC is the supported path, so it is used first and
#    the old service is kept only as a fallback for older Gazebo builds.
#
# And the watchdog no longer trusts /gui/currently_tracked to decide whether to
# act: that topic is not guaranteed to keep publishing while tracking is
# healthy, so "no message" was being read as "lost" and vice versa. It simply
# re-asserts the track every 20 s, which is a no-op when the GUI is already
# tracking the robot. Only move_to (which SNAPS the camera, and would be
# unusable on a timer) stays conditional on the tracking topic going quiet.
FOLLOW_ROBOT = (
    ': gui_follow watchdog; '        # marker: lets run.sh pkill this by name
    'req=\'data: "youbot"\'; '
    'moveto() { gz service -s /gui/move_to --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    # The supported API (topic), then the deprecated one, so this works on
    # Harmonic and on anything older that still only has the service.
    'track() { gz topic -t /gui/track -m gz.msgs.TrackVisual '
    '  -p \'name: "youbot", inherit_orientation: true, min_dist: 3.0, '
    'max_dist: 9.0\' >/dev/null 2>&1; '
    '  gz service -s /gui/follow --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    'tracked() { timeout 3 gz topic -e -t /gui/currently_tracked -n 1 '
    '  2>/dev/null | grep -q youbot; }; '
    'locked=0; '
    'for i in $(seq 1 40); do '
    '  moveto; track; '
    '  if tracked; then locked=1; break; fi; sleep 1; done; '
    'if [ "$locked" = 1 ]; then '
    '  echo "[view] Gazebo camera is following the robot (confirmed)."; '
    'else '
    '  echo "[view] the Gazebo camera would not lock on -- right-click youbot '
    'in the Entity Tree > Follow."; fi; '
    'quiet=0; '
    'while sleep 20; do '
    '  track; '                       # cheap, idempotent, always re-asserted
    '  if tracked; then quiet=0; continue; fi; '
    '  quiet=$((quiet + 1)); '
    '  if [ "$quiet" = 3 ]; then '
    '    echo "[view] the Gazebo camera lost the robot -- snapping back to it. '
    'The robot itself is fine, check its truth pose in truth_monitor."; '
    '    moveto; fi; '
    'done'
)


def generate_launch_description() -> LaunchDescription:
    gz_share = Path(get_package_share_directory("youbot_gazebo"))
    bringup_share = Path(get_package_share_directory("youbot_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    slam_share = Path(get_package_share_directory("youbot_slam"))

    world = str(gz_share / "worlds" / "greenhouse.sdf")
    robot_desc = (gz_share / "urdf" / "youbot_gz.urdf").read_text()
    bridge_cfg = str(gz_share / "config" / "gz_bridge.yaml")
    params = str(bringup_share / "config" / "youbot_params.yaml")
    rviz_cfg = str(bringup_share / "config" / "youbot.rviz")
    toolbox_cfg = str(slam_share / "config" / "slam_toolbox.yaml")

    res_path = os.pathsep.join(
        [str(gz_share.parent), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]).strip(os.pathsep)

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    sim_time = {"use_sim_time": True}

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

    def control(executable, localized=False):
        remaps = [("odom", "pose_slam")] if localized else []
        if executable in GUARDED:
            remaps = remaps + [("cmd_vel", "cmd_vel_raw")]
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params, sim_time],
                    remappings=remaps)

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),

        gz_sim,
        gz_sim_headless,

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_desc}, sim_time]),

        # Spawn INSIDE safety_node's fence (|x| < 4.60, |y| < 2.15). The old
        # -4.6 sat exactly on it, so the guard opened every run by shoving the
        # robot back in before the mission had said anything.
        Node(package="ros_gz_sim", executable="create", name="spawn_youbot",
             output="screen",
             arguments=["-topic", "robot_description", "-name", "youbot",
                        "-x", "-4.40", "-y", "1.85", "-z", "0.0"]),

        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             output="screen",
             parameters=[{"config_file": bridge_cfg}, sim_time]),

        # --- drifting odometry + the industrial SLAM backend -------------------
        Node(package="youbot_slam", executable="noisy_odom", name="noisy_odom",
             output="screen", parameters=[{"publish_tf": True}, sim_time]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[toolbox_cfg, sim_time]),
        Node(package="youbot_slam", executable="pose_from_tf", name="pose_from_tf",
             output="screen", parameters=[sim_time]),

        # --- the UNCHANGED control stack, reading the toolbox pose -------------
        control("safety_node", localized=True),   # /cmd_vel_raw -> /cmd_vel
        control("mapping_node", localized=True),
        control("planning_node", localized=True),
        control("navigation_node", localized=True),
        control("mission_node", localized=True),
        # The camera head. Not localized=True: it only reads /joint_states and
        # writes the head angle, so it needs no pose. But it MUST be present --
        # the detector publishes no fruit positions without a settled angle.
        control("camera_pan_node"),
        control("strawberry_detector", localized=True),  # needs the pose
                                                       # to place fruit on the map
        control("arm_node", localized=True),   # workspace limit uses the estimate


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

        RegisterEventHandler(OnShutdown(
            # A FUNCTION, not a prebuilt action: shutdown can be reached more
            # than once (Ctrl-C racing a node exit), and re-executing the same
            # ExecuteProcess instance is an error. This hands back a new one.
            on_shutdown=lambda event, context: [ExecuteProcess(
                cmd=["bash", "-c", 'pkill -9 -f "gz sim"; pkill -9 -f "gz-sim"; '
                     'pkill -9 -f "ruby.*gz sim"; pkill -9 -f parameter_bridge; '
                     'pkill -9 -f rviz2; pkill -9 -f rqt_image_view'],
                output="screen")])),
    ])
