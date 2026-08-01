"""Level-3 autonomy bringup: the full Gazebo stack, localised by SLAM.

Same digital twin as youbot_gazebo/gazebo.launch.py, but the control stack no
longer receives the ground-truth pose. Instead:

    ground truth /odom -> noisy_odom -> /odom_noisy   (realistic encoder drift)
    /odom_noisy + /scan -> slam_node -> /pose_slam + TF map->base_link

and every control node (mapping, planning, navigation, mission) is REMAPPED to
read /pose_slam where it used to read /odom. Not one line of the control stack
changes -- localisation is swapped underneath it, which is the whole argument:
the same software runs wherever the robot is dropped.

    ros2 launch youbot_slam gazebo_slam.launch.py            # SLAM corrects
    ros2 launch youbot_slam gazebo_slam.launch.py slam:=false pose_topic:=odom_noisy
        # failure demo: the stack runs on the drifting odometry alone, gets lost.
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

    res_path = os.pathsep.join(
        [str(gz_share.parent), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]).strip(os.pathsep)

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    use_slam = LaunchConfiguration("slam")
    pose_topic = LaunchConfiguration("pose_topic")
    calib_mode = LaunchConfiguration("calib")
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

    # Control nodes read the estimated pose instead of ground truth. The topic
    # name inside each node is still "odom" -- only the wiring changes.
    # navigation and mission both drive the base; both go through the guard.
    GUARDED = ("navigation_node", "mission_node")

    def control(executable, localized=False):
        remaps = [("odom", pose_topic)] if localized else []
        if executable in GUARDED:
            remaps = remaps + [("cmd_vel", "cmd_vel_raw")]
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params, sim_time],
                    remappings=remaps)

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true",
                              description="true: scan matching corrects the drifting "
                                          "odometry. false: the stack runs on the "
                                          "drifting odometry alone (failure demo)."),
        DeclareLaunchArgument("pose_topic", default_value="pose_slam",
                              description="Pose source for the control stack. Pass "
                                          "odom_noisy with slam:=false for the "
                                          "failure demo."),
        DeclareLaunchArgument("calib", default_value="auto",
                              description="auto: calibrate on the first run, "
                                          "then apply the saved bias with no "
                                          "reference at all. calibrate/apply "
                                          "force one or the other. Staying in "
                                          "calibrate hands SLAM a "
                                          "ground-truth-assisted prior it "
                                          "cannot beat."),

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

        # --- the level-3 layer -------------------------------------------------
        Node(package="youbot_slam", executable="noisy_odom", name="noisy_odom",
             output="screen", parameters=[sim_time]),
        # Bias calibration: in a straight aisle the lidar cannot constrain X
        # (aperture problem), so odometry's systematic scale bias integrates
        # straight into the pose. Learn it once with the reference, then run
        # with calib:=apply and no reference at all.
        Node(package="youbot_slam", executable="odom_calibrator",
             name="odom_calibrator", output="screen",
             parameters=[{"mode": calib_mode}, sim_time]),
        Node(package="youbot_slam", executable="slam_node", name="slam_node",
             output="screen", parameters=[sim_time],
             remappings=[("odom_noisy", "odom_calibrated")],
             condition=IfCondition(use_slam)),
        # Failure demo (slam:=false): TF comes from the drifting odometry and the
        # control stack is pointed at it via pose_topic:=odom_noisy.
        Node(package="youbot_control", executable="odom_tf", name="odom_tf",
             output="screen", parameters=[sim_time],
             remappings=[("odom", "odom_noisy")],
             condition=UnlessCondition(use_slam)),

        # Ground truth vs belief, on one RViz picture + one scoreboard.
        Node(package="youbot_slam", executable="truth_monitor",
             name="truth_monitor", output="screen", parameters=[sim_time]),

        # --- the UNCHANGED control stack, rewired to the SLAM pose -------------
        control("safety_node", localized=True),   # /cmd_vel_raw -> /cmd_vel
        control("mapping_node", localized=True),
        control("planning_node", localized=True),
        control("navigation_node", localized=True),
        control("mission_node", localized=True),
        control("strawberry_detector"),
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

        RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(
                cmd=["bash", "-c", 'pkill -9 -f "gz sim"; pkill -9 -f "gz-sim"; '
                     'pkill -9 -f "ruby.*gz sim"; pkill -9 -f parameter_bridge; '
                     'pkill -9 -f rviz2; pkill -9 -f rqt_image_view'],
                output="screen"),
        ])),
    ])
