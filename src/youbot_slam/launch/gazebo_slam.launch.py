"""Level-3 autonomy bringup: the full Gazebo stack, localised by SLAM.

Same digital twin as youbot_gazebo/gazebo.launch.py, but the control stack no
longer receives the ground-truth pose. One of two SLAM backends corrects the
drifting odometry:

    slam_backend:=toolbox  (default, recommended for real robot)
        /odom -> noisy_odom (publish_tf) -> /odom_noisy + TF odom->base_link
        /scan + TF -> slam_toolbox -> TF map->odom
        TF map->base_link -> pose_from_tf -> /pose_slam

    slam_backend:=custom
        /odom -> noisy_odom -> /odom_noisy
        /odom_noisy -> odom_calibrator -> /odom_calibrated
        /odom_calibrated + /scan -> slam_node -> /pose_slam + TF map->base_link

    slam_backend:=none (failure demo / calibrated odometry only)
        /odom -> noisy_odom -> /odom_noisy -> odom_calibrator -> /odom_calibrated
        Stack runs on calibrated odometry alone.

Detection backend (default: colour threshold, optional: YOLOv8):
    detector:=colour    colour-threshold (default, no extra deps)
    detector:=yolo      YOLOv8 model (requires ultralytics + weights)

Usage:
    ros2 launch youbot_slam gazebo_slam.launch.py                       # slam_toolbox + colour
    ros2 launch youbot_slam gazebo_slam.launch.py slam_backend:=custom  # custom SLAM
    ros2 launch youbot_slam gazebo_slam.launch.py slam_backend:=none pose_topic:=odom_calibrated
    ros2 launch youbot_slam gazebo_slam.launch.py detector:=yolo yolo_weights:=/path/to/best.pt
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


FOLLOW_ROBOT = (
    ': gui_follow watchdog; '
    'parent=$PPID; '
    'gone() { ! kill -0 "$parent" 2>/dev/null; }; '
    'req=\'data: "youbot"\'; '
    'moveto() { gz service -s /gui/move_to --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    'track() { gz topic -t /gui/track -m gz.msgs.TrackVisual '
    '  -p \'name: "youbot", inherit_orientation: true, min_dist: 3.0, '
    'max_dist: 9.0\' >/dev/null 2>&1; '
    '  gz service -s /gui/follow --reqtype gz.msgs.StringMsg '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$req" >/dev/null 2>&1; }; '
    'tracked() { timeout 3 gz topic -e -t /gui/currently_tracked -n 1 '
    '  2>/dev/null | grep -q youbot; }; '
    'ovr=\'pose: {position: {x: 0, y: -7, z: 11}, '
    'orientation: {x: -0.3403, y: 0.3403, z: 0.6199, w: 0.6199}}\'; '
    'overview() { gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$ovr" >/dev/null 2>&1; }; '
    'locked=0; '
    'for i in $(seq 1 40); do '
    '  if gone; then exit 0; fi; '
    '  overview; moveto; track; '
    '  if tracked; then locked=1; break; fi; sleep 1; done; '
    'if [ "$locked" = 1 ]; then '
    '  echo "[view] Gazebo camera is following the robot (confirmed)."; '
    'else '
    '  overview; '
    '  echo "[view] the Gazebo camera would not lock onto the robot, so it is '
    'parked on an overview of the whole greenhouse instead -- the robot IS '
    'there, small and yellow. To follow it: Entity Tree > right-click youbot '
    '> Follow."; fi; '
    'quiet=0; '
    'while sleep 20; do '
    '  if gone; then exit 0; fi; '
    '  track; '
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
    KILL_SIM = str(bringup_share / "scripts" / "kill_sim.sh")
    rviz_cfg = str(bringup_share / "config" / "youbot.rviz")
    toolbox_cfg = str(slam_share / "config" / "slam_toolbox.yaml")

    res_path = os.pathsep.join(
        [str(gz_share.parent), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]).strip(os.pathsep)

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    slam_backend = LaunchConfiguration("slam_backend")
    pose_topic = LaunchConfiguration("pose_topic")
    calib_mode = LaunchConfiguration("calib")
    detector_backend = LaunchConfiguration("detector")
    yolo_weights = LaunchConfiguration("yolo_weights")
    sim_time = {"use_sim_time": True}

    # Conditions for SLAM backends
    use_toolbox = IfCondition(PythonExpression(["'", slam_backend, "' == 'toolbox'"]))
    use_custom = IfCondition(PythonExpression(["'", slam_backend, "' == 'custom'"]))
    use_none = IfCondition(PythonExpression(["'", slam_backend, "' == 'none'"]))

    # Conditions for detector backends
    use_yolo = IfCondition(PythonExpression(["'", detector_backend, "' == 'yolo'"]))
    use_colour = IfCondition(PythonExpression(["'", detector_backend, "' == 'colour'"]))

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

    GUARDED = ("navigation_node", "mission_node")

    def control(executable, localized=False):
        remaps = [("odom", pose_topic)] if localized else []
        if executable in GUARDED:
            remaps = remaps + [("cmd_vel", "cmd_vel_raw")]
        return Node(package="youbot_control", executable=executable, name=executable,
                    output="screen", parameters=[params, sim_time],
                    remappings=remaps)

    # --- slam_toolbox backend nodes ---
    toolbox_noisy_odom = Node(
        package="youbot_slam", executable="noisy_odom", name="noisy_odom",
        output="screen", parameters=[{"publish_tf": True}, sim_time],
        condition=use_toolbox)
    toolbox_slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen",
        parameters=[toolbox_cfg, sim_time],
        condition=use_toolbox)
    toolbox_pose = Node(
        package="youbot_slam", executable="pose_from_tf", name="pose_from_tf",
        output="screen", parameters=[sim_time],
        condition=use_toolbox)

    # --- custom SLAM backend nodes ---
    custom_noisy_odom = Node(
        package="youbot_slam", executable="noisy_odom", name="noisy_odom",
        output="screen", parameters=[sim_time],
        condition=use_custom)
    custom_calibrator = Node(
        package="youbot_slam", executable="odom_calibrator",
        name="odom_calibrator", output="screen",
        parameters=[{"mode": calib_mode}, sim_time],
        condition=use_custom)
    slam_node = Node(
        package="youbot_slam", executable="slam_node", name="slam_node",
        output="screen", parameters=[sim_time],
        remappings=[("odom_noisy", "odom_calibrated")],
        condition=use_custom)

    # --- no-SLAM mode nodes ---
    none_noisy_odom = Node(
        package="youbot_slam", executable="noisy_odom", name="noisy_odom",
        output="screen", parameters=[sim_time],
        condition=use_none)
    none_calibrator = Node(
        package="youbot_slam", executable="odom_calibrator",
        name="odom_calibrator", output="screen",
        parameters=[{"mode": calib_mode}, sim_time],
        condition=use_none)
    none_tf = Node(
        package="youbot_control", executable="odom_tf", name="odom_tf",
        output="screen", parameters=[sim_time],
        remappings=[("odom", "odom_calibrated")],
        condition=use_none)

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),
        DeclareLaunchArgument(
            "drive_model", default_value="false",
            description="true = give the base a real drivetrain."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("slam_backend", default_value="toolbox",
                              description="SLAM backend: 'toolbox' (recommended, "
                                          "graph-based with loop closure), "
                                          "'custom' (correlative scan matcher), "
                                          "'none' (no SLAM, calibrated odometry only)."),
        DeclareLaunchArgument("pose_topic", default_value="pose_slam",
                              description="Pose source for the control stack. Both "
                                          "SLAM backends publish /pose_slam. Pass "
                                          "'odom_calibrated' with slam_backend:=none."),
        DeclareLaunchArgument("calib", default_value="auto",
                              description="Odometry calibration mode for custom/none "
                                          "backends. Ignored for slam_toolbox."),
        DeclareLaunchArgument("detector", default_value="colour",
                              description="Detection backend: 'colour' (threshold, "
                                          "default) or 'yolo' (YOLOv8 model)."),
        DeclareLaunchArgument("yolo_weights", default_value="",
                              description="Path to a YOLOv8 .pt weights file. "
                                          "Used when detector:=yolo."),

        gz_sim,
        gz_sim_headless,

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_desc}, sim_time]),

        Node(package="ros_gz_sim", executable="create", name="spawn_youbot",
             output="screen",
             arguments=["-topic", "robot_description", "-name", "youbot",
                        "-x", "-4.40", "-y", "1.85", "-z", "0.0"]),

        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             output="screen",
             parameters=[{"config_file": bridge_cfg}, sim_time]),

        # --- SLAM backends (only one set is active) -----------------------
        toolbox_noisy_odom, toolbox_slam, toolbox_pose,
        custom_noisy_odom, custom_calibrator, slam_node,
        none_noisy_odom, none_calibrator, none_tf,

        RegisterEventHandler(OnProcessExit(
            target_action=slam_node,
            on_exit=lambda event, context: (
                [LogInfo(msg="slam_node died -- stopping the stack."),
                 Shutdown(reason="slam_node died")]
                if event.returncode not in (0, None) else []))),

        # --- evaluation tools (always present) ----------------------------
        Node(package="youbot_slam", executable="truth_monitor",
             name="truth_monitor", output="screen", parameters=[sim_time]),
        Node(package="youbot_slam", executable="map_eval",
             name="map_eval", output="screen", parameters=[sim_time]),
        Node(package="youbot_slam", executable="perf_monitor",
             name="perf_monitor", output="screen", parameters=[sim_time]),

        # --- the control stack, reading the SLAM pose ---------------------
        Node(package="youbot_gazebo", executable="drive_model_node",
             name="drive_model_node", output="screen",
             parameters=[{"enabled": LaunchConfiguration("drive_model")},
                         sim_time]),
        control("safety_node", localized=True),
        control("mapping_node", localized=True),
        control("planning_node", localized=True),
        control("navigation_node", localized=True),
        control("mission_node", localized=True),
        control("camera_pan_node"),
        Node(package="youbot_control", executable="strawberry_detector",
             name="strawberry_detector", output="screen",
             parameters=[params, sim_time],
             remappings=[("odom", pose_topic)],
             condition=use_colour),
        Node(package="youbot_control", executable="yolo_detector",
             name="strawberry_detector", output="screen",
             parameters=[params, {"weights": yolo_weights}, sim_time],
             remappings=[("odom", pose_topic)],
             condition=use_yolo),
        control("arm_node", localized=True),

        TimerAction(period=6.0, actions=[ExecuteProcess(
            cmd=["bash", "-c", FOLLOW_ROBOT], output="screen",
            condition=IfCondition(use_gui))]),

        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_cfg], parameters=[sim_time],
             condition=IfCondition(use_rviz)),

        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: [ExecuteProcess(
                cmd=["bash", KILL_SIM, "--now", "--quiet"], output="screen")])),
    ])
