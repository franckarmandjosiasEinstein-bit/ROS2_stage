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
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
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
    ': gui_follow watchdog; '        # marker: kill_sim.sh finds it by this
    # DIE WITH THE LAUNCH. This loop is the one process that reliably outlived
    # a Ctrl-C: bash sitting in `sleep 20` does not act on a signal until the
    # sleep returns, and if launch is killed outright (terminal closed, SIGKILL)
    # its OnShutdown handler never runs at all, so nothing ever asks. Checking
    # that our parent is still there costs nothing and makes the leak
    # self-healing even when the tidy path is skipped entirely.
    'parent=$PPID; '
    'gone() { ! kill -0 "$parent" 2>/dev/null; }; '
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
    # A CAMERA POSE THAT DOES NOT DEPEND ON FINDING THE ROBOT.
    #
    # Everything above asks the GUI to look at an ENTITY, and the GUI answers
    # "Target: 'youbot' not found" whenever its render scene has not caught up
    # with the server. That is a race, it is lost perhaps one run in three,
    # and when it is lost the camera stays wherever it started -- which, in a
    # closed 10 x 5 m greenhouse with no <gui> block in the world, is a view
    # of a wall. The robot was driving perfectly and the operator could not
    # see it, which is not a small thing when watching the run IS the test.
    #
    # /gui/move_to/pose takes a pose instead of a name, so it cannot fail for
    # want of an entity. (0, -7, 11) looking at the origin clears the 2.5 m
    # south wall by 1.4 m and sees the whole floor through the transparent
    # roof, so the robot is in frame wherever it is. Tracking still overrides
    # this the moment it works; this is the floor, not the ceiling.
    #
    # Deliberately NOT a <gui> block in greenhouse.sdf: declaring one replaces
    # the default plugin set, and getting that list wrong costs the Entity
    # Tree and the play/pause controls -- a worse problem than the one being
    # fixed.
    'ovr=\'pose: {position: {x: 0, y: -7, z: 11}, '
    'orientation: {x: -0.3403, y: 0.3403, z: 0.6199, w: 0.6199}}\'; '
    'overview() { gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera '
    '  --reptype gz.msgs.Boolean --timeout 2000 --req "$ovr" >/dev/null 2>&1; }; '
    'locked=0; '
    'for i in $(seq 1 40); do '
    '  if gone; then exit 0; fi; '
    '  overview; moveto; track; '   # frame the arena first, then try to lock on
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
    '  if gone; then exit 0; fi; '    # the launch is over; so are we
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

    world = str(gz_share / "worlds" / "greenhouse.sdf")
    robot_desc = (gz_share / "urdf" / "youbot_gz.urdf").read_text()
    bridge_cfg = str(gz_share / "config" / "gz_bridge.yaml")
    params = str(bringup_share / "config" / "youbot_params.yaml")
    # Every process this launch file starts, and every process THOSE
    # start, is stopped by this one script on shutdown. See its header.
    KILL_SIM = str(bringup_share / "scripts" / "kill_sim.sh")
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

    slam_node = Node(
        package="youbot_slam", executable="slam_node", name="slam_node",
        output="screen", parameters=[sim_time],
        remappings=[("odom_noisy", "odom_calibrated")],
        condition=IfCondition(use_slam))

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),
        DeclareLaunchArgument(
            "drive_model", default_value="false",
            description="true = give the base a real drivetrain (latency, "
                        "lag, wheel limits, slip). false = kinematic, as "
                        "measured in the report."),
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
        slam_node,
        # Without a pose there is no mission, no map and no harvest: the
        # whole run is already over, and the only thing left to decide is
        # whether you find out now or after ninety seconds of warnings
        # buried in Gazebo output. Stop the launch and say why.
        RegisterEventHandler(OnProcessExit(
            target_action=slam_node,
            # Only when it CRASHED. A clean exit here is slam_node responding
            # to the Ctrl-C that is already shutting everything down, and
            # calling Shutdown() again from inside a shutdown re-runs the
            # cleanup action below -- "ExecuteLocal action: executed more than
            # once" -- while printing an alarming "slam_node died" over a
            # perfectly normal exit.
            on_exit=lambda event, context: (
                [LogInfo(msg="slam_node died -- no pose, so nothing "
                             "downstream can run. Stopping the whole stack; "
                             "its traceback is above."),
                 Shutdown(reason="slam_node died")]
                if event.returncode not in (0, None) else []))),
        # slam:=false. TF now follows pose_topic rather than being nailed to
        # odom_noisy, so the two configurations this argument is actually used
        # for are both coherent:
        #
        #   pose_topic:=odom_noisy      the failure demo -- the stack runs on
        #                               raw drifting odometry and gets lost.
        #   pose_topic:=odom_calibrated the working harvest -- calibrated
        #                               odometry, no ground truth at runtime.
        #
        # With the topic hard-coded, the second one put the control stack on
        # the calibrated pose while RViz and every TF consumer saw the raw
        # one: the robot behaved correctly and the picture disagreed with it,
        # which is the worst way to run a demo.
        Node(package="youbot_control", executable="odom_tf", name="odom_tf",
             output="screen", parameters=[sim_time],
             remappings=[("odom", pose_topic)],
             condition=UnlessCondition(use_slam)),

        # Ground truth vs belief, on one RViz picture + one scoreboard.
        Node(package="youbot_slam", executable="truth_monitor",
             name="truth_monitor", output="screen", parameters=[sim_time]),

        # Scores the map the robot built against the real greenhouse: interior
        # dimensions, plant-row positions, coverage. Reads the world file and
        # publishes nothing -- a measuring instrument, not part of the loop.
        Node(package="youbot_slam", executable="map_eval",
             name="map_eval", output="screen", parameters=[sim_time]),

        # The third axis of the evaluation, after the pose and the map: TIME.
        # Lap times, distance, throughput, and the share of the run spent
        # asked-to-move-but-not-moving. A correct harvester that takes all day
        # is not a harvester.
        Node(package="youbot_slam", executable="perf_monitor",
             name="perf_monitor", output="screen", parameters=[sim_time]),

        # --- the UNCHANGED control stack, rewired to the SLAM pose -------------
        # THE DRIVETRAIN. Sits between /cmd_vel and Gazebo and models what a
        # kinematic VelocityControl has none of: command latency, motor lag,
        # per-wheel acceleration and speed limits (which DISTORT a twist, not
        # merely scale it), and roller slip. Disabled by default, so this is a
        # pass-through and the measured baseline is unchanged. It must always
        # be present: the bridge listens on /cmd_vel_exec, which is its output.
        Node(package="youbot_gazebo", executable="drive_model_node",
             name="drive_model_node", output="screen",
             parameters=[{"enabled": LaunchConfiguration("drive_model")},
                         sim_time]),

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
            # And it calls ONE script, shared by all three launch files.
            # The inline pkill list that used to live here missed the two
            # things that actually leak: the gui_follow watchdog (an infinite
            # `while sleep 20` loop, which does not act on a signal until the
            # sleep returns) and the gz sim server and GUI, which ros_gz_sim
            # forks from a ruby wrapper -- launch kills the wrapper and the
            # two forks keep the world alive. Duplicating the patterns three
            # times is how they drifted apart in the first place.
            on_shutdown=lambda event, context: [ExecuteProcess(
                cmd=["bash", KILL_SIM, "--now", "--quiet"], output="screen")])),
    ])
