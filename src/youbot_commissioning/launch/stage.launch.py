"""Launch one commissioning stage with the right profile and parameter file.

    ros2 launch youbot_commissioning stage.launch.py stage:=1
    ros2 launch youbot_commissioning stage.launch.py stage:=6 profile:=hardware

The point of going through a launch file rather than `ros2 run` is that the
profile and the parameter file travel together. Selecting `profile:=hardware`
without also loading limits.yaml would give you the hardware LABEL on a report
produced with simulation THRESHOLDS -- a report that says "PASS, hardware" and
means nothing. That is exactly the class of mistake that cost this project a
run, so it is made structurally impossible here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

STAGES = {
    "0": "stage0_estop",
    "1": "stage1_wheels",
    "2": "stage2_umbmark",
    "3": "stage3_lidar",
    "4": "stage4_manual_map",
    "5": "stage5_localisation",
    "6": "stage6_navigation",
    "7": "stage7_dataset",
    "9": "stage9_pick",
    # stage 8 is offline and takes command-line arguments, not ROS parameters:
    #   ros2 run youbot_commissioning stage8_detector -- --labels ... --predict ...
}


def _launch(context, *args, **kwargs):
    stage = LaunchConfiguration("stage").perform(context)
    profile = LaunchConfiguration("profile").perform(context)

    if stage not in STAGES:
        raise RuntimeError(
            f"unknown stage {stage!r}. Valid: {', '.join(sorted(STAGES))} "
            "(stage 8 is offline: ros2 run youbot_commissioning stage8_detector)")
    if profile not in ("sim", "hardware"):
        raise RuntimeError(f"profile must be 'sim' or 'hardware', not {profile!r}")

    share = FindPackageShare("youbot_commissioning").perform(context)
    params = os.path.join(share, "config", "limits.yaml")

    return [Node(
        package="youbot_commissioning",
        executable=STAGES[stage],
        name=STAGES[stage],
        output="screen",
        emulate_tty=True,          # so the printed PROCEDURE keeps its layout
        parameters=[params, {"profile": profile}],
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stage", description="0..7 or 9"),
        DeclareLaunchArgument(
            "profile", default_value="sim",
            description="'sim' to rehearse, 'hardware' on the real robot"),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        OpaqueFunction(function=_launch),
    ])
