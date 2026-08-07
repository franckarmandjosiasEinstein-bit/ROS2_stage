"""Bring up the greenhouse, the robot, and the mobile node.

    ros2 launch agri_robot agri.launch.py
    ros2 launch agri_robot agri.launch.py gui:=false               # headless
    ros2 launch agri_robot agri.launch.py targets:="P1,1R;P2,4R"   # no Cloud

WHAT IS STARTED

    gz sim                  worlds/greenhouse_cloud.sdf -- the Phase B
                            greenhouse plus the 48 red crosses
    robot_state_publisher   urdf/youbot_agri.urdf -- the Phase B youbot plus
                            the sensor boom and its floor camera
    ros_gz_bridge           config/gz_bridge_agri.yaml
    agri_robot              the node: MQTT in, wheels and cameras out

The Cloud is NOT started here. It is a separate process -- on a separate
machine in any deployment worth the name -- and starting it from the robot's
launch file would quietly make that untrue:

    agri-cloud --keys keys --store store

WHERE THE WORLD AND THE ROBOT COME FROM

Both are GENERATED (agri.world.make_world, agri.world.make_robot) and both
are installed into this package's share directory by setup.py, so a colcon
install is self-contained. If they are missing, that means the generators
have not been run, and this file says so and stops rather than letting gz
open an empty world and leaving you to wonder where the greenhouse went.

The robot's meshes still belong to Phase B's youbot_gazebo package and are
referenced as package://youbot_gazebo/..., so that package's share directory
is put on GZ_SIM_RESOURCE_PATH -- the same thing the Phase B launch file
does, and the reason the base is a CAD body rather than a grey box.

SHUTDOWN

Phase B's kill_sim.sh is reused verbatim when it is installed. gz sim forks
processes that outlive a plain SIGINT to the launcher; the symptom is a
second run that starts, finds the world already loaded, and behaves
inexplicably. That script was written for exactly this and there is no
reason to write a second one.
"""

import os
from pathlib import Path

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = "worlds/greenhouse_cloud.sdf"
URDF = "urdf/youbot_agri.urdf"


def _source_root() -> Path | None:
    """cloud_agri/ in a source tree, for running without installing.

    Three ways, in order of how much they can be trusted:

      AGRI_HOME       said explicitly by whoever is launching. Always wins.
      walking up      works from a source checkout, and from an install made
                      with --symlink-install, because __file__ then resolves
                      back through the symlink into the source tree.
      (nothing)       a plain colcon install, where the only copy is the one
                      setup.py placed in share/.

    The environment variable exists because that last case depends on
    setup.py's data_files having worked, and that is exactly the kind of
    thing that behaves differently on one machine. One export turns a
    mysterious empty world into a fixed problem.
    """
    named = os.environ.get("AGRI_HOME")
    if named and (Path(named) / WORLD).exists():
        return Path(named)
    for parent in Path(__file__).resolve().parents:
        if (parent / WORLD).exists():
            return parent
    return None


def _asset(rel: str, share: Path) -> Path:
    """The source tree if we can find it, else the installed copy."""
    for base in (_source_root(), share):
        if base is not None and (base / rel).exists():
            return base / rel
    raise RuntimeError(
        f"cannot find {rel}.\n"
        f"  looked in {share}\n"
        f"  and walked up from {Path(__file__).resolve().parent}\n"
        "\n"
        "If cloud_agri/ is somewhere this cannot see, say so:\n"
        "    export AGRI_HOME=/path/to/cloud_agri\n"
        "\n"
        "If the file was never generated, generate it, from cloud_agri/:\n"
        "    python3 -m agri.world.make_world\n"
        "    python3 -m agri.world.make_robot")


def _share(package: str) -> Path | None:
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError:
        return None


def _python_path(src: Path | None) -> str:
    """PYTHONPATH for the robot node, because it will NOT inherit yours.

    colcon generates the console script with the shebang of the interpreter
    that ran setup.py -- and colcon itself is a system package, so that is
    /usr/bin/python3 even when a virtual environment is active. The node
    therefore starts under the SYSTEM python, which has never heard of agri,
    and dies on `from agri import keys` a second after the simulator opens.

    Two additions fix it, and neither is a hack:

      the source tree   agri/ is a plain package directory, importable by
                        any interpreter that can see cloud_agri/.
      VIRTUAL_ENV       where qrcode, zxing-cpp and paho actually live. An
                        editable install's .pth file is NOT read from
                        PYTHONPATH, which is why the source tree is added
                        directly rather than relied upon through pip.
    """
    parts = []
    if src is not None:
        parts.append(str(src))
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        parts += sorted(str(p) for p in Path(venv).glob("lib/python3*/site-packages"))
    inherited = os.environ.get("PYTHONPATH", "")
    if inherited:
        parts.append(inherited)
    return os.pathsep.join(parts)


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("agri_robot"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = str(_asset(WORLD, share))
    robot_desc = _asset(URDF, share).read_text()
    bridge_cfg = str(share / "config" / "gz_bridge_agri.yaml")

    # package://youbot_gazebo/meshes/... -> the share directory ABOVE
    # youbot_gazebo, which is what gz resolves package:// against.
    paths = [str(p.parent) for p in (_share("youbot_gazebo"),) if p]
    src = _source_root()
    if src is not None:                     # source tree: meshes live here
        paths.append(str(src.parent / "src"))
    res_path = os.pathsep.join(
        paths + [os.environ.get("GZ_SIM_RESOURCE_PATH", "")]).strip(os.pathsep)

    use_gui = LaunchConfiguration("gui")
    sim_time = {"use_sim_time": True}
    default_keys = str((src or share) / "keys")
    py_path = _python_path(src)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v3 ", world]}.items(),
        condition=IfCondition(use_gui))
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v3 ", world]}.items(),
        condition=UnlessCondition(use_gui))

    actions = [
        # THE LINE WHOSE ABSENCE MAKES THE ROBOT INVISIBLE.
        #
        # res_path was computed correctly and then never applied, so gz
        # could not resolve model://youbot_gazebo/meshes/base.stl and the
        # robot rendered as nothing at all -- while every other part of the
        # launch reported success. A variable that is computed and unused is
        # the one kind of bug that reads as finished work.
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path),

        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open the 2D lidar view beside Gazebo. This is where "
                        "a robot that stopped for no visible reason explains "
                        "itself."),
        DeclareLaunchArgument("broker", default_value="localhost"),
        DeclareLaunchArgument("keys", default_value=default_keys),
        DeclareLaunchArgument(
            "targets", default_value="",
            description='Semicolon-separated stations to visit with NO Cloud, '
                        'e.g. "P1,1R;P2,4R". Diagnostic only: the reports are '
                        'built and discarded.'),

        gz, gz_headless,

        Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_desc}, sim_time]),

        # Spawn in the west headland of the south aisle, facing +x. The
        # sensor boom then reaches x = -4.08, which is exactly where
        # driver.route() expects the robot to be when it sets off.
        Node(package="ros_gz_sim", executable="create", name="spawn_youbot",
             output="screen",
             arguments=["-topic", "robot_description", "-name", "youbot",
                        "-x", "-4.58", "-y", "-1.85", "-z", "0.0"]),

        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="gz_bridge", output="screen",
             parameters=[{"config_file": bridge_cfg}, sim_time]),

        # TF map->base_link, and the catalogue's own view of the greenhouse
        # as markers. RViz needs the first to draw anything at all.
        Node(package="agri_robot", executable="viz_node", name="agri_viz",
             output="screen", parameters=[sim_time]),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             output="log", parameters=[sim_time],
             arguments=["-d", str(share / "config" / "agri.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),

        Node(package="agri_robot", executable="robot_node", name="agri_robot",
             output="screen",
             additional_env={"PYTHONPATH": py_path},
             parameters=[{"broker": LaunchConfiguration("broker"),
                          "keys": LaunchConfiguration("keys"),
                          "standalone_targets": LaunchConfiguration("targets")},
                         sim_time]),
    ]

    bringup = _share("youbot_bringup")
    kill_sim = bringup / "scripts" / "kill_sim.sh" if bringup else None
    if kill_sim is not None and kill_sim.exists():
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(cmd=["bash", str(kill_sim), "--now", "--quiet"],
                           output="screen")])))

    return LaunchDescription(actions)
