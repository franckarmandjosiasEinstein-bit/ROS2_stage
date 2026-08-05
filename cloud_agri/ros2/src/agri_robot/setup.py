"""Install the node, and the two GENERATED assets it cannot run without.

The world and the URDF are produced by agri.world.make_world and
agri.world.make_robot from cloud_agri/. They are copied into this package's
share directory at build time so that an installed workspace is
self-contained -- otherwise `ros2 launch` works from the source tree and
fails everywhere else, which is the kind of difference that only shows up on
the machine you are demonstrating on.

If they are missing, the build still succeeds and the LAUNCH FILE says what
to run. Failing the build instead would mean a fresh clone cannot be built
until the generators have been run, which is a worse first impression than a
clear error at launch.
"""

import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "agri_robot"
HERE = Path(__file__).resolve().parent
# cloud_agri/, three levels up from ros2/src/agri_robot/
PROJECT = HERE.parents[2]


def generated(kind: str, pattern: str) -> list:
    """The generated assets, as paths RELATIVE to this file's directory.

    Relative is not a style choice. colcon asserts it:

        AssertionError: 'data_files' must be relative,
        '/.../cloud_agri/worlds/greenhouse_cloud.sdf' is absolute

    -- and it asserts it at BUILD time, so an absolute path here does not
    produce a subtly wrong install, it produces a package that will not
    build at all. The paths climb out of the package with '../../..'
    because the world and the robot belong to cloud_agri, not to this ROS
    package: they are generated from the catalogue and are shared with the
    Cloud and the test suite, which have no idea ROS exists.
    """
    files = sorted(os.path.relpath(p, HERE)
                   for p in (PROJECT / kind).glob(pattern))
    return [(f"share/{package_name}/{kind}", files)] if files else []


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        *generated("worlds", "*.sdf"),
        *generated("urdf", "*.urdf"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias",
    maintainer_email="josiasarmand40@gmail.com",
    description="Smart-agriculture mobile node: red-cross docking, sealed "
                "plant measurements, MQTT to a Cloud.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_node = agri_robot.robot_node:main",
        ],
    },
)
