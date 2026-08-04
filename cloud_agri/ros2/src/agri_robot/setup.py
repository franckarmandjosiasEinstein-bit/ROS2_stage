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

from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "agri_robot"
# cloud_agri/, four levels up from ros2/src/agri_robot/setup.py
PROJECT = Path(__file__).resolve().parents[3]


def generated(kind: str, pattern: str) -> list:
    files = sorted(str(p) for p in (PROJECT / kind).glob(pattern))
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
