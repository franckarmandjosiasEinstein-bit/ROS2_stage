from setuptools import find_packages, setup

package_name = "youbot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias Einstein",
    maintainer_email="josiasarmand40@gmail.com",
    description="Autonomous YouBot control nodes (mapping, planning, navigation, mission).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapping_node = youbot_control.mapping_node:main",
            "planning_node = youbot_control.planning_node:main",
            "navigation_node = youbot_control.navigation_node:main",
            "mission_node = youbot_control.mission_node:main",
            "vision_node = youbot_control.vision_node:main",
            "strawberry_detector = youbot_control.strawberry_detector:main",
            "odom_tf = youbot_control.odom_tf:main",
            "arm_node = youbot_control.arm_node:main",
            "sim_node = youbot_control.sim_node:main",
        ],
    },
)
