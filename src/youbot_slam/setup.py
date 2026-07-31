from glob import glob

from setuptools import setup

package_name = "youbot_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias Einstein",
    maintainer_email="josiasarmand40@gmail.com",
    description="Level-3 autonomy: homemade online SLAM + slam_toolbox comparison.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "noisy_odom = youbot_slam.noisy_odom:main",
            "slam_node = youbot_slam.slam_node:main",
            "pose_from_tf = youbot_slam.pose_from_tf:main",
        ],
    },
)
