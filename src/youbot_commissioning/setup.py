from glob import glob

from setuptools import setup

package_name = "youbot_commissioning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".lib"],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias Einstein",
    maintainer_email="josiasarmand40@gmail.com",
    description="Staged bring-up acceptance tests for the real greenhouse.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stage0_estop = youbot_commissioning.stage0_estop:main",
            "stage1_wheels = youbot_commissioning.stage1_wheels:main",
            "stage2_umbmark = youbot_commissioning.stage2_umbmark:main",
            "stage3_lidar = youbot_commissioning.stage3_lidar:main",
            "stage4_manual_map = youbot_commissioning.stage4_manual_map:main",
            "stage5_localisation = youbot_commissioning.stage5_localisation:main",
            "stage6_navigation = youbot_commissioning.stage6_navigation:main",
            "stage7_dataset = youbot_commissioning.stage7_dataset:main",
            "stage8_detector = youbot_commissioning.stage8_detector:main",
            "stage9_pick = youbot_commissioning.stage9_pick:main",
        ],
    },
)
