from glob import glob

from setuptools import setup

package_name = "youbot_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf") + glob("worlds/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/meshes", glob("meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias Einstein",
    maintainer_email="josiasarmand40@gmail.com",
    description="Gazebo Harmonic digital twin of the strawberry greenhouse for the YouBot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_model_node = youbot_gazebo.drive_model_node:main",
        ],
    },
)
