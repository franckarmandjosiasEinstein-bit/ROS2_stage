from glob import glob

from setuptools import setup

package_name = "youbot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franck Armand Josias Einstein",
    maintainer_email="josiasarmand40@gmail.com",
    description="Launch files and parameters that bring up the YouBot control stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
