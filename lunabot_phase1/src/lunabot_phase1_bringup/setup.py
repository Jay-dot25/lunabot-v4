from glob import glob
import os

from setuptools import setup

package_name = "lunabot_phase1_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot Team",
    maintainer_email="maintainers@lunabot.invalid",
    description="Integrated launch and ROS-Gazebo bridges for LunaBot Phase 1.",
    license="Apache-2.0",
)
