from setuptools import find_packages, setup

package_name = "lunabot_phase1_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot Team",
    maintainer_email="maintainers@lunabot.invalid",
    description="Phase 1 command watchdog, manual terminal driver and LiDAR status monitor.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_guard = lunabot_phase1_tools.command_guard_node:main",
            "obstacle_monitor = lunabot_phase1_tools.obstacle_monitor_node:main",
            "keyboard_teleop = lunabot_phase1_tools.keyboard_teleop:main",
        ],
    },
)
