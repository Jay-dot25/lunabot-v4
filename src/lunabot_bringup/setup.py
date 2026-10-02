from setuptools import find_packages, setup

package_name = "lunabot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/foundation.launch.py", "launch/perception.launch.py", "launch/lunabot_goal.launch.py"]),
        ("share/" + package_name + "/config", ["config/package_foundation.yaml", "config/goal_system.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/goal_system.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot Team",
    maintainer_email="maintainers@lunabot.invalid",
    description="LunaBot launch and shared configuration package.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "lunabot_bringup_info = lunabot_bringup.package_info:main",
    ]},
)
