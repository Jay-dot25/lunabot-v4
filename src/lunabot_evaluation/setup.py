from setuptools import find_packages, setup

package_name = "lunabot_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot Team",
    maintainer_email="maintainers@lunabot.invalid",
    description="LunaBot mission metrics and experiment foundations.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "lunabot_evaluation_info = lunabot_evaluation.package_info:main",
        "status_compat_bridge = lunabot_evaluation.status_compat_bridge:main",
        "mission_metrics = lunabot_evaluation.mission_metrics_node:main",
    ]},
)
