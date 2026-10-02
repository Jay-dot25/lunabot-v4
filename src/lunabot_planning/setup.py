from setuptools import find_packages, setup

package_name = "lunabot_planning"

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
    description="LunaBot global and incremental planning foundations.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "lunabot_planning_info = lunabot_planning.package_info:main",
    ]},
)
