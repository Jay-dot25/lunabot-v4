from setuptools import find_packages, setup

package_name = "lunabot_common"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/terrain_classes.yaml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="LunaBot Team", maintainer_email="maintainers@lunabot.invalid",
    description="Shared dependency-light LunaBot configuration and data contracts.",
    license="Apache-2.0", tests_require=["pytest"],
    entry_points={"console_scripts": ["lunabot_common_info = lunabot_common.package_info:main"]},
)
