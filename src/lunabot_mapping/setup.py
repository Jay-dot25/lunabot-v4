from setuptools import find_packages, setup

package_name = "lunabot_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/traversability.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot Team",
    maintainer_email="maintainers@lunabot.invalid",
    description="LunaBot semantic mapping and traversability foundations.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "lunabot_mapping_info = lunabot_mapping.package_info:main",
        "semantic_fusion_node = lunabot_mapping.semantic_fusion_node:main",
        "traversability_node = lunabot_mapping.traversability_node:main",
    ]},
)
