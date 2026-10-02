from setuptools import find_packages,setup
package_name="lunabot_localization"
setup(name=package_name,version="0.1.0",packages=find_packages(exclude=("test",)),
 data_files=[("share/ament_index/resource_index/packages",["resource/"+package_name]),("share/"+package_name,["package.xml"]),("share/"+package_name+"/config",["config/ekf.yaml","config/slam_toolbox_filtered.yaml"]),("share/"+package_name+"/launch",["launch/localization.launch.py"])],
 install_requires=["setuptools"],zip_safe=True,maintainer="LunaBot Team",maintainer_email="maintainers@lunabot.invalid",description="GPS-denied localization and evaluation.",license="Apache-2.0",
 entry_points={"console_scripts":["lunabot_localization_info = lunabot_localization.package_info:main","localization_evaluator = lunabot_localization.localization_evaluator:main"]})
