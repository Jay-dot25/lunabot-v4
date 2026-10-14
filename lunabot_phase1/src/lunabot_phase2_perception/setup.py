from glob import glob
from pathlib import Path
from os.path import join
from setuptools import setup

package_name = "lunabot_phase2_perception"


def _install_tree(source):
    data = []
    source_path = Path(source)
    if not source_path.exists():
        return data
    for path in sorted(source_path.rglob("*")):
        if path.is_file():
            relative_parent = path.parent.relative_to(source_path)
            target = Path("share") / package_name / source_path.name / relative_parent
            data.append((str(target), [str(path)]))
    return data


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [join("resource", package_name)]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        *_install_tree("worlds"),
        *_install_tree("models"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LunaBot contributors",
    maintainer_email="lunabot@example.com",
    description="Phase 2 dual-head terrain and shadow inference for the Phase 1 camera.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "eyes_of_luna_node = lunabot_phase2_perception.eyes_of_luna_node:main",
        ],
    },
)
