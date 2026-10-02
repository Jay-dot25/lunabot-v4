#!/usr/bin/env python3
"""Validate LunaBot ROS 2 package foundations without requiring ROS installation."""

from __future__ import annotations

import ast
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
EXPECTED = {
    "lunabot_common": "ament_python",
    "lunabot_msgs": "ament_cmake",
    "lunabot_gazebo": "ament_cmake",
    "lunabot_perception": "ament_python",
    "lunabot_mapping": "ament_python",
    "lunabot_planning": "ament_python",
    "lunabot_control": "ament_python",
    "lunabot_evaluation": "ament_python",
    "lunabot_bringup": "ament_python",
    "lunabot_localization": "ament_python",
}


def fail(message: str) -> None:
    raise ValueError(message)


def package_metadata(path: Path) -> tuple[str, str]:
    xml_path = path / "package.xml"
    root = ET.parse(xml_path).getroot()
    name_node = root.find("name")
    version_node = root.find("version")
    license_node = root.find("license")
    maintainer_node = root.find("maintainer")
    export_node = root.find("export")
    build_node = export_node.find("build_type") if export_node is not None else None
    if name_node is None or not name_node.text:
        fail(f"{xml_path}: missing package name")
    if version_node is None or not (version_node.text or "").strip():
        fail(f"{xml_path}: package version missing")
    version_parts = (version_node.text or "").strip().split(".")
    if len(version_parts) != 3 or not all(part.isdigit() for part in version_parts):
        fail(f"{xml_path}: version must use numeric MAJOR.MINOR.PATCH")
    if license_node is None or not (license_node.text or "").strip():
        fail(f"{xml_path}: missing license")
    if maintainer_node is None or not maintainer_node.attrib.get("email"):
        fail(f"{xml_path}: maintainer email missing")
    if build_node is None or not build_node.text:
        fail(f"{xml_path}: build type missing")
    return name_node.text.strip(), build_node.text.strip()


def validate_python_package(path: Path, name: str) -> None:
    for relative in ("setup.py", "setup.cfg", f"resource/{name}", f"{name}/__init__.py"):
        if not (path / relative).is_file():
            fail(f"{name}: missing {relative}")
    ast.parse((path / "setup.py").read_text(encoding="utf-8"), filename=str(path / "setup.py"))
    for source in (path / name).glob("*.py"):
        ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    proc = subprocess.run(
        [sys.executable, "setup.py", "--name"], cwd=path,
        capture_output=True, text=True, check=False, timeout=30,
    )
    if proc.returncode != 0:
        fail(f"{name}: setup.py --name failed: {proc.stderr.strip()}")
    if proc.stdout.strip().splitlines()[-1] != name:
        fail(f"{name}: setup.py reports {proc.stdout.strip()!r}")
    setup_text = (path / "setup.py").read_text(encoding="utf-8")
    if "ament_index/resource_index/packages" not in setup_text:
        fail(f"{name}: ament resource index installation missing")
    cfg = (path / "setup.cfg").read_text(encoding="utf-8")
    if f"lib/{name}" not in cfg:
        fail(f"{name}: ROS script install directory missing")


def validate_cmake_package(path: Path, name: str) -> None:
    cmake = path / "CMakeLists.txt"
    if not cmake.is_file():
        fail(f"{name}: CMakeLists.txt missing")
    text = cmake.read_text(encoding="utf-8")
    for required in (f"project({name})", "find_package(ament_cmake REQUIRED)", "ament_package()"):
        if required not in text:
            fail(f"{name}: CMakeLists.txt missing {required!r}")


def validate_all() -> list[str]:
    discovered: dict[str, Path] = {}
    details: list[str] = []
    for xml_path in sorted(SRC.glob("*/package.xml")):
        path = xml_path.parent
        name, build_type = package_metadata(path)
        if name in discovered:
            fail(f"duplicate ROS package name: {name}")
        discovered[name] = path
        expected_type = EXPECTED.get(name)
        if expected_type is None:
            fail(f"unexpected ROS package: {name}")
        if build_type != expected_type:
            fail(f"{name}: expected {expected_type}, found {build_type}")
        if build_type == "ament_python":
            validate_python_package(path, name)
        else:
            validate_cmake_package(path, name)
        details.append(f"{name}:{build_type}")
    missing = sorted(set(EXPECTED) - set(discovered))
    if missing:
        fail("missing ROS packages: " + ", ".join(missing))
    if len(discovered) != len(EXPECTED):
        fail(f"expected {len(EXPECTED)} packages, found {len(discovered)}")

    launch_file = SRC / "lunabot_bringup/launch/foundation.launch.py"
    ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))
    if not (SRC / "lunabot_bringup/config/package_foundation.yaml").is_file():
        fail("bringup foundation parameter file missing")
    for letter in "abcdefghijkl":
        if not (ROOT / f"launch-{letter}").is_file():
            fail(f"legacy launcher launch-{letter} was not preserved")
    return details


def main() -> int:
    try:
        details = validate_all()
    except (OSError, ValueError, ET.ParseError, subprocess.TimeoutExpired) as exc:
        print(f"ROS_PACKAGE_FOUNDATION_FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"ROS_PACKAGE_FOUNDATION_PASS packages={len(details)}")
    for detail in details:
        print(f"  {detail}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
