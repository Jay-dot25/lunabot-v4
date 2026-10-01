#!/usr/bin/env python3
"""Phase 2 ROS package foundation tests that do not require ROS."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "validate_ros_packages", ROOT / "tools/validate_ros_packages.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RosPackageFoundationTests(unittest.TestCase):
    def test_all_expected_packages_validate(self):
        details = MODULE.validate_all()
        self.assertEqual(len(details), 9)
        self.assertEqual({item.split(":")[0] for item in details}, set(MODULE.EXPECTED))

    def test_python_package_modules_import_without_ros(self):
        packages = [name for name, kind in MODULE.EXPECTED.items()
                    if kind == "ament_python" and name != "lunabot_common"]
        inserted = []
        try:
            for name in packages:
                path = str(ROOT / "src" / name)
                sys.path.insert(0, path)
                inserted.append(path)
                module = __import__(f"{name}.package_info", fromlist=["PACKAGE_NAME"])
                self.assertEqual(module.PACKAGE_NAME, name)
                self.assertEqual(module.PACKAGE_VERSION, "0.1.0")
        finally:
            for path in inserted:
                sys.path.remove(path)
            for name in packages:
                sys.modules.pop(name, None)
                sys.modules.pop(f"{name}.package_info", None)

    def test_common_canonical_and_compatibility_load_same_schema(self):
        from lunabot_common import load_terrain_config as compatibility_loader
        from src.lunabot_common.lunabot_common import load_terrain_config as canonical_loader

        compatibility = compatibility_loader()
        canonical = canonical_loader()
        self.assertEqual(compatibility.schema_version, canonical.schema_version)
        self.assertEqual(compatibility.classes, canonical.classes)

    def test_bringup_installs_launch_and_config(self):
        setup = (ROOT / "src/lunabot_bringup/setup.py").read_text(encoding="utf-8")
        self.assertIn("launch/foundation.launch.py", setup)
        self.assertIn("config/package_foundation.yaml", setup)

    def test_gazebo_package_installs_assets(self):
        cmake = (ROOT / "src/lunabot_gazebo/CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("install(DIRECTORY models worlds", cmake)

    def test_phase_three_message_interfaces_exist(self):
        message_dir = ROOT / "src/lunabot_msgs/msg"
        self.assertEqual(
            {path.name for path in message_dir.glob("*.msg")},
            {"TerrainPrediction.msg", "PlannerStatus.msg", "ReplanEvent.msg",
             "SafetyStatus.msg", "MissionMetrics.msg"},
        )


if __name__ == "__main__":
    unittest.main()
