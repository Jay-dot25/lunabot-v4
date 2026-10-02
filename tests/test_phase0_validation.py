#!/usr/bin/env python3
"""Unit tests for the Phase 0 validation harness."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("validate_all", ROOT / "tools/validate_all.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ValidationHarnessTests(unittest.TestCase):
    def test_repository_config_loads(self):
        config = MODULE.load_config(ROOT / "config/validation.yaml")
        self.assertEqual(config["schema_version"], 1)
        self.assertEqual(len(config["phase_validators"]), 12)
        self.assertEqual(config["config_validators"], [
            "tools/validate_terrain_config.py",
            "tools/validate_ros_packages.py",
            "tools/validate_interfaces.py",
            "tools/validate_dataset_tools.py",
            "tools/validate_scenarios.py",
            "tools/validate_ground_truth.py",
            "tools/validate_ml_framework.py",
            "tools/validate_ml_inference.py",
            "tools/validate_localization.py",
            "tools/validate_semantic_fusion.py",
        ])

    def test_config_rejects_missing_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps({"python_roots": []}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing"):
                MODULE.load_config(path)

    def test_config_rejects_invalid_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text("not valid", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "invalid"):
                MODULE.load_config(path)

    def test_result_schema(self):
        item = MODULE.result("example", "unit", "pass", "ok", 0.1234)
        self.assertEqual(item["status"], "pass")
        self.assertEqual(item["duration_seconds"], 0.123)

    def test_all_configured_validators_exist(self):
        config = MODULE.load_config(ROOT / "config/validation.yaml")
        validators = config["phase_validators"] + config.get("config_validators", [])
        missing = [item for item in validators if not (ROOT / item).is_file()]
        self.assertEqual(missing, [])

    def test_source_discovery_excludes_generated_build_trees(self):
        files = MODULE.discover_files(["src"], "*.py")
        for path in files:
            self.assertNotIn("build", path.relative_to(ROOT).parts)
            self.assertNotIn("install", path.relative_to(ROOT).parts)


if __name__ == "__main__":
    unittest.main()
