#!/usr/bin/env python3
"""Phase 5 terrain ground-truth and scenario tests."""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

try:
    import numpy as np
except ImportError:  # static-only hosts may not have the terrain dependency
    np = None

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(ROOT / "ml"))

from lunabot_ml.dataset_manifest import read_mask_png  # noqa: E402
from scenario_schema import (  # noqa: E402
    ScenarioError, load_scenario, normalized_runtime_manifest, validate_scenario)
from validate_scenarios import validate_all as validate_all_scenarios  # noqa: E402

TERRAIN = None
if np is not None:
    SPEC = importlib.util.spec_from_file_location(
        "terrain_generator", TOOLS / "generate_lunar_terrain.py")
    TERRAIN = importlib.util.module_from_spec(SPEC)
    assert SPEC.loader is not None
    SPEC.loader.exec_module(TERRAIN)


class SimulationGroundTruthTests(unittest.TestCase):
    def test_scenario_suite_validates(self):
        identifiers = validate_all_scenarios()
        self.assertEqual(set(identifiers),
                         {"baseline_habitat", "dynamic_obstacle", "low_sun"})

    def test_runtime_manifest_forbids_navigation_ground_truth(self):
        scenario = load_scenario(ROOT / "config/scenarios/dynamic_obstacle.json")
        runtime = normalized_runtime_manifest(scenario)
        self.assertTrue(runtime["ground_truth"]["evaluation_only"])
        self.assertFalse(runtime["ground_truth"]["navigation_allowed"])
        self.assertTrue(any(obj["dynamic"] for obj in runtime["objects"]))

    def test_invalid_scenario_semantic_id_rejected(self):
        scenario = load_scenario(ROOT / "config/scenarios/baseline_habitat.json")
        invalid = copy.deepcopy(scenario)
        invalid["objects"][0]["semantic_id"] = 99
        with self.assertRaisesRegex(ScenarioError, "semantic_id"):
            validate_scenario(invalid)

    def test_dynamic_object_requires_trigger(self):
        scenario = load_scenario(ROOT / "config/scenarios/dynamic_obstacle.json")
        invalid = copy.deepcopy(scenario)
        del invalid["objects"][0]["insertion_trigger"]
        with self.assertRaisesRegex(ScenarioError, "insertion_trigger"):
            validate_scenario(invalid)

    def test_ground_truth_navigation_must_be_false(self):
        scenario = load_scenario(ROOT / "config/scenarios/baseline_habitat.json")
        invalid = copy.deepcopy(scenario)
        invalid["ground_truth_navigation_allowed"] = True
        with self.assertRaisesRegex(ScenarioError, "must be false"):
            validate_scenario(invalid)

    @unittest.skipUnless(np is not None, "numpy is required by terrain generator")
    def test_crater_generation_is_deterministic(self):
        first = TERRAIN.make_craters(np.random.default_rng(42), 0.2)
        second = TERRAIN.make_craters(np.random.default_rng(42), 0.2)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 1 + 14 + 44)

    @unittest.skipUnless(np is not None, "numpy is required by terrain generator")
    def test_crater_density_validation(self):
        with self.assertRaisesRegex(ValueError, "density"):
            TERRAIN.make_craters(np.random.default_rng(1), 0.0)

    @unittest.skipUnless(np is not None, "numpy is required by terrain generator")
    def test_semantic_layer_precedence_and_ranges(self):
        axis = np.linspace(-20, 20, 9)
        x, y = np.meshgrid(axis, axis)
        z = np.zeros((9, 9), dtype=float)
        normals = np.zeros((9, 9, 3), dtype=float)
        normals[..., 2] = 1.0
        craters = [(20.0, 20.0, 0.8, 0.2, "small")]
        labels, slope, roughness, illumination = TERRAIN.semantic_layers(
            x, y, z, normals, craters, 1.0)
        self.assertEqual(labels.dtype, np.uint8)
        self.assertTrue(set(np.unique(labels)) <= {1, 2, 3, 6, 7})
        self.assertEqual(labels[-1, -1], 6)
        self.assertEqual(labels[4, 4], 1)  # spawn pad dominates
        self.assertTrue(np.all(np.isfinite(slope)))
        self.assertTrue(np.all(np.isfinite(roughness)))
        self.assertTrue(np.all(np.isfinite(illumination)))

    @unittest.skipUnless(np is not None, "numpy is required by terrain generator")
    def test_ground_truth_artifacts_and_checksums(self):
        axis = np.linspace(-20, 20, 8)
        x, y = np.meshgrid(axis, axis)
        z = 0.01 * x + 0.02 * y
        normals = np.zeros((8, 8, 3), dtype=float)
        normals[..., 2] = 1.0
        craters = [(15.0, 15.0, 3.0, 0.5, "small")]
        with tempfile.TemporaryDirectory() as directory:
            metadata_path = Path(directory) / "terrain.json"
            mask_path = Path(directory) / "semantic.png"
            metadata = TERRAIN.write_ground_truth(
                str(metadata_path), str(mask_path), x, y, z, normals,
                craters, 5.0, 42, 1.0)
            self.assertTrue(metadata["evaluation_only"])
            self.assertEqual(metadata["frame_id"], "map")
            self.assertEqual(read_mask_png(mask_path)[:2], (8, 8))
            for relative in metadata["layers"].values():
                self.assertTrue((Path(directory) / relative).is_file())
            stored = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(stored["sha256"], metadata["sha256"])

    def test_camera_info_contract_is_installed(self):
        setup = (ROOT / "src/lunabot_perception/setup.py").read_text(encoding="utf-8")
        source = (ROOT / "src/lunabot_perception/lunabot_perception/camera_info_publisher.py").read_text(encoding="utf-8")
        self.assertIn("config/camera_info.yaml", setup)
        self.assertIn("camera_info_publisher", setup)
        self.assertIn("/lunabot/camera/camera_info", source)
        self.assertIn("/lunabot/depth/camera_info", source)
        matrix_lengths = {}
        for node in ast.walk(ast.parse(source)):
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Attribute)
                    and node.targets[0].attr in {"k", "r", "p"}
                    and isinstance(node.value, ast.List)):
                matrix_lengths[node.targets[0].attr] = len(node.value.elts)
        self.assertEqual(matrix_lengths, {"k": 9, "r": 9, "p": 12})


if __name__ == "__main__":
    unittest.main()
