#!/usr/bin/env python3
"""Shape/loss/interface tests; synthetic fixtures are not model-quality evidence."""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ML = ROOT / "ml"
sys.path.insert(0, str(ML))

from lunabot_ml.eyes_of_luna_loss import multitask_loss  # noqa: E402
from lunabot_ml.eyes_of_luna_metrics import (  # noqa: E402
    confusion_matrix,
    shadow_report,
    terrain_report,
)
from lunabot_ml.eyes_of_luna_model import build_model, load_model_config  # noqa: E402
from lunabot_ml.eyes_of_luna_runtime import (  # noqa: E402
    EyesOfLunaInferenceError,
    config_sha256,
    file_sha256,
    infer_rgb,
    load_checkpoint,
    load_exported_model,
    save_prediction_masks,
)


@unittest.skipUnless(importlib.util.find_spec("torch"), "PyTorch is not installed")
class EyesOfLunaModelTests(unittest.TestCase):
    def test_default_config_and_model_output_contract(self):
        import torch

        config = load_model_config()
        config["model"]["base_channels"] = 4
        model = build_model(config).eval()
        image = torch.zeros(1, 3, 32, 48)
        with torch.no_grad():
            terrain, shadow = model(image)
        self.assertEqual(tuple(terrain.shape), (1, 4, 32, 48))
        self.assertEqual(tuple(shadow.shape), (1, 1, 32, 48))
        self.assertFalse(torch.allclose(terrain[:, :1], shadow))

    def test_exported_model_reloads_with_both_heads(self):
        import torch

        config = load_model_config()
        config["model"]["base_channels"] = 4
        model = build_model(config).eval()
        image = torch.rand(1, 3, 32, 48)
        exported = torch.export.export(model, (image,))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test-only.pt2"
            torch.export.save(exported, path)
            reloaded = torch.export.load(path).module()
            expected = model(image)
            actual = reloaded(image)
        self.assertEqual(len(actual), 2)
        self.assertTrue(torch.allclose(expected[0], actual[0], atol=1e-5))
        self.assertTrue(torch.allclose(expected[1], actual[1], atol=1e-5))

    def test_export_bundle_checksums_and_reload_in_temporary_directory(self):
        import numpy as np
        import torch

        config = load_model_config()
        config["model"]["base_channels"] = 4
        config["training"]["input_size"] = [24, 32]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = build_model(config).eval()
            exported = torch.export.export(model, (torch.zeros(1, 3, 24, 32),))
            model_path = root / "test-only.pt2"
            torch.export.save(exported, model_path)
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n",
                                  encoding="utf-8")
            bundle = {
                "format": "eyes_of_luna_export_v1",
                "model_file": model_path.name,
                "model_sha256": file_sha256(model_path),
                "config_file": config_path.name,
                "config_file_sha256": file_sha256(config_path),
                "config_sha256": config_sha256(config),
            }
            bundle_path = root / "deployment.json"
            bundle_path.write_text(json.dumps(bundle), encoding="utf-8")
            loaded, loaded_config, _bundle, device = load_exported_model(bundle_path)
            result = infer_rgb(loaded, np.zeros((19, 27, 3), dtype=np.uint8),
                               loaded_config, device)
            self.assertEqual(result["terrain_mask"].shape, (19, 27))
            self.assertEqual(result["shadow_mask"].shape, (19, 27))

    def test_multitask_loss_masks_ignore_pixels_and_backpropagates(self):
        import torch

        terrain_logits = torch.randn(1, 4, 8, 8, requires_grad=True)
        shadow_logits = torch.randn(1, 1, 8, 8, requires_grad=True)
        terrain_target = torch.arange(64).reshape(1, 8, 8) % 4
        shadow_target = torch.zeros((1, 8, 8), dtype=torch.long)
        # A shadow may overlap terrain class 0; the heads stay independent.
        shadow_target[0, 0, 0] = 1
        terrain_target[0, 0, 1] = 255
        shadow_target[0, 1, 0] = 255
        loss, components = multitask_loss(
            terrain_logits, shadow_logits, terrain_target, shadow_target)
        self.assertTrue(torch.isfinite(loss))
        self.assertIn("terrain_cross_entropy", components)
        self.assertIn("shadow_binary_cross_entropy", components)
        loss.backward()
        self.assertTrue(torch.isfinite(terrain_logits.grad).all())
        self.assertTrue(torch.isfinite(shadow_logits.grad).all())
        self.assertGreater(float(terrain_logits.grad.abs().sum()), 0.0)
        self.assertGreater(float(shadow_logits.grad.abs().sum()), 0.0)

    def test_all_ignored_targets_produce_zero_finite_loss(self):
        import torch

        terrain_logits = torch.randn(1, 4, 4, 4, requires_grad=True)
        shadow_logits = torch.randn(1, 1, 4, 4, requires_grad=True)
        ignored = torch.full((1, 4, 4), 255, dtype=torch.long)
        loss, _ = multitask_loss(terrain_logits, shadow_logits, ignored, ignored)
        self.assertTrue(torch.isfinite(loss))
        self.assertEqual(float(loss.detach()), 0.0)
        loss.backward()
        self.assertEqual(float(terrain_logits.grad.abs().sum()), 0.0)
        self.assertEqual(float(shadow_logits.grad.abs().sum()), 0.0)

    def test_standalone_preprocessing_returns_both_masks_at_source_size(self):
        import numpy as np

        config = load_model_config()
        config["model"]["base_channels"] = 4
        config["training"]["input_size"] = [24, 32]
        model = build_model(config).eval()
        rgb = np.zeros((19, 27, 3), dtype=np.uint8)
        output = infer_rgb(model, rgb, config, device="cpu")
        self.assertEqual(output["terrain_mask"].shape, (19, 27))
        self.assertEqual(output["shadow_mask"].shape, (19, 27))
        self.assertEqual(output["terrain_confidence"].shape, (19, 27))
        self.assertEqual(set(np.unique(output["terrain_mask"])).issubset({0, 1, 2, 3}), True)
        self.assertEqual(set(np.unique(output["shadow_mask"])).issubset({0, 1}), True)
        self.assertGreaterEqual(output["latency_ms"], 0.0)

    def test_saved_machine_masks_preserve_terrain_and_shadow_ids(self):
        import numpy as np
        from PIL import Image

        terrain = np.array([[0, 1], [2, 3]], dtype=np.uint8)
        shadow = np.array([[0, 1], [1, 0]], dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            paths = save_prediction_masks(Path(directory) / "frame", terrain, shadow)
            terrain_saved = np.asarray(Image.open(paths["terrain_mask"]))
            shadow_saved = np.asarray(Image.open(paths["shadow_mask"]))
            preview_saved = np.asarray(Image.open(paths["shadow_visualization"]))
        self.assertEqual(set(np.unique(terrain_saved)), {0, 1, 2, 3})
        self.assertEqual(set(np.unique(shadow_saved)), {0, 1})
        self.assertEqual(set(np.unique(preview_saved)), {0, 255})
        self.assertTrue(np.array_equal(shadow_saved, shadow))

    def test_missing_checkpoint_fails_without_fabricating_an_artifact(self):
        config = load_model_config()
        with self.assertRaisesRegex(EyesOfLunaInferenceError, "checkpoint missing"):
            load_checkpoint(ROOT / "models" / "eyes_of_luna.pt", config)

    def test_confusion_metrics_report_missing_heldout_classes(self):
        # Known synthetic values test metric math only; this is not an evaluation result.
        terrain = confusion_matrix([0, 1, 3], [0, 1, 3], 4)
        result = terrain_report(terrain)
        self.assertEqual(result["missing_classes"], ["rock_boulder"])
        self.assertIsNone(result["per_class"][2]["iou"])
        shadow = confusion_matrix([0, 1, 0], [0, 1, 1], 2)
        shadow_result = shadow_report(shadow)
        self.assertAlmostEqual(shadow_result["recall"], 0.5)
        self.assertEqual(shadow_result["false_negative"], 1)

    def test_training_refuses_to_start_without_real_dataset_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "training-output"
            completed = subprocess.run(
                [sys.executable, str(ML / "train_eyes_of_luna.py"),
                 "--dataset", str(root / "missing-dataset"),
                 "--output", str(output)], capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 1)
            self.assertIn("manifest missing", completed.stdout)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
