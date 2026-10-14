#!/usr/bin/env python3
"""Tests for Phase 2 data contracts; every image here is synthetic test data only."""
from __future__ import annotations

import hashlib
import json
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ML = ROOT / "ml"
sys.path.insert(0, str(ML))

from lunabot_ml.dataset_manifest import DatasetError, write_mask_png  # noqa: E402
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    collect_sample,
    load_task_config,
    save_capture_group_split,
    split_by_capture_group,
    validate_dataset,
    validate_rgb_png,
)


def write_rgb_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    """Test-only PNG writer; its arbitrary pixels are not training annotations."""
    if len(pixels) != width * height * 3:
        raise ValueError("incorrect test RGB byte count")

    def chunk(name: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + name + payload
                + struct.pack(">I", zlib.crc32(name + payload) & 0xFFFFFFFF))

    rows = b"".join(b"\x00" + pixels[y * width * 3:(y + 1) * width * 3]
                    for y in range(height))
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
                     + chunk(b"IDAT", zlib.compress(rows, 9))
                     + chunk(b"IEND", b""))


class EyesOfLunaDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.dataset = self.root / "dataset"
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def metadata(group_id: str, method: str = "manual") -> dict:
        annotation = {
            "method": method,
            "review_status": "human_reviewed",
            "reviewer_id": "test-fixture-reviewer",
            "reviewed_at_utc": "2026-10-14T12:00:00+00:00",
        }
        if method == "manual":
            annotation["annotation_tool"] = "test-only-fixture"
        else:
            annotation["generator"] = {
                "implementation": "test-only-semantic-camera",
                "version": "fixture-1",
                "class_mapping": "test-only",
            }
        return {
            "capture": {
                "camera_topic": "/camera/image_raw",
                "timestamp_ns": 1_000_000_000,
                "world_id": f"world-{group_id}",
                "session_id": f"session-{group_id}",
                "source": "synthetic_unit_test_fixture",
                "simulator": {
                    "name": "test-fixture",
                    "version": "0",
                    "renderer": "test-fixture",
                },
                "camera_pose_ground_truth": [0.5, 0, 0.5, 0, 0, 0, 1],
                "rover_pose_ground_truth": [0, 0, 0, 0, 0, 0, 1],
                "rover_pose_odom": [0.01, 0, 0, 0, 0, 0, 1],
                "lighting": {"fixture": "not a real lighting measurement"},
            },
            "annotations": {"terrain": dict(annotation), "shadow": dict(annotation)},
        }

    def make_inputs(self, suffix: str, terrain: bytes | None = None,
                    shadow: bytes | None = None, width: int = 4,
                    height: int = 3) -> tuple[Path, Path, Path]:
        rgb = self.inputs / f"rgb-{suffix}.png"
        terrain_path = self.inputs / f"terrain-{suffix}.png"
        shadow_path = self.inputs / f"shadow-{suffix}.png"
        salt = hashlib.sha256(suffix.encode("utf-8")).digest()
        rgb_pixels = bytes((index + salt[index % len(salt)]) % 256
                           for index in range(width * height * 3))
        write_rgb_png(rgb, width, height, rgb_pixels)
        terrain = terrain if terrain is not None else bytes([0, 1, 2, 3] * (width * height // 4))
        shadow = shadow if shadow is not None else bytes([0, 1] * (width * height // 2))
        write_mask_png(terrain_path, width, height, terrain)
        write_mask_png(shadow_path, width, height, shadow)
        return rgb, terrain_path, shadow_path

    def add(self, sample_id: str, group_id: str,
            terrain: bytes | None = None, shadow: bytes | None = None):
        rgb, terrain_path, shadow_path = self.make_inputs(sample_id, terrain, shadow)
        return collect_sample(self.dataset, sample_id, group_id, rgb, terrain_path,
                              shadow_path, self.metadata(group_id))

    def test_task_config_declares_independent_heads_and_phase1_camera(self):
        config = load_task_config()
        self.assertEqual(config["camera"]["topic"], "/camera/image_raw")
        self.assertEqual(config["terrain_head"]["output_channels"], 4)
        self.assertEqual(config["shadow_head"]["output_channels"], 1)
        self.assertTrue(config["shadow_head"]["independent_of_terrain"])

    def test_register_and_validate_two_independent_masks(self):
        terrain = bytes([0, 1, 2, 3] * 3)
        shadow = bytes([1, 0, 0, 1] * 3)
        record = self.add("frame-001", "group-001", terrain, shadow)
        self.assertEqual(record["class_pixel_counts"]["terrain"],
                         {"0": 3, "1": 3, "2": 3, "3": 3})
        self.assertEqual(record["class_pixel_counts"]["shadow"], {"0": 6, "1": 6})
        report = validate_dataset(self.dataset)
        self.assertTrue(report["valid"], report["errors"])
        self.assertTrue(report["coverage_complete"])
        self.assertEqual(report["sample_count"], 1)

    def test_shadow_can_overlap_any_terrain_class(self):
        terrain = bytes([0, 1, 2, 3] * 3)
        shadow = bytes([1, 0, 0, 0] * 3)
        self.add("overlap-001", "group-001", terrain, shadow)
        report = validate_dataset(self.dataset)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(report["terrain_pixel_counts"]["0"], 3)
        self.assertEqual(report["shadow_pixel_counts"]["1"], 3)

    def test_rejects_unreviewed_or_incomplete_annotation_metadata(self):
        rgb, terrain, shadow = self.make_inputs("unreviewed")
        metadata = self.metadata("group-001")
        metadata["annotations"]["shadow"]["review_status"] = "unreviewed"
        with self.assertRaisesRegex(DatasetError, "not marked human_reviewed"):
            collect_sample(self.dataset, "unreviewed", "group-001", rgb, terrain,
                           shadow, metadata)
        self.assertFalse((self.dataset / "manifest.json").exists())

    def test_semantic_camera_requires_implementation_details_and_human_review(self):
        rgb, terrain, shadow = self.make_inputs("semantic")
        metadata = self.metadata("group-001", "semantic_camera")
        del metadata["annotations"]["terrain"]["generator"]["implementation"]
        with self.assertRaisesRegex(DatasetError, "generator.implementation"):
            collect_sample(self.dataset, "semantic", "group-001", rgb, terrain,
                           shadow, metadata)

    def test_rejects_unknown_mask_ids(self):
        with self.assertRaisesRegex(DatasetError, "invalid class IDs"):
            self.add("bad-terrain", "group-001", bytes([4] * 12), bytes([0, 1] * 6))
        with self.assertRaisesRegex(DatasetError, "invalid class IDs"):
            self.add("bad-shadow", "group-001", bytes([0, 1, 2, 3] * 3), bytes([2] * 12))

    def test_rejects_dimension_mismatch(self):
        rgb, terrain, shadow = self.make_inputs("dimensions")
        write_mask_png(shadow, 2, 2, bytes([0, 1, 0, 1]))
        with self.assertRaisesRegex(DatasetError, "dimensions differ"):
            collect_sample(self.dataset, "dimensions", "group-001", rgb, terrain,
                           shadow, self.metadata("group-001"))

    def test_rgb_png_is_fully_decoded_and_corruption_is_rejected(self):
        rgb, _, _ = self.make_inputs("rgb")
        self.assertEqual(validate_rgb_png(rgb), (4, 3))
        rgb.write_bytes(b"not a PNG")
        with self.assertRaisesRegex(DatasetError, "not a PNG"):
            validate_rgb_png(rgb)

    def test_duplicate_sample_and_duplicate_rgb_are_rejected(self):
        self.add("frame-001", "group-001")
        rgb, terrain, shadow = self.make_inputs("different-frame")
        with self.assertRaisesRegex(DatasetError, "duplicate sample_id"):
            collect_sample(self.dataset, "frame-001", "group-002", rgb, terrain,
                           shadow, self.metadata("group-002"))
        record = self.add("frame-002", "group-002")
        copied_rgb = self.dataset / record["paths"]["rgb"]
        with self.assertRaisesRegex(DatasetError, "duplicate RGB image"):
            collect_sample(self.dataset, "frame-003", "group-003", copied_rgb,
                           self.inputs / "terrain-frame-002.png",
                           self.inputs / "shadow-frame-002.png", self.metadata("group-003"))

    def test_checksum_tampering_is_detected(self):
        record = self.add("frame-001", "group-001")
        path = self.dataset / record["paths"]["terrain_mask"]
        path.write_bytes(path.read_bytes() + b"tamper")
        report = validate_dataset(self.dataset)
        self.assertFalse(report["valid"])
        self.assertTrue(any("checksum mismatch for terrain_mask" in error
                            for error in report["errors"]))

    def test_split_is_reproducible_and_keeps_capture_groups_separate(self):
        for index in range(6):
            self.add(f"frame-{index:03d}", f"group-{index:03d}")
        first = split_by_capture_group(self.dataset, seed=17)
        second = split_by_capture_group(self.dataset, seed=17)
        self.assertEqual(first, second)
        group_sets = [set(first["splits"][name]["groups"])
                      for name in ("train", "validation", "test")]
        sample_sets = [set(first["splits"][name]["samples"])
                       for name in ("train", "validation", "test")]
        self.assertTrue(all(group_sets))
        self.assertEqual(set.union(*group_sets), {f"group-{index:03d}" for index in range(6)})
        self.assertTrue(all(not (group_sets[i] & group_sets[j])
                            for i in range(3) for j in range(i + 1, 3)))
        self.assertTrue(all(not (sample_sets[i] & sample_sets[j])
                            for i in range(3) for j in range(i + 1, 3)))
        output = save_capture_group_split(self.dataset, first)
        self.assertTrue(output.is_file())
        self.assertEqual(json.loads(output.read_text()) , first)

    def test_split_requires_three_capture_groups(self):
        self.add("frame-001", "group-001")
        with self.assertRaisesRegex(DatasetError, "at least three capture groups"):
            split_by_capture_group(self.dataset)

    def test_split_writer_rejects_capture_group_leakage(self):
        for index in range(3):
            self.add(f"frame-{index:03d}", f"group-{index:03d}")
        split = split_by_capture_group(self.dataset)
        split["splits"]["validation"]["groups"].append(
            split["splits"]["train"]["groups"][0])
        with self.assertRaisesRegex(DatasetError, "capture-group leakage"):
            save_capture_group_split(self.dataset, split)

    def test_empty_dataset_is_not_reported_valid(self):
        rgb, terrain, shadow = self.make_inputs("unused")
        collect_sample(self.dataset, "frame-001", "group-001", rgb, terrain,
                       shadow, self.metadata("group-001"))
        (self.dataset / "manifest.json").unlink()
        from lunabot_ml.eyes_of_luna_dataset import _empty_manifest, _save_manifest
        _save_manifest(self.dataset, _empty_manifest())
        report = validate_dataset(self.dataset)
        self.assertFalse(report["valid"])
        self.assertIn("dataset contains no samples", report["errors"])

    def test_collection_cli_registers_reviewed_test_fixture(self):
        rgb, terrain, shadow = self.make_inputs("cli-collect")
        metadata_path = self.root / "metadata.json"
        metadata_path.write_text(json.dumps(self.metadata("group-cli")), encoding="utf-8")
        completed = subprocess.run(
            [sys.executable, str(ML / "collect_eyes_of_luna_sample.py"),
             "--dataset", str(self.dataset), "--sample-id", "frame-cli",
             "--group-id", "group-cli", "--rgb", str(rgb),
             "--terrain-mask", str(terrain), "--shadow-mask", str(shadow),
             "--metadata", str(metadata_path)],
            capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("EYES_OF_LUNA_COLLECTION_PASS", completed.stdout)

    def test_validation_cli_reports_success_for_test_fixture(self):
        self.add("frame-001", "group-001")
        completed = subprocess.run(
            [sys.executable, str(ML / "validate_eyes_of_luna_dataset.py"),
             "--dataset", str(self.dataset)],
            capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("EYES_OF_LUNA_VALIDATION_PASS", completed.stdout)
        self.assertIn("semantic correctness is not machine-verified", completed.stdout)

    def test_split_cli_persists_capture_group_split(self):
        for index in range(3):
            self.add(f"frame-{index:03d}", f"group-{index:03d}")
        completed = subprocess.run(
            [sys.executable, str(ML / "split_eyes_of_luna_dataset.py"),
             "--dataset", str(self.dataset), "--seed", "11"],
            capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("EYES_OF_LUNA_SPLIT_PASS", completed.stdout)
        output = self.dataset / "splits" / "capture_group_split.json"
        self.assertTrue(output.is_file())


if __name__ == "__main__":
    unittest.main()
