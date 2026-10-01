#!/usr/bin/env python3
"""Phase 4 dataset collection, integrity, and split tests."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ML = ROOT / "ml"
sys.path.insert(0, str(ML))

from lunabot_ml.dataset_manifest import (  # noqa: E402
    DatasetError,
    add_sample,
    load_manifest,
    png_info,
    read_mask_png,
    split_by_world,
    validate_dataset,
    validate_metadata,
    write_mask_png,
)


class DatasetToolsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.dataset = self.root / "dataset"
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def metadata(timestamp: int = 1_000_000_000):
        return {
            "rgb_timestamp_ns": timestamp,
            "depth_timestamp_ns": timestamp + 1_000,
            "mask_timestamp_ns": timestamp + 2_000,
            "sync_tolerance_ns": 50_000,
            "camera_intrinsics": {"fx": 320.0, "fy": 320.0, "cx": 2.0, "cy": 1.0},
            "camera_pose": [0, 0, 1, 0, 0, 0, 1],
            "rover_pose": [0, 0, 0, 0, 0, 0, 1],
            "lighting": {"sun_elevation_deg": 15.0},
        }

    def files(self, suffix: str, label: int = 1, width: int = 4, height: int = 3):
        output = []
        for name, offset in (("rgb", 10), ("depth", 20), ("mask", label)):
            path = self.inputs / f"{name}-{suffix}.png"
            value = label if name == "mask" else (offset + label) % 256
            write_mask_png(path, width, height, bytes([value]) * width * height)
            output.append(path)
        return output

    def add(self, sample_id: str, world_id: str, label: int = 1):
        rgb, depth, mask = self.files(sample_id, label)
        return add_sample(self.dataset, sample_id, world_id, rgb, depth, mask,
                          self.metadata(), set(range(9)))

    def test_png_round_trip_and_crc(self):
        path = self.inputs / "mask.png"
        pixels = bytes(range(12))
        write_mask_png(path, 4, 3, pixels)
        self.assertEqual(read_mask_png(path), (4, 3, pixels))
        self.assertEqual(png_info(path)["bit_depth"], 8)

    def test_collect_creates_complete_manifest(self):
        record = self.add("sample-001", "world-01", 6)
        self.assertEqual(record["class_histogram"], {"6": 12})
        manifest = load_manifest(self.dataset)
        self.assertEqual(len(manifest["samples"]), 1)
        for relative in record["paths"].values():
            self.assertTrue((self.dataset / relative).is_file())

    def test_valid_dataset_report(self):
        self.add("sample-001", "world-01", 1)
        report = validate_dataset(self.dataset, set(range(9)))
        self.assertTrue(report["valid"])
        self.assertEqual(report["sample_count"], 1)
        self.assertEqual(report["world_count"], 1)
        self.assertEqual(report["class_pixels"], {"1": 12})

    def test_duplicate_sample_id_rejected(self):
        self.add("sample-001", "world-01", 1)
        rgb, depth, mask = self.files("different", 2)
        with self.assertRaisesRegex(DatasetError, "duplicate sample_id"):
            add_sample(self.dataset, "sample-001", "world-01", rgb, depth, mask,
                       self.metadata(), set(range(9)))

    def test_duplicate_triplet_rejected(self):
        rgb, depth, mask = self.files("same", 2)
        add_sample(self.dataset, "sample-001", "world-01", rgb, depth, mask,
                   self.metadata(), set(range(9)))
        with self.assertRaisesRegex(DatasetError, "duplicate image triplet"):
            add_sample(self.dataset, "sample-002", "world-01", rgb, depth, mask,
                       self.metadata(), set(range(9)))

    def test_invalid_class_rejected_without_partial_files(self):
        rgb, depth, mask = self.files("invalid", 99)
        with self.assertRaisesRegex(DatasetError, "invalid class IDs"):
            add_sample(self.dataset, "sample-001", "world-01", rgb, depth, mask,
                       self.metadata(), set(range(9)))
        self.assertFalse((self.dataset / "manifest.json").exists())

    def test_dimension_mismatch_rejected(self):
        rgb, depth, mask = self.files("dimensions", 1)
        write_mask_png(depth, 2, 2, bytes([1]) * 4)
        with self.assertRaisesRegex(DatasetError, "dimensions differ"):
            add_sample(self.dataset, "sample-001", "world-01", rgb, depth, mask,
                       self.metadata(), set(range(9)))

    def test_timestamp_mismatch_rejected(self):
        metadata = self.metadata()
        metadata["mask_timestamp_ns"] += 1_000_000
        with self.assertRaisesRegex(DatasetError, "synchronization tolerance"):
            validate_metadata(metadata)

    def test_missing_metadata_rejected(self):
        metadata = self.metadata()
        del metadata["camera_pose"]
        with self.assertRaisesRegex(DatasetError, "metadata missing"):
            validate_metadata(metadata)

    def test_checksum_tampering_detected(self):
        record = self.add("sample-001", "world-01", 1)
        path = self.dataset / record["paths"]["rgb"]
        path.write_bytes(path.read_bytes() + b"tamper")
        report = validate_dataset(self.dataset, set(range(9)))
        self.assertFalse(report["valid"])
        self.assertTrue(any("checksum mismatch" in error for error in report["errors"]))

    def test_missing_file_detected(self):
        record = self.add("sample-001", "world-01", 1)
        (self.dataset / record["paths"]["depth"]).unlink()
        report = validate_dataset(self.dataset, set(range(9)))
        self.assertFalse(report["valid"])
        self.assertTrue(any("missing file" in error for error in report["errors"]))

    def test_split_is_reproducible_and_has_no_world_leakage(self):
        for index in range(6):
            self.add(f"sample-{index}", f"world-{index}", index % 9)
        first = split_by_world(self.dataset, seed=17)
        second = split_by_world(self.dataset, seed=17)
        self.assertEqual(first, second)
        world_sets = [set(value["worlds"]) for value in first["splits"].values()]
        self.assertTrue(all(world_sets))
        self.assertFalse(world_sets[0] & world_sets[1])
        self.assertFalse(world_sets[0] & world_sets[2])
        self.assertFalse(world_sets[1] & world_sets[2])
        self.assertEqual(set.union(*world_sets), {f"world-{index}" for index in range(6)})

    def test_split_requires_three_worlds(self):
        self.add("sample-001", "world-01", 1)
        with self.assertRaisesRegex(DatasetError, "at least three worlds"):
            split_by_world(self.dataset)

    def test_split_rejects_invalid_ratios(self):
        for index in range(3):
            self.add(f"sample-{index}", f"world-{index}", index)
        with self.assertRaisesRegex(DatasetError, "summing to 1"):
            split_by_world(self.dataset, ratios=(0.8, 0.3, 0.1))

    def test_cli_validation_and_report(self):
        self.add("sample-001", "world-01", 1)
        validation = subprocess.run(
            [sys.executable, str(ML / "validate_dataset.py"),
             "--dataset", str(self.dataset)], capture_output=True, text=True)
        self.assertEqual(validation.returncode, 0, validation.stdout + validation.stderr)
        self.assertIn("DATASET_VALIDATION_PASS", validation.stdout)
        report_path = self.root / "report.md"
        report = subprocess.run(
            [sys.executable, str(ML / "report_dataset.py"),
             "--dataset", str(self.dataset), "--output", str(report_path)],
            capture_output=True, text=True)
        self.assertEqual(report.returncode, 0, report.stdout + report.stderr)
        self.assertIn("Samples: **1**", report_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
