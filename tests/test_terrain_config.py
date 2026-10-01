#!/usr/bin/env python3
"""Tests for the shared multiclass terrain schema."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from lunabot_common.terrain_config import (
    DEFAULT_CONFIG_PATH,
    TerrainConfigError,
    load_terrain_config,
    validate_terrain_data,
)


class TerrainConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))

    def changed(self, callback):
        value = copy.deepcopy(self.raw)
        callback(value)
        return value

    def test_goal_schema_loads(self):
        config = load_terrain_config()
        self.assertEqual(config.schema_version, 1)
        self.assertEqual(len(config.classes), 9)
        self.assertEqual(config.unknown.id, 0)
        self.assertEqual(
            {item.name for item in config.classes},
            {"unknown", "flat_regolith", "rough_regolith", "bedrock",
             "small_rock", "large_rock", "crater", "shadow", "habitat"},
        )

    def test_ids_names_and_colors_are_unique(self):
        config = load_terrain_config()
        self.assertEqual(len({item.id for item in config.classes}), len(config.classes))
        self.assertEqual(len({item.name for item in config.classes}), len(config.classes))
        self.assertEqual(len({item.color_rgb for item in config.classes}), len(config.classes))

    def test_cost_and_confidence_ranges(self):
        config = load_terrain_config()
        for item in config.classes:
            self.assertGreaterEqual(item.traversal_cost, 0)
            self.assertLessEqual(item.traversal_cost, 100)
            self.assertGreaterEqual(item.confidence_threshold, 0.0)
            self.assertLessEqual(item.confidence_threshold, 1.0)
            if item.lethal:
                self.assertEqual(item.traversal_cost, 100)

    def test_unknown_fallback_for_unconfigured_id(self):
        config = load_terrain_config()
        self.assertEqual(config.class_for_id(255), config.unknown)

    def test_confidence_threshold(self):
        config = load_terrain_config()
        rock = config.class_for_name("large_rock")
        self.assertEqual(config.classify(rock.id, 0.64), config.unknown)
        self.assertEqual(config.classify(rock.id, 0.65), rock)

    def test_invalid_confidence_rejected(self):
        config = load_terrain_config()
        for invalid in (-0.1, 1.1, float("nan"), float("inf"), True, "0.5"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TerrainConfigError):
                    config.classify(1, invalid)  # type: ignore[arg-type]

    def test_palette_is_id_indexed(self):
        config = load_terrain_config()
        palette = config.to_palette()
        for item in config.classes:
            self.assertEqual(palette[item.id], item.color_rgb)
        with self.assertRaises(TerrainConfigError):
            config.to_palette(8)

    def test_duplicate_id_rejected(self):
        data = self.changed(lambda value: value["classes"][1].update(id=0))
        with self.assertRaisesRegex(TerrainConfigError, "duplicate.*id"):
            validate_terrain_data(data)

    def test_duplicate_name_rejected(self):
        data = self.changed(lambda value: value["classes"][1].update(name="unknown"))
        with self.assertRaisesRegex(TerrainConfigError, "duplicate.*name"):
            validate_terrain_data(data)

    def test_duplicate_color_rejected(self):
        color = self.raw["classes"][0]["color_rgb"]
        data = self.changed(lambda value: value["classes"][1].update(color_rgb=color))
        with self.assertRaisesRegex(TerrainConfigError, "duplicate.*color"):
            validate_terrain_data(data)

    def test_out_of_range_values_rejected(self):
        changes = (
            lambda value: value["classes"][1].update(traversal_cost=101),
            lambda value: value["classes"][1].update(confidence_threshold=-0.1),
            lambda value: value["classes"][1].update(color_rgb=[0, 0, 256]),
        )
        for change in changes:
            with self.subTest(change=change):
                with self.assertRaises(TerrainConfigError):
                    validate_terrain_data(self.changed(change))

    def test_lethal_class_requires_cost_100(self):
        data = self.changed(lambda value: value["classes"][1].update(
            lethal=True, traversal_cost=99))
        with self.assertRaisesRegex(TerrainConfigError, "must have traversal_cost 100"):
            validate_terrain_data(data)

    def test_unknown_must_exist_and_use_zero(self):
        data = self.changed(lambda value: value.update(unknown_class="missing"))
        with self.assertRaisesRegex(TerrainConfigError, "must name"):
            validate_terrain_data(data)
        data = self.changed(lambda value: (
            value["classes"][0].update(id=9),
            value["classes"][8].update(id=0),
        ))
        with self.assertRaisesRegex(TerrainConfigError, "must use ID 0"):
            validate_terrain_data(data)

    def test_malformed_file_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "terrain.yaml"
            path.write_text("not-json", encoding="utf-8")
            with self.assertRaisesRegex(TerrainConfigError, "invalid JSON-compatible YAML"):
                load_terrain_config(path)


if __name__ == "__main__":
    unittest.main()
