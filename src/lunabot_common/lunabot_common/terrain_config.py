"""Validated, dependency-free terrain class configuration.

The `.yaml` file is JSON-compatible YAML so baseline tools and ROS nodes can
load the same schema without requiring PyYAML. Phase 2 will install this module
as part of a ROS 2 package; until then it is importable from the repository root.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Iterable, Mapping, Sequence

def _default_config_path() -> Path:
    """Resolve the schema in a source checkout or isolated ROS installation."""
    source_candidate = Path(__file__).resolve().parents[3] / "config/terrain_classes.yaml"
    if source_candidate.is_file():
        return source_candidate
    # ament_python isolated installs place this module below
    # <prefix>/lib/pythonX.Y/site-packages/lunabot_common.
    installed_candidate = (
        Path(__file__).resolve().parents[4]
        / "share/lunabot_common/config/terrain_classes.yaml"
    )
    return installed_candidate


DEFAULT_CONFIG_PATH = _default_config_path()
_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


class TerrainConfigError(ValueError):
    """Raised when terrain class configuration violates the shared contract."""


@dataclass(frozen=True)
class TerrainClass:
    id: int
    name: str
    display_name: str
    color_rgb: tuple[int, int, int]
    traversal_cost: int
    lethal: bool
    confidence_threshold: float
    description: str


class TerrainConfig:
    """Immutable indexed terrain schema."""

    def __init__(self, schema_version: int, unknown_class: str,
                 classes: Iterable[TerrainClass]) -> None:
        values = tuple(classes)
        self.schema_version = schema_version
        self.unknown_class = unknown_class
        self.classes = values
        self._by_id: Mapping[int, TerrainClass] = MappingProxyType(
            {item.id: item for item in values})
        self._by_name: Mapping[str, TerrainClass] = MappingProxyType(
            {item.name: item for item in values})

    @property
    def by_id(self) -> Mapping[int, TerrainClass]:
        return self._by_id

    @property
    def by_name(self) -> Mapping[str, TerrainClass]:
        return self._by_name

    @property
    def unknown(self) -> TerrainClass:
        return self._by_name[self.unknown_class]

    def class_for_id(self, class_id: int) -> TerrainClass:
        """Return a configured class, falling back safely to unknown."""
        return self._by_id.get(class_id, self.unknown)

    def class_for_name(self, name: str) -> TerrainClass:
        try:
            return self._by_name[name]
        except KeyError as exc:
            raise TerrainConfigError(f"unknown terrain class name: {name}") from exc

    def classify(self, predicted_id: int, confidence: float) -> TerrainClass:
        """Apply the class confidence threshold and return unknown if uncertain."""
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
            raise TerrainConfigError("confidence must be a finite number")
        confidence = float(confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise TerrainConfigError("confidence must be within [0, 1]")
        terrain_class = self.class_for_id(predicted_id)
        if confidence < terrain_class.confidence_threshold:
            return self.unknown
        return terrain_class

    def to_palette(self, size: int = 256) -> list[tuple[int, int, int]]:
        """Return an ID-indexed RGB palette; unspecified IDs use unknown color."""
        if size <= max(self._by_id):
            raise TerrainConfigError("palette size cannot represent all class IDs")
        palette = [self.unknown.color_rgb] * size
        for class_id, terrain_class in self._by_id.items():
            palette[class_id] = terrain_class.color_rgb
        return palette


def _require_int(value: object, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TerrainConfigError(f"{field} must be an integer")
    if not minimum <= value <= maximum:
        raise TerrainConfigError(f"{field} must be within [{minimum}, {maximum}]")
    return value


def _require_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TerrainConfigError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise TerrainConfigError(f"{field} must be within [0, 1]")
    return result


def _require_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TerrainConfigError(f"{field} must be a non-empty string")
    return value.strip()


def _parse_color(value: object, field: str) -> tuple[int, int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise TerrainConfigError(f"{field} must contain exactly three channels")
    return tuple(_require_int(channel, f"{field}[{index}]", 0, 255)
                 for index, channel in enumerate(value))  # type: ignore[return-value]


def validate_terrain_data(data: object) -> TerrainConfig:
    if not isinstance(data, dict):
        raise TerrainConfigError("configuration root must be an object")
    schema_version = _require_int(data.get("schema_version"), "schema_version", 1, 1)
    unknown_class = _require_text(data.get("unknown_class"), "unknown_class")
    raw_classes = data.get("classes")
    if not isinstance(raw_classes, list) or not raw_classes:
        raise TerrainConfigError("classes must be a non-empty list")

    classes: list[TerrainClass] = []
    seen_ids: set[int] = set()
    seen_names: set[str] = set()
    seen_colors: set[tuple[int, int, int]] = set()
    for index, raw in enumerate(raw_classes):
        prefix = f"classes[{index}]"
        if not isinstance(raw, dict):
            raise TerrainConfigError(f"{prefix} must be an object")
        class_id = _require_int(raw.get("id"), f"{prefix}.id", 0, 255)
        name = _require_text(raw.get("name"), f"{prefix}.name")
        if not _NAME_PATTERN.fullmatch(name):
            raise TerrainConfigError(f"{prefix}.name must use lower_snake_case")
        color = _parse_color(raw.get("color_rgb"), f"{prefix}.color_rgb")
        if class_id in seen_ids:
            raise TerrainConfigError(f"duplicate terrain class id: {class_id}")
        if name in seen_names:
            raise TerrainConfigError(f"duplicate terrain class name: {name}")
        if color in seen_colors:
            raise TerrainConfigError(f"duplicate terrain visualization color: {color}")
        seen_ids.add(class_id)
        seen_names.add(name)
        seen_colors.add(color)
        traversal_cost = _require_int(
            raw.get("traversal_cost"), f"{prefix}.traversal_cost", 0, 100)
        lethal = raw.get("lethal")
        if not isinstance(lethal, bool):
            raise TerrainConfigError(f"{prefix}.lethal must be boolean")
        if lethal and traversal_cost != 100:
            raise TerrainConfigError(f"lethal class {name} must have traversal_cost 100")
        classes.append(TerrainClass(
            id=class_id,
            name=name,
            display_name=_require_text(raw.get("display_name"), f"{prefix}.display_name"),
            color_rgb=color,
            traversal_cost=traversal_cost,
            lethal=lethal,
            confidence_threshold=_require_float(
                raw.get("confidence_threshold"), f"{prefix}.confidence_threshold"),
            description=_require_text(raw.get("description"), f"{prefix}.description"),
        ))

    if unknown_class not in seen_names:
        raise TerrainConfigError("unknown_class must name a configured class")
    unknown = next(item for item in classes if item.name == unknown_class)
    if unknown.id != 0:
        raise TerrainConfigError("unknown class must use ID 0")
    if unknown.lethal:
        raise TerrainConfigError("unknown class policy must not be marked lethal")
    return TerrainConfig(schema_version, unknown_class, classes)


def load_terrain_config(path: Path | str = DEFAULT_CONFIG_PATH) -> TerrainConfig:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise TerrainConfigError(f"cannot read terrain config {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise TerrainConfigError(f"invalid JSON-compatible YAML in {path}: {exc}") from exc
    return validate_terrain_data(data)
