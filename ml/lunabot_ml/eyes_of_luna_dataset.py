"""Dependency-light dataset contract for Phase 2's independent terrain/shadow heads.

This module validates files and declared annotation provenance. It cannot determine
whether a human actually reviewed a mask or whether the semantics are correct.
"""
from __future__ import annotations

import json
import math
import os
import random
import re
import shutil
import tempfile
import zlib
from collections import Counter
from datetime import datetime
from pathlib import Path

from .dataset_manifest import DatasetError, png_info, read_mask_png, sha256, utc_now

TASK_NAME = "eyes_of_luna"
DATASET_SCHEMA_VERSION = 1
CAMERA_TOPIC = "/camera/image_raw"
IGNORE_ID = 255
TERRAIN_CLASSES = {
    0: "regolith",
    1: "bedrock",
    2: "rock_boulder",
    3: "crater",
}
SHADOW_CLASSES = {
    0: "not_shadow",
    1: "shadow",
}
_SAMPLE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_MAX_RGB_PIXELS = 20_000_000
DEFAULT_TASK_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "eyes_of_luna.json"


def load_task_config(path: Path = DEFAULT_TASK_CONFIG) -> dict:
    """Load and strictly check the Phase 2 task/output contract."""
    try:
        config = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"invalid Eyes of Luna config: {exc}") from exc
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise DatasetError("Eyes of Luna config must use schema_version 1")
    if config.get("task") != TASK_NAME:
        raise DatasetError(f"Eyes of Luna config task must be {TASK_NAME}")
    camera = config.get("camera", {})
    if (camera.get("topic") != CAMERA_TOPIC or camera.get("source_resolution") != [640, 480]
            or camera.get("color_order") != "RGB"):
        raise DatasetError("camera contract must be RGB 640x480 from /camera/image_raw")
    terrain = config.get("terrain_head", {})
    if (terrain.get("output_channels") != len(TERRAIN_CLASSES)
            or terrain.get("activation") != "softmax"
            or terrain.get("class_ids") != _class_schema()["terrain"]
            or terrain.get("ignore_id") != IGNORE_ID):
        raise DatasetError("terrain head must be a four-class softmax with ignore ID 255")
    shadow = config.get("shadow_head", {})
    if (shadow.get("output_channels") != 1 or shadow.get("activation") != "sigmoid"
            or shadow.get("negative_id") != 0 or shadow.get("positive_id") != 1
            or shadow.get("ignore_id") != IGNORE_ID
            or shadow.get("independent_of_terrain") is not True):
        raise DatasetError("shadow head must be an independent binary sigmoid with ignore ID 255")
    dataset = config.get("dataset", {})
    if (dataset.get("schema") != "eyes_of_luna_camera_v1"
            or dataset.get("group_split_required") is not True
            or dataset.get("human_review_required_for_each_head") is not True):
        raise DatasetError("dataset config must require group splits and reviewed masks")
    return config


def _class_schema() -> dict:
    return {
        "terrain": {str(key): value for key, value in TERRAIN_CLASSES.items()},
        "shadow": {str(key): value for key, value in SHADOW_CLASSES.items()},
        "ignore_id": IGNORE_ID,
    }


def _safe_id(value: str, name: str) -> str:
    if not isinstance(value, str) or not _SAMPLE_ID.fullmatch(value):
        raise DatasetError(f"{name} must match {_SAMPLE_ID.pattern}")
    return value


def _nonempty_text(value, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise DatasetError(f"{name} must be a non-empty string")
    return value.strip()


def _pose(value, name: str) -> None:
    if not isinstance(value, list) or len(value) != 7 or any(
        isinstance(item, bool) or not isinstance(item, (int, float))
        or not math.isfinite(float(item)) for item in value
    ):
        raise DatasetError(f"{name} must be [x,y,z,qx,qy,qz,qw] with finite values")


def validate_rgb_png(path: Path) -> tuple[int, int]:
    """Check a non-interlaced, 8-bit RGB PNG including its decompressed rows."""
    info = png_info(Path(path))
    if info["bit_depth"] != 8 or info["color_type"] != 2 or info["interlace"] != 0:
        raise DatasetError(f"{path}: RGB image must be non-interlaced 8-bit RGB PNG")
    width, height = info["width"], info["height"]
    if width * height > _MAX_RGB_PIXELS:
        raise DatasetError(f"{path}: image exceeds the {_MAX_RGB_PIXELS}-pixel safety limit")
    compressed = b"".join(payload for name, payload in info["chunks"] if name == b"IDAT")
    try:
        decoder = zlib.decompressobj()
        decoded = decoder.decompress(compressed, height * (width * 3 + 1) + 1)
        if decoder.unconsumed_tail:
            raise DatasetError(f"{path}: oversized or corrupt PNG image data")
        decoded += decoder.flush()
    except zlib.error as exc:
        raise DatasetError(f"{path}: corrupt PNG image data: {exc}") from exc
    expected_size = height * (width * 3 + 1)
    if len(decoded) != expected_size or not decoder.eof or decoder.unused_data:
        raise DatasetError(f"{path}: unexpected or truncated RGB image data")
    stride = width * 3 + 1
    for row in range(height):
        if decoded[row * stride] > 4:
            raise DatasetError(f"{path}: unsupported PNG filter {decoded[row * stride]}")
    return width, height


def validate_annotation_metadata(metadata: dict, sample_id: str, group_id: str) -> None:
    """Validate capture details and require human review of both independent masks."""
    if not isinstance(metadata, dict):
        raise DatasetError("sample metadata must be an object")
    if metadata.get("schema_version") not in (None, DATASET_SCHEMA_VERSION):
        raise DatasetError("unsupported sample metadata schema_version")
    if metadata.get("sample_id", sample_id) != sample_id:
        raise DatasetError("metadata sample_id does not match manifest")
    if metadata.get("group_id", group_id) != group_id:
        raise DatasetError("metadata group_id does not match manifest")

    capture = metadata.get("capture")
    if not isinstance(capture, dict):
        raise DatasetError("metadata capture must be an object")
    if capture.get("camera_topic") != CAMERA_TOPIC:
        raise DatasetError(f"capture.camera_topic must be {CAMERA_TOPIC}")
    timestamp = capture.get("timestamp_ns")
    if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
        raise DatasetError("capture.timestamp_ns must be a non-negative integer")
    _safe_id(capture.get("world_id"), "capture.world_id")
    _safe_id(capture.get("session_id"), "capture.session_id")
    _nonempty_text(capture.get("source"), "capture.source")
    _pose(capture.get("camera_pose_ground_truth"), "capture.camera_pose_ground_truth")
    _pose(capture.get("rover_pose_ground_truth"), "capture.rover_pose_ground_truth")
    if "rover_pose_odom" in capture:
        _pose(capture["rover_pose_odom"], "capture.rover_pose_odom")

    simulator = capture.get("simulator")
    if not isinstance(simulator, dict):
        raise DatasetError("capture.simulator must record name, version, and renderer")
    for field in ("name", "version", "renderer"):
        _nonempty_text(simulator.get(field), f"capture.simulator.{field}")
    lighting = capture.get("lighting")
    if not isinstance(lighting, dict) or not lighting:
        raise DatasetError("capture.lighting must record the lighting conditions")

    annotations = metadata.get("annotations")
    if not isinstance(annotations, dict):
        raise DatasetError("metadata annotations must contain terrain and shadow reviews")
    for head in ("terrain", "shadow"):
        annotation = annotations.get(head)
        if not isinstance(annotation, dict):
            raise DatasetError(f"annotations.{head} must be an object")
        if annotation.get("review_status") != "human_reviewed":
            raise DatasetError(f"annotations.{head} is not marked human_reviewed")
        _nonempty_text(annotation.get("reviewer_id"), f"annotations.{head}.reviewer_id")
        reviewed_at = _nonempty_text(
            annotation.get("reviewed_at_utc"), f"annotations.{head}.reviewed_at_utc")
        try:
            parsed = datetime.fromisoformat(reviewed_at.replace("Z", "+00:00"))
        except ValueError as exc:
            raise DatasetError(f"annotations.{head}.reviewed_at_utc must be ISO 8601") from exc
        if parsed.tzinfo is None:
            raise DatasetError(f"annotations.{head}.reviewed_at_utc must include a timezone")
        method = annotation.get("method")
        if method == "manual":
            _nonempty_text(annotation.get("annotation_tool"),
                           f"annotations.{head}.annotation_tool")
        elif method == "semantic_camera":
            generator = annotation.get("generator")
            if not isinstance(generator, dict):
                raise DatasetError(f"annotations.{head}.generator must document semantic camera use")
            for field in ("implementation", "version", "class_mapping"):
                _nonempty_text(generator.get(field), f"annotations.{head}.generator.{field}")
        else:
            raise DatasetError(
                f"annotations.{head}.method must be manual or semantic_camera")


def _empty_manifest() -> dict:
    return {
        "schema_version": DATASET_SCHEMA_VERSION,
        "task": TASK_NAME,
        "class_schema": _class_schema(),
        "created_at_utc": utc_now(),
        "updated_at_utc": utc_now(),
        "samples": [],
    }


def load_manifest(dataset: Path, create: bool = False) -> dict:
    path = Path(dataset) / "manifest.json"
    if not path.exists():
        if create:
            return _empty_manifest()
        raise DatasetError(f"manifest missing: {path}")
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"invalid manifest {path}: {exc}") from exc
    if not isinstance(manifest, dict) or manifest.get("schema_version") != DATASET_SCHEMA_VERSION:
        raise DatasetError("unsupported or missing Phase 2 dataset schema_version")
    if manifest.get("task") != TASK_NAME:
        raise DatasetError(f"manifest task must be {TASK_NAME}")
    if manifest.get("class_schema") != _class_schema():
        raise DatasetError("manifest class schema does not match the Phase 2 output contract")
    if not isinstance(manifest.get("samples"), list):
        raise DatasetError("manifest samples must be a list")
    return manifest


def _save_manifest(dataset: Path, manifest: dict) -> None:
    dataset.mkdir(parents=True, exist_ok=True)
    manifest["updated_at_utc"] = utc_now()
    target = dataset / "manifest.json"
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=dataset,
                                     prefix=".manifest-", delete=False) as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, target)


def collect_sample(dataset: Path, sample_id: str, group_id: str, rgb: Path,
                   terrain_mask: Path, shadow_mask: Path, metadata: dict) -> dict:
    """Register one RGB frame and its separately reviewed terrain/shadow masks."""
    dataset = Path(dataset)
    sample_id, group_id = _safe_id(sample_id, "sample_id"), _safe_id(group_id, "group_id")
    validate_annotation_metadata(metadata, sample_id, group_id)
    inputs = {
        "rgb": Path(rgb),
        "terrain_mask": Path(terrain_mask),
        "shadow_mask": Path(shadow_mask),
    }
    for name, path in inputs.items():
        if not path.is_file():
            raise DatasetError(f"{name} input missing: {path}")

    width, height = validate_rgb_png(inputs["rgb"])
    terrain_width, terrain_height, terrain_pixels = read_mask_png(inputs["terrain_mask"])
    shadow_width, shadow_height, shadow_pixels = read_mask_png(inputs["shadow_mask"])
    dimensions = {(width, height), (terrain_width, terrain_height),
                  (shadow_width, shadow_height)}
    if len(dimensions) != 1:
        raise DatasetError(f"RGB/terrain/shadow dimensions differ: {sorted(dimensions)}")
    invalid_terrain = sorted(set(terrain_pixels) - set(TERRAIN_CLASSES) - {IGNORE_ID})
    if invalid_terrain:
        raise DatasetError(f"terrain mask contains invalid class IDs: {invalid_terrain}")
    invalid_shadow = sorted(set(shadow_pixels) - set(SHADOW_CLASSES) - {IGNORE_ID})
    if invalid_shadow:
        raise DatasetError(f"shadow mask contains invalid class IDs: {invalid_shadow}")

    manifest = load_manifest(dataset, create=True)
    if any(item.get("sample_id") == sample_id for item in manifest["samples"]
           if isinstance(item, dict)):
        raise DatasetError(f"duplicate sample_id: {sample_id}")
    input_hashes = {name: sha256(path) for name, path in inputs.items()}
    for item in manifest["samples"]:
        hashes = item.get("sha256", {}) if isinstance(item, dict) else {}
        if hashes.get("rgb") == input_hashes["rgb"]:
            raise DatasetError(f"duplicate RGB image already stored as {item.get('sample_id')}")

    relative_paths = {
        "rgb": Path("rgb") / f"{sample_id}.png",
        "terrain_mask": Path("terrain_masks") / f"{sample_id}.png",
        "shadow_mask": Path("shadow_masks") / f"{sample_id}.png",
        "metadata": Path("metadata") / f"{sample_id}.json",
    }
    for relative in relative_paths.values():
        target = dataset / relative
        if target.exists():
            raise DatasetError(f"destination already exists: {target}")
    copied: list[Path] = []
    try:
        for name in ("rgb", "terrain_mask", "shadow_mask"):
            target = dataset / relative_paths[name]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(inputs[name], target)
            copied.append(target)
        stored_metadata = dict(metadata)
        stored_metadata.update({
            "schema_version": DATASET_SCHEMA_VERSION,
            "sample_id": sample_id,
            "group_id": group_id,
        })
        metadata_path = dataset / relative_paths["metadata"]
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        copied.append(metadata_path)
        metadata_path.write_text(
            json.dumps(stored_metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")

        terrain_counts = Counter(terrain_pixels)
        shadow_counts = Counter(shadow_pixels)
        record = {
            "sample_id": sample_id,
            "group_id": group_id,
            "width": width,
            "height": height,
            "paths": {name: path.as_posix() for name, path in relative_paths.items()},
            "sha256": {
                **{name: sha256(dataset / relative_paths[name]) for name in inputs},
                "metadata": sha256(metadata_path),
            },
            "class_pixel_counts": {
                "terrain": {str(key): value for key, value in sorted(terrain_counts.items())},
                "shadow": {str(key): value for key, value in sorted(shadow_counts.items())},
            },
        }
        manifest["samples"].append(record)
        manifest["samples"].sort(key=lambda item: item["sample_id"])
        _save_manifest(dataset, manifest)
        return record
    except Exception:
        for path in copied:
            path.unlink(missing_ok=True)
        raise


def _canonical_paths(sample_id: str) -> dict[str, str]:
    return {
        "rgb": f"rgb/{sample_id}.png",
        "terrain_mask": f"terrain_masks/{sample_id}.png",
        "shadow_mask": f"shadow_masks/{sample_id}.png",
        "metadata": f"metadata/{sample_id}.json",
    }


def validate_dataset(dataset: Path) -> dict:
    """Check file integrity, label IDs, dimensions, and declared human review."""
    dataset = Path(dataset)
    manifest = load_manifest(dataset)
    errors: list[str] = []
    sample_ids: set[str] = set()
    group_counts: Counter = Counter()
    seen_rgb_hashes: set[str] = set()
    terrain_totals: Counter = Counter()
    shadow_totals: Counter = Counter()
    sample_count = len(manifest["samples"])

    if sample_count == 0:
        errors.append("dataset contains no samples")

    for record in manifest["samples"]:
        if not isinstance(record, dict):
            errors.append("manifest sample record must be an object")
            continue
        sample_id = record.get("sample_id")
        try:
            sample_id = _safe_id(sample_id, "sample_id")
        except DatasetError as exc:
            errors.append(str(exc))
            continue
        if sample_id in sample_ids:
            errors.append(f"duplicate sample_id: {sample_id}")
        sample_ids.add(sample_id)
        try:
            group_id = _safe_id(record.get("group_id"), "group_id")
        except DatasetError as exc:
            errors.append(f"{sample_id}: {exc}")
            continue
        group_counts[group_id] += 1

        expected_paths = _canonical_paths(sample_id)
        paths = record.get("paths")
        hashes = record.get("sha256")
        if not isinstance(paths, dict) or not isinstance(hashes, dict):
            errors.append(f"{sample_id}: paths and sha256 must be objects")
            continue
        actual_hashes: dict[str, str] = {}
        resolved_files: dict[str, Path] = {}
        for name, expected in expected_paths.items():
            relative = paths.get(name)
            if relative != expected:
                errors.append(f"{sample_id}: non-canonical {name} path")
                continue
            path = dataset / expected
            try:
                resolved = path.resolve(strict=True)
                resolved.relative_to(dataset.resolve())
            except (OSError, ValueError):
                errors.append(f"{sample_id}: unsafe or missing file {expected}")
                continue
            if path.is_symlink() or not resolved.is_file():
                errors.append(f"{sample_id}: unsafe or missing file {expected}")
                continue
            resolved_files[name] = resolved
            try:
                actual_hashes[name] = sha256(resolved)
            except OSError as exc:
                errors.append(f"{sample_id}: cannot hash {expected}: {exc}")
                continue
            if hashes.get(name) != actual_hashes[name]:
                errors.append(f"{sample_id}: checksum mismatch for {name}")

        if "rgb" in resolved_files:
            if actual_hashes.get("rgb") in seen_rgb_hashes:
                errors.append(f"{sample_id}: duplicate RGB image")
            seen_rgb_hashes.add(actual_hashes["rgb"])

        if all(name in resolved_files for name in ("rgb", "terrain_mask", "shadow_mask")):
            try:
                width, height = validate_rgb_png(resolved_files["rgb"])
                terrain_width, terrain_height, terrain_pixels = read_mask_png(
                    resolved_files["terrain_mask"])
                shadow_width, shadow_height, shadow_pixels = read_mask_png(
                    resolved_files["shadow_mask"])
                if len({(width, height), (terrain_width, terrain_height),
                        (shadow_width, shadow_height)}) != 1:
                    errors.append(f"{sample_id}: RGB/terrain/shadow dimensions differ")
                if record.get("width") != width or record.get("height") != height:
                    errors.append(f"{sample_id}: manifest dimensions do not match files")
                invalid_terrain = sorted(set(terrain_pixels) - set(TERRAIN_CLASSES) - {IGNORE_ID})
                invalid_shadow = sorted(set(shadow_pixels) - set(SHADOW_CLASSES) - {IGNORE_ID})
                if invalid_terrain:
                    errors.append(f"{sample_id}: invalid terrain class IDs {invalid_terrain}")
                if invalid_shadow:
                    errors.append(f"{sample_id}: invalid shadow class IDs {invalid_shadow}")
                terrain_totals.update(terrain_pixels)
                shadow_totals.update(shadow_pixels)
                recorded_counts = record.get("class_pixel_counts")
                actual_counts = {
                    "terrain": {str(key): value for key, value in
                                sorted(Counter(terrain_pixels).items())},
                    "shadow": {str(key): value for key, value in
                               sorted(Counter(shadow_pixels).items())},
                }
                if recorded_counts != actual_counts:
                    errors.append(f"{sample_id}: class_pixel_counts do not match masks")
            except DatasetError as exc:
                errors.append(f"{sample_id}: {exc}")

        if "metadata" in resolved_files:
            try:
                metadata = json.loads(resolved_files["metadata"].read_text(encoding="utf-8"))
                validate_annotation_metadata(metadata, sample_id, group_id)
            except (DatasetError, OSError, json.JSONDecodeError) as exc:
                errors.append(f"{sample_id}: invalid metadata: {exc}")

    missing_terrain = sorted(set(TERRAIN_CLASSES) - set(terrain_totals))
    missing_shadow = sorted(set(SHADOW_CLASSES) - set(shadow_totals))
    return {
        "valid": not errors,
        "errors": errors,
        "task": TASK_NAME,
        "sample_count": sample_count,
        "capture_group_count": len(group_counts),
        "samples_per_group": dict(sorted(group_counts.items())),
        "terrain_pixel_counts": {str(key): value for key, value in sorted(terrain_totals.items())},
        "shadow_pixel_counts": {str(key): value for key, value in sorted(shadow_totals.items())},
        "missing_terrain_classes": missing_terrain,
        "missing_shadow_classes": missing_shadow,
        "coverage_complete": not missing_terrain and not missing_shadow,
        "three_way_split_possible": len(group_counts) >= 3,
        "annotation_review": "declared by metadata; semantic correctness is not machine-verified",
    }


def split_by_capture_group(dataset: Path, seed: int = 42,
                           ratios: tuple[float, float, float] = (0.70, 0.15, 0.15)) -> dict:
    """Create deterministic train/validation/test splits without group leakage."""
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise DatasetError("seed must be an integer")
    if len(ratios) != 3 or any(value <= 0 for value in ratios) or not math.isclose(sum(ratios), 1.0):
        raise DatasetError("split ratios must be three positive values summing to 1")
    validation = validate_dataset(dataset)
    if not validation["valid"]:
        raise DatasetError("dataset validation failed: " + "; ".join(validation["errors"]))
    manifest = load_manifest(dataset)
    groups = sorted({item["group_id"] for item in manifest["samples"]})
    if len(groups) < 3:
        raise DatasetError("at least three capture groups are required for train/validation/test")

    random.Random(seed).shuffle(groups)
    counts = [max(1, round(len(groups) * value)) for value in ratios]
    while sum(counts) > len(groups):
        index = max(range(3), key=lambda i: counts[i])
        if counts[index] <= 1:
            raise DatasetError("cannot allocate non-empty capture-group splits")
        counts[index] -= 1
    while sum(counts) < len(groups):
        index = max(range(3), key=lambda i: ratios[i] - counts[i] / len(groups))
        counts[index] += 1

    names = ("train", "validation", "test")
    result = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "task": TASK_NAME,
        "seed": seed,
        "ratios": dict(zip(names, ratios)),
        "splits": {},
    }
    cursor = 0
    for name, count in zip(names, counts):
        selected_groups = sorted(groups[cursor:cursor + count])
        cursor += count
        selected = set(selected_groups)
        sample_ids = sorted(item["sample_id"] for item in manifest["samples"]
                            if item["group_id"] in selected)
        result["splits"][name] = {
            "groups": selected_groups,
            "samples": sample_ids,
        }
    return result


def validate_capture_group_split(dataset: Path, split: dict) -> dict:
    """Validate a complete, non-leaking split document against its dataset."""
    if not isinstance(split, dict) or split.get("schema_version") != DATASET_SCHEMA_VERSION or split.get("task") != TASK_NAME:
        raise DatasetError("invalid Phase 2 split document")
    names = ("train", "validation", "test")
    sections = split.get("splits")
    if not isinstance(sections, dict) or set(sections) != set(names):
        raise DatasetError("split document must contain train, validation, and test")
    group_sets, sample_sets = [], []
    for name in names:
        section = sections[name]
        if not isinstance(section, dict):
            raise DatasetError(f"{name} split must be an object")
        groups, samples = section.get("groups"), section.get("samples")
        if (not isinstance(groups, list) or not isinstance(samples, list)
                or any(not isinstance(value, str) for value in groups + samples)):
            raise DatasetError(f"{name} split groups and samples must be string lists")
        if len(groups) != len(set(groups)) or len(samples) != len(set(samples)):
            raise DatasetError(f"{name} split contains duplicate IDs")
        group_sets.append(set(groups))
        sample_sets.append(set(samples))
    if any(not groups for groups in group_sets) or any(not samples for samples in sample_sets):
        raise DatasetError("each split must contain at least one group and sample")
    if (group_sets[0] & group_sets[1] or group_sets[0] & group_sets[2]
            or group_sets[1] & group_sets[2]):
        raise DatasetError("capture-group leakage detected in split document")
    if (sample_sets[0] & sample_sets[1] or sample_sets[0] & sample_sets[2]
            or sample_sets[1] & sample_sets[2]):
        raise DatasetError("sample leakage detected in split document")

    validation = validate_dataset(dataset)
    if not validation["valid"]:
        raise DatasetError("dataset validation failed: " + "; ".join(validation["errors"]))
    manifest = load_manifest(dataset)
    expected_groups = {item["group_id"] for item in manifest["samples"]}
    expected_samples = {item["sample_id"] for item in manifest["samples"]}
    if set.union(*group_sets) != expected_groups or set.union(*sample_sets) != expected_samples:
        raise DatasetError("split document must assign every dataset group and sample exactly once")
    for name, groups, samples in zip(names, group_sets, sample_sets):
        expected_samples_for_groups = {
            item["sample_id"] for item in manifest["samples"]
            if item["group_id"] in groups
        }
        if samples != expected_samples_for_groups:
            raise DatasetError(f"{name} split samples do not match their capture groups")
    return split


def load_capture_group_split(dataset: Path) -> dict:
    path = Path(dataset) / "splits" / "capture_group_split.json"
    try:
        split = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"invalid or missing capture-group split {path}: {exc}") from exc
    return validate_capture_group_split(dataset, split)


def save_capture_group_split(dataset: Path, split: dict) -> Path:
    """Atomically persist a complete split after checking coverage and leakage."""
    validate_capture_group_split(dataset, split)
    directory = Path(dataset) / "splits"
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / "capture_group_split.json"
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=directory,
                                     prefix=".split-", delete=False) as stream:
        json.dump(split, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, target)
    return target
