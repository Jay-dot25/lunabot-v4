"""Dependency-light LunaBot semantic dataset format and integrity checks."""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
import re
import shutil
import struct
import tempfile
import zlib
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

SCHEMA_VERSION = 1
MANIFEST_NAME = "manifest.json"
_SAMPLE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
REQUIRED_METADATA = {
    "rgb_timestamp_ns", "depth_timestamp_ns", "mask_timestamp_ns",
    "camera_intrinsics", "camera_pose", "rover_pose", "lighting",
}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class DatasetError(ValueError):
    """Dataset contract or integrity failure."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _chunks(data: bytes):
    offset = len(PNG_SIGNATURE)
    while offset + 12 <= len(data):
        length = struct.unpack(">I", data[offset:offset + 4])[0]
        name = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        crc_at = offset + 8 + length
        if crc_at + 4 > len(data):
            raise DatasetError("truncated PNG chunk")
        expected = struct.unpack(">I", data[crc_at:crc_at + 4])[0]
        actual = zlib.crc32(name + payload) & 0xFFFFFFFF
        if expected != actual:
            raise DatasetError(f"PNG CRC mismatch in {name.decode('ascii', 'replace')}")
        yield name, payload
        offset = crc_at + 4
        if name == b"IEND":
            return
    raise DatasetError("PNG has no valid IEND chunk")


def png_info(path: Path) -> dict:
    data = path.read_bytes()
    if not data.startswith(PNG_SIGNATURE):
        raise DatasetError(f"{path}: not a PNG file")
    chunks = list(_chunks(data))
    ihdr = next((payload for name, payload in chunks if name == b"IHDR"), None)
    if ihdr is None or len(ihdr) != 13:
        raise DatasetError(f"{path}: invalid PNG IHDR")
    width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(
        ">IIBBBBB", ihdr)
    if width <= 0 or height <= 0:
        raise DatasetError(f"{path}: invalid PNG dimensions")
    if compression != 0 or filtering != 0:
        raise DatasetError(f"{path}: unsupported PNG encoding")
    return {
        "width": width, "height": height, "bit_depth": bit_depth,
        "color_type": color_type, "interlace": interlace,
        "chunks": chunks,
    }


def _paeth(a: int, b: int, c: int) -> int:
    estimate = a + b - c
    pa, pb, pc = abs(estimate - a), abs(estimate - b), abs(estimate - c)
    return a if pa <= pb and pa <= pc else b if pb <= pc else c


def read_mask_png(path: Path) -> tuple[int, int, bytes]:
    info = png_info(path)
    if info["bit_depth"] != 8 or info["color_type"] != 0 or info["interlace"] != 0:
        raise DatasetError(f"{path}: mask must be non-interlaced 8-bit grayscale PNG")
    compressed = b"".join(payload for name, payload in info["chunks"] if name == b"IDAT")
    try:
        raw = zlib.decompress(compressed)
    except zlib.error as exc:
        raise DatasetError(f"{path}: corrupt PNG image data: {exc}") from exc
    width, height = info["width"], info["height"]
    stride = width
    if len(raw) != height * (stride + 1):
        raise DatasetError(f"{path}: unexpected decompressed mask size")
    output = bytearray()
    previous = bytearray(stride)
    for row_index in range(height):
        start = row_index * (stride + 1)
        filter_type = raw[start]
        source = raw[start + 1:start + 1 + stride]
        row = bytearray(stride)
        for x, value in enumerate(source):
            left = row[x - 1] if x else 0
            above = previous[x]
            upper_left = previous[x - 1] if x else 0
            if filter_type == 0:
                predictor = 0
            elif filter_type == 1:
                predictor = left
            elif filter_type == 2:
                predictor = above
            elif filter_type == 3:
                predictor = (left + above) // 2
            elif filter_type == 4:
                predictor = _paeth(left, above, upper_left)
            else:
                raise DatasetError(f"{path}: unsupported PNG filter {filter_type}")
            row[x] = (value + predictor) & 0xFF
        output.extend(row)
        previous = row
    return width, height, bytes(output)


def write_mask_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    """Write deterministic grayscale masks (also used by test fixtures)."""
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise DatasetError("invalid mask dimensions or pixel count")
    path.parent.mkdir(parents=True, exist_ok=True)
    def chunk(name: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + name + payload
                + struct.pack(">I", zlib.crc32(name + payload) & 0xFFFFFFFF))
    rows = b"".join(b"\x00" + pixels[y * width:(y + 1) * width]
                    for y in range(height))
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    path.write_bytes(PNG_SIGNATURE + chunk(b"IHDR", ihdr)
                     + chunk(b"IDAT", zlib.compress(rows, 9)) + chunk(b"IEND", b""))


def empty_manifest() -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": utc_now(),
        "updated_at_utc": utc_now(),
        "samples": [],
    }


def load_manifest(dataset: Path, create: bool = False) -> dict:
    path = dataset / MANIFEST_NAME
    if not path.exists():
        if create:
            return empty_manifest()
        raise DatasetError(f"manifest missing: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError(f"invalid manifest {path}: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema_version") != SCHEMA_VERSION:
        raise DatasetError("unsupported or missing dataset schema_version")
    if not isinstance(data.get("samples"), list):
        raise DatasetError("manifest samples must be a list")
    return data


def save_manifest(dataset: Path, manifest: dict) -> None:
    dataset.mkdir(parents=True, exist_ok=True)
    manifest["updated_at_utc"] = utc_now()
    target = dataset / MANIFEST_NAME
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=dataset,
                                     prefix=".manifest-", delete=False) as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, target)


def validate_metadata(metadata: dict) -> None:
    if not isinstance(metadata, dict):
        raise DatasetError("metadata must be an object")
    missing = sorted(REQUIRED_METADATA - metadata.keys())
    if missing:
        raise DatasetError("metadata missing fields: " + ", ".join(missing))
    timestamps = [metadata[name] for name in
                  ("rgb_timestamp_ns", "depth_timestamp_ns", "mask_timestamp_ns")]
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
           for value in timestamps):
        raise DatasetError("timestamps must be non-negative integer nanoseconds")
    tolerance = metadata.get("sync_tolerance_ns", 50_000_000)
    if isinstance(tolerance, bool) or not isinstance(tolerance, int) or tolerance < 0:
        raise DatasetError("sync_tolerance_ns must be a non-negative integer")
    if max(timestamps) - min(timestamps) > tolerance:
        raise DatasetError("RGB/depth/mask timestamps exceed synchronization tolerance")
    intrinsics = metadata["camera_intrinsics"]
    if not isinstance(intrinsics, dict) or not all(
            isinstance(intrinsics.get(key), (int, float)) and
            math.isfinite(float(intrinsics[key])) and float(intrinsics[key]) > 0
            for key in ("fx", "fy", "cx", "cy")):
        raise DatasetError("camera_intrinsics requires positive finite fx, fy, cx, cy")
    for pose_name in ("camera_pose", "rover_pose"):
        pose = metadata[pose_name]
        if not isinstance(pose, list) or len(pose) != 7 or not all(
                isinstance(value, (int, float)) and math.isfinite(float(value))
                for value in pose):
            raise DatasetError(f"{pose_name} must be [x,y,z,qx,qy,qz,qw]")
    if not isinstance(metadata["lighting"], dict):
        raise DatasetError("lighting must be an object")


def _safe_id(value: str, name: str) -> str:
    if not isinstance(value, str) or not _SAMPLE_ID.fullmatch(value):
        raise DatasetError(f"{name} must match {_SAMPLE_ID.pattern}")
    return value


def add_sample(dataset: Path, sample_id: str, world_id: str, rgb: Path,
               depth: Path, mask: Path, metadata: dict,
               allowed_class_ids: set[int]) -> dict:
    sample_id, world_id = _safe_id(sample_id, "sample_id"), _safe_id(world_id, "world_id")
    validate_metadata(metadata)
    for name, path in (("rgb", rgb), ("depth", depth), ("mask", mask)):
        if not path.is_file():
            raise DatasetError(f"{name} input missing: {path}")
    rgb_info, depth_info = png_info(rgb), png_info(depth)
    width, height, labels = read_mask_png(mask)
    dimensions = {(rgb_info["width"], rgb_info["height"]),
                  (depth_info["width"], depth_info["height"]), (width, height)}
    if len(dimensions) != 1:
        raise DatasetError(f"RGB/depth/mask dimensions differ: {sorted(dimensions)}")
    invalid = sorted(set(labels) - allowed_class_ids)
    if invalid:
        raise DatasetError(f"mask contains invalid class IDs: {invalid}")
    manifest = load_manifest(dataset, create=True)
    if any(item.get("sample_id") == sample_id for item in manifest["samples"]):
        raise DatasetError(f"duplicate sample_id: {sample_id}")
    source_hashes = {name: sha256(path) for name, path in
                     (("rgb", rgb), ("depth", depth), ("mask", mask))}
    signature = tuple(source_hashes[name] for name in ("rgb", "depth", "mask"))
    for item in manifest["samples"]:
        hashes = item.get("sha256", {})
        if tuple(hashes.get(name) for name in ("rgb", "depth", "mask")) == signature:
            raise DatasetError(f"duplicate image triplet already stored as {item.get('sample_id')}")

    destinations = {
        "rgb": Path("rgb") / f"{sample_id}.png",
        "depth": Path("depth") / f"{sample_id}.png",
        "mask": Path("masks") / f"{sample_id}.png",
        "metadata": Path("metadata") / f"{sample_id}.json",
    }
    copied: list[Path] = []
    try:
        for name, source in (("rgb", rgb), ("depth", depth), ("mask", mask)):
            target = dataset / destinations[name]
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                raise DatasetError(f"destination already exists: {target}")
            shutil.copyfile(source, target)
            copied.append(target)
        histogram = {str(key): value for key, value in sorted(Counter(labels).items())}
        stored_metadata = dict(metadata)
        stored_metadata.update({
            "sample_id": sample_id, "world_id": world_id,
            "width": width, "height": height, "class_histogram": histogram,
        })
        metadata_target = dataset / destinations["metadata"]
        metadata_target.parent.mkdir(parents=True, exist_ok=True)
        metadata_target.write_text(json.dumps(stored_metadata, indent=2, sort_keys=True) + "\n",
                                   encoding="utf-8")
        copied.append(metadata_target)
        record = {
            "sample_id": sample_id, "world_id": world_id,
            "width": width, "height": height,
            "paths": {name: path.as_posix() for name, path in destinations.items()},
            "sha256": {**source_hashes, "metadata": sha256(metadata_target)},
            "class_histogram": histogram,
        }
        manifest["samples"].append(record)
        manifest["samples"].sort(key=lambda item: item["sample_id"])
        save_manifest(dataset, manifest)
        return record
    except Exception:
        for path in copied:
            path.unlink(missing_ok=True)
        raise


def validate_dataset(dataset: Path, allowed_class_ids: set[int]) -> dict:
    manifest = load_manifest(dataset)
    errors: list[str] = []
    sample_ids: set[str] = set()
    signatures: set[tuple[str, str, str]] = set()
    worlds: Counter = Counter()
    classes: Counter = Counter()
    for record in manifest["samples"]:
        sample_id = record.get("sample_id")
        if sample_id in sample_ids:
            errors.append(f"duplicate sample_id: {sample_id}")
        sample_ids.add(sample_id)
        worlds[record.get("world_id", "")] += 1
        paths = record.get("paths", {})
        hashes = record.get("sha256", {})
        actual: dict[str, str] = {}
        for name in ("rgb", "depth", "mask", "metadata"):
            relative = paths.get(name)
            if not isinstance(relative, str):
                errors.append(f"{sample_id}: missing {name} path")
                continue
            path = dataset / relative
            if not path.is_file():
                errors.append(f"{sample_id}: missing file {relative}")
                continue
            actual[name] = sha256(path)
            if hashes.get(name) != actual[name]:
                errors.append(f"{sample_id}: checksum mismatch for {name}")
        if all(name in actual for name in ("rgb", "depth", "mask")):
            signature = tuple(actual[name] for name in ("rgb", "depth", "mask"))
            if signature in signatures:
                errors.append(f"{sample_id}: duplicate image triplet")
            signatures.add(signature)
            try:
                rgb_info = png_info(dataset / paths["rgb"])
                depth_info = png_info(dataset / paths["depth"])
                width, height, labels = read_mask_png(dataset / paths["mask"])
                if len({(rgb_info["width"], rgb_info["height"]),
                        (depth_info["width"], depth_info["height"]),
                        (width, height)}) != 1:
                    errors.append(f"{sample_id}: dimensions differ")
                invalid = sorted(set(labels) - allowed_class_ids)
                if invalid:
                    errors.append(f"{sample_id}: invalid class IDs {invalid}")
                classes.update(labels)
            except DatasetError as exc:
                errors.append(f"{sample_id}: {exc}")
        if "metadata" in actual:
            try:
                metadata = json.loads((dataset / paths["metadata"]).read_text(encoding="utf-8"))
                validate_metadata(metadata)
            except (DatasetError, json.JSONDecodeError) as exc:
                errors.append(f"{sample_id}: invalid metadata: {exc}")
    return {
        "valid": not errors, "errors": errors, "sample_count": len(manifest["samples"]),
        "world_count": len(worlds), "samples_per_world": dict(sorted(worlds.items())),
        "class_pixels": {str(key): value for key, value in sorted(classes.items())},
    }


def split_by_world(dataset: Path, seed: int = 42,
                   ratios: tuple[float, float, float] = (0.7, 0.15, 0.15)) -> dict:
    if len(ratios) != 3 or any(value < 0 for value in ratios) or not math.isclose(sum(ratios), 1.0):
        raise DatasetError("split ratios must be three non-negative values summing to 1")
    manifest = load_manifest(dataset)
    worlds = sorted({item["world_id"] for item in manifest["samples"]})
    if len(worlds) < 3:
        raise DatasetError("at least three worlds are required for leakage-free train/val/test splits")
    random.Random(seed).shuffle(worlds)
    counts = [max(1, round(len(worlds) * value)) for value in ratios]
    while sum(counts) > len(worlds):
        index = max(range(3), key=lambda i: counts[i])
        if counts[index] <= 1:
            raise DatasetError("cannot allocate non-empty splits")
        counts[index] -= 1
    while sum(counts) < len(worlds):
        index = max(range(3), key=lambda i: ratios[i] - counts[i] / len(worlds))
        counts[index] += 1
    names = ("train", "validation", "test")
    output = {"schema_version": 1, "seed": seed, "ratios": dict(zip(names, ratios)), "splits": {}}
    cursor = 0
    for name, count in zip(names, counts):
        selected = sorted(worlds[cursor:cursor + count])
        cursor += count
        selected_set = set(selected)
        sample_ids = sorted(item["sample_id"] for item in manifest["samples"]
                            if item["world_id"] in selected_set)
        output["splits"][name] = {"worlds": selected, "samples": sample_ids}
    split_dir = dataset / "splits"
    split_dir.mkdir(parents=True, exist_ok=True)
    (split_dir / "world_split.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output
