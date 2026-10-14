#!/usr/bin/env python3
"""Extract sampled RGB PNGs and provenance sidecars from a ROS 2 bag.

The sidecars intentionally remain incomplete for dataset registration: human
reviewed terrain/shadow masks and verified capture/pose/lighting metadata must
be added before collect_eyes_of_luna_sample.py will accept a sample.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE_IMPORT_ROOT = ROOT / "lunabot_phase1" / "src" / "lunabot_phase2_perception"
if ROS_PACKAGE_IMPORT_ROOT.is_dir():
    sys.path.insert(0, str(ROS_PACKAGE_IMPORT_ROOT))

from lunabot_phase2_perception.image_conversion import (  # noqa: E402
    ImageEncodingError,
    decode_image_rgb,
)

_SAMPLE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
DEFAULT_IMAGE_TOPIC = "/camera/image_raw"


def validate_group_id(group_id: str) -> str:
    if (not isinstance(group_id, str) or len(group_id) > 100
            or not _SAMPLE_ID.fullmatch(group_id)):
        raise ValueError("group_id must contain 1-100 letters, digits, '.', '_' or '-'")
    return group_id


def save_capture_sample(image, output: Path, group_id: str, frame_index: int,
                        bag_timestamp_ns: int, source_bag: str,
                        image_topic: str = DEFAULT_IMAGE_TOPIC) -> dict:
    """Save one RGB image and an explicitly unreviewed capture metadata sidecar."""
    from PIL import Image

    group_id = validate_group_id(group_id)
    if frame_index < 0:
        raise ValueError("frame_index must be non-negative")
    rgb = decode_image_rgb(image)
    stamp = image.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    sample_id = f"{group_id}_frame{frame_index:06d}"
    output = Path(output)
    rgb_dir = output / "rgb"
    metadata_dir = output / "capture_metadata"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = rgb_dir / f"{sample_id}.png"
    metadata_path = metadata_dir / f"{sample_id}.json"
    Image.fromarray(rgb, mode="RGB").save(rgb_path)
    capture = {
        "camera_topic": image_topic,
        "timestamp_ns": stamp_ns,
        "bag_timestamp_ns": int(bag_timestamp_ns),
        "session_id": group_id,
        "frame_id": str(image.header.frame_id),
        "width": int(image.width),
        "height": int(image.height),
        "encoding": str(image.encoding),
        "step": int(image.step),
        "source": f"rosbag2:{source_bag}",
    }
    sidecar = {
        "schema_version": 1,
        "sample_id": sample_id,
        "group_id": group_id,
        "capture": capture,
        "annotation_status": "pending_human_review",
        "review_requirements": ["terrain", "shadow"],
        "ground_truth_and_lighting_metadata": "must_be_filled_from_capture_evidence",
    }
    metadata_path.write_text(json.dumps(sidecar, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return {
        "sample_id": sample_id,
        "group_id": group_id,
        "frame_index": int(frame_index),
        "rgb": str(rgb_path.relative_to(output)),
        "capture_metadata": str(metadata_path.relative_to(output)),
        **capture,
    }


def extract_bag(bag_path: Path, output: Path, group_id: str,
                every_n: int = 30, max_frames: int | None = None,
                image_topic: str = DEFAULT_IMAGE_TOPIC,
                storage_id: str = "sqlite3") -> dict:
    """Read a ROS 2 bag and export every Nth camera image without modifying it."""
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 rosbag2_py, rclpy, and rosidl_runtime_py are required to extract a bag") from exc

    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"bag path does not exist: {bag_path}")
    group_id = validate_group_id(group_id)
    if every_n < 1:
        raise ValueError("every_n must be at least 1")
    if max_frames is not None and max_frames < 1:
        raise ValueError("max_frames must be at least 1")
    output = Path(output).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if image_topic not in topic_types:
        raise ValueError(f"image topic is missing from the bag: {image_topic}")
    if topic_types[image_topic] != "sensor_msgs/msg/Image":
        raise ValueError(
            f"expected sensor_msgs/msg/Image on {image_topic}, got {topic_types[image_topic]}")
    image_type = get_message(topic_types[image_topic])

    image_messages = 0
    frames = []
    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        if topic != image_topic:
            continue
        source_index = image_messages
        image_messages += 1
        if source_index % every_n != 0:
            continue
        image = deserialize_message(serialized, image_type)
        record = save_capture_sample(
            image, output, group_id, source_index, int(bag_timestamp_ns),
            bag_path.name, image_topic)
        frames.append(record)
        if max_frames is not None and len(frames) >= max_frames:
            break

    if not frames:
        raise ValueError(f"no images extracted from {image_topic}")
    manifest = {
        "schema_version": 1,
        "source_bag": str(bag_path),
        "image_topic": image_topic,
        "group_id": group_id,
        "every_n": int(every_n),
        "camera_messages_scanned": image_messages,
        "frames_written": len(frames),
        "frames": frames,
        "annotation_status": "pending_human_review",
    }
    manifest_path = output / "capture_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True,
                        help="ROS 2 bag directory containing metadata.yaml and storage file")
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory for sampled RGB PNGs and capture sidecars")
    parser.add_argument("--group-id", required=True,
                        help="safe capture-group ID, e.g. session_01")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--every-n", type=int, default=30,
                        help="export every Nth image message (default: 30)")
    parser.add_argument("--max-frames", type=int,
                        help="optional upper limit on exported frames")
    parser.add_argument("--storage-id", default="sqlite3")
    args = parser.parse_args()
    try:
        manifest = extract_bag(
            args.bag, args.output, args.group_id, args.every_n,
            args.max_frames, args.image_topic, args.storage_id)
    except (FileNotFoundError, FileExistsError, ImageEncodingError,
            ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"EYES_OF_LUNA_EXTRACTION_FAIL: {exc}")
        return 1
    print(json.dumps({
        "output": str(Path(args.output).expanduser().resolve()),
        "camera_messages_scanned": manifest["camera_messages_scanned"],
        "frames_written": manifest["frames_written"],
        "group_id": manifest["group_id"],
        "annotation_status": manifest["annotation_status"],
    }, indent=2, sort_keys=True))
    print("EYES_OF_LUNA_EXTRACTION_COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
