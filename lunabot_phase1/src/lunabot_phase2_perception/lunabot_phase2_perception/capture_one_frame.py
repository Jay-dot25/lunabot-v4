"""Save one live camera frame and an explicitly unreviewed provenance sidecar.

The sidecar preserves the ROS image header/encoding but intentionally omits
unknown scene, pose, renderer, lighting, and annotation facts. It is an inspection
capture, not a dataset registration record.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_IMAGE_TOPIC = "/camera/image_raw"


def build_capture_sidecar(image, image_topic: str, image_path: Path,
                          image_sha256: str, saved_at_utc: str | None = None) -> dict:
    """Build honest capture provenance from fields present in sensor_msgs/Image."""
    stamp = image.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    return {
        "schema_version": 1,
        "sample_id": Path(image_path).stem,
        "capture": {
            "camera_topic": str(image_topic),
            "timestamp_ns": sec * 1_000_000_000 + nanosec,
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "frame_id": str(image.header.frame_id),
            "width": int(image.width),
            "height": int(image.height),
            "encoding": str(image.encoding),
            "step": int(image.step),
            "is_bigendian": int(image.is_bigendian),
            "source": "live_ros_topic",
            "saved_at_utc": saved_at_utc or datetime.now(timezone.utc).isoformat(),
            "image_sha256": str(image_sha256),
        },
        "annotation_status": "pending_human_review",
        "registration_status": "blocked_pending_capture_metadata_and_both_reviewed_masks",
        "required_before_dataset_registration": [
            "verified world/session/capture-group identifiers",
            "simulator and renderer names/versions",
            "lighting conditions",
            "camera and rover poses from synchronized evidence",
            "terrain mask and independent shadow mask",
            "human review provenance for both masks",
        ],
        "note": (
            "This sidecar records only the ROS image message and saved PNG. "
            "It does not infer scene, pose, lighting, terrain labels, or shadows."
        ),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_capture(image, topic: str, output: Path, bridge, cv2) -> Path:
    """Write a PNG and header sidecar without replacing existing user files."""
    output = output.expanduser().resolve()
    metadata_path = output.with_suffix(".json")
    if output.suffix.lower() != ".png":
        raise ValueError("--output must have a .png extension")
    if output.exists() or metadata_path.exists():
        raise FileExistsError(f"capture output or sidecar already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    temporary_image = None
    temporary_metadata = None
    published_image = False
    published_metadata = False
    try:
        with tempfile.NamedTemporaryFile(
                dir=output.parent, prefix=f".{output.stem}-", suffix=".png", delete=False) as temp:
            temporary_image = Path(temp.name)
        bgr = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        if not cv2.imwrite(str(temporary_image), bgr):
            raise OSError(f"OpenCV could not encode PNG: {temporary_image}")
        metadata = build_capture_sidecar(
            image, topic, output, _sha256(temporary_image))
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=output.parent,
                prefix=f".{metadata_path.stem}-", suffix=".json", delete=False) as temp:
            json.dump(metadata, temp, indent=2, sort_keys=True)
            temp.write("\n")
            temporary_metadata = Path(temp.name)
        # Hard-link publication is exclusive on the same filesystem: never
        # overwrite a concurrent user's image or sidecar.
        os.link(temporary_image, output)
        published_image = True
        os.link(temporary_metadata, metadata_path)
        published_metadata = True
        return metadata_path
    except Exception:
        if published_image:
            output.unlink(missing_ok=True)
        if published_metadata:
            metadata_path.unlink(missing_ok=True)
        raise
    finally:
        if temporary_image is not None:
            temporary_image.unlink(missing_ok=True)
        if temporary_metadata is not None:
            temporary_metadata.unlink(missing_ok=True)


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--output", type=Path, required=True,
                        help="new PNG path; a JSON sidecar is written beside it")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="seconds to wait for one image (default: 30)")
    parsed, ros_args = parser.parse_known_args(args)
    if not parsed.topic.strip():
        parser.error("--topic must not be empty")
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        parser.error("--timeout must be a finite positive number")
    if parsed.output.suffix.lower() != ".png":
        parser.error("--output must have a .png extension")

    output = parsed.output.expanduser().resolve()
    metadata_path = output.with_suffix(".json")
    if output.exists() or metadata_path.exists():
        print(f"EYES_OF_LUNA_CAPTURE_FAIL: output or sidecar already exists: {output}")
        return 2

    try:
        import cv2
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print(f"EYES_OF_LUNA_CAPTURE_FAIL: ROS 2 image dependencies unavailable: {exc}")
        return 2

    class OneFrameCapture(Node):
        def __init__(self):
            super().__init__("eyes_of_luna_one_frame_capture")
            self.bridge = CvBridge()
            self.done = False
            self.error = None
            self.metadata_path = None
            self.subscription = self.create_subscription(
                Image, parsed.topic, self._callback, qos_profile_sensor_data)

        def _callback(self, message):
            try:
                self.metadata_path = _write_capture(
                    message, parsed.topic, output, self.bridge, cv2)
            except Exception as exc:  # propagate clear one-shot errors to caller
                self.error = exc
            self.done = True

    rclpy.init(args=ros_args)
    node = OneFrameCapture()
    timed_out = False
    try:
        deadline = time.monotonic() + parsed.timeout
        while rclpy.ok() and not node.done:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                timed_out = True
                break
            rclpy.spin_once(node, timeout_sec=min(0.5, remaining))
    except KeyboardInterrupt:
        print("EYES_OF_LUNA_CAPTURE_FAIL: interrupted before a frame arrived")
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if timed_out:
        print(f"EYES_OF_LUNA_CAPTURE_FAIL: no image received on {parsed.topic} "
              f"within {parsed.timeout:g} seconds")
        return 1
    if node.error is not None:
        print(f"EYES_OF_LUNA_CAPTURE_FAIL: {node.error}")
        return 1
    print(json.dumps({
        "image": str(output),
        "metadata": str(node.metadata_path),
        "topic": parsed.topic,
        "annotation_status": "pending_human_review",
        "registration_status": "blocked_pending_capture_metadata_and_both_reviewed_masks",
    }, indent=2, sort_keys=True))
    print("EYES_OF_LUNA_CAPTURE_COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
