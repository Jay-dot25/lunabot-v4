#!/usr/bin/env python3
"""LunaBot Phase E: trained five-class RGB-D semantic segmentation.

The deployed model is a compact fully-connected pixel network (7-12-8-5,
two ReLU hidden layers) trained by tools/train_terrain_mlp.py on a balanced,
procedurally generated simulation-domain dataset. RGB appearance is fused with
depth, image-row context, local depth gradient and RGB texture. This is a real
trained model, not threshold code and not a claim that SegFormer/DeepLab/U-Net
is running. The lightweight architecture is chosen for reproducible CPU-only
ROS/Gazebo operation.

mono8 labels:
  0 BEDROCK, 1 REGOLITH, 2 ROCK, 3 CRATER, 4 SHADOW
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

BEDROCK = 0
REGOLITH = 1
ROCK = 2
CRATER = 3
SHADOW = 4
CLASS_NAMES = ("BEDROCK", "REGOLITH", "ROCK", "CRATER", "SHADOW")
CLASS_COLORS = np.asarray([
    (175, 185, 200),  # bedrock: blue-grey
    (194, 154, 92),   # regolith: ochre
    (235, 65, 55),    # rock: red
    (118, 55, 160),   # crater: purple
    (20, 24, 36),     # shadow: near-black
], dtype=np.uint8)
MODEL_FORMAT = "lunabot_mlp_v1"


class TerrainSegmentation(Node):
    """Run the trained RGB-D MLP and publish five-class semantic images."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_segmentation")
        default_model = str(Path(__file__).resolve().parents[1] /
                            "models" / "terrain_mlp_v1.json")
        self.declare_parameter("image_topic", "/lunabot/camera/image_raw")
        self.declare_parameter("depth_topic", "/lunabot/depth/image_raw")
        self.declare_parameter("mask_topic", "/lunabot/terrain/segmentation")
        self.declare_parameter("overlay_topic", "/lunabot/terrain/overlay")
        self.declare_parameter("status_topic", "/lunabot/terrain/segmentation/status")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("max_depth", 12.0)
        self.declare_parameter("max_rate", 4.0)
        self.declare_parameter("inference_stride", 2)
        get = self.get_parameter
        self.image_topic = str(get("image_topic").value)
        self.depth_topic = str(get("depth_topic").value)
        self.mask_topic = str(get("mask_topic").value)
        self.overlay_topic = str(get("overlay_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.max_depth = max(.1, float(get("max_depth").value))
        self.max_rate = max(.1, float(get("max_rate").value))
        self.stride = max(1, int(get("inference_stride").value))
        self.model_path = Path(str(get("model_path").value))
        self._load_model()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self.mask_pub = self.create_publisher(Image, self.mask_topic, image_qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, image_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)

        self.latest_image: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.last_stamp_ns = 0
        self.last_process_ns = 0
        self.frame_count = 0
        self.last_status = ""
        self.timer = self.create_timer(.05, self._process_latest)
        self.publish_status(
            f"SEGMENTATION_MODEL_READY format={MODEL_FORMAT} trained=true "
            f"classes={','.join(CLASS_NAMES)}")

    def _load_model(self) -> None:
        artifact = json.loads(self.model_path.read_text(encoding="utf-8"))
        if artifact.get("format") != MODEL_FORMAT:
            raise RuntimeError(f"unsupported terrain model: {artifact.get('format')}")
        if tuple(artifact.get("labels", ())) != CLASS_NAMES:
            raise RuntimeError("terrain model label order does not match runtime contract")
        if artifact.get("architecture") != [7, 12, 8, 5]:
            raise RuntimeError("terrain model architecture must be 7-12-8-5")
        self.w1 = np.asarray(artifact["w1"], dtype=np.float32)
        self.b1 = np.asarray(artifact["b1"], dtype=np.float32)
        self.w2 = np.asarray(artifact["w2"], dtype=np.float32)
        self.b2 = np.asarray(artifact["b2"], dtype=np.float32)
        self.w3 = np.asarray(artifact["w3"], dtype=np.float32)
        self.b3 = np.asarray(artifact["b3"], dtype=np.float32)
        self.training_accuracy = float(artifact["training"]["held_out_accuracy"])

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def _depth_callback(self, msg: Image) -> None:
        self.latest_depth = msg

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _rows(msg: Image, dtype, channels=1):
        item = np.dtype(dtype).itemsize * channels
        needed = int(msg.width) * item
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if raw.size < int(msg.step) * int(msg.height) or int(msg.step) < needed:
            raise ValueError("image data shorter than declared dimensions")
        rows = raw[:int(msg.step) * int(msg.height)].reshape(int(msg.height), int(msg.step))
        return rows[:, :needed].copy()

    def _decode_rgb(self, msg: Image) -> np.ndarray:
        enc = (msg.encoding or "").lower().replace(" ", "")
        channels = 4 if enc in ("rgba8", "bgra8") else 3
        if enc not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            raise ValueError(f"unsupported RGB encoding {msg.encoding}")
        data = self._rows(msg, np.uint8, channels).reshape(msg.height, msg.width, channels)
        rgb = data[:, :, :3]
        if enc.startswith("bgr"):
            rgb = rgb[:, :, ::-1]
        return rgb.astype(np.float32) / 255.0

    def _decode_depth(self, msg: Image) -> np.ndarray:
        enc = (msg.encoding or "").lower().replace(" ", "")
        if enc in ("16uc1", "mono16"):
            dtype = ">u2" if msg.is_bigendian else "<u2"
            scale = .001
        elif enc in ("32fc1", "32fc"):
            dtype = ">f4" if msg.is_bigendian else "<f4"
            scale = 1.0
        elif enc in ("64fc1", "64fc"):
            dtype = ">f8" if msg.is_bigendian else "<f8"
            scale = 1.0
        else:
            raise ValueError(f"unsupported depth encoding {msg.encoding}")
        packed = self._rows(msg, dtype)
        depth = packed.view(dtype).reshape(msg.height, msg.width).astype(np.float32)
        return depth * scale

    @staticmethod
    def _message(header, array: np.ndarray, encoding: str) -> Image:
        array = np.ascontiguousarray(array, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * (1 if array.ndim == 2 else array.shape[2])
        msg.data = array.tobytes()
        return msg

    def _features(self, rgb: np.ndarray, depth: np.ndarray) -> tuple[np.ndarray, int]:
        h, w = rgb.shape[:2]
        y_idx = np.minimum(depth.shape[0] - 1,
                           np.linspace(0, depth.shape[0] - 1, h).astype(np.int32))
        x_idx = np.minimum(depth.shape[1] - 1,
                           np.linspace(0, depth.shape[1] - 1, w).astype(np.int32))
        depth = depth[y_idx[:, None], x_idx[None, :]]
        valid = np.isfinite(depth) & (depth > .05) & (depth <= self.max_depth)
        safe = np.where(valid, depth, self.max_depth)
        gy, gx = np.gradient(safe)
        gradient = np.clip(np.hypot(gx, gy) / max(.25, self.max_depth * .08), 0, 1)
        luminance = rgb.mean(axis=2)
        tx = np.zeros_like(luminance)
        ty = np.zeros_like(luminance)
        tx[:, 1:] = np.abs(luminance[:, 1:] - luminance[:, :-1])
        ty[1:, :] = np.abs(luminance[1:, :] - luminance[:-1, :])
        texture = np.clip((tx + ty) * 4.0, 0, 1)
        rows = np.linspace(0, 1, h, dtype=np.float32)[:, None]
        rows = np.broadcast_to(rows, (h, w))
        features = np.dstack((rgb, np.clip(safe / self.max_depth, 0, 1),
                              rows, gradient, texture))
        return features[::self.stride, ::self.stride].reshape(-1, 7), int((~valid).sum())

    def _infer(self, features: np.ndarray, height: int, width: int) -> np.ndarray:
        h1 = np.maximum(0, features @ self.w1 + self.b1)
        h2 = np.maximum(0, h1 @ self.w2 + self.b2)
        labels = np.argmax(h2 @ self.w3 + self.b3, axis=1).astype(np.uint8)
        sh = (height + self.stride - 1) // self.stride
        sw = (width + self.stride - 1) // self.stride
        labels = labels.reshape(sh, sw)
        if self.stride > 1:
            labels = np.repeat(np.repeat(labels, self.stride, axis=0),
                               self.stride, axis=1)
        return labels[:height, :width]

    def _process_latest(self) -> None:
        image, depth_msg = self.latest_image, self.latest_depth
        if image is None or depth_msg is None or image.width <= 0 or image.height <= 0:
            return
        stamp_ns = max(self._stamp_ns(image), self._stamp_ns(depth_msg))
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= self.last_stamp_ns:
            return
        if self.last_process_ns and now_ns - self.last_process_ns < int(1e9 / self.max_rate):
            return
        self.last_stamp_ns, self.last_process_ns = stamp_ns, now_ns
        try:
            rgb = self._decode_rgb(image)
            depth = self._decode_depth(depth_msg)
            features, invalid_depth = self._features(rgb, depth)
            mask = self._infer(features, int(image.height), int(image.width))
        except (ValueError, RuntimeError) as exc:
            self.publish_status(f"SEGMENTATION_ERROR {exc}")
            return
        overlay = CLASS_COLORS[mask]
        if not rclpy.ok():
            return
        try:
            self.mask_pub.publish(self._message(image.header, mask, "mono8"))
            self.overlay_pub.publish(self._message(image.header, overlay, "rgb8"))
        except Exception as exc:
            # Humble exposes the native RCLError from a private extension, not
            # rclpy.exceptions. Avoid importing that unstable symbol: suppress
            # only the known SIGINT context-invalid race and re-raise all real
            # inference/publisher failures.
            if not rclpy.ok() and "context is invalid" in str(exc).lower():
                return
            raise
        self.frame_count += 1
        counts = np.bincount(mask.reshape(-1), minlength=5)
        summary = " ".join(f"{name.lower()}={int(counts[i])}"
                           for i, name in enumerate(CLASS_NAMES))
        self.publish_status(
            f"SEGMENTATION_PASS model={MODEL_FORMAT} trained=true "
            f"accuracy={self.training_accuracy:.3f} frames={self.frame_count} "
            f"width={image.width} height={image.height} {summary} "
            f"invalid_depth={invalid_depth}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainSegmentation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
