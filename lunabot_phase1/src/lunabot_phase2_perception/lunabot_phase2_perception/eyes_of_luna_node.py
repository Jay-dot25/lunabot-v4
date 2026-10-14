"""ROS 2 image subscriber publishing independent terrain and shadow masks."""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .image_conversion import decode_image_rgb


class EyesOfLunaNode(Node):
    def __init__(self):
        super().__init__("eyes_of_luna_perception")
        defaults = {
            "model_bundle_path": "",
            "ml_path": os.environ.get("LUNABOT_ML_PATH", ""),
            "device": "auto",
            "image_topic": "/camera/image_raw",
            "terrain_topic": "/eyes_of_luna/terrain_mask",
            "shadow_topic": "/eyes_of_luna/shadow_mask",
            "status_topic": "/eyes_of_luna/status",
            "queue_size": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.frames = 0
        self.dropped_frames = 0
        self.started = time.monotonic()
        self.backend = None
        self.config = None
        self.device = "cpu"
        self.bundle = {}
        self.model_bundle_path = str(self.get_parameter("model_bundle_path").value)
        self.error = ""
        self.bridge = CvBridge()

        self.terrain_pub = self.create_publisher(
            Image, str(self.get_parameter("terrain_topic").value), 10)
        self.shadow_pub = self.create_publisher(
            Image, str(self.get_parameter("shadow_topic").value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)

        try:
            ml_path = self._resolve_ml_path(str(self.get_parameter("ml_path").value))
            sys.path.insert(0, str(ml_path))
            from lunabot_ml.eyes_of_luna_runtime import load_exported_model
            self.backend, self.config, self.bundle, self.device = load_exported_model(
                Path(self.model_bundle_path).expanduser(),
                str(self.get_parameter("device").value))
            self.get_logger().info(
                f"Eyes of Luna loaded {self.bundle['model_file']} on {self.device}")
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(f"inference disabled safely: {self.error}")

        queue_size = int(self.get_parameter("queue_size").value)
        if not 1 <= queue_size <= 100:
            self.get_logger().warning("queue_size must be 1..100; using 1")
            queue_size = 1
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._callback, qos)
        self.timer = self.create_timer(1.0, self._heartbeat)

    @staticmethod
    def _resolve_ml_path(requested: str) -> Path:
        if requested:
            path = Path(requested).expanduser().resolve()
            if (path / "lunabot_ml").is_dir():
                return path
            raise FileNotFoundError(f"ml_path does not contain lunabot_ml: {path}")
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "ml"
            if (candidate / "lunabot_ml").is_dir():
                return candidate
        raise FileNotFoundError(
            "cannot find repository ml/lunabot_ml; set the ml_path parameter or "
            "LUNABOT_ML_PATH to the repository's ml directory")

    def _publish_status(self, header=None, state="READY", detail="ok", latency_ms=0.0,
                        source_encoding="", width=0, height=0):
        message = String()
        payload = {
            "state": state,
            "detail": detail,
            "model_file": self.bundle.get("model_file", "") if self.bundle else "",
            "device": self.device,
            "inference_time_ms": float(latency_ms),
            "frames_per_second": self.frames / max(1e-6, time.monotonic() - self.started),
            "dropped_frames": self.dropped_frames,
            "source_encoding": str(source_encoding),
            "width": int(width),
            "height": int(height),
        }
        if header is not None:
            payload["stamp_sec"] = int(header.stamp.sec)
            payload["stamp_nanosec"] = int(header.stamp.nanosec)
            payload["frame_id"] = str(header.frame_id)
        message.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(message)

    def _heartbeat(self):
        if self.backend is None:
            self._publish_status(state="ERROR", detail=self.error)

    def _callback(self, image: Image) -> None:
        if self.backend is None:
            self.dropped_frames += 1
            self._publish_status(
                image.header, state="ERROR", detail=self.error,
                source_encoding=image.encoding, width=image.width, height=image.height)
            return
        try:
            from lunabot_ml.eyes_of_luna_runtime import infer_rgb
            rgb = decode_image_rgb(image)
            output = infer_rgb(self.backend, rgb, self.config, self.device)
            terrain = self.bridge.cv2_to_imgmsg(output["terrain_mask"], encoding="mono8")
            shadow = self.bridge.cv2_to_imgmsg(output["shadow_mask"], encoding="mono8")
            terrain.header = image.header
            shadow.header = image.header
            self.terrain_pub.publish(terrain)
            self.shadow_pub.publish(shadow)
            self.frames += 1
            self._publish_status(
                image.header, latency_ms=output["latency_ms"],
                source_encoding=image.encoding, width=image.width, height=image.height)
        except Exception as exc:
            self.dropped_frames += 1
            self._publish_status(
                image.header, state="DEGRADED", detail=str(exc),
                source_encoding=image.encoding, width=image.width, height=image.height)
            self.get_logger().error(f"inference frame failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = EyesOfLunaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
