"""Publish calibrated RGB/depth CameraInfo missing from the Gazebo image bridge."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_camera_info_publisher")
        defaults = {
            "width": 640, "height": 480,
            "fx": 554.256, "fy": 554.256, "cx": 320.0, "cy": 240.0,
            "rgb_topic": "/lunabot/camera/camera_info",
            "depth_topic": "/lunabot/depth/camera_info",
            "rgb_frame": "sensor_head", "depth_frame": "sensor_head",
            "publish_rate": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.rgb_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("rgb_topic").value), qos)
        self.depth_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("depth_topic").value), qos)
        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self._publish()

    def _message(self, frame: str) -> CameraInfo:
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        message = CameraInfo()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame
        message.width, message.height = width, height
        message.distortion_model = "plumb_bob"
        message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _publish(self) -> None:
        self.rgb_pub.publish(self._message(str(self.get_parameter("rgb_frame").value)))
        self.depth_pub.publish(self._message(str(self.get_parameter("depth_frame").value)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisher()
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
