#!/usr/bin/env python3
"""Capture synchronized Gazebo RGB-D frames for auditable Phase E training.

Run while launch-e is active. Frames are stored as compressed NumPy archives
plus PPM previews; no labels are inferred or fabricated by this tool.
"""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class Capture(Node):
    def __init__(self):
        super().__init__('phase_e_dataset_capture')
        self.declare_parameter('rgb_topic', '/lunabot/camera/image_raw')
        self.declare_parameter('depth_topic', '/lunabot/depth/image_raw')
        self.declare_parameter('output_dir', 'datasets/phase_e_gazebo')
        self.declare_parameter('count', 30)
        self.declare_parameter('interval', 2.0)
        g = self.get_parameter
        self.out = Path(str(g('output_dir').value)).expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.target = max(1, int(g('count').value))
        self.interval = max(.25, float(g('interval').value))
        self.rgb = self.depth = None
        self.last_saved_stamp = -1
        self.saved = 0
        self.last_wall = 0.0
        self.create_subscription(Image, str(g('rgb_topic').value), self.on_rgb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, str(g('depth_topic').value), self.on_depth,
                                 qos_profile_sensor_data)
        self.timer = self.create_timer(.1, self.tick)
        self.get_logger().info(f'CAPTURE_READY output={self.out} target={self.target}')

    @staticmethod
    def stamp(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def rows(msg, bytes_per_pixel):
        raw = np.frombuffer(bytes(msg.data), np.uint8)
        need = int(msg.step) * int(msg.height)
        if raw.size < need or msg.step < msg.width * bytes_per_pixel:
            raise ValueError('short image buffer')
        return raw[:need].reshape(msg.height, msg.step)[:, :msg.width * bytes_per_pixel].copy()

    def decode_rgb(self, msg):
        enc = msg.encoding.lower()
        channels = 4 if enc in ('rgba8', 'bgra8') else 3
        if enc not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            raise ValueError(f'unsupported RGB encoding {msg.encoding}')
        a = self.rows(msg, channels).reshape(msg.height, msg.width, channels)[:, :, :3]
        if enc.startswith('bgr'):
            a = a[:, :, ::-1]
        return np.ascontiguousarray(a, dtype=np.uint8)

    def decode_depth(self, msg):
        enc = msg.encoding.lower().replace(' ', '')
        if enc in ('16uc1', 'mono16'):
            dt, scale, size = ('>u2' if msg.is_bigendian else '<u2'), .001, 2
        elif enc in ('32fc1', '32fc'):
            dt, scale, size = ('>f4' if msg.is_bigendian else '<f4'), 1., 4
        elif enc in ('64fc1', '64fc'):
            dt, scale, size = ('>f8' if msg.is_bigendian else '<f8'), 1., 8
        else:
            raise ValueError(f'unsupported depth encoding {msg.encoding}')
        return self.rows(msg, size).view(dt).reshape(msg.height, msg.width).astype(np.float32) * scale

    def on_rgb(self, msg): self.rgb = msg
    def on_depth(self, msg): self.depth = msg

    def tick(self):
        if self.saved >= self.target:
            self.get_logger().info(f'CAPTURE_COMPLETE frames={self.saved} output={self.out}')
            rclpy.shutdown(); return
        if self.rgb is None or self.depth is None or time.monotonic() - self.last_wall < self.interval:
            return
        rs, ds = self.stamp(self.rgb), self.stamp(self.depth)
        if rs == self.last_saved_stamp or abs(rs - ds) > 200_000_000:
            return
        try:
            rgb, depth = self.decode_rgb(self.rgb), self.decode_depth(self.depth)
        except ValueError as exc:
            self.get_logger().error(str(exc)); return
        idx = self.saved
        stem = f'frame_{idx:04d}'
        np.savez_compressed(self.out / f'{stem}.npz', rgb=rgb, depth=depth,
                            rgb_stamp_ns=np.int64(rs), depth_stamp_ns=np.int64(ds))
        with (self.out / f'{stem}.ppm').open('wb') as f:
            f.write(f'P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n'.encode()); f.write(rgb.tobytes())
        meta = {'frame': stem, 'rgb_topic': self.rgb.header.frame_id,
                'depth_frame': self.depth.header.frame_id, 'rgb_encoding': self.rgb.encoding,
                'depth_encoding': self.depth.encoding, 'rgb_stamp_ns': rs,
                'depth_stamp_ns': ds, 'shape': list(rgb.shape)}
        (self.out / f'{stem}.json').write_text(json.dumps(meta, indent=2) + '\n')
        self.saved += 1; self.last_saved_stamp = rs; self.last_wall = time.monotonic()
        self.get_logger().info(f'CAPTURED {self.saved}/{self.target} {stem}')

def main():
    rclpy.init(); node = Capture()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
