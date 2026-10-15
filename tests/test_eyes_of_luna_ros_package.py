"""Static ROS package contracts; these do not exercise a ROS runtime."""
from __future__ import annotations

import ast
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

PACKAGE_IMPORT_ROOT = Path(__file__).resolve().parents[1] / "lunabot_phase1/src/lunabot_phase2_perception"
sys.path.insert(0, str(PACKAGE_IMPORT_ROOT))
from lunabot_phase2_perception.image_conversion import (  # noqa: E402
    ImageEncodingError,
    decode_image_rgb,
)
from lunabot_phase2_perception.capture_one_frame import (  # noqa: E402
    _write_capture,
    build_capture_sidecar,
)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ml"))
from extract_eyes_of_luna_frames import save_capture_sample  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "lunabot_phase1/src/lunabot_phase2_perception"


class EyesOfLunaRosPackageTests(unittest.TestCase):
    def test_manifest_and_setup_register_the_ament_python_executable(self):
        manifest = ET.parse(PACKAGE / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "lunabot_phase2_perception")
        self.assertEqual(manifest.findtext("export/build_type"), "ament_python")
        dependencies = {element.text for element in manifest.findall("exec_depend")}
        self.assertTrue({
            "rclpy", "sensor_msgs", "std_msgs", "cv_bridge", "launch", "launch_ros",
        }.issubset(dependencies))
        setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
        self.assertIn("eyes_of_luna_node = lunabot_phase2_perception.eyes_of_luna_node:main", setup)
        self.assertIn("capture_one_camera_frame = lunabot_phase2_perception.capture_one_frame:main", setup)
        self.assertIn("launch/*.launch.py", setup)

    def test_node_and_launch_are_syntactically_valid_and_use_phase1_camera(self):
        node_path = PACKAGE / "lunabot_phase2_perception/eyes_of_luna_node.py"
        launch_path = PACKAGE / "launch/eyes_of_luna.launch.py"
        node_source = node_path.read_text(encoding="utf-8")
        launch_source = launch_path.read_text(encoding="utf-8")
        ast.parse(node_source, filename=str(node_path))
        ast.parse(launch_source, filename=str(launch_path))
        self.assertIn('"image_topic": "/camera/image_raw"', node_source)
        self.assertIn('default_value="/camera/image_raw"', launch_source)
        self.assertIn('encoding="mono8"', node_source)
        self.assertIn('"/eyes_of_luna/terrain_mask"', node_source)
        self.assertIn('"/eyes_of_luna/shadow_mask"', node_source)
        self.assertIn('"/eyes_of_luna/status"', node_source)

    def test_camera_decoder_supports_phase1_rgb_encoding_and_padded_rows(self):
        import numpy as np

        image = SimpleNamespace(
            width=2, height=1, step=8, encoding="R8G8B8",
            data=bytes((10, 20, 30, 40, 50, 60, 0, 0)),
        )
        decoded = decode_image_rgb(image)
        self.assertEqual(decoded.shape, (1, 2, 3))
        self.assertEqual(decoded.tolist(), [[[10, 20, 30], [40, 50, 60]]])
        self.assertEqual(decoded.dtype, np.uint8)
        self.assertTrue(decoded.flags.c_contiguous)

    def test_camera_decoder_normalizes_bgr_bgra_and_mono(self):
        bgr = SimpleNamespace(width=1, height=1, step=3, encoding="bgr8",
                              data=bytes((3, 2, 1)))
        bgra = SimpleNamespace(width=1, height=1, step=4, encoding="bgra8",
                               data=bytes((3, 2, 1, 255)))
        mono = SimpleNamespace(width=1, height=1, step=1, encoding="mono8",
                               data=bytes((7,)))
        self.assertEqual(decode_image_rgb(bgr).tolist(), [[[1, 2, 3]]])
        self.assertEqual(decode_image_rgb(bgra).tolist(), [[[1, 2, 3]]])
        self.assertEqual(decode_image_rgb(mono).tolist(), [[[7, 7, 7]]])

    def test_camera_decoder_rejects_unknown_encoding_and_short_data(self):
        unknown = SimpleNamespace(width=1, height=1, step=3, encoding="yuv422",
                                  data=bytes((1, 2, 3)))
        short = SimpleNamespace(width=2, height=1, step=6, encoding="rgb8",
                                data=bytes((1, 2, 3)))
        with self.assertRaisesRegex(ImageEncodingError, "unsupported"):
            decode_image_rgb(unknown)
        with self.assertRaisesRegex(ImageEncodingError, "shorter"):
            decode_image_rgb(short)

    def test_bag_extractor_saves_rgb_png_and_unreviewed_provenance(self):
        import json
        import numpy as np
        from PIL import Image

        image = SimpleNamespace(
            width=2, height=1, step=6, encoding="R8G8B8",
            data=bytes((10, 20, 30, 40, 50, 60)),
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=123, nanosec=456), frame_id="camera_optical_frame"),
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            record = save_capture_sample(
                image, output, "session01", 12, 9_000_000_123, "bag01", "/camera/image_raw")
            rgb = np.asarray(Image.open(output / record["rgb"]))
            sidecar = json.loads((output / record["capture_metadata"]).read_text())
        self.assertEqual(record["sample_id"], "session01_frame000012")
        self.assertEqual(rgb.tolist(), [[[10, 20, 30], [40, 50, 60]]])
        self.assertEqual(sidecar["capture"]["timestamp_ns"], 123_000_000_456)
        self.assertEqual(sidecar["capture"]["bag_timestamp_ns"], 9_000_000_123)
        self.assertEqual(sidecar["capture"]["encoding"], "R8G8B8")
        self.assertEqual(sidecar["capture"]["frame_id"], "camera_optical_frame")
        self.assertEqual(sidecar["annotation_status"], "pending_human_review")
        self.assertNotIn("annotations", sidecar)

    def test_live_frame_sidecar_preserves_ros_metadata_and_stays_unreviewed(self):
        image = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=123, nanosec=456),
                frame_id="camera_optical_frame",
            ),
            width=640,
            height=480,
            encoding="rgb8",
            step=1920,
            is_bigendian=0,
        )
        sidecar = build_capture_sidecar(
            image, "/camera/image_raw", Path("scene01_frame.png"),
            "a" * 64, "2026-10-15T00:00:00+00:00")
        capture = sidecar["capture"]
        self.assertEqual(sidecar["sample_id"], "scene01_frame")
        self.assertEqual(capture["timestamp_ns"], 123_000_000_456)
        self.assertEqual(capture["frame_id"], "camera_optical_frame")
        self.assertEqual(capture["width"], 640)
        self.assertEqual(capture["height"], 480)
        self.assertEqual(capture["encoding"], "rgb8")
        self.assertEqual(capture["step"], 1920)
        self.assertEqual(capture["image_sha256"], "a" * 64)
        self.assertEqual(sidecar["annotation_status"], "pending_human_review")
        self.assertEqual(
            sidecar["registration_status"],
            "blocked_pending_capture_metadata_and_both_reviewed_masks")
        self.assertNotIn("annotations", sidecar)
        self.assertTrue(any("pose" in value for value in
                            sidecar["required_before_dataset_registration"]))

    def test_one_frame_writer_binds_metadata_hash_and_never_overwrites(self):
        import hashlib
        import json

        image = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=7, nanosec=9), frame_id="camera_optical_frame"),
            width=1, height=1, encoding="rgb8", step=3, is_bigendian=0,
        )

        class Bridge:
            @staticmethod
            def imgmsg_to_cv2(message, desired_encoding):
                self.assertEqual(desired_encoding, "bgr8")
                return b"encoded-bgr-frame"

        class Cv2:
            @staticmethod
            def imwrite(path, frame):
                Path(path).write_bytes(frame)
                return True

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "scene01_frame.png"
            metadata_path = _write_capture(image, "/camera/image_raw", output, Bridge(), Cv2())
            saved = output.read_bytes()
            sidecar = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(sidecar["capture"]["image_sha256"],
                             hashlib.sha256(saved).hexdigest())
            self.assertEqual(metadata_path.name, "scene01_frame.json")
            with self.assertRaises(FileExistsError):
                _write_capture(image, "/camera/image_raw", output, Bridge(), Cv2())
            self.assertEqual(output.read_bytes(), saved)

    def test_phase2_launch_does_not_change_standalone_phase1_launch(self):
        phase1_launch = ROOT / "lunabot_phase1/src/lunabot_phase1_bringup/launch/phase1.launch.py"
        source = phase1_launch.read_text(encoding="utf-8")
        self.assertNotIn("lunabot_phase2_perception", source)
        self.assertNotIn("eyes_of_luna_node", source)


if __name__ == "__main__":
    unittest.main()
