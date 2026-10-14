"""Static integrity checks for optional Phase 2 Gazebo capture assets."""
from __future__ import annotations

import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE = ROOT / "lunabot_phase1/src/lunabot_phase2_perception"
MODEL_ROOT = ROS_PACKAGE / "models/eyes_of_luna_terrain"
SCENES = ("scene01", "scene02", "scene03")


def _obj_stats(path: Path):
    vertices = []
    normals = []
    faces = []
    normal_refs = []
    for line in path.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == "vn":
            normals.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == "f":
            records = [value.split("/") for value in fields[1:]]
            faces.append(tuple(int(value[0]) for value in records))
            normal_refs.append(tuple(int(value[2]) if len(value) > 2 and value[2] else 0
                                     for value in records))
    return vertices, normals, faces, normal_refs


class EyesOfLunaCaptureWorldTests(unittest.TestCase):
    def test_three_worlds_have_unique_names_and_phase1_camera_rover(self):
        names = set()
        for scene in SCENES:
            path = ROS_PACKAGE / "worlds" / f"eyes_of_luna_capture_{scene}.sdf"
            root = ET.parse(path).getroot()
            self.assertEqual(root.tag, "sdf")
            world = root.find("world")
            self.assertIsNotNone(world)
            name = world.get("name")
            self.assertNotIn(name, names)
            names.add(name)
            self.assertEqual(name, f"eyes_of_luna_capture_{scene}")
            includes = [include.findtext("uri") for include in world.findall("include")]
            self.assertIn("model://lunabot_rover", includes)
            models = {model.get("name") for model in world.findall("model")}
            self.assertIn("regolith_surface", models)
            self.assertIn("bedrock_outcrop", models)
            self.assertGreaterEqual(sum(name.startswith("boulder_") for name in models), 4)
            plugins = {plugin.get("name") for plugin in world.findall("plugin")}
            self.assertIn("gz::sim::systems::SceneBroadcaster", plugins)
            self.assertIn("gz::sim::systems::Sensors", plugins)
        self.assertEqual(len(names), 3)

    def test_scene_meshes_exist_and_include_real_crater_relief(self):
        bounds = []
        for scene in SCENES:
            terrain = MODEL_ROOT / "meshes" / f"regolith_{scene}.obj"
            bedrock = MODEL_ROOT / "meshes" / f"bedrock_{scene}.obj"
            self.assertTrue(terrain.is_file())
            self.assertTrue(bedrock.is_file())
            for asset in (terrain, bedrock, MODEL_ROOT / "meshes" / "boulder_irregular.obj"):
                vertices, normals, faces, normal_refs = _obj_stats(asset)
                self.assertGreater(len(vertices), 20)
                self.assertGreater(len(faces), 20)
                self.assertEqual(len(normals), len(vertices), f"{asset} needs one normal per vertex")
                self.assertTrue(all(math.isfinite(value) for point in vertices for value in point))
                self.assertTrue(all(math.isfinite(value) for normal in normals for value in normal))
                self.assertTrue(all(abs(math.sqrt(sum(value * value for value in normal)) - 1.0) < 1e-5
                                    for normal in normals))
                self.assertTrue(all(1 <= index <= len(vertices) for face in faces for index in face))
                self.assertTrue(all(1 <= index <= len(normals)
                                    for face in normal_refs for index in face))
            terrain_vertices, _, terrain_faces, _ = _obj_stats(terrain)
            z_values = [point[2] for point in terrain_vertices]
            self.assertLess(min(z_values), -0.4, "crater bowls must dip below the surrounding regolith")
            self.assertGreater(max(z_values), 0.05, "crater rims must rise above the surrounding regolith")
            bounds.append((min(point[0] for point in terrain_vertices),
                           max(point[0] for point in terrain_vertices),
                           min(point[1] for point in terrain_vertices),
                           max(point[1] for point in terrain_vertices)))
            self.assertGreater(len(terrain_faces), 10_000)
        self.assertEqual(len(set(bounds)), 1, "scenes share terrain extents, not identical crater assets")
        self.assertNotEqual(
            (MODEL_ROOT / "meshes" / "regolith_scene01.obj").read_bytes(),
            (MODEL_ROOT / "meshes" / "regolith_scene02.obj").read_bytes(),
        )

    def test_world_mesh_uris_and_camera_topic_are_resolvable(self):
        rover_sdf = (ROOT / "lunabot_phase1/src/lunabot_phase1_description/models/"
                     "lunabot_rover/model.sdf")
        rover_root = ET.parse(rover_sdf).getroot()
        camera_topics = [sensor.findtext("topic") for sensor in rover_root.findall(".//sensor")
                         if sensor.get("name") == "front_camera"]
        self.assertEqual(camera_topics, ["/camera/image_raw"])
        for scene in SCENES:
            root = ET.parse(ROS_PACKAGE / "worlds" / f"eyes_of_luna_capture_{scene}.sdf").getroot()
            world_text = " ".join(element.text or "" for element in root.iter("uri"))
            for asset_name in (f"regolith_{scene}.obj", f"bedrock_{scene}.obj", "boulder_irregular.obj"):
                self.assertIn(f"model://eyes_of_luna_terrain/meshes/{asset_name}", world_text)
                self.assertTrue((MODEL_ROOT / "meshes" / asset_name).is_file())

    def test_optional_capture_launch_does_not_start_inference_or_change_phase1(self):
        launch_path = ROS_PACKAGE / "launch/eyes_of_luna_capture.launch.py"
        source = launch_path.read_text(encoding="utf-8")
        ast.parse(source, filename=str(launch_path))
        self.assertIn('"scene01", "scene02", "scene03"', source)
        self.assertIn('"ign", "gazebo", "-r", "-v", "3"', source)
        self.assertIn('"override_frame_id": "camera_optical_frame"', source)
        self.assertIn('"lunabot_phase1_tools"', source)
        self.assertNotIn("eyes_of_luna_node", source)
        phase1_launch = (ROOT / "lunabot_phase1/src/lunabot_phase1_bringup/launch/"
                         "phase1.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("eyes_of_luna_capture", phase1_launch)
        self.assertNotIn("lunabot_phase2_perception", phase1_launch)

    def test_generated_assets_are_installed_with_phase2_package(self):
        setup = (ROS_PACKAGE / "setup.py").read_text(encoding="utf-8")
        self.assertIn('_install_tree("worlds")', setup)
        self.assertIn('_install_tree("models")', setup)
        manifest = ET.parse(ROS_PACKAGE / "package.xml").getroot()
        dependencies = {item.text for item in manifest.findall("exec_depend")}
        self.assertTrue({
            "ros_ign_bridge", "lunabot_phase1_bringup", "lunabot_phase1_description",
            "lunabot_phase1_tools", "tf2_ros",
        }.issubset(dependencies))


if __name__ == "__main__":
    unittest.main()
