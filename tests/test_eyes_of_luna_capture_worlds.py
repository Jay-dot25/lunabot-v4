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

    def test_spawn_camera_fov_covers_near_boulder_bedrock_and_crater(self):
        from lunabot_phase2.tools.generate_capture_worlds import SCENES as CAPTURE_SCENES

        rover_sdf = (ROOT / "lunabot_phase1/src/lunabot_phase1_description/models/"
                     "lunabot_rover/model.sdf")
        rover_root = ET.parse(rover_sdf).getroot()
        camera = next(sensor for sensor in rover_root.findall(".//sensor")
                      if sensor.get("name") == "front_camera")
        half_fov = float(camera.findtext("camera/horizontal_fov")) / 2.0
        far_clip = float(camera.findtext("camera/clip/far"))
        margin = math.radians(2.0)

        for scene in SCENES:
            root = ET.parse(ROS_PACKAGE / "worlds" / f"eyes_of_luna_capture_{scene}.sdf").getroot()
            world = root.find("world")
            spawn = next(include for include in world.findall("include")
                         if include.findtext("uri") == "model://lunabot_rover")
            spawn_pose = [float(value) for value in spawn.findtext("pose").split()]
            spawn_x, spawn_y, spawn_yaw = spawn_pose[0], spawn_pose[1], spawn_pose[5]
            models = {model.get("name"): model for model in world.findall("model")}
            targets = {
                "bedrock_outcrop": [float(value) for value in
                                    models["bedrock_outcrop"].findtext("pose").split()],
                "boulder_01": [float(value) for value in
                               models["boulder_01"].findtext("pose").split()],
            }
            crater_x, crater_y = CAPTURE_SCENES[scene]["craters"][0][:2]
            targets["nearest crater"] = [crater_x, crater_y]

            for name, pose in targets.items():
                distance = math.hypot(pose[0] - spawn_x, pose[1] - spawn_y)
                self.assertGreater(distance, 2.0,
                                   f"{scene} spawn is too close to {name}")
                self.assertLess(distance, far_clip,
                                f"{scene} target {name} is beyond the camera far clip")
                bearing = math.atan2(pose[1] - spawn_y, pose[0] - spawn_x)
                relative = math.atan2(math.sin(bearing - spawn_yaw),
                                      math.cos(bearing - spawn_yaw))
                self.assertLessEqual(
                    abs(relative), half_fov - margin,
                    f"{scene} spawn view misses {name} by {math.degrees(relative):.1f} degrees",
                )

    def test_scene01_initial_view_fits_feature_extents_and_vertical_relief(self):
        from lunabot_phase2.tools.generate_capture_worlds import SCENES as CAPTURE_SCENES

        scene = CAPTURE_SCENES["scene01"]
        world_root = ET.parse(
            ROS_PACKAGE / "worlds" / "eyes_of_luna_capture_scene01.sdf").getroot()
        world = world_root.find("world")
        rover_root = ET.parse(
            ROOT / "lunabot_phase1/src/lunabot_phase1_description/models/"
            "lunabot_rover/model.sdf").getroot()
        sensor = next(item for item in rover_root.findall(".//sensor")
                      if item.get("name") == "front_camera")
        camera = sensor.find("camera")
        horizontal_half_fov = float(camera.findtext("horizontal_fov")) / 2.0
        image_width = float(camera.findtext("image/width"))
        image_height = float(camera.findtext("image/height"))
        vertical_half_fov = math.atan(
            math.tan(horizontal_half_fov) / (image_width / image_height))
        margin = math.radians(2.0)

        base_link = next(link for link in rover_root.findall(".//link")
                         if link.get("name") == "base_link")
        camera_link = next(link for link in rover_root.findall(".//link")
                           if link.get("name") == "camera_link")
        base_z = float(base_link.findtext("pose").split()[2])
        camera_link_z = float(camera_link.findtext("pose").split()[2])
        sensor_pose = sensor.findtext("pose") or "0 0 0 0 0 0"
        camera_z = base_z + camera_link_z + float(sensor_pose.split()[2])

        spawn = next(include for include in world.findall("include")
                     if include.findtext("uri") == "model://lunabot_rover")
        spawn_pose = [float(value) for value in spawn.findtext("pose").split()]
        spawn_x, spawn_y = spawn_pose[0], spawn_pose[1]
        spawn_z, spawn_yaw = spawn_pose[2], spawn_pose[5]
        camera_z += spawn_z
        models = {model.get("name"): model for model in world.findall("model")}
        bedrock_pose = [float(value) for value in
                        models["bedrock_outcrop"].findtext("pose").split()]
        boulder_pose = [float(value) for value in
                        models["boulder_01"].findtext("pose").split()]

        crater_x, crater_y, radius, depth, rim_height, rim_width = scene["craters"][0]
        crater_outer_radius = radius + rim_width
        bedrock_radius = 3.1 * scene.get("bedrock_width_scale", 1.0)
        boulder_spec = scene["boulders"][0]
        boulder_mesh = MODEL_ROOT / "meshes" / "boulder_irregular.obj"
        boulder_vertices = _obj_stats(boulder_mesh)[0]
        boulder_radius = max(
            math.hypot(x * boulder_spec[2], y * boulder_spec[3])
            for x, y, _ in boulder_vertices
        )

        targets = (
            ("crater", crater_x, crater_y, crater_outer_radius),
            ("boulder", boulder_pose[0], boulder_pose[1], boulder_radius),
            ("bedrock", bedrock_pose[0], bedrock_pose[1], bedrock_radius),
        )
        target_distances = {}
        for name, x, y, radius_bound in targets:
            distance = math.hypot(x - spawn_x, y - spawn_y)
            target_distances[name] = distance
            bearing = math.atan2(y - spawn_y, x - spawn_x)
            relative = math.atan2(math.sin(bearing - spawn_yaw),
                                  math.cos(bearing - spawn_yaw))
            angular_extent = math.asin(radius_bound / distance)
            self.assertLess(
                abs(relative) + angular_extent + margin, horizontal_half_fov,
                f"scene01 {name} extent would be clipped by the horizontal FOV",
            )
            self.assertGreater(distance - radius_bound, 2.0,
                               f"scene01 spawn is too close to {name}")

        crater_boulder_separation = math.hypot(
            crater_x - boulder_pose[0], crater_y - boulder_pose[1])
        self.assertGreater(crater_boulder_separation,
                           crater_outer_radius + boulder_radius + 0.1,
                           "scene01 boulder overlaps the crater rim")
        crater_bedrock_separation = math.hypot(
            crater_x - bedrock_pose[0], crater_y - bedrock_pose[1])
        self.assertGreater(crater_bedrock_separation,
                           crater_outer_radius + bedrock_radius + 0.1,
                           "scene01 outcrop overlaps the crater rim")

        def assert_vertical_visible(name: str, target_z: float, distance: float) -> None:
            elevation = math.atan2(target_z - camera_z, distance)
            self.assertLess(
                abs(elevation) + margin, vertical_half_fov,
                f"scene01 {name} lies outside the camera's vertical FOV",
            )

        crater_distance = target_distances["crater"]
        assert_vertical_visible("crater floor", -depth, crater_distance)
        assert_vertical_visible("near crater rim", rim_height,
                                crater_distance - crater_outer_radius)
        assert_vertical_visible("far crater rim", rim_height,
                                crater_distance + crater_outer_radius)
        bedrock_scale = scene.get("bedrock_height_scale", 1.0)
        bedrock_peak = 0.025 + bedrock_scale * (0.49 - 0.025)
        self.assertGreaterEqual(bedrock_peak, 0.85,
                                "scene01 bedrock should have visible vertical relief")
        assert_vertical_visible("bedrock peak", bedrock_pose[2] + bedrock_peak,
                                target_distances["bedrock"])

    def test_scene01_layout_is_visible_without_feature_overlap(self):
        from lunabot_phase2.tools.generate_capture_worlds import SCENES as CAPTURE_SCENES

        scene = CAPTURE_SCENES["scene01"]
        world_path = ROS_PACKAGE / "worlds" / "eyes_of_luna_capture_scene01.sdf"
        root = ET.parse(world_path).getroot()
        world = root.find("world")
        spawn = next(include for include in world.findall("include")
                     if include.findtext("uri") == "model://lunabot_rover")
        spawn_pose = [float(value) for value in spawn.findtext("pose").split()]
        spawn_x, spawn_y, spawn_yaw = spawn_pose[0], spawn_pose[1], spawn_pose[5]
        models = {model.get("name"): model for model in world.findall("model")}
        bedrock_pose = [float(value) for value in
                        models["bedrock_outcrop"].findtext("pose").split()]
        boulder_pose = [float(value) for value in
                        models["boulder_01"].findtext("pose").split()]
        crater_x, crater_y, radius, _, _, rim_width = scene["craters"][0]
        outer_radius = radius + rim_width
        bedrock_radius = 3.1 * scene.get("bedrock_width_scale", 1.0)
        crater_distance = math.hypot(crater_x - spawn_x, crater_y - spawn_y)
        crater_bearing = math.atan2(crater_y - spawn_y, crater_x - spawn_x)
        crater_relative = math.atan2(math.sin(crater_bearing - spawn_yaw),
                                     math.cos(crater_bearing - spawn_yaw))

        # Put the bowl centrally in the middle distance, not near the horizon.
        self.assertGreater(crater_distance, 6.0)
        self.assertLess(crater_distance, 8.0)
        self.assertLess(abs(crater_relative), math.radians(2.0))
        self.assertGreater(crater_distance - outer_radius, 3.0,
                           "crater rim must not begin immediately in front of the rover")

        # Keep the outcrop and boulder on opposite sides so neither hides the bowl.
        target_angles = {}
        for name, pose, radius_bound in (
                ("bedrock", bedrock_pose, bedrock_radius),
                ("boulder", boulder_pose, 0.9)):
            distance = math.hypot(pose[0] - spawn_x, pose[1] - spawn_y)
            bearing = math.atan2(pose[1] - spawn_y, pose[0] - spawn_x)
            relative = math.atan2(math.sin(bearing - spawn_yaw),
                                  math.cos(bearing - spawn_yaw))
            target_angles[name] = relative
            self.assertGreater(abs(relative), math.radians(10.0), name)
            self.assertLess(abs(relative), math.radians(22.0), name)
            separation = math.hypot(pose[0] - crater_x, pose[1] - crater_y)
            self.assertGreater(separation, outer_radius + radius_bound,
                               f"{name} must not overlap the nearest crater")
        self.assertLess(target_angles["bedrock"] * target_angles["boulder"], 0.0,
                        "bedrock and boulder should be on opposite sides of the view")

        self.assertGreaterEqual(scene.get("bedrock_height_scale", 1.0), 1.8)
        bedrock_z = [point[2] for point in
                     _obj_stats(MODEL_ROOT / "meshes" / "bedrock_scene01.obj")[0]]
        self.assertGreater(max(bedrock_z), 0.85,
                           "the target outcrop needs enough relief to be visually distinct")

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
