"""Static integration contracts for the isolated Phase 1 workspace."""

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
DESCRIPTION = SRC / "lunabot_phase1_description"
SIMULATION = SRC / "lunabot_phase1_simulation"
BRINGUP = SRC / "lunabot_phase1_bringup"
TOOLS = SRC / "lunabot_phase1_tools"


class ProjectFileTests(unittest.TestCase):
    def test_ros_package_manifests_are_well_formed_and_unique(self):
        manifests = sorted(SRC.glob("*/package.xml"))
        self.assertEqual(len(manifests), 4)
        names = []
        for manifest in manifests:
            package = ET.parse(manifest).getroot()
            self.assertEqual(package.tag, "package")
            names.append(package.findtext("name"))
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(
            set(names),
            {
                "lunabot_phase1_description",
                "lunabot_phase1_simulation",
                "lunabot_phase1_tools",
                "lunabot_phase1_bringup",
            },
        )

    def test_world_and_rover_sdf_are_xml_and_use_the_fortress_schema(self):
        world_path = SIMULATION / "worlds" / "lunar_base_camp.sdf"
        model_path = DESCRIPTION / "models" / "lunabot_rover" / "model.sdf"
        for path in (world_path, model_path):
            root = ET.parse(path).getroot()
            self.assertEqual(root.tag, "sdf")
            self.assertEqual(root.attrib.get("version"), "1.8")

    def test_fortress_world_system_plugins_match_fortress_library_and_class_names(self):
        world = ET.parse(SIMULATION / "worlds" / "lunar_base_camp.sdf").getroot().find("world")
        plugins = {plugin.attrib["name"]: plugin for plugin in world.findall("plugin")}
        expected = {
            "gz::sim::systems::Physics": "ignition-gazebo-physics-system",
            "gz::sim::systems::UserCommands": "ignition-gazebo-user-commands-system",
            "gz::sim::systems::SceneBroadcaster": "ignition-gazebo-scene-broadcaster-system",
            "gz::sim::systems::Sensors": "ignition-gazebo-sensors-system",
        }
        for plugin_name, library in expected.items():
            with self.subTest(plugin=plugin_name):
                self.assertEqual(plugins[plugin_name].attrib["filename"], library)
        self.assertEqual(plugins["gz::sim::systems::Sensors"].findtext("render_engine"), "ogre2")

    def test_world_has_lunar_gravity_physical_obstacles_and_included_rover(self):
        world = ET.parse(SIMULATION / "worlds" / "lunar_base_camp.sdf").getroot().find("world")
        self.assertEqual(world.attrib["name"], "lunar_base_camp")
        self.assertEqual(world.findtext("gravity"), "0 0 -1.62")
        self.assertIsNotNone(world.find("include/uri"))
        self.assertEqual(world.findtext("include/uri"), "model://lunabot_rover")

        models = {model.attrib["name"]: model for model in world.findall("model")}
        for name in (
            "small_rock",
            "large_boulder",
            "low_obstacle",
            "forward_blocker",
            "cluster_rock_a",
            "cluster_rock_b",
            "cluster_rock_c",
        ):
            with self.subTest(obstacle=name):
                model = models[name]
                self.assertEqual(model.findtext("static"), "true")
                self.assertIsNotNone(model.find("link/collision/geometry"))
                self.assertIsNotNone(model.find("link/visual/geometry"))

    def test_rover_has_one_diff_drive_and_real_wheel_joints(self):
        model = ET.parse(
            DESCRIPTION / "models" / "lunabot_rover" / "model.sdf"
        ).getroot().find("model")
        plugins = model.findall("plugin")
        drive = [p for p in plugins if p.attrib.get("name") == "gz::sim::systems::DiffDrive"]
        self.assertEqual(len(drive), 1)
        self.assertEqual(drive[0].attrib.get("filename"), "ignition-gazebo-diff-drive-system")
        self.assertEqual(drive[0].findtext("topic"), "/cmd_vel_sim")
        self.assertEqual(drive[0].findtext("odom_topic"), "/odom")
        self.assertEqual(drive[0].findtext("tf_topic"), "/tf")
        self.assertEqual(drive[0].findtext("frame_id"), "odom")
        self.assertEqual(drive[0].findtext("child_frame_id"), "base_footprint")
        self.assertEqual(drive[0].findtext("left_joint"), "left_wheel_joint")
        self.assertEqual(drive[0].findtext("right_joint"), "right_wheel_joint")
        joint_names = {joint.attrib["name"] for joint in model.findall("joint")}
        self.assertTrue({"left_wheel_joint", "right_wheel_joint"}.issubset(joint_names))
        self.assertEqual(drive[0].findtext("wheel_radius"), "0.16")
        self.assertEqual(drive[0].findtext("wheel_separation"), "0.62")
        self.assertEqual(drive[0].findtext("max_linear_velocity"), "0.35")
        self.assertEqual(drive[0].findtext("max_angular_velocity"), "0.80")
        guard_config = (BRINGUP / "config" / "command_guard.yaml").read_text(encoding="utf-8")
        self.assertIn("linear_velocity_limit_mps: 0.35", guard_config)
        self.assertIn("angular_velocity_limit_radps: 0.80", guard_config)
        for wheel in ("left_wheel", "right_wheel"):
            self.assertIsNotNone(model.find(f"link[@name='{wheel}']/collision"))

    def test_live_camera_and_lidar_are_configured_in_the_rover(self):
        model = ET.parse(
            DESCRIPTION / "models" / "lunabot_rover" / "model.sdf"
        ).getroot().find("model")
        sensors = {sensor.attrib["type"]: sensor for sensor in model.findall(".//sensor")}
        self.assertIn("camera", sensors)
        self.assertIn("gpu_lidar", sensors)
        camera = sensors["camera"]
        self.assertEqual(camera.findtext("topic"), "/camera/image_raw")
        self.assertEqual(camera.findtext("camera/image/width"), "640")
        self.assertEqual(camera.findtext("camera/image/height"), "480")
        self.assertEqual(camera.findtext("camera/optical_frame_id"), "camera_optical_frame")
        self.assertEqual(camera.findtext("update_rate"), "20")
        lidar = sensors["gpu_lidar"]
        self.assertEqual(lidar.findtext("topic"), "/scan")
        self.assertEqual(lidar.findtext("ray/scan/horizontal/samples"), "720")
        self.assertEqual(lidar.findtext("ray/range/min"), "0.12")
        self.assertEqual(lidar.findtext("ray/range/max"), "15.0")
        self.assertEqual(lidar.findtext("ray/noise/type"), "gaussian")
        self.assertIsNone(lidar.find("ray/range/noise"))

    def test_static_tf_extrinsics_match_the_sdf_sensor_mounts(self):
        model = ET.parse(
            DESCRIPTION / "models" / "lunabot_rover" / "model.sdf"
        ).getroot().find("model")
        camera_pose = model.find("link[@name='camera_link']/pose").text
        lidar_pose = model.find("link[@name='lidar_link']/pose").text
        self.assertEqual(camera_pose, "0.40 0 0.28 0 0 0")
        self.assertEqual(lidar_pose, "0.06 0 0.20 0 0 0")
        integrated = (BRINGUP / "launch" / "phase1.launch.py").read_text(encoding="utf-8")
        self.assertIn('(0.0, 0.0, 0.32)', integrated)
        self.assertIn('(0.40, 0.0, 0.28)', integrated)
        self.assertIn('(0.06, 0.0, 0.20)', integrated)
        self.assertIn('(-1.57079632679, 0.0, -1.57079632679)', integrated)

    def test_obstacle_monitor_has_threshold_angle_and_scan_watchdog(self):
        config = (BRINGUP / "config" / "obstacle_monitor.yaml").read_text(encoding="utf-8")
        self.assertIn("warning_distance_m: 1.50", config)
        self.assertIn("forward_half_angle_rad: 0.5235987756", config)
        self.assertIn("scan_timeout_sec: 1.00", config)
        node = (TOOLS / "lunabot_phase1_tools" / "obstacle_monitor_node.py").read_text(encoding="utf-8")
        self.assertIn("scan_is_stale", node)
        self.assertNotIn("/cmd_vel", node)

    def test_bridge_maps_humble_fortress_ros_contracts_and_guarded_command(self):
        bridge = (BRINGUP / "config" / "bridge.yaml").read_text(encoding="utf-8")
        for topic in ("/clock", "/odom", "/tf", "/cmd_vel_sim"):
            self.assertIn(f'ros_topic_name: "{topic}"', bridge)
            self.assertIn(f'gz_topic_name: "{topic}"', bridge)
        self.assertIn('gz_type_name: "ignition.msgs.Odometry"', bridge)
        self.assertIn('gz_type_name: "ignition.msgs.Pose_V"', bridge)
        self.assertIn('gz_type_name: "ignition.msgs.Twist"', bridge)
        self.assertNotIn('ros_topic_name: "/cmd_vel"', bridge)
        self.assertNotIn('gz_topic_name: "/cmd_vel"', bridge)

        camera = (BRINGUP / "config" / "camera_bridge.yaml").read_text(encoding="utf-8")
        self.assertIn('ros_topic_name: "/camera/image_raw"', camera)
        self.assertIn('ros_topic_name: "/camera/camera_info"', camera)
        self.assertIn('gz_type_name: "ignition.msgs.Image"', camera)
        self.assertIn('gz_type_name: "ignition.msgs.CameraInfo"', camera)
        lidar = (BRINGUP / "config" / "lidar_bridge.yaml").read_text(encoding="utf-8")
        self.assertIn('ros_topic_name: "/scan"', lidar)
        self.assertIn('gz_type_name: "ignition.msgs.LaserScan"', lidar)
        for config in (bridge, camera, lidar):
            self.assertNotIn("qos_profile:", config)
            self.assertNotIn("frame_id:", config)

        launch = (BRINGUP / "launch" / "phase1.launch.py").read_text(encoding="utf-8")
        self.assertIn('"override_frame_id": "camera_optical_frame"', launch)
        self.assertIn('"override_frame_id": "lidar_link"', launch)

    def test_humble_fortress_target_is_consistent_across_launch_manifest_and_smoke_check(self):
        launch = (BRINGUP / "launch" / "phase1.launch.py").read_text(encoding="utf-8")
        self.assertIn('["ign", "gazebo", "-r", "-v", "3", world]', launch)
        self.assertIn('name="IGN_GAZEBO_RESOURCE_PATH"', launch)
        self.assertNotIn("ros_gz_sim", launch)

        manifest = ET.parse(BRINGUP / "package.xml").getroot()
        dependencies = {item.text for item in manifest.findall("exec_depend")}
        self.assertIn("ros_ign_bridge", dependencies)
        self.assertNotIn("ros_gz_sim", dependencies)

        smoke = (ROOT / "scripts" / "phase1_smoke_test.sh").read_text(encoding="utf-8")
        self.assertIn('"${ROS_DISTRO}" != "humble"', smoke)
        self.assertIn("/opt/ros/humble/setup.bash", smoke)
        self.assertIn("wait_for_node()", smoke)
        self.assertIn('grep -Eq "(^|/)${node_name}$"', smoke)
        self.assertNotIn('"${ROS_DISTRO}" != "jazzy"', smoke)
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("Ubuntu 22.04 (Jammy) + ROS 2 Humble + Gazebo Fortress", readme)
        self.assertNotIn("Gazebo Harmonic", readme)

    def test_launch_files_parse_and_keep_one_dynamic_tf_owner(self):
        launch_files = sorted((BRINGUP / "launch").glob("*.launch.py"))
        self.assertEqual({p.name for p in launch_files}, {
            "phase1.launch.py", "monitor.launch.py", "teleop.launch.py"
        })
        for path in launch_files:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        integrated = (BRINGUP / "launch" / "phase1.launch.py").read_text(encoding="utf-8")
        self.assertEqual(integrated.count('"tf_topic"'), 0)  # TF is configured in SDF.
        self.assertEqual(integrated.count('"base_footprint", "base_link"'), 1)
        self.assertIn('"camera_optical_frame"', integrated)
        self.assertIn('"lidar_link"', integrated)

    def test_ros_nodes_defer_sigint_shutdown_until_best_effort_stop(self):
        guard = (TOOLS / "lunabot_phase1_tools" / "command_guard_node.py").read_text(encoding="utf-8")
        teleop = (TOOLS / "lunabot_phase1_tools" / "keyboard_teleop.py").read_text(encoding="utf-8")
        for node_source in (guard, teleop):
            self.assertIn("SignalHandlerOptions.NO", node_source)
            self.assertIn("if rclpy.ok():", node_source)
            self.assertIn("except Exception as exc:", node_source)
        self.assertIn("node.publish_stop_burst()", guard)
        self.assertIn("node.publish(0.0, 0.0)", teleop)

    def test_python_entry_points_and_ros_runtime_dependencies_are_declared(self):
        setup = (TOOLS / "setup.py").read_text(encoding="utf-8")
        for entry in ("command_guard", "obstacle_monitor", "keyboard_teleop"):
            self.assertIn(entry, setup)
        manifest = ET.parse(TOOLS / "package.xml").getroot()
        dependencies = {item.text for item in manifest.findall("exec_depend")}
        self.assertTrue({"rclpy", "geometry_msgs", "sensor_msgs", "std_msgs"}.issubset(dependencies))


if __name__ == "__main__":
    unittest.main()
