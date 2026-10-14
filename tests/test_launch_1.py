"""Static contracts for the separate clean Phase 1 launcher."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "launch-1"
LAUNCH_SCRIPT = ROOT / "scripts" / "launch-1.sh"


class Launch1EntryPointTests(unittest.TestCase):
    def test_root_entrypoint_is_executable_and_dispatches_to_its_script(self):
        self.assertTrue(WRAPPER.is_file())
        self.assertTrue(os.access(WRAPPER, os.X_OK))
        self.assertTrue(LAUNCH_SCRIPT.is_file())
        wrapper_text = WRAPPER.read_text(encoding="utf-8")
        self.assertIn("readlink -f", wrapper_text)
        self.assertIn("scripts/launch-1.sh", wrapper_text)

    def test_script_targets_only_the_standalone_phase1_workspace(self):
        script = LAUNCH_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('WORKSPACE_DIR="$REPO_DIR/lunabot_phase1"', script)
        self.assertIn("/opt/ros/humble/setup.bash", script)
        self.assertIn("install/setup.bash", script)
        self.assertIn("ros2 launch lunabot_phase1_bringup phase1.launch.py", script)
        self.assertIn("ros_ign_bridge", script)
        self.assertIn("command -v ign", script)

    def test_help_is_available_from_a_symlink_before_ros_preflight(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            linked_entry = Path(temp_dir) / "launch-1"
            linked_entry.symlink_to(WRAPPER)
            result = subprocess.run(
                [str(linked_entry), "--help"],
                cwd=temp_dir,
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Usage: launch-1", result.stdout)
        self.assertIn("launch-a through launch-l", result.stdout)


if __name__ == "__main__":
    unittest.main()
