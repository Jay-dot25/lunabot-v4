#!/usr/bin/env python3
import ast,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
class Tests(unittest.TestCase):
 def test_gazebo_stage_is_minimal_and_delayed(self):
  text=(R/'src/lunabot_bringup/launch/phase17_gazebo.launch.py').read_text();ast.parse(text);self.assertIn('gz_sim.launch.py',text);self.assertIn("executable='create'",text);self.assertIn('TimerAction',text);self.assertNotIn('rviz2',text);self.assertNotIn('terrain_inference',text)
 def test_world_uses_installed_lightweight_mesh(self):
  text=(R/'src/lunabot_gazebo/worlds/lunar_world.sdf').read_text();self.assertNotIn('file://meshes/',text);self.assertEqual(text.count('<uri>meshes/lunar_terrain_collision.obj</uri>'),2)
 def test_launch_is_installed(self):self.assertIn('phase17_gazebo.launch.py',(R/'src/lunabot_bringup/setup.py').read_text())
if __name__=='__main__':unittest.main()
