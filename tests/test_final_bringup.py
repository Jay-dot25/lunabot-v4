#!/usr/bin/env python3
import ast,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
class Tests(unittest.TestCase):
 def setUp(self):self.path=R/'src/lunabot_bringup/launch/lunabot_goal.launch.py';self.text=self.path.read_text()
 def test_launch_syntax_and_modes(self):
  ast.parse(self.text);self.assertIn("choices=['simulation','bag_replay','evaluation']",self.text);self.assertIn("'ros2','bag','play'",self.text);self.assertIn("'gz_sim.launch.py'",self.text)
 def test_complete_production_graph(self):
  for executable in ('terrain_inference_node','semantic_fusion_node','traversability_node','dstar_lite_planner','replan_verifier','regulated_path_follower','safety_supervisor','mission_metrics'):self.assertIn("executable='%s'"%executable,self.text)
 def test_simulation_spawns_rover_and_bridges_io(self):
  self.assertIn("executable='create'",self.text);self.assertIn("executable='parameter_bridge'",self.text);self.assertIn("'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'",self.text);self.assertIn("'/lunabot/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'",self.text)
 def test_rviz_presentation_layers(self):
  text=(R/'src/lunabot_bringup/rviz/goal_system.rviz').read_text()
  for topic in ('/lunabot/traversability/map','/lunabot/semantic_map/class','/lunabot/semantic_map/confidence','/lunabot/planning/path'):self.assertIn(topic,text)
 def test_assets_are_installed(self):
  setup=(R/'src/lunabot_bringup/setup.py').read_text();self.assertIn('lunabot_goal.launch.py',setup);self.assertIn('goal_system.yaml',setup);self.assertIn('goal_system.rviz',setup)
 def test_wrappers_are_fail_fast_and_execution_opt_in(self):
  demo=(R/'scripts/run_final_demo.sh').read_text();experiment=(R/'scripts/run_experiment.sh').read_text();self.assertIn('set -euo pipefail',demo);self.assertIn('set -euo pipefail',experiment);self.assertIn('== --execute',experiment)
 def test_navigation_graph_does_not_use_ground_truth(self):self.assertNotIn('ground_truth',self.text.lower())
if __name__=='__main__':unittest.main()
