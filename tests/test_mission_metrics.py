#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'src/lunabot_evaluation'))
from lunabot_evaluation.mission_metrics import MissionMetricsCollector
class Tests(unittest.TestCase):
 def test_lengths_efficiency_cost_and_terrain(self):
  c=MissionMetricsCollector(3);c.start(0);c.set_plan([(0,0),(3,0)]);c.pose(0,0,0);c.pose(1,1,0,2,5);c.pose(2,1,1,1,2);c.finish(2,True);r=c.result();self.assertEqual(r['planned_path_length_m'],3);self.assertEqual(r['executed_path_length_m'],2);self.assertEqual(r['path_efficiency'],1.5);self.assertEqual(r['accumulated_traversability_cost'],7);self.assertEqual(r['terrain_distance_m'],[0,1,1])
 def test_collision_count_duration_and_details(self):
  c=MissionMetricsCollector();c.start(0);c.contact(1,'rock',True,2.5,4);c.contact(1.5,'rock',True);c.contact(3,'rock',False);c.finish(4,False,'collision');r=c.result();self.assertEqual(r['collision_count'],1);self.assertEqual(r['collision_duration_s'],2);self.assertEqual(r['collisions'][0]['relative_speed'],2.5);self.assertFalse(r['success'])
 def test_active_contact_closes_at_finish(self):
  c=MissionMetricsCollector();c.start(1);c.contact(2,'wall',True);c.finish(5,False);self.assertEqual(c.result()['collision_duration_s'],3)
 def test_replans_clearance_and_tracking(self):
  c=MissionMetricsCollector();c.start(0);c.replan(10);c.replan(30);c.clearance(.4);c.clearance(.2);c.tracking_error(.3);c.tracking_error(.4);c.finish(2,True);r=c.result();self.assertEqual(r['mean_replan_time_ms'],20);self.assertEqual(r['max_replan_time_ms'],30);self.assertEqual(r['minimum_clearance_m'],.2);self.assertAlmostEqual(r['controller_cross_track_rmse_m'],.35355339)
 def test_node_requires_physics_contact_topic_and_atomic_json(self):
  text=(R/'src/lunabot_evaluation/lunabot_evaluation/mission_metrics_node.py').read_text();self.assertIn("'/lunabot/simulation/contact'",text);self.assertIn('os.replace',text);self.assertIn('MissionMetrics',text)
if __name__=='__main__':unittest.main()
