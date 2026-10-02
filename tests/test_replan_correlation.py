#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_planning'))
from lunabot_planning.replan_correlation import ReplanCorrelation
class ReplanCorrelationTests(unittest.TestCase):
 def run_case(self,offset=0):
  c=ReplanCorrelation();c.set_initial_path([(0,x) for x in range(6)]);self.assertTrue(c.sensor_detection(100+offset,[(0,3)]));self.assertTrue(c.lethal_update(120+offset,[(0,3)]));self.assertTrue(c.replan_requested(125+offset,'obstacle'));self.assertTrue(c.path_published(140+offset,[(0,0),(1,0),(1,1),(1,2),(1,3),(1,4),(1,5),(0,5)]));c.goal_reached=True;return c.result()
 def test_twenty_deterministic_runs(self):
  for run in range(20):
   result=self.run_case(run*1000);self.assertTrue(result['mission_success']);self.assertEqual(result['total_latency_ms'],40/1e6)
 def test_motion_only_cannot_pass(self):
  c=ReplanCorrelation();c.set_initial_path([(0,0),(0,1)]);self.assertFalse(c.replan_requested(10,'start_moved'));c.path_published(20,[(0,1)]);self.assertFalse(c.result()['valid_replan'])
 def test_detection_must_intersect_active_path(self):
  c=ReplanCorrelation();c.set_initial_path([(0,0)]);self.assertFalse(c.sensor_detection(1,[(4,4)]));self.assertFalse(c.lethal_update(2,[(4,4)]))
 def test_new_path_must_exclude_blocked_footprint(self):
  c=ReplanCorrelation();c.set_initial_path([(0,0),(0,1)]);c.sensor_detection(1,[(0,1)]);c.lethal_update(2,[(0,1)]);c.replan_requested(3,'obstacle');c.path_published(4,[(0,0),(0,1)]);self.assertFalse(c.result()['valid_replan'])
 def test_collision_prevents_mission_success(self):
  c=ReplanCorrelation();c.set_initial_path([(0,0)]);c.sensor_detection(1,[(0,0)]);c.lethal_update(2,[(0,0)]);c.replan_requested(3,'obstacle');c.path_published(4,[(1,0)]);c.goal_reached=True;c.collisions=1;self.assertTrue(c.result()['valid_replan']);self.assertFalse(c.result()['mission_success'])
if __name__=='__main__':unittest.main()
