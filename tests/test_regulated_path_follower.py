#!/usr/bin/env python3
import math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_control'))
from lunabot_control.regulated_pure_pursuit import RegulatedPurePursuit
class Tests(unittest.TestCase):
 def test_straight_path_acceleration_limit(self):
  c=RegulatedPurePursuit(max_linear=1,max_accel=.2);r=c.command([(0,0),(2,0)],(0,0,0),.5);self.assertAlmostEqual(r.linear,.1);self.assertAlmostEqual(r.angular,0)
 def test_curvature_reduces_speed(self):
  straight=RegulatedPurePursuit(max_accel=100).command([(0,0),(1,0)],(0,0,0),1);turn=RegulatedPurePursuit(max_accel=100).command([(0,0),(0,1)],(0,0,0),1);self.assertLess(turn.linear,straight.linear);self.assertGreater(turn.angular,0)
 def test_terrain_and_clearance_regulation(self):
  path=[(0,0),(2,0)];a=RegulatedPurePursuit(max_accel=100).command(path,(0,0,0),1);b=RegulatedPurePursuit(max_accel=100).command(path,(0,0,0),1,clearance=.1,roughness=.8,uncertainty=.8);self.assertLess(b.linear,a.linear)
 def test_deceleration_limit(self):
  c=RegulatedPurePursuit(max_linear=1,max_accel=10,max_decel=.2);c.command([(0,0),(2,0)],(0,0,0),1);r=c.command([(0,0),(0,2)],(0,0,0),.5);self.assertGreaterEqual(r.linear,.9)
 def test_goal_and_no_path_stop(self):
  c=RegulatedPurePursuit();self.assertEqual(c.command([], (0,0,0),.1).linear,0);self.assertEqual(c.command([(0,0)],(0,0,0),.1).linear,0)
 def test_tracking_errors_reported(self):
  r=RegulatedPurePursuit(max_accel=100).command([(0,1),(2,1)],(0,0,0),1);self.assertAlmostEqual(r.cross_track_error,1);self.assertNotEqual(r.heading_error,0)
 def test_production_topic_ownership(self):
  text=(ROOT/'src/lunabot_control/lunabot_control/path_follower_node.py').read_text();self.assertIn("'/cmd_vel_nav'",text);self.assertNotIn("create_publisher(Twist,'/cmd_vel'",text);self.assertIn('localization_timeout',text);self.assertIn('path_timeout',text)
if __name__=='__main__':unittest.main()
