#!/usr/bin/env python3
import math,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'src/lunabot_control'))
from lunabot_control.safety_supervisor import SafetySupervisor
class Tests(unittest.TestCase):
 def healthy(self,c,**kw):return c.decide(10,nav=(9.9,.2,.3),localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9,**kw)
 def test_healthy_command_and_clamp(self):
  c=SafetySupervisor(max_linear=.2,max_angular=.4);d=c.decide(10,nav=(9.9,2.,2.),localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9);self.assertTrue(d.safe);self.assertEqual((d.linear,d.angular),(.2,.4))
 def test_teleop_priority(self):
  c=SafetySupervisor();d=c.decide(10,nav=(9.9,.2,0),teleop=(9.9,-.1,0),localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9);self.assertEqual(d.source,'teleop');self.assertLess(d.linear,0)
 def test_estop_latches(self):
  c=SafetySupervisor();c.set_estop(True);self.assertEqual(self.healthy(c).reason,'emergency_stop');c.set_estop(False);self.assertTrue(self.healthy(c).safe)
 def test_all_timeouts_fail_closed(self):
  c=SafetySupervisor();base=dict(now=10,nav=(9.9,.1,0),localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9)
  for field,reason in [('localization_stamp','localization_lost'),('sensor_stamp','sensor_timeout'),('path_stamp','path_expired')]:
   args=dict(base);args[field]=0;self.assertEqual(c.decide(**args).reason,reason)
  self.assertEqual(c.decide(10,localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9).reason,'command_timeout')
 def test_obstacle_slope_and_invalid_stop(self):
  c=SafetySupervisor();self.assertEqual(self.healthy(c,obstacle=True).reason,'obstacle');self.assertEqual(self.healthy(c,slope=30).reason,'excessive_slope');self.assertEqual(c.decide(10,nav=(9.9,math.nan,0),localization_stamp=9.9,sensor_stamp=9.9,path_stamp=9.9).state,'fault')
 def test_sole_final_publisher_contract(self):
  files=list((R/'src').rglob('*.py'));owners=[p for p in files if "create_publisher(Twist,'/cmd_vel'" in p.read_text()];self.assertEqual([p.name for p in owners],['safety_supervisor_node.py'])
if __name__=='__main__':unittest.main()
