#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_planning'))
from lunabot_planning.dstar_lite import DStarLite
class DStarLiteTests(unittest.TestCase):
 def test_initial_path_and_state(self):
  planner=DStarLite(5,5,[0]*25,(2,0),(2,4));self.assertTrue(planner.compute_shortest_path());self.assertEqual(planner.path()[0],(2,0));self.assertEqual(planner.path()[-1],(2,4));self.assertTrue(planner.g);self.assertTrue(planner.rhs);self.assertEqual(planner.km,0)
 def test_changed_cell_incremental_repair(self):
  planner=DStarLite(7,5,[0]*35,(2,0),(2,6));planner.compute_shortest_path();initial=planner.path();original_g=id(planner.g);affected=planner.update_costs({(2,3):100});self.assertLess(len(affected),35);self.assertTrue(planner.compute_shortest_path());repaired=planner.path();self.assertEqual(id(planner.g),original_g);self.assertIn((2,3),initial);self.assertNotIn((2,3),repaired);self.assertGreater(len(repaired),len(initial))
 def test_start_motion_updates_km(self):
  planner=DStarLite(5,5,[0]*25,(0,0),(4,4));planner.compute_shortest_path();planner.move_start((0,1));self.assertEqual(planner.km,1);self.assertTrue(planner.compute_shortest_path())
 def test_no_path(self):
  costs=[0]*9
  for c in range(3):costs[3+c]=100
  planner=DStarLite(3,3,costs,(0,0),(2,2));self.assertFalse(planner.compute_shortest_path());self.assertEqual(planner.path(),[])
 def test_weighted_cost_avoids_expensive_shortcut(self):
  costs=[0]*15;costs[1*5+2]=99;planner=DStarLite(5,3,costs,(1,0),(1,4));planner.compute_shortest_path();self.assertNotIn((1,2),planner.path())
if __name__=='__main__':unittest.main()
