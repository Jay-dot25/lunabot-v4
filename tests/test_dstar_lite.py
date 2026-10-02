#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_planning'))
from lunabot_planning.dstar_lite import DStarLite
import heapq,random

def optimal_cost(width,height,costs,start,goal):
 queue=[(0,start)];best={start:0}
 while queue:
  value,node=heapq.heappop(queue)
  if value!=best[node]:continue
  if node==goal:return value
  r,c=node
  for nxt in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
   rr,cc=nxt
   if not (0<=rr<height and 0<=cc<width) or costs[rr*width+cc]>=100:continue
   candidate=value+1+costs[rr*width+cc]
   if candidate<best.get(nxt,float('inf')):best[nxt]=candidate;heapq.heappush(queue,(candidate,nxt))
 return float('inf')
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
 def test_cost_decrease_repairs_to_shorter_path(self):
  costs=[0]*15;costs[7]=100;planner=DStarLite(5,3,costs,(1,0),(1,4));planner.compute_shortest_path();before=planner.path_cost(planner.path());planner.update_costs({(1,2):0});self.assertTrue(planner.compute_shortest_path());self.assertLess(planner.path_cost(planner.path()),before);self.assertIn((1,2),planner.path())
 def test_random_grids_match_fresh_optimal_cost(self):
  rng=random.Random(112)
  for _ in range(30):
   costs=[100 if rng.random()<.18 else rng.choice((0,0,1,3,8)) for _ in range(64)];start=(0,0);goal=(7,7);costs[0]=costs[-1]=0
   planner=DStarLite(8,8,costs,start,goal);found=planner.compute_shortest_path();expected=optimal_cost(8,8,costs,start,goal)
   self.assertEqual(found,expected<float('inf'))
   if found:self.assertEqual(planner.path_cost(planner.path()),expected)
 def test_local_repair_expands_less_than_fresh_search(self):
  costs=[0]*2500;planner=DStarLite(50,50,costs,(25,0),(25,49));planner.compute_shortest_path();planner.update_costs({(5,40):100});planner.compute_shortest_path();repair_expanded=planner.expanded
  fresh=DStarLite(50,50,planner.costs,(25,0),(25,49));fresh.compute_shortest_path();self.assertLess(repair_expanded,fresh.expanded)
if __name__=='__main__':unittest.main()
