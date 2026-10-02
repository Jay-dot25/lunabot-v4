#!/usr/bin/env python3
"""Phase 10 layered traversability tests."""
import math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_mapping'))
from lunabot_mapping.traversability import REASON,build_traversability
class TraversabilityTests(unittest.TestCase):
 def build(self,classes=None,confidence=None,elevation=None,stale=None,**cfg):
  return build_traversability(classes or [1]*9,confidence or [1.]*9,elevation or [0.]*9,stale or [False]*9,3,3,1.,{'footprint_radius_m':0.,'inflation_radius_m':0.,**cfg})
 def test_semantic_uncertainty_and_clamp(self):
  high=self.build(confidence=[0.]*9,unknown_is_lethal=False);low=self.build(confidence=[1.]*9,unknown_is_lethal=False);self.assertGreater(high['cost'][4],low['cost'][4]);self.assertLessEqual(high['cost'][4],99)
 def test_unknown_stale_and_invalid_policies(self):
  unknown=self.build(classes=[0]*9,unknown_is_lethal=True);self.assertEqual(unknown['reason'][4],REASON['UNKNOWN']);self.assertEqual(unknown['cost'][4],100)
  stale=self.build(stale=[False]*4+[True]+[False]*4);self.assertEqual(stale['reason'][4],REASON['STALE'])
  elevation=[0.]*9;elevation[4]=math.nan;invalid=self.build(elevation=elevation);self.assertEqual(invalid['reason'][4],REASON['INVALID_ELEVATION'])
 def test_slope_cross_slope_and_step_limits(self):
  elevation=[0.]*9;elevation[4]=.2;slope=self.build(elevation=elevation,max_step_m=1.,max_slope_deg=5.,max_cross_slope_deg=5.);self.assertEqual(slope['reason'][4],REASON['SLOPE'])
  step=self.build(elevation=elevation,max_step_m=.1,max_slope_deg=89.,max_cross_slope_deg=89.);self.assertEqual(step['reason'][4],REASON['STEP'])
 def test_footprint_inflation_and_debug_reason(self):
  classes=[1]*9;classes[4]=6;result=build_traversability(classes,[1.]*9,[0.]*9,[False]*9,3,3,1.,{'footprint_radius_m':1.,'inflation_radius_m':0.});self.assertEqual(result['reason'][4],REASON['SEMANTIC_LETHAL']);self.assertEqual(result['reason'][1],REASON['INFLATED']);self.assertEqual(result['cost'][1],100)
 def test_longer_safe_route_costs_less_than_short_crater_route(self):
  classes=[1]*25;classes[2*5+2]=6;result=self.build_route(classes)
  short=[(2,0),(2,1),(2,2),(2,3),(2,4)];safe=[(2,0),(1,0),(1,1),(1,2),(1,3),(1,4),(2,4)]
  total=lambda path:sum(result['cost'][r*5+c] for r,c in path)
  self.assertGreater(len(safe),len(short));self.assertLess(total(safe),total(short))
 def build_route(self,classes):return build_traversability(classes,[1.]*25,[0.]*25,[False]*25,5,5,1.,{'footprint_radius_m':0.,'inflation_radius_m':0.})
 def test_node_has_single_production_output(self):
  source=(ROOT/'src/lunabot_mapping/lunabot_mapping/traversability_node.py').read_text();self.assertIn('/lunabot/traversability/map',source);self.assertIn('/lunabot/traversability/reason',source);self.assertIn('ApproximateTimeSynchronizer',source)
if __name__=='__main__':unittest.main()
