#!/usr/bin/env python3
"""Phase 9 calibrated semantic fusion tests."""
import math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_mapping'))
from lunabot_mapping.semantic_fusion import SemanticFusionGrid,project_pixel
IDENTITY=(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)
class SemanticFusionTests(unittest.TestCase):
 def test_known_pinhole_geometry(self):
  point=project_pixel(420,240,2.,{'fx':500.,'fy':500.,'cx':320.,'cy':240.},IDENTITY);self.assertEqual(point,(.4,0.,2.))
 def test_transform_is_applied(self):
  transform=(1.,0.,0.,1.,0.,1.,0.,2.,0.,0.,1.,3.,0.,0.,0.,1.);self.assertEqual(project_pixel(0,0,1.,{'fx':1.,'fy':1.,'cx':0.,'cy':0.},transform),(1.,2.,4.))
 def test_repeated_observations_improve_confidence(self):
  grid=SemanticFusionGrid(10,10,1.,(0.,0.),stale_after_s=10);grid.observe(1.2,1.2,2.,2,.8,1);first=grid.value(1,1,1)['confidence'];grid.observe(1.2,1.2,2.,2,.8,2);second=grid.value(1,1,2)['confidence'];self.assertGreater(second,first)
 def test_hazard_evidence_dominates_traversable(self):
  grid=SemanticFusionGrid(10,10,1.,(0.,0.));grid.observe(1.2,1.2,0.,5,.5,1)
  for stamp in range(2,8):grid.observe(1.2,1.2,0.,1,1.,stamp)
  self.assertEqual(grid.value(1,1,8)['class_id'],5)
 def test_unknown_stale_and_bounds_are_explicit(self):
  grid=SemanticFusionGrid(2,2,1.,(0.,0.),stale_after_s=1);self.assertEqual(grid.value(0,0,0)['class_id'],0);self.assertFalse(grid.observe(-1,0,0,1,1.,0));grid.observe(.2,.2,1.,1,1.,0);self.assertTrue(grid.value(0,0,2_000_000_000)['stale']);self.assertEqual(grid.value(0,0,2_000_000_000)['class_id'],0)
 def test_ros_node_contract(self):
  source=(ROOT/'src/lunabot_mapping/lunabot_mapping/semantic_fusion_node.py').read_text();
  for token in ('ApproximateTimeSynchronizer','lookup_transform','depth_msg.header.stamp','project_pixel','/lunabot/semantic_map/class'):self.assertIn(token,source)
if __name__=='__main__':unittest.main()
