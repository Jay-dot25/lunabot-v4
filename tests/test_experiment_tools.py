#!/usr/bin/env python3
import json,sys,tempfile,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'tools'))
from experiment_tools import load_manifest,mean_ci,summarize
class Tests(unittest.TestCase):
 def test_factorial_matrix_is_deterministic(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'m.json';p.write_text(json.dumps({'repetitions':2,'factors':{'seed':[2,1],'planner':['a','b']}}));_,a=load_manifest(p);_,b=load_manifest(p);self.assertEqual(a,b);self.assertEqual(len(a),8);self.assertEqual(len({x['trial_id'] for x in a}),8)
 def test_confidence_interval(self):
  r=mean_ci([1,2,3,4]);self.assertEqual(r['n'],4);self.assertEqual(r['mean'],2.5);self.assertLess(r['ci95_low'],r['mean']);self.assertGreater(r['ci95_high'],r['mean'])
 def test_single_and_empty_intervals_are_explicit(self):
  self.assertIsNone(mean_ci([])['mean']);self.assertEqual(mean_ci([3])['ci95_low'],3)
 def test_planner_comparison_summary(self):
  rows=[{'planner':'a','success':1,'mission_duration_s':2},{'planner':'a','success':0,'mission_duration_s':4},{'planner':'b','success':1,'mission_duration_s':1}];s=summarize(rows);self.assertEqual(s['a']['success']['mean'],.5);self.assertEqual(s['b']['mission_duration_s']['n'],1)
 def test_manifest_has_required_comparisons_and_repetitions(self):
  data,trials=load_manifest(R/'config/experiments.json');self.assertEqual(set(data['factors']['planner']),{'geometric_astar','semantic_weighted_astar','semantic_dstar_lite'});self.assertGreaterEqual(data['repetitions'],30);self.assertTrue(trials)
 def test_runner_defaults_to_dry_run_and_summary_writes_csv(self):
  runner=(R/'tools/run_experiments.py').read_text();summary=(R/'tools/summarize_experiments.py').read_text();self.assertIn("'--execute'",runner);self.assertIn("if args.execute",runner);self.assertIn("with_suffix('.csv')",summary)
if __name__=='__main__':unittest.main()
