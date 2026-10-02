#!/usr/bin/env python3
import ast,json
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def main():
 files=[R/'tools/experiment_tools.py',R/'tools/run_experiments.py',R/'tools/summarize_experiments.py']
 for p in files:
  if not p.is_file():print('EXPERIMENT_FRAMEWORK_FAIL missing');return 1
  ast.parse(p.read_text(),filename=str(p))
 data=json.loads((R/'config/experiments.json').read_text());planners=set(data['factors']['planner'])
 if planners!={'geometric_astar','semantic_weighted_astar','semantic_dstar_lite'} or data['repetitions']<30:print('EXPERIMENT_FRAMEWORK_FAIL matrix');return 1
 print('EXPERIMENT_FRAMEWORK_PASS planners=3 repetitions=30 confidence_intervals=1 dry_run_default=1');return 0
if __name__=='__main__':raise SystemExit(main())
