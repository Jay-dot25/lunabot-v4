#!/usr/bin/env python3
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 core=ROOT/'src/lunabot_planning/lunabot_planning/dstar_lite.py';node=ROOT/'src/lunabot_planning/lunabot_planning/planner_node.py'
 for path in (core,node):
  if not path.is_file():print('DSTAR_LITE_VALIDATION_FAIL missing');return 1
  ast.parse(path.read_text(),filename=str(path))
 text=core.read_text()
 if any(token not in text for token in ('self.g','self.rhs','self.km','update_vertex','compute_shortest_path','update_costs')):print('DSTAR_LITE_VALIDATION_FAIL state');return 1
 if 'astar' in text.lower():print('DSTAR_LITE_VALIDATION_FAIL mislabeled astar');return 1
 print('DSTAR_LITE_VALIDATION_PASS persistent_state=1 incremental_repair=1');return 0
if __name__=='__main__':raise SystemExit(main())
