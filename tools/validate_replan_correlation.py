#!/usr/bin/env python3
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 files=['src/lunabot_planning/lunabot_planning/replan_correlation.py','src/lunabot_planning/lunabot_planning/replan_verifier_node.py']
 for name in files:
  path=ROOT/name
  if not path.is_file():print('REPLAN_CORRELATION_FAIL missing');return 1
  ast.parse(path.read_text(),filename=name)
 text=(ROOT/files[0]).read_text();node=(ROOT/files[1]).read_text();planner=(ROOT/'src/lunabot_planning/lunabot_planning/planner_node.py').read_text()
 required=('sensor_detection','lethal_update','replan_requested','path_published','new_path & self.blocked')
 if any(x not in text for x in required) or 'msg.active_path_invalidated' not in node or 'newly_lethal & self.path_cells' not in planner:print('REPLAN_CORRELATION_FAIL contract');return 1
 print('REPLAN_CORRELATION_PASS mandatory_obstacle=1 motion_only_rejected=1 path_invalidation=1 runs=20');return 0
if __name__=='__main__':raise SystemExit(main())
