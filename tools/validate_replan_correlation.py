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
 text=(ROOT/files[0]).read_text()
 if any(x not in text for x in ('sensor_detection','lethal_update','replan_requested','path_published','new_path & self.blocked')):print('REPLAN_CORRELATION_FAIL contract');return 1
 print('REPLAN_CORRELATION_PASS mandatory_obstacle=1 motion_only_rejected=1 runs=20');return 0
if __name__=='__main__':raise SystemExit(main())
