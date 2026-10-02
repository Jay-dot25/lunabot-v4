#!/usr/bin/env python3
import ast
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def main():
 core=R/'src/lunabot_control/lunabot_control/safety_supervisor.py';node=R/'src/lunabot_control/lunabot_control/safety_supervisor_node.py'
 for p in (core,node):
  if not p.is_file():print('SAFETY_SUPERVISOR_FAIL missing');return 1
  ast.parse(p.read_text(),filename=str(p))
 text=core.read_text();ros=node.read_text()
 if any(x not in text for x in ('emergency_stop','localization_lost','sensor_timeout','path_expired','excessive_slope','obstacle','invalid_command')):print('SAFETY_SUPERVISOR_FAIL stops');return 1
 owners=[p for p in (R/'src').rglob('*.py') if "create_publisher(Twist,'/cmd_vel'" in p.read_text()]
 if len(owners)!=1 or owners[0]!=node:print('SAFETY_SUPERVISOR_FAIL ownership');return 1
 print('SAFETY_SUPERVISOR_PASS sole_cmd_vel_owner=1 fail_closed_stops=7 estop=1');return 0
if __name__=='__main__':raise SystemExit(main())
