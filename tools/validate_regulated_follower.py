#!/usr/bin/env python3
import ast
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def main():
 core=R/'src/lunabot_control/lunabot_control/regulated_pure_pursuit.py';node=R/'src/lunabot_control/lunabot_control/path_follower_node.py'
 for p in (core,node):
  if not p.is_file():print('REGULATED_FOLLOWER_FAIL missing');return 1
  ast.parse(p.read_text(),filename=str(p))
 text=core.read_text();ros=node.read_text()
 if any(x not in text for x in ('cross_track_error','heading_error','curvature_factor','clearance_factor','terrain_factor','max_accel','max_decel')):print('REGULATED_FOLLOWER_FAIL contract');return 1
 if "'/cmd_vel_nav'" not in ros or "create_publisher(Twist,'/cmd_vel'" in ros:print('REGULATED_FOLLOWER_FAIL ownership');return 1
 print('REGULATED_FOLLOWER_PASS regulation=3 acceleration_limits=1 tracking_metrics=1 nav_topic_only=1');return 0
if __name__=='__main__':raise SystemExit(main())
