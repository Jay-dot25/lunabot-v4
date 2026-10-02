#!/usr/bin/env python3
import ast
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def main():
 launch=R/'src/lunabot_bringup/launch/lunabot_goal.launch.py';config=R/'src/lunabot_bringup/config/goal_system.yaml';rviz=R/'src/lunabot_bringup/rviz/goal_system.rviz'
 if not all(p.is_file() for p in (launch,config,rviz)):print('FINAL_BRINGUP_FAIL missing');return 1
 text=launch.read_text();ast.parse(text,filename=str(launch))
 required=('simulation','bag_replay','evaluation','semantic_fusion_node','traversability_node','dstar_lite_planner','regulated_path_follower','safety_supervisor','mission_metrics')
 if any(x not in text for x in required) or 'ground_truth' in text.lower():print('FINAL_BRINGUP_FAIL graph');return 1
 print('FINAL_BRINGUP_PASS modes=3 production_nodes=8 rviz=1 ground_truth_navigation=0');return 0
if __name__=='__main__':raise SystemExit(main())
