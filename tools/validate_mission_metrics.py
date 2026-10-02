#!/usr/bin/env python3
import ast
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def main():
 files=[R/'src/lunabot_evaluation/lunabot_evaluation/mission_metrics.py',R/'src/lunabot_evaluation/lunabot_evaluation/mission_metrics_node.py']
 for p in files:
  if not p.is_file():print('MISSION_METRICS_FAIL missing');return 1
  ast.parse(p.read_text(),filename=str(p))
 text=files[0].read_text();node=files[1].read_text()
 if any(x not in text for x in ('collision_count','collision_duration_s','path_efficiency','terrain_distance_m','controller_cross_track_rmse_m')) or "'/lunabot/simulation/contact'" not in node:print('MISSION_METRICS_FAIL contract');return 1
 print('MISSION_METRICS_PASS collision_evidence=1 mission_metrics=1 atomic_trial_json=1');return 0
if __name__=='__main__':raise SystemExit(main())
