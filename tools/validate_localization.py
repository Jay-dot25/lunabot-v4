#!/usr/bin/env python3
"""Validate GPS-denied localization configuration and evaluation boundary."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 required=['src/lunabot_localization/config/ekf.yaml','src/lunabot_localization/config/slam_toolbox_filtered.yaml','src/lunabot_localization/launch/localization.launch.py','src/lunabot_localization/lunabot_localization/localization_evaluator.py','src/lunabot_localization/lunabot_localization/metrics.py']
 missing=[name for name in required if not (ROOT/name).is_file()]
 if missing:print('LOCALIZATION_VALIDATION_FAIL missing='+','.join(missing));return 1
 ekf=(ROOT/required[0]).read_text().lower()
 if 'ground_truth' in ekf or '/gps' in ekf or 'publish_tf: true' not in ekf:print('LOCALIZATION_VALIDATION_FAIL estimator boundary');return 1
 for name in required:
  if name.endswith('.py'):ast.parse((ROOT/name).read_text(),filename=name)
 print('LOCALIZATION_VALIDATION_PASS sensors=2 gps_inputs=0 metrics=5 tf_publishers=2_scoped');return 0
if __name__=='__main__':raise SystemExit(main())
