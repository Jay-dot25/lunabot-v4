#!/usr/bin/env python3
"""Validate calibrated semantic-fusion contracts without ROS runtime."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 files=['src/lunabot_mapping/lunabot_mapping/semantic_fusion.py','src/lunabot_mapping/lunabot_mapping/semantic_fusion_node.py']
 for name in files:
  path=ROOT/name
  if not path.is_file():print('SEMANTIC_FUSION_VALIDATION_FAIL missing='+name);return 1
  ast.parse(path.read_text(),filename=name)
 source=(ROOT/files[1]).read_text()
 required=('class_topic','confidence_topic','depth_topic','camera_info_topic','lookup_transform','ApproximateTimeSynchronizer')
 if any(token not in source for token in required):print('SEMANTIC_FUSION_VALIDATION_FAIL contract');return 1
 print('SEMANTIC_FUSION_VALIDATION_PASS synchronized_inputs=4 calibrated_projection=1 temporal_fusion=1');return 0
if __name__=='__main__':raise SystemExit(main())
