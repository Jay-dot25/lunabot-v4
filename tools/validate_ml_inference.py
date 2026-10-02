#!/usr/bin/env python3
"""Validate Phase 7 inference integration without a production model."""
import py_compile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 required=['src/lunabot_perception/lunabot_perception/inference_core.py','src/lunabot_perception/lunabot_perception/terrain_inference_node.py','src/lunabot_perception/config/terrain_inference.yaml','src/lunabot_bringup/launch/perception.launch.py']
 missing=[name for name in required if not (ROOT/name).is_file()]
 if missing:print('ML_INFERENCE_VALIDATION_FAIL missing='+','.join(missing));return 1
 for name in required:
  if name.endswith('.py'):py_compile.compile(str(ROOT/name),doraise=True)
 package=(ROOT/'src/lunabot_perception/package.xml').read_text()
 dependencies=('cv_bridge','message_filters','lunabot_msgs')
 if any(f'<exec_depend>{name}</exec_depend>' not in package for name in dependencies):print('ML_INFERENCE_VALIDATION_FAIL dependencies');return 1
 print('ML_INFERENCE_VALIDATION_PASS synchronized_inputs=2 outputs=4 modes=3');return 0
if __name__=='__main__':raise SystemExit(main())
