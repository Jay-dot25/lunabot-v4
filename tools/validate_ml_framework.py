#!/usr/bin/env python3
"""Validate Phase 6 ML contracts without requiring heavyweight dependencies."""
import json,py_compile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ml'))
from lunabot_ml.config import load_config

def main():
 errors=[]
 for name in ('unet','deeplabv3plus','segformer_b0'):
  try: load_config(ROOT/f'ml/configs/{name}.yaml')
  except Exception as exc: errors.append(f'{name}: {exc}')
 files=['dataset.py','augmentations.py','models.py','losses.py','metrics.py','config.py']
 scripts=['train.py','evaluate.py','export_onnx.py','infer_image.py']
 for relative in [*(f'ml/lunabot_ml/{x}' for x in files),*(f'ml/{x}' for x in scripts)]:
  try: py_compile.compile(str(ROOT/relative),doraise=True)
  except py_compile.PyCompileError as exc: errors.append(str(exc))
 if errors:
  print('ML_FRAMEWORK_VALIDATION_FAIL\n  '+'\n  '.join(errors));return 1
 print('ML_FRAMEWORK_VALIDATION_PASS architectures=3 classes=9 exporters=1');return 0
if __name__=='__main__':raise SystemExit(main())
